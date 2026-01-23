#include <Arduino.h>
#include "driver/i2s.h"
#include "esp_dsp.h"
#include <math.h>

// ===================== PARAMETERS =====================
#define SAMPLE_RATE     48000
#define FRAME_SAMPLES   2048

#define MIC_DISTANCE    0.15f
#define SPEED_OF_SOUND  343.0f

#define I2S_PORT        I2S_NUM_0
#define BCLK_PIN        5
#define WS_PIN          6
#define DATA_PIN        7

#define FIR_TAPS        63
#define ENERGY_THRESHOLD 3000.0f
#define LAG_EMA_ALPHA   0.2f

// ===================== ALIGNED BUFFERS =====================
// IMPORTANT: ESP32-S3 DSP instructions require 16-byte memory alignment.
// If you remove __attribute__((aligned(16))), it WILL crash.

// FIR Coefficients (Must be non-const and aligned)
static float fir_bp[FIR_TAPS] __attribute__((aligned(16))) = {
 -0.001274,-0.001581,-0.001921,-0.002292,-0.002690,
 -0.003111,-0.003549,-0.003997,-0.004447,-0.004890,
 -0.005316,-0.005713,-0.006069,-0.006370,-0.006603,
 -0.006753,-0.006807,-0.006753,-0.006581,-0.006280,
 -0.005842,-0.005259,-0.004528,-0.003646,-0.002616,
 -0.001442,-0.000135, 0.001289, 0.002812, 0.004411,
  0.006062,
  0.007736,
  0.006062, 0.004411, 0.002812, 0.001289,-0.000135,
 -0.001442,-0.002616,-0.003646,-0.004528,-0.005259,
 -0.005842,-0.006280,-0.006581,-0.006753,-0.006807,
 -0.006753,-0.006603,-0.006370,-0.006069,-0.005713,
 -0.005316,-0.004890,-0.004447,-0.003997,-0.003549,
 -0.003111,-0.002690,-0.002292,-0.001921,-0.001581,
 -0.001274
};

// Filter State Memory
static float fir_state_L[FRAME_SAMPLES + FIR_TAPS] __attribute__((aligned(16)));
static float fir_state_R[FRAME_SAMPLES + FIR_TAPS] __attribute__((aligned(16)));

// Processing Buffers
static float xL[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xR[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xLf[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xRf[FRAME_SAMPLES] __attribute__((aligned(16)));
static float window[FRAME_SAMPLES] __attribute__((aligned(16)));

// FFT Buffers
static float X1[2*FRAME_SAMPLES] __attribute__((aligned(16)));
static float X2[2*FRAME_SAMPLES] __attribute__((aligned(16)));
static float Rxy[2*FRAME_SAMPLES] __attribute__((aligned(16)));

// FIR Instance Structures
fir_f32_t fir_inst_L;
fir_f32_t fir_inst_R;

// ===================== STRUCT =====================
typedef struct {
    int32_t left[FRAME_SAMPLES];
    int32_t right[FRAME_SAMPLES];
} audio_frame_t;

QueueHandle_t audio_queue;
static int32_t i2s_rx[FRAME_SAMPLES * 2]; // I2S raw buffer

// ===================== I2S SETUP =====================
void setup_i2s()
{
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 6,
        .dma_buf_len = 1024, // Optimized DMA length
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pins = {
        .bck_io_num = BCLK_PIN,
        .ws_io_num = WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = DATA_PIN
    };

    i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
    i2s_set_pin(I2S_PORT, &pins);
}

// ===================== TASK: I2S =====================
void i2s_task(void *p)
{
    // Fix: Static to prevent stack overflow
    static audio_frame_t frame;
    size_t bytes;

    while (1) {
        i2s_read(I2S_PORT, i2s_rx, sizeof(i2s_rx), &bytes, portMAX_DELAY);
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            frame.left[i]  = i2s_rx[2*i+1] >> 8;
            frame.right[i] = i2s_rx[2*i]   >> 8;
        }
        xQueueSend(audio_queue, &frame, portMAX_DELAY);
    }
}

// ===================== TASK: GCC-PHAT =====================
void gccphat_task(void *p)
{
    // Fix: Static to prevent stack overflow
    static audio_frame_t f;
    
    static float lag_smoothed = 0;
    static bool lag_valid = false;

    while (1) {
        if(xQueueReceive(audio_queue, &f, portMAX_DELAY) != pdTRUE) continue;

        for (int i = 0; i < FRAME_SAMPLES; i++) {
            xL[i] = (float)f.left[i];
            xR[i] = (float)f.right[i];
        }

        // Fix: Use S3 Optimized Filter (aes3)
        dsps_fir_f32_aes3(&fir_inst_L, xL, xLf, FRAME_SAMPLES);
        dsps_fir_f32_aes3(&fir_inst_R, xR, xRf, FRAME_SAMPLES);

        // Fix: Optimized Dot Product for Energy
        float rmsL = 0, rmsR = 0;
        dsps_dotprod_f32(xLf, xLf, &rmsL, FRAME_SAMPLES);
        dsps_dotprod_f32(xRf, xRf, &rmsR, FRAME_SAMPLES);
        
        rmsL = sqrtf(rmsL/FRAME_SAMPLES);
        rmsR = sqrtf(rmsR/FRAME_SAMPLES);

        if ((rmsL + rmsR) < ENERGY_THRESHOLD) {
            continue; // noise-only frame
        }

        for (int i = 0; i < FRAME_SAMPLES; i++) {
            X1[2*i]   = (xLf[i]/rmsL) * window[i];
            X1[2*i+1] = 0;
            X2[2*i]   = (xRf[i]/rmsR) * window[i];
            X2[2*i+1] = 0;
        }

        dsps_fft2r_fc32(X1, FRAME_SAMPLES);
        dsps_fft2r_fc32(X2, FRAME_SAMPLES);
        dsps_bit_rev_fc32(X1, FRAME_SAMPLES);
        dsps_bit_rev_fc32(X2, FRAME_SAMPLES);

        for (int i = 0; i < FRAME_SAMPLES; i++) {
            float re = X1[2*i]*X2[2*i] + X1[2*i+1]*X2[2*i+1];
            float im = X1[2*i+1]*X2[2*i] - X1[2*i]*X2[2*i+1];
            float mag = sqrtf(re*re + im*im) + 1e-9f;
            Rxy[2*i]   = re / mag;
            Rxy[2*i+1] = im / mag;
        }

        dsps_fft2r_fc32(Rxy, FRAME_SAMPLES);
        dsps_bit_rev_fc32(Rxy, FRAME_SAMPLES);

        int bestLag = 0;
        float bestVal = 0;
        
        // Fix: Limit search range to reasonable physics (prevent false peaks)
        // 20 samples is enough for 10cm @ 48kHz
        for (int lag = -20; lag <= 20; lag++) {
            int idx = (lag + FRAME_SAMPLES) % FRAME_SAMPLES;
            float v = fabsf(Rxy[2*idx]); // Use Real part magnitude
            if (v > bestVal) {
                bestVal = v;
                bestLag = lag;
            }
        }

        if (!lag_valid) {
            lag_smoothed = bestLag;
            lag_valid = true;
        } else {
            lag_smoothed = LAG_EMA_ALPHA*bestLag + (1.0f-LAG_EMA_ALPHA)*lag_smoothed;
        }

        float tdoa = lag_smoothed / SAMPLE_RATE;
        float ratio = (tdoa * SPEED_OF_SOUND) / MIC_DISTANCE;
        
        // Fix: NaN protection
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;
        
        float angle = asinf(ratio) * 180.0f / PI;

        Serial.print("Lag: ");
        Serial.print(lag_smoothed, 2);
        Serial.print(" | Angle (deg): ");
        Serial.println(angle, 2);
    }
}

// ===================== SETUP =====================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Initialize FFT
    dsps_fft2r_init_fc32(NULL, FRAME_SAMPLES);

    // Initialize Window
    for (int i = 0; i < FRAME_SAMPLES; i++)
        window[i] = 0.5f * (1 - cosf(2*PI*i/(FRAME_SAMPLES-1)));

    // Initialize FIR Filter Instances
    memset(fir_state_L, 0, sizeof(fir_state_L));
    memset(fir_state_R, 0, sizeof(fir_state_R));
    dsps_fir_init_f32(&fir_inst_L, fir_bp, fir_state_L, FIR_TAPS);
    dsps_fir_init_f32(&fir_inst_R, fir_bp, fir_state_R, FIR_TAPS);

    setup_i2s();
    
    audio_queue = xQueueCreate(2, sizeof(audio_frame_t));

    // Increase stack size slightly for safety
    xTaskCreatePinnedToCore(i2s_task, "I2S", 8192, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(gccphat_task, "GCC", 16384, NULL, 8, NULL, 1);
}

void loop() {}