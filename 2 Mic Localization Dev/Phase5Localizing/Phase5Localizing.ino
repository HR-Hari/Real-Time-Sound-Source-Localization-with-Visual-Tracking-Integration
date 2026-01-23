#include <Arduino.h>
#include "driver/i2s.h"
#include "esp_dsp.h"
#include <math.h>

// ===================== PARAMETERS =====================
#define SAMPLE_RATE     48000
// Maximized Frame Size (Must be power of 2 for FFT)
#define FRAME_SAMPLES   2048  
#define MIC_DISTANCE    0.14f 
#define SPEED_OF_SOUND  343.0f

#define I2S_PORT        I2S_NUM_0
#define BCLK_PIN        5
#define WS_PIN          6
#define DATA_PIN        7

#define FIR_TAPS        16    
#define ENERGY_THRESH   4000.0f
#define EMA_ALPHA       0.9f 

// ===================== ALIGNED GLOBALS =====================
// Note: Larger buffers = More RAM usage. 
// 4096 * 4 bytes * ~10 buffers = ~160KB RAM (Safe for ESP32-S3)

static int32_t i2s_rx[FRAME_SAMPLES * 2];

// Raw & Filtered Buffers
static float xL_raw[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xR_raw[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xL_filt[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xR_filt[FRAME_SAMPLES] __attribute__((aligned(16)));

// FFT Processing Buffers
static float window[FRAME_SAMPLES] __attribute__((aligned(16)));
static float X1[2 * FRAME_SAMPLES] __attribute__((aligned(16)));
static float X2[2 * FRAME_SAMPLES] __attribute__((aligned(16)));
static float Rxy[2 * FRAME_SAMPLES] __attribute__((aligned(16)));

// FIR State & Coeffs
static float fir_state_L[FRAME_SAMPLES + FIR_TAPS] __attribute__((aligned(16)));
static float fir_state_R[FRAME_SAMPLES + FIR_TAPS] __attribute__((aligned(16)));
fir_f32_t fir_inst_L;
fir_f32_t fir_inst_R;

// 16-Tap Averager Coefficients (Kill 21kHz)
static float fir_coeffs[FIR_TAPS] __attribute__((aligned(16))) = {
    0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 
    0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625, 0.0625
};

// ===================== STRUCT =====================
typedef struct {
    int32_t left[FRAME_SAMPLES];
    int32_t right[FRAME_SAMPLES];
} audio_frame_t;

QueueHandle_t audio_queue;

// ===================== I2S SETUP =====================
void setup_i2s() {
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 4,
        .dma_buf_len = 1024, // Keep DMA chunks manageable
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

void i2s_task(void *p) {
    size_t bytes;
    static audio_frame_t frame; // Static to save stack
    
    while (1) {
        // We read in chunks to fill our massive 4096 buffer
        // Note: i2s_read handles the looping internally if we ask for huge bytes,
        // but it's safer to read exactly what we need.
        i2s_read(I2S_PORT, i2s_rx, sizeof(i2s_rx), &bytes, portMAX_DELAY);
        
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            frame.left[i]  = i2s_rx[2*i + 1] >> 8;
            frame.right[i] = i2s_rx[2*i]     >> 8;
        }
        
        // Wait 0: If queue is full (because main task is sleeping), 
        // drop this frame and keep reading to clear hardware buffer.
        xQueueSend(audio_queue, &frame, 0); 
    }
}

// ===================== GCC-PHAT TASK =====================
void gccphat_task(void *p) {
    audio_frame_t f;
    static float lag_smoothed = 0;

    while (1) {
        // 0. FLUSH OLD DATA
        // Since we sleep for 1000ms, the queue fills up with old audio.
        // We read everything available until the queue is empty, keeping only the last one.
        while(uxQueueMessagesWaiting(audio_queue) > 0) {
            xQueueReceive(audio_queue, &f, 0);
        }
        
        // Now wait for the very next FRESH frame
        if(xQueueReceive(audio_queue, &f, portMAX_DELAY) != pdTRUE) continue;

        // 1. Convert to Float
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            xL_raw[i] = (float)f.left[i];
            xR_raw[i] = (float)f.right[i];
        }

        // 2. APPLY FILTER (The Noise Killer)
        dsps_fir_f32_aes3(&fir_inst_L, xL_raw, xL_filt, FRAME_SAMPLES);
        dsps_fir_f32_aes3(&fir_inst_R, xR_raw, xR_filt, FRAME_SAMPLES);

        // 3. RMS GATE
        float energy = 0;
        dsps_dotprod_f32(xL_filt, xL_filt, &energy, FRAME_SAMPLES);
        float rms = sqrtf(energy / FRAME_SAMPLES);

        if (rms < ENERGY_THRESH) {
            Serial.println("... Waiting for sound ...");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue; 
        }

        // 4. PREPARE FFT (Windowing)
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            X1[2*i]   = xL_filt[i] * window[i];
            X1[2*i+1] = 0;
            X2[2*i]   = xR_filt[i] * window[i];
            X2[2*i+1] = 0;
        }

        // 5. FFT FORWARD
        dsps_fft2r_fc32(X1, FRAME_SAMPLES);
        dsps_bit_rev_fc32(X1, FRAME_SAMPLES);
        dsps_fft2r_fc32(X2, FRAME_SAMPLES);
        dsps_bit_rev_fc32(X2, FRAME_SAMPLES);

        // 6. PHAT TRANSFORM
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            float re = X1[2*i] * X2[2*i] + X1[2*i+1] * X2[2*i+1];
            float im = X1[2*i+1] * X2[2*i] - X1[2*i] * X2[2*i+1];
            float mag = sqrtf(re*re + im*im) + 1e-9f; 
            Rxy[2*i]   = re / mag;
            Rxy[2*i+1] = im / mag;
        }

        // 7. INVERSE FFT
        dsps_fft2r_fc32(Rxy, FRAME_SAMPLES);
        dsps_bit_rev_fc32(Rxy, FRAME_SAMPLES);

        // 8. FIND PEAK LAG
        int bestLag = 0;
        float bestVal = 0;
        int search_radius = 40; // Increased radius because frame is larger? 
                                // Actually, physical delay samples is constant (SampleRate * Dist / Speed).
                                // 0.1m / 343 * 48000 = ~14 samples.
                                // We keep radius tight to avoid noise peaks.

        for (int lag = -search_radius; lag <= search_radius; lag++) {
            int idx = (lag + FRAME_SAMPLES) % FRAME_SAMPLES;
            float v = Rxy[2*idx]; 
            if (v > bestVal) {
                bestVal = v;
                bestLag = lag;
            }
        }

        // 9. QUADRATIC INTERPOLATION
        float fine_lag = (float)bestLag;
        if (bestLag > -search_radius && bestLag < search_radius) {
            int idx = (bestLag + FRAME_SAMPLES) % FRAME_SAMPLES;
            int idx_prev = (bestLag - 1 + FRAME_SAMPLES) % FRAME_SAMPLES;
            int idx_next = (bestLag + 1 + FRAME_SAMPLES) % FRAME_SAMPLES;
            
            float y1 = Rxy[2*idx_prev];
            float y2 = Rxy[2*idx];
            float y3 = Rxy[2*idx_next];
            
            float denom = 2 * (2 * y2 - y1 - y3);
            if (denom != 0) {
                float delta = (y3 - y1) / denom;
                if (delta > -1.0f && delta < 1.0f) fine_lag += delta;
            }
        }

        // 10. SMOOTHING
        lag_smoothed = (EMA_ALPHA * fine_lag) + ((1.0f - EMA_ALPHA) * lag_smoothed);

        float tdoa = lag_smoothed / SAMPLE_RATE;
        float ratio = (tdoa * SPEED_OF_SOUND) / MIC_DISTANCE;
        
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < -1.0f) ratio = -1.0f;

        float angle = asinf(ratio) * (180.0f / PI);

        // 11. PRINT AND SLEEP
        Serial.println("========================================");
        Serial.printf("Angle:  %5.1f deg\n", angle);
        Serial.printf("Lag:    %5.2f samples\n", lag_smoothed);
        Serial.printf("Energy: %.0f\n", rms);
        Serial.println("========================================");
        
        // THIS IS THE DELAY YOU WANTED
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    dsps_fft2r_init_fc32(NULL, FRAME_SAMPLES);
    dsps_wind_hann_f32(window, FRAME_SAMPLES);

    memset(fir_state_L, 0, sizeof(fir_state_L));
    memset(fir_state_R, 0, sizeof(fir_state_R));
    dsps_fir_init_f32(&fir_inst_L, fir_coeffs, fir_state_L, FIR_TAPS);
    dsps_fir_init_f32(&fir_inst_R, fir_coeffs, fir_state_R, FIR_TAPS);

    setup_i2s();
    
    // Increased Queue size slightly
    audio_queue = xQueueCreate(4, sizeof(audio_frame_t));

    // Increased Stack sizes for the heavier processing
    xTaskCreatePinnedToCore(i2s_task, "I2S", 8192, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(gccphat_task, "GCC", 32768, NULL, 8, NULL, 1);
    
    Serial.println("HIGH RES MODE (4096 Samples) STARTED");
}

void loop() {}