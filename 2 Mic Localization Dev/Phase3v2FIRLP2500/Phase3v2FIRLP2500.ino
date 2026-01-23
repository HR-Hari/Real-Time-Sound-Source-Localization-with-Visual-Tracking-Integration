#include <Arduino.h>
#include "driver/i2s.h"
#include "esp_dsp.h"
#include <math.h>

// ===================== PARAMETERS =====================
#define SAMPLE_RATE     48000
#define FRAME_SAMPLES   512
#define MIC_DISTANCE    0.10f
#define SPEED_OF_SOUND  343.0f

#define I2S_PORT        I2S_NUM_0
#define BCLK_PIN        5
#define WS_PIN          6
#define DATA_PIN        7

// === NEW FILTER SETTINGS ===
#define FIR_TAPS        16   // 16-tap Moving Average

// ===================== ALIGNED GLOBALS =====================
static int32_t i2s_rx[FRAME_SAMPLES * 2];

// Buffers
static float xL_raw[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xL_filt[FRAME_SAMPLES] __attribute__((aligned(16)));

// FFT Buffers
static float window[FRAME_SAMPLES] __attribute__((aligned(16)));
static float X1[2 * FRAME_SAMPLES] __attribute__((aligned(16)));

// FIR State
static float fir_state_L[FRAME_SAMPLES + FIR_TAPS] __attribute__((aligned(16)));
static float fir_state_R[FRAME_SAMPLES + FIR_TAPS] __attribute__((aligned(16)));

// FIR Instances
fir_f32_t fir_inst_L;
fir_f32_t fir_inst_R;

// ===================== COEFFICIENTS =====================
// 16-Tap Moving Average Filter
// Each coefficient is simply 1.0 / 16.0 = 0.0625
// This creates massive notches at 3kHz, 6kHz... 21kHz
static float fir_coeffs[FIR_TAPS] __attribute__((aligned(16))) = {
    0.0625, 0.0625, 0.0625, 0.0625, 
    0.0625, 0.0625, 0.0625, 0.0625, 
    0.0625, 0.0625, 0.0625, 0.0625, 
    0.0625, 0.0625, 0.0625, 0.0625
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
        .dma_buf_len = FRAME_SAMPLES,
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

// ===================== I2S TASK =====================
void i2s_task(void *p) {
    size_t bytes;
    static audio_frame_t frame;

    while (1) {
        i2s_read(I2S_PORT, i2s_rx, sizeof(i2s_rx), &bytes, portMAX_DELAY);
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            frame.left[i]  = i2s_rx[2*i + 1] >> 8;
            frame.right[i] = i2s_rx[2*i]     >> 8;
        }
        xQueueSend(audio_queue, &frame, portMAX_DELAY);
    }
}

// ===================== FFT VERIFY TASK =====================
// ===================== FFT PEAK FINDER TASK =====================
void fft_verify_task(void *p) {
    audio_frame_t f;

    while (1) {
        xQueueReceive(audio_queue, &f, portMAX_DELAY);

        // 1. Convert to Float
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            xL_raw[i] = (float)f.left[i];
        }

        // 2. APPLY FILTER (The Noise Killer)
        dsps_fir_f32_aes3(&fir_inst_L, xL_raw, xL_filt, FRAME_SAMPLES);

        // 3. WINDOWING
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            X1[2*i]   = xL_filt[i] * window[i];
            X1[2*i+1] = 0.0f;
        }

        // 4. FFT
        dsps_fft2r_fc32(X1, FRAME_SAMPLES);
        dsps_bit_rev_fc32(X1, FRAME_SAMPLES);

        // 5. FIND DOMINANT FREQUENCY
        float max_mag = 0;
        int max_bin = 0;

        // Only search up to 4kHz (Bin ~42) because of our low-pass filter
        // Searching higher might accidentally pick up residual ultrasonic noise
        for (int k = 1; k < 45; k++) { // Start at 1 to ignore DC (0Hz)
            float re = X1[2*k];
            float im = X1[2*k + 1];
            float mag = sqrtf(re*re + im*im);

            if (mag > max_mag) {
                max_mag = mag;
                max_bin = k;
            }
        }

        // 6. OUTPUT
        if (max_mag > 500.0f) { // Noise Gate
            float dominant_freq = (float)max_bin * SAMPLE_RATE / FRAME_SAMPLES;
            
            Serial.print("🔊 DOMINANT: ");
            Serial.print(dominant_freq, 1);
            Serial.print(" Hz  (Mag: ");
            Serial.print(max_mag, 0);
            Serial.println(")");
        } else {
            Serial.println("... silence ...");
        }

        vTaskDelay(pdMS_TO_TICKS(250)); // Update 4 times per second
    }
} 


// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    dsps_fft2r_init_fc32(NULL, FRAME_SAMPLES);
    dsps_wind_hann_f32(window, FRAME_SAMPLES);

    // Init FIR Instances
    memset(fir_state_L, 0, sizeof(fir_state_L));
    memset(fir_state_R, 0, sizeof(fir_state_R));
    
    // Using simple hardcoded averaging coefficients
    dsps_fir_init_f32(&fir_inst_L, fir_coeffs, fir_state_L, FIR_TAPS);
    dsps_fir_init_f32(&fir_inst_R, fir_coeffs, fir_state_R, FIR_TAPS);

    setup_i2s();
    audio_queue = xQueueCreate(3, sizeof(audio_frame_t));

    xTaskCreatePinnedToCore(i2s_task, "I2S", 6144, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(fft_verify_task, "FFT_VERIFY", 8192, NULL, 8, NULL, 1);
}

void loop() {}