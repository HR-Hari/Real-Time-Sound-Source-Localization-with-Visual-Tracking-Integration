#include <Arduino.h>
#include "driver/i2s.h"
#include "esp_dsp.h"
#include <math.h>

// ===================== PARAMETERS =====================
#define SAMPLE_RATE     48000
#define FRAME_SAMPLES   512
#define I2S_PORT        I2S_NUM_0
#define BCLK_PIN        5
#define WS_PIN          6
#define DATA_PIN        7
#define FIR_TAPS        16   // 16-tap Averager

// ===================== ALIGNED GLOBALS =====================
static int32_t i2s_rx[FRAME_SAMPLES * 2];

// Buffers
static float xL_raw[FRAME_SAMPLES] __attribute__((aligned(16)));
static float xL_filt[FRAME_SAMPLES] __attribute__((aligned(16)));

// FIR State & Instance
static float fir_state_L[FRAME_SAMPLES + FIR_TAPS] __attribute__((aligned(16)));
fir_f32_t fir_inst_L;

// 16-Tap Moving Average Coefficients
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

// ===================== RMS DEBUG TASK =====================
void rms_debug_task(void *p) {
    audio_frame_t f;

    while (1) {
        xQueueReceive(audio_queue, &f, portMAX_DELAY);

        // 1. Convert to Float
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            xL_raw[i] = (float)f.left[i];
        }

        // 2. CALCULATE RAW RMS (Before Filter)
        float energy_raw = 0;
        dsps_dotprod_f32(xL_raw, xL_raw, &energy_raw, FRAME_SAMPLES);
        float rms_raw = sqrtf(energy_raw / FRAME_SAMPLES);

        // 3. APPLY FILTER
        dsps_fir_f32_aes3(&fir_inst_L, xL_raw, xL_filt, FRAME_SAMPLES);

        // 4. CALCULATE FILTERED RMS (After Filter)
        float energy_filt = 0;
        dsps_dotprod_f32(xL_filt, xL_filt, &energy_filt, FRAME_SAMPLES);
        float rms_filt = sqrtf(energy_filt / FRAME_SAMPLES);

        // 5. PRINT COMPARISON
        // We use Serial.printf for clean formatting
        Serial.printf("Raw: %6.0f  |  Filtered: %6.0f", rms_raw, rms_filt);

        // Add a visual indicator if signal is detected
        if (rms_filt > 500) {
            Serial.print("  <<< SIGNAL DETECTED");
        }
        Serial.println();

        vTaskDelay(pdMS_TO_TICKS(100)); // Update 10 times per second
    }
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    memset(fir_state_L, 0, sizeof(fir_state_L));
    dsps_fir_init_f32(&fir_inst_L, fir_coeffs, fir_state_L, FIR_TAPS);

    setup_i2s();
    audio_queue = xQueueCreate(3, sizeof(audio_frame_t));

    xTaskCreatePinnedToCore(i2s_task, "I2S", 6144, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(rms_debug_task, "RMS", 8192, NULL, 8, NULL, 1);
}

void loop() {}