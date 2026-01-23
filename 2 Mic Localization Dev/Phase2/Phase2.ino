#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>

// ===================== I2S CONFIG =====================
#define I2S_PORT        I2S_NUM_0
#define I2S_BCLK_PIN    5
#define I2S_WS_PIN      6
#define I2S_DATA_PIN    7

#define SAMPLE_RATE     48000
#define FRAME_SAMPLES   512

// ===================== DATA STRUCTURES =====================
typedef struct {
    int32_t left[FRAME_SAMPLES];
    int32_t right[FRAME_SAMPLES];
} audio_frame_t;

// ===================== QUEUES =====================
QueueHandle_t audio_queue;
QueueHandle_t result_queue;

// ===================== STATIC BUFFERS =====================
static int32_t i2s_rx_buffer[FRAME_SAMPLES * 2];
static audio_frame_t i2s_frame;

// ===================== RESULT STRUCT =====================
typedef struct {
    float rmsL;
    float rmsR;
    float diff;
} rms_result_t;

// ==========================================================
// I2S INITIALIZATION
// ==========================================================
void setup_i2s()
{
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = FRAME_SAMPLES,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK_PIN,
        .ws_io_num = I2S_WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DATA_PIN
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
    i2s_zero_dma_buffer(I2S_PORT);
}

// ==========================================================
// TASK: I2S CAPTURE (CORE 0)
// ==========================================================
void i2s_task(void *param)
{
    size_t bytes_read;

    while (1) {
        i2s_read(I2S_PORT,
                 i2s_rx_buffer,
                 sizeof(i2s_rx_buffer),
                 &bytes_read,
                 portMAX_DELAY);

        if (bytes_read != sizeof(i2s_rx_buffer)) {
            continue;
        }

        // De-interleave stereo samples
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            i2s_frame.left[i]  = i2s_rx_buffer[2 * i + 1] >> 8;
            i2s_frame.right[i] = i2s_rx_buffer[2 * i]     >> 8;
        }

        xQueueSend(audio_queue, &i2s_frame, portMAX_DELAY);
    }
}

// ==========================================================
// TASK: DSP VALIDATION (CORE 1)
// ==========================================================
void dsp_task(void *param)
{
    audio_frame_t frame;
    rms_result_t result;

    while (1) {
        if (xQueueReceive(audio_queue, &frame, portMAX_DELAY)) {

            float energyL = 0.0f;
            float energyR = 0.0f;

            for (int i = 0; i < FRAME_SAMPLES; i++) {
                float l = (float)frame.left[i];
                float r = (float)frame.right[i];

                energyL += l * l;
                energyR += r * r;
            }

            result.rmsL = sqrtf(energyL / FRAME_SAMPLES);
            result.rmsR = sqrtf(energyR / FRAME_SAMPLES);

            if (isnan(result.rmsL) || isnan(result.rmsR)) {
                continue;   // defensive: drop bad frame
            }

            result.diff = fabsf(result.rmsL - result.rmsR);

            xQueueSend(result_queue, &result, portMAX_DELAY);
        }
    }
}

// ==========================================================
// TASK: OUTPUT / LOGGER (CORE 1)
// ==========================================================
void output_task(void *param)
{
    rms_result_t res;

    while (1) {
        if (xQueueReceive(result_queue, &res, portMAX_DELAY)) {
            Serial.print("RMS L: ");
            Serial.print(res.rmsL, 2);
            Serial.print(" | RMS R: ");
            Serial.print(res.rmsR, 2);
            Serial.print(" | |Δ|: ");
            Serial.println(res.diff, 2);
        }
    }
}

// ==========================================================
// SETUP
// ==========================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== Phase 2.5: Channel Validation ===");

    setup_i2s();

    audio_queue  = xQueueCreate(3, sizeof(audio_frame_t));
    result_queue = xQueueCreate(5, sizeof(rms_result_t));

    xTaskCreatePinnedToCore(
        i2s_task,
        "I2S Task",
        6144,
        NULL,
        10,
        NULL,
        0
    );

    xTaskCreatePinnedToCore(
        dsp_task,
        "DSP Task",
        8192,
        NULL,
        8,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        output_task,
        "Output Task",
        2048,
        NULL,
        5,
        NULL,
        1
    );
}

void loop() {}
