#include <Arduino.h>
#include "driver/i2s.h"
#include "esp_dsp.h"
#include <math.h>

// ===================== PARAMETERS =====================
#define SAMPLE_RATE     48000
#define FRAME_SAMPLES   512
#define MIC_DISTANCE    0.1f     // meters (as requested)
#define SPEED_OF_SOUND  343.0f

#define I2S_PORT        I2S_NUM_0
#define BCLK_PIN        5
#define WS_PIN          6
#define DATA_PIN        7

// ===================== STRUCTS =====================
typedef struct {
    int32_t left[FRAME_SAMPLES];
    int32_t right[FRAME_SAMPLES];
} audio_frame_t;

QueueHandle_t audio_queue;

// ===================== BUFFERS =====================
static int32_t i2s_rx[FRAME_SAMPLES * 2];
static audio_frame_t frame;

static float window[FRAME_SAMPLES];

// ===================== I2S SETUP =====================
void setup_i2s() {
    i2s_config_t cfg = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 4,
        .dma_buf_len = FRAME_SAMPLES
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
void i2s_task(void *p) {
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
    audio_frame_t f;

    static float X1[2 * FRAME_SAMPLES];
    static float X2[2 * FRAME_SAMPLES];
    static float Rxy[2 * FRAME_SAMPLES];

    // ---- PHYSICAL MAX LAG (computed once) ----
    const int MAX_LAG = (int)((MIC_DISTANCE / SPEED_OF_SOUND) * SAMPLE_RATE);

    while (1) {
        xQueueReceive(audio_queue, &f, portMAX_DELAY);

        // ---------- DC OFFSET REMOVAL ----------
        float meanL = 0.0f, meanR = 0.0f;
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            meanL += f.left[i];
            meanR += f.right[i];
        }
        meanL /= FRAME_SAMPLES;
        meanR /= FRAME_SAMPLES;

        // ---------- RMS NORMALIZATION ----------
        float rmsL = 0.0f, rmsR = 0.0f;
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            float l = f.left[i]  - meanL;
            float r = f.right[i] - meanR;
            rmsL += l * l;
            rmsR += r * r;
        }
        rmsL = sqrtf(rmsL / FRAME_SAMPLES) + 1e-9f;
        rmsR = sqrtf(rmsR / FRAME_SAMPLES) + 1e-9f;

        // ---------- PACK + WINDOW ----------
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            X1[2*i]   = ((f.left[i]  - meanL) / rmsL) * window[i];
            X1[2*i+1] = 0.0f;

            X2[2*i]   = ((f.right[i] - meanR) / rmsR) * window[i];
            X2[2*i+1] = 0.0f;
        }

        // ---------- FFT ----------
        dsps_fft2r_fc32(X1, FRAME_SAMPLES);
        dsps_fft2r_fc32(X2, FRAME_SAMPLES);
        dsps_bit_rev_fc32(X1, FRAME_SAMPLES);
        dsps_bit_rev_fc32(X2, FRAME_SAMPLES);

        // ---------- GCC-PHAT ----------
        for (int i = 0; i < FRAME_SAMPLES; i++) {
            float re = X1[2*i]*X2[2*i] + X1[2*i+1]*X2[2*i+1];
            float im = X1[2*i+1]*X2[2*i] - X1[2*i]*X2[2*i+1];
            float mag = sqrtf(re*re + im*im) + 1e-9f;

            Rxy[2*i]   = re / mag;
            Rxy[2*i+1] = im / mag;
        }

        // ---------- IFFT ----------
        dsps_fft2r_fc32(Rxy, FRAME_SAMPLES);
        dsps_bit_rev_fc32(Rxy, FRAME_SAMPLES);

        // ---------- PHYSICALLY CONSTRAINED PEAK SEARCH ----------
        int bestLag = 0;
        float bestVal = 0.0f;

        for (int lag = -MAX_LAG; lag <= MAX_LAG; lag++) {
            int idx = (lag + FRAME_SAMPLES) % FRAME_SAMPLES;
            float v = fabsf(Rxy[2*idx]);
            if (v > bestVal) {
                bestVal = v;
                bestLag = lag;
            }
        }

        // ---------- ANGLE COMPUTATION (CLAMPED) ----------
        float tdoa = (float)bestLag / SAMPLE_RATE;
        float x = (tdoa * SPEED_OF_SOUND) / MIC_DISTANCE;

        // numerical safety
        x = fmaxf(-1.0f, fminf(1.0f, x));

        float angle = asinf(x) * 180.0f / PI;

        Serial.print("Lag: ");
        Serial.print(bestLag);
        Serial.print(" | Angle (deg): ");
        Serial.println(angle, 2);
    }
}

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    delay(1000);

    dsps_fft2r_init_fc32(NULL, FRAME_SAMPLES);

    for (int i = 0; i < FRAME_SAMPLES; i++) {
        window[i] = 0.5f * (1.0f - cosf(2 * PI * i / (FRAME_SAMPLES - 1)));
    }

    setup_i2s();
    audio_queue = xQueueCreate(3, sizeof(audio_frame_t));

    xTaskCreatePinnedToCore(i2s_task, "I2S", 6144, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(gccphat_task, "GCCPHAT", 8192, NULL, 8, NULL, 1);
}

void loop() {}
