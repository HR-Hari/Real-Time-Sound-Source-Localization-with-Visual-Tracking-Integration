#include <Arduino.h>
#include "driver/i2s.h"
#include "esp_dsp.h"
#include <math.h>

// ===================== CONFIG =====================
#define SAMPLE_RATE     48000
#define FRAME_SAMPLES   1024 
#define I2S_PORT        I2S_NUM_0
#define BCLK_PIN        5
#define WS_PIN          6
#define DATA_PIN        7
#define FIR_TAPS        63

// ===================== GLOBALS =====================
static int32_t i2s_rx[FRAME_SAMPLES * 2];
static float xL[FRAME_SAMPLES];
static float xR[FRAME_SAMPLES];
static float xLf[FRAME_SAMPLES];
static float xRf[FRAME_SAMPLES];

// Buffers for the filter Delay Line (State)
static float fir_state_L[FRAME_SAMPLES + FIR_TAPS];
static float fir_state_R[FRAME_SAMPLES + FIR_TAPS];

// FIR Instance Structures
fir_f32_t fir_inst_L;
fir_f32_t fir_inst_R;

// Bandpass Coefficients (No 'const' allowed for DSP init)
static float fir_bp[FIR_TAPS] = {
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

void setup() {
    Serial.begin(115200);
    setup_i2s();
    
    // Clear filter state memory
    memset(fir_state_L, 0, sizeof(fir_state_L));
    memset(fir_state_R, 0, sizeof(fir_state_R));

    // Initialize FIR Structures
    dsps_fir_init_f32(&fir_inst_L, fir_bp, fir_state_L, FIR_TAPS);
    dsps_fir_init_f32(&fir_inst_R, fir_bp, fir_state_R, FIR_TAPS);
    
    Serial.println("Starting Energy Calibration...");
    delay(1000);
    
    Serial.println("Combined_Energy,Silence_Baseline,Active_Speech_Target");
}

void loop() {
    size_t bytes;
    i2s_read(I2S_PORT, i2s_rx, sizeof(i2s_rx), &bytes, portMAX_DELAY);

    // 1. Convert to float
    for (int i = 0; i < FRAME_SAMPLES; i++) {
        xL[i] = (float)(i2s_rx[2*i+1] >> 8); 
        xR[i] = (float)(i2s_rx[2*i]   >> 8);
    }

    // 2. Apply FIR Filter (USING S3 SPECIFIC FUNCTION: aes3)
    dsps_fir_f32_aes3(&fir_inst_L, xL, xLf, FRAME_SAMPLES);
    dsps_fir_f32_aes3(&fir_inst_R, xR, xRf, FRAME_SAMPLES);

    // 3. Calculate RMS
    float energyL = 0;
    float energyR = 0;
    dsps_dotprod_f32(xLf, xLf, &energyL, FRAME_SAMPLES);
    dsps_dotprod_f32(xRf, xRf, &energyR, FRAME_SAMPLES);

    energyL = sqrtf(energyL / FRAME_SAMPLES);
    energyR = sqrtf(energyR / FRAME_SAMPLES);
    
    float totalEnergy = energyL + energyR;

    // 4. Print for Serial Plotter
    Serial.print("Energy:");
    Serial.print(totalEnergy);
    Serial.print(",");
    Serial.print("Threshold_Low:1000"); 
    Serial.print(",");
    Serial.println("Threshold_High:5000"); 
    
    delay(10); 
}