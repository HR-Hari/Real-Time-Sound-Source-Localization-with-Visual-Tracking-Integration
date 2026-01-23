#include <Arduino.h>
#include "driver/i2s.h"

// ================== I2S CONFIG ==================
#define I2S_PORT       I2S_NUM_0

#define I2S_BCLK_PIN   5
#define I2S_WS_PIN     6
#define I2S_DATA_PIN   7

#define SAMPLE_RATE    48000
#define FRAME_SAMPLES  512   // per channel

// Raw I2S buffer (stereo: L,R,L,R...)
int32_t i2s_rx_buffer[FRAME_SAMPLES * 2];

// Separated channels
int32_t left_ch[FRAME_SAMPLES];
int32_t right_ch[FRAME_SAMPLES];

// =================================================

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

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=== Phase 1: I2S MEMS Stereo Validation (Corrected) ===");

  setup_i2s();

  Serial.println("I2S initialized. Feed 1 kHz tone.");
}

void loop()
{
  size_t bytes_read = 0;

  // Read one frame
  i2s_read(I2S_PORT,
            i2s_rx_buffer,
            sizeof(i2s_rx_buffer),
            &bytes_read,
            portMAX_DELAY);

  if (bytes_read != sizeof(i2s_rx_buffer)) {
    Serial.println("Partial I2S read!");
    return;
  }

  // ================== IMPORTANT FIX ==================
  // Swapped channel extraction
  // Many MEMS mics output LEFT data in RIGHT slot and vice versa
  for (int i = 0; i < FRAME_SAMPLES; i++) {
    left_ch[i]  = i2s_rx_buffer[2 * i + 1] >> 8;
    right_ch[i] = i2s_rx_buffer[2 * i]     >> 8;
  }
  // ===================================================

  // ---- Print first 16 samples ----
  Serial.println("\nL\tR");
  for (int i = 0; i < 16; i++) {
    Serial.print(left_ch[i]);
    Serial.print("\t");
    Serial.println(right_ch[i]);
  }

  // ---- Simple time-domain correlation check ----
  int max_lag = 20;
  int best_lag = 0;
  int64_t best_corr = 0;

  for (int lag = -max_lag; lag <= max_lag; lag++) {
    int64_t corr = 0;
    for (int n = max_lag; n < FRAME_SAMPLES - max_lag; n++) {
      corr += (int64_t)left_ch[n] * right_ch[n - lag];
    }
    if (corr > best_corr) {
      best_corr = corr;
      best_lag = lag;
    }
  }

  Serial.print("Estimated lag (samples): ");
  Serial.println(best_lag);

  delay(1000);
}
