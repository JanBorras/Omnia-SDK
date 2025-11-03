#include <stdio.h>
#include "edge/hal/i2c.h"
#include "edge/hal/time.h"
#include "edge/ml/tflm_api.h"

// Minimal BH1750 read sequence (pseudocode): I2C addr 0x23
static int bh1750_init(uint8_t bus){
  uint8_t pwr_on = 0x01;  // POWER_ON
  uint8_t cont_h = 0x10;  // CONTINUOUS_HIGH_RESOLUTION_MODE
  edge_i2c_write(bus, 0x23, &pwr_on, 1);
  edge_delay_ms(10);
  edge_i2c_write(bus, 0x23, &cont_h, 1);
  return 0;
}

static float bh1750_read_lux(uint8_t bus){
  uint8_t rx[2]={0};
  edge_i2c_read(bus, 0x23, rx, 2);
  uint16_t raw = ((uint16_t)rx[0]<<8) | rx[1];
  float lux = raw / 1.2f; // default conversion
  return lux;
}

int main(void){
  // Init I2C @400kHz on bus 0
  edge_i2c_cfg_t i2c0 = {.bus=0, .freq_hz=400000};
  edge_i2c_init(i2c0);
  bh1750_init(0);

  // TFLM stub context
  static uint8_t arena[64*1024];
  edge_tflm_ctx_t ctx = { .model_data=NULL, .model_size=0, .arena=arena, .arena_size=sizeof(arena) };
  edge_tflm_init(&ctx);

  while(1){
    float lux = bh1750_read_lux(0);
    float in[1] = { lux };
    float out[1] = { 0 };
    edge_tflm_infer(&ctx, in, 1, out, 1);

    printf("[solar_demo] lux=%.1f  P_est=%.3f W  t=%u us\n", lux, out[0], edge_tflm_last_latency_us());
    edge_delay_ms(500);
  }
  return 0;
}
