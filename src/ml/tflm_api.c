#include "edge/ml/tflm_api.h"
#include <stdint.h>
#include <string.h>

// NOTE: This is a stub facade. Wire up real TFLM interpreter here.
static uint32_t last_latency_us = 0;

int edge_tflm_init(edge_tflm_ctx_t* ctx){
  (void)ctx;
  last_latency_us = 0;
  return 0;
}

int edge_tflm_infer(edge_tflm_ctx_t* ctx, const float* input, uint32_t in_len,
                    float* output, uint32_t out_len){
  (void)ctx;
  // Dummy: copy first input to first output to prove the plumbing.
  if (in_len>0 && out_len>0){
    output[0] = input[0] * 0.001f; // pretend "power estimate"
  }
  last_latency_us = 100; // placeholder
  return 0;
}

uint32_t edge_tflm_last_latency_us(void){
  return last_latency_us;
}
