#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

// Minimal TFLM facade so apps don't depend on TFLM headers directly
typedef struct {
  const uint8_t* model_data;
  uint32_t model_size;
  void* arena;
  uint32_t arena_size;
} edge_tflm_ctx_t;

// Initialize interpreter with model and arena.
// Returns 0 on success
int edge_tflm_init(edge_tflm_ctx_t* ctx);

// Run inference: single input/output flat arrays (demo purpose)
int edge_tflm_infer(edge_tflm_ctx_t* ctx, const float* input, uint32_t in_len,
                    float* output, uint32_t out_len);

// Optional: get last latency (us)
uint32_t edge_tflm_last_latency_us(void);

#ifdef __cplusplus
}
#endif
