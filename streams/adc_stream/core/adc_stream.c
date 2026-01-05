#include "adc_stream.h"
#include <string.h>

// --- Depèn del teu projecte CubeMX ---
#include "main.h"        // o on declares hadc1
#include "adc.h"         // hadc1
// extern ADC_HandleTypeDef hadc1;

#ifndef ADC_STREAM_MAX_BUF
#define ADC_STREAM_MAX_BUF 256u
#endif

static adc_stream_cfg_t g_cfg;

// DMA buffer: len = 2*block_n (half/full callback)
static uint16_t g_dma_buf[ADC_STREAM_MAX_BUF];

// Guardem la len real
static size_t g_dma_len = 0;

void adc_stream_init(const adc_stream_cfg_t* cfg)
{
  if (!cfg) return;
  g_cfg = *cfg;

  // DMA len = 2*block
  g_dma_len = g_cfg.block_n * 2u;
  if (g_dma_len > ADC_STREAM_MAX_BUF) {
    // Si vols, aquí fes assert/log o retalla
    g_dma_len = ADC_STREAM_MAX_BUF;
  }

  // Netegem buffer per higiene
  memset(g_dma_buf, 0, sizeof(g_dma_buf));
}

int adc_stream_start(void)
{
  if (!g_cfg.on_block || g_cfg.block_n == 0 || g_dma_len == 0) return -1;

  // IMPORTANT:
  // Si el teu trigger és TIM3 TRGO, assegura't que TIM3 està engegat abans o després,
  // però que ja corre quan esperes mostres.
  // HAL_TIM_Base_Start(&htim3);  // si no el tens ja arrencat

  // Arrenca ADC + DMA en circular
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_dma_buf, (uint32_t)g_dma_len) != HAL_OK) {
    return -2;
  }
  return 0;
}

void adc_stream_stop(void)
{
  HAL_ADC_Stop_DMA(&hadc1);
}

// --- Callbacks HAL: es disparen a mitja transferència i a transferència completa ---
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc != &hadc1) return;
  if (!g_cfg.on_block) return;

  // Primera meitat: [0 .. block_n-1]
  g_cfg.on_block(&g_dma_buf[0], g_cfg.block_n, g_cfg.user);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc != &hadc1) return;
  if (!g_cfg.on_block) return;

  // Segona meitat: [block_n .. 2*block_n-1]
  g_cfg.on_block(&g_dma_buf[g_cfg.block_n], g_cfg.block_n, g_cfg.user);
}
