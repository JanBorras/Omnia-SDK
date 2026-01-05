// SPDX-License-Identifier: Apache-2.0
/*
 * Copyright (c) 2025 Jan Borràs Ros
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file adc_stream.c
 * @brief Simple ADC streaming helper using DMA circular mode (HAL-based).
 *
 * This module provides a minimal abstraction to continuously stream ADC
 * samples using DMA in circular mode, invoking a user callback on each
 * half-buffer and full-buffer completion.
 *
 * Design intent:
 * - Zero heap usage.
 * - Deterministic, low-latency data delivery.
 * - Suitable for real-time signal acquisition (e.g. EMG @ kHz rates).
 *
 * Scope:
 * - Single ADC instance (hadc1).
 * - One active stream at a time (global/static state).
 *
 * This module is intentionally simple and CubeMX-oriented.
 * Higher-level buffering, synchronization, or transport logic must be
 * implemented by the caller.
 */

#include "adc_stream.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Project-specific HAL dependencies                                          */
/* -------------------------------------------------------------------------- */

/*
 * These headers are expected to provide:
 *   - ADC_HandleTypeDef hadc1;
 *
 * Adjust includes if your CubeMX layout differs.
 */
#include "main.h"
#include "adc.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Maximum DMA buffer length (in uint16 samples).
 *
 * Actual DMA length is computed as:
 *   dma_len = 2 * block_n
 *
 * This macro provides an upper bound to avoid uncontrolled memory usage.
 */
#ifndef ADC_STREAM_MAX_BUF
#define ADC_STREAM_MAX_BUF 256u
#endif

/* -------------------------------------------------------------------------- */
/* Static state                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Active stream configuration (copied at init time).
 */
static adc_stream_cfg_t g_cfg;

/**
 * @brief DMA buffer used by ADC in circular mode.
 *
 * Layout:
 *   - First half  : samples [0 .. block_n - 1]
 *   - Second half : samples [block_n .. 2*block_n - 1]
 */
static uint16_t g_dma_buf[ADC_STREAM_MAX_BUF];

/**
 * @brief Actual DMA buffer length in samples.
 *
 * Equals 2 * block_n (clamped to ADC_STREAM_MAX_BUF).
 */
static size_t g_dma_len = 0;

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void adc_stream_init(const adc_stream_cfg_t* cfg)
{
  if (!cfg) return;

  /* Shallow copy: caller-owned callbacks and user pointer */
  g_cfg = *cfg;

  /* DMA length = two blocks (half + full callbacks) */
  g_dma_len = g_cfg.block_n * 2u;

  if (g_dma_len > ADC_STREAM_MAX_BUF) {
    /*
     * Safety clamp.
     * In a stricter design this could assert or return an error,
     * but here we prefer robustness over hard failure.
     */
    g_dma_len = ADC_STREAM_MAX_BUF;
  }

  /* Clear buffer for hygiene (not strictly required) */
  memset(g_dma_buf, 0, sizeof(g_dma_buf));
}

int adc_stream_start(void)
{
  if (!g_cfg.on_block || g_cfg.block_n == 0 || g_dma_len == 0) {
    return -1;
  }

  /*
   * IMPORTANT:
   * If the ADC is externally triggered (e.g. TIMx TRGO),
   * ensure the timer is running before or immediately after
   * starting the ADC DMA.
   *
   * Example:
   *   HAL_TIM_Base_Start(&htim3);
   */

  /* Start ADC in DMA circular mode */
  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t*)g_dma_buf,
                        (uint32_t)g_dma_len) != HAL_OK) {
    return -2;
  }

  return 0;
}

void adc_stream_stop(void)
{
  HAL_ADC_Stop_DMA(&hadc1);
}

/* -------------------------------------------------------------------------- */
/* HAL callbacks                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief ADC DMA half-transfer complete callback.
 *
 * Called by the HAL when the first half of the DMA buffer
 * has been filled.
 *
 * Data range:
 *   g_dma_buf[0 .. block_n - 1]
 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc != &hadc1) return;
  if (!g_cfg.on_block) return;

  g_cfg.on_block(&g_dma_buf[0],
                 g_cfg.block_n,
                 g_cfg.user);
}

/**
 * @brief ADC DMA full-transfer complete callback.
 *
 * Called by the HAL when the second half of the DMA buffer
 * has been filled.
 *
 * Data range:
 *   g_dma_buf[block_n .. 2*block_n - 1]
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc != &hadc1) return;
  if (!g_cfg.on_block) return;

  g_cfg.on_block(&g_dma_buf[g_cfg.block_n],
                 g_cfg.block_n,
                 g_cfg.user);
}
