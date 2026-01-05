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


#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file adc_stream.h
 * @brief Lightweight ADC streaming interface using DMA block callbacks.
 *
 * This module defines a minimal configuration-driven interface to stream
 * ADC samples in fixed-size blocks. It is designed to work with DMA circular
 * mode and HAL callbacks (half-transfer / full-transfer).
 *
 * Design goals:
 * - No dynamic allocation.
 * - Deterministic, low-latency delivery of ADC samples.
 * - Suitable for real-time signal processing (e.g. EMG, biosignals).
 *
 * The implementation is platform-specific (STM32 HAL),
 * but the interface is intentionally simple and portable.
 */

/* -------------------------------------------------------------------------- */
/* Callback types                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief ADC block callback.
 *
 * Called whenever a block of ADC samples is ready.
 *
 * @param samples Pointer to a contiguous block of ADC samples.
 *                The memory is owned by the driver and valid only
 *                for the duration of the callback.
 * @param n       Number of samples in the block.
 * @param user    User-provided opaque pointer (from adc_stream_cfg_t).
 *
 * Notes:
 * - This callback is typically invoked from an ISR context
 *   (DMA half/full transfer interrupt).
 * - Keep processing short and non-blocking.
 * - If heavier processing is required, copy data or signal a task.
 */
typedef void (*adc_block_cb_t)(const uint16_t* samples, size_t n, void* user);

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief ADC streaming configuration.
 *
 * This structure fully describes how the ADC stream operates.
 * It is copied internally at initialization time.
 */
typedef struct {
  uint32_t       fs_hz;      /**< Sampling frequency in Hz (informative). */
  size_t         block_n;    /**< Samples per block (e.g. 32 or 64). */
  adc_block_cb_t on_block;   /**< Callback invoked on each completed block. */
  void*          user;       /**< User-defined context pointer. */
} adc_stream_cfg_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the ADC streaming module.
 *
 * Copies the provided configuration and prepares internal buffers.
 * This function does NOT start ADC conversions.
 *
 * @param cfg Pointer to a valid configuration structure.
 */
void adc_stream_init(const adc_stream_cfg_t* cfg);

/**
 * @brief Start ADC streaming.
 *
 * Starts ADC conversions using DMA in circular mode.
 *
 * @return 0   on success
 * @return <0  on error (invalid config or HAL failure)
 *
 * Notes:
 * - The ADC trigger source (e.g. timer TRGO) must already be configured.
 * - The block callback will start being invoked after this call.
 */
int adc_stream_start(void);

/**
 * @brief Stop ADC streaming.
 *
 * Stops the ADC DMA transfer.
 * Safe to call even if the stream is not running.
 */
void adc_stream_stop(void);

#ifdef __cplusplus
}
#endif
