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

#ifndef SEN0240_H
#define SEN0240_H

#include <stdint.h>
#include "omnia_types.h"
#include "omnia_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file sen0240.h
 * @brief Logical driver for the SEN0240 EMG sensor.
 *
 * This module implements the logical handling of the SEN0240 EMG sensor.
 * It is completely hardware-agnostic and relies on a pre-configured
 * @ref omnia_adc_handle_t provided by the platform.
 *
 * The driver:
 * - Does NOT configure or own the ADC hardware.
 * - Assumes the ADC channel, resolution and vref are already set.
 * - Provides raw, voltage, centered and normalized readings.
 * - Supports baseline calibration via multi-sample averaging.
 */

/**
 * @brief SEN0240 logical sensor context.
 *
 * This structure represents the logical state of the EMG sensor.
 * It contains:
 * - A copy of the ADC handle used to read samples.
 * - Calibration parameters used to center and normalize the signal.
 *
 * The ADC handle is copied by value during initialization, allowing the
 * driver to remain independent from the lifetime of external objects.
 */
typedef struct {
    /** ADC handle associated with the SEN0240 sensor. */
    omnia_adc_handle_t adc;

    /** Baseline offset in volts (typically ~1.5 V). */
    float              v_offset;

    /** Gain factor used to normalize the centered signal. */
    float              gain_volts;
} sen0240_t;

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the SEN0240 sensor context.
 *
 * This function does NOT configure the ADC hardware. It assumes that:
 * - The ADC is already initialized and configured.
 * - The provided ADC handle accurately reflects that configuration.
 *
 * @param s           Sensor context to initialize.
 * @param adc         Pointer to a configured ADC handle (copied by value).
 * @param v_offset    Baseline voltage in volts (typically 1.5f).
 * @param gain_volts  Gain factor used for normalization (e.g. vref / 2).
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if @p s or @p adc is NULL.
 * @return OMNIA_ENOTSUP if the active port does not provide ADC support.
 */
omnia_status_t sen0240_init(sen0240_t* s,
                            const omnia_adc_handle_t* adc,
                            float v_offset,
                            float gain_volts);

/* -------------------------------------------------------------------------- */
/* Reading API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a raw ADC sample from the SEN0240.
 *
 * The returned value is the raw ADC code (no conversion or calibration).
 *
 * @param s        Sensor context.
 * @param out_raw  Output raw ADC value.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return Other error codes propagated from the ADC backend.
 */
omnia_status_t sen0240_read_raw(sen0240_t* s, uint16_t* out_raw);

/**
 * @brief Read an absolute voltage sample.
 *
 * The returned voltage is referenced to the ADC vref and is not baseline-corrected.
 *
 * @param s          Sensor context.
 * @param out_volts  Output voltage in volts.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return Other error codes propagated from the ADC backend.
 */
omnia_status_t sen0240_read_volts(sen0240_t* s, float* out_volts);

/**
 * @brief Read a baseline-centered voltage sample.
 *
 * The baseline offset configured at initialization is subtracted from the
 * measured voltage.
 *
 * Example:
 * @code
 * v_offset = 1.5 V
 * measured = 1.55 V
 * centered = +0.05 V
 * @endcode
 *
 * @param s             Sensor context.
 * @param out_centered  Output centered voltage in volts.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return Other error codes if ADC reading fails.
 */
omnia_status_t sen0240_read_centered(sen0240_t* s, float* out_centered);

/**
 * @brief Read a centered and normalized sample.
 *
 * Computation:
 * @code
 * norm = (v - v_offset) / gain_volts
 * @endcode
 *
 * This representation is convenient for ML pipelines, typically producing
 * values in the approximate range [-1, +1].
 *
 * @param s        Sensor context.
 * @param out_norm Output centered and normalized value.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return Other error codes if ADC reading fails.
 */
omnia_status_t sen0240_read_centered_norm(sen0240_t* s, float* out_norm);

/* -------------------------------------------------------------------------- */
/* Calibration                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Calibrate the baseline offset by averaging multiple samples.
 *
 * The function reads @p n_samples consecutive voltage samples and computes
 * their mean to establish a new baseline offset.
 *
 * This calibration is typically performed while the muscle is at rest.
 *
 * @param s           Sensor context.
 * @param n_samples   Number of samples to average.
 * @param delay_ms    Delay in milliseconds between samples (0 for none).
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if parameters are invalid.
 * @return OMNIA_ENOTSUP if the active port does not provide timing support.
 * @return Other error codes if ADC reading fails.
 */
omnia_status_t sen0240_calibrate_baseline(sen0240_t* s,
                                          uint32_t n_samples,
                                          uint32_t delay_ms);

#ifdef __cplusplus
}
#endif

#endif /* SEN0240_H */
