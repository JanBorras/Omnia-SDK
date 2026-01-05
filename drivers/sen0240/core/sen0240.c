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
 * @file sen0240.c
 * @brief SEN0240 EMG sensor logical driver (ADC-based).
 *
 * This module implements the logical handling of the SEN0240 EMG sensor.
 * It does NOT configure or own the ADC hardware. Instead, it relies on
 * a pre-configured @ref omnia_adc_handle_t provided by the platform.
 *
 * Responsibilities:
 * - Validate ADC availability through the Omnia port contract.
 * - Store calibration parameters (baseline offset and gain).
 * - Provide raw, voltage, centered and normalized readings.
 * - Support baseline calibration through multi-sample averaging.
 */

#include "sen0240.h"
#include "omnia_port.h" /* for omnia_port_has_adc() */

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the SEN0240 sensor context.
 *
 * This function does NOT configure the ADC hardware. It assumes:
 * - The ADC is already initialized and configured (channel, sampling time, vref, etc.).
 * - The provided @ref omnia_adc_handle_t accurately reflects that configuration.
 *
 * The function:
 * - Validates input pointers.
 * - Checks that the active port exposes ADC support.
 * - Copies the ADC handle and calibration parameters into the sensor context.
 *
 * @param s           Sensor context.
 * @param adc         Pointer to a configured ADC handle (copied by value).
 * @param v_offset    Baseline voltage in volts (typically ~1.5 V).
 * @param gain_volts  Scaling factor used for signal normalization.
 *
 * @return OMNIA_OK            on success.
 * @return OMNIA_EINVAL        if @p s or @p adc is NULL.
 * @return OMNIA_ENOTSUP       if the active port does not provide ADC support.
 */
omnia_status_t sen0240_init(sen0240_t* s,
                            const omnia_adc_handle_t* adc,
                            float v_offset,
                            float gain_volts)
{
    if (!s || !adc) {
        return OMNIA_EINVAL;
    }

    /* Ensure the active port declares ADC support. */
    if (!omnia_port_has_adc()) {
        return OMNIA_ENOTSUP;
    }

    /* Copy ADC handle by value to decouple from caller lifetime. */
    s->adc = *adc;

    /* Store baseline offset (volts). */
    s->v_offset = v_offset;

    /* Store gain factor, guarding against invalid values. */
    s->gain_volts = (gain_volts > 0.0f) ? gain_volts : 1.0f;

    return OMNIA_OK;
}

/* -------------------------------------------------------------------------- */
/* Raw and voltage readings                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a raw ADC sample from the SEN0240.
 *
 * No calibration or conversion is applied. The returned value is the
 * direct ADC code (e.g. 0..4095 for a 12-bit ADC).
 *
 * @param s        Sensor context.
 * @param out_raw  Output raw ADC value.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if @p s or @p out_raw is NULL.
 * @return Other error codes propagated from the ADC backend.
 */
omnia_status_t sen0240_read_raw(sen0240_t* s, uint16_t* out_raw)
{
    if (!s || !out_raw) {
        return OMNIA_EINVAL;
    }

    return omnia_adc_read_raw(&s->adc, out_raw);
}

/**
 * @brief Read a voltage sample from the SEN0240.
 *
 * The returned value is the absolute voltage measured at the ADC pin,
 * referenced to the ADC vref. No baseline subtraction is applied.
 *
 * @param s          Sensor context.
 * @param out_volts  Output voltage in volts.
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if @p s or @p out_volts is NULL.
 * @return Other error codes propagated from the ADC backend.
 */
omnia_status_t sen0240_read_volts(sen0240_t* s, float* out_volts)
{
    if (!s || !out_volts) {
        return OMNIA_EINVAL;
    }

    return omnia_adc_read_volts(&s->adc, out_volts);
}

/* -------------------------------------------------------------------------- */
/* Centered and normalized readings                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a baseline-centered voltage sample.
 *
 * Flow:
 * 1. Read absolute voltage from the ADC.
 * 2. Subtract the configured baseline offset.
 *
 * Example:
 * - v_offset = 1.5 V
 * - measured = 1.55 V
 * - centered = +0.05 V
 *
 * @param s              Sensor context.
 * @param out_centered   Output centered voltage (volts).
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if @p s or @p out_centered is NULL.
 * @return Other error codes if ADC reading fails.
 */
omnia_status_t sen0240_read_centered(sen0240_t* s, float* out_centered)
{
    if (!s || !out_centered) {
        return OMNIA_EINVAL;
    }

    float v = 0.0f;

    omnia_status_t st = omnia_adc_read_volts(&s->adc, &v);
    if (st != OMNIA_OK) {
        return st;
    }

    *out_centered = v - s->v_offset;
    return OMNIA_OK;
}

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
 * @return OMNIA_EINVAL  if @p s or @p out_norm is NULL.
 * @return Other error codes if ADC reading fails.
 */
omnia_status_t sen0240_read_centered_norm(sen0240_t* s, float* out_norm)
{
    if (!s || !out_norm) {
        return OMNIA_EINVAL;
    }

    float centered = 0.0f;

    omnia_status_t st = sen0240_read_centered(s, &centered);
    if (st != OMNIA_OK) {
        return st;
    }

    *out_norm = centered / s->gain_volts;
    return OMNIA_OK;
}

/* -------------------------------------------------------------------------- */
/* Baseline calibration                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Calibrate the baseline offset by averaging multiple samples.
 *
 * The function reads @p n_samples consecutive voltage samples, optionally
 * waiting @p delay_ms milliseconds between each read, and computes their
 * mean to establish a new baseline offset.
 *
 * This is typically executed while the muscle is at rest.
 *
 * @param s           Sensor context.
 * @param n_samples   Number of samples to average.
 * @param delay_ms    Delay in milliseconds between samples (0 for none).
 *
 * @return OMNIA_OK      on success.
 * @return OMNIA_EINVAL  if @p s is NULL or @p n_samples is zero.
 * @return OMNIA_ENOTSUP if the active port does not provide delay_us().
 * @return Other error codes if ADC reading fails.
 */
omnia_status_t sen0240_calibrate_baseline(sen0240_t* s,
                                          uint32_t n_samples,
                                          uint32_t delay_ms)
{
    if (!s || n_samples == 0U) {
        return OMNIA_EINVAL;
    }

    omnia_port_t* port = omnia_port_get();
    if (!port || !port->v || !port->v->delay_us) {
        return OMNIA_ENOTSUP;
    }

    float acc = 0.0f;

    for (uint32_t i = 0; i < n_samples; ++i) {
        float v = 0.0f;

        omnia_status_t st = sen0240_read_volts(s, &v);
        if (st != OMNIA_OK) {
            return st;
        }

        acc += v;

        if (delay_ms > 0U) {
            port->v->delay_us(delay_ms * 1000U);
        }
    }

    s->v_offset = acc / (float)n_samples;
    return OMNIA_OK;
}
