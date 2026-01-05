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
 * @file app_emg.c
 * @brief EMG processing: baseline calibration, normalization and block statistics.
 *
 * This module processes raw ADC samples from the SEN0240 front-end and exposes:
 * - Latest raw value and voltage.
 * - Centered signal (volts minus baseline offset).
 * - Normalized signal in [-1, 1] using a configurable gain.
 * - Saturation flag and optional per-block peak/RMS metrics.
 *
 * The baseline (offset) is acquired during a calibration phase as the mean voltage
 * over a fixed number of samples.
 */

#include "app_emg.h"
#include "omnia_port.h"
#include <math.h>

/**
 * @brief Clamp a floating point value to the provided bounds.
 *
 * @param x  Input value.
 * @param lo Lower bound.
 * @param hi Upper bound.
 * @return Clamped value.
 */
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**
 * @brief Initialize the EMG processing instance.
 *
 * This function initializes the SEN0240 driver and resets all runtime outputs
 * and calibration state.
 *
 * @param a               EMG processing instance.
 * @param adc             ADC handle used by the sensor front-end.
 * @param v_offset_initial Initial baseline voltage (used until calibration completes).
 * @param gain_volts      Voltage that maps to a normalized amplitude of 1.0.
 */
void app_emg_init(app_emg_t* a,
                  const omnia_adc_handle_t* adc,
                  float v_offset_initial,
                  float gain_volts)
{
    if (!a || !adc) return;

    (void)sen0240_init(&a->sen, adc, v_offset_initial, gain_volts);

    a->raw       = 0;
    a->volts     = 0.0f;
    a->centered  = 0.0f;
    a->norm      = 0.0f;
    a->saturated = 0;

    a->state        = APP_EMG_STATE_RUN;
    a->has_baseline = 0;

    a->calib_samples = 0;
    a->calib_target  = APP_EMG_CALIB_SAMPLES;
    a->calib_accum   = 0.0f;

    a->gain_volts    = (gain_volts > 0.0f) ? gain_volts : 1.0f;

    a->last_block_rms  = 0.0f;
    a->last_block_peak = 0.0f;
}

/**
 * @brief Start baseline calibration.
 *
 * Calibration accumulates the mean voltage over @ref a->calib_target samples.
 * During calibration, signal outputs are suppressed (centered/norm set to 0).
 *
 * @param a EMG processing instance.
 */
void app_emg_start_calibration(app_emg_t* a)
{
    if (!a) return;

    a->state         = APP_EMG_STATE_CALIBRATING;
    a->calib_samples = 0;
    a->calib_accum   = 0.0f;
    a->has_baseline  = 0;

    a->centered  = 0.0f;
    a->norm      = 0.0f;
    a->saturated = 0;

    a->last_block_rms  = 0.0f;
    a->last_block_peak = 0.0f;
}

/**
 * @brief Process a single sample (compatibility API).
 *
 * This helper reads one raw sample from the sensor and forwards it to the
 * block-based processor.
 *
 * @param a EMG processing instance.
 */
void app_emg_step(app_emg_t* a)
{
    if (!a) return;

    uint16_t raw = 0;
    if (sen0240_read_raw(&a->sen, &raw) != OMNIA_OK) {
        return;
    }

    app_emg_process_block_raw(a, &raw, 1);
}

/**
 * @brief Process a block of raw ADC samples.
 *
 * Typical use case is DMA-driven acquisition. For each sample, the function:
 * - Converts raw to volts.
 * - Publishes the latest raw and voltage values.
 * - If calibrating: updates baseline estimation and suppresses signal outputs.
 * - If running with baseline: computes centered and normalized values.
 * - Updates saturation flag and accumulates optional per-block statistics.
 *
 * @param a   EMG processing instance.
 * @param raw Pointer to raw ADC samples.
 * @param n   Number of samples in the block.
 */
void app_emg_process_block_raw(app_emg_t* a,
                               const uint16_t* raw,
                               size_t n)
{
    if (!a || !raw || n == 0) return;

    float peak = 0.0f;
    float acc2 = 0.0f;

    for (size_t i = 0; i < n; ++i) {
        const uint16_t r = raw[i];
        const float v   = omnia_adc_raw_to_volts(&a->sen.adc, r);

        /* Publish latest raw/voltage (always). */
        a->raw   = r;
        a->volts = v;

        if (a->state == APP_EMG_STATE_CALIBRATING) {
            /* Baseline estimation as mean voltage. */
            a->calib_accum   += v;
            a->calib_samples += 1U;

            if (a->calib_samples >= a->calib_target) {
                const float new_offset =
                    a->calib_accum / (float)a->calib_samples;

                a->sen.v_offset = new_offset;
                a->state        = APP_EMG_STATE_RUN;
                a->has_baseline = 1;
            }

            /* Suppress signal outputs during calibration. */
            a->centered  = 0.0f;
            a->norm      = 0.0f;
            a->saturated = 0;
            continue;
        }

        /* If baseline is not available, do not emit signal outputs. */
        if (!a->has_baseline) {
            a->centered  = 0.0f;
            a->norm      = 0.0f;
            a->saturated = 0;
            continue;
        }

        /* RUN: compute centered and normalized signals. */
        const float centered = v - a->sen.v_offset;
        float norm = centered / a->gain_volts;
        norm = clampf(norm, -1.0f, 1.0f);

        a->centered  = centered;
        a->norm      = norm;
        a->saturated = (fabsf(norm) > 0.98f) ? 1 : 0;

        /* Optional per-block statistics for UI/telemetry. */
        const float an = fabsf(norm);
        if (an > peak) peak = an;
        acc2 += norm * norm;
    }

    /* Update block summary only when running and baseline is available. */
    if (a->state == APP_EMG_STATE_RUN && a->has_baseline) {
        a->last_block_peak = peak;
        a->last_block_rms  = (n > 0) ? sqrtf(acc2 / (float)n) : 0.0f;
    } else {
        a->last_block_peak = 0.0f;
        a->last_block_rms  = 0.0f;
    }
}
