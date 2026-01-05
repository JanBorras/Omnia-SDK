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

#ifndef APP_EMG_H
#define APP_EMG_H

#include "sen0240.h"
#include "omnia_adc.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file app_emg.h
 * @brief EMG processing API: calibration, normalization and signal metrics.
 *
 * This interface exposes a small EMG processing pipeline on top of the SEN0240
 * front-end. It supports:
 * - Baseline acquisition via calibration (mean voltage over a fixed sample count).
 * - Conversion of raw ADC samples to volts, centering and normalization.
 * - Optional block-level metrics (RMS/peak) for UI/telemetry.
 *
 * The primary processing entry point is block-based to support DMA acquisition.
 */

/**
 * @brief EMG processing state.
 */
typedef enum {
    /** Normal operation: baseline is available and signal is produced. */
    APP_EMG_STATE_RUN = 0,

    /** Calibration in progress: baseline is being estimated. */
    APP_EMG_STATE_CALIBRATING
} app_emg_state_t;

/**
 * @brief EMG processing context.
 *
 * The structure stores both the sensor front-end and the current processing
 * state and outputs. "Latest sample" fields are updated while processing blocks
 * and are suitable for UI display and debugging.
 */
typedef struct {
    /** SEN0240 front-end driver instance. */
    sen0240_t        sen;

    /** Latest raw ADC sample (for UI/debug). */
    uint16_t         raw;

    /** Latest sample converted to volts. */
    float            volts;

    /** Latest centered value (volts minus baseline). */
    float            centered;

    /** Latest normalized value in [-1, 1]. */
    float            norm;

    /** Non-zero when the normalized value is close to saturation. */
    uint8_t          saturated;

    /** Current processing state. */
    app_emg_state_t  state;

    /** Non-zero when a valid baseline (offset) is available. */
    uint8_t          has_baseline;

    /** Number of samples accumulated during calibration. */
    uint32_t         calib_samples;

    /** Target number of samples required to complete calibration. */
    uint32_t         calib_target;

    /** Accumulator used to compute mean voltage during calibration. */
    float            calib_accum;

    /** Gain used to map centered volts to normalized amplitude. */
    float            gain_volts;

    /** Optional RMS of the last processed block (RUN state only). */
    float            last_block_rms;

    /** Optional peak (abs) of the last processed block (RUN state only). */
    float            last_block_peak;
} app_emg_t;

/**
 * @brief Default number of calibration samples.
 *
 * This value defines the calibration window length. For example, at 2 kHz:
 * - 12000 samples ≈ 6 s
 * - 15000 samples ≈ 7.5 s
 */
#ifndef APP_EMG_CALIB_SAMPLES
#define APP_EMG_CALIB_SAMPLES 15000U
#endif

/**
 * @brief Initialize an EMG processing instance.
 *
 * This binds the ADC front-end, resets runtime outputs, sets the initial
 * baseline offset and configures the gain used for normalization.
 *
 * @param a                EMG instance to initialize.
 * @param adc              ADC handle used by the sensor front-end.
 * @param v_offset_initial Initial baseline voltage used until calibration completes.
 * @param gain_volts       Voltage that maps to a normalized amplitude of 1.0.
 */
void app_emg_init(app_emg_t*  a,
                  const omnia_adc_handle_t* adc,
                  float v_offset_initial,
                  float gain_volts);

/**
 * @brief Start baseline calibration.
 *
 * During calibration, the baseline offset is estimated as the mean voltage
 * over @ref APP_EMG_CALIB_SAMPLES samples (or @ref app_emg_t::calib_target if modified).
 *
 * @param a EMG instance.
 */
void app_emg_start_calibration(app_emg_t* a);

/**
 * @brief Process a single sample (legacy/compatibility API).
 *
 * This mode is kept for compatibility but is not recommended at high sample rates.
 * Prefer @ref app_emg_process_block_raw for DMA-driven acquisition.
 *
 * @param a EMG instance.
 */
void app_emg_step(app_emg_t* a);

/**
 * @brief Process a block of raw ADC samples.
 *
 * This is the primary processing entry point and is intended for DMA-captured blocks.
 * Behavior:
 * - If calibrating: updates baseline estimation and suppresses signal outputs.
 * - If baseline is available: computes centered and normalized values for each sample
 *   while updating the "latest sample" outputs.
 * - Updates optional block-level RMS/peak metrics when running.
 *
 * @param a   EMG instance.
 * @param raw Pointer to raw ADC samples.
 * @param n   Number of samples in the block.
 */
void app_emg_process_block_raw(app_emg_t* a,
                               const uint16_t* raw,
                               size_t n);

/**
 * @brief Check whether the EMG instance is currently calibrating.
 *
 * @param a EMG instance.
 * @return Non-zero if calibrating, zero otherwise.
 */
static inline int app_emg_is_calibrating(const app_emg_t* a)
{
    return a && (a->state == APP_EMG_STATE_CALIBRATING);
}

/**
 * @brief Check whether a valid baseline is available.
 *
 * @param a EMG instance.
 * @return Non-zero if baseline is available, zero otherwise.
 */
static inline int app_emg_has_baseline(const app_emg_t* a)
{
    return a && (a->has_baseline != 0);
}

#ifdef __cplusplus
}
#endif

#endif /* APP_EMG_H */
