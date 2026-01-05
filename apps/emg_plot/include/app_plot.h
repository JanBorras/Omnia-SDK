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

#ifndef APP_PLOT_H
#define APP_PLOT_H

#include <stdint.h>
#include "st7735.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file app_plot.h
 * @brief Lightweight plotting and UI interface for EMG visualization.
 *
 * This module provides a minimal user interface for small TFT displays
 * (ST7735 family). It is designed for low-overhead rendering and clear
 * state feedback in embedded systems.
 *
 * Features:
 * - Scrolling normalized EMG waveform.
 * - Numeric diagnostics (raw, volts, centered, normalized).
 * - Explicit visual states: idle, calibrating and run.
 *
 * Rendering is throttled in time to avoid excessive redraws.
 */

/**
 * @brief Plot/UI context.
 *
 * This structure holds all state required to render the EMG waveform
 * and associated status information on the display.
 */
typedef struct {
    /** Pointer to the ST7735 display instance. */
    ST7735_t *lcd;

    /** Current horizontal drawing position for the waveform. */
    uint16_t  x;

    /** UI refresh period in milliseconds (e.g., 100 ms). */
    uint32_t  info_period_ms;

    /** Timestamp of the last UI update (milliseconds). */
    uint64_t  last_info_ms;

    /** Flag indicating first frame rendering. */
    uint8_t   first_frame;

    /** Previous calibration state (used for state transitions). */
    int       last_is_calibrating;

    /** Previous baseline availability state. */
    int       last_has_baseline;

    /** Cached string for last raw value (to avoid unnecessary redraw). */
    char last_raw[16];

    /** Cached string for last voltage value. */
    char last_v[16];

    /** Cached string for last centered value. */
    char last_c[16];

    /** Cached string for last normalized value. */
    char last_n[16];
} app_plot_t;

/**
 * @brief Initialize the plotting subsystem and draw the initial frame.
 *
 * This function binds the display instance, resets internal state and
 * renders the static UI elements (background, banner and user hints).
 *
 * @param p   Plot instance to initialize.
 * @param lcd ST7735 display descriptor.
 */
void app_plot_init(app_plot_t *p, ST7735_t *lcd);

/**
 * @brief Draw a single normalized EMG sample on the waveform plot.
 *
 * This function renders one point of the scrolling waveform. It should
 * be called at a UI-friendly rate, not at the raw acquisition frequency.
 *
 * @param p          Plot instance.
 * @param norm       Normalized EMG value in the range [-1, 1].
 * @param saturated  Non-zero if the signal is saturated.
 */
void app_plot_draw(app_plot_t *p, float norm, int saturated);

/**
 * @brief Update numeric information and UI state.
 *
 * This function updates the header area with numeric values and manages
 * transitions between idle, calibrating and run states. Updates are
 * throttled according to @ref app_plot_t::info_period_ms.
 *
 * @param p               Plot instance.
 * @param raw             Latest raw ADC value.
 * @param volts           Converted voltage.
 * @param centered        Centered signal value.
 * @param norm            Normalized signal value.
 * @param saturated       Non-zero if the signal is saturated.
 * @param is_calibrating  Non-zero if calibration is in progress.
 * @param has_baseline    Non-zero if a valid baseline is available.
 */
void app_plot_draw_info(app_plot_t *p,
                        uint16_t raw,
                        float volts,
                        float centered,
                        float norm,
                        int saturated,
                        int is_calibrating,
                        int has_baseline);

#ifdef __cplusplus
}
#endif

#endif /* APP_PLOT_H */
