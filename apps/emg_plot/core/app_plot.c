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
 * @file app_plot.c
 * @brief Lightweight UI rendering for EMG visualization on ST7735 displays.
 *
 * This module provides a minimal plotting and status interface intended for
 * resource-constrained embedded targets. It renders:
 * - A scrolling normalized EMG waveform.
 * - Numeric diagnostics (raw, volts, centered, normalized).
 * - Explicit visual states for idle, calibration and run modes.
 *
 * Design goals:
 * - Deterministic execution time.
 * - Minimal redraw and bandwidth usage.
 * - Clear separation between signal rendering and UI state handling.
 */

#include "app_plot.h"
#include "omnia_port.h"   /* millis() */
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------
 * Internal visual helpers
 * ------------------------------------------------------------------- */

/**
 * @brief Paint a vertical background gradient.
 *
 * The gradient provides visual depth while preserving sufficient contrast
 * for waveform rendering.
 *
 * @param p Plot instance.
 */
static void app_plot_paint_gradient(app_plot_t *p)
{
    ST7735_t *lcd = p->lcd;

    for (uint16_t y = 0; y < lcd->height; ++y) {
        uint8_t g = (uint8_t)((y * 40U) / lcd->height); /* 0..40 */
        uint16_t col = rgb565(g, g, g);
        st7735_draw_line(lcd, 0, y, lcd->width - 1U, y, col);
    }
}

/**
 * @brief Initialize and draw the static UI frame.
 *
 * This includes the background gradient, title banner, separators and
 * initial user hints. It is typically called at boot or after a full UI reset.
 *
 * @param p Plot instance.
 */
static void app_plot_init_frame(app_plot_t *p)
{
    ST7735_t *lcd = p->lcd;

    app_plot_paint_gradient(p);

    uint16_t banner_h = 28;
    st7735_filled_rectangle(lcd, 0, 0, lcd->width, banner_h,
                            rgb565(0, 40, 90));

    const char *title = "OMNIA SDK";
    uint16_t title_x =
        (lcd->width - (uint16_t)strlen(title) * 6U) / 2U;
    st7735_draw_string(lcd, title_x, 6, title, WHITE, 1);

    const char *sub = "EMG ANALYTICS";
    uint16_t sub_x =
        (lcd->width - (uint16_t)strlen(sub) * 6U) / 2U;
    st7735_draw_string(lcd, sub_x, 16, sub, CYAN, 1);

    st7735_draw_line(lcd, 10, banner_h + 2,
                     lcd->width - 10, banner_h + 2,
                     rgb565(180, 180, 255));

    st7735_draw_circle(lcd,
                       lcd->width / 2U,
                       lcd->height - 10U,
                       30,
                       rgb565(50, 80, 150));

    const char *hint = "Prem CAL";
    uint16_t hint_x =
        (lcd->width - (uint16_t)strlen(hint) * 6U) / 2U;
    st7735_draw_string(lcd,
                       hint_x,
                       lcd->height - 18U,
                       hint,
                       YELLOW,
                       1);
}

/**
 * @brief Get current system time in milliseconds.
 *
 * The value is retrieved through the active platform port abstraction.
 *
 * @return Current time in milliseconds, or 0 if unavailable.
 */
static inline uint64_t now_ms(void)
{
    omnia_port_t *port = omnia_port_get();
    if (port && port->v && port->v->millis) {
        return port->v->millis();
    }
    return 0;
}

/* -------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------- */

/**
 * @brief Initialize the plotting subsystem.
 *
 * This function binds the display instance, resets internal state and
 * renders the initial static frame.
 *
 * @param p   Plot instance.
 * @param lcd ST7735 display descriptor.
 */
void app_plot_init(app_plot_t *p, ST7735_t *lcd)
{
    if (!p || !lcd) return;

    p->lcd = lcd;
    p->x   = 0;

    /* Numeric info refresh rate (human-readable). */
    p->info_period_ms = 100; /* 10 Hz */
    p->last_info_ms   = 0;

    p->last_raw[0] = '\0';
    p->last_v[0]   = '\0';
    p->last_c[0]   = '\0';
    p->last_n[0]   = '\0';

    p->last_is_calibrating = 0;
    p->last_has_baseline   = 0;
    p->first_frame         = 1;

    /* Draw initial static frame immediately. */
    app_plot_init_frame(p);
    p->first_frame = 0;
}

/**
 * @brief Draw a single normalized EMG sample.
 *
 * The waveform scrolls horizontally across the screen and wraps when
 * reaching the display width. Only the current column is cleared and
 * redrawn to minimize rendering cost.
 *
 * @param p          Plot instance.
 * @param norm       Normalized signal value in [-1, 1].
 * @param saturated  Non-zero if the signal is saturated.
 */
void app_plot_draw(app_plot_t *p, float norm, int saturated)
{
    if (!p || !p->lcd) return;

    if (norm >  1.f) norm =  1.f;
    if (norm < -1.f) norm = -1.f;

    uint16_t plot_top    = 34;
    uint16_t plot_bottom = p->lcd->height - 1U;
    uint16_t plot_h      = plot_bottom - plot_top;
    uint16_t mid         = plot_top + plot_h / 2U;

    uint16_t y =
        mid - (uint16_t)(norm * (float)(plot_h / 2U - 2U));
    uint16_t color = saturated ? RED : CYAN;

    /* Clear current column using background gradient. */
    for (uint16_t yy = plot_top; yy <= plot_bottom; ++yy) {
        uint8_t g = (uint8_t)((yy * 40U) / p->lcd->height);
        uint16_t col_bg = rgb565(g, g, g);
        st7735_draw_pixel(p->lcd, p->x, yy, col_bg);
    }

    if (y >= plot_top && y <= plot_bottom) {
        st7735_draw_pixel(p->lcd, p->x, y, color);
    }

    p->x++;
    if (p->x >= p->lcd->width) {
        p->x = 0;
    }
}

/**
 * @brief Draw numeric information and manage UI state transitions.
 *
 * This function is throttled in time and handles three explicit states:
 * - IDLE: no baseline and not calibrating.
 * - CALIBRATING: baseline acquisition in progress.
 * - RUN: baseline available, normal operation.
 *
 * The UI is selectively refreshed to avoid unnecessary redraws.
 *
 * @param p               Plot instance.
 * @param raw             Latest raw ADC value.
 * @param volts           Converted voltage.
 * @param centered        Centered signal (volts minus baseline).
 * @param norm            Normalized signal.
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
                        int has_baseline)
{
    if (!p || !p->lcd) return;

    /* Time-based throttling. */
    uint64_t t = now_ms();
    if (p->last_info_ms != 0 &&
        (t - p->last_info_ms) < p->info_period_ms) {
        return;
    }
    p->last_info_ms = t;

    /* ---------------------------------------------------------
     * IDLE state: no baseline and not calibrating
     * --------------------------------------------------------- */
    if (!has_baseline && !is_calibrating) {
        if (p->last_has_baseline || p->last_is_calibrating) {
            app_plot_init_frame(p);
            p->x = 0;

            p->last_raw[0] = '\0';
            p->last_v[0]   = '\0';
            p->last_c[0]   = '\0';
            p->last_n[0]   = '\0';
        }

        p->last_is_calibrating = 0;
        p->last_has_baseline   = 0;
        return;
    }

    /* ---------------------------------------------------------
     * CALIBRATING state
     * --------------------------------------------------------- */
    if (is_calibrating) {
        if (!p->last_is_calibrating) {
            app_plot_paint_gradient(p);
            p->x = 0;

            st7735_filled_rectangle(p->lcd, 0, 0,
                                     p->lcd->width, 18, BLACK);
            st7735_draw_string(p->lcd, 2, 4,
                               "Calibrating...", CYAN, 1);

            st7735_filled_rectangle(p->lcd,
                                     p->lcd->width - 30, 0,
                                     30, 18, MAGENTA);
            st7735_draw_string(p->lcd,
                               p->lcd->width - 28, 4,
                               "CAL", WHITE, 1);

            p->last_raw[0] = '\0';
            p->last_v[0]   = '\0';
            p->last_c[0]   = '\0';
            p->last_n[0]   = '\0';
        }

        p->last_is_calibrating = 1;
        p->last_has_baseline   = has_baseline;
        return;
    }

    /* ---------------------------------------------------------
     * RUN state
     * --------------------------------------------------------- */
    if (!p->last_has_baseline || p->last_is_calibrating) {
        st7735_filled_rectangle(p->lcd, 0, 0,
                                 p->lcd->width, 18, BLACK);

        p->last_raw[0] = '\0';
        p->last_v[0]   = '\0';
        p->last_c[0]   = '\0';
        p->last_n[0]   = '\0';
    }

    char buf_raw[16];
    char buf_v[16];
    char buf_c[16];
    char buf_n[16];

    snprintf(buf_raw, sizeof(buf_raw), "RAW:%4u", raw);
    snprintf(buf_v,   sizeof(buf_v),   "V:%.2f",  volts);
    snprintf(buf_c,   sizeof(buf_c),   "C:%.2f",  centered);
    snprintf(buf_n,   sizeof(buf_n),   "N:%.2f",  norm);

    if (strcmp(buf_raw, p->last_raw) != 0) {
        st7735_filled_rectangle(p->lcd, 0, 0, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 2, 2, buf_raw, WHITE, 1);
        strcpy(p->last_raw, buf_raw);
    }

    if (strcmp(buf_v, p->last_v) != 0) {
        st7735_filled_rectangle(p->lcd, 64, 0, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 70, 2, buf_v, GREEN, 1);
        strcpy(p->last_v, buf_v);
    }

    if (strcmp(buf_c, p->last_c) != 0) {
        st7735_filled_rectangle(p->lcd, 0, 10, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 2, 12, buf_c, YELLOW, 1);
        strcpy(p->last_c, buf_c);
    }

    if (strcmp(buf_n, p->last_n) != 0) {
        st7735_filled_rectangle(p->lcd, 64, 10, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 70, 12,
                           buf_n, saturated ? RED : CYAN, 1);
        strcpy(p->last_n, buf_n);
    }

    p->last_is_calibrating = 0;
    p->last_has_baseline   = 1;
}
