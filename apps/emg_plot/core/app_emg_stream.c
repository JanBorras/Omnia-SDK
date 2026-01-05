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
 * @file app_emg_stream.c
 * @brief EMG streaming logic over UART, including configuration and raw data packets.
 *
 * This module bridges the EMG processing pipeline with an external host (e.g., PC)
 * by streaming raw samples and periodically sending a configuration descriptor
 * that allows the receiver to correctly interpret the data.
 *
 * Design goals:
 * - Non-blocking operation.
 * - Explicit separation between configuration and data streaming.
 * - Robust behavior under transient UART congestion.
 */

#include "app_emg_stream.h"
#include "omnia_port.h"
#include <string.h>

/**
 * @brief Convert a voltage value to an approximate ADC raw code.
 *
 * This helper is primarily used to derive the baseline raw value
 * from a voltage offset when building the stream configuration.
 *
 * @param adc   ADC descriptor containing resolution and reference voltage.
 * @param volts Input voltage.
 * @return Approximated ADC raw code.
 */
static uint16_t volts_to_raw(const omnia_adc_handle_t* adc, float volts)
{
    if (!adc || adc->resolution_bits == 0) return 0;

    const uint32_t max_code = (1u << adc->resolution_bits) - 1u;
    const float vref = (float)adc->vref_mv / 1000.0f;
    if (vref <= 0.0f) return 0;

    float x = volts / vref;
    if (x < 0.0f) x = 0.0f;
    if (x > 1.0f) x = 1.0f;

    uint32_t raw = (uint32_t)(x * (float)max_code + 0.5f);
    if (raw > max_code) raw = max_code;

    return (uint16_t)raw;
}

/**
 * @brief Build a stream configuration snapshot from the current EMG state.
 *
 * The configuration acts as a lightweight context descriptor that allows
 * the host to interpret raw ADC samples (resolution, reference, channel,
 * sampling rate, and baseline).
 *
 * @param s EMG stream instance.
 */
static void build_cfg_from_state(app_emg_stream_t* s)
{
    /* Clear cached configuration structure. */
    memset(&s->cfg_cache, 0, sizeof(s->cfg_cache));

    s->cfg_cache.resolution_bits = s->emg->sen.adc.resolution_bits;
    s->cfg_cache.vref_mv         = (uint16_t)s->emg->sen.adc.vref_mv;
    s->cfg_cache.adc_channel     = s->adc_channel;
    s->cfg_cache.sample_rate_hz  = s->sample_rate_hz;

    /* Baseline expressed in raw ADC units, derived from voltage offset. */
    s->cfg_cache.baseline_raw =
        volts_to_raw(&s->emg->sen.adc, s->emg->sen.v_offset);

    /* Optional gain field (Q15). Unused by default. */
    s->cfg_cache.gain_q15 = 0;
}

/**
 * @brief Attempt to send the pending configuration packet over UART.
 *
 * This function is non-blocking. If the UART is busy, the configuration
 * remains pending and will be retried on subsequent calls.
 *
 * @param s       EMG stream instance.
 * @param time_ms Current system time in milliseconds.
 */
static void try_send_cfg(app_emg_stream_t* s, uint32_t time_ms)
{
    if (!s->cfg_pending) return;

    size_t n = emg_build_cfg_packet(s->seq++, time_ms,
                                   &s->cfg_cache,
                                   s->tx_buf, sizeof(s->tx_buf));
    if (n == 0) return;

    omnia_status_t st = omnia_uart_write(&s->uart, s->tx_buf, n);
    if (st == OMNIA_OK) {
        s->cfg_pending = 0;
        s->sent_cfg++;
    }
    /* OMNIA_EBUSY is tolerated: the packet will be retried later. */
}

/**
 * @brief Stream raw ADC samples to the host.
 *
 * Samples are split into protocol-sized chunks. The function never blocks:
 * if the UART is busy, packets are dropped to preserve real-time behavior.
 *
 * @param s       EMG stream instance.
 * @param raw     Pointer to raw ADC samples.
 * @param n       Number of samples.
 * @param time_ms Current system time in milliseconds.
 */
static void send_raw_stream(app_emg_stream_t* s,
                            const uint16_t* raw,
                            size_t n,
                            uint32_t time_ms)
{
    while (n > 0) {
        uint8_t ns = (n > EMG_PKT_MAX_SAMPLES)
                     ? (uint8_t)EMG_PKT_MAX_SAMPLES
                     : (uint8_t)n;

        size_t pkt_n = emg_build_raw_u16_packet(s->seq++, time_ms,
                                               raw, ns,
                                               s->tx_buf, sizeof(s->tx_buf));
        if (pkt_n == 0) return;

        omnia_status_t st = omnia_uart_write(&s->uart, s->tx_buf, pkt_n);
        if (st == OMNIA_OK) {
            s->sent_raw++;
        } else if (st == OMNIA_EBUSY) {
            /* Drop packet to avoid blocking the stream. */
            s->dropped_raw++;
        } else {
            /* OMNIA_EIO or other errors may be handled or counted here. */
        }

        raw += ns;
        n   -= ns;
    }
}

/**
 * @brief Initialize an EMG stream instance.
 *
 * This function sets up internal state, binds dependencies, and schedules
 * an initial configuration packet to be sent to the host.
 *
 * @param s              Stream instance to initialize.
 * @param emg            EMG processing instance.
 * @param plot           Optional UI/plot instance (may be NULL).
 * @param uart           UART handle used for streaming.
 * @param sample_rate_hz Sampling rate in Hz.
 * @param adc_channel    ADC channel identifier.
 */
void app_emg_stream_init(app_emg_stream_t* s,
                         app_emg_t* emg,
                         app_plot_t* plot,
                         const omnia_uart_handle_t* uart,
                         uint16_t sample_rate_hz,
                         uint8_t adc_channel)
{
    if (!s || !emg || !uart) return;

    memset(s, 0, sizeof(*s));
    s->emg            = emg;
    s->plot           = plot;
    s->uart           = *uart;
    s->sample_rate_hz = sample_rate_hz;
    s->adc_channel    = adc_channel;
    s->seq            = 0;

    /* UI update rate (human-friendly). */
    s->plot_period_ms = 33;
    s->last_plot_ms   = 0;

    /* Send configuration at boot (deferred until UART is available). */
    build_cfg_from_state(s);
    s->cfg_pending = 1;
}

/**
 * @brief Request an explicit configuration update to be sent to the host.
 *
 * @param s       EMG stream instance.
 * @param time_ms Current system time in milliseconds.
 */
void app_emg_stream_request_cfg(app_emg_stream_t* s, uint32_t time_ms)
{
    if (!s) return;

    build_cfg_from_state(s);
    s->cfg_pending = 1;
    try_send_cfg(s, time_ms);
}

/**
 * @brief Start EMG calibration and update UI and configuration state.
 *
 * @param s EMG stream instance.
 */
void app_emg_stream_start_calibration(app_emg_stream_t* s)
{
    if (!s || !s->emg) return;

    app_emg_start_calibration(s->emg);

    /* Immediate UI feedback, if enabled. */
    if (s->plot) {
        app_plot_draw_info(s->plot, 0, 0.0f, 0.0f, 0.0f, 0, 1, 0);
    }

    /* Configuration may still be useful during calibration. */
    build_cfg_from_state(s);
    s->cfg_pending = 1;
}

/**
 * @brief Process a block of raw EMG samples.
 *
 * The function performs EMG processing, conditionally updates and sends
 * configuration data, streams raw samples, and refreshes the UI at a
 * throttled rate.
 *
 * @param s       EMG stream instance.
 * @param raw     Pointer to raw ADC samples.
 * @param n       Number of samples.
 * @param time_ms Current system time in milliseconds.
 */
void app_emg_stream_process_block(app_emg_stream_t* s,
                                  const uint16_t* raw,
                                  size_t n,
                                  uint32_t time_ms)
{
    if (!s || !s->emg || !raw || n == 0) return;

    /* Track baseline transition to detect calibration completion. */
    uint8_t had_baseline_before =
        (uint8_t)app_emg_has_baseline(s->emg);

    app_emg_process_block_raw(s->emg, raw, n);

    uint8_t has_baseline_now =
        (uint8_t)app_emg_has_baseline(s->emg);
    uint8_t is_cal =
        (uint8_t)app_emg_is_calibrating(s->emg);

    /* Send updated configuration when baseline becomes available. */
    if (!had_baseline_before && has_baseline_now) {
        build_cfg_from_state(s);
        s->cfg_pending = 1;
    }

    /* Attempt non-blocking configuration transmission. */
    try_send_cfg(s, time_ms);

    /* Always stream raw samples (even during calibration, if desired). */
    send_raw_stream(s, raw, n, time_ms);

    /* UI update (throttled). */
    if (s->plot &&
        (time_ms - s->last_plot_ms) >= s->plot_period_ms) {

        s->last_plot_ms = time_ms;

        if (has_baseline_now && !is_cal) {
            app_plot_draw(s->plot,
                          s->emg->norm,
                          s->emg->saturated);
        }

        /* draw_info() performs its own internal throttling. */
        app_plot_draw_info(s->plot,
                           s->emg->raw,
                           s->emg->volts,
                           s->emg->centered,
                           s->emg->norm,
                           s->emg->saturated,
                           is_cal ? 1 : 0,
                           has_baseline_now ? 1 : 0);
    }
}
