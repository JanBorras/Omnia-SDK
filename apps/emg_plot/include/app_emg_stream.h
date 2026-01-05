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

#ifndef APP_EMG_STREAM_H
#define APP_EMG_STREAM_H

#include <stdint.h>
#include <stddef.h>

#include "app_emg.h"
#include "app_plot.h"
#include "emg_packet.h"
#include "omnia_uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file app_emg_stream.h
 * @brief EMG streaming interface over UART.
 *
 * This module defines the public interface for streaming EMG data and
 * configuration metadata to an external host (e.g., PC).
 *
 * Responsibilities:
 * - Forward raw ADC samples using a packet-based protocol.
 * - Transmit a configuration descriptor describing ADC and signal context.
 * - Coordinate EMG processing, optional UI updates and UART transport.
 *
 * The API is designed to be non-blocking and tolerant to transient
 * communication congestion.
 */

/**
 * @brief EMG stream context.
 *
 * This structure aggregates all state required to stream EMG data,
 * including protocol sequencing, configuration caching, transport
 * handles and optional UI throttling.
 */
typedef struct {
    /** EMG processing instance (mandatory). */
    app_emg_t*           emg;

    /** Optional plot/UI instance. May be NULL if no UI is required. */
    app_plot_t*          plot;

    /** UART handle used for packet transmission. */
    omnia_uart_handle_t  uart;

    /** EMG sampling rate in Hz (informational, sent in configuration). */
    uint16_t             sample_rate_hz;

    /** ADC channel identifier (informational). */
    uint8_t              adc_channel;

    /** Protocol sequence number. */
    uint8_t              seq;

    /** Configuration pending flag (retry until successfully sent). */
    uint8_t              cfg_pending;

    /** Cached configuration payload. */
    emg_cfg_payload_t    cfg_cache;

    /** Temporary transmit buffer used for packet construction. */
    uint8_t              tx_buf[256];

    /** UI refresh period in milliseconds (additional throttling). */
    uint32_t             plot_period_ms;

    /** Timestamp of the last UI update. */
    uint32_t             last_plot_ms;

    /** Number of raw packets dropped due to UART congestion. */
    uint32_t             dropped_raw;

    /** Number of raw packets successfully transmitted. */
    uint32_t             sent_raw;

    /** Number of configuration packets successfully transmitted. */
    uint32_t             sent_cfg;

} app_emg_stream_t;

/**
 * @brief Initialize an EMG stream instance.
 *
 * This function binds all required dependencies, resets internal counters
 * and schedules an initial configuration packet to be sent to the host.
 *
 * @param s              Stream instance to initialize.
 * @param emg            EMG processing instance.
 * @param plot           Optional plot/UI instance (may be NULL).
 * @param uart           UART handle used for streaming.
 * @param sample_rate_hz Sampling rate in Hz.
 * @param adc_channel    ADC channel identifier.
 */
void app_emg_stream_init(app_emg_stream_t* s,
                         app_emg_t* emg,
                         app_plot_t* plot,
                         const omnia_uart_handle_t* uart,
                         uint16_t sample_rate_hz,
                         uint8_t adc_channel);

/**
 * @brief Start EMG calibration through the stream interface.
 *
 * Typically called in response to a user action (e.g., CAL button press).
 *
 * @param s EMG stream instance.
 */
void app_emg_stream_start_calibration(app_emg_stream_t* s);

/**
 * @brief Process and stream a block of raw ADC samples.
 *
 * This function should be called for each acquired ADC block (e.g., DMA
 * transfer). It processes EMG data, streams raw samples and updates the UI
 * if enabled.
 *
 * @param s       EMG stream instance.
 * @param raw     Pointer to raw ADC samples.
 * @param n       Number of samples in the block.
 * @param time_ms Current system time in milliseconds.
 */
void app_emg_stream_process_block(app_emg_stream_t* s,
                                  const uint16_t* raw,
                                  size_t n,
                                  uint32_t time_ms);

/**
 * @brief Force transmission of the configuration packet.
 *
 * Useful at boot or when the host requests a context refresh.
 * The transmission is non-blocking and may be retried if the UART is busy.
 *
 * @param s       EMG stream instance.
 * @param time_ms Current system time in milliseconds.
 */
void app_emg_stream_request_cfg(app_emg_stream_t* s, uint32_t time_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_EMG_STREAM_H */
