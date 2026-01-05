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
 * @file emg_stream.h
 * @brief EMG streaming pipeline: buffering, framing, packetization and BLE transport.
 *
 * This module sits between the ADC/EMG acquisition layer and the BLE transport.
 * It provides:
 *  - Sample buffering with bounded memory.
 *  - Deterministic framing of raw EMG samples.
 *  - Robust packet transmission over unreliable links (HM-19).
 *  - Optional configuration (CFG) packet management.
 *
 * No dynamic allocation is performed. All buffers are caller-owned.
 */

#ifndef EMG_STREAM_H
#define EMG_STREAM_H

#include <stdint.h>
#include <stddef.h>

#include "omnia_types.h"
#include "hm19.h"
#include "emg_packet.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Error codes returned by the EMG stream API.
 */
typedef enum {
    EMG_STREAM_OK = 0,

    EMG_STREAM_EINVAL = -1,  /**< Invalid argument */
    EMG_STREAM_ENOMEM = -2,  /**< Insufficient buffer capacity */
    EMG_STREAM_EIO    = -3,  /**< Transport or I/O error */
} emg_stream_err_t;

/**
 * @brief Runtime configuration for the EMG streaming engine.
 */
typedef struct {
    /* Sampling */
    uint32_t sample_rate_hz;      /**< Sampling frequency (Hz), e.g. 2000 */
    uint32_t sample_period_us;    /**< Sample period in microseconds (optional) */

    /* Framing */
    uint8_t  frame_samples;       /**< Samples per frame (8..32) */
    uint32_t tx_timeout_ms;       /**< BLE transmit timeout */

    /* Transport policy */
    uint8_t  require_connected;   /**< If 1, transmit only when BLE is connected */

    /* Flush budget (single-core friendly) */
    uint8_t  max_flush_packets_per_tick; /**< Max packets per emg_stream_tick() */
    uint32_t flush_budget_ms;            /**< Time budget per tick (0 = unlimited) */

    /* Ring buffer overflow policy */
    uint8_t  drop_oldest_on_full; /**< 0 = drop newest, 1 = drop oldest */

    /* CFG packet policy */
    uint8_t  send_cfg_on_connect; /**< Send CFG on connection event */
    uint32_t cfg_resend_ms;       /**< Periodic CFG resend (0 = disabled) */

} emg_stream_config_t;

/**
 * @brief EMG streaming context (opaque state container).
 *
 * This structure owns:
 *  - The sample ring buffer.
 *  - Frame assembly state.
 *  - Pending transmission state.
 *  - Protocol sequencing and statistics.
 *
 * It must be zero-initialized via emg_stream_init().
 */
typedef struct {
    /* Ring buffer (raw samples) */
    uint16_t* buf;
    uint32_t  cap;
    uint32_t  head;
    uint32_t  tail;
    uint32_t  count;

    /* Timestamp of the oldest sample in the ring */
    uint32_t  t0_ms;
    uint32_t  t0_valid;

    /* Frame builder */
    uint16_t  frame_tmp[EMG_PKT_MAX_SAMPLES];
    uint8_t   frame_len;

    /* Timestamp of first sample in current frame */
    uint32_t  frame_t0_ms;
    uint8_t   frame_t0_valid;

    /* Pending frame (robust flush) */
    uint16_t  pending_frame[EMG_PKT_MAX_SAMPLES];
    uint8_t   pending_len;
    uint32_t  pending_t_ms;
    uint8_t   pending_valid;

    /* Protocol sequence counter */
    uint8_t   seq;

    /* BLE device handle */
    hm19_t*   ble;

    /* Configuration */
    emg_stream_config_t cfg;

    /* CFG payload and state */
    emg_cfg_payload_t cfg_payload;
    uint8_t   cfg_valid;
    uint8_t   cfg_sent;
    uint32_t  last_cfg_try_ms;

    /* Connection tracking */
    int       last_connected; /**< -1 unknown, 0 disconnected, 1 connected */

    /* Statistics */
    uint32_t pushed_samples;
    uint32_t dropped_samples;
    uint32_t sent_packets;
    uint32_t tx_errors;
    uint32_t enqueued_frames;
    uint32_t dequeued_frames;

    uint32_t sent_cfg;
    uint32_t cfg_errors;

} emg_stream_t;

/**
 * @brief Initialize an EMG stream context.
 *
 * @param s               Stream context to initialize.
 * @param ble             Initialized HM-19 BLE driver.
 * @param cfg             Stream configuration.
 * @param ring_buf_samples Caller-provided ring buffer (raw samples).
 * @param cap_samples     Capacity of the ring buffer (in samples).
 *
 * @return EMG_STREAM_OK on success, error code otherwise.
 */
emg_stream_err_t emg_stream_init(emg_stream_t* s,
                                 hm19_t* ble,
                                 const emg_stream_config_t* cfg,
                                 uint16_t* ring_buf_samples,
                                 uint32_t cap_samples);

/**
 * @brief Set or update the configuration (CFG) payload to be sent.
 *
 * Typically called after calibration and before streaming.
 */
void emg_stream_set_cfg(emg_stream_t* s, const emg_cfg_payload_t* cfg);

/**
 * @brief Update only the baseline field in the CFG payload.
 *
 * @param force_resend If non-zero, forces the CFG to be resent.
 */
void emg_stream_update_baseline_raw(emg_stream_t* s,
                                    uint16_t baseline_raw,
                                    uint8_t force_resend);

/**
 * @brief Push a single raw EMG sample into the stream.
 *
 * @param t_ms Timestamp in milliseconds of the sample.
 */
void emg_stream_push_sample(emg_stream_t* s,
                            uint16_t sample_raw,
                            uint32_t t_ms);

/**
 * @brief Push a block of raw EMG samples.
 *
 * Typically used from an ADC DMA callback.
 *
 * @param t0_ms Timestamp of the first sample in the block.
 */
void emg_stream_push_block(emg_stream_t* s,
                           const uint16_t* samples_raw,
                           uint32_t n,
                           uint32_t t0_ms);

/**
 * @brief Periodic service function.
 *
 * Attempts to:
 *  - Send CFG packets if required.
 *  - Flush queued frames according to policy and budget.
 *
 * Should be called regularly from the main loop or a low-priority task.
 */
void emg_stream_tick(emg_stream_t* s);

/**
 * @brief Get the number of samples currently queued.
 */
static inline uint32_t emg_stream_queued_samples(const emg_stream_t* s)
{
    return s ? s->count : 0;
}

#ifdef __cplusplus
}
#endif

#endif /* EMG_STREAM_H */
