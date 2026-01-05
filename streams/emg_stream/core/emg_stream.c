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
 * @file emg_stream.c
 * @brief EMG raw streaming over BLE (HM-19) with buffering, framing, and optional CFG metadata.
 *
 * This module turns a high-rate ADC sample stream (e.g., 2 kHz EMG raw ADC counts)
 * into fixed-size frames and sends them over a BLE UART bridge (HM-19).
 *
 * Key properties:
 * - No dynamic allocations (caller provides ring buffer storage).
 * - Non-blocking send semantics: HM19/UART is async and may refuse when busy.
 * - Loss policy is explicit: drop-oldest or drop-newest when ring is full.
 * - Timestamps are consistent: a per-sample time advance is maintained even under overflow.
 *
 * Typical usage pattern:
 * - Call emg_stream_init()
 * - Optionally call emg_stream_set_cfg() / emg_stream_update_baseline_raw()
 * - Push samples (or blocks) using emg_stream_push_sample() / emg_stream_push_block()
 * - Periodically call emg_stream_tick() from the main loop to flush queued frames
 *
 * Concurrency notes:
 * - emg_stream_push_* may be called from an ISR (ADC DMA callbacks) if it does not block.
 * - emg_stream_tick() should run from the main context (or a task), not from ISR.
 *
 * IMPORTANT:
 * This file assumes hm19_send_raw() is non-blocking at the UART layer. If you later
 * switch to a blocking UART, you must re-evaluate ISR usage and budgets.
 */

#include "emg_stream.h"
#include "omnia_port.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Time helpers                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read current monotonic time in milliseconds from the active Omnia port.
 *
 * @return Current time in ms, or 0 if the port is not available.
 *
 * Notes:
 * - 0 is a valid value for "unknown"; the code generally tolerates this.
 * - The port provides coarse time via millis(); fine time is derived from sample_period_us.
 */
static uint64_t now_ms(void)
{
  omnia_port_t* p = omnia_port_get();
  if (!p || !p->v || !p->v->millis) return 0;
  return p->v->millis();
}

/**
 * @brief Advance a base timestamp by N samples using the configured sample period.
 *
 * @param s         Stream instance (used to read cfg.sample_period_us).
 * @param base_ms   Base timestamp (ms).
 * @param n_samples How many samples to advance.
 *
 * @return base_ms advanced by floor(n_samples * sample_period_us / 1000).
 *
 * Rationale:
 * - We keep timestamps consistent even when the transport is busy and buffering occurs.
 * - The stream uses integer milliseconds; fractional parts are truncated on purpose.
 */
static uint32_t advance_time_ms(const emg_stream_t* s, uint32_t base_ms, uint32_t n_samples)
{
  if (!s || s->cfg.sample_period_us == 0) return base_ms;
  uint64_t us = (uint64_t)n_samples * (uint64_t)s->cfg.sample_period_us;
  return base_ms + (uint32_t)(us / 1000u);
}

/* -------------------------------------------------------------------------- */
/* Ring buffer helpers                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Circular increment helper.
 * @param i   Current index.
 * @param cap Ring capacity.
 * @return Next index in [0..cap-1].
 */
static inline uint32_t ring_next(uint32_t i, uint32_t cap) { return (i + 1u) % cap; }

/**
 * @brief Push one raw sample into the ring buffer.
 *
 * @param s Stream instance.
 * @param v Sample to push (raw ADC count).
 *
 * @return 1 if the sample was stored, 0 if dropped.
 *
 * Overflow policy:
 * - If drop_oldest_on_full == 0: drop newest sample (do not modify buffer).
 * - If drop_oldest_on_full == 1: advance tail (drop oldest) then store newest.
 *
 * Timestamp invariant:
 * - When dropping the oldest sample, we must also advance t0_ms by one sample to keep
 *   "front timestamp" aligned to the actual first sample present in the ring.
 */
static int ring_push(emg_stream_t* s, uint16_t v)
{
  if (!s || !s->buf || s->cap == 0) return 0;

  if (s->count >= s->cap) {
    if (!s->cfg.drop_oldest_on_full) {
      s->dropped_samples++;
      return 0; // drop newest
    }

    /* Drop oldest */
    s->tail = ring_next(s->tail, s->cap);
    s->count--;
    s->dropped_samples++;

    /* Keep time aligned with ring front */
    if (s->t0_valid) {
      s->t0_ms = advance_time_ms(s, s->t0_ms, 1u);
    }
  }

  s->buf[s->head] = v;
  s->head = ring_next(s->head, s->cap);
  s->count++;
  return 1;
}

/**
 * @brief Pop one raw sample from the ring buffer.
 *
 * @param s   Stream instance.
 * @param out Output sample.
 *
 * @return 1 if a sample was popped, 0 if the ring is empty.
 *
 * Notes:
 * - This function only manipulates indices/counters; it does not update t0_ms.
 *   The caller that pops N samples is responsible for advancing the timestamp.
 */
static int ring_pop(emg_stream_t* s, uint16_t* out)
{
  if (!s || !out || s->count == 0) return 0;
  *out = s->buf[s->tail];
  s->tail = ring_next(s->tail, s->cap);
  s->count--;
  return 1;
}

/* -------------------------------------------------------------------------- */
/* TX policy (connection gating)                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Query connection state via HM-19 "STATE" pin (if enabled in hm19 flags).
 *
 * @param s Stream instance.
 * @return 1 connected, 0 not connected, -1 unknown/not supported.
 *
 * Notes:
 * - hm19_is_connected() may return -1 if STATE checking is disabled or pin missing.
 * - This module can operate in both "require connected" and "best effort" modes.
 */
static int connected_now(emg_stream_t* s)
{
  if (!s || !s->ble) return -1;
  return hm19_is_connected(s->ble); // 1/0/-1
}

/**
 * @brief Decide whether it's valid to attempt sending right now.
 *
 * @param s Stream instance.
 * @return 1 if sending is allowed, 0 otherwise.
 *
 * Behavior:
 * - If require_connected == 0: always allowed to attempt sending.
 * - If require_connected == 1: only allowed when STATE reports connected.
 */
static int should_send_now(emg_stream_t* s)
{
  if (!s || !s->ble) return 0;

  if (!s->cfg.require_connected) return 1;

  int c = connected_now(s);
  return (c == 1) ? 1 : 0;
}

/* -------------------------------------------------------------------------- */
/* Packet senders                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Try to send a CFG packet (metadata) over BLE.
 *
 * @param s    Stream instance.
 * @param t_ms Timestamp to embed in the packet.
 *
 * @return 1 on success, 0 on failure.
 *
 * Notes:
 * - CFG packet is optional and can be configured to be sent on connect.
 * - Failure is not fatal; it increments cfg_errors and the stream continues.
 */
static int try_send_cfg(emg_stream_t* s, uint32_t t_ms)
{
  if (!s || !s->ble) return 0;
  if (!s->cfg_valid) return 0;

  uint8_t tx_buf[EMG_PKT_COMMON_HDR_LEN + EMG_PKT_CFG_PAYLOAD_LEN + EMG_PKT_CRC_LEN];

  size_t pkt_len = emg_build_cfg_packet(s->seq, t_ms, &s->cfg_payload, tx_buf, sizeof(tx_buf));
  if (pkt_len == 0) {
    s->cfg_errors++;
    return 0;
  }

  omnia_status_t st = hm19_send_raw(s->ble, tx_buf, pkt_len, s->cfg.tx_timeout_ms);
  if (st == OMNIA_OK) {
    s->seq++;
    s->cfg_sent = 1;
    s->sent_cfg++;
    return 1;
  }

  s->cfg_errors++;
  return 0;
}

/**
 * @brief Try to send one RAW frame packet over BLE.
 *
 * @param s    Stream instance.
 * @param frame Pointer to raw samples.
 * @param n    Number of samples in the frame (<= EMG_PKT_MAX_SAMPLES).
 * @param t_ms Timestamp of the first sample in the frame.
 *
 * @return 1 on success, 0 on failure.
 *
 * Failure reasons:
 * - Packet build failure (tx_errors++).
 * - Transport busy or error (hm19_send_raw != OMNIA_OK, tx_errors++).
 */
static int try_send_raw_frame(emg_stream_t* s, const uint16_t* frame, uint8_t n, uint32_t t_ms)
{
  if (!s || !s->ble || !frame || n == 0) return 0;

  uint8_t tx_buf[EMG_PKT_COMMON_HDR_LEN + EMG_PKT_RAW_HDR_LEN + (EMG_PKT_MAX_SAMPLES * 2u) + EMG_PKT_CRC_LEN];

  size_t pkt_len = emg_build_raw_u16_packet(s->seq, t_ms, frame, n, tx_buf, sizeof(tx_buf));
  if (pkt_len == 0) {
    s->tx_errors++;
    return 0;
  }

  omnia_status_t st = hm19_send_raw(s->ble, tx_buf, pkt_len, s->cfg.tx_timeout_ms);
  if (st == OMNIA_OK) {
    s->seq++;
    s->sent_packets++;
    return 1;
  }

  s->tx_errors++;
  return 0;
}

/* -------------------------------------------------------------------------- */
/* Queue frame ops (ring <-> frames)                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enqueue a complete frame into the ring buffer.
 *
 * @param s          Stream instance.
 * @param frame      Frame samples to enqueue.
 * @param n          Number of samples in frame.
 * @param t_ms_first Timestamp of the first sample in the frame.
 *
 * Timestamp behavior:
 * - If the ring was empty (count==0), we set t0_ms to t_ms_first.
 * - If the ring was non-empty, we keep its existing t0_ms (front timestamp).
 */
static void enqueue_frame(emg_stream_t* s, const uint16_t* frame, uint8_t n, uint32_t t_ms_first)
{
  if (!s || !frame || n == 0) return;

  if (s->count == 0) {
    s->t0_ms = t_ms_first;
    s->t0_valid = 1;
  } else if (!s->t0_valid) {
    /* Defensive: ring has data but timestamp missing -> re-anchor */
    s->t0_ms = t_ms_first;
    s->t0_valid = 1;
  }

  for (uint8_t i = 0; i < n; ++i) {
    (void)ring_push(s, frame[i]);
  }

  s->enqueued_frames++;
}

/**
 * @brief Dequeue up to one frame from the ring buffer.
 *
 * @param s             Stream instance.
 * @param out_frame     Output buffer for samples (size >= EMG_PKT_MAX_SAMPLES).
 * @param out_t_ms_first Optional output: timestamp of the first sample dequeued.
 *
 * @return Number of samples dequeued (0 if ring empty).
 *
 * Timestamp behavior:
 * - The first sample time is the current t0_ms (front-of-ring time).
 * - After popping N samples:
 *   - If ring becomes empty: t0_valid cleared.
 *   - Else: t0_ms advances by N samples.
 */
static uint8_t dequeue_frame(emg_stream_t* s, uint16_t* out_frame, uint32_t* out_t_ms_first)
{
  if (!s || !out_frame || s->count == 0) return 0;

  uint32_t t_first = s->t0_ms;
  if (out_t_ms_first) *out_t_ms_first = t_first;

  uint8_t want = s->cfg.frame_samples;
  if (want == 0) want = 16;
  if (want > EMG_PKT_MAX_SAMPLES) want = EMG_PKT_MAX_SAMPLES;

  uint8_t n = 0;
  while (n < want) {
    uint16_t v;
    if (!ring_pop(s, &v)) break;
    out_frame[n++] = v;
  }

  if (s->count == 0) {
    s->t0_valid = 0;
  } else {
    s->t0_ms = advance_time_ms(s, t_first, (uint32_t)n);
  }

  s->dequeued_frames++;
  return n;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

emg_stream_err_t emg_stream_init(emg_stream_t* s,
                                 hm19_t* ble,
                                 const emg_stream_config_t* cfg,
                                 uint16_t* ring_buf_samples,
                                 uint32_t cap_samples)
{
  /**
   * @brief Initialize EMG stream state.
   *
   * Preconditions:
   * - cfg->frame_samples in [1..EMG_PKT_MAX_SAMPLES]
   * - Either sample_rate_hz or sample_period_us must be provided
   * - ring buffer storage must be provided by caller
   *
   * Postconditions:
   * - Stream starts in "no pending", "no queued", seq=0, cfg not sent.
   */
  if (!s || !ble || !cfg || !ring_buf_samples || cap_samples == 0) return EMG_STREAM_EINVAL;
  if (cfg->frame_samples == 0 || cfg->frame_samples > EMG_PKT_MAX_SAMPLES) return EMG_STREAM_EINVAL;
  if (cfg->sample_rate_hz == 0 && cfg->sample_period_us == 0) return EMG_STREAM_EINVAL;

  memset(s, 0, sizeof(*s));
  s->ble = ble;
  s->cfg = *cfg;

  /* Derive sample period if not explicitly provided */
  if (s->cfg.sample_period_us == 0) {
    s->cfg.sample_period_us = 1000000u / s->cfg.sample_rate_hz;
  }

  /* Ring buffer */
  s->buf = ring_buf_samples;
  s->cap = cap_samples;
  s->head = s->tail = s->count = 0;
  s->t0_valid = 0;

  /* Frame accumulator */
  s->frame_len = 0;
  s->frame_t0_valid = 0;

  /* Pending send slot (prevents duplication on busy link) */
  s->pending_valid = 0;

  /* Packet sequence */
  s->seq = 0;

  /* CFG metadata state */
  s->cfg_valid = 0;
  s->cfg_sent = 0;
  s->last_cfg_try_ms = 0;

  /* Connection edge tracking */
  s->last_connected = -1;

  return EMG_STREAM_OK;
}

/**
 * @brief Set the current CFG payload (metadata) to be sent over the link.
 *
 * This marks cfg_valid=1 and forces a resend (cfg_sent=0).
 * The actual sending is controlled by send_cfg_on_connect / cfg_resend_ms logic.
 */
void emg_stream_set_cfg(emg_stream_t* s, const emg_cfg_payload_t* cfg)
{
  if (!s || !cfg) return;
  s->cfg_payload = *cfg;
  s->cfg_valid = 1;
  s->cfg_sent = 0; // force resend
}

/**
 * @brief Update only the baseline_raw field of the CFG payload.
 *
 * @param s            Stream instance.
 * @param baseline_raw New baseline in ADC counts.
 * @param force_resend If non-zero, clears cfg_sent to force a new CFG send.
 */
void emg_stream_update_baseline_raw(emg_stream_t* s, uint16_t baseline_raw, uint8_t force_resend)
{
  if (!s) return;
  s->cfg_payload.baseline_raw = baseline_raw;
  s->cfg_valid = 1;
  if (force_resend) s->cfg_sent = 0;
}

/**
 * @brief Push one raw ADC sample into the stream.
 *
 * This function:
 * - accumulates samples into a fixed-size frame (cfg.frame_samples),
 * - attempts to send immediately if the link is available and no backlog exists,
 * - otherwise enqueues the frame into the ring buffer for later flushing.
 *
 * @param s          Stream instance.
 * @param sample_raw Raw ADC sample.
 * @param t_ms       Timestamp of this sample (ms).
 *
 * Notes:
 * - Immediate send is only attempted when:
 *   (ring empty) AND (no pending frame) AND (should_send_now() true).
 * - If immediate send fails (busy/error), the frame is enqueued to preserve ordering.
 */
void emg_stream_push_sample(emg_stream_t* s, uint16_t sample_raw, uint32_t t_ms)
{
  if (!s) return;

  s->pushed_samples++;

  /* Anchor frame start time on the first sample */
  if (s->frame_len == 0) {
    s->frame_t0_ms = t_ms;
    s->frame_t0_valid = 1;
  }

  s->frame_tmp[s->frame_len++] = sample_raw;

  /* Not enough samples yet to form a frame */
  if (s->frame_len < s->cfg.frame_samples) {
    return;
  }

  /* Frame complete */
  uint8_t n = s->frame_len;
  s->frame_len = 0;

  uint32_t t_first = s->frame_t0_valid ? s->frame_t0_ms : t_ms;
  s->frame_t0_valid = 0;

  /* Fast path: send immediately if possible and we are not backlogged */
  if (s->count == 0 && !s->pending_valid && should_send_now(s)) {

    /* Optional: attempt to send CFG before RAW (best-effort, non-fatal) */
    if (s->cfg.send_cfg_on_connect && s->cfg_valid && !s->cfg_sent) {
      (void)try_send_cfg(s, t_first);
    }

    if (try_send_raw_frame(s, s->frame_tmp, n, t_first)) {
      return;
    }
  }

  /* Slow path: enqueue for later flush */
  enqueue_frame(s, s->frame_tmp, n, t_first);
}

/**
 * @brief Push a contiguous block of raw samples (e.g., from DMA) into the stream.
 *
 * @param s           Stream instance.
 * @param samples_raw Pointer to input samples.
 * @param n           Number of samples.
 * @param t0_ms       Timestamp for the first sample in the block.
 *
 * Timestamp rule:
 * - Each sample i is timestamped as t0_ms advanced by i * sample_period_us.
 */
void emg_stream_push_block(emg_stream_t* s,
                           const uint16_t* samples_raw,
                           uint32_t n,
                           uint32_t t0_ms)
{
  if (!s || !samples_raw || n == 0) return;

  for (uint32_t i = 0; i < n; ++i) {
    uint32_t t_ms = advance_time_ms(s, t0_ms, i);
    emg_stream_push_sample(s, samples_raw[i], t_ms);
  }
}

/**
 * @brief Decide whether it is time to attempt sending the CFG packet.
 *
 * Policy:
 * - Only relevant when send_cfg_on_connect==1 and cfg_valid==1 and cfg_sent==0.
 * - Triggered by:
 *   - Rising edge of connection state (not connected -> connected), OR
 *   - Periodic retry based on cfg_resend_ms (if non-zero).
 *
 * Connection gating:
 * - If require_connected==1: only try when connected_now()==1.
 * - If require_connected==0: periodic retry is allowed even without STATE pin,
 *   but still respects rising edge if available.
 */
static void maybe_send_cfg(emg_stream_t* s)
{
  if (!s) return;
  if (!s->cfg.send_cfg_on_connect) return;
  if (!s->cfg_valid) return;
  if (s->cfg_sent) return;

  int c = connected_now(s);
  uint32_t t_ms = (uint32_t)now_ms();

  /* Rising edge detection: (prev != connected) -> connected */
  int rising = (c == 1) && (s->last_connected != 1);

  /* Time-based retry */
  int time_ok = 0;
  if (s->cfg.cfg_resend_ms != 0) {
    uint32_t last = s->last_cfg_try_ms;
    if (last == 0 || (t_ms >= last && (t_ms - last) >= s->cfg.cfg_resend_ms)) {
      time_ok = 1;
    }
  }

  s->last_connected = c;

  if (s->cfg.require_connected) {
    if (c != 1) return;
    if (!rising && !time_ok) return;
  } else {
    if (!rising && !time_ok) return;
  }

  s->last_cfg_try_ms = t_ms;
  (void)try_send_cfg(s, t_ms);
}

/**
 * @brief Periodic flush function (call from main loop).
 *
 * This function:
 * - optionally attempts to send CFG (before RAW flush),
 * - enforces connection requirement if configured,
 * - flushes queued frames within budgets:
 *     - max_flush_packets_per_tick
 *     - flush_budget_ms
 * - uses a "pending" slot to avoid duplicating frames on transient TX failures.
 *
 * Pending slot rationale:
 * - If sending fails, we keep the dequeued frame as pending and exit.
 * - Next tick retries sending the same pending frame first.
 * - This prevents reordering, duplication, or re-enqueue overhead.
 */
void emg_stream_tick(emg_stream_t* s)
{
  if (!s) return;

  /* CFG may be attempted before flushing RAW */
  maybe_send_cfg(s);

  /* If required, do not flush when not connected */
  if (s->cfg.require_connected) {
    int c = connected_now(s);
    s->last_connected = c;
    if (c != 1) return;
  }

  uint64_t t_start = now_ms();
  uint8_t sent_now = 0;

  /* Pending frame first */
  if (s->pending_valid) {
    if (try_send_raw_frame(s, s->pending_frame, s->pending_len, s->pending_t_ms)) {
      s->pending_valid = 0;
    } else {
      return; /* Link busy/error: do not touch the ring */
    }
  }

  while (s->count > 0) {

    /* Packet-count budget */
    if (s->cfg.max_flush_packets_per_tick && sent_now >= s->cfg.max_flush_packets_per_tick) {
      break;
    }

    /* Time budget */
    if (s->cfg.flush_budget_ms) {
      uint64_t t_now = now_ms();
      if (t_now >= t_start && (t_now - t_start) >= s->cfg.flush_budget_ms) {
        break;
      }
    }

    /* Dequeue one frame from ring into pending slot */
    uint32_t t_ms_first = 0;
    uint8_t n = dequeue_frame(s, s->pending_frame, &t_ms_first);
    if (n == 0) break;

    s->pending_len   = n;
    s->pending_t_ms  = t_ms_first;
    s->pending_valid = 1;

    /* Best-effort CFG attempt before RAW */
    if (s->cfg.send_cfg_on_connect && s->cfg_valid && !s->cfg_sent) {
      (void)try_send_cfg(s, t_ms_first);
    }

    if (try_send_raw_frame(s, s->pending_frame, s->pending_len, s->pending_t_ms)) {
      s->pending_valid = 0;
      sent_now++;
      continue;
    }

    /* Send failed: keep pending and exit (no duplication, no re-enqueue) */
    return;
  }
}
