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
 * @file emg_packet.c
 * @brief EMG packet builder and CRC utilities.
 *
 * This module implements the serialization logic for EMG packets sent over
 * the transport layer (UART/BLE/etc.). It is fully platform-agnostic and
 * does not depend on Omnia port or HAL APIs.
 *
 * Packet format (generic):
 *   [SYNC0][SYNC1][LEN_L][LEN_H][TYPE][... PAYLOAD ...][CRC_L][CRC_H]
 *
 * Notes:
 * - All multi-byte fields are encoded in little-endian.
 * - CRC16-CCITT is computed over the region [TYPE .. last payload byte],
 *   explicitly excluding SYNC, LEN, and the CRC field itself.
 */

#include "emg_packet.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Little-endian write helpers                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Write a 16-bit unsigned integer in little-endian format.
 *
 * @param buf Destination buffer (must have at least 2 bytes).
 * @param v   Value to encode.
 */
static inline void write_u16_le(uint8_t* buf, uint16_t v)
{
  buf[0] = (uint8_t)(v & 0xFFu);
  buf[1] = (uint8_t)((v >> 8) & 0xFFu);
}

/**
 * @brief Write a 16-bit signed integer in little-endian format.
 *
 * @param buf Destination buffer (must have at least 2 bytes).
 * @param v   Value to encode.
 */
static inline void write_i16_le(uint8_t* buf, int16_t v)
{
  write_u16_le(buf, (uint16_t)v);
}

/**
 * @brief Write a 32-bit unsigned integer in little-endian format.
 *
 * @param buf Destination buffer (must have at least 4 bytes).
 * @param v   Value to encode.
 */
static inline void write_u32_le(uint8_t* buf, uint32_t v)
{
  buf[0] = (uint8_t)(v & 0xFFu);
  buf[1] = (uint8_t)((v >> 8) & 0xFFu);
  buf[2] = (uint8_t)((v >> 16) & 0xFFu);
  buf[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* -------------------------------------------------------------------------- */
/* CRC16-CCITT                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Compute CRC16-CCITT over a byte buffer.
 *
 * Polynomial: 0x1021  
 * Initial value: 0xFFFF  
 *
 * @param data Pointer to input data.
 * @param len  Number of bytes.
 *
 * @return Computed CRC16 value.
 */
uint16_t emg_crc16_ccitt(const uint8_t* data, size_t len)
{
  uint16_t crc = 0xFFFFu;
  const uint16_t poly = 0x1021u;

  for (size_t i = 0; i < len; ++i) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u)
              ? (uint16_t)((crc << 1) ^ poly)
              : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

/**
 * @brief Compute CRC over the packet logical payload region.
 *
 * The CRC is computed over:
 *   [TYPE .. last payload byte]
 *
 * Excluded fields:
 *   - SYNC (2 bytes)
 *   - LEN  (2 bytes)
 *   - CRC  (2 bytes)
 *
 * @param out_buf   Pointer to the start of the packet buffer.
 * @param len_field Value of the LEN field (payload + CRC).
 *
 * @return Computed CRC16 value.
 */
static inline uint16_t
compute_crc_over_len_region(const uint8_t* out_buf, uint16_t len_field)
{
  /* Layout:
   * [SYNC0][SYNC1][LENL][LENH][TYPE .. PAYLOAD .. CRC_L][CRC_H]
   */
  const uint8_t* start = out_buf + 4u;           /* TYPE starts here */
  const size_t   n     = (size_t)len_field - 2u; /* exclude CRC itself */
  return emg_crc16_ccitt(start, n);
}

/* -------------------------------------------------------------------------- */
/* Packet builders                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build a RAW_U16 EMG packet.
 *
 * Payload layout:
 *   TYPE (1)
 *   SEQ  (1)
 *   TIME_MS (4)
 *   NSAMPLES (1)
 *   SAMPLES[nsamples] (uint16 LE each)
 *   CRC (2)
 *
 * @param seq          Sequence counter.
 * @param time_ms      Timestamp in milliseconds.
 * @param samples      Pointer to raw ADC samples.
 * @param nsamples     Number of samples (1..EMG_PKT_MAX_SAMPLES).
 * @param out_buf      Destination buffer.
 * @param out_buf_size Size of destination buffer in bytes.
 *
 * @return Total packet size in bytes on success, 0 on error.
 */
size_t emg_build_raw_u16_packet(uint8_t         seq,
                                uint32_t        time_ms,
                                const uint16_t* samples,
                                uint8_t         nsamples,
                                uint8_t*        out_buf,
                                size_t          out_buf_size)
{
  if (!samples || !out_buf) return 0;
  if (nsamples == 0 || nsamples > EMG_PKT_MAX_SAMPLES) return 0;

  const uint16_t len_field = emg_pkt_len_raw(nsamples);
  const size_t total_size = emg_pkt_total_from_len(len_field);
  if (out_buf_size < total_size) return 0;

  uint8_t* p = out_buf;

  /* SYNC */
  *p++ = EMG_PKT_SYNC0;
  *p++ = EMG_PKT_SYNC1;

  /* LEN */
  write_u16_le(p, len_field);
  p += 2;

  /* TYPE */
  *p++ = (uint8_t)EMG_PKT_TYPE_RAW_U16;

  /* SEQ */
  *p++ = seq;

  /* TIME (ms) */
  write_u32_le(p, time_ms);
  p += 4;

  /* NSAMPLES */
  *p++ = nsamples;

  /* DATA: raw samples (uint16, LE) */
  for (uint8_t i = 0; i < nsamples; ++i) {
    write_u16_le(p, samples[i]);
    p += 2;
  }

  /* CRC */
  {
    uint16_t crc = compute_crc_over_len_region(out_buf, len_field);
    write_u16_le(p, crc);
    p += 2;
  }

  return ((size_t)(p - out_buf) == total_size) ? total_size : 0;
}

/**
 * @brief Build a configuration (CFG) packet.
 *
 * Payload layout (fixed-size):
 *   TYPE (1)
 *   SEQ  (1)
 *   TIME_MS (4)
 *   resolution_bits (1)
 *   vref_mv (2)
 *   adc_channel (1)
 *   sample_rate_hz (2)
 *   baseline_raw (2)
 *   gain_q15 (2)
 *   CRC (2)
 *
 * @param seq          Sequence counter.
 * @param time_ms      Timestamp in milliseconds.
 * @param cfg          Pointer to configuration payload.
 * @param out_buf      Destination buffer.
 * @param out_buf_size Size of destination buffer in bytes.
 *
 * @return Total packet size in bytes on success, 0 on error.
 */
size_t emg_build_cfg_packet(uint8_t                  seq,
                            uint32_t                 time_ms,
                            const emg_cfg_payload_t* cfg,
                            uint8_t*                 out_buf,
                            size_t                   out_buf_size)
{
  if (!cfg || !out_buf) return 0;

  const uint16_t len_field = emg_pkt_len_cfg();
  const size_t total_size = emg_pkt_total_from_len(len_field);
  if (out_buf_size < total_size) return 0;

  uint8_t* p = out_buf;

  /* SYNC */
  *p++ = EMG_PKT_SYNC0;
  *p++ = EMG_PKT_SYNC1;

  /* LEN */
  write_u16_le(p, len_field);
  p += 2;

  /* TYPE */
  *p++ = (uint8_t)EMG_PKT_TYPE_CFG;

  /* SEQ */
  *p++ = seq;

  /* TIME (ms) */
  write_u32_le(p, time_ms);
  p += 4;

  /* CFG payload (fixed layout) */
  *p++ = cfg->resolution_bits;
  write_u16_le(p, cfg->vref_mv);        p += 2;
  *p++ = cfg->adc_channel;
  write_u16_le(p, cfg->sample_rate_hz); p += 2;
  write_u16_le(p, cfg->baseline_raw);   p += 2;
  write_i16_le(p, cfg->gain_q15);        p += 2;

  /* CRC */
  {
    uint16_t crc = compute_crc_over_len_region(out_buf, len_field);
    write_u16_le(p, crc);
    p += 2;
  }

  return ((size_t)(p - out_buf) == total_size) ? total_size : 0;
}
