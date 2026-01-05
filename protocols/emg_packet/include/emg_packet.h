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
 * @file emg_packet.h
 * @brief EMG packet format definitions and builders.
 *
 * This header defines the binary packet format used to stream EMG data
 * from the embedded device to a host (PC, logger, BLE gateway, etc.).
 *
 * Design goals:
 * - Deterministic, byte-oriented format (easy to parse in C/Python).
 * - Little-endian encoding for all multi-byte fields.
 * - Explicit length field to allow framing over unreliable transports.
 * - CRC16-CCITT for payload integrity.
 *
 * This module is platform-agnostic and does not depend on Omnia port,
 * HAL, or RTOS APIs.
 */

#ifndef EMG_PACKET_H
#define EMG_PACKET_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Sync bytes                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Packet synchronization bytes.
 *
 * These two bytes mark the start of a packet and allow the receiver
 * to re-synchronize after data loss or corruption.
 */
#define EMG_PKT_SYNC0  0xAAu
#define EMG_PKT_SYNC1  0x55u

/* -------------------------------------------------------------------------- */
/* Packet types                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief EMG packet type identifiers.
 */
typedef enum {
  /** Raw EMG samples (uint16 ADC counts). */
  EMG_PKT_TYPE_RAW_U16 = 0x00u,

  /** Configuration / context metadata packet. */
  EMG_PKT_TYPE_CFG     = 0x01u,
} emg_pkt_type_t;

/* -------------------------------------------------------------------------- */
/* Limits                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Maximum number of raw samples per RAW packet.
 *
 * This upper bound is chosen to keep packets small and suitable
 * for real-time streaming over UART/BLE.
 */
#define EMG_PKT_MAX_SAMPLES  32u

/* -------------------------------------------------------------------------- */
/* CRC                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Compute CRC16-CCITT over a byte buffer.
 *
 * Parameters:
 * - Polynomial: 0x1021
 * - Initial value: 0xFFFF
 * - No bit reflection
 *
 * @param data Pointer to input data.
 * @param len  Number of bytes.
 *
 * @return Computed CRC16 value.
 */
uint16_t emg_crc16_ccitt(const uint8_t* data, size_t len);

/* -------------------------------------------------------------------------- */
/* Configuration payload                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configuration payload carried by EMG_PKT_TYPE_CFG packets.
 *
 * This structure is serialized verbatim into the packet payload
 * (all multi-byte fields are little-endian).
 *
 * It provides enough context for the receiver to correctly interpret
 * raw EMG samples (scaling, sampling rate, baseline, etc.).
 */
typedef struct {
  uint8_t  resolution_bits;   /**< ADC resolution in bits (e.g. 12). */
  uint16_t vref_mv;           /**< ADC reference voltage in millivolts (e.g. 3300). */
  uint8_t  adc_channel;       /**< ADC channel index (informative). */
  uint16_t sample_rate_hz;    /**< Sampling rate in Hz (e.g. 2000). */
  uint16_t baseline_raw;      /**< Baseline value in raw ADC counts. */
  int16_t  gain_q15;          /**< Optional gain in Q15 format (0 if unused). */
} emg_cfg_payload_t;

/* -------------------------------------------------------------------------- */
/* Packet size helpers                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Common header length excluding SYNC and LEN.
 *
 * Layout:
 *   TYPE (1) + SEQ (1) + TIME_MS (4)
 */
#define EMG_PKT_COMMON_HDR_LEN   (2u /*SYNC*/ + 2u /*LEN*/ + 1u /*TYPE*/ + 1u /*SEQ*/ + 4u /*TIME*/)

/** Length of CRC field in bytes. */
#define EMG_PKT_CRC_LEN          (2u)

/** RAW packet payload header length (NSAMPLES field). */
#define EMG_PKT_RAW_HDR_LEN      (1u /*NS*/)

/** Configuration payload length in bytes (fixed). */
#define EMG_PKT_CFG_PAYLOAD_LEN  (1u + 2u + 1u + 2u + 2u + 2u)  /* = 10 bytes */

/**
 * @brief Compute LEN field for a RAW packet.
 *
 * LEN includes:
 *   TYPE + SEQ + TIME + NS + DATA + CRC
 *
 * @param nsamples Number of raw samples.
 *
 * @return Value to be written into the LEN field.
 */
static inline uint16_t emg_pkt_len_raw(uint8_t nsamples)
{
  return (uint16_t)(1u + 1u + 4u + 1u + (uint16_t)nsamples * 2u + 2u);
}

/**
 * @brief Compute LEN field for a CFG packet.
 *
 * LEN includes:
 *   TYPE + SEQ + TIME + CFG_PAYLOAD + CRC
 *
 * @return Value to be written into the LEN field.
 */
static inline uint16_t emg_pkt_len_cfg(void)
{
  return (uint16_t)(1u + 1u + 4u + EMG_PKT_CFG_PAYLOAD_LEN + 2u);
}

/**
 * @brief Compute total packet size from LEN field.
 *
 * Total size = SYNC(2) + LEN(2) + LEN
 *
 * @param len_field Value of the LEN field.
 *
 * @return Total packet size in bytes.
 */
static inline size_t emg_pkt_total_from_len(uint16_t len_field)
{
  return (size_t)(4u + (size_t)len_field);
}

/**
 * @brief Compute total size of a RAW packet.
 */
static inline size_t emg_pkt_total_raw(uint8_t nsamples)
{
  return emg_pkt_total_from_len(emg_pkt_len_raw(nsamples));
}

/**
 * @brief Compute total size of a CFG packet.
 */
static inline size_t emg_pkt_total_cfg(void)
{
  return emg_pkt_total_from_len(emg_pkt_len_cfg());
}

/* -------------------------------------------------------------------------- */
/* Packet builders                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Build a RAW_U16 EMG packet.
 *
 * @param seq          Sequence counter.
 * @param time_ms      Timestamp in milliseconds.
 * @param samples      Pointer to raw ADC samples.
 * @param nsamples     Number of samples.
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
                                size_t          out_buf_size);

/**
 * @brief Build a configuration (CFG) packet.
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
                            size_t                   out_buf_size);

#ifdef __cplusplus
}
#endif

#endif /* EMG_PACKET_H */
