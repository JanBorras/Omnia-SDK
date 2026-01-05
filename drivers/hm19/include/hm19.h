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

#ifndef HM19_H
#define HM19_H

#include "omnia_port.h"
#include "omnia_uart.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file hm19.h
 * @brief HM-19 BLE module driver (UART transport and AT command helpers).
 *
 * This header defines the public API for controlling and communicating with
 * an HM-19 BLE module through the Omnia UART and Port contracts.
 *
 * The driver supports:
 * - DATA mode raw transport over UART.
 * - AT command send + line-based response read.
 * - Optional RX draining before AT transactions.
 * - Optional chunked TX to accommodate UART/DMA limitations.
 * - Optional connection state probing through a GPIO pin.
 * - Optional mutex protection for thread-safe access when the platform provides it.
 */

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief HM-19 operating mode (logical).
 *
 * Note: The mode field is currently used as a software-level state indicator.
 * Switching between modes may require additional module-specific sequences
 * depending on the firmware variant.
 */
typedef enum {
    /** Data passthrough mode (UART payload). */
    HM19_MODE_DATA = 0,

    /** AT command mode (command/response transactions). */
    HM19_MODE_AT
} hm19_mode_t;

/**
 * @brief AT line ending convention.
 */
typedef enum {
    /** Line feed ("\n"). */
    HM19_AT_EOL_LF = 0,

    /** Carriage return + line feed ("\r\n"). */
    HM19_AT_EOL_CRLF
} hm19_at_eol_t;

/**
 * @brief EN pin polarity.
 *
 * EN polarity depends on board wiring/module variant.
 */
typedef enum {
    /** EN is active high (logic 1 enables). */
    HM19_EN_ACTIVE_HIGH = 0,

    /** EN is active low (logic 0 enables). */
    HM19_EN_ACTIVE_LOW
} hm19_en_polarity_t;

/**
 * @brief Driver behavior flags (bitmask).
 */
typedef enum {
    /** Protect public operations with a mutex when available. */
    HM19_F_USE_MUTEX             = 1u << 0,

    /** Drain pending RX bytes before sending AT commands. */
    HM19_F_DRAIN_RX_BEFORE_AT    = 1u << 1,

    /** Split large TX buffers into smaller chunks. */
    HM19_F_CHUNK_TX              = 1u << 2,

    /** Enable GPIO-based connected-state checks. */
    HM19_F_CHECK_CONNECTED       = 1u << 3,

    /** Drop outgoing data when not connected (requires connected-state checks). */
    HM19_F_DROP_IF_NOT_CONNECTED = 1u << 4
} hm19_flags_t;

/**
 * @brief HM-19 configuration.
 *
 * All fields have sensible defaults when @ref hm19_init is called with cfg == NULL.
 */
typedef struct {
    /** Behavior flags bitmask (see @ref hm19_flags_t). */
    uint32_t flags;

    /** AT line ending convention. */
    hm19_at_eol_t      at_eol;

    /** EN pin polarity. */
    hm19_en_polarity_t en_pol;

    /** Boot delay after reset/enable, in milliseconds. */
    uint32_t           boot_delay_ms;

    /** Time budget for draining RX before AT, in milliseconds. */
    uint32_t           drain_budget_ms;

    /** TX chunk size in bytes when chunked TX is enabled. */
    uint16_t           tx_chunk_size;

    /** Maximum size used when reading one AT response line. */
    uint16_t           max_at_line;
} hm19_config_t;

/**
 * @brief HM-19 device instance.
 *
 * The UART handle is provided by the application/platform and is not owned
 * by the driver. The optional mutex is created/destroyed via port hooks.
 */
typedef struct {
    /** UART handle used for communication (not owned). */
    const omnia_uart_handle_t* uart;

    /** Optional enable pin (0/NULL if unused). */
    omnia_gpio_t               pin_en;

    /** Optional connection-state pin (0/NULL if unused). */
    omnia_gpio_t               pin_state;

    /** Current logical driver mode. */
    hm19_mode_t                mode;

    /** Driver configuration (copied on init). */
    hm19_config_t              cfg;

    /** Optional mutex handle (created via port hooks). */
    omnia_mutex_t              lock;
} hm19_t;

/* -------------------------------------------------------------------------- */
/* Lifecycle                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the HM-19 device instance.
 *
 * The driver copies the provided configuration (or applies defaults), optionally
 * creates a mutex if enabled and supported, and performs a best-effort boot delay.
 *
 * @param dev       Device instance to initialize.
 * @param uart      UART handle used for communication.
 * @param pin_en    Optional EN pin (0/NULL if unused).
 * @param pin_state Optional STATE pin for connection probing (0/NULL if unused).
 * @param cfg       Optional configuration (NULL selects defaults).
 * @return Status code.
 */
omnia_status_t hm19_init(hm19_t* dev,
                         const omnia_uart_handle_t* uart,
                         omnia_gpio_t pin_en,
                         omnia_gpio_t pin_state,
                         const hm19_config_t* cfg);

/**
 * @brief Deinitialize the HM-19 device instance.
 *
 * Releases the optional mutex (if created).
 *
 * @param dev Device instance.
 */
void hm19_deinit(hm19_t* dev);

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/** @brief Set AT line ending convention. */
void hm19_set_at_eol(hm19_t* dev, hm19_at_eol_t eol);

/** @brief Set EN pin polarity. */
void hm19_set_en_polarity(hm19_t* dev, hm19_en_polarity_t pol);

/** @brief Set boot delay in milliseconds. */
void hm19_set_boot_delay_ms(hm19_t* dev, uint32_t ms);

/** @brief Replace the driver flags bitmask. */
void hm19_set_flags(hm19_t* dev, uint32_t flags);

/** @brief Set TX chunk size in bytes (used when HM19_F_CHUNK_TX is enabled). */
void hm19_set_tx_chunk(hm19_t* dev, uint16_t chunk_size);

/** @brief Set drain budget in milliseconds (used when HM19_F_DRAIN_RX_BEFORE_AT is enabled). */
void hm19_set_drain_budget(hm19_t* dev, uint32_t budget_ms);

/* -------------------------------------------------------------------------- */
/* Status                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Query connection state through the optional STATE pin.
 *
 * This function returns -1 when connection probing is unsupported or disabled.
 *
 * @param dev Device instance.
 * @return 1 if connected, 0 if not connected, -1 if unknown/unsupported.
 */
int hm19_is_connected(hm19_t* dev);

/* -------------------------------------------------------------------------- */
/* Transport                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Send raw bytes to the module over UART.
 *
 * Behavior may be affected by configuration flags:
 * - Chunked TX (HM19_F_CHUNK_TX).
 * - Drop if not connected (HM19_F_DROP_IF_NOT_CONNECTED).
 * - Mutex protection (HM19_F_USE_MUTEX).
 *
 * @param dev        Device instance.
 * @param data       Data buffer.
 * @param len        Buffer length.
 * @param timeout_ms Timeout in milliseconds (per write operation).
 * @return Status code.
 */
omnia_status_t hm19_send_raw(hm19_t* dev,
                             const uint8_t* data,
                             size_t len,
                             uint32_t timeout_ms);

/**
 * @brief Send a text line to the module (appends configured EOL).
 *
 * @param dev        Device instance.
 * @param line       Null-terminated line (without EOL).
 * @param timeout_ms Timeout in milliseconds.
 * @return Status code.
 */
omnia_status_t hm19_send_line(hm19_t* dev,
                              const char* line,
                              uint32_t timeout_ms);

/**
 * @brief Send an AT command and read a single response line.
 *
 * If @p resp is NULL (or resp_size == 0), the command is sent and no response
 * is read. When enabled, RX may be drained before sending the command.
 *
 * @param dev        Device instance.
 * @param cmd        AT command string (without EOL).
 * @param resp       Optional response buffer (may be NULL).
 * @param resp_size  Response buffer size.
 * @param timeout_ms Timeout in milliseconds.
 * @return Status code.
 */
omnia_status_t hm19_send_at(hm19_t* dev,
                            const char* cmd,
                            char* resp,
                            size_t resp_size,
                            uint32_t timeout_ms);

/* -------------------------------------------------------------------------- */
/* Multi-line AT                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Send an AT command and capture up to N response lines.
 *
 * Lines are appended to @p out_buf separated by '\n'. The function stops when:
 * - @p max_lines lines have been captured,
 * - output buffer capacity is reached, or
 * - a UART read fails.
 *
 * @param dev        Device instance.
 * @param cmd        AT command string (without EOL).
 * @param out_buf    Output buffer.
 * @param out_size   Output buffer size in bytes.
 * @param max_lines  Maximum number of lines to capture.
 * @param timeout_ms Timeout in milliseconds per line.
 * @return Number of lines captured on success, negative value on error.
 */
int hm19_send_at_multi(hm19_t* dev,
                       const char* cmd,
                       char* out_buf,
                       size_t out_size,
                       uint8_t max_lines,
                       uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* HM19_H */
