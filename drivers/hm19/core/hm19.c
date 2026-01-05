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
 * @file hm19.c
 * @brief HM-19 BLE module driver (UART transport + AT command helper layer).
 *
 * This driver provides:
 * - Raw data transmission over UART (DATA mode).
 * - AT command transmission and line-based response parsing.
 * - Optional GPIO-based connected-state probing.
 * - Optional mutex protection for thread-safe access.
 *
 * Design goals:
 * - No heap allocations (mutex object is created via port hooks if enabled).
 * - Deterministic behavior with bounded reads/drains.
 * - Best-effort robustness under noisy UART conditions (drain-before-AT, chunked TX).
 */

#include "hm19.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Port glue helpers                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get the active Omnia port instance.
 *
 * @return Pointer to active port, or NULL if not registered.
 */
static inline omnia_port_t* hm19_port(void) { return omnia_port_get(); }

/**
 * @brief Build a default configuration for the HM-19 driver.
 *
 * @return Default configuration structure.
 */
static hm19_config_t hm19_default_cfg(void)
{
    hm19_config_t c;
    c.flags           = (HM19_F_USE_MUTEX | HM19_F_DRAIN_RX_BEFORE_AT | HM19_F_CHUNK_TX);
    c.at_eol          = HM19_AT_EOL_CRLF;
    c.en_pol          = HM19_EN_ACTIVE_HIGH;
    c.boot_delay_ms   = 100;
    c.drain_budget_ms = 20;
    c.tx_chunk_size   = 64;
    c.max_at_line     = 128;
    return c;
}

/**
 * @brief Get current time in milliseconds.
 *
 * Uses the Omnia port millis() hook when available.
 *
 * @return Current time in ms, or 0 if unavailable.
 */
static inline uint64_t hm19_now_ms(void)
{
    omnia_port_t* p = hm19_port();
    if (!p || !p->v || !p->v->millis) return 0;
    return p->v->millis();
}

/**
 * @brief Delay for a given number of milliseconds.
 *
 * Implemented as repeated delay_us(1000). If delay_us is not available, this
 * becomes a no-op.
 *
 * @param ms Delay duration in milliseconds.
 */
static inline void hm19_delay_ms(uint32_t ms)
{
    omnia_port_t* port = hm19_port();
    if (!port || !port->v || !port->v->delay_us) return;
    while (ms--) port->v->delay_us(1000);
}

/**
 * @brief Write a GPIO level using the port contract (best-effort).
 *
 * @param pin   GPIO identifier.
 * @param level Logic level to write.
 */
static inline void hm19_gpio_write(omnia_gpio_t pin, int level)
{
    omnia_port_t* port = hm19_port();
    if (!port || !port->v || !port->v->gpio_write) return;
    (void)port->v->gpio_write(pin, level);
}

/**
 * @brief Read a GPIO level using the port contract.
 *
 * @param pin GPIO identifier.
 * @return GPIO level (0/1) or -1 if unavailable.
 */
static inline int hm19_gpio_read(omnia_gpio_t pin)
{
    omnia_port_t* port = hm19_port();
    if (!port || !port->v || !port->v->gpio_read) return -1;
    return port->v->gpio_read(pin);
}

/**
 * @brief Compute the GPIO level that enables the module.
 *
 * @param dev HM-19 instance.
 * @return Logic level corresponding to "ON".
 */
static inline int hm19_en_level_on(const hm19_t* dev)
{
    return (dev && dev->cfg.en_pol == HM19_EN_ACTIVE_LOW) ? 0 : 1;
}

/**
 * @brief Compute the GPIO level that disables the module.
 *
 * @param dev HM-19 instance.
 * @return Logic level corresponding to "OFF".
 */
static inline int hm19_en_level_off(const hm19_t* dev)
{
    return (dev && dev->cfg.en_pol == HM19_EN_ACTIVE_LOW) ? 1 : 0;
}

/* -------------------------------------------------------------------------- */
/* Mutex helpers                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Acquire the optional device mutex (if enabled and available).
 *
 * @param dev HM-19 instance.
 */
static inline void hm19_lock(hm19_t* dev)
{
    if (!dev) return;
    if (!(dev->cfg.flags & HM19_F_USE_MUTEX)) return;

    omnia_port_t* port = hm19_port();
    if (!port || !port->v) return;
    if (dev->lock && port->v->mutex_lock) port->v->mutex_lock(dev->lock);
}

/**
 * @brief Release the optional device mutex (if enabled and available).
 *
 * @param dev HM-19 instance.
 */
static inline void hm19_unlock(hm19_t* dev)
{
    if (!dev) return;
    if (!(dev->cfg.flags & HM19_F_USE_MUTEX)) return;

    omnia_port_t* port = hm19_port();
    if (!port || !port->v) return;
    if (dev->lock && port->v->mutex_unlock) port->v->mutex_unlock(dev->lock);
}

/* -------------------------------------------------------------------------- */
/* Connected-state probing                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Query connection state via a GPIO pin (optional feature).
 *
 * This is only available when:
 * - HM19_F_CHECK_CONNECTED is enabled, and
 * - a state pin is configured, and
 * - the underlying port provides gpio_read().
 *
 * @param dev HM-19 instance.
 * @return 1 if connected, 0 if not connected, -1 if unsupported/unknown.
 */
int hm19_is_connected(hm19_t* dev)
{
    if (!dev) return -1;
    if (!(dev->cfg.flags & HM19_F_CHECK_CONNECTED)) return -1;
    if (!dev->pin_state) return -1;

    int v = hm19_gpio_read(dev->pin_state);
    if (v < 0) return -1;

    return (v != 0) ? 1 : 0;
}

/* -------------------------------------------------------------------------- */
/* Hardware reset                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Perform a hardware reset (best-effort).
 *
 * Current implementation is a conservative boot delay only. The original EN-pin
 * toggling logic is intentionally disabled to avoid incorrect polarity/wiring
 * assumptions across boards.
 *
 * @param dev HM-19 instance.
 */
static void hm19_hw_reset(hm19_t* dev)
{
    if (!dev) return;

    uint32_t boot_ms = dev->cfg.boot_delay_ms ? dev->cfg.boot_delay_ms : 100;
    hm19_delay_ms(boot_ms);
}

/* -------------------------------------------------------------------------- */
/* TX core (NO LOCK)                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Write a buffer to UART, ensuring all bytes are transmitted (no lock).
 *
 * Optionally:
 * - Drops data if not connected (when configured).
 * - Splits TX into chunks (when configured) to fit platform UART limitations.
 *
 * @param dev        HM-19 instance.
 * @param data       Data buffer.
 * @param len        Buffer length.
 * @param timeout_ms Per-write timeout in milliseconds.
 * @return Status code.
 */
static omnia_status_t hm19_uart_write_all_nolock(hm19_t* dev,
                                                const uint8_t* data,
                                                size_t len,
                                                uint32_t timeout_ms)
{
    if (!dev || !dev->uart || !data || len == 0) return OMNIA_EINVAL;

    if (dev->cfg.flags & HM19_F_DROP_IF_NOT_CONNECTED) {
        int c = hm19_is_connected(dev);
        if (c == 0) return OMNIA_EIO;
    }

    if (!(dev->cfg.flags & HM19_F_CHUNK_TX)) {
        return omnia_uart_write(dev->uart, data, len, timeout_ms);
    }

    uint16_t chunk = dev->cfg.tx_chunk_size ? dev->cfg.tx_chunk_size : 64;

    size_t off = 0;
    while (off < len) {
        size_t n = len - off;
        if (n > chunk) n = chunk;

        omnia_status_t st = omnia_uart_write(dev->uart, data + off, n, timeout_ms);
        if (st != OMNIA_OK) return st;

        off += n;
    }

    return OMNIA_OK;
}

/**
 * @brief Write the configured AT end-of-line marker (no lock).
 *
 * @param dev        HM-19 instance.
 * @param timeout_ms Per-write timeout in milliseconds.
 * @return Status code.
 */
static omnia_status_t hm19_write_eol_nolock(hm19_t* dev, uint32_t timeout_ms)
{
    if (!dev) return OMNIA_EINVAL;

    uint8_t eol[2];
    size_t n = 0;

    if (dev->cfg.at_eol == HM19_AT_EOL_CRLF) {
        eol[n++] = '\r';
        eol[n++] = '\n';
    } else {
        eol[n++] = '\n';
    }

    return hm19_uart_write_all_nolock(dev, eol, n, timeout_ms);
}

/**
 * @brief Send a line terminated by EOL (no lock).
 *
 * @param dev        HM-19 instance.
 * @param line       Null-terminated line (without EOL).
 * @param timeout_ms Per-write timeout in milliseconds.
 * @return Status code.
 */
static omnia_status_t hm19_send_line_nolock(hm19_t* dev,
                                           const char* line,
                                           uint32_t timeout_ms)
{
    if (!dev || !line) return OMNIA_EINVAL;

    const size_t len = strlen(line);
    if (len > 0) {
        omnia_status_t st =
            hm19_uart_write_all_nolock(dev, (const uint8_t*)line, len, timeout_ms);
        if (st != OMNIA_OK) return st;
    }

    return hm19_write_eol_nolock(dev, timeout_ms);
}

/* -------------------------------------------------------------------------- */
/* Drain RX (NO LOCK, time-based)                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Drain pending RX bytes before issuing an AT command (no lock).
 *
 * This reduces the chance of mixing old asynchronous data with AT responses.
 * The drain is bounded either by elapsed time (millis) or by a conservative
 * iteration budget if millis() is unavailable.
 *
 * @param dev HM-19 instance.
 */
static void hm19_drain_rx_nolock(hm19_t* dev)
{
    if (!dev || !dev->uart) return;
    if (!(dev->cfg.flags & HM19_F_DRAIN_RX_BEFORE_AT)) return;

    uint32_t budget_ms = dev->cfg.drain_budget_ms ? dev->cfg.drain_budget_ms : 20;
    uint64_t t0 = hm19_now_ms();

    uint8_t ch = 0;

    while (1) {
        if (t0 != 0) {
            uint64_t now = hm19_now_ms();
            if (now >= t0 && (now - t0) >= budget_ms) break;
        } else {
            if (budget_ms-- == 0) break;
        }

        /* Read one byte with a short timeout (1 ms). */
        omnia_status_t st = omnia_uart_read(dev->uart, &ch, 1, 1);
        if (st != OMNIA_OK) break;
    }
}

/* -------------------------------------------------------------------------- */
/* Read line (NO LOCK)                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read a single line from UART (no lock).
 *
 * The line terminator is CR or LF. Empty lines are skipped.
 * The output is always NUL-terminated.
 *
 * @param dev        HM-19 instance.
 * @param buf        Output buffer.
 * @param buf_size   Output buffer size in bytes.
 * @param timeout_ms Per-byte timeout in milliseconds.
 * @return OMNIA_OK if a non-empty line is read, OMNIA_EIO otherwise.
 */
static omnia_status_t hm19_read_line_nolock(hm19_t* dev,
                                           char* buf,
                                           size_t buf_size,
                                           uint32_t timeout_ms)
{
    if (!dev || !dev->uart || !buf || buf_size == 0) return OMNIA_EINVAL;

    size_t idx = 0;

    while (idx + 1 < buf_size) {
        uint8_t ch = 0;
        omnia_status_t st = omnia_uart_read(dev->uart, &ch, 1, timeout_ms);
        if (st != OMNIA_OK) break;

        if (ch == '\r' || ch == '\n') {
            if (idx == 0) continue; /* skip empty lines */
            break;
        }

        buf[idx++] = (char)ch;
    }

    buf[idx] = '\0';
    return (idx > 0) ? OMNIA_OK : OMNIA_EIO;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the HM-19 driver instance.
 *
 * The driver is initialized in DATA mode and may optionally create a mutex
 * via the Omnia port hooks when enabled by configuration.
 *
 * A best-effort AT ping may be issued to validate transport wiring.
 *
 * @param dev       HM-19 instance to initialize.
 * @param uart      UART handle used for communication.
 * @param pin_en    Optional enable pin (0/NULL if unused).
 * @param pin_state Optional state pin for connection probing (0/NULL if unused).
 * @param cfg       Optional configuration (NULL selects defaults).
 * @return Status code.
 */
omnia_status_t hm19_init(hm19_t* dev,
                         const omnia_uart_handle_t* uart,
                         omnia_gpio_t pin_en,
                         omnia_gpio_t pin_state,
                         const hm19_config_t* cfg)
{
    if (!dev || !uart) return OMNIA_EINVAL;

    memset(dev, 0, sizeof(*dev));

    dev->uart      = uart;
    dev->pin_en    = pin_en;
    dev->pin_state = pin_state;
    dev->mode      = HM19_MODE_DATA;

    dev->cfg = cfg ? *cfg : hm19_default_cfg();

    dev->lock = NULL;
    if (dev->cfg.flags & HM19_F_USE_MUTEX) {
        omnia_port_t* port = hm19_port();
        if (port && port->v && port->v->mutex_create) {
            dev->lock = port->v->mutex_create();
        } else {
            /* Disable mutex feature if not supported by the platform. */
            dev->cfg.flags &= ~HM19_F_USE_MUTEX;
        }
    }

    hm19_hw_reset(dev);

    /* Best-effort AT ping (non-fatal). */
    {
        char r[32] = {0};
        (void)hm19_send_at_raw_noeol(dev, "AT", r, sizeof(r), 200);
    }

    return OMNIA_OK;
}

/**
 * @brief Deinitialize the HM-19 driver instance.
 *
 * Releases the optional mutex using port hooks when available.
 *
 * @param dev HM-19 instance.
 */
void hm19_deinit(hm19_t* dev)
{
    if (!dev) return;

    omnia_port_t* port = hm19_port();
    if (dev->lock && port && port->v && port->v->mutex_destroy) {
        port->v->mutex_destroy(dev->lock);
    }
    dev->lock = NULL;
}

/** @brief Set AT line ending convention. */
void hm19_set_at_eol(hm19_t* dev, hm19_at_eol_t eol) { if (dev) dev->cfg.at_eol = eol; }

/** @brief Set EN pin polarity. */
void hm19_set_en_polarity(hm19_t* dev, hm19_en_polarity_t pol) { if (dev) dev->cfg.en_pol = pol; }

/** @brief Set boot delay in milliseconds. */
void hm19_set_boot_delay_ms(hm19_t* dev, uint32_t ms) { if (dev) dev->cfg.boot_delay_ms = ms; }

/** @brief Set configuration flags bitmask. */
void hm19_set_flags(hm19_t* dev, uint32_t flags) { if (dev) dev->cfg.flags = flags; }

/** @brief Set TX chunk size in bytes (used when chunked TX is enabled). */
void hm19_set_tx_chunk(hm19_t* dev, uint16_t chunk_size) { if (dev) dev->cfg.tx_chunk_size = chunk_size; }

/** @brief Set RX drain budget in milliseconds (used when drain-before-AT is enabled). */
void hm19_set_drain_budget(hm19_t* dev, uint32_t budget_ms) { if (dev) dev->cfg.drain_budget_ms = budget_ms; }

/**
 * @brief Send raw data over UART (DATA mode).
 *
 * This function is mutex-protected if enabled. It may also chunk the transfer
 * depending on configuration.
 *
 * @param dev        HM-19 instance.
 * @param data       Data buffer.
 * @param len        Buffer length.
 * @param timeout_ms Per-write timeout in milliseconds.
 * @return Status code.
 */
omnia_status_t hm19_send_raw(hm19_t* dev,
                            const uint8_t* data,
                            size_t len,
                            uint32_t timeout_ms)
{
    if (!dev || !dev->uart || !data || len == 0) return OMNIA_EINVAL;

    hm19_lock(dev);
    omnia_status_t st = hm19_uart_write_all_nolock(dev, data, len, timeout_ms);
    hm19_unlock(dev);

    return st;
}

/**
 * @brief Send a text line (appends configured EOL).
 *
 * @param dev        HM-19 instance.
 * @param line       Null-terminated line (without EOL).
 * @param timeout_ms Timeout in milliseconds.
 * @return Status code.
 */
omnia_status_t hm19_send_line(hm19_t* dev,
                              const char* line,
                              uint32_t timeout_ms)
{
    if (!dev || !line) return OMNIA_EINVAL;

    hm19_lock(dev);
    omnia_status_t st = hm19_send_line_nolock(dev, line, timeout_ms);
    hm19_unlock(dev);

    return st;
}

/**
 * @brief Send an AT command and read a single-line response.
 *
 * When enabled, RX is drained before issuing the command to avoid mixing
 * asynchronous data with the AT response.
 *
 * @param dev        HM-19 instance.
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
                            uint32_t timeout_ms)
{
    if (!dev || !dev->uart || !cmd) return OMNIA_EINVAL;

    hm19_lock(dev);

    hm19_drain_rx_nolock(dev);

    omnia_status_t st = hm19_send_line_nolock(dev, cmd, timeout_ms);
    if (st != OMNIA_OK) {
        hm19_unlock(dev);
        return st;
    }

    if (!resp || resp_size == 0) {
        hm19_unlock(dev);
        return OMNIA_OK;
    }

    st = hm19_read_line_nolock(dev, resp, resp_size, timeout_ms);

    hm19_unlock(dev);
    return st;
}

/* -------------------------------------------------------------------------- */
/* Internal AT helpers                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Send an AT command without appending EOL and read a single-line response.
 *
 * This is useful for modules/firmware variants that are sensitive to line endings
 * or when the command already includes terminators.
 *
 * @param dev        HM-19 instance.
 * @param cmd        AT command string (as-is).
 * @param resp       Optional response buffer (may be NULL).
 * @param resp_size  Response buffer size.
 * @param timeout_ms Timeout in milliseconds.
 * @return Status code.
 */
static omnia_status_t hm19_send_at_raw_noeol(hm19_t* dev,
                                            const char* cmd,
                                            char* resp,
                                            size_t resp_size,
                                            uint32_t timeout_ms)
{
    if (!dev || !dev->uart || !cmd) return OMNIA_EINVAL;

    hm19_lock(dev);
    hm19_drain_rx_nolock(dev);

    omnia_status_t st =
        hm19_uart_write_all_nolock(dev, (const uint8_t*)cmd, strlen(cmd), timeout_ms);
    if (st != OMNIA_OK) {
        hm19_unlock(dev);
        return st;
    }

    if (!resp || resp_size == 0) {
        hm19_unlock(dev);
        return OMNIA_OK;
    }

    st = hm19_read_line_nolock(dev, resp, resp_size, timeout_ms);
    hm19_unlock(dev);

    return st;
}

/**
 * @brief Send an AT command and capture multiple response lines.
 *
 * Lines are appended to @p out_buf separated by '\n'. Reading stops when
 * @p max_lines is reached, the buffer is full, or a read fails.
 *
 * @param dev        HM-19 instance.
 * @param cmd        AT command string (without EOL).
 * @param out_buf    Output buffer to store lines.
 * @param out_size   Output buffer size.
 * @param max_lines  Maximum number of lines to capture.
 * @param timeout_ms Timeout in milliseconds per line.
 * @return Number of lines captured on success, negative value on parameter/IO errors.
 */
int hm19_send_at_multi(hm19_t* dev,
                       const char* cmd,
                       char* out_buf,
                       size_t out_size,
                       uint8_t max_lines,
                       uint32_t timeout_ms)
{
    if (!dev || !dev->uart || !cmd || !out_buf || out_size == 0 || max_lines == 0) {
        return -1;
    }

    hm19_lock(dev);

    hm19_drain_rx_nolock(dev);

    omnia_status_t st = hm19_send_line_nolock(dev, cmd, timeout_ms);
    if (st != OMNIA_OK) {
        hm19_unlock(dev);
        return -2;
    }

    size_t out_idx = 0;
    uint8_t lines = 0;

    uint16_t tmp_cap = dev->cfg.max_at_line ? dev->cfg.max_at_line : 128;
    char tmp[512];

    if (tmp_cap > (uint16_t)sizeof(tmp)) tmp_cap = (uint16_t)sizeof(tmp);

    while (lines < max_lines) {
        tmp[0] = '\0';
        st = hm19_read_line_nolock(dev, tmp, tmp_cap, timeout_ms);
        if (st != OMNIA_OK) break;

        size_t L = strlen(tmp);
        if (L == 0) continue;

        if (out_idx + L + 2 > out_size) break;

        memcpy(out_buf + out_idx, tmp, L);
        out_idx += L;
        out_buf[out_idx++] = '\n';
        out_buf[out_idx] = '\0';

        lines++;
    }

    hm19_unlock(dev);
    return (int)lines;
}
