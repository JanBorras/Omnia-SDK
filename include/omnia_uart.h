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
 * @file omnia_uart.h
 * @brief UART abstraction helpers built on top of the Omnia port contract.
 *
 * This header provides a thin, inline-only wrapper around the UART-related
 * functions exposed by the active omnia_port_vtable_t.
 *
 * Design principles:
 * - UART is always treated as asynchronous (non-blocking).
 * - No direct dependency on vendor HAL or driver types.
 * - All hardware access is delegated through the Omnia port contract.
 *
 * The goal is to offer a uniform, portable UART API usable by:
 * - Drivers (e.g. BLE modules, sensors),
 * - Application-level services (streaming, logging),
 * without leaking platform-specific details.
 */

#ifndef OMNIA_UART_H
#define OMNIA_UART_H

#include "omnia_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* UART handle                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Logical descriptor for a UART channel.
 *
 * This structure represents a UART instance as seen by the SDK.
 * It does not own the hardware; it only references a platform-specific
 * opaque handle provided by the port implementation.
 */
typedef struct {
    omnia_uart_t dev;   /**< Opaque UART handle (e.g. UART_HandleTypeDef*) */
    uint32_t     baud;  /**< Configured baud rate (informative only) */
} omnia_uart_handle_t;

/* -------------------------------------------------------------------------- */
/* Write                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Asynchronously write data to a UART.
 *
 * This function never blocks. The exact behavior depends on the platform
 * implementation, but the contract guarantees:
 * - OMNIA_OK    : data accepted (queued or transmission started).
 * - OMNIA_EBUSY : transmitter busy / queue full (data not accepted).
 * - OMNIA_EIO   : hardware error.
 *
 * @param h     UART handle.
 * @param data  Pointer to data buffer.
 * @param len   Number of bytes to write.
 *
 * @return Status code as defined by the Omnia contract.
 */
static inline omnia_status_t
omnia_uart_write(const omnia_uart_handle_t* h,
                 const uint8_t* data,
                 size_t len)
{
    if (!h || !data || len == 0) {
        return OMNIA_EINVAL;
    }

    omnia_port_t* port = omnia_port_get();
    if (!port || !port->v || !port->v->uart_write) {
        return OMNIA_ENOTSUP;
    }

    return port->v->uart_write(h->dev, data, len);
}

/* -------------------------------------------------------------------------- */
/* Read                                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Asynchronously read data from a UART.
 *
 * This function never blocks. Typical semantics:
 * - OMNIA_OK    : one or more bytes read.
 * - OMNIA_EBUSY : no data available at the moment.
 * - OMNIA_EIO   : hardware error.
 *
 * @param h          UART handle.
 * @param data       Destination buffer.
 * @param len        Maximum number of bytes to read.
 * @param out_nread  Number of bytes actually read.
 *
 * @return Status code as defined by the Omnia contract.
 */
static inline omnia_status_t
omnia_uart_read(const omnia_uart_handle_t* h,
                uint8_t* data,
                size_t len,
                size_t* out_nread)
{
    if (!h || !data || len == 0 || !out_nread) {
        return OMNIA_EINVAL;
    }

    *out_nread = 0;

    omnia_port_t* port = omnia_port_get();
    if (!port || !port->v || !port->v->uart_read) {
        return OMNIA_ENOTSUP;
    }

    return port->v->uart_read(h->dev, data, len, out_nread);
}

/* -------------------------------------------------------------------------- */
/* Status helpers                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check whether the UART transmitter is busy.
 *
 * @param h UART handle.
 *
 * @return 1 if busy or unsupported, 0 if idle.
 */
static inline int
omnia_uart_tx_busy(const omnia_uart_handle_t* h)
{
    if (!h) {
        return 1;
    }

    omnia_port_t* port = omnia_port_get();
    if (!port || !port->v || !port->v->uart_tx_busy) {
        return 1;
    }

    return port->v->uart_tx_busy(h->dev);
}

/**
 * @brief Query how many bytes are available in the UART RX buffer.
 *
 * @param h UART handle.
 *
 * @return Number of bytes available, or 0 if unsupported.
 */
static inline size_t
omnia_uart_rx_available(const omnia_uart_handle_t* h)
{
    if (!h) {
        return 0;
    }

    omnia_port_t* port = omnia_port_get();
    if (!port || !port->v || !port->v->uart_rx_available) {
        return 0;
    }

    return port->v->uart_rx_available(h->dev);
}

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_UART_H */
