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
 * @file omnia_port_ex.h
 * @brief Extended Omnia port descriptor with optional RTOS hooks and capabilities.
 *
 * This header defines an extended view of the Omnia port contract.
 * It complements the base omnia_port interface by allowing platforms to:
 * - Expose optional RTOS operations.
 * - Advertise platform capabilities via a bitmask.
 * - Attach an opaque user-defined context.
 *
 * The classic omnia_port API remains valid and unchanged; this extension
 * is purely additive and optional.
 */

#ifndef OMNIA_PORT_EX_H
#define OMNIA_PORT_EX_H

#include "omnia_port.h"
#include "omnia_rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Extended Omnia port descriptor.
 *
 * This structure aggregates:
 * - The standard I/O vtable (GPIO, SPI, UART, ADC, timing, etc.).
 * - Optional RTOS operations (mutexes, threads, queues, delays…).
 * - A capability bitmask describing platform features.
 * - An opaque user context pointer owned by the platform.
 *
 * Typical use cases:
 * - Feature detection without probing function pointers.
 * - Clean separation between bare-metal and RTOS-enabled ports.
 * - Passing platform-specific context without leaking HAL types.
 */
typedef struct {
    /**
     * @brief Base Omnia port vtable.
     *
     * This is the same vtable registered via omnia_port_register().
     * It must never be NULL.
     */
    const omnia_port_vtable_t* io;

    /**
     * @brief Optional RTOS operations table.
     *
     * May be NULL on bare-metal platforms.
     */
    const omnia_rtos_ops_t* rtos;

    /**
     * @brief Platform capability bitmask.
     *
     * Bit meaning is platform-defined, but typical examples include:
     * - bit 0: RTOS available
     * - bit 1: SPI DMA supported
     * - bit 2: ISR-safe UART TX
     */
    uint32_t caps;

    /**
     * @brief Optional user-defined context pointer.
     *
     * This pointer is not interpreted by the core and can be used by
     * platform code to attach arbitrary state.
     */
    void* user_ctx;
} omnia_port_ex_t;

/* -------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get the RTOS operations table associated with the active port.
 *
 * @param p Omnia port handle.
 * @return Pointer to RTOS operations, or NULL if not available.
 */
const omnia_rtos_ops_t* omnia_port_get_rtos(omnia_port_t* p);

/**
 * @brief Get the capability bitmask of the active port.
 *
 * @param p Omnia port handle.
 * @return Platform capability bitmask.
 */
uint32_t omnia_port_caps(omnia_port_t* p);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_PORT_EX_H */
