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
 * @file omnia_port.c
 * @brief Core implementation of the Omnia platform port contract.
 *
 * The Omnia Port provides a thin abstraction layer over platform-specific
 * hardware and OS services. Exactly one port implementation is expected
 * to be linked at build time and registered at runtime.
 *
 * Design principles:
 * - Single active port selected at link/build time.
 * - Minimal mandatory contract for portability.
 * - Optional capabilities exposed via feature-detection helpers.
 * - No dynamic allocation or global initialization order dependencies.
 */

#include "omnia_port.h"

/* -------------------------------------------------------------------------- */
/* Global port state                                                          */
/* -------------------------------------------------------------------------- */

/** Single active port instance. */
static omnia_port_t g_port;

/** Initialization guard. */
static int g_inited = 0;

/* -------------------------------------------------------------------------- */
/* Port validation and registration                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Validate a platform port vtable.
 *
 * The following functions are considered mandatory for a usable port:
 * - GPIO write and mode control (basic peripheral control).
 * - delay_us and millis (portable timing and synchronization).
 *
 * All other services (SPI, UART, ADC, concurrency, heap, logging) are optional
 * and may be queried at runtime via capability helpers.
 *
 * @param v Port vtable to validate.
 * @return OMNIA_OK if valid, error code otherwise.
 */
omnia_status_t omnia_port_validate(const omnia_port_vtable_t* v)
{
    if (!v) return OMNIA_EINVAL;

    if (!v->gpio_write) return OMNIA_EINVAL;
    if (!v->gpio_mode)  return OMNIA_EINVAL;
    if (!v->delay_us)   return OMNIA_EINVAL;
    if (!v->millis)     return OMNIA_EINVAL;

    return OMNIA_OK;
}

/**
 * @brief Register a platform port implementation.
 *
 * This function validates the provided vtable and installs it as the
 * active global port. Only one port is supported at runtime.
 *
 * @param v Port vtable to register.
 * @return OMNIA_OK on success, error code otherwise.
 */
omnia_status_t omnia_port_register(const omnia_port_vtable_t* v)
{
    if (!v) return OMNIA_EINVAL;

    omnia_status_t st = omnia_port_validate(v);
    if (st != OMNIA_OK) return st;

    g_port.v = v;
    g_inited = 1;
    return OMNIA_OK;
}

/**
 * @brief Get the active platform port.
 *
 * @return Pointer to the active port, or NULL if not registered.
 */
omnia_port_t* omnia_port_get(void)
{
    return g_inited ? &g_port : NULL;
}

/* -------------------------------------------------------------------------- */
/* Capability helpers (feature detection)                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get the active port vtable, if available.
 *
 * @return Pointer to port vtable, or NULL if no port is registered.
 */
static const omnia_port_vtable_t* port_v(void)
{
    omnia_port_t* p = omnia_port_get();
    return (p && p->v) ? p->v : NULL;
}

/**
 * @brief Check whether ADC support is available.
 *
 * ADC is considered present if at least one raw read function is provided.
 *
 * @return Non-zero if ADC is available, zero otherwise.
 */
int omnia_port_has_adc(void)
{
    const omnia_port_vtable_t* v = port_v();
    return (v && v->adc_read != NULL);
}

/**
 * @brief Check whether SPI support is available.
 *
 * SPI is considered present if a basic transmit function is provided.
 *
 * @return Non-zero if SPI is available, zero otherwise.
 */
int omnia_port_has_spi(void)
{
    const omnia_port_vtable_t* v = port_v();
    return (v && v->spi_tx != NULL);
}

/**
 * @brief Check whether UART support is available.
 *
 * UART is considered present only if both read and write operations exist,
 * following an asynchronous contract.
 *
 * @return Non-zero if UART is available, zero otherwise.
 */
int omnia_port_has_uart(void)
{
    const omnia_port_vtable_t* v = port_v();
    return (v &&
            v->uart_write != NULL &&
            v->uart_read  != NULL);
}

/**
 * @brief Check whether mutex-based concurrency support is available.
 *
 * Concurrency is considered present only if the full mutex lifecycle
 * is implemented.
 *
 * @return Non-zero if mutex support is available, zero otherwise.
 */
int omnia_port_has_mutex(void)
{
    const omnia_port_vtable_t* v = port_v();
    return (v &&
            v->mutex_create  != NULL &&
            v->mutex_lock    != NULL &&
            v->mutex_unlock  != NULL &&
            v->mutex_destroy != NULL);
}

/**
 * @brief Check whether dynamic memory allocation is available.
 *
 * Heap support is considered present only if both allocation and free
 * functions are provided.
 *
 * @return Non-zero if heap support is available, zero otherwise.
 */
int omnia_port_has_malloc(void)
{
    const omnia_port_vtable_t* v = port_v();
    return (v &&
            v->malloc_fn != NULL &&
            v->free_fn   != NULL);
}

/**
 * @brief Check whether logging support is available.
 *
 * @return Non-zero if logging is available, zero otherwise.
 */
int omnia_port_has_log(void)
{
    const omnia_port_vtable_t* v = port_v();
    return (v && v->log != NULL);
}
