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
 * @file omnia_port_ex.c
 * @brief Extended Omnia port registration (optional capabilities and RTOS hooks).
 *
 * This module provides an optional extension layer on top of the core
 * Omnia port contract. It allows platforms to expose:
 * - RTOS-specific operations (mutexes, threads, delays, etc.).
 * - A compact capability bitmask.
 * - A user-defined opaque context pointer.
 *
 * The classic Omnia port API remains fully supported and is used as the
 * compatibility path.
 */

#include "omnia_port_ex.h"

/* -------------------------------------------------------------------------- */
/* Global extended port state                                                  */
/* -------------------------------------------------------------------------- */

/** Extended port descriptor (single instance). */
static omnia_port_ex_t g_port_ex;

/* -------------------------------------------------------------------------- */
/* Registration                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Register an extended Omnia port.
 *
 * This function augments the classic Omnia port registration with optional
 * RTOS operations, a capability bitmask and a user-defined context pointer.
 *
 * The underlying I/O vtable is still registered through
 * @ref omnia_port_register to preserve backward compatibility.
 *
 * @param io        Core I/O port vtable (mandatory).
 * @param rtos      Optional RTOS operations table (may be NULL).
 * @param caps      Capability bitmask describing platform features.
 * @param user_ctx  User-defined opaque pointer associated with the port.
 * @return OMNIA_OK on success, error code otherwise.
 */
omnia_status_t omnia_port_register_ex(const omnia_port_vtable_t* io,
                                      const omnia_rtos_ops_t* rtos,
                                      uint32_t caps,
                                      void* user_ctx)
{
    if (!io) return OMNIA_EINVAL;

    g_port_ex.io       = io;
    g_port_ex.rtos     = rtos;    /* optional */
    g_port_ex.caps     = caps;
    g_port_ex.user_ctx = user_ctx;

    /* Preserve legacy registration path for compatibility. */
    return omnia_port_register(io);
}

/* -------------------------------------------------------------------------- */
/* Accessors                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get RTOS operations associated with the active port.
 *
 * @param p Active port instance (currently unused).
 * @return Pointer to RTOS operations, or NULL if not provided.
 */
const omnia_rtos_ops_t* omnia_port_get_rtos(omnia_port_t* p)
{
    (void)p;
    return g_port_ex.rtos;
}

/**
 * @brief Get capability bitmask associated with the active port.
 *
 * @param p Active port instance (currently unused).
 * @return Capability bitmask.
 */
uint32_t omnia_port_caps(omnia_port_t* p)
{
    (void)p;
    return g_port_ex.caps;
}
