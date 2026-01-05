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
 * @file omnia_types.h
 * @brief Core types and status codes shared across the Omnia SDK.
 *
 * This header defines the most fundamental, dependency-free building blocks
 * used throughout the entire Omnia architecture:
 *
 * - Unified status/error codes.
 * - Lightweight helpers for result checking.
 * - Explicit boolean type for portability.
 * - Forward declarations for opaque handles.
 *
 * Design goals:
 * - No dependency on platform headers.
 * - Stable ABI: changes here are *architectural* decisions.
 * - Readability in higher-level code and documentation (TFM-friendly).
 */

#ifndef OMNIA_TYPES_H
#define OMNIA_TYPES_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Status / error codes                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Unified status codes used across the Omnia SDK.
 *
 * These values are intentionally minimal and platform-agnostic.
 * They form the common error vocabulary between:
 * - Platform ports
 * - Drivers
 * - Middleware
 * - AI runtime
 *
 * Mapping to vendor-specific errors (HAL, RTOS, etc.) must be done
 * at the platform boundary.
 */
typedef enum {
    OMNIA_OK = 0,        /**< Operation completed successfully */
    OMNIA_EINVAL,        /**< Invalid argument or NULL pointer */
    OMNIA_EIO,           /**< I/O or hardware error */
    OMNIA_EBUSY,         /**< Resource busy / operation not accepted */
    OMNIA_ETIMEDOUT,     /**< Operation timed out */
    OMNIA_ENOMEM,        /**< Out of memory */
    OMNIA_ENOTSUP,       /**< Operation not supported on this platform */
} omnia_status_t;

/* -------------------------------------------------------------------------- */
/* Status helpers                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check whether a status represents success.
 *
 * @param s Status code.
 * @return Non-zero if status is OMNIA_OK, zero otherwise.
 */
static inline int omnia_succeeded(omnia_status_t s)
{
    return s == OMNIA_OK;
}

/**
 * @brief Check whether a status represents failure.
 *
 * @param s Status code.
 * @return Non-zero if status is not OMNIA_OK, zero otherwise.
 */
static inline int omnia_failed(omnia_status_t s)
{
    return s != OMNIA_OK;
}

/*
 * Alternative macro-based helpers (same semantics, less debugger-friendly):
 *
 * #define OMNIA_SUCCEEDED(s) ((s) == OMNIA_OK)
 * #define OMNIA_FAILED(s)    ((s) != OMNIA_OK)
 */

/* -------------------------------------------------------------------------- */
/* Explicit boolean type                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Explicit boolean type for portability and clarity.
 *
 * This avoids relying on <stdbool.h> in low-level or C89-style code paths
 * and makes intent explicit in APIs and documentation.
 */
typedef enum {
    OMNIA_FALSE = 0,
    OMNIA_TRUE  = 1
} omnia_bool_t;

/* -------------------------------------------------------------------------- */
/* Opaque forward declarations                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Opaque handle to the active platform port.
 *
 * The actual structure is private to the core implementation.
 * Applications and drivers must treat this as an opaque token.
 */
typedef struct omnia_port_s omnia_port_t;

/*
 * Optional future extension: SDK versioning for diagnostics.
 *
 * #define OMNIA_SDK_VERSION_MAJOR 0
 * #define OMNIA_SDK_VERSION_MINOR 1
 * #define OMNIA_SDK_VERSION_PATCH 0
 */

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_TYPES_H */
