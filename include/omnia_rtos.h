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
 * @file omnia_rtos.h
 * @brief Optional RTOS abstraction layer for Omnia SDK.
 *
 * This header defines a *purely optional* RTOS contract that can be
 * attached to the Omnia platform via omnia_port_ex.
 *
 * Design principles:
 * - The core SDK does NOT require an RTOS.
 * - All pointers in this table may be NULL if the platform is bare-metal.
 * - No vendor RTOS headers (FreeRTOS, Zephyr, CMSIS-RTOS, etc.) leak outside
 *   the platform layer.
 *
 * This interface is intentionally minimal and generic:
 * - It covers only the primitives required by higher-level services
 *   (streaming, BLE drivers, async pipelines, etc.).
 * - ISR-awareness is explicit via in_isr().
 *
 * Ownership rules:
 * - All objects (tasks, semaphores, queues, timers) are owned by the platform.
 * - The SDK never assumes memory layout or lifetime beyond the opaque handles.
 */

#ifndef OMNIA_RTOS_H
#define OMNIA_RTOS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Opaque RTOS handle types                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Opaque task/thread handle.
 */
typedef void* omnia_task_t;

/**
 * @brief Opaque semaphore handle.
 */
typedef void* omnia_sem_t;

/**
 * @brief Opaque queue handle.
 */
typedef void* omnia_queue_t;

/**
 * @brief Opaque timer handle.
 */
typedef void* omnia_timer_t;

/* -------------------------------------------------------------------------- */
/* Callback types                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Task entry function.
 *
 * @param arg User-provided argument.
 */
typedef void (*omnia_task_fn)(void* arg);

/**
 * @brief Timer callback function.
 *
 * @param arg User-provided argument.
 */
typedef void (*omnia_timer_cb)(void* arg);

/* -------------------------------------------------------------------------- */
/* RTOS operations table                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief RTOS operations vtable.
 *
 * All fields are optional. If a platform does not support an RTOS,
 * this entire table may be NULL.
 *
 * The SDK must always check for NULL before calling any operation.
 */
typedef struct {
    /* -------------------------- Basic synchronization --------------------- */

    /**
     * @brief Create a mutex.
     *
     * @return Opaque mutex handle, or NULL on failure.
     */
    omnia_mutex_t (*mutex_create)(void);

    /**
     * @brief Lock a mutex.
     *
     * Must be safe to call from task context.
     */
    void          (*mutex_lock)(omnia_mutex_t);

    /**
     * @brief Unlock a mutex.
     */
    void          (*mutex_unlock)(omnia_mutex_t);

    /**
     * @brief Destroy a mutex.
     */
    void          (*mutex_destroy)(omnia_mutex_t);

    /* ------------------------------- Tasks -------------------------------- */

    /**
     * @brief Create a task/thread.
     *
     * @param fn    Task entry function.
     * @param name  Optional human-readable name (may be ignored).
     * @param stack Stack size in bytes (or words, RTOS-defined).
     * @param arg   Argument passed to task entry.
     * @param prio  Task priority (relative, RTOS-defined).
     *
     * @return Task handle, or NULL on failure.
     */
    omnia_task_t  (*task_create)(omnia_task_fn fn,
                                 const char* name,
                                 uint32_t stack,
                                 void* arg,
                                 int prio);

    /**
     * @brief Delete a task.
     *
     * If called on the current task, semantics are RTOS-defined.
     */
    void          (*task_delete)(omnia_task_t t);

    /**
     * @brief Yield execution to another task.
     */
    void          (*yield)(void);

    /**
     * @brief Sleep the current task for a number of milliseconds.
     *
     * @param ms Sleep duration in milliseconds.
     */
    void          (*sleep_ms)(uint32_t ms);

    /* -------------------------- Semaphores / Queues ------------------------ */

    /**
     * @brief Create a semaphore.
     *
     * @param initial Initial count.
     * @param max     Maximum count.
     * @return Semaphore handle, or NULL on failure.
     */
    omnia_sem_t   (*sem_create)(uint32_t initial, uint32_t max);

    /**
     * @brief Take (acquire) a semaphore.
     *
     * @param sem         Semaphore handle.
     * @param timeout_ms  Timeout in milliseconds.
     * @return true on success, false on timeout/failure.
     */
    bool          (*sem_take)(omnia_sem_t sem, uint32_t timeout_ms);

    /**
     * @brief Give (release) a semaphore.
     */
    void          (*sem_give)(omnia_sem_t sem);

    /**
     * @brief Destroy a semaphore.
     */
    void          (*sem_destroy)(omnia_sem_t sem);

    /**
     * @brief Create a message queue.
     *
     * @param item_size Size of each item in bytes.
     * @param len       Maximum number of items.
     * @return Queue handle, or NULL on failure.
     */
    omnia_queue_t (*queue_create)(size_t item_size, size_t len);

    /**
     * @brief Send an item to a queue.
     *
     * @param q           Queue handle.
     * @param item        Pointer to item to send.
     * @param timeout_ms  Timeout in milliseconds.
     * @return true on success, false on timeout/failure.
     */
    bool          (*queue_send)(omnia_queue_t q,
                                const void* item,
                                uint32_t timeout_ms);

    /**
     * @brief Receive an item from a queue.
     *
     * @param q           Queue handle.
     * @param item        Output buffer.
     * @param timeout_ms  Timeout in milliseconds.
     * @return true on success, false on timeout/failure.
     */
    bool          (*queue_recv)(omnia_queue_t q,
                                void* item,
                                uint32_t timeout_ms);

    /**
     * @brief Destroy a queue.
     */
    void          (*queue_destroy)(omnia_queue_t q);

    /* ------------------------------ Timers -------------------------------- */

    /**
     * @brief Create a high-level software timer.
     *
     * @param period_ms   Timer period in milliseconds.
     * @param auto_reload true for periodic timers, false for one-shot.
     * @param cb          Callback function.
     * @param arg         Argument passed to callback.
     *
     * @return Timer handle, or NULL on failure.
     */
    omnia_timer_t (*timer_create)(uint32_t period_ms,
                                  bool auto_reload,
                                  omnia_timer_cb cb,
                                  void* arg);

    /**
     * @brief Start a timer.
     *
     * @return true on success, false on failure.
     */
    bool          (*timer_start)(omnia_timer_t);

    /**
     * @brief Stop a timer.
     *
     * @return true on success, false on failure.
     */
    bool          (*timer_stop)(omnia_timer_t);

    /**
     * @brief Destroy a timer.
     */
    void          (*timer_destroy)(omnia_timer_t);

    /* ------------------------------ Context -------------------------------- */

    /**
     * @brief Check whether execution is currently in interrupt context.
     *
     * Useful to select FromISR variants when required by the RTOS.
     *
     * @return true if in ISR context, false otherwise.
     */
    bool          (*in_isr)(void);

} omnia_rtos_ops_t;

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_RTOS_H */
