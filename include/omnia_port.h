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
 * @file omnia_port.h
 * @brief Omnia platform contract (public): portable I/O and services via vtable.
 *
 * This header defines the Omnia "port" abstraction: a stable interface
 * implemented by each platform (STM32, ESP32, MSP430, ...).
 *
 * Design goals:
 * - Keep applications independent from vendor HALs.
 * - Provide a minimal set of services required by the SDK core and drivers.
 * - Allow optional services (RTOS, heap, logging) without breaking portability.
 *
 * The contract is expressed as a function table (vtable). The core interacts
 * exclusively through this vtable, and never accesses platform internals.
 */

#ifndef OMNIA_PORT_H
#define OMNIA_PORT_H

#include "omnia_types.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Opaque platform handle types                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Opaque GPIO handle.
 *
 * The platform decides what this pointer represents:
 * - a numeric pin id casted to pointer,
 * - a pointer to a pin descriptor,
 * - a vendor HAL structure, etc.
 */
typedef void* omnia_gpio_t;

/** @brief Opaque SPI bus/handle. */
typedef void* omnia_spi_t;

/** @brief Opaque mutex handle (optional service). */
typedef void* omnia_mutex_t;

/** @brief Opaque ADC device/channel handle. */
typedef void* omnia_adc_t;

/** @brief Opaque UART device handle. */
typedef void* omnia_uart_t;

/* -------------------------------------------------------------------------- */
/* GPIO configuration                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief GPIO pull configuration.
 */
typedef enum {
    OMNIA_GPIO_PULL_NONE = 0, /**< No pull resistor. */
    OMNIA_GPIO_PULL_UP   = 1, /**< Pull-up enabled. */
    OMNIA_GPIO_PULL_DOWN = 2, /**< Pull-down enabled. */
} omnia_gpio_pull_t;

/* -------------------------------------------------------------------------- */
/* Port vtable contract                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief Platform contract vtable.
 *
 * This structure defines the services a platform may expose to the SDK.
 *
 * UART semantics (ASYNC ALWAYS):
 * - uart_write() must never block.
 *   - OMNIA_OK     : accepted/enqueued/sending
 *   - OMNIA_EBUSY  : cannot accept now (queue full / TX busy)
 *   - OMNIA_EIO    : hardware error
 *
 * - uart_read() must never block.
 *   - OMNIA_OK     : one or more bytes returned (out_nread > 0)
 *   - OMNIA_EBUSY  : no data available
 *   - OMNIA_EIO    : hardware error
 *
 * ADC semantics:
 * - adc_read()/adc_read_n() may be blocking (polling, DMA ring, etc.).
 *
 * Optional services:
 * - Concurrency: mutex_* may be NULL on bare-metal platforms.
 * - Memory: malloc_fn/free_fn may be NULL to enforce no-heap builds.
 * - Log: log may be NULL if no logging is available.
 */
typedef struct {
    /* ------------------------------ GPIO --------------------------------- */

    /**
     * @brief Write a logic level to a GPIO pin.
     *
     * @param pin   Opaque GPIO handle.
     * @param level Logic level (0 or 1).
     * @return Status code.
     */
    omnia_status_t (*gpio_write)(omnia_gpio_t pin, int level);

    /**
     * @brief Read a logic level from a GPIO pin.
     *
     * @param pin Opaque GPIO handle.
     * @return 0/1 on success, or negative value on error if implemented so.
     */
    int            (*gpio_read)(omnia_gpio_t pin);

    /**
     * @brief Configure GPIO direction and pull mode.
     *
     * @param pin       Opaque GPIO handle.
     * @param is_output Non-zero for output, 0 for input.
     * @param pull      Pull configuration.
     * @return Status code.
     */
    omnia_status_t (*gpio_mode)(omnia_gpio_t pin, int is_output, omnia_gpio_pull_t pull);

    /* ------------------------------- SPI --------------------------------- */

    /**
     * @brief Transmit bytes over SPI.
     *
     * @param bus SPI handle.
     * @param buf Data buffer.
     * @param n   Number of bytes.
     * @return Status code.
     */
    omnia_status_t (*spi_tx)(omnia_spi_t bus, const uint8_t* buf, size_t n);

    /**
     * @brief Transmit 16-bit words over SPI (optional).
     *
     * If NULL, drivers may fall back to spi_tx() with manual packing.
     *
     * @param bus SPI handle.
     * @param buf 16-bit buffer.
     * @param n   Number of 16-bit elements.
     * @return Status code.
     */
    omnia_status_t (*spi_tx16)(omnia_spi_t bus, const uint16_t* buf, size_t n);

    /**
     * @brief Transmit and receive bytes over SPI (optional).
     *
     * @param bus SPI handle.
     * @param tx  TX buffer (may be NULL if platform supports RX-only).
     * @param rx  RX buffer (may be NULL if platform supports TX-only).
     * @param n   Number of bytes.
     * @return Status code.
     */
    omnia_status_t (*spi_txrx)(omnia_spi_t bus, const uint8_t* tx, uint8_t* rx, size_t n);

    /* ------------------------------- ADC --------------------------------- */

    /**
     * @brief Read one ADC sample (RAW code).
     *
     * @param dev  ADC device/channel handle.
     * @param out  Output RAW sample.
     * @return Status code.
     */
    omnia_status_t (*adc_read)(omnia_adc_t dev, uint16_t* out);

    /**
     * @brief Read N ADC samples (RAW codes).
     *
     * May be implemented using polling or DMA mechanisms.
     *
     * @param dev ADC device/channel handle.
     * @param buf Output buffer.
     * @param n   Number of samples.
     * @return Status code.
     */
    omnia_status_t (*adc_read_n)(omnia_adc_t dev, uint16_t* buf, size_t n);

    /* ------------------------------- UART -------------------------------- */

    /**
     * @brief Asynchronous UART write (never blocks).
     *
     * @param uart UART handle.
     * @param data Data buffer.
     * @param len  Number of bytes.
     * @return Status code (see contract notes above).
     */
    omnia_status_t (*uart_write)(omnia_uart_t uart, const uint8_t* data, size_t len);

    /**
     * @brief Asynchronous UART read (never blocks).
     *
     * @param uart       UART handle.
     * @param data       Output buffer.
     * @param len        Max bytes to read.
     * @param out_nread  Bytes actually read.
     * @return Status code (see contract notes above).
     */
    omnia_status_t (*uart_read)(omnia_uart_t uart, uint8_t* data, size_t len, size_t* out_nread);

    /* ------------------------- UART status helpers ------------------------ */

    /**
     * @brief Check if UART TX is currently busy.
     *
     * @param uart UART handle.
     * @return 0 if not busy, 1 if busy (implementation-defined).
     */
    int            (*uart_tx_busy)(omnia_uart_t uart);

    /**
     * @brief Get number of bytes available in UART RX buffer.
     *
     * @param uart UART handle.
     * @return Number of available bytes.
     */
    size_t         (*uart_rx_available)(omnia_uart_t uart);

    /* -------------------------------- Time -------------------------------- */

    /**
     * @brief Busy-wait delay in microseconds.
     *
     * @param us Delay duration in microseconds.
     */
    void           (*delay_us)(uint32_t us);

    /**
     * @brief Millisecond monotonic clock.
     *
     * Used for coarse timing, timeouts and profiling fallback.
     *
     * @return Monotonic time in milliseconds.
     */
    uint64_t       (*millis)(void);

    /* -------------------------- Concurrency (optional) --------------------- */

    /**
     * @brief Create a mutex (optional).
     *
     * @return Opaque mutex handle, or NULL on failure.
     */
    omnia_mutex_t  (*mutex_create)(void);

    /** @brief Lock a mutex (optional). */
    void           (*mutex_lock)(omnia_mutex_t m);

    /** @brief Unlock a mutex (optional). */
    void           (*mutex_unlock)(omnia_mutex_t m);

    /** @brief Destroy a mutex (optional). */
    void           (*mutex_destroy)(omnia_mutex_t m);

    /* --------------------------- Memory (optional) ------------------------- */

    /**
     * @brief Allocate memory from platform heap (optional).
     *
     * Intended only for components that explicitly allow heap usage.
     */
    void*          (*malloc_fn)(size_t);

    /** @brief Free memory allocated by malloc_fn (optional). */
    void           (*free_fn)(void*);

    /* ---------------------------- Logging (optional) ----------------------- */

    /**
     * @brief Log a formatted message (optional).
     *
     * Level meaning is implementation-defined (e.g. 0=debug..3=error).
     */
    void           (*log)(int level, const char* fmt, ...);

    /* ---------------------- Platform-specific context ---------------------- */

    /**
     * @brief Opaque platform-specific context pointer.
     *
     * This can be used by the platform to attach extra state to the vtable.
     * The core does not interpret this pointer.
     */
    void*          user_ctx;

} omnia_port_vtable_t;

/* -------------------------------------------------------------------------- */
/* Runtime handle                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Runtime port handle.
 *
 * The active port is selected at build/link time and registered at runtime.
 * The core uses this handle to access the vtable.
 */
struct omnia_port_s {
    const omnia_port_vtable_t* v; /**< Bound vtable (must be valid after register). */
};

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Register the active platform vtable.
 *
 * This validates the vtable and stores it as the active port implementation.
 *
 * @param v Platform vtable.
 * @return Status code.
 */
omnia_status_t omnia_port_register(const omnia_port_vtable_t* v);

/**
 * @brief Validate a platform vtable against the minimum required contract.
 *
 * This function checks the subset of operations required for the SDK to run.
 * Optional operations are allowed to be NULL.
 *
 * @param v Platform vtable.
 * @return Status code.
 */
omnia_status_t omnia_port_validate(const omnia_port_vtable_t* v);

/**
 * @brief Get the active port handle.
 *
 * @return Pointer to active port, or NULL if not registered.
 */
omnia_port_t*  omnia_port_get(void);

/* -------------------------------------------------------------------------- */
/* Feature detection helpers                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Check if the active port provides ADC support.
 * @return Non-zero if ADC is available, 0 otherwise.
 */
int omnia_port_has_adc(void);

/**
 * @brief Check if the active port provides SPI support.
 * @return Non-zero if SPI is available, 0 otherwise.
 */
int omnia_port_has_spi(void);

/**
 * @brief Check if the active port provides UART support.
 * @return Non-zero if UART is available, 0 otherwise.
 */
int omnia_port_has_uart(void);

/**
 * @brief Check if the active port provides mutex support.
 * @return Non-zero if mutex API is available, 0 otherwise.
 */
int omnia_port_has_mutex(void);

/**
 * @brief Check if the active port provides heap allocation support.
 * @return Non-zero if malloc/free are available, 0 otherwise.
 */
int omnia_port_has_malloc(void);

/**
 * @brief Check if the active port provides logging support.
 * @return Non-zero if log() is available, 0 otherwise.
 */
int omnia_port_has_log(void);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_PORT_H */
