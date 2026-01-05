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
 * @file omnia_port_stm32.h
 * @brief STM32 platform port for the Omnia SDK.
 *
 * This header exposes the STM32-specific entry points required to bind
 * a concrete STM32 HAL implementation to the generic Omnia port contract.
 *
 * Key ideas:
 * - The Omnia core and drivers never include STM32 HAL headers directly.
 * - All STM32-specific details live behind this port boundary.
 * - Initialization is explicit and must be performed at boot time.
 *
 * This file is platform-specific, but still part of the public SDK API
 * for STM32-based targets.
 */

#ifndef OMNIA_PORT_STM32_H
#define OMNIA_PORT_STM32_H

#include <stdint.h>
#include "omnia_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the STM32 Omnia port and register its vtable.
 *
 * This function must be called exactly once during system startup,
 * before using any driver or service that relies on the Omnia port
 * abstraction (GPIO, SPI, UART, ADC, timing, etc.).
 *
 * Typical usage:
 * @code
 * int main(void)
 * {
 *     HAL_Init();
 *     SystemClock_Config();
 *
 *     omnia_port_stm32_init();
 *     // Now it is safe to initialize drivers (ST7735, HM-19, etc.)
 * }
 * @endcode
 *
 * Internally, this function:
 * - Builds a static omnia_port_vtable_t backed by STM32 HAL calls.
 * - Validates the vtable against the Omnia contract.
 * - Registers it as the active platform port.
 */
void omnia_port_stm32_init(void);

/* -------------------------------------------------------------------------- */
/* GPIO helpers                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief Pack an STM32 GPIO port and pin into an opaque omnia_gpio_t.
 *
 * This helper allows application and driver code to remain completely
 * independent from STM32 HAL types.
 *
 * The returned omnia_gpio_t is opaque and must only be interpreted by
 * the STM32 port implementation.
 *
 * Example usage (STM32 main.c):
 * @code
 * screen.cs  = omnia_mkpin(GPIOB, GPIO_PIN_9);
 * screen.dc  = omnia_mkpin(GPIOB, GPIO_PIN_7);
 * screen.rst = omnia_mkpin(GPIOB, GPIO_PIN_8);
 * screen.bl  = omnia_mkpin(GPIOB, GPIO_PIN_6);
 * @endcode
 *
 * @param port Pointer to the GPIO port (typically GPIO_TypeDef*).
 * @param pin  GPIO pin number (HAL definition, e.g. GPIO_PIN_x).
 *
 * @return Opaque GPIO handle usable by the Omnia port API.
 */
omnia_gpio_t omnia_mkpin(void* port, uint16_t pin);

/* -------------------------------------------------------------------------- */
/* ADC helpers                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Start ADC1 DMA ring acquisition on STM32.
 *
 * This is a platform-specific helper intended for applications that
 * use continuous ADC sampling via DMA (e.g. EMG signal acquisition).
 *
 * The exact behavior (buffer size, circular mode, interrupts) is
 * defined by the STM32 port implementation and CubeMX configuration.
 *
 * @return OMNIA_OK on success, or an error code if ADC/DMA startup fails.
 */
omnia_status_t omnia_port_stm32_adc1_dma_start(void);

#ifdef __cplusplus
}
#endif

#endif /* OMNIA_PORT_STM32_H */
