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
 * @file main.c
 * @brief Minimal example that binds a platform port and initializes an ST7735 LCD.
 *
 * This example demonstrates:
 * - Platform port registration and retrieval via the Omnia port contract.
 * - ST7735 driver initialization using the abstract port interface.
 *
 * @note Hardware handles/pins are platform-specific and must be provided by the user.
 */

#include "omnia_port.h"
#include "st7735.h"

/**
 * @brief Register the STM32 platform port implementation.
 *
 * This function is resolved at link time by selecting the STM32 port build.
 */
void omnia_port_stm32_init(void);

int main(void)
{
    /* Register platform port implementation and obtain the active port instance. */
    omnia_port_stm32_init();
    omnia_port_t *P = omnia_port_get();

    /**
     * @brief ST7735 display descriptor.
     *
     * The driver relies on:
     * - @ref omnia_port_t for timing/GPIO abstractions.
     * - A platform SPI handle for transfers.
     * - Control pins (CS/DC/RST) mapped to the platform GPIO identifiers.
     */
    st7735_t lcd = {
        .port = P,
        .spi  = /* handle SPI */,
        .pin_cs  = /* pin */,
        .pin_dc  = /* pin */,
        .pin_rst = /* pin */,
        .width = 128,
        .height = 160
    };

    /* Initialize display and clear screen (RGB565 = 0x0000 is black). */
    st7735_init(&lcd);
    st7735_fill(&lcd, 0x0000);

    /* Main loop. */
    for (;;) {
        /* Add application logic here. */
    }
}
