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
 * @file st7735.h
 * @brief Minimal ST7735 display driver public API (RGB565 + 5x8 font).
 *
 * This header defines:
 * - ST7735 command constants and MADCTL flags.
 * - A small RGB565 color helper and a set of common color constants.
 * - The @ref ST7735_t device descriptor, which binds the driver to the Omnia
 *   platform contract (GPIO/SPI/timing) through @ref omnia_port_t.
 * - A lightweight drawing API (pixels, lines, rectangles, circles, triangles)
 *   and simple text rendering using a fixed 5x8 bitmap font.
 *
 * Design notes:
 * - The driver is intentionally allocation-free. All temporary buffers live
 *   inside @ref ST7735_t (e.g. @ref ST7735_t::fill_buf).
 * - The driver assumes the SPI peripheral is configured by the platform.
 * - Pins are expressed using Omnia GPIO identifiers; no vendor HAL types leak
 *   through this interface.
 */

#ifndef ST7735_H
#define ST7735_H

#include <stdint.h>

#include "fonts.h"
#include "omnia_port.h"

/* -------------------------------------------------------------------------- */
/* RGB565 utilities                                                           */
/* -------------------------------------------------------------------------- */

/** @name RGB565 common colors
 *  @{
 */
#define BLUE     0x001F
#define WHITE    0xFFFF
#define BLACK    0x0000
#define RED      0xF800
#define GREEN    0x07E0
#define YELLOW   0xFFE0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
/** @} */

/**
 * @brief Convert 8-bit RGB (0..255) to RGB565.
 *
 * @param r Red   component (0..255).
 * @param g Green component (0..255).
 * @param b Blue  component (0..255).
 * @return Packed RGB565 value (5/6/5).
 */
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t r5 = (uint16_t)(r >> 3) & 0x1Fu;
  uint16_t g6 = (uint16_t)(g >> 2) & 0x3Fu;
  uint16_t b5 = (uint16_t)(b >> 3) & 0x1Fu;
  return (uint16_t)((r5 << 11) | (g6 << 5) | b5);
}

/* -------------------------------------------------------------------------- */
/* ST7735 command set                                                         */
/* -------------------------------------------------------------------------- */

/** @brief Command table delay marker (used by some init sequences). */
#define DELAY                 0x80

#define NOP                   0x00
#define SWRESET               0x01
#define RDDID                 0x04
#define RDDST                 0x09

#define SLPIN                 0x10
#define SLPOUT                0x11
#define PTLON                 0x12
#define NORON                 0x13

#define INVOFF                0x20
#define INVON                 0x21
#define DISPOFF               0x28
#define DISPON                0x29
#define RAMRD                 0x2E
#define CASET                 0x2A
#define RASET                 0x2B
#define RAMWR                 0x2C

#define PTLAR                 0x30
#define MADCTL                0x36
#define COLMOD                0x3A

#define FRMCTR1               0xB1
#define FRMCTR2               0xB2
#define FRMCTR3               0xB3
#define INVCTR                0xB4
#define DISSET5               0xB6

#define PWCTR1                0xC0
#define PWCTR2                0xC1
#define PWCTR3                0xC2
#define PWCTR4                0xC3
#define PWCTR5                0xC4
#define VMCTR1                0xC5

#define RDID1                 0xDA
#define RDID2                 0xDB
#define RDID3                 0xDC
#define RDID4                 0xDD

#define GMCTRP1               0xE0
#define GMCTRN1               0xE1

#define PWCTR6                0xFC

/* -------------------------------------------------------------------------- */
/* MADCTL flags                                                               */
/* -------------------------------------------------------------------------- */

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08

/* -------------------------------------------------------------------------- */
/* Font geometry                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Font width in columns (pixels).
 *
 * Kept for compatibility with existing code, but the canonical value is
 * @ref CHARS_COLS_LENGTH from fonts.h.
 */
#define CHARS_COLS_LENGTH     5

/** @brief Font height in rows (pixels). */
#define CHARS_ROWS_LENGTH     8

/* -------------------------------------------------------------------------- */
/* Internal block fill buffer                                                  */
/* -------------------------------------------------------------------------- */

/** @brief Number of RGB565 pixels per fill block transfer. */
#define ST7735_FILL_BLOCK 64

/* -------------------------------------------------------------------------- */
/* Device descriptor                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief ST7735 device descriptor.
 *
 * This structure binds the driver to a platform implementation:
 * - @ref port provides GPIO/SPI/timing callbacks via the Omnia port contract.
 * - @ref spi is an opaque SPI handle owned by the platform.
 * - Control pins are expressed using Omnia GPIO identifiers.
 *
 * @note Some pins (e.g. @ref bl) may be optional depending on the board.
 */
typedef struct {
  /** Platform contract instance (must be registered before use). */
  omnia_port_t* port;

  /** Opaque SPI handle used by the platform SPI implementation. */
  omnia_spi_t   spi;

  /** Chip select pin. */
  omnia_gpio_t  cs;

  /** Data/command select pin. */
  omnia_gpio_t  dc;

  /** Reset pin (optional: may be 0/NULL depending on your gpio type). */
  omnia_gpio_t  rst;

  /** Backlight enable pin (optional). */
  omnia_gpio_t  bl;

  /** @deprecated Legacy compatibility fields (not required by the driver). */
  omnia_gpio_t  sclk;

  /** @deprecated Legacy compatibility fields (not required by the driver). */
  omnia_gpio_t  simo;

  /** Display width in pixels. */
  uint16_t      width;

  /** Display height in pixels. */
  uint16_t      height;

  /** Cached address window (driver-internal). */
  uint16_t      x0, y0, x1, y1;

  /** If non-zero, rotate display output by 180 degrees. */
  uint8_t       rotate_180;

  /** Internal pixel buffer used for large fill operations (no heap). */
  uint16_t      fill_buf[ST7735_FILL_BLOCK];
} ST7735_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the display (pins + reset + basic controller setup).
 *
 * The platform must provide a valid @ref omnia_port_t and SPI implementation.
 * The SPI peripheral configuration (mode/clock) is assumed to be handled
 * by the platform.
 *
 * @param screen Display descriptor.
 */
void st7735_init(ST7735_t *screen);

/**
 * @brief Perform a hardware reset sequence (if reset pin is available).
 *
 * @param screen Display descriptor.
 */
void st7735_reset(ST7735_t *screen);

/**
 * @brief Send a single ST7735 command byte.
 *
 * @param screen Display descriptor.
 * @param cmd Command byte.
 */
void st7735_send_command(ST7735_t *screen, uint8_t cmd);

/**
 * @brief Send a single data byte.
 *
 * @param screen Display descriptor.
 * @param data Data byte.
 */
void st7735_send_data(ST7735_t *screen, uint8_t data);

/**
 * @brief Send a single 16-bit RGB565 data word.
 *
 * @param screen Display descriptor.
 * @param data RGB565 pixel.
 */
void st7735_send_16bits_data(ST7735_t *screen, uint16_t data);

/**
 * @brief Set the active drawing window (inclusive coordinates).
 *
 * @param screen Display descriptor.
 * @param x0 Left coordinate.
 * @param y0 Top coordinate.
 * @param x1 Right coordinate.
 * @param y1 Bottom coordinate.
 */
void st7735_set_address_window(ST7735_t *screen,
                               uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1);

/**
 * @brief Draw a single pixel.
 *
 * Bounds are checked in the implementation.
 *
 * @param screen Display descriptor.
 * @param x X coordinate.
 * @param y Y coordinate.
 * @param color RGB565 pixel color.
 */
void st7735_draw_pixel(ST7735_t *screen,
                       uint16_t x, uint16_t y, uint16_t color);

/**
 * @brief Draw a single character using the built-in 5x8 font.
 *
 * @param screen Display descriptor.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param c ASCII character to render.
 * @param color RGB565 color.
 * @param size Pixel scale factor (1 = native).
 */
void st7735_draw_char(ST7735_t *screen,
                      uint16_t x, uint16_t y,
                      char c, uint16_t color, uint8_t size);

/**
 * @brief Draw a null-terminated ASCII string.
 *
 * Supports newline (`\n`) in the implementation.
 *
 * @param screen Display descriptor.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param str Null-terminated string.
 * @param color RGB565 color.
 * @param size Pixel scale factor.
 */
void st7735_draw_string(ST7735_t *screen,
                        uint16_t x, uint16_t y,
                        const char* str, uint16_t color, uint8_t size);

/**
 * @brief Fill the entire screen with a single color.
 *
 * @param screen Display descriptor.
 * @param color RGB565 fill color.
 */
void st7735_fill_screen(ST7735_t *screen, uint16_t color);

/**
 * @brief Draw a line (Bresenham-style).
 *
 * @param screen Display descriptor.
 * @param x0 Start X.
 * @param y0 Start Y.
 * @param x1 End X.
 * @param y1 End Y.
 * @param color RGB565 color.
 */
void st7735_draw_line(ST7735_t *screen,
                      int16_t x0, int16_t y0,
                      int16_t x1, int16_t y1,
                      uint16_t color);

/**
 * @brief Draw a rectangle outline.
 *
 * @param screen Display descriptor.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param w Width in pixels.
 * @param h Height in pixels.
 * @param color RGB565 color.
 */
void st7735_draw_rectangle(ST7735_t *screen,
                           uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief Draw a filled rectangle.
 *
 * @param screen Display descriptor.
 * @param x Left coordinate.
 * @param y Top coordinate.
 * @param w Width in pixels.
 * @param h Height in pixels.
 * @param color RGB565 color.
 */
void st7735_filled_rectangle(ST7735_t *screen,
                             uint16_t x, uint16_t y,
                             uint16_t w, uint16_t h, uint16_t color);

/**
 * @brief Draw a circle outline.
 *
 * @param screen Display descriptor.
 * @param x0 Center X.
 * @param y0 Center Y.
 * @param r Radius.
 * @param color RGB565 color.
 */
void st7735_draw_circle(ST7735_t *screen,
                        int16_t x0, int16_t y0, int16_t r, uint16_t color);

/**
 * @brief Draw a filled circle.
 *
 * @param screen Display descriptor.
 * @param x0 Center X.
 * @param y0 Center Y.
 * @param r Radius.
 * @param color RGB565 color.
 */
void st7735_filled_circle(ST7735_t *screen,
                          int16_t x0, int16_t y0, int16_t r, uint16_t color);

/**
 * @brief Draw a triangle outline.
 *
 * @param screen Display descriptor.
 * @param x0 Vertex 0 X.
 * @param y0 Vertex 0 Y.
 * @param x1 Vertex 1 X.
 * @param y1 Vertex 1 Y.
 * @param x2 Vertex 2 X.
 * @param y2 Vertex 2 Y.
 * @param color RGB565 color.
 */
void st7735_draw_triangle(ST7735_t *screen,
                          int16_t x0, int16_t y0,
                          int16_t x1, int16_t y1,
                          int16_t x2, int16_t y2,
                          uint16_t color);

/**
 * @brief Draw a filled triangle.
 *
 * @param screen Display descriptor.
 * @param x0 Vertex 0 X.
 * @param y0 Vertex 0 Y.
 * @param x1 Vertex 1 X.
 * @param y1 Vertex 1 Y.
 * @param x2 Vertex 2 X.
 * @param y2 Vertex 2 Y.
 * @param color RGB565 color.
 */
void st7735_filled_triangle(ST7735_t *screen,
                            int16_t x0, int16_t y0,
                            int16_t x1, int16_t y1,
                            int16_t x2, int16_t y2,
                            uint16_t color);

#endif /* ST7735_H */
