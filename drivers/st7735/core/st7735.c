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
 * @file st7735.c
 * @brief ST7735 display driver (SPI) implemented over the Omnia Port contract.
 *
 * This driver is hardware-agnostic: it performs all I/O through the Omnia Port
 * vtable (GPIO, SPI, delays). The display instance (ST7735_t) carries:
 * - A bound Omnia port instance
 * - An opaque SPI handle (owned by the platform)
 * - GPIO pins used for control (CS/DC/RST/BL)
 *
 * Design constraints:
 * - No dynamic allocation.
 * - Predictable behavior on resource-constrained MCUs.
 * - Optional fast path for 16-bit SPI transfers when the port provides it.
 *
 * Notes on robustness:
 * - OMNIA_ASSERT is used to enforce invariants during bring-up.
 * - Production builds may redefine OMNIA_ASSERT to avoid traps.
 */

#include "st7735.h"
#include <stddef.h>

#include "omnia_port.h" /* for omnia_port_has_spi() */

/* -------------------------------------------------------------------------- */
/* Debug trap and assertions                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Platform-specific trap used by OMNIA_ASSERT.
 *
 * On ARM targets this triggers a breakpoint instruction. On other targets
 * it falls back to abort(). Users can override OMNIA_ASSERT at build time.
 */
#if defined(__arm__) || defined(__thumb__)
  #define OMNIA_TRAP() __asm volatile("bkpt 15")
#else
  #include <stdlib.h>
  #define OMNIA_TRAP() abort()
#endif

#ifndef OMNIA_ASSERT
/** @brief Assertion macro used by the driver to enforce runtime invariants. */
#define OMNIA_ASSERT(expr) do { if (!(expr)) OMNIA_TRAP(); } while (0)
#endif

/* -------------------------------------------------------------------------- */
/* Port binding helpers                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Get the I/O vtable from a display instance.
 *
 * This is the central binding point: all hardware operations go through
 * the port vtable. The driver assumes @ref ST7735_t::port was registered
 * and validated by the platform layer.
 *
 * @param d Display instance.
 * @return Port vtable pointer.
 */
static inline const omnia_port_vtable_t* io(ST7735_t* d)
{
    OMNIA_ASSERT(d != NULL);
    OMNIA_ASSERT(d->port != NULL);
    OMNIA_ASSERT(d->port->v != NULL);
    return d->port->v;
}

/* -------------------------------------------------------------------------- */
/* GPIO helpers (control signals)                                              */
/* -------------------------------------------------------------------------- */

static inline void CS_LOW(ST7735_t* d)
{
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(d->cs != NULL);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->cs, 0);
}

static inline void CS_HIGH(ST7735_t* d)
{
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(d->cs != NULL);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->cs, 1);
}

static inline void DC_CMD(ST7735_t* d)
{
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(d->dc != NULL);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->dc, 0);
}

static inline void DC_DATA(ST7735_t* d)
{
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(d->dc != NULL);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->dc, 1);
}

static inline void RST_LOW(ST7735_t* d)
{
    if (!d->rst) return;
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->rst, 0);
}

static inline void RST_HIGH(ST7735_t* d)
{
    if (!d->rst) return;
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->rst, 1);
}

static inline void BL_ON(ST7735_t* d)
{
    if (!d->bl) return;
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->bl, 1);
}

static inline void BL_OFF(ST7735_t* d)
{
    if (!d->bl) return;
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(v->gpio_write != NULL);
    v->gpio_write(d->bl, 0);
}

/* -------------------------------------------------------------------------- */
/* Timing and SPI helpers                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Busy-wait delay in milliseconds implemented via delay_us().
 *
 * @param d  Display instance.
 * @param ms Delay in milliseconds.
 */
static inline void delay_ms(ST7735_t* d, uint32_t ms)
{
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(v->delay_us != NULL);
    while (ms--) {
        v->delay_us(1000);
    }
}

/**
 * @brief Transmit a buffer as bytes over SPI.
 *
 * @param d   Display instance.
 * @param buf Byte buffer.
 * @param n   Number of bytes.
 */
static inline void spi_tx8(ST7735_t* d, const uint8_t* buf, size_t n)
{
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(v->spi_tx != NULL);
    v->spi_tx(d->spi, buf, n);
}

/**
 * @brief Transmit a buffer as 16-bit words over SPI (RGB565).
 *
 * If the platform provides a native 16-bit transfer hook, it is used.
 * Otherwise, the function falls back to serializing each word into two bytes.
 *
 * @param d   Display instance.
 * @param buf Word buffer.
 * @param n   Number of 16-bit words.
 */
static inline void spi_tx16(ST7735_t* d, const uint16_t* buf, size_t n)
{
    const omnia_port_vtable_t* v = io(d);
    if (v->spi_tx16) {
        v->spi_tx16(d->spi, buf, n);
    } else {
        OMNIA_ASSERT(v->spi_tx != NULL);
        for (size_t i = 0; i < n; i++) {
            uint8_t b[2] = { (uint8_t)(buf[i] >> 8), (uint8_t)buf[i] };
            v->spi_tx(d->spi, b, 2);
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Pin initialization                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configure ST7735 control pins as outputs and set initial levels.
 *
 * This function uses the Omnia GPIO contract to:
 * - configure CS/DC as mandatory outputs
 * - configure RST/BL if provided
 * - set known safe initial logic levels
 *
 * @param lcd Display instance.
 */
static void st7735_pins_init(ST7735_t* lcd)
{
    omnia_port_t* p = lcd->port;
    OMNIA_ASSERT(p && p->v && p->v->gpio_mode);

    /* Configure outputs (no pull). */
    p->v->gpio_mode(lcd->cs,  1, OMNIA_GPIO_PULL_NONE);
    p->v->gpio_mode(lcd->dc,  1, OMNIA_GPIO_PULL_NONE);
    if (lcd->rst) p->v->gpio_mode(lcd->rst, 1, OMNIA_GPIO_PULL_NONE);
    if (lcd->bl)  p->v->gpio_mode(lcd->bl,  1, OMNIA_GPIO_PULL_NONE);

    /* Initial logic levels. */
    CS_HIGH(lcd);
    DC_CMD(lcd);
    if (lcd->rst) RST_HIGH(lcd);
    if (lcd->bl)  BL_OFF(lcd);
}

/* -------------------------------------------------------------------------- */
/* Low-level primitives                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Send a single ST7735 command byte.
 *
 * @param d   Display instance.
 * @param cmd Command opcode.
 */
void st7735_send_command(ST7735_t* d, uint8_t cmd)
{
    DC_CMD(d);
    CS_LOW(d);
    spi_tx8(d, &cmd, 1);
    CS_HIGH(d);
}

/**
 * @brief Send a single data byte.
 *
 * @param d    Display instance.
 * @param data Data byte.
 */
void st7735_send_data(ST7735_t* d, uint8_t data)
{
    DC_DATA(d);
    CS_LOW(d);
    spi_tx8(d, &data, 1);
    CS_HIGH(d);
}

/**
 * @brief Send a single 16-bit data word (RGB565).
 *
 * @param d    Display instance.
 * @param data 16-bit data word.
 */
void st7735_send_16bits_data(ST7735_t* d, uint16_t data)
{
    DC_DATA(d);
    CS_LOW(d);
    spi_tx16(d, &data, 1);
    CS_HIGH(d);
}

/**
 * @brief Perform a hardware reset sequence using the optional RST pin.
 *
 * If the instance has no RST pin, the function becomes a no-op at the GPIO level,
 * but still enforces the timing dependency.
 *
 * @param d Display instance.
 */
void st7735_reset(ST7735_t* d)
{
    const omnia_port_vtable_t* v = io(d);
    OMNIA_ASSERT(v->delay_us != NULL);

    RST_LOW(d);
    v->delay_us(10u * 1000u);
    RST_HIGH(d);
    v->delay_us(10u * 1000u);
}

/* -------------------------------------------------------------------------- */
/* Addressing                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Set the active address window (CASET/RASET) and prepare for RAM writes.
 *
 * This function updates the cached window in the instance and programs the
 * controller for subsequent RAMWR writes.
 *
 * @param d   Display instance.
 * @param x0  Start column.
 * @param y0  Start row.
 * @param x1  End column (inclusive).
 * @param y1  End row (inclusive).
 */
void st7735_set_address_window(ST7735_t* d,
                               uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1)
{
    d->x0 = x0; d->y0 = y0; d->x1 = x1; d->y1 = y1;

    uint8_t data[4];

    /* Column (CASET) */
    st7735_send_command(d, CASET);
    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)(x0 & 0xFFu);
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)(x1 & 0xFFu);
    DC_DATA(d);
    CS_LOW(d);
    spi_tx8(d, data, 4);
    CS_HIGH(d);

    /* Row (RASET) */
    st7735_send_command(d, RASET);
    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)(y0 & 0xFFu);
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)(y1 & 0xFFu);
    DC_DATA(d);
    CS_LOW(d);
    spi_tx8(d, data, 4);
    CS_HIGH(d);

    /* Prepare RAM write */
    st7735_send_command(d, RAMWR);
}

/* -------------------------------------------------------------------------- */
/* Pixel and primitives                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Draw a single pixel.
 *
 * Out-of-bounds writes are ignored.
 *
 * @param d      Display instance.
 * @param x      Column coordinate.
 * @param y      Row coordinate.
 * @param color  RGB565 color.
 */
void st7735_draw_pixel(ST7735_t* d, uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= d->width || y >= d->height) return;

    st7735_set_address_window(d, x, y, x, y);
    st7735_send_16bits_data(d, color);
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the ST7735 controller and clear the screen.
 *
 * Preconditions:
 * - The Omnia port must be registered and provide SPI support.
 * - The instance must carry valid SPI handle and control pins.
 * - The fill buffer must be provided by the caller (no heap allocation).
 *
 * This function:
 * - Configures control pins.
 * - Resets the controller.
 * - Runs the minimal init sequence (sleep out, color mode, MADCTL, display on).
 * - Optionally enables backlight and clears the screen.
 *
 * @param d Display instance.
 */
void st7735_init(ST7735_t* d)
{
    OMNIA_ASSERT(d != NULL);
    OMNIA_ASSERT(d->port != NULL);
    OMNIA_ASSERT(d->port->v != NULL);
    OMNIA_ASSERT(d->spi != NULL);
    OMNIA_ASSERT(d->fill_buf != NULL);

    /* Explicit dependency: this driver requires SPI. */
    OMNIA_ASSERT(omnia_port_has_spi());

    st7735_pins_init(d);
    st7735_reset(d);

    st7735_send_command(d, SWRESET); delay_ms(d, 150);
    st7735_send_command(d, SLPOUT ); delay_ms(d, 120);

    /* 16-bit color mode (RGB565). */
    st7735_send_command(d, COLMOD);
    st7735_send_data(d, 0x05);
    delay_ms(d, 10);

    /* Orientation: 0° or 180°. */
    uint8_t mad = MADCTL_BGR;
    if (d->rotate_180) {
        mad |= (MADCTL_MX | MADCTL_MY);
    }
    st7735_send_command(d, MADCTL);
    st7735_send_data(d, mad);

    st7735_send_command(d, DISPON);
    delay_ms(d, 100);

    if (d->bl) BL_ON(d);
    st7735_fill_screen(d, BLACK);
}

/* -------------------------------------------------------------------------- */
/* Text rendering (5x7 font)                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Draw a single character using the built-in 5x7 font table.
 *
 * Characters outside ASCII 32..126 are replaced with '?'.
 *
 * @param d      Display instance.
 * @param x      Start x coordinate.
 * @param y      Start y coordinate.
 * @param c      Character to draw.
 * @param color  RGB565 color.
 * @param size   Integer scaling factor (>= 1).
 */
void st7735_draw_char(ST7735_t* d,
                      uint16_t x, uint16_t y,
                      char c, uint16_t color, uint8_t size)
{
    if (c < 32 || c > 126) c = '?';
    const uint8_t* glyph = FONTS[(uint8_t)c - 32u]; /* 5 columns */

    for (uint8_t col = 0; col < CHARS_COLS_LENGTH; ++col) {
        uint8_t bits = glyph[col];
        for (uint8_t row = 0; row < CHARS_ROWS_LENGTH; ++row) {
            if (bits & 0x01u) {
                for (uint8_t dx = 0; dx < size; ++dx) {
                    for (uint8_t dy = 0; dy < size; ++dy) {
                        st7735_draw_pixel(d,
                                          (uint16_t)(x + (uint16_t)col * size + dx),
                                          (uint16_t)(y + (uint16_t)row * size + dy),
                                          color);
                    }
                }
            }
            bits >>= 1;
        }
    }
}

/**
 * @brief Draw a null-terminated string.
 *
 * Newlines ('\n') move the cursor to the next text line using the font height.
 *
 * @param d      Display instance.
 * @param x      Start x coordinate.
 * @param y      Start y coordinate.
 * @param s      Null-terminated string.
 * @param color  RGB565 color.
 * @param size   Integer scaling factor (>= 1).
 */
void st7735_draw_string(ST7735_t* d,
                        uint16_t x, uint16_t y,
                        const char* s, uint16_t color, uint8_t size)
{
    uint16_t cx = x;

    while (*s) {
        if (*s == '\n') {
            y  = (uint16_t)(y + (uint16_t)CHARS_ROWS_LENGTH * size + 1u);
            cx = x;
            s++;
            continue;
        }

        st7735_draw_char(d, cx, y, *s, color, size);
        cx = (uint16_t)(cx + (uint16_t)(CHARS_COLS_LENGTH + 1u) * size);
        s++;
    }
}

/* -------------------------------------------------------------------------- */
/* Drawing primitives                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Fill the entire screen with a solid color.
 *
 * The implementation uses a caller-provided fill buffer to keep memory usage
 * predictable and to enable efficient SPI bursts.
 *
 * @param d      Display instance.
 * @param color  RGB565 color.
 */
void st7735_fill_screen(ST7735_t* d, uint16_t color)
{
    OMNIA_ASSERT(d->fill_buf != NULL);

    st7735_set_address_window(d, 0, 0, (uint16_t)(d->width - 1u), (uint16_t)(d->height - 1u));

    for (uint16_t i = 0; i < ST7735_FILL_BLOCK; i++) {
        d->fill_buf[i] = color;
    }

    uint32_t total = (uint32_t)d->width * (uint32_t)d->height;

    while (total) {
        uint16_t n = (total > ST7735_FILL_BLOCK) ? ST7735_FILL_BLOCK : (uint16_t)total;

        DC_DATA(d);
        CS_LOW(d);
        spi_tx16(d, d->fill_buf, n);
        CS_HIGH(d);

        total -= n;
    }
}

/**
 * @brief Draw a line using an integer Bresenham implementation.
 *
 * @param d      Display instance.
 * @param x0     Start x.
 * @param y0     Start y.
 * @param x1     End x.
 * @param y1     End y.
 * @param color  RGB565 color.
 */
void st7735_draw_line(ST7735_t* d,
                      int16_t x0, int16_t y0,
                      int16_t x1, int16_t y1,
                      uint16_t color)
{
    int16_t dx = (int16_t)(x1 - x0);
    int16_t dy = (int16_t)(y1 - y0);
    int16_t sx = (dx >= 0) ? 1 : -1; dx = (int16_t)(sx * dx);
    int16_t sy = (dy >= 0) ? 1 : -1; dy = (int16_t)(sy * dy);
    int16_t err = (dx > dy ? dx : -dy) / 2;
    int16_t e2;

    for (;;) {
        if (x0 >= 0 && y0 >= 0 &&
            x0 < (int16_t)d->width &&
            y0 < (int16_t)d->height) {
            st7735_draw_pixel(d, (uint16_t)x0, (uint16_t)y0, color);
        }

        if (x0 == x1 && y0 == y1) break;
        e2 = err;
        if (e2 > -dx) { err = (int16_t)(err - dy); x0 = (int16_t)(x0 + sx); }
        if (e2 <  dy) { err = (int16_t)(err + dx); y0 = (int16_t)(y0 + sy); }
    }
}

/**
 * @brief Draw a rectangle outline.
 *
 * @param d      Display instance.
 * @param x      Top-left x.
 * @param y      Top-left y.
 * @param w      Width in pixels.
 * @param h      Height in pixels.
 * @param color  RGB565 color.
 */
void st7735_draw_rectangle(ST7735_t* d,
                           uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h,
                           uint16_t color)
{
    if (!w || !h) return;
    st7735_draw_line(d, (int16_t)x,         (int16_t)y,         (int16_t)(x + w - 1u), (int16_t)y,         color);
    st7735_draw_line(d, (int16_t)x,         (int16_t)(y + h - 1u), (int16_t)(x + w - 1u), (int16_t)(y + h - 1u), color);
    st7735_draw_line(d, (int16_t)x,         (int16_t)y,         (int16_t)x,         (int16_t)(y + h - 1u), color);
    st7735_draw_line(d, (int16_t)(x + w - 1u), (int16_t)y,         (int16_t)(x + w - 1u), (int16_t)(y + h - 1u), color);
}

/**
 * @brief Draw a filled rectangle.
 *
 * This function programs a window and bursts RGB565 data using the fill buffer.
 *
 * @param d      Display instance.
 * @param x      Top-left x.
 * @param y      Top-left y.
 * @param w      Width.
 * @param h      Height.
 * @param color  RGB565 color.
 */
void st7735_filled_rectangle(ST7735_t* d,
                             uint16_t x, uint16_t y,
                             uint16_t w, uint16_t h,
                             uint16_t color)
{
    if (!w || !h) return;

    uint16_t x1 = (uint16_t)(x + w - 1u);
    uint16_t y1 = (uint16_t)(y + h - 1u);

    st7735_set_address_window(d, x, y, x1, y1);

    uint32_t total = (uint32_t)w * (uint32_t)h;

    OMNIA_ASSERT(d->fill_buf != NULL);
    for (uint16_t i = 0; i < ST7735_FILL_BLOCK; i++) {
        d->fill_buf[i] = color;
    }

    while (total) {
        uint16_t n = (total > ST7735_FILL_BLOCK) ? ST7735_FILL_BLOCK : (uint16_t)total;
        DC_DATA(d);
        CS_LOW(d);
        spi_tx16(d, d->fill_buf, n);
        CS_HIGH(d);
        total -= n;
    }
}

/**
 * @brief Draw a circle outline (midpoint algorithm).
 *
 * @param d      Display instance.
 * @param x0     Center x.
 * @param y0     Center y.
 * @param r      Radius.
 * @param color  RGB565 color.
 */
void st7735_draw_circle(ST7735_t* d,
                        int16_t x0, int16_t y0,
                        int16_t r, uint16_t color)
{
    int16_t f     = (int16_t)(1 - r);
    int16_t ddF_x = 1;
    int16_t ddF_y = (int16_t)(-2 * r);
    int16_t x     = 0;
    int16_t y     = r;

    st7735_draw_pixel(d, (uint16_t)x0,     (uint16_t)(y0 + r), color);
    st7735_draw_pixel(d, (uint16_t)x0,     (uint16_t)(y0 - r), color);
    st7735_draw_pixel(d, (uint16_t)(x0 + r), (uint16_t)y0,     color);
    st7735_draw_pixel(d, (uint16_t)(x0 - r), (uint16_t)y0,     color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y = (int16_t)(ddF_y + 2);
            f     = (int16_t)(f + ddF_y);
        }
        x++;
        ddF_x = (int16_t)(ddF_x + 2);
        f     = (int16_t)(f + ddF_x);

        st7735_draw_pixel(d, (uint16_t)(x0 + x), (uint16_t)(y0 + y), color);
        st7735_draw_pixel(d, (uint16_t)(x0 - x), (uint16_t)(y0 + y), color);
        st7735_draw_pixel(d, (uint16_t)(x0 + x), (uint16_t)(y0 - y), color);
        st7735_draw_pixel(d, (uint16_t)(x0 - x), (uint16_t)(y0 - y), color);
        st7735_draw_pixel(d, (uint16_t)(x0 + y), (uint16_t)(y0 + x), color);
        st7735_draw_pixel(d, (uint16_t)(x0 - y), (uint16_t)(y0 + x), color);
        st7735_draw_pixel(d, (uint16_t)(x0 + y), (uint16_t)(y0 - x), color);
        st7735_draw_pixel(d, (uint16_t)(x0 - y), (uint16_t)(y0 - x), color);
    }
}

/**
 * @brief Draw a filled circle.
 *
 * @param d      Display instance.
 * @param x0     Center x.
 * @param y0     Center y.
 * @param r      Radius.
 * @param color  RGB565 color.
 */
void st7735_filled_circle(ST7735_t* d,
                          int16_t x0, int16_t y0,
                          int16_t r, uint16_t color)
{
    st7735_draw_line(d, x0, (int16_t)(y0 - r), x0, (int16_t)(y0 + r), color);

    int16_t f     = (int16_t)(1 - r);
    int16_t ddF_x = 1;
    int16_t ddF_y = (int16_t)(-2 * r);
    int16_t x     = 0;
    int16_t y     = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y = (int16_t)(ddF_y + 2);
            f     = (int16_t)(f + ddF_y);
        }
        x++;
        ddF_x = (int16_t)(ddF_x + 2);
        f     = (int16_t)(f + ddF_x);

        st7735_draw_line(d, (int16_t)(x0 - x), (int16_t)(y0 + y), (int16_t)(x0 + x), (int16_t)(y0 + y), color);
        st7735_draw_line(d, (int16_t)(x0 - x), (int16_t)(y0 - y), (int16_t)(x0 + x), (int16_t)(y0 - y), color);
        st7735_draw_line(d, (int16_t)(x0 - y), (int16_t)(y0 + x), (int16_t)(x0 + y), (int16_t)(y0 + x), color);
        st7735_draw_line(d, (int16_t)(x0 - y), (int16_t)(y0 - x), (int16_t)(x0 + y), (int16_t)(y0 - x), color);
    }
}

/**
 * @brief Swap two int16_t values.
 */
static inline void swap_i16(int16_t* a, int16_t* b)
{
    int16_t t = *a;
    *a = *b;
    *b = t;
}

/**
 * @brief Draw a triangle outline.
 *
 * @param d      Display instance.
 * @param x0,y0  First vertex.
 * @param x1,y1  Second vertex.
 * @param x2,y2  Third vertex.
 * @param color  RGB565 color.
 */
void st7735_draw_triangle(ST7735_t* d,
                          int16_t x0, int16_t y0,
                          int16_t x1, int16_t y1,
                          int16_t x2, int16_t y2,
                          uint16_t color)
{
    st7735_draw_line(d, x0, y0, x1, y1, color);
    st7735_draw_line(d, x1, y1, x2, y2, color);
    st7735_draw_line(d, x2, y2, x0, y0, color);
}

/**
 * @brief Draw a filled triangle using scanline rasterization.
 *
 * @param d      Display instance.
 * @param x0,y0  First vertex.
 * @param x1,y1  Second vertex.
 * @param x2,y2  Third vertex.
 * @param color  RGB565 color.
 */
void st7735_filled_triangle(ST7735_t* d,
                            int16_t x0, int16_t y0,
                            int16_t x1, int16_t y1,
                            int16_t x2, int16_t y2,
                            uint16_t color)
{
    if (y0 > y1) { swap_i16(&y0, &y1); swap_i16(&x0, &x1); }
    if (y1 > y2) { swap_i16(&y1, &y2); swap_i16(&x1, &x2); }
    if (y0 > y1) { swap_i16(&y0, &y1); swap_i16(&x0, &x1); }

    if (y0 == y2) {
        int16_t minx = x0, maxx = x0;
        if (x1 < minx) minx = x1;
        if (x2 < minx) minx = x2;
        if (x1 > maxx) maxx = x1;
        if (x2 > maxx) maxx = x2;
        st7735_draw_line(d, minx, y0, maxx, y0, color);
        return;
    }

    int32_t dx01 = x1 - x0, dy01 = y1 - y0;
    int32_t dx02 = x2 - x0, dy02 = y2 - y0;
    int32_t dx12 = x2 - x1, dy12 = y2 - y1;

    int32_t sa = 0, sb = 0;
    int16_t y;

    int16_t last = (y1 == y2) ? y1 : (int16_t)(y1 - 1);
    for (y = y0; y <= last; y++) {
        int16_t a = x0;
        int16_t b = x0;
        if (dy01) a = (int16_t)(a + (int16_t)(sa / dy01));
        if (dy02) b = (int16_t)(b + (int16_t)(sb / dy02));
        sa += dx01;
        sb += dx02;
        if (a > b) { int16_t t = a; a = b; b = t; }
        st7735_draw_line(d, a, y, b, y, color);
    }

    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        int16_t a = x1;
        int16_t b = x0;
        if (dy12) a = (int16_t)(a + (int16_t)(sa / dy12));
        if (dy02) b = (int16_t)(b + (int16_t)(sb / dy02));
        sa += dx12;
        sb += dx02;
        if (a > b) { int16_t t = a; a = b; b = t; }
        st7735_draw_line(d, a, y, b, y, color);
    }
}
