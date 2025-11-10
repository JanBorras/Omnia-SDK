#include "st7735.h"

/*** Initialization and Configuration ***/
void st7735_init(ST7735_t *screen) {
    gpio_init_output(&screen->rst);
    gpio_init_output(&screen->dc);
    gpio_init_output(&screen->bl);
    gpio_init_function(&screen->sclk);
    gpio_init_function(&screen->simo);
    gpio_init_function(&screen->cs);

    spi_init(screen->spi);

    st7735_reset(screen);
    st7735_send_command(screen, SWRESET);
    timer_delay_ms(150);

    st7735_send_command(screen, SLPOUT);
    timer_delay_ms(200);

    st7735_send_command(screen, COLMOD);
    st7735_send_data(screen, 0x05);

    st7735_send_command(screen, MADCTL);
    st7735_send_data(screen, 0xA0);

    st7735_fill_screen(screen, WHITE);

    st7735_send_command(screen, DISPON);
    timer_delay_ms(200);
}

void st7735_reset(ST7735_t *screen) {
    gpio_set_low(&screen->rst);
    timer_delay_ms(200);
    gpio_set_high(&screen->rst);
    timer_delay_ms(200);
}

/*** Data and Command Transmission ***/
void st7735_command_mode(ST7735_t *screen) {
    gpio_set_low(&screen->dc);
}

void st7735_data_mode(ST7735_t *screen) {
    gpio_set_high(&screen->dc);
}

void st7735_send_command(ST7735_t *screen, uint8_t cmd) {
    st7735_command_mode(screen);
    spi_transmit_8bit(screen->spi, cmd);
}

void st7735_send_data(ST7735_t *screen, uint8_t data) {
    st7735_data_mode(screen);
    spi_transmit_8bit(screen->spi, data);
}

void st7735_send_16bits_data(ST7735_t *screen, uint16_t data) {
    spi_transmit_16bit(screen->spi, data);
}

/*** Addressing and Pixel Manipulation ***/
void st7735_set_address_window(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    if (x1 >= SIZE_X) x1 = SIZE_X;
    if (y1 >= SIZE_Y) y1 = SIZE_Y;

    st7735_send_command(screen, CASET);
    st7735_send_data(screen, 0x00);
    st7735_send_data(screen, x0);
    st7735_send_data(screen, 0x00);
    st7735_send_data(screen, x1);

    st7735_send_command(screen, RASET);
    st7735_send_data(screen, 0x00);
    st7735_send_data(screen, y0);
    st7735_send_data(screen, 0x00);
    st7735_send_data(screen, y1);

    st7735_send_command(screen, RAMWR);
}

void st7735_draw_pixel(ST7735_t *screen, uint8_t x, uint8_t y, uint16_t color) {
    st7735_set_address_window(screen, x, y, x, y);
    st7735_data_mode(screen);
    st7735_send_16bits_data(screen, color);
}

void st7735_draw_char(ST7735_t *screen, uint8_t x, uint8_t y, char c, uint16_t color, uint8_t size) {
    uint8_t col, row, i, j;

    if (c < 32 || c > 126) {
        c = '?';
    }

    const uint8_t *char_bitmap = FONTS[c - 32];
    uint8_t char_width = CHARS_COLS_LEN * size;
    uint8_t char_height = CHARS_ROWS_LEN * size;

    if (x + char_width > screen->width || y + char_height > screen->height) {
        return;
    }

    for (col = 0; col < CHARS_COLS_LEN; col++) {
        uint8_t line = char_bitmap[col];
        for (row = 0; row < CHARS_ROWS_LEN; row++) {
            if (line & 0x01) {
                for (i = 0; i < size; i++) {
                    for (j = 0; j < size; j++) {
                        st7735_draw_pixel(screen, x + (col * size) + i, y + (row * size) + j, color);
                    }
                }
            }
            line >>= 1;
        }
    }
}


void st7735_draw_string(ST7735_t *screen, uint8_t x, uint8_t y, const char* str, uint16_t color, uint8_t size) {
    while (*str) {
        if (x + CHARS_COLS_LEN > screen->width) {
            x = 0;
            y += CHARS_ROWS_LEN + 1;
            if (y + CHARS_ROWS_LEN > screen->height) {
                break;
            }
        }
        st7735_draw_char(screen, x, y, *str, color, size);
        x += CHARS_COLS_LEN + 1;
        str++;
    }
}

void st7735_fill_screen(ST7735_t *screen, uint16_t color) {
    uint16_t i;
    static const uint16_t total_pixels = SIZE_X * SIZE_Y;
    st7735_set_address_window(screen, 0, 0, SIZE_X, SIZE_Y);
    st7735_data_mode(screen);
    for (i = 0; i < total_pixels; i++) {
        st7735_send_16bits_data(screen, color);
    }
}

void st7735_draw_line(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        st7735_draw_pixel(screen, x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        int e2 = err * 2;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx) { err += dx; y0 += sy; }
    }
}

void st7735_draw_rectangle(ST7735_t *screen, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    st7735_draw_line(screen, x, y, x + w - 1, y, color);
    st7735_draw_line(screen, x, y + h - 1, x + w - 1, y + h - 1, color);
    st7735_draw_line(screen, x, y, x, y + h - 1, color);
    st7735_draw_line(screen, x + w - 1, y, x + w - 1, y + h - 1, color);
}

void st7735_filled_rectangle(ST7735_t *screen, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color) {
    uint16_t i;
    uint16_t total_pixels = w * h;  // Calculate total pixels to fill

    st7735_set_address_window(screen, x, y, x + w - 1, y + h - 1);  // Set the address window
    st7735_data_mode(screen);  // Switch to data mode

    for (i = 0; i < total_pixels; i++) {
        st7735_send_16bits_data(screen, color);  // Send pixel data
    }
}

void st7735_draw_circle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t r, uint16_t color) {
    int x = r, y = 0;
    int err = 1 - x;

    while (x >= y) {
        st7735_draw_pixel(screen, x0 + x, y0 + y, color);
        st7735_draw_pixel(screen, x0 - x, y0 + y, color);
        st7735_draw_pixel(screen, x0 + x, y0 - y, color);
        st7735_draw_pixel(screen, x0 - x, y0 - y, color);
        st7735_draw_pixel(screen, x0 + y, y0 + x, color);
        st7735_draw_pixel(screen, x0 - y, y0 + x, color);
        st7735_draw_pixel(screen, x0 + y, y0 - x, color);
        st7735_draw_pixel(screen, x0 - y, y0 - x, color);

        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

void st7735_filled_circle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t r, uint16_t color) {
    int x = r, y = 0;
    int err = 1 - x;

    while (x >= y) {
        st7735_draw_line(screen, x0 - x, y0 + y, x0 + x, y0 + y, color);
        st7735_draw_line(screen, x0 - x, y0 - y, x0 + x, y0 - y, color);
        st7735_draw_line(screen, x0 - y, y0 + x, x0 + y, y0 + x, color);
        st7735_draw_line(screen, x0 - y, y0 - x, x0 + y, y0 - x, color);

        y++;
        if (err <= 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

void st7735_draw_triangle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {
    st7735_draw_line(screen, x0, y0, x1, y1, color);
    st7735_draw_line(screen, x1, y1, x2, y2, color);
    st7735_draw_line(screen, x2, y2, x0, y0, color);
}

void st7735_filled_triangle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {
    int16_t a, b, y, last;

    if (y0 > y1) { SWAP(x0, x1); SWAP(y0, y1); }
    if (y1 > y2) { SWAP(x1, x2); SWAP(y1, y2); }
    if (y0 > y1) { SWAP(x0, x1); SWAP(y0, y1); }

    int16_t dx01 = x1 - x0, dy01 = y1 - y0;
    int16_t dx02 = x2 - x0, dy02 = y2 - y0;
    int16_t dx12 = x2 - x1, dy12 = y2 - y1;

    int32_t sa = 0, sb = 0;

    if (y1 == y2) last = y1; else last = y1 - 1;

    for (y = y0; y <= last; y++) {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if (a > b) SWAP(a, b);
        st7735_draw_line(screen, a, y, b, y, color);
    }

    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);

    for (; y <= y2; y++) {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b) SWAP(a, b);
        st7735_draw_line(screen, a, y, b, y, color);
    }
}

/*** Utility Functions ***/
uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0x1F) << 11) | ((green & 0x3F) << 5) | (blue & 0x1F);
}
