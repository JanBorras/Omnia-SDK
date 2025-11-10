#ifndef ST7735_H
#define ST7735_H

#include <msp430.h>
#include <stdint.h>
#include "fonts.h"
#include "omnia_port.h"

// RGB565 Colors
#define BLUE 0x001F
#define WHITE 0xFFFF
#define BLACK 0x0000
#define RED 0xF800
#define GREEN 0x07E0

// ST7735 commands
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

// AREA definition
#define MAX_X                 161               // max columns / MV = 0 in MADCTL
#define MAX_Y                 130               // max rows / MV = 0 in MADCTL
#define SIZE_X                (MAX_X - 1)         // columns max counter
#define SIZE_Y                (MAX_Y - 1)         // rows max counter
#define CACHE_SIZE_MEM        (MAX_X * MAX_Y)   // whole pixels
#define CHARS_COLS_LEN        5                 // number of columns for chars
#define CHARS_ROWS_LEN        8                 // number of rows for chars

// MISC
#define SWAP(a, b) { int temp = a; a = b; b = temp; }

/*** ST7735 Screen Structure ***/
typedef struct {
    uint8_t width;
    uint8_t height;
    uint16_t *framebuffer;
    gpiopin_t rst;
    gpiopin_t dc;
    gpiopin_t bl;
    gpiopin_t sclk;
    gpiopin_t simo;
    gpiopin_t cs;
    spi_t *spi;
} ST7735_t;

// Function prototypes

omnia_status_t st7735_init(ST7735_t *screen);
void st7735_reset(ST7735_t *screen);
void st7735_command_mode(ST7735_t *screen);
void st7735_data_mode(ST7735_t *screen);
void st7735_send_command(ST7735_t *screen, uint8_t cmd);
void st7735_send_16bits_data(ST7735_t *screen, uint16_t data);
void st7735_send_data(ST7735_t *screen, uint8_t data);
void st7735_set_address_window(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void st7735_draw_pixel(ST7735_t *screen, uint8_t x, uint8_t y, uint16_t color);
void st7735_draw_char(ST7735_t *screen, uint8_t x, uint8_t y, char c, uint16_t color, uint8_t size);
void st7735_draw_string(ST7735_t *screen, uint8_t x, uint8_t y, const char* str, uint16_t color, uint8_t size);
void st7735_fill_screen(ST7735_t *screen, uint16_t color);
void st7735_draw_line(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint16_t color);
void st7735_draw_rectangle(ST7735_t *screen, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void st7735_filled_rectangle(ST7735_t *screen, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
void st7735_draw_circle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t r, uint16_t color);
void st7735_filled_circle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t r, uint16_t color);
void st7735_draw_triangle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
void st7735_filled_triangle(ST7735_t *screen, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);

// RGB565 encoding function
uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue);



#endif
