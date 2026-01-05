#ifndef ST7735_H
#define ST7735_H

#include <stdint.h>
#include "fonts.h"
#include "omnia_port.h"

// ===== Colors RGB565 =====
#define BLUE     0x001F
#define WHITE    0xFFFF
#define BLACK    0x0000
#define RED      0xF800
#define GREEN    0x07E0
#define YELLOW   0xFFE0
#define CYAN     0x07FF
#define MAGENTA  0xF81F

// Accepta r,g,b en 0–255 i els escala a 5/6/5
static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b){
  uint16_t r5 = (uint16_t)(r >> 3) & 0x1F;  // 8 -> 5 bits
  uint16_t g6 = (uint16_t)(g >> 2) & 0x3F;  // 8 -> 6 bits
  uint16_t b5 = (uint16_t)(b >> 3) & 0x1F;  // 8 -> 5 bits
  return (r5 << 11) | (g6 << 5) | b5;
}

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

// MADCTL flags
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08

// Fonts (coherència noms)
#define CHARS_COLS_LENGTH     5   // nombre de columnes per caràcter
#define CHARS_ROWS_LENGTH     8   // nombre de files per caràcter

// Buffer per farcir pantalla / rectangles grans
#define ST7735_FILL_BLOCK 64

// ===== Dispositiu =====
typedef struct {
  omnia_port_t* port;     // vtable & timing
  omnia_spi_t   spi;      // bus/handle opac

  // Pins de control
  omnia_gpio_t  cs;
  omnia_gpio_t  dc;
  omnia_gpio_t  rst;
  omnia_gpio_t  bl;       // backlight (opcional; pot ser NULL)

  // (compat amb el teu main antic; no calen per a la lògica)
  omnia_gpio_t  sclk;
  omnia_gpio_t  simo;

  uint16_t       width;
  uint16_t       height;

  // finestra corrent (cache)
  uint16_t x0, y0, x1, y1;

  uint8_t rotate_180;

  // 🔥 buffer intern per operacions de bloc (farcir pantalla, rectangles grans…)
  uint16_t      fill_buf[ST7735_FILL_BLOCK];
} ST7735_t;

// ===== API pública =====
void st7735_init(ST7735_t *screen);
void st7735_reset(ST7735_t *screen);

void st7735_send_command(ST7735_t *screen, uint8_t cmd);
void st7735_send_data(ST7735_t *screen, uint8_t data);
void st7735_send_16bits_data(ST7735_t *screen, uint16_t data);

void st7735_set_address_window(ST7735_t *screen,
                               uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1);
void st7735_draw_pixel(ST7735_t *screen,
                       uint16_t x, uint16_t y, uint16_t color);

void st7735_draw_char(ST7735_t *screen,
                      uint16_t x, uint16_t y,
                      char c, uint16_t color, uint8_t size);
void st7735_draw_string(ST7735_t *screen,
                        uint16_t x, uint16_t y,
                        const char* str, uint16_t color, uint8_t size);

void st7735_fill_screen(ST7735_t *screen, uint16_t color);
void st7735_draw_line(ST7735_t *screen,
                      int16_t x0, int16_t y0,
                      int16_t x1, int16_t y1,
                      uint16_t color);
void st7735_draw_rectangle(ST7735_t *screen,
                           uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h, uint16_t color);
void st7735_filled_rectangle(ST7735_t *screen,
                             uint16_t x, uint16_t y,
                             uint16_t w, uint16_t h, uint16_t color);
void st7735_draw_circle(ST7735_t *screen,
                        int16_t x0, int16_t y0, int16_t r, uint16_t color);
void st7735_filled_circle(ST7735_t *screen,
                          int16_t x0, int16_t y0, int16_t r, uint16_t color);
void st7735_draw_triangle(ST7735_t *screen,
                          int16_t x0, int16_t y0,
                          int16_t x1, int16_t y1,
                          int16_t x2, int16_t y2,
                          uint16_t color);
void st7735_filled_triangle(ST7735_t *screen,
                            int16_t x0, int16_t y0,
                            int16_t x1, int16_t y1,
                            int16_t x2, int16_t y2,
                            uint16_t color);

#endif // ST7735_H
