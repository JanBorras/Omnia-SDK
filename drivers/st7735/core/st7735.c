#include "st7735.h"
#include <stddef.h>

#include "omnia_port.h"   // <-- per omnia_port_has_spi

/* --- Trampa portàtil per a asserts de debug --- */
#if defined(__arm__) || defined(__thumb__)
  #define OMNIA_TRAP() __asm volatile("bkpt 15")
#else
  #include <stdlib.h>
  #define OMNIA_TRAP() abort()
#endif

#ifndef OMNIA_ASSERT
#define OMNIA_ASSERT(expr) do { if (!(expr)) OMNIA_TRAP(); } while (0)
#endif

// ========== Helpers privats que mapejen 1:1 a omnia_port ==========

static inline const omnia_port_vtable_t* io(ST7735_t* d)
{
  OMNIA_ASSERT(d != NULL);
  OMNIA_ASSERT(d->port != NULL);
  OMNIA_ASSERT(d->port->v != NULL);
  return d->port->v;
}

static inline void CS_LOW(ST7735_t* d){
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(d->cs != NULL);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->cs, 0);
}

static inline void CS_HIGH(ST7735_t* d){
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(d->cs != NULL);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->cs, 1);
}

static inline void DC_CMD(ST7735_t* d){
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(d->dc != NULL);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->dc, 0);
}

static inline void DC_DATA(ST7735_t* d){
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(d->dc != NULL);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->dc, 1);
}

static inline void RST_LOW(ST7735_t* d){
  if (!d->rst) return;
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->rst, 0);
}

static inline void RST_HIGH(ST7735_t* d){
  if (!d->rst) return;
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->rst, 1);
}

static inline void BL_ON(ST7735_t* d){
  if (!d->bl) return;
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->bl, 1);
}

static inline void BL_OFF(ST7735_t* d){
  if (!d->bl) return;
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(v->gpio_write != NULL);
  v->gpio_write(d->bl, 0);
}

static inline void delay_ms(ST7735_t* d, uint32_t ms){
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(v->delay_us != NULL);
  while (ms--) {
    v->delay_us(1000);
  }
}

static inline void spi_tx8(ST7735_t* d, const uint8_t* buf, size_t n){
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(v->spi_tx != NULL);
  v->spi_tx(d->spi, buf, n);
}

static inline void spi_tx16(ST7735_t* d, const uint16_t* buf, size_t n){
  const omnia_port_vtable_t* v = io(d);
  if (v->spi_tx16) {
    v->spi_tx16(d->spi, buf, n);
  } else {
    OMNIA_ASSERT(v->spi_tx != NULL);
    for (size_t i = 0; i < n; i++){
      uint8_t b[2] = { (uint8_t)(buf[i] >> 8), (uint8_t)buf[i] };
      v->spi_tx(d->spi, b, 2);
    }
  }
}

/* --- Inicialització de pins específica del driver ------------------------ */

static void st7735_pins_init(ST7735_t *lcd)
{
  omnia_port_t *p = lcd->port;
  OMNIA_ASSERT(p && p->v && p->v->gpio_mode);

  // Tots com a sortides, sense pull
  p->v->gpio_mode(lcd->cs,  1, OMNIA_GPIO_PULL_NONE);
  p->v->gpio_mode(lcd->dc,  1, OMNIA_GPIO_PULL_NONE);
  if (lcd->rst) p->v->gpio_mode(lcd->rst, 1, OMNIA_GPIO_PULL_NONE);
  if (lcd->bl)  p->v->gpio_mode(lcd->bl,  1, OMNIA_GPIO_PULL_NONE);

  // Nivells inicials lògics per la pantalla
  CS_HIGH(lcd);
  DC_CMD(lcd);
  if (lcd->rst) RST_HIGH(lcd);
  if (lcd->bl)  BL_OFF(lcd);
}

// ========== Primitives bàsiques ==========

void st7735_send_command(ST7735_t *d, uint8_t cmd){
  DC_CMD(d);
  CS_LOW(d);
  spi_tx8(d, &cmd, 1);
  CS_HIGH(d);
}

void st7735_send_data(ST7735_t *d, uint8_t data){
  DC_DATA(d);
  CS_LOW(d);
  spi_tx8(d, &data, 1);
  CS_HIGH(d);
}

void st7735_send_16bits_data(ST7735_t *d, uint16_t data){
  DC_DATA(d);
  CS_LOW(d);
  spi_tx16(d, &data, 1);
  CS_HIGH(d);
}

void st7735_reset(ST7735_t *d){
  const omnia_port_vtable_t* v = io(d);
  OMNIA_ASSERT(v->delay_us != NULL);

  RST_LOW(d);
  v->delay_us(10 * 1000);
  RST_HIGH(d);
  v->delay_us(10 * 1000);
}

void st7735_set_address_window(ST7735_t *d,
                               uint16_t x0, uint16_t y0,
                               uint16_t x1, uint16_t y1){
  d->x0 = x0; d->y0 = y0; d->x1 = x1; d->y1 = y1;

  uint8_t data[4];

  // Column (CASET)
  st7735_send_command(d, CASET);
  data[0] = (uint8_t)(x0 >> 8);
  data[1] = (uint8_t)(x0 & 0xFF);
  data[2] = (uint8_t)(x1 >> 8);
  data[3] = (uint8_t)(x1 & 0xFF);
  DC_DATA(d);
  CS_LOW(d);
  spi_tx8(d, data, 4);
  CS_HIGH(d);

  // Row (RASET)
  st7735_send_command(d, RASET);
  data[0] = (uint8_t)(y0 >> 8);
  data[1] = (uint8_t)(y0 & 0xFF);
  data[2] = (uint8_t)(y1 >> 8);
  data[3] = (uint8_t)(y1 & 0xFF);
  DC_DATA(d);
  CS_LOW(d);
  spi_tx8(d, data, 4);
  CS_HIGH(d);

  // RAMWR
  st7735_send_command(d, RAMWR);
}

void st7735_draw_pixel(ST7735_t *d,
                       uint16_t x, uint16_t y,
                       uint16_t color){

  if (x >= d->width || y >= d->height) return;

  st7735_set_address_window(d, x, y, x, y);
  st7735_send_16bits_data(d, color);
}

// ========== Inicialització ==========

void st7735_init(ST7735_t *d){
  OMNIA_ASSERT(d != NULL);
  OMNIA_ASSERT(d->port != NULL);
  OMNIA_ASSERT(d->port->v != NULL);
  OMNIA_ASSERT(d->spi != NULL);
  OMNIA_ASSERT(d->fill_buf != NULL);

  // Declaració explícita de dependència: aquest driver NECESSITA SPI
  OMNIA_ASSERT(omnia_port_has_spi());

  // 1) Configurar pins via vtable (gpio_mode)
  st7735_pins_init(d);

  // 2) Reset i seqüència d’inicialització
  st7735_reset(d);

  st7735_send_command(d, SWRESET); delay_ms(d, 150);
  st7735_send_command(d, SLPOUT ); delay_ms(d, 120);

  // format de color 16-bit (RGB565)
  st7735_send_command(d, COLMOD);
  st7735_send_data(d, 0x05);
  delay_ms(d, 10);

  // --- ORIENTACIÓ: 0° o 180° ---
  uint8_t mad = MADCTL_BGR;           // base: BGR, sense flip
  if (d->rotate_180) {
    mad |= MADCTL_MX | MADCTL_MY;     // flip 180°
  }

  st7735_send_command(d, MADCTL);
  st7735_send_data(d, mad);

  st7735_send_command(d, DISPON);
  delay_ms(d, 100);

  if (d->bl) BL_ON(d);
  st7735_fill_screen(d, BLACK);
}

// ========== Dibuix de text (fonts 5x8 amb fonts.h) ==========

void st7735_draw_char(ST7735_t *d,
                      uint16_t x, uint16_t y,
                      char c, uint16_t color, uint8_t size){
  if (c < 32 || c > 126) c = '?';
  const uint8_t* glyph = FONTS[(uint8_t)c - 32]; // 5 columnes, 8 files

  for (uint8_t col = 0; col < CHARS_COLS_LENGTH; ++col){
    uint8_t bits = glyph[col];
    for (uint8_t row = 0; row < CHARS_ROWS_LENGTH; ++row){
      if (bits & 0x01){
        for (uint8_t dx = 0; dx < size; ++dx){
          for (uint8_t dy = 0; dy < size; ++dy){
            st7735_draw_pixel(d,
                              x + col * size + dx,
                              y + row * size + dy,
                              color);
          }
        }
      }
      bits >>= 1;
    }
  }
}

void st7735_draw_string(ST7735_t *d,
                        uint16_t x, uint16_t y,
                        const char* s, uint16_t color, uint8_t size){
  uint16_t cx = x;
  while (*s){
    if (*s == '\n'){
      y  += CHARS_ROWS_LENGTH * size + 1;
      cx  = x;
      s++;
      continue;
    }
    st7735_draw_char(d, cx, y, *s, color, size);
    cx += (CHARS_COLS_LENGTH + 1) * size;
    s++;
  }
}

// ========== Figures bàsiques ==========

void st7735_fill_screen(ST7735_t *d, uint16_t color){
  OMNIA_ASSERT(d->fill_buf != NULL);

  st7735_set_address_window(d, 0, 0, d->width - 1, d->height - 1);

  for (uint16_t i = 0; i < ST7735_FILL_BLOCK; i++) {
    d->fill_buf[i] = color;
  }

  uint32_t total = (uint32_t)d->width * (uint32_t)d->height;

  while (total) {
    uint16_t n = (total > ST7735_FILL_BLOCK)
                   ? ST7735_FILL_BLOCK
                   : (uint16_t)total;

    DC_DATA(d);
    CS_LOW(d);
    spi_tx16(d, d->fill_buf, n);
    CS_HIGH(d);

    total -= n;
  }
}

void st7735_draw_line(ST7735_t *d,
                      int16_t x0, int16_t y0,
                      int16_t x1, int16_t y1,
                      uint16_t color){
  int16_t dx = (x1 - x0);
  int16_t dy = (y1 - y0);
  int16_t sx = (dx >= 0) ? 1 : -1; dx = (int16_t)(sx * dx);
  int16_t sy = (dy >= 0) ? 1 : -1; dy = (int16_t)(sy * dy);
  int16_t err = (dx > dy ? dx : -dy) / 2;
  int16_t e2;

  for(;;){
    if (x0 >= 0 && y0 >= 0 &&
        x0 < (int16_t)d->width &&
        y0 < (int16_t)d->height){
      st7735_draw_pixel(d, (uint16_t)x0, (uint16_t)y0, color);
    }

    if (x0 == x1 && y0 == y1) break;
    e2 = err;
    if (e2 > -dx){ err -= dy; x0 += sx; }
    if (e2 <  dy){ err += dx; y0 += sy; }
  }
}

void st7735_draw_rectangle(ST7735_t *d,
                           uint16_t x, uint16_t y,
                           uint16_t w, uint16_t h,
                           uint16_t color){
  if (!w || !h) return;
  st7735_draw_line(d, x,         y,         x + w - 1, y,         color);
  st7735_draw_line(d, x,         y + h - 1, x + w - 1, y + h - 1, color);
  st7735_draw_line(d, x,         y,         x,         y + h - 1, color);
  st7735_draw_line(d, x + w - 1, y,         x + w - 1, y + h - 1, color);
}

void st7735_filled_rectangle(ST7735_t *d,
                             uint16_t x, uint16_t y,
                             uint16_t w, uint16_t h,
                             uint16_t color){
  if (!w || !h) return;
  uint16_t x1 = x + w - 1;
  uint16_t y1 = y + h - 1;

  st7735_set_address_window(d, x, y, x1, y1);

  uint32_t total = (uint32_t)w * (uint32_t)h;

  OMNIA_ASSERT(d->fill_buf != NULL);
  for (uint16_t i = 0; i < ST7735_FILL_BLOCK; i++) {
    d->fill_buf[i] = color;
  }

  while (total){
    uint16_t n = (total > ST7735_FILL_BLOCK)
                   ? ST7735_FILL_BLOCK
                   : (uint16_t)total;
    DC_DATA(d);
    CS_LOW(d);
    spi_tx16(d, d->fill_buf, n);
    CS_HIGH(d);
    total -= n;
  }
}

void st7735_draw_circle(ST7735_t *d,
                        int16_t x0, int16_t y0,
                        int16_t r, uint16_t color){
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  st7735_draw_pixel(d, x0,     y0 + r, color);
  st7735_draw_pixel(d, x0,     y0 - r, color);
  st7735_draw_pixel(d, x0 + r, y0,     color);
  st7735_draw_pixel(d, x0 - r, y0,     color);

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    st7735_draw_pixel(d, x0 + x, y0 + y, color);
    st7735_draw_pixel(d, x0 - x, y0 + y, color);
    st7735_draw_pixel(d, x0 + x, y0 - y, color);
    st7735_draw_pixel(d, x0 - x, y0 - y, color);
    st7735_draw_pixel(d, x0 + y, y0 + x, color);
    st7735_draw_pixel(d, x0 - y, y0 + x, color);
    st7735_draw_pixel(d, x0 + y, y0 - x, color);
    st7735_draw_pixel(d, x0 - y, y0 - x, color);
  }
}

void st7735_filled_circle(ST7735_t *d,
                          int16_t x0, int16_t y0,
                          int16_t r, uint16_t color){
  st7735_draw_line(d, x0, y0 - r, x0, y0 + r, color);

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x < y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    st7735_draw_line(d, x0 - x, y0 + y, x0 + x, y0 + y, color);
    st7735_draw_line(d, x0 - x, y0 - y, x0 + x, y0 - y, color);
    st7735_draw_line(d, x0 - y, y0 + x, x0 + y, y0 + x, color);
    st7735_draw_line(d, x0 - y, y0 - x, x0 + y, y0 - x, color);
  }
}

static inline void swap_i16(int16_t *a, int16_t *b){
  int16_t t = *a;
  *a = *b;
  *b = t;
}

void st7735_draw_triangle(ST7735_t *d,
                          int16_t x0, int16_t y0,
                          int16_t x1, int16_t y1,
                          int16_t x2, int16_t y2,
                          uint16_t color){
  st7735_draw_line(d, x0, y0, x1, y1, color);
  st7735_draw_line(d, x1, y1, x2, y2, color);
  st7735_draw_line(d, x2, y2, x0, y0, color);
}

void st7735_filled_triangle(ST7735_t *d,
                            int16_t x0, int16_t y0,
                            int16_t x1, int16_t y1,
                            int16_t x2, int16_t y2,
                            uint16_t color){
  if (y0 > y1){ swap_i16(&y0, &y1); swap_i16(&x0, &x1); }
  if (y1 > y2){ swap_i16(&y1, &y2); swap_i16(&x1, &x2); }
  if (y0 > y1){ swap_i16(&y0, &y1); swap_i16(&x0, &x1); }

  if (y0 == y2){
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

  int16_t last = (y1 == y2) ? y1 : (y1 - 1);
  for (y = y0; y <= last; y++) {
    int16_t a = x0;
    int16_t b = x0;
    if (dy01) a += (int16_t)(sa / dy01);
    if (dy02) b += (int16_t)(sb / dy02);
    sa += dx01;
    sb += dx02;
    if (a > b){ int16_t t = a; a = b; b = t; }
    st7735_draw_line(d, a, y, b, y, color);
  }

  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for (; y <= y2; y++) {
    int16_t a = x1;
    int16_t b = x0;
    if (dy12) a += (int16_t)(sa / dy12);
    if (dy02) b += (int16_t)(sb / dy02);
    sa += dx12;
    sb += dx02;
    if (a > b){ int16_t t = a; a = b; b = t; }
    st7735_draw_line(d, a, y, b, y, color);
  }
}
