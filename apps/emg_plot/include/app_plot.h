#ifndef APP_PLOT_H
#define APP_PLOT_H

#include <stdint.h>
#include "st7735.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    ST7735_t *lcd;
    uint16_t  x;

    /* Throttle d’UI (ms) */
    uint32_t  info_period_ms;     // p.ex. 100 ms
    uint64_t  last_info_ms;

    /* Estat visual */
    uint8_t   first_frame;
    int       last_is_calibrating;
    int       last_has_baseline;

    /* Cache strings per evitar redraw */
    char last_raw[16];
    char last_v[16];
    char last_c[16];
    char last_n[16];
} app_plot_t;

/* Init + pinta frame inicial (degradat + banner + Prem CAL) */
void app_plot_init(app_plot_t *p, ST7735_t *lcd);

/* Dibuixa 1 punt/traça (crida-ho només a ritme d’UI, no a 2kHz) */
void app_plot_draw(app_plot_t *p, float norm, int saturated);

/* Actualitza capçalera (estats + números) amb throttle per ms */
void app_plot_draw_info(app_plot_t *p,
                        uint16_t raw, float volts,
                        float centered, float norm,
                        int saturated,
                        int is_calibrating,
                        int has_baseline);

#ifdef __cplusplus
}
#endif

#endif /* APP_PLOT_H */
