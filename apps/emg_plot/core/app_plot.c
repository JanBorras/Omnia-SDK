#include "app_plot.h"
#include "omnia_port.h"   // per millis()
#include <stdio.h>
#include <string.h>

/* -------------------------------------------------------------------
 * Helpers visuals
 * ------------------------------------------------------------------- */
static void app_plot_paint_gradient(app_plot_t *p)
{
    ST7735_t *lcd = p->lcd;

    for (uint16_t y = 0; y < lcd->height; ++y) {
        uint8_t g = (uint8_t)((y * 40U) / lcd->height);  // 0..40
        uint16_t col = rgb565(g, g, g);
        st7735_draw_line(lcd, 0, y, lcd->width - 1U, y, col);
    }
}

static void app_plot_init_frame(app_plot_t *p)
{
    ST7735_t *lcd = p->lcd;

    app_plot_paint_gradient(p);

    uint16_t banner_h = 28;
    st7735_filled_rectangle(lcd, 0, 0, lcd->width, banner_h, rgb565(0, 40, 90));

    const char *title = "OMNIA SDK";
    uint16_t title_x = (lcd->width - (uint16_t)strlen(title) * 6U) / 2U;
    st7735_draw_string(lcd, title_x, 6, title, WHITE, 1);

    const char *sub = "EMG ANALYTICS";
    uint16_t sub_x = (lcd->width - (uint16_t)strlen(sub) * 6U) / 2U;
    st7735_draw_string(lcd, sub_x, 16, sub, CYAN, 1);

    st7735_draw_line(lcd, 10, banner_h + 2, lcd->width - 10, banner_h + 2,
                     rgb565(180, 180, 255));

    st7735_draw_circle(lcd, lcd->width / 2U, lcd->height - 10U, 30,
                       rgb565(50, 80, 150));

    const char *hint = "Prem CAL";
    uint16_t hint_x = (lcd->width - (uint16_t)strlen(hint) * 6U) / 2U;
    st7735_draw_string(lcd, hint_x, lcd->height - 18U, hint, YELLOW, 1);
}

static inline uint64_t now_ms(void)
{
    omnia_port_t* port = omnia_port_get();
    if (port && port->v && port->v->millis) return port->v->millis();
    return 0;
}

/* -------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------- */
void app_plot_init(app_plot_t *p, ST7735_t *lcd)
{
    if (!p || !lcd) return;

    p->lcd = lcd;
    p->x   = 0;

    p->info_period_ms = 100;     // números a 10 Hz (canviable)
    p->last_info_ms   = 0;

    p->last_raw[0] = '\0';
    p->last_v[0]   = '\0';
    p->last_c[0]   = '\0';
    p->last_n[0]   = '\0';

    p->last_is_calibrating = 0;
    p->last_has_baseline   = 0;
    p->first_frame         = 1;

    /* Pinta el frame inicial ja aquí (no depenguis de draw_info) */
    app_plot_init_frame(p);
    p->first_frame = 0;
}

void app_plot_draw(app_plot_t *p, float norm, int saturated)
{
    if (!p || !p->lcd) return;

    if (norm >  1.f) norm =  1.f;
    if (norm < -1.f) norm = -1.f;

    uint16_t plot_top    = 34;
    uint16_t plot_bottom = p->lcd->height - 1U;
    uint16_t plot_h      = plot_bottom - plot_top;
    uint16_t mid         = plot_top + plot_h / 2U;

    uint16_t y = mid - (uint16_t)(norm * (float)(plot_h/2U - 2U));
    uint16_t color = saturated ? RED : CYAN;

    /* Neteja columna restaurant degradat */
    for (uint16_t yy = plot_top; yy <= plot_bottom; ++yy) {
        uint8_t g = (uint8_t)((yy * 40U) / p->lcd->height);
        uint16_t col_bg = rgb565(g, g, g);
        st7735_draw_pixel(p->lcd, p->x, yy, col_bg);
    }

    if (y >= plot_top && y <= plot_bottom) {
        st7735_draw_pixel(p->lcd, p->x, y, color);
    }

    p->x++;
    if (p->x >= p->lcd->width) p->x = 0;
}

void app_plot_draw_info(app_plot_t *p,
                        uint16_t raw, float volts,
                        float centered, float norm,
                        int saturated,
                        int is_calibrating,
                        int has_baseline)
{
    if (!p || !p->lcd) return;

    /* Throttle temporal (això és clau amb blocs) */
    uint64_t t = now_ms();
    if (p->last_info_ms != 0 && (t - p->last_info_ms) < p->info_period_ms) {
        return;
    }
    p->last_info_ms = t;

    /* ---------------------------------------------------------
       Estat IDLE: no baseline i no calibrant → mantenim frame “Prem CAL”
       --------------------------------------------------------- */
    if (!has_baseline && !is_calibrating) {
        /* Si venim d’un altre estat, repintem el frame sencer */
        if (p->last_has_baseline || p->last_is_calibrating) {
            app_plot_init_frame(p);
            p->x = 0;

            p->last_raw[0] = '\0';
            p->last_v[0]   = '\0';
            p->last_c[0]   = '\0';
            p->last_n[0]   = '\0';
        }

        p->last_is_calibrating = 0;
        p->last_has_baseline   = 0;
        return;
    }

    /* ---------------------------------------------------------
       Estat CALIBRANT: neteja total 1 cop quan hi entrem
       --------------------------------------------------------- */
    if (is_calibrating) {
        if (!p->last_is_calibrating) {
            app_plot_paint_gradient(p);
            p->x = 0;

            st7735_filled_rectangle(p->lcd, 0, 0, p->lcd->width, 18, BLACK);
            st7735_draw_string(p->lcd, 2, 4, "Calibrant...", CYAN, 1);

            st7735_filled_rectangle(p->lcd, p->lcd->width - 30, 0, 30, 18, MAGENTA);
            st7735_draw_string(p->lcd, p->lcd->width - 28, 4, "CAL", WHITE, 1);

            p->last_raw[0] = '\0';
            p->last_v[0]   = '\0';
            p->last_c[0]   = '\0';
            p->last_n[0]   = '\0';
        }

        p->last_is_calibrating = 1;
        p->last_has_baseline   = has_baseline;
        return;
    }

    /* ---------------------------------------------------------
       Estat RUN: tenim baseline → números
       --------------------------------------------------------- */
    if (!p->last_has_baseline || p->last_is_calibrating) {
        st7735_filled_rectangle(p->lcd, 0, 0, p->lcd->width, 18, BLACK);

        p->last_raw[0] = '\0';
        p->last_v[0]   = '\0';
        p->last_c[0]   = '\0';
        p->last_n[0]   = '\0';
    }

    char buf_raw[16];
    char buf_v[16];
    char buf_c[16];
    char buf_n[16];

    snprintf(buf_raw, sizeof(buf_raw), "RAW:%4u", raw);
    snprintf(buf_v,   sizeof(buf_v),   "V:%.2f",  volts);
    snprintf(buf_c,   sizeof(buf_c),   "C:%.2f",  centered);
    snprintf(buf_n,   sizeof(buf_n),   "N:%.2f",  norm);

    if (strcmp(buf_raw, p->last_raw) != 0) {
        st7735_filled_rectangle(p->lcd, 0, 0, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 2, 2, buf_raw, WHITE, 1);
        strcpy(p->last_raw, buf_raw);
    }

    if (strcmp(buf_v, p->last_v) != 0) {
        st7735_filled_rectangle(p->lcd, 64, 0, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 70, 2, buf_v, GREEN, 1);
        strcpy(p->last_v, buf_v);
    }

    if (strcmp(buf_c, p->last_c) != 0) {
        st7735_filled_rectangle(p->lcd, 0, 10, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 2, 12, buf_c, YELLOW, 1);
        strcpy(p->last_c, buf_c);
    }

    if (strcmp(buf_n, p->last_n) != 0) {
        st7735_filled_rectangle(p->lcd, 64, 10, 64, 10, BLACK);
        st7735_draw_string(p->lcd, 70, 12, buf_n, saturated ? RED : CYAN, 1);
        strcpy(p->last_n, buf_n);
    }

    p->last_is_calibrating = 0;
    p->last_has_baseline   = 1;
}
