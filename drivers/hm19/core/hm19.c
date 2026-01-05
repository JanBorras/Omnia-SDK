#include "hm19.h"
#include <string.h>

static inline omnia_port_t* hm19_port(void) { return omnia_port_get(); }

static hm19_config_t hm19_default_cfg(void)
{
  hm19_config_t c;
  c.flags           = (HM19_F_USE_MUTEX | HM19_F_DRAIN_RX_BEFORE_AT | HM19_F_CHUNK_TX);
  c.at_eol          = HM19_AT_EOL_CRLF;
  c.en_pol          = HM19_EN_ACTIVE_HIGH;
  c.boot_delay_ms   = 100;
  c.drain_budget_ms = 20;
  c.tx_chunk_size   = 64;
  c.max_at_line     = 128;
  return c;
}

static inline uint64_t hm19_now_ms(void)
{
  omnia_port_t* p = hm19_port();
  if (!p || !p->v || !p->v->millis) return 0;
  return p->v->millis();
}

static inline void hm19_delay_ms(uint32_t ms)
{
  omnia_port_t* port = hm19_port();
  if (!port || !port->v || !port->v->delay_us) return;
  while (ms--) port->v->delay_us(1000);
}

static inline void hm19_gpio_write(omnia_gpio_t pin, int level)
{
  omnia_port_t* port = hm19_port();
  if (!port || !port->v || !port->v->gpio_write) return;
  (void)port->v->gpio_write(pin, level);
}

static inline int hm19_gpio_read(omnia_gpio_t pin)
{
  omnia_port_t* port = hm19_port();
  if (!port || !port->v || !port->v->gpio_read) return -1;
  return port->v->gpio_read(pin);
}

static inline int hm19_en_level_on(const hm19_t* dev)
{
  return (dev && dev->cfg.en_pol == HM19_EN_ACTIVE_LOW) ? 0 : 1;
}

static inline int hm19_en_level_off(const hm19_t* dev)
{
  return (dev && dev->cfg.en_pol == HM19_EN_ACTIVE_LOW) ? 1 : 0;
}

/* ---------------- Mutex ---------------- */

static inline void hm19_lock(hm19_t* dev)
{
  if (!dev) return;
  if (!(dev->cfg.flags & HM19_F_USE_MUTEX)) return;

  omnia_port_t* port = hm19_port();
  if (!port || !port->v) return;
  if (dev->lock && port->v->mutex_lock) port->v->mutex_lock(dev->lock);
}

static inline void hm19_unlock(hm19_t* dev)
{
  if (!dev) return;
  if (!(dev->cfg.flags & HM19_F_USE_MUTEX)) return;

  omnia_port_t* port = hm19_port();
  if (!port || !port->v) return;
  if (dev->lock && port->v->mutex_unlock) port->v->mutex_unlock(dev->lock);
}

/* ---------------- Connected ---------------- */

int hm19_is_connected(hm19_t* dev)
{
  if (!dev) return -1;
  if (!(dev->cfg.flags & HM19_F_CHECK_CONNECTED)) return -1;
  if (!dev->pin_state) return -1;

  int v = hm19_gpio_read(dev->pin_state);
  if (v < 0) return -1;
  return (v != 0) ? 1 : 0;
}

/* ---------------- HW reset ---------------- */

// static void hm19_hw_reset(hm19_t* dev)
// {
//   if (!dev) return;

//   uint32_t boot_ms = dev->cfg.boot_delay_ms ? dev->cfg.boot_delay_ms : 100;

//   if (!dev->pin_en) {
//     hm19_delay_ms(boot_ms);
//     return;
//   }

//   const int on  = hm19_en_level_on(dev);
//   const int off = hm19_en_level_off(dev);

//   hm19_gpio_write(dev->pin_en, off);
//   hm19_delay_ms(10);
//   hm19_gpio_write(dev->pin_en, on);
//   hm19_delay_ms(boot_ms);
// }

static void hm19_hw_reset(hm19_t* dev)
{
  if (!dev) return;
  uint32_t boot_ms = dev->cfg.boot_delay_ms ? dev->cfg.boot_delay_ms : 100;
  hm19_delay_ms(boot_ms);
}

/* ---------------- TX core (NO LOCK) ---------------- */

static omnia_status_t hm19_uart_write_all_nolock(hm19_t* dev,
                                                const uint8_t* data,
                                                size_t len,
                                                uint32_t timeout_ms)
{
  if (!dev || !dev->uart || !data || len == 0) return OMNIA_EINVAL;

  if (dev->cfg.flags & HM19_F_DROP_IF_NOT_CONNECTED) {
    int c = hm19_is_connected(dev);
    if (c == 0) return OMNIA_EIO;
  }

  if (!(dev->cfg.flags & HM19_F_CHUNK_TX)) {
    return omnia_uart_write(dev->uart, data, len, timeout_ms);
  }

  uint16_t chunk = dev->cfg.tx_chunk_size ? dev->cfg.tx_chunk_size : 64;

  size_t off = 0;
  while (off < len) {
    size_t n = len - off;
    if (n > chunk) n = chunk;

    omnia_status_t st = omnia_uart_write(dev->uart, data + off, n, timeout_ms);
    if (st != OMNIA_OK) return st;

    off += n;
  }
  return OMNIA_OK;
}

static omnia_status_t hm19_write_eol_nolock(hm19_t* dev, uint32_t timeout_ms)
{
  if (!dev) return OMNIA_EINVAL;

  uint8_t eol[2];
  size_t n = 0;

  if (dev->cfg.at_eol == HM19_AT_EOL_CRLF) {
    eol[n++] = '\r';
    eol[n++] = '\n';
  } else {
    eol[n++] = '\n';
  }

  return hm19_uart_write_all_nolock(dev, eol, n, timeout_ms);
}

static omnia_status_t hm19_send_line_nolock(hm19_t* dev,
                                            const char* line,
                                            uint32_t timeout_ms)
{
  if (!dev || !line) return OMNIA_EINVAL;

  const size_t len = strlen(line);
  if (len > 0) {
    omnia_status_t st = hm19_uart_write_all_nolock(dev, (const uint8_t*)line, len, timeout_ms);
    if (st != OMNIA_OK) return st;
  }
  return hm19_write_eol_nolock(dev, timeout_ms);
}

/* ---------------- Drain RX (NO LOCK, time-based) ---------------- */

static void hm19_drain_rx_nolock(hm19_t* dev)
{
  if (!dev || !dev->uart) return;
  if (!(dev->cfg.flags & HM19_F_DRAIN_RX_BEFORE_AT)) return;

  uint32_t budget_ms = dev->cfg.drain_budget_ms ? dev->cfg.drain_budget_ms : 20;
  uint64_t t0 = hm19_now_ms();

  uint8_t ch = 0;

  while (1) {
    /* si millis() no existeix, fem un màxim d’iteracions conservador */
    if (t0 != 0) {
      uint64_t now = hm19_now_ms();
      if (now >= t0 && (now - t0) >= budget_ms) break;
    } else {
      if (budget_ms-- == 0) break;
    }

    /* read 1 byte amb timeout 1ms */
    omnia_status_t st = omnia_uart_read(dev->uart, &ch, 1, 1);
    if (st != OMNIA_OK) break;
  }
}

/* ---------------- Read line (NO LOCK) ---------------- */

static omnia_status_t hm19_read_line_nolock(hm19_t* dev,
                                           char* buf,
                                           size_t buf_size,
                                           uint32_t timeout_ms)
{
  if (!dev || !dev->uart || !buf || buf_size == 0) return OMNIA_EINVAL;

  size_t idx = 0;

  while (idx + 1 < buf_size) {
    uint8_t ch = 0;
    omnia_status_t st = omnia_uart_read(dev->uart, &ch, 1, timeout_ms);
    if (st != OMNIA_OK) break;

    if (ch == '\r' || ch == '\n') {
      if (idx == 0) continue;
      break;
    }

    buf[idx++] = (char)ch;
  }

  buf[idx] = '\0';
  return (idx > 0) ? OMNIA_OK : OMNIA_EIO;
}

/* ---------------- Public API ---------------- */

omnia_status_t hm19_init(hm19_t* dev,
                         const omnia_uart_handle_t* uart,
                         omnia_gpio_t pin_en,
                         omnia_gpio_t pin_state,
                         const hm19_config_t* cfg)
{
  if (!dev || !uart) return OMNIA_EINVAL;

  memset(dev, 0, sizeof(*dev));

  dev->uart      = uart;
  dev->pin_en    = pin_en;
  dev->pin_state = pin_state;
  dev->mode      = HM19_MODE_DATA;

  dev->cfg = cfg ? *cfg : hm19_default_cfg();

  dev->lock = NULL;
  if (dev->cfg.flags & HM19_F_USE_MUTEX) {
    omnia_port_t* port = hm19_port();
    if (port && port->v && port->v->mutex_create) {
      dev->lock = port->v->mutex_create();
    } else {
      dev->cfg.flags &= ~HM19_F_USE_MUTEX;
    }
  }

  hm19_hw_reset(dev);

  /* Ping AT (best-effort) */
  {
    // char r[16] = {0};
    // (void)hm19_send_at(dev, "AT", r, sizeof(r), 200);
    char r[32] = {0};
    (void)hm19_send_at_raw_noeol(dev, "AT", r, sizeof(r), 200);
  }

  return OMNIA_OK;
}

void hm19_deinit(hm19_t* dev)
{
  if (!dev) return;

  omnia_port_t* port = hm19_port();
  if (dev->lock && port && port->v && port->v->mutex_destroy) {
    port->v->mutex_destroy(dev->lock);
  }
  dev->lock = NULL;
}

void hm19_set_at_eol(hm19_t* dev, hm19_at_eol_t eol) { if (dev) dev->cfg.at_eol = eol; }
void hm19_set_en_polarity(hm19_t* dev, hm19_en_polarity_t pol) { if (dev) dev->cfg.en_pol = pol; }
void hm19_set_boot_delay_ms(hm19_t* dev, uint32_t ms) { if (dev) dev->cfg.boot_delay_ms = ms; }
void hm19_set_flags(hm19_t* dev, uint32_t flags) { if (dev) dev->cfg.flags = flags; }
void hm19_set_tx_chunk(hm19_t* dev, uint16_t chunk_size) { if (dev) dev->cfg.tx_chunk_size = chunk_size; }
void hm19_set_drain_budget(hm19_t* dev, uint32_t budget_ms) { if (dev) dev->cfg.drain_budget_ms = budget_ms; }

omnia_status_t hm19_send_raw(hm19_t* dev,
                            const uint8_t* data,
                            size_t len,
                            uint32_t timeout_ms)
{
  if (!dev || !dev->uart || !data || len == 0) return OMNIA_EINVAL;

  hm19_lock(dev);
  omnia_status_t st = hm19_uart_write_all_nolock(dev, data, len, timeout_ms);
  hm19_unlock(dev);
  return st;
}

omnia_status_t hm19_send_line(hm19_t* dev,
                              const char* line,
                              uint32_t timeout_ms)
{
  if (!dev || !line) return OMNIA_EINVAL;

  hm19_lock(dev);
  omnia_status_t st = hm19_send_line_nolock(dev, line, timeout_ms);
  hm19_unlock(dev);
  return st;
}

omnia_status_t hm19_send_at(hm19_t* dev,
                            const char* cmd,
                            char* resp,
                            size_t resp_size,
                            uint32_t timeout_ms)
{
  if (!dev || !dev->uart || !cmd) return OMNIA_EINVAL;

  hm19_lock(dev);

  hm19_drain_rx_nolock(dev);

  omnia_status_t st = hm19_send_line_nolock(dev, cmd, timeout_ms);
  if (st != OMNIA_OK) {
    hm19_unlock(dev);
    return st;
  }

  if (!resp || resp_size == 0) {
    hm19_unlock(dev);
    return OMNIA_OK;
  }

  st = hm19_read_line_nolock(dev, resp, resp_size, timeout_ms);

  hm19_unlock(dev);
  return st;
}

static omnia_status_t hm19_send_at_raw_noeol(hm19_t* dev,
                                            const char* cmd,
                                            char* resp,
                                            size_t resp_size,
                                            uint32_t timeout_ms)
{
  if (!dev || !dev->uart || !cmd) return OMNIA_EINVAL;

  hm19_lock(dev);
  hm19_drain_rx_nolock(dev);

  omnia_status_t st = hm19_uart_write_all_nolock(dev, (const uint8_t*)cmd, strlen(cmd), timeout_ms);
  if (st != OMNIA_OK) { hm19_unlock(dev); return st; }

  if (!resp || resp_size == 0) { hm19_unlock(dev); return OMNIA_OK; }

  st = hm19_read_line_nolock(dev, resp, resp_size, timeout_ms);
  hm19_unlock(dev);
  return st;
}


int hm19_send_at_multi(hm19_t* dev,
                       const char* cmd,
                       char* out_buf,
                       size_t out_size,
                       uint8_t max_lines,
                       uint32_t timeout_ms)
{
  if (!dev || !dev->uart || !cmd || !out_buf || out_size == 0 || max_lines == 0) {
    return -1;
  }

  hm19_lock(dev);

  hm19_drain_rx_nolock(dev);

  omnia_status_t st = hm19_send_line_nolock(dev, cmd, timeout_ms);
  if (st != OMNIA_OK) {
    hm19_unlock(dev);
    return -2;
  }

  size_t out_idx = 0;
  uint8_t lines = 0;

  uint16_t tmp_cap = dev->cfg.max_at_line ? dev->cfg.max_at_line : 128;
  char tmp[512];

  if (tmp_cap > (uint16_t)sizeof(tmp)) tmp_cap = (uint16_t)sizeof(tmp);

  while (lines < max_lines) {
    tmp[0] = '\0';
    st = hm19_read_line_nolock(dev, tmp, tmp_cap, timeout_ms);
    if (st != OMNIA_OK) break;

    size_t L = strlen(tmp);
    if (L == 0) continue;

    if (out_idx + L + 2 > out_size) break;

    memcpy(out_buf + out_idx, tmp, L);
    out_idx += L;
    out_buf[out_idx++] = '\n';
    out_buf[out_idx] = '\0';

    lines++;
  }

  hm19_unlock(dev);
  return (int)lines;
}
