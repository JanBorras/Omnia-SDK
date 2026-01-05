# Port README (Omnia SDK) — Quick Start

This is the **practical checklist** for implementing a new Omnia platform port.
It complements `docs/port_contract.md`.

The philosophy is simple: **common code must not know your hardware**.
Your port is the only place allowed to touch STM32 HAL, ESP-IDF, registers, RTOS primitives, etc.

---

## 0) Where things live (recommended repo layout)

```
include/
  omnia_port.h
  omnia_adc.h
  omnia_uart.h
drivers/
  sen0240/
  hm19/
services/
  emg_stream/
ports/
  stm32/
    port_stm32.c
    port_stm32.h
  esp32/
    port_esp32.c
    port_esp32.h
docs/
  port_contract.md
  port_README.md   <-- this file
apps/
  app_emg.c
```

---

## 1) The minimum viable port (to run something)

You need to provide at least:

- `delay_us(us)`
- `millis()`

Then you can run basic logic and test scaffolding.

---

## 2) To run the EMG prototype (SEN0240 + streaming)

You must implement:

### ADC
- `adc_read(dev, &out)`  (blocking, 1 sample)
- `adc_read_n(dev, buf, n)` (blocking, fill exactly `n` samples)

### UART (for HM-19 BLE bridge)
- `uart_write(uart, data, len, timeout_ms)` (blocking: OK => all bytes sent)
- `uart_read(uart, data, len, timeout_ms)`  (blocking: OK => all bytes read)

### Time
- `millis()`
- `delay_us(us)`

Optional but recommended:
- `gpio_read/write` (STATE / EN pins for HM-19)
- `mutex_*` (to serialize UART/BLE transactions)
- `log()` for debugging

---

## 3) STM32H755 (HAL) — Implementation Notes

### 3.1 Handles (opaque types)
Typical mapping:
- `omnia_adc_t`  -> `ADC_HandleTypeDef*`
- `omnia_uart_t` -> `UART_HandleTypeDef*`
- `omnia_gpio_t` -> pointer to a small struct `{GPIO_TypeDef* port; uint16_t pin;}`

### 3.2 ADC single sample (`adc_read`)
Fast & simple (blocking):
- Start conversion
- Poll for completion
- Read value

If using DMA circular capture, `adc_read` may read the latest sample from the DMA ring.

### 3.3 ADC block (`adc_read_n`) — blocking
Two common approaches:

**A) Short path (works now):**
- Loop `n` times calling the HAL blocking read.

**B) Better (recommended):**
- Run ADC with timer trigger + DMA circular.
- Maintain a software read index into the DMA ring.
- `adc_read_n` blocks until `n` new samples are available, then copies them out.

### 3.4 Timing
- `millis()` can use `HAL_GetTick()` (1ms SysTick) or a monotonic timer.
- `delay_us()` can use a hardware timer or DWT cycle counter (preferred).

### 3.5 Dual-core warning (H755)
Decide which core owns the peripherals:
- If M4 owns ADC/DMA/UART, keep it there.
- If M7 reads DMA buffers, handle cache coherency (MPU / cache invalidate).

For the first prototype: prefer a simpler ownership model (one core does capture + streaming).

---

## 4) ESP32 (ESP-IDF) — Implementation Notes

### 4.1 Handles
Typical mapping:
- `omnia_adc_t`  -> ADC continuous driver handle or an internal context struct
- `omnia_uart_t` -> UART port number or ESP-IDF UART handle

### 4.2 ADC block capture (`adc_read_n`)
Recommended:
- Use ADC continuous driver (or I2S ADC + DMA) to get stable sampling.
- Accumulate samples until you have `n`, then return `OMNIA_OK`.

### 4.3 UART
Use ESP-IDF UART driver:
- Ensure blocking write semantics match the contract.

### 4.4 Timing
- `millis()` -> `esp_timer_get_time()/1000` or `xTaskGetTickCount()` (careful with tick rate)
- `delay_us()` -> `ets_delay_us()` (best effort) or `esp_rom_delay_us()`

---

## 5) Smoke test checklist (before you touch EMG)

### 5.1 Time
- Print `millis()` increasing.
- Verify `delay_us(1000)` feels like ~1ms (rough).

### 5.2 UART
- Loopback test (TX->RX) or echo test:
  - write N bytes
  - read N bytes
  - compare

### 5.3 ADC
- `adc_read`: read 100 samples, confirm they change with input.
- `adc_read_n`: read blocks of 32, confirm it always returns full blocks.

---

## 6) Recommended prototype settings (EMG @ 2kHz)

Start with:
- `sample_rate_hz = 2000`
- `block_n = 32` or `64`
- `emg_packet frame_samples = 32`
- ring buffer in `emg_stream`: 5–10 seconds if RAM allows

Rule:
- If you see queue growth / drops: increase block size or flush budget.
- If you need lower latency: decrease block size.

---

## 7) Common pitfalls

- Returning `OMNIA_OK` without valid output data (breaks drivers silently).
- Partial UART writes on OK (corrupts binary stream).
- `adc_read_n` returning fewer than `n` samples on OK (breaks EMG framing).
- Doing BLE/UART transmit inside ISR/callback (creates jitter and deadlocks).
- On STM32H7 M7: DMA buffer in cached RAM without cache maintenance.

---

## 8) Definition of “Done” for a new port

A port is considered complete for the EMG prototype when:
- `sen0240_read_i16()` works repeatedly without errors
- `emg_stream` queues and flushes without runaway drops
- PC receives packets with continuous `SEQ` (no gaps under normal conditions)

If these are true: your port is real.
