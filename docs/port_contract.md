# Port Contract (Omnia SDK)

This document defines the **platform port contract** for Omnia SDK.  
Anything outside the platform-specific port **must rely only on this contract**.

The goal is brutal clarity: if a port respects these rules, **all common code** (drivers, services, apps) should run unchanged across platforms (STM32, ESP32, …).

---

## 1) Vocabulary

- **Port**: the platform-specific implementation that fills `omnia_port_vtable_t`.
- **Common code**: everything that is *not* in the platform port (drivers like `sen0240`, `hm19`, services like `emg_stream`, packet builders, apps).
- **Handles are opaque**: `omnia_gpio_t`, `omnia_adc_t`, `omnia_uart_t`, etc. are `void*` and platform-defined.

---

## 2) Registration and Lifetime

### 2.1 Registration
The platform **must** call:

- `omnia_port_register(&vtable)` exactly once during boot (before any driver use).

`omnia_port_validate()` should be used in debug builds (or always, if you prefer fail-fast).

### 2.2 Lifetime rules
All opaque handles stored in the port and/or drivers must remain valid for the whole time they are used.

Examples:
- `omnia_adc_t` may point to `ADC_HandleTypeDef*` (STM32 HAL) or an ESP-IDF ADC handle.
- `omnia_uart_t` may point to `UART_HandleTypeDef*` or an ESP-IDF UART port descriptor.

---

## 3) Mandatory vs Optional Functions

### 3.1 Mandatory for a minimal system
A port is considered usable if it provides at least:

- `delay_us(us)`
- `millis()`

Everything else is feature-dependent. Drivers must check availability.

### 3.2 Feature checks
Common code will use:

- `omnia_port_has_adc()` -> expects `v->adc_read != NULL`
- `omnia_port_has_uart()` -> expects `v->uart_write != NULL` (and usually `uart_read`)
- `omnia_port_has_spi()` -> expects SPI functions

Drivers must return `OMNIA_ENOTSUP` when the required backend function is missing.

---

## 4) Return Codes and Error Semantics

The port must use consistent error reporting.

Recommended mapping:
- `OMNIA_OK`: operation completed successfully.
- `OMNIA_EINVAL`: invalid arguments (NULL pointers, n==0, invalid params).
- `OMNIA_ENOTSUP`: feature not implemented in this port.
- `OMNIA_EIO`: hardware/driver failure, transfer error, peripheral not ready, etc.
- If you have timeout-specific codes (e.g. `OMNIA_ETIMEDOUT`), use them consistently.

**Rule:** never return `OMNIA_OK` if output data is invalid.

---

## 5) Timing Contract

### 5.1 `millis()`
- Must be **monotonic non-decreasing**.
- Wrap-around is allowed if using a 32-bit counter, but prefer 64-bit if possible.
- Resolution should be 1 ms or better.

### 5.2 `delay_us(us)`
- Best effort is acceptable.
- Must block the caller for approximately `us` microseconds.
- Must not crash if called with `us == 0`.

---

## 6) UART Contract

### 6.1 `uart_write(uart, data, len, timeout_ms)` (recommended blocking)
**Blocking semantics (required):**
- If it returns `OMNIA_OK`, then **all `len` bytes have been accepted/sent**.
- If it returns an error, common code assumes the transfer **did not complete**.

Timeout:
- If the platform supports timeouts, it must honor `timeout_ms`.
- If it cannot, it may ignore `timeout_ms` but should document this and still behave consistently.

### 6.2 `uart_read(uart, data, len, timeout_ms)` (blocking read)
- If it returns `OMNIA_OK`, common code assumes **`len` bytes were read**.
- If timeout expires before reading all bytes, return an error (preferably a timeout code; otherwise `OMNIA_EIO`).

**Important:** drivers may call `uart_read(..., 1, timeout_ms)` repeatedly to parse lines.

---

## 7) ADC Contract (Critical for EMG)

### 7.1 `adc_read(dev, &out)` (single sample)
**Blocking semantics (required):**
- Must write one valid sample to `*out` and return `OMNIA_OK`.
- If it cannot produce a sample, return an error code.

Sample format:
- Sample must be in the native ADC numeric range (e.g. `0..4095` for 12-bit).
- Common code expects the raw value to match the ADC resolution configured in `omnia_adc_handle_t`.

### 7.2 `adc_read_n(dev, buf, n)` (N samples) — **Blocking**
For the first functional prototype, we choose the shortest path:

✅ **`adc_read_n` is BLOCKING**

**Contract:**
- If it returns `OMNIA_OK`, then **exactly `n` samples** are written into `buf[0..n-1]`.
- If it returns an error, contents of `buf` are undefined (caller must treat them as invalid).

Implementation flexibility:
- STM32: can copy from a DMA circular buffer, or do a blocking loop read, or start a DMA and wait.
- ESP32: can read from ADC continuous/I2S DMA driver, or loop with `adc_read`.

**Note:** For real-time systems, blocking should be bounded. If needed later, we can extend the contract with a non-blocking mode or an `available()` call. For now: keep it simple and deterministic.

---

## 8) GPIO Contract

### 8.1 `gpio_write(pin, level)`
- Must set pin output to `level` (0 or non-zero).
- If pin is invalid or not configured, return an error.

### 8.2 `gpio_read(pin)`
- Must return the logical level (0 or 1).
- If pin is invalid, may return a negative value OR 0 (but document it). Prefer returning 0 and logging errors.

### 8.3 `gpio_mode(pin, is_output, pull)`
- Must configure direction and pull.
- Pull values follow `omnia_gpio_pull_t`.

---

## 9) Mutex and Concurrency (Optional)

Mutex functions are optional. If provided:

- `mutex_create()` returns a valid mutex handle or NULL on failure.
- `mutex_lock()` / `mutex_unlock()` must be safe and consistent.

Common code may use mutexes to serialize UART/BLE transactions.
If mutexes are not supported, common code must still behave (but without concurrency safety).

---

## 10) Memory Allocation (Optional)

If `malloc_fn/free_fn` are provided, drivers/services may use them instead of libc.
If not provided, common code should avoid dynamic allocation or provide a fallback.

---

## 11) Logging (Optional)

`log(level, fmt, ...)` is optional.
If missing, drivers must still function.

---

## 12) Performance Guidelines (Non-binding but practical)

### 12.1 Don’t do heavy work in interrupts
For ADC streaming (DMA or continuous capture):
- ISR/callback should only signal “block ready”.
- Conversion/packetizing/BLE transmit should run in main loop or a task.

### 12.2 Prefer stable sample timing
For EMG:
- Prefer timer-triggered ADC + DMA (STM32),
- or ADC continuous + DMA (ESP32),
to minimize jitter.

---

## 13) Compliance Checklist

A port is compliant if:

- [ ] `omnia_port_register()` called before any driver
- [ ] `millis()` monotonic
- [ ] `delay_us()` works (best effort OK)
- [ ] `uart_write()` returns OK only if all bytes transferred
- [ ] `adc_read()` returns OK only if sample is valid
- [ ] `adc_read_n()` is **blocking** and fills exactly `n` samples on OK
- [ ] Missing features return `OMNIA_ENOTSUP`
- [ ] Handles stay valid during use

---

## 14) Practical Examples

### 14.1 STM32 (DMA circular)
- ADC triggered by timer (TRGO)
- DMA fills circular buffer
- `adc_read_n()` copies `n` samples from the DMA ring (blocking until available if needed)

### 14.2 ESP32 (ADC continuous)
- ADC continuous driver produces frames
- `adc_read_n()` reads until `n` samples accumulated (blocking)

---

## 15) Future Extensions (Not used in prototype)
Possible later improvements:
- Non-blocking ADC reads (`EAGAIN`)
- `adc_available()` and timestamped frames
- Async UART with callbacks or ring buffers
- Power/performance instrumentation hooks

For the first prototype: **keep blocking semantics and move fast**.
