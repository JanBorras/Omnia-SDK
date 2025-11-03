# EdgeAI SDK Skeleton (ESP32 + STM32H7 friendly)

This is a **minimal skeleton** to illustrate a clean separation between:
- `edge/hal/*` (portable HAL API)
- platform bindings (`src/hal/esp32`, `src/hal/stm32h7`)
- `edge/ml/*` (TFLM facade)
- `apps/solar_demo` (BH1750 + dummy inference)

## Build (host preview)
```bash
mkdir build && cd build
cmake .. 
cmake --build . -j
./apps/solar_demo/solar_demo
```

## Build (ESP32)
Use ESP-IDF and replace this cmake with `idf.py`. Port `edge_*` functions to IDF drivers.

## Build (STM32H7)
Use STM32CubeIDE or arm-none-eabi toolchain and bind `edge_*` to HAL.

## Notes
- TFLM integration is stubbed. Drop real interpreter into `src/ml/tflm_api.c`.
- BH1750 I2C ops are simplified; add proper error handling and delays.
``` 
