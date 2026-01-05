# Omnia SDK

**Omnia SDK** is a lightweight, portable Edge-AI / TinyML software development kit for microcontrollers.  
It is designed to **decouple application logic from hardware details**, enabling the same codebase to run on different MCU families with minimal adaptation.

The SDK targets real embedded use cases: sensor acquisition, data streaming, logging, and on-device inference, while keeping memory footprint and architectural complexity under control.

---

## Motivation

Current embedded and TinyML ecosystems suffer from fragmentation:

- Vendor-specific HALs and SDKs that lock code to a single platform  
- RTOS-heavy frameworks when only a small subset of features is needed  
- ML runtimes that are portable in theory, but hard to integrate cleanly in practice  

**Omnia SDK** addresses this by providing:
- A **minimal, explicit hardware contract**
- A **small, predictable runtime**
- A **single build system** for multiple targets

The philosophy is to stay **as simple as possible**, but **no simpler than required**.

---

## Design Principles

- **Separation of concerns**  
  Application logic never touches registers, vendor HALs, or board-specific code.

- **Explicit hardware contract**  
  All platform dependencies are isolated behind a stable function table (vtable-style HAL).

- **Minimalism over completeness**  
  No full RTOS unless explicitly required. No hidden allocators. No magic background tasks.

- **Portability by construction**  
  New boards are added by implementing a small, well-defined port layer.

- **TinyML-first mindset**  
  Designed to integrate efficiently with lightweight inference engines such as TensorFlow Lite Micro.

---

## Architecture Overview

At a high level, Omnia SDK is structured as follows:

```
Application
   |
   v
Omnia Core (portable)
   |
   v
Platform Contract (vtable / HAL)
   |
   v
Port Implementation (STM32, ESP32, ...)
   |
   v
Vendor HAL / SDK
```

- **Core**: portable logic, drivers, utilities, ML pipelines  
- **Port**: platform-specific glue code  
- **Vendor HAL**: STM32Cube, ESP-IDF, etc. (used but not exposed)

This structure ensures that **only the port layer changes** when targeting a new MCU.

---

## Features

- Cross-platform embedded SDK written in C  
- Clean hardware abstraction (GPIO, UART, SPI, timing, etc.)  
- Deterministic execution model (bare-metal friendly)  
- Optional integration with RTOS-based systems  
- Ready for TinyML inference workflows  
- CMake-based build system with cross-compilation toolchains  
- Example applications and PC-side utilities  

---

## Supported Platforms

Current and planned support:

- **STM32** (ARM Cortex-M, via STM32Cube HAL)  
- **ESP32** (Xtensa, via ESP-IDF)  
- **Host / PC simulation** (for testing and validation)  

The architecture is extensible to other platforms (RISC-V, MSP430, etc.) by implementing a new port.

---

## TinyML Integration

Omnia SDK is designed to work naturally with **TensorFlow Lite for Microcontrollers (TFLM)**:

- Static memory allocation  
- No dynamic heap dependency  
- Compatible with bare-metal or RTOS environments  
- Easy integration with CMSIS-NN or other optimized backends (when available)  

The SDK does not reimplement ML runtimes.  
Instead, it **integrates proven tools cleanly**, without leaking framework assumptions into application code.

---

## Build System

The project uses **CMake** as a unified build system.

Benefits:
- One project, multiple targets  
- Explicit toolchains per platform  
- Reproducible builds  
- Easy CI integration  

Typical flow:

```bash
mkdir build
cd build
cmake -DOMNIA_PORT=stm32h7 ..
cmake --build .
```

(Exact flags depend on the selected platform and toolchain.)

---

## Repository Structure

```
omnia_sdk/
├── core/           # Portable SDK core
├── ports/          # Platform-specific implementations
├── apps/           # Example applications
├── cmake/          # Toolchains and CMake helpers
├── tools/          # PC-side utilities and scripts
└── CMakeLists.txt
```

---

## When to Use Omnia SDK

Omnia SDK is a good fit if you need:

- A **portable embedded codebase**
- Tight control over memory and execution  
- On-device inference on microcontrollers  
- To avoid vendor lock-in  
- A middle ground between raw bare-metal and a full RTOS stack  

It is **not** intended to replace:
- Full-featured IoT OSes  
- High-level application frameworks  
- Cloud-oriented SDKs  

---

## Project Status

This project is under **active development** and is used as the foundation for academic and experimental Edge-AI work.

APIs may evolve, but the **core architectural principles are stable**.

---

## License

Specify your license here (e.g. MIT, BSD, Apache-2.0).

---

## Philosophy

> *Embedded systems should be predictable, explicit, and honest about their constraints.*
