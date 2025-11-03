cmake_minimum_required(VERSION 3.20)
project(EdgeAI_SDK_Skeleton C)

set(CMAKE_C_STANDARD 11)

# Options to select target platform and backends
option(EDGE_TARGET_ESP32 "Build for ESP32 (ESP-IDF)" OFF)
option(EDGE_TARGET_STM32H7 "Build for STM32H7 (CubeHAL)" OFF)
option(USE_TFLM "Enable TensorFlow Lite Micro" ON)
option(USE_CMSIS_NN "Enable CMSIS-NN kernels (STM32 only)" OFF)
option(USE_ESP_NN "Enable ESP-NN kernels (ESP32 only)" OFF)

include_directories(include)

add_subdirectory(src)

add_subdirectory(apps/solar_demo)
