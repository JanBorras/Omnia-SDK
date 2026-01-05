cmake_minimum_required(VERSION 3.22)

project(omnia C CXX)

# ---------------- Standards ----------------
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# TFLM backend is C++, keep it predictable
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------- Opcions globals ----------------
set(OMNIA_PORT "stm32h755" CACHE STRING "stm32h755")
option(OMNIA_ENABLE_HM19       "Enable HM19 BLE driver" OFF)
option(OMNIA_ENABLE_EMG_STREAM "Enable EMG stream layer" OFF)
option(OMNIA_ENABLE_EMG_PACKET "Enable EMG packet protocol" ON)
option(OMNIA_ENABLE_APPS       "Build Omnia apps" ON)

# AI layer
option(OMNIA_ENABLE_AI   "Enable Omnia AI contract layer" ON)
option(OMNIA_ENABLE_TFLM "Enable TFLM backend for Omnia AI" ON)

message(STATUS "OMNIA_PORT = ${OMNIA_PORT}")
message(STATUS "OMNIA_ENABLE_AI = ${OMNIA_ENABLE_AI}")
message(STATUS "OMNIA_ENABLE_TFLM = ${OMNIA_ENABLE_TFLM}")

# =================================================
# Port core
# =================================================
add_library(omnia_port STATIC
  core/omnia_port_default.c
  core/omnia_port_ex.c
)
target_include_directories(omnia_port PUBLIC include)

# ---------------- HAL port ----------------
if(OMNIA_PORT STREQUAL "stm32h755")
  target_sources(omnia_port PRIVATE
    ports/stm32h7/src/port_stm32.c
  )
  target_include_directories(omnia_port PUBLIC
    ports/stm32h7/inc
  )
  target_compile_definitions(omnia_port PUBLIC
    OMNIA_PORT_STM32H755=1
  )
elseif(OMNIA_PORT STREQUAL "esp32")
  target_sources(omnia_port PRIVATE
    ports/esp32/port_esp32.c
    ports/esp32/hal_esp32.c
  )
  target_include_directories(omnia_port PUBLIC
    ports/esp32
  )
  target_compile_definitions(omnia_port PUBLIC
    OMNIA_PORT_ESP32=1
  )
elseif(OMNIA_PORT STREQUAL "msp430")
  target_sources(omnia_port PRIVATE
    ports/msp430/port_msp430.c
  )
  target_include_directories(omnia_port PUBLIC
    ports/msp430
  )
  target_compile_definitions(omnia_port PUBLIC
    OMNIA_PORT_MSP430=1
  )
else()
  message(FATAL_ERROR "Unsupported OMNIA_PORT=${OMNIA_PORT}")
endif()

# =================================================
# AI Contract (wrapper) + TFLM backend
# =================================================

if(OMNIA_ENABLE_AI)
  add_library(omnia_ai STATIC
    core/omnia_ai.c
  )

  # Public includes: apps only need include/omnia_ai.h
  target_include_directories(omnia_ai PUBLIC
    include
  )

  # Private includes: internal header is not for apps
  target_include_directories(omnia_ai PRIVATE
    include   # contains omnia_ai_internal.h in your tree; keep PRIVATE usage in code discipline
  )

  target_link_libraries(omnia_ai PUBLIC
    omnia_port
  )
endif()

if(OMNIA_ENABLE_AI AND OMNIA_ENABLE_TFLM)
  add_library(omnia_ai_tflm STATIC
    backends/tflm/core/omnia_ai_tflm_backend.cc
  )

  target_include_directories(omnia_ai_tflm PUBLIC
    include
    backends/tflm/include
  )

  target_include_directories(omnia_ai_tflm PRIVATE
    include   # omnia_ai_internal.h (internal helpers)
  )

  target_link_libraries(omnia_ai_tflm PUBLIC
    omnia_ai
  )

  # Embedded C++ sane defaults
  target_compile_options(omnia_ai_tflm PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions -fno-rtti -fno-threadsafe-statics>
  )

  # ---- TFLM link hook ----
  # If you have TFLM added elsewhere as a target, link it here.
  # Example:
  #   add_subdirectory(third_party/tflm tflm_build)
  #   target_link_libraries(omnia_ai_tflm PUBLIC tflm)
  #
  # Or if an existing target is already defined:
  #   target_link_libraries(omnia_ai_tflm PUBLIC tflm)

endif()

# =================================================
# Drivers
# =================================================

add_library(st7735 STATIC
  drivers/st7735/core/st7735.c
  drivers/st7735/core/fonts.c
)
target_include_directories(st7735 PUBLIC
  include
  drivers/st7735/include
)
target_link_libraries(st7735 PUBLIC omnia_port)

add_library(sen0240 STATIC
  drivers/sen0240/core/sen0240.c
)
target_include_directories(sen0240 PUBLIC
  include
  drivers/sen0240/include
)
target_link_libraries(sen0240 PUBLIC omnia_port)

# ---------------- HM19 ----------------
if(OMNIA_ENABLE_HM19)
  add_library(hm19 STATIC
    drivers/hm19/core/hm19.c
  )
  target_include_directories(hm19 PUBLIC
    include
    drivers/hm19/include
  )
  target_link_libraries(hm19 PUBLIC omnia_port)
endif()

# =================================================
# Protocols
# =================================================

if(OMNIA_ENABLE_EMG_PACKET)
  add_library(emg_packet STATIC
    protocols/emg_packet/core/emg_packet.c
  )
  target_include_directories(emg_packet PUBLIC
    include
    protocols/emg_packet/include
  )
endif()

# =================================================
# Streams
# =================================================

# adc_stream (always safe to build; doesn't imply EMG streaming)
add_library(adc_stream STATIC
  streams/adc_stream/core/adc_stream.c
)
target_include_directories(adc_stream PUBLIC
  include
  streams/adc_stream/include
)
target_link_libraries(adc_stream PUBLIC
  omnia_port
)

if(OMNIA_ENABLE_EMG_STREAM)
  add_library(emg_stream STATIC
    streams/emg_stream/core/emg_stream.c
  )
  target_include_directories(emg_stream PUBLIC
    include
    streams/emg_stream/include
  )

  target_link_libraries(emg_stream PUBLIC
    omnia_port
    emg_packet
    adc_stream
  )

  if(OMNIA_ENABLE_HM19)
    target_link_libraries(emg_stream PUBLIC hm19)
  endif()
endif()

# =================================================
# Apps
# =================================================

if(OMNIA_ENABLE_APPS)
  add_library(app_emg_plot STATIC
    apps/emg_plot/core/app_emg.c
    apps/emg_plot/core/app_plot.c
    apps/emg_plot/core/app_emg_stream.c
  )
  target_include_directories(app_emg_plot PUBLIC
    include
    apps/emg_plot/include
  )

  target_link_libraries(app_emg_plot PUBLIC
    st7735
    sen0240
    omnia_port
    emg_packet
    adc_stream
    m
  )

  if(OMNIA_ENABLE_EMG_STREAM)
    target_link_libraries(app_emg_plot PUBLIC emg_stream)
  endif()

  # Optional: if this app will run inference, link AI libs
  if(OMNIA_ENABLE_AI)
    target_link_libraries(app_emg_plot PUBLIC omnia_ai)
  endif()
  if(OMNIA_ENABLE_AI AND OMNIA_ENABLE_TFLM)
    target_link_libraries(app_emg_plot PUBLIC omnia_ai_tflm)
  endif()
endif()

# =================================================
# Omnia all-in-one (umbrella)
# =================================================
add_library(omnia_all INTERFACE)

target_link_libraries(omnia_all INTERFACE
  omnia_port
  st7735
  sen0240
  adc_stream
)

# Optional math umbrella
target_link_libraries(omnia_all INTERFACE m)

if(OMNIA_ENABLE_HM19)
  target_link_libraries(omnia_all INTERFACE hm19)
endif()

if(OMNIA_ENABLE_EMG_PACKET)
  target_link_libraries(omnia_all INTERFACE emg_packet)
endif()

if(OMNIA_ENABLE_EMG_STREAM)
  target_link_libraries(omnia_all INTERFACE emg_stream)
endif()

if(OMNIA_ENABLE_APPS)
  target_link_libraries(omnia_all INTERFACE app_emg_plot)
endif()

if(OMNIA_ENABLE_AI)
  target_link_libraries(omnia_all INTERFACE omnia_ai)
endif()

if(OMNIA_ENABLE_AI AND OMNIA_ENABLE_TFLM)
  target_link_libraries(omnia_all INTERFACE omnia_ai_tflm)
endif()
