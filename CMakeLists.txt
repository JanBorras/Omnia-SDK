cmake_minimum_required(VERSION 3.22)

# Root CMake for the Omnia SDK (HAL-agnostic drivers + Port/RTOS adapters)
project(omnia C)

# ---- Global compile settings -------------------------------------------------
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Options:
# - OMNIA_PORT: selects the HAL adapter
# - OMNIA_RTOS: selects the RTOS adapter (optional)
option(OMNIA_PORT "HAL port adapter (msp430fr2xx|stm32h755|esp32)" "stm32h755")
option(OMNIA_RTOS "RTOS adapter (none|freertos|zephyr)" "none")

message(STATUS "OMNIA_PORT = ${OMNIA_PORT}")
message(STATUS "OMNIA_RTOS = ${OMNIA_RTOS}")

# ---- Omnia Port (IO contract + optional RTOS ops) ----------------------------
# Core: minimal registry & accessors
add_library(omnia_port STATIC
  core/omnia_port_default.c
)
target_include_directories(omnia_port
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- HAL adapter selection ----------------------------------------------------
# Each adapter must provide an omnia_port_vtable_t (IO) and an init() function
if(OMNIA_PORT STREQUAL "msp430fr2xx")
  target_sources(omnia_port PRIVATE ports/msp430fr2xx/port_msp430.c)
  target_compile_definitions(omnia_port PUBLIC OMNIA_PORT_MSP430FR2XX=1)
elseif(OMNIA_PORT STREQUAL "stm32h755")
  target_sources(omnia_port PRIVATE ports/stm32h755/port_stm32.c)
  target_compile_definitions(omnia_port PUBLIC OMNIA_PORT_STM32H755=1)
  # target_link_libraries(omnia_port PUBLIC stm32hal)  # <-- link your STM32 HAL target here
elseif(OMNIA_PORT STREQUAL "esp32")
  target_sources(omnia_port PRIVATE ports/esp32/port_esp32.c)
  target_compile_definitions(omnia_port PUBLIC OMNIA_PORT_ESP32=1)
  # ESP-IDF typically manages its own build; adapt if integrating with idf.cmake.
else()
  message(FATAL_ERROR "Unknown OMNIA_PORT='${OMNIA_PORT}'. Use: msp430fr2xx|stm32h755|esp32")
endif()

# ---- RTOS adapter selection (optional) ---------------------------------------
# RTOS adds extended ops (tasks, queues, timers, mutex, etc.) without touching drivers
if(OMNIA_RTOS STREQUAL "none")
  target_compile_definitions(omnia_port PUBLIC OMNIA_CAP_RTOS=0)
elseif(OMNIA_RTOS STREQUAL "freertos")
  target_sources(omnia_port PRIVATE
    core/omnia_port_ex.c
    ports/common/rtos_freertos.c
  )
  target_compile_definitions(omnia_port PUBLIC OMNIA_CAP_RTOS=1 OMNIA_RTOS_FREERTOS=1)
  # Link your FreeRTOS kernel target here if available (example):
  # target_link_libraries(omnia_port PUBLIC freertos_kernel)
elseif(OMNIA_RTOS STREQUAL "zephyr")
  target_sources(omnia_port PRIVATE
    core/omnia_port_ex.c
    ports/common/rtos_zephyr.c
  )
  target_compile_definitions(omnia_port PUBLIC OMNIA_CAP_RTOS=1 OMNIA_RTOS_ZEPHYR=1)
  # Zephyr apps are usually built with zephyr.cmake; adjust to your integration.
else()
  message(FATAL_ERROR "Unknown OMNIA_RTOS='${OMNIA_RTOS}'. Use: none|freertos|zephyr")
endif()

# ---- Drivers (HAL-agnostic) --------------------------------------------------
# Example driver: ST7735 LCD. It only depends on the Omnia Port contract.
add_library(st7735 STATIC
  drivers/st7735/st7735.c
)
target_include_directories(st7735
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/st7735
)
target_link_libraries(st7735 PUBLIC omnia_port)

# ---- (Optional) warnings & sanity --------------------------------------------
if (MSVC)
  target_compile_options(omnia_port PRIVATE /W4)
  target_compile_options(st7735    PRIVATE /W4)
else()
  target_compile_options(omnia_port PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(st7735    PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---- (Optional) install rules ------------------------------------------------
# install(TARGETS omnia_port st7735
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)
# install(DIRECTORY include/ DESTINATION include)

# ---- Usage notes -------------------------------------------------------------
# Configure with:
#   cmake -S . -B build -DOMNIA_PORT=stm32h755 -DOMNIA_RTOS=freertos
# Build:
#   cmake --build build --config Release
