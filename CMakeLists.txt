cmake_minimum_required(VERSION 3.22)
project(omnia C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---------------- Opcions globals ----------------
set(OMNIA_PORT "stm32h755" CACHE STRING "stm32h755")
option(OMNIA_ENABLE_HM19       "Enable HM19 BLE driver" OFF)
option(OMNIA_ENABLE_EMG_STREAM "Enable EMG stream layer" OFF)
option(OMNIA_ENABLE_EMG_PACKET "Enable EMG packet protocol" ON)
option(OMNIA_ENABLE_APPS       "Build Omnia apps" ON)

message(STATUS "OMNIA_PORT = ${OMNIA_PORT}")

# ---------------- Port core ----------------
add_library(omnia_port STATIC
  core/omnia_port_default.c
)
target_include_directories(omnia_port PUBLIC include)

# ---------------- HAL port ----------------
if(OMNIA_PORT STREQUAL "stm32h755")
  target_sources(omnia_port PRIVATE ports/stm32h7/src/port_stm32.c)
  target_include_directories(omnia_port PUBLIC ports/stm32h7/inc)
  target_compile_definitions(omnia_port PUBLIC OMNIA_PORT_STM32H755=1)
else()
  message(FATAL_ERROR "Unsupported OMNIA_PORT=${OMNIA_PORT}")
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
  )

  # hm19 només si està activat i existeix el target
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
    apps/emg_plot/include
  )

  target_link_libraries(app_emg_plot PUBLIC
    st7735
    sen0240
    omnia_port
    emg_packet
    m              # <- aquí és on realment cal (sqrtf, fabsf, etc.)
  )

  if(OMNIA_ENABLE_EMG_STREAM)
    target_link_libraries(app_emg_plot PUBLIC emg_stream)
  endif()
endif()

# =================================================
# Omnia all-in-one
# =================================================
add_library(omnia_all INTERFACE)

target_link_libraries(omnia_all INTERFACE
  omnia_port
  st7735
  sen0240
)

# Opcional: paraigua (no fa mal, però no confiïs només en això)
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
