add_library(edge_hal
  hal/hal_common.c
)

add_library(edge_ml
  ml/tflm_api.c
)

target_include_directories(edge_hal PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_include_directories(edge_ml PUBLIC ${CMAKE_SOURCE_DIR}/include)

# Platform overrides (link ordering ensures platform-specific symbols override weak ones)
if(EDGE_TARGET_ESP32)
  add_library(edge_hal_esp32 hal/esp32/hal_esp32.c)
  target_link_libraries(edge_hal PUBLIC edge_hal_esp32)
endif()

if(EDGE_TARGET_STM32H7)
  add_library(edge_hal_stm32 hal/stm32h7/hal_stm32h7.c)
  target_link_libraries(edge_hal PUBLIC edge_hal_stm32)
endif()
