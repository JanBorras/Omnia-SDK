# Stub toolchain for STM32H7. In practice, set arm-none-eabi-* compilers and flags.
set(CMAKE_SYSTEM_NAME Generic)
set(EDGE_TARGET_STM32H7 ON)
message(STATUS "Using STM32H7 toolchain stub. Set arm-none-eabi-gcc and HAL paths.")
