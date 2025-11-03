# Stub toolchain for ESP32. In practice, configure via ESP-IDF (idf.py).
set(CMAKE_SYSTEM_NAME Generic)
set(EDGE_TARGET_ESP32 ON)
message(STATUS "Using ESP32 toolchain stub. For real builds, use ESP-IDF (idf.py).")
