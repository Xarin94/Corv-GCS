# MAVLink C headers generated from tools/mavlink/corv.xml (MIT licensed output of mavgen).
add_library(corv_mavlink INTERFACE)
target_include_directories(corv_mavlink SYSTEM INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/mavlink)
