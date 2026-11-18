# Operator input devices: game controllers and joysticks (SDL3, optional like
# OpenSSL for the cellular link) and the RC mapping of JoystickManager.js.
add_library(corv_input STATIC
    JoystickMath.h JoystickMath.cpp
    Gamepad.h Gamepad.cpp
)
target_include_directories(corv_input PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_input PUBLIC corv_core Qt6::Core)
if(TARGET SDL3::SDL3)
    target_compile_definitions(corv_input PUBLIC CORV_HAVE_SDL3)
    target_link_libraries(corv_input PRIVATE SDL3::SDL3)
endif()
