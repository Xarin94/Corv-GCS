qt_add_executable(corv-link-cli main.cpp)
target_link_libraries(corv-link-cli PRIVATE corv_vehicle)
set_target_properties(corv-link-cli PROPERTIES WIN32_EXECUTABLE OFF)
