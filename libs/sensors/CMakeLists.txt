# Sensors next to the MAVLink link (guide, "Sensori"): the Livox LiDAR over
# its own UDP protocol and ROS through rosbridge, each on a thread of its own.
add_library(corv_sensors STATIC
    LivoxProtocol.h LivoxProtocol.cpp
    LidarCore.h LidarCore.cpp
    RosPoints.h RosPoints.cpp
    RosClient.h RosClient.cpp
)
target_include_directories(corv_sensors PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_sensors PUBLIC corv_scene corv_vehicle corv_link corv_core Qt6::Core Qt6::Network Qt6::WebSockets)
