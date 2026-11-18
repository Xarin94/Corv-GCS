add_library(corv_mission STATIC
    RouteModel.h RouteModel.cpp
    RouteGeo.h RouteGeo.cpp
    Platforms.h Platforms.cpp
    RouteCompiler.h RouteCompiler.cpp
    RadioLink.h RadioLink.cpp
    MissionTransfer.h MissionTransfer.cpp
    MissionStore.h MissionStore.cpp
)
target_include_directories(corv_mission PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
# QColor for the link styles only: Gui, no widgets
target_link_libraries(corv_mission PUBLIC corv_core Qt6::Core Qt6::Gui)
