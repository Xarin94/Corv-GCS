qt_add_library(corv_gpu STATIC)
qt_add_qml_module(corv_gpu
    URI Corv.Gpu
    VERSION 1.0
    SOURCES
        TerrainRenderer.h TerrainRenderer.cpp
        OverlayRenderer.h OverlayRenderer.cpp
        OutlineRenderer.h OutlineRenderer.cpp
        PointCloudRenderer.h PointCloudRenderer.cpp
        RosSurfaceRenderer.h RosSurfaceRenderer.cpp
        FrameTicker.h FrameTicker.cpp
        SceneItem.h SceneItem.cpp
)
target_include_directories(corv_gpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
# QRhi headers ship with the GuiPrivate include path (limited compatibility guarantees: stay on 6.12)
target_link_libraries(corv_gpu PUBLIC corv_terrain corv_scene corv_vehicle Qt6::Gui Qt6::GuiPrivate Qt6::Quick)

corv_add_shaders(corv_gpu "/shaders" ${CMAKE_SOURCE_DIR}/shaders
    ${CMAKE_SOURCE_DIR}/shaders/terrain.vert
    ${CMAKE_SOURCE_DIR}/shaders/terrain.frag
    ${CMAKE_SOURCE_DIR}/shaders/line.vert
    ${CMAKE_SOURCE_DIR}/shaders/line.frag
    ${CMAKE_SOURCE_DIR}/shaders/mesh.vert
    ${CMAKE_SOURCE_DIR}/shaders/mesh.frag
    ${CMAKE_SOURCE_DIR}/shaders/fill.vert
    ${CMAKE_SOURCE_DIR}/shaders/fill.frag
    ${CMAKE_SOURCE_DIR}/shaders/water.vert
    ${CMAKE_SOURCE_DIR}/shaders/water.frag
    ${CMAKE_SOURCE_DIR}/shaders/particles.vert
    ${CMAKE_SOURCE_DIR}/shaders/particles.frag
    ${CMAKE_SOURCE_DIR}/shaders/label.vert
    ${CMAKE_SOURCE_DIR}/shaders/label.frag
    ${CMAKE_SOURCE_DIR}/shaders/outline.vert
    ${CMAKE_SOURCE_DIR}/shaders/outline.frag
    ${CMAKE_SOURCE_DIR}/shaders/points.vert
    ${CMAKE_SOURCE_DIR}/shaders/livepoints.vert
    ${CMAKE_SOURCE_DIR}/shaders/points.frag
    ${CMAKE_SOURCE_DIR}/shaders/rosmesh.vert
    ${CMAKE_SOURCE_DIR}/shaders/rosmesh.frag
    ${CMAKE_SOURCE_DIR}/shaders/rosvolume.vert
    ${CMAKE_SOURCE_DIR}/shaders/rosvolume.frag)
