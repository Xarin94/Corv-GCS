# Renderer-neutral scene data: no graphics API (guide, "Struttura del progetto")
add_library(corv_scene STATIC
    TerrainData.h
    ModelData.h ModelData.cpp
    GeoZone.h GeoZone.cpp
    GeoZoneRaster.h GeoZoneRaster.cpp
    CellOverlay.h CellOverlay.cpp
    PointCloudData.h PointCloudData.cpp
    SurfaceTiles.h SurfaceTiles.cpp
    SurfaceRaster.h SurfaceRaster.cpp
    SurfaceVolume.h SurfaceVolume.cpp
    RosSurfaceData.h RosSurfaceData.cpp
    FramePacer.h FramePacer.cpp
)
target_include_directories(corv_scene PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_scene PUBLIC corv_core Qt6::Core Qt6::Gui)
if(WIN32)
    target_link_libraries(corv_scene PRIVATE dwmapi)   # FramePacer: the vblank timing of DWM
endif()
