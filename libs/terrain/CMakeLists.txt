add_library(corv_terrain STATIC
    Hgt.h Hgt.cpp
    HgtDownloader.h HgtDownloader.cpp
    TileStore.h TileStore.cpp
    Bc1.h Bc1.cpp
    ChunkImagery.h ChunkImagery.cpp
    SatelliteDetail.h SatelliteDetail.cpp
    TerrainManager.h TerrainManager.cpp
)
set_target_properties(corv_terrain PROPERTIES AUTOMOC ON)
target_include_directories(corv_terrain PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_terrain PUBLIC corv_scene corv_core Qt6::Core Qt6::Gui Qt6::Network Qt6::Sql Qt6::Concurrent)
# Development fallback for SRTM tiles: <source>/topo
target_compile_definitions(corv_terrain PRIVATE CORV_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
