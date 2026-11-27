add_library(corv_vehicle STATIC
    VehicleState.h
    FlightModes.h FlightModes.cpp
    SceneOrigin.h SceneOrigin.cpp
    GpsReceivers.h GpsReceivers.cpp
    RelativeNav.h RelativeNav.cpp
    StateMapper.h StateMapper.cpp
    PoseSmoother.h PoseSmoother.cpp
    DemoFlight.h DemoFlight.cpp
    TrajectoryPredictor.h TrajectoryPredictor.cpp
    VehicleHub.h VehicleHub.cpp
    CommandSender.h CommandSender.cpp
    ConnectionManager.h ConnectionManager.cpp
    ParamCatalog.h ParamCatalog.cpp ParamCatalogData.cpp
    ParamMeta.h ParamMeta.cpp
    LogDownloader.h LogDownloader.cpp
    TerrainFeeder.h TerrainFeeder.cpp
    ParamStore.h ParamStore.cpp
    ServoMap.h ServoMap.cpp
    GeodesicGrid.h GeodesicGrid.cpp
    MagSamples.h MagSamples.cpp
    EkfMonitor.h EkfMonitor.cpp
    SpectrumMonitor.h SpectrumMonitor.cpp
)
target_include_directories(corv_vehicle PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_vehicle PUBLIC corv_link corv_core Qt6::Core Qt6::Gui)

# Parameter metadata (descriptions, ranges, values, bitmasks) of each firmware, from
# ArduPilot's apm.pdef.json: resources/params, made by tools/gen-param-meta.py
file(GLOB CORV_PARAM_META ${CMAKE_SOURCE_DIR}/resources/params/*.pdef.json.gz)
qt_add_resources(corv_vehicle "corv_param_meta"
    PREFIX "/params"
    BASE ${CMAKE_SOURCE_DIR}/resources/params
    FILES ${CORV_PARAM_META})
