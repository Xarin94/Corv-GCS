# Services of the guide's "Altri sottosistemi": log replay and the spectra
# computed from logs, ADS-B traffic from OpenSky (geozones, LTE cells, SITL later).
add_library(corv_services STATIC
    DataFlash.h
    LogSpectrum.h LogSpectrum.cpp
    LogIndex.h LogIndex.cpp
    LogReplay.h LogReplay.cpp
    AdsbClient.h AdsbClient.cpp
    Rtcm.h Rtcm.cpp
    Ltm.h Ltm.cpp
    GeoZoneStore.h GeoZoneStore.cpp
    CellDb.h CellDb.cpp
    UpdateCheck.h UpdateCheck.cpp
)
target_include_directories(corv_services PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_services PUBLIC corv_scene corv_vehicle corv_link corv_core Qt6::Core Qt6::Gui Qt6::Network Qt6::Concurrent)
