add_library(corv_core STATIC
    Geo.h Geo.cpp
    RingBuffer.h
    LruCache.h
    DataRoot.h DataRoot.cpp
    Settings.h Settings.cpp
    ElectronSettings.h ElectronSettings.cpp
    DebugLog.h DebugLog.cpp
    Inflate.h Inflate.cpp
    Clock.h
)
target_include_directories(corv_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_core PUBLIC Qt6::Core)
target_compile_definitions(corv_core PUBLIC
    CORV_VERSION="${CORV_VERSION_FULL}"
    CORV_UPDATE_REPO="${CORV_UPDATE_REPO}"
    CORV_APP_NAME="${CORV_APP_NAME}")
