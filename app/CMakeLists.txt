qt_add_executable(corv-gcs WIN32 MACOSX_BUNDLE main.cpp TestDriver.h TestDriver.cpp DebugServer.h DebugServer.cpp)

# macOS: an application bundle, what macdeployqt and the DMG need (the CI's
# "Package (macOS)" step); its data root is the per-user folder (DataRoot)
if(APPLE)
    set(CORV_ICNS ${CMAKE_SOURCE_DIR}/packaging/macos/corv-gcs.icns)
    target_sources(corv-gcs PRIVATE ${CORV_ICNS})
    set_source_files_properties(${CORV_ICNS} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    set_target_properties(corv-gcs PROPERTIES
        MACOSX_BUNDLE_GUI_IDENTIFIER ${CORV_APP_ID}
        MACOSX_BUNDLE_BUNDLE_NAME "${CORV_APP_NAME}"
        MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
        MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
        MACOSX_BUNDLE_ICON_FILE corv-gcs.icns)
endif()

file(GLOB CORV_FONTS ${CMAKE_SOURCE_DIR}/resources/fonts/*)
file(GLOB CORV_ICONS ${CMAKE_SOURCE_DIR}/resources/icons/*.png)
# SITL default parameters and frame models (sitl-defaults of the Electron version)
file(GLOB CORV_SITL ${CMAKE_SOURCE_DIR}/resources/sitl-defaults/*)
# Vehicle models of the 3D view (models/ next to the installation still wins)
file(GLOB CORV_MODELS ${CMAKE_SOURCE_DIR}/resources/models/*.glb)
qt_add_resources(corv-gcs "corv_assets"
    PREFIX "/"
    BASE ${CMAKE_SOURCE_DIR}/resources
    FILES ${CORV_FONTS} ${CORV_ICONS} ${CORV_SITL} ${CORV_MODELS})

target_link_libraries(corv-gcs PRIVATE corv_ui corv_uiplugin corv_gpu corv_gpuplugin Qt6::Quick Qt6::QuickControls2 Qt6::Svg Qt6::Network)

if(WIN32)
    target_sources(corv-gcs PRIVATE ${CMAKE_SOURCE_DIR}/packaging/windows/corv-gcs.rc)
endif()

# SDL3.dll next to the executable (joystick input), when the build has it
if(TARGET SDL3::SDL3)
    add_custom_command(TARGET corv-gcs POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_FILE:SDL3::SDL3> $<TARGET_FILE_DIR:corv-gcs>)
endif()

# The LTE cells shipped with the application (cell database, CellDb): celldb/ next to
# the executable, 10 MB that are mapped from disk when needed rather than compiled in
add_custom_command(TARGET corv-gcs POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/resources/celldb $<TARGET_FILE_DIR:corv-gcs>/celldb
    COMMENT "Copying the bundled LTE cells")
