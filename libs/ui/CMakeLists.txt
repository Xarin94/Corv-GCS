set_source_files_properties(qml/Theme.qml qml/Icons.qml PROPERTIES QT_QML_SINGLETON_TYPE TRUE)

qt_add_library(corv_ui STATIC)
qt_add_qml_module(corv_ui
    URI Corv.Ui
    VERSION 1.0
    SOURCES
        HudItem.h HudItem.cpp
        HudInstruments.h HudInstruments.cpp
        VehicleModel.h VehicleModel.cpp
        GcsController.h GcsController.cpp
        MapItem.h MapItem.cpp
        PlanController.h PlanController.cpp
        PlanMapItem.h PlanMapItem.cpp
        ProfileItem.h ProfileItem.cpp
        SysConfig.h SysConfig.cpp
        DialogHub.h DialogHub.cpp
        ParamsController.h ParamsController.cpp
        LogsController.h LogsController.cpp
        CalibrationController.h CalibrationController.cpp
        MagCalView.h MagCalView.cpp
        EkfController.h EkfController.cpp
        FftController.h FftController.cpp
        AnnunciatorController.h AnnunciatorController.cpp
        ReplayController.h ReplayController.cpp
        RotorLoadItem.h RotorLoadItem.cpp
        FlapIndicatorItem.h FlapIndicatorItem.cpp
        HudPaint.h
        RtkController.h RtkController.cpp
        TelForwardController.h TelForwardController.cpp
        LteController.h LteController.cpp
        JoystickController.h JoystickController.cpp
        GeoZoneController.h GeoZoneController.cpp
        GeoZonePainter.h GeoZonePainter.cpp
        SetupController.h SetupController.cpp
        LidarController.h LidarController.cpp
        RosController.h RosController.cpp
        MountPreviewItem.h MountPreviewItem.cpp
        SitlController.h SitlController.cpp
        UpdateController.h UpdateController.cpp
    QML_FILES
        qml/Main.qml
        qml/Theme.qml
        qml/TitleBar.qml
        qml/HeaderBar.qml
        qml/CommandBar.qml
        qml/GcsSidebar.qml
        qml/Icons.qml
        qml/SvgIcon.qml
        qml/OutlinedText.qml
        qml/SquareButton.qml
        qml/DataCell.qml
        qml/EdgeTab.qml
        qml/CmdButton.qml
        qml/FlightDataView.qml
        qml/SetupPage.qml
        qml/SetupPanel.qml
        qml/CfgLabel.qml
        qml/ConnectionPanel.qml
        qml/CfgSelect.qml
        qml/DarkCombo.qml
        qml/CfgInput.qml
        qml/CfgCheck.qml
        qml/CfgSlider.qml
        qml/GcsButton.qml
        qml/SysPanel.qml
        qml/SysRow.qml
        qml/SysConfigPage.qml
        qml/FpIconButton.qml
        qml/FpField.qml
        qml/FpPopover.qml
        qml/FlightPlanPage.qml
        qml/MissionLibrary.qml
        qml/Dialogs.qml
        qml/UpdateDialog.qml
        qml/ParametersPage.qml
        qml/BitmaskEditor.qml
        qml/LogDownloadPage.qml
        qml/CfgNote.qml
        qml/CalButton.qml
        qml/CalInstruction.qml
        qml/CalResult.qml
        qml/CalTable.qml
        qml/AccelGlyph.qml
        qml/AccelCalPage.qml
        qml/CompassCalPage.qml
        qml/SensorCalPage.qml
        qml/EkfPanel.qml
        qml/FftPanel.qml
        qml/LeftBar.qml
        qml/AnnunciatorColumn.qml
        qml/ZoneAnnunciators.qml
        qml/GeoZoneLegend.qml
        qml/ReplayTimeline.qml
        qml/ModeBanner.qml
        qml/CfgRow.qml
        qml/CfgValue.qml
        qml/RtkPage.qml
        qml/TelForwardPage.qml
        qml/CellularLinkPage.qml
        qml/JoystickPage.qml
        qml/ParamField.qml
        qml/ParamSelect.qml
        qml/FlightModesPage.qml
        qml/FailsafePage.qml
        qml/PidTuningPage.qml
        qml/ExtTuningPage.qml
        qml/RadioCalPage.qml
        qml/VibrationPage.qml
        qml/ServoRelayPage.qml
        qml/LidarPage.qml
        qml/RosPage.qml
        qml/SensorStrip.qml
        qml/ArmDialog.qml
        qml/SimulationPage.qml
)
target_include_directories(corv_ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_ui PUBLIC corv_mission corv_vehicle corv_services corv_gpu corv_input corv_sensors Qt6::Quick Qt6::Qml Qt6::QuickControls2 Qt6::Network Qt6::Sql Qt6::Concurrent)
# Development fallback for the 3D models: <source>/resources/models
target_compile_definitions(corv_ui PRIVATE CORV_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

# HUD instruments: white drawings tinted on the GPU (HudInstruments.cpp)
corv_add_shaders(corv_ui "/hudshaders" ${CMAKE_SOURCE_DIR}/shaders
    ${CMAKE_SOURCE_DIR}/shaders/hudtint.vert
    ${CMAKE_SOURCE_DIR}/shaders/hudtint.frag)

# REMEMBER KEY of the cellular link: DPAPI
if(WIN32)
    target_link_libraries(corv_ui PRIVATE Crypt32)
elseif(UNIX AND NOT APPLE)
    target_link_libraries(corv_ui PRIVATE ${CMAKE_DL_LIBS})
endif()
