cmake_minimum_required(VERSION 3.25)

# The one place the version lives: window title, debug log header and installer read it from here.
project(CorvGcs VERSION 2.0.3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Qt 6.12 LTS, LGPLv3 modules only (see docs/IMPLEMENTATION.md, "Decisioni di base").
find_package(Qt6 6.12 REQUIRED COMPONENTS
    Core Gui GuiPrivate Qml Quick QuickControls2 Network Sql Concurrent SerialPort WebSockets Svg Test ShaderTools)
qt_standard_project_setup(REQUIRES 6.12)

# SDL3 for game controllers and joysticks (SETUP → JOYSTICK / RC): optional, as OpenSSL
# for the cellular link. tools/build.ps1 passes SDL3_DIR from build/tools/sdl3.
find_package(SDL3 CONFIG QUIET)
if(TARGET SDL3::SDL3)
    message(STATUS "SDL3 ${SDL3_VERSION}: joystick input")
else()
    message(WARNING "SDL3 not found: built without joystick input")
endif()

set(CORV_APP_ID "com.corvsystems.gcs")
set(CORV_APP_NAME "CORV GCS")

# The full version the program reports and the update check compares: the
# release tag without its v on CI (-DCORV_VERSION_LABEL=2.0.0-alpha.8), else
# <project version>-dev, which ranks above every alpha of that version and so
# never offers a local build an update
set(CORV_VERSION_LABEL "" CACHE STRING "Full version, e.g. 2.0.0-alpha.8 (empty: <project version>-dev)")
if(CORV_VERSION_LABEL)
    # The tag as given (v2.0.0-alpha.8) or without its v
    string(REGEX REPLACE "^v" "" CORV_VERSION_FULL "${CORV_VERSION_LABEL}")
else()
    set(CORV_VERSION_FULL "${PROJECT_VERSION}-dev")
endif()
# The public repository whose GitHub releases the update check reads
set(CORV_UPDATE_REPO "Xarin94/Corv-GCS" CACHE STRING "owner/repo of the releases the update check reads")
message(STATUS "CORV GCS ${CORV_VERSION_FULL}, updates from ${CORV_UPDATE_REPO}")

if(MSVC)
    add_compile_options(/W3 /permissive- /Zc:__cplusplus /utf-8)
    add_compile_definitions(_USE_MATH_DEFINES NOMINMAX WIN32_LEAN_AND_MEAN)
endif()

include(cmake/CorvShaders.cmake)

enable_testing()

# Libraries in dependency order: only gpu and ui know about graphics (guide, "Struttura del progetto").
foreach(dir IN ITEMS
        third_party
        libs/core libs/link libs/vehicle libs/input libs/mission libs/terrain libs/scene libs/gpu
        libs/sensors libs/video libs/services libs/ui
        app tools/linkcli tests)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${dir}/CMakeLists.txt)
        add_subdirectory(${dir})
    endif()
endforeach()
