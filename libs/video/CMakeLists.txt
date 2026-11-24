# FPV video: the camera's RTSP stream, decoded with FFmpeg (guide, "Video FPV")
add_library(corv_video STATIC
    Rtp.h Rtp.cpp
    RtspClient.h RtspClient.cpp
    VideoDecoder.h VideoDecoder.cpp
    FpvStream.h FpvStream.cpp
)
target_include_directories(corv_video PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_video PUBLIC corv_scene corv_core Qt6::Core Qt6::Network)

# libavcodec and libavutil: optional, as SDL3 and OpenSSL. FFMPEG_ROOT (include/,
# lib/, bin/ on Windows) is an LGPL build to ship with the program; without it a
# local build takes the system's through pkg-config (CORV_SYSTEM_FFMPEG=OFF: no,
# which is what a package made without FFMPEG_ROOT needs: it would not start where
# that library is not installed). Without either the program builds and the FPV
# button says there is no decoder.
option(CORV_SYSTEM_FFMPEG "Use the system's FFmpeg when FFMPEG_ROOT is not set" ON)
set(FFMPEG_ROOT "$ENV{FFMPEG_ROOT}" CACHE PATH "FFmpeg (include/, lib/) for the FPV video")
set(CORV_FFMPEG "")
if(FFMPEG_ROOT)
    find_path(CORV_AVCODEC_INCLUDE libavcodec/avcodec.h PATHS ${FFMPEG_ROOT}/include NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    find_library(CORV_AVCODEC_LIB NAMES avcodec PATHS ${FFMPEG_ROOT}/lib ${FFMPEG_ROOT}/bin NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    find_library(CORV_AVUTIL_LIB NAMES avutil PATHS ${FFMPEG_ROOT}/lib ${FFMPEG_ROOT}/bin NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
    if(CORV_AVCODEC_INCLUDE AND CORV_AVCODEC_LIB AND CORV_AVUTIL_LIB)
        target_include_directories(corv_video PRIVATE ${CORV_AVCODEC_INCLUDE})
        target_link_libraries(corv_video PRIVATE ${CORV_AVCODEC_LIB} ${CORV_AVUTIL_LIB})
        set(CORV_FFMPEG "${FFMPEG_ROOT}")
    endif()
elseif(APPLE AND CMAKE_OSX_ARCHITECTURES MATCHES ";")
    # A universal build needs libraries with every architecture; Homebrew's have one
    message(STATUS "Universal build: FFmpeg only from FFMPEG_ROOT")
elseif(CORV_SYSTEM_FFMPEG AND NOT WIN32)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(AVCODEC QUIET IMPORTED_TARGET libavcodec>=58 libavutil>=56)
        if(AVCODEC_FOUND)
            target_link_libraries(corv_video PRIVATE PkgConfig::AVCODEC)
            set(CORV_FFMPEG "libavcodec ${AVCODEC_libavcodec_VERSION}")
        endif()
    endif()
endif()
if(CORV_FFMPEG)
    target_compile_definitions(corv_video PRIVATE CORV_HAVE_FFMPEG)
    message(STATUS "FPV video: ${CORV_FFMPEG}")
else()
    message(WARNING "FFmpeg not found: built without the FPV video decoder")
endif()
