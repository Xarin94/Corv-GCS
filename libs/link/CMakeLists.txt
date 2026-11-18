add_library(corv_link STATIC
    Mavlink.h Mavlink.cpp
    MavMessage.h
    MavlinkParser.h MavlinkParser.cpp
    Transport.h
    SerialTransport.h SerialTransport.cpp
    NetTransports.h NetTransports.cpp
    TlogWriter.h TlogWriter.cpp
    LinkTrace.h LinkTrace.cpp
    MessageBus.h MessageBus.cpp
    LinkManager.h LinkManager.cpp
    MavFtp.h MavFtp.cpp
    LteLink.h LteLink.cpp
    MspSession.h MspSession.cpp
)
target_include_directories(corv_link PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(corv_link PUBLIC corv_core corv_mavlink Qt6::Core Qt6::Network Qt6::SerialPort)

# The cellular link's end-to-end crypto (AES-256-GCM, Ed25519) comes from
# OpenSSL's libcrypto; without it the build has no cellular link.
find_package(OpenSSL 1.1.1 COMPONENTS Crypto)
# A universal macOS build needs a libcrypto with every architecture (Homebrew's
# has only the runner's): without it the build has no cellular link
if(OpenSSL_FOUND AND APPLE AND CMAKE_OSX_ARCHITECTURES)
    execute_process(COMMAND lipo -archs ${OPENSSL_CRYPTO_LIBRARY} OUTPUT_VARIABLE _corv_ssl_archs
                    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    foreach(_a ${CMAKE_OSX_ARCHITECTURES})
        if(NOT " ${_corv_ssl_archs} " MATCHES " ${_a} ")
            set(OpenSSL_FOUND FALSE)
        endif()
    endforeach()
    if(NOT OpenSSL_FOUND)
        message(WARNING "OpenSSL (${_corv_ssl_archs}) lacks one of ${CMAKE_OSX_ARCHITECTURES}: built without the cellular (LTE) link")
    endif()
endif()
if(OpenSSL_FOUND)
    target_compile_definitions(corv_link PUBLIC CORV_HAVE_OPENSSL)
    target_link_libraries(corv_link PUBLIC OpenSSL::Crypto)
    message(STATUS "Cellular link: OpenSSL ${OPENSSL_VERSION}")
else()
    message(WARNING "OpenSSL not found: built without the cellular (LTE) link")
endif()
