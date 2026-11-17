# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# 1, 3, 4 are other WiCAN variants not supported by this branch
set(WICAN_V300 2)
set(WICAN_PROTO 5)
set(WICAN_EB_FD 6)

# Variant selection: override with idf.py -DHARDWARE_VER_NAME=proto.
# The variant implies the chip target (V300 = ESP32-C3, proto/eb-fd = ESP32-S3),
# so the right sdkconfig.defaults.<target> file is picked up automatically.
if(NOT DEFINED HARDWARE_VER_NAME)
    set(HARDWARE_VER_NAME "proto")
endif()

if(HARDWARE_VER_NAME STREQUAL "proto")
    set(HARDWARE_VER ${WICAN_PROTO})
    set(HW_VER "PROTO")
    set(HW_PREF "proto")
    set(HW_TARGET "esp32s3")
elseif(HARDWARE_VER_NAME STREQUAL "eb-fd")
    set(HARDWARE_VER ${WICAN_EB_FD})
    set(HW_VER "EB-FD")
    set(HW_PREF "eb-fd")
    set(HW_TARGET "esp32s3")
elseif(HARDWARE_VER_NAME STREQUAL "v300")
    set(HARDWARE_VER ${WICAN_V300})
    set(HW_VER "OBD")
    set(HW_PREF "obd")
    set(HW_TARGET "esp32c3")
else()
    message(FATAL_ERROR "Unknown HARDWARE_VER_NAME '${HARDWARE_VER_NAME}' (expected 'v300', 'proto', or 'eb-fd')")
endif()

# Default the chip target from the variant (overridable with -DIDF_TARGET or
# IDF_TARGET env var). A little redundant, since we still error below
# if the IDF_TARGET is mismatched with the current HARDWARE_VER_NAME,
# but we don't want to silently ignore a user's IDF_TARGET.
set(IDF_TARGET "${HW_TARGET}" CACHE STRING "IDF target chip")
if(DEFINED ENV{IDF_TARGET} AND NOT "$ENV{IDF_TARGET}" STREQUAL "")
    set(EFFECTIVE_IDF_TARGET "$ENV{IDF_TARGET}")
else()
    set(EFFECTIVE_IDF_TARGET "${IDF_TARGET}")
endif()
if(NOT EFFECTIVE_IDF_TARGET STREQUAL HW_TARGET)
    message(FATAL_ERROR "Variant '${HARDWARE_VER_NAME}' requires IDF_TARGET=${HW_TARGET}, got '${EFFECTIVE_IDF_TARGET}'")
endif()

# Keep the generated sdkconfig inside the build dir so the hardware
# variants (separate build dirs, different IDF targets) don't fight over
# a single top-level sdkconfig file.
if(NOT DEFINED SDKCONFIG)
    set(SDKCONFIG "${CMAKE_BINARY_DIR}/sdkconfig")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

add_definitions(-DWICAN_V300=${WICAN_V300})
add_definitions(-DWICAN_PROTO=${WICAN_PROTO})
add_definitions(-DWICAN_EB_FD=${WICAN_EB_FD})
add_definitions(-DHARDWARE_VER=${HARDWARE_VER})
add_definitions(-DHARDWARE_VERSION="${HW_VER}")

execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(GIT_SHA1 "${GIT_SHA}")
string(REPLACE "." "" GIT_SHA "${GIT_SHA}")

add_definitions(-DGIT_SHA="${GIT_SHA1}")

set(PROJECT_PREF "wican-fw")

# Form the project binary name
set(PRJ_BIN "${PROJECT_PREF}_${HW_PREF}_${GIT_SHA}")
project("${PRJ_BIN}")
