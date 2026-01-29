# tcxPave - Path manipulation toolkit for TrussC
# Ported from baku89's pave (https://github.com/baku89/pave)

cmake_minimum_required(VERSION 3.16)

# Addon name
set(ADDON_NAME tcxPave)

# Header files
set(ADDON_HEADERS
    src/Pave.h
    src/PaveTypes.h
    src/PaveLocation.h
    src/PaveData.h
    src/PaveLine.h
    src/PaveCubicBezier.h
    src/PaveArc.h
    src/PaveSegment.h
    src/PaveCurve.h
    src/PavePath.h
    src/PaveDistort.h
)

# Create INTERFACE library (header-only)
add_library(${ADDON_NAME} INTERFACE)

# Set include directories
target_include_directories(${ADDON_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

message(STATUS "[${ADDON_NAME}] Header-only addon configured")
