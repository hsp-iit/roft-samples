cmake_minimum_required(VERSION 3.16)

project(rotf-samples
        LANGUAGES CXX)

include(GNUInstallDirs)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
            message(STATUS "Setting build type to 'Release' as none was specified.")
            set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
    endif()
endif()

find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()

option(BUILD_TRACKER        "Build the 6D tracker (roft-samples-tracker)." OFF)
option(BUILD_REALSENSE      "Build the Intel RealSense streamer (roft-samples-rs)." OFF)
option(BUILD_ICUB_HANDOVER  "Build the handover appliance with iCub (roft-samples-handover)." OFF)

if (BUILD_TRACKER)
   add_subdirectory(src/roft)
endif()

if (BUILD_REALSENSE)
   add_subdirectory(src/rs)
endif()

if (BUILD_ICUB_HANDOVER)
  add_subdirectory(src/grasp-data)
  add_subdirectory(src/grasp-viewer)
  add_subdirectory(src/handover)
endif()
