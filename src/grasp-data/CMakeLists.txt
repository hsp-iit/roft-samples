#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(LIBRARY_TARGET_NAME roft-samples-grasp-data)

# Eigen
find_package(Eigen3)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           eigen
                           os
                           sig
)

# Thrift
set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/grasp_data.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Executable
add_library(${LIBRARY_TARGET_NAME} include/RankableCandidate.h ${${EXE_TARGET_NAME}_THRIFT_SRC})

target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC
                                                  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                  "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/src/grasp-data/>"
                                                  )

target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC YARP::YARP_os YARP::YARP_sig Eigen3::Eigen)
