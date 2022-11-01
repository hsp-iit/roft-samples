#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME roft-samples-tracker)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           os
)

# Eigen
find_package(Eigen3 REQUIRED)

# Robots I/O facilities
find_package(RobotsIO REQUIRED)

# ROFT library
find_package(ROFTLib REQUIRED)

# Check if NVOF is available
find_package(OpenCV REQUIRED)
if (NOT OpenCV_opencv_cudaoptflow_FOUND)
   message(FATAL_ERROR "This software requires OpenCV compiled with BUILD_opencv_cudaoptflow in order to be compiled.")
endif()

# Thrift
set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/idl.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Executable
add_executable(${EXE_TARGET_NAME}
               include/tracker.h
               include/ROFTFilter.h
               src/main.cpp
               src/tracker.cpp
               src/ROFTFilter.cpp
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
)

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         Eigen3::Eigen
                                         ROFTLib::ROFT
                                         RobotsIO::RobotsIO
                                         YARP::YARP_os
                                         YARP::YARP_init
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Configuration files
yarp_install(FILES app/roft.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES app/conf/config.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
yarp_install(DIRECTORY meshes/DOPE_textured/ DESTINATION ${ICUBCONTRIB_DATA_INSTALL_DIR}/${EXE_TARGET_NAME}/meshes/DOPE_textured)
