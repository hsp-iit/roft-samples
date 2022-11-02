#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME roft-samples-handover)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           dev
                           eigen
                           os
                           sig
)

# iCub
find_package(ICUB REQUIRED COMPONENTS
                           iKin
)

# Eigen
find_package(Eigen3 REQUIRED)

# Thrift
set(${EXE_TARGET_NAME}_THRIFT_HDR thrift/idl.thrift)

yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_SRC ${${EXE_TARGET_NAME}_THRIFT_HDR})

# Executable
add_executable(${EXE_TARGET_NAME}
               include/cardinal_points_grasp.h
               include/module.h
               include/iCubCartesian.h
               include/iCubGaze.h
               include/iCubMotors.h
               include/iCubMotorsPositions.h
               include/Utils.h
               src/main.cpp
               src/module.cpp
               src/iCubCartesian.cpp
               src/iCubGaze.cpp
               src/iCubMotors.cpp
               src/iCubMotorsPositions.cpp
               src/Utils.cpp
               ${${EXE_TARGET_NAME}_THRIFT_SRC}
)

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         Eigen3::Eigen
                                         ICUB::iKin
                                         YARP::YARP_dev
                                         YARP::YARP_eigen
                                         YARP::YARP_init
                                         YARP::YARP_os
                                         YARP::YARP_sig
                                         roft-samples-grasp-data
)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Configuration files
yarp_install(FILES app/roft-samples-handover.xml DESTINATION ${ICUBCONTRIB_APPLICATIONS_TEMPLATES_INSTALL_DIR})
yarp_install(FILES app/conf/config.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
