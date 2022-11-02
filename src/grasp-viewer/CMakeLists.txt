#===============================================================================
#
# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT)
#
# This software may be modified and distributed under the terms of the
# GPL-2+ license. See the accompanying LICENSE file for details.
#
#===============================================================================

set(EXE_TARGET_NAME roft-samples-grasp-viewer)

# YARP
find_package(YARP REQUIRED COMPONENTS
                           dev
                           eigen
                           os
                           math
                           sig
)

# Eigen
find_package(Eigen3 REQUIRED)

# VTK
find_package(VTK REQUIRED)
if (VTK_VERSION VERSION_LESS "8.90.0")
  include(${VTK_USE_FILE})
endif()

# Executable
add_executable(${EXE_TARGET_NAME}
               include/module.h
               include/viewer.h
               include/iCubGaze.h
               src/main.cpp
               src/module.cpp
               src/iCubGaze.cpp
               src/vtkUpdateHandler.cpp
)

target_include_directories(${EXE_TARGET_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(${EXE_TARGET_NAME} PRIVATE
                                         Eigen3::Eigen
                                         YARP::YARP_dev
                                         YARP::YARP_eigen
                                         YARP::YARP_init
                                         YARP::YARP_math
                                         YARP::YARP_os
                                         YARP::YARP_sig
                                         ${VTK_LIBRARIES}
                                         roft-samples-grasp-data
)
if (VTK_VERSION VERSION_GREATER_EQUAL "8.90.0")
  vtk_module_autoinit(TARGETS roft-track-n-grasp MODULES ${VTK_LIBRARIES})
endif()

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

# Configuration files
yarp_install(FILES app/conf/config.ini DESTINATION ${ICUBCONTRIB_CONTEXTS_INSTALL_DIR}/${EXE_TARGET_NAME})
