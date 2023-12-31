# Copyright 2022 Nobleo Technology B.V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.0.2)
project(nobleo_odometry_calibration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Werror -Wall -Wextra -pedantic)

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

find_package(catkin REQUIRED
  COMPONENTS
    backward_ros
    dynamic_reconfigure
    rosbag
    roscpp
    roslint
    tf2_ros
)

find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/Calibrator.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    roscpp
)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDES}
  ${Boost_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(${PROJECT_NAME}
  src/calibrator.cpp
  src/config.cpp
  src/math.cpp
  src/solver.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

add_executable(offline
  src/bag_buffer.cpp
  src/bag_player.cpp
  src/offline.cpp
)
target_link_libraries(offline
  ${PROJECT_NAME}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} offline
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

# Roslint cpp
set(ROSLINT_CPP_OPTS "--filter=-whitespace/braces")
roslint_cpp()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(unittests
    test/main.cpp
    test/test_cost_function.cpp
    test/test_math.cpp
    test/test_solver.cpp
  )
  target_link_libraries(unittests ${PROJECT_NAME})
endif()

