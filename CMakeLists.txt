cmake_minimum_required(VERSION 3.0.2)
project(nobleo_gps_calibration)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    backward_ros

    rospy
)
catkin_package(
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# Roslint cpp
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

