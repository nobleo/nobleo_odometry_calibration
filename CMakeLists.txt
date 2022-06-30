cmake_minimum_required(VERSION 3.0.2)
project(nobleo_gps_calibration)

find_package(catkin REQUIRED
  COMPONENTS
    backward_ros
    roscpp
    roslint
)
catkin_package(
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(main
  src/main.cpp
)

install(TARGETS main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Roslint cpp
set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

