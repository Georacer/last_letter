cmake_minimum_required(VERSION 2.8.3)
project(uav_utils)

set (CMAKE_CXX_FLAGS "-Wall -DBT_USE_DOUBLE_PRECISION")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	tf
	geometry_msgs
	roscpp
	rospy
	sensor_msgs
	std_msgs
	nav_msgs
	last_letter_msgs
	)
find_package(LAPACK REQUIRED)

find_python_module(geomag REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES uav_utils
)

###########
## Build ##
###########
#
## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(uav_utils src/uav_utils.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(uav_utils last_letter_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(uav_utils ${LAPACK_LIBRARIES} ${catkin_LIBRARIES})

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find uav_utils
	OUTPUT_VARIABLE UAV_UTILS_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${UAV_UTILS_DIR}/doc/doxygen_rosdoc_lite ${UAV_UTILS_DIR})

#############
## Install ##
#############

install(PROGRAMS
   bin/uav_utils
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
