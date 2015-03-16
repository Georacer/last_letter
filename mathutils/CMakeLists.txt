cmake_minimum_required(VERSION 2.8.3)
project(mathutils)

set (CMAKE_CXX_FLAGS "-Wall -DBT_USE_DOUBLE_PRECISION")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS tf geometry_msgs roscpp rospy sensor_msgs std_msgs)
find_package (LAPACK REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${LAPACK_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mathutils src/mathutils.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(mathutils ${LAPACK_LIBRARIES} ${catkin_LIBRARIES})

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find mathutils
	OUTPUT_VARIABLE MATHUTILS_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${MATHUTILS_DIR}/doc/doxygen ${MATHUTILS_DIR})

#############
## Install ##
#############

install(PROGRAMS
bin/mathutils
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
