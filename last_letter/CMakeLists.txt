cmake_minimum_required(VERSION 2.8.3)
project(last_letter)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	nav_msgs
	geometry_msgs
	sensor_msgs
	last_letter_msgs
	message_generation
	uav_utils
	mathutils
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES modelPlaneLib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(modelPlaneLib src/modelPlaneLib.cpp)

## Add cmake target dependencies of the executable/library
add_executable(uav_publisher src/state_publisher.cpp)
add_executable(quat2euler src/quat2euler.cpp)
add_executable(model src/model.cpp)
add_executable(joy2chan src/joy2chan.cpp)
add_executable(environment src/environment.cpp)
add_executable(virtualJoy src/virtualJoy.cpp)
add_executable(autopilot_ll src/controller_ll.cpp)
add_executable(autopilot_HCUAV src/controller_HCUAV.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(uav_publisher last_letter_msgs_gencpp ${catkin_EXPORTED_TARGETS})
add_dependencies(quat2euler last_letter_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(model last_letter_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(joy2chan last_letter_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(environment last_letter_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(virtualJoy ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(autopilot_ll last_letter_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(autopilot_HCUAV last_letter_msgs_gencpp ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(uav_publisher ${catkin_LIBRARIES})
target_link_libraries(quat2euler mathutils ${catkin_LIBRARIES})
target_link_libraries(model mathutils uav_utils modelPlaneLib ${catkin_LIBRARIES})
target_link_libraries(joy2chan ${catkin_LIBRARIES})
target_link_libraries(environment mathutils uav_utils ${catkin_LIBRARIES})
target_link_libraries(virtualJoy mathutils uav_utils ${catkin_LIBRARIES})
target_link_libraries(autopilot_ll mathutils uav_utils ${catkin_LIBRARIES})
target_link_libraries(autopilot_HCUAV mathutils uav_utils ${catkin_LIBRARIES})
target_link_libraries(modelPlaneLib mathutils ${catkin_LIBRARIES})

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find last_letter
	OUTPUT_VARIABLE LAST_LETTER_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${LAST_LETTER_DIR}/doc/doxygen ${LAST_LETTER_DIR})
