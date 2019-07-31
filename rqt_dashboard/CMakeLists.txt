cmake_minimum_required(VERSION 2.8.3)
project(rqt_dashboard)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	rqt_gui
	rqt_gui_py
	message_generation
	nav_msgs
	geometry_msgs
	std_msgs
	last_letter_msgs
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(FILES
	Dashboard.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	nav_msgs
	std_msgs
	geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(dash_backend src/backend.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(dash_backend  ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(dash_backend ${catkin_LIBRARIES})

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find rqt_dashboard
	OUTPUT_VARIABLE RQT_DASHBOARD_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${RQT_DASHBOARD_DIR}/doc/doxygen_rosdoc_lite ${RQT_DASHBOARD_DIR})

#############
## Install ##
#############

# Convert to proper python script installation command
#  install(PROGRAMS
#    scripts/rqt_dashboard
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
