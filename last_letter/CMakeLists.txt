cmake_minimum_required(VERSION 2.8.3)
project(last_letter)

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

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
	# add_message_files(FILES
	# )

## Generate services in the 'srv' folder
	add_service_files(DIRECTORY srv FILES
	)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
	# generate_messages(
	# 	DEPENDENCIES
	# 	std_msgs
	# 	nav_msgs
	# 	geometry_msgs
	# 	sensor_msgs
	# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES modelPlaneLib
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find last_letter
	OUTPUT_VARIABLE LAST_LETTER_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${LAST_LETTER_DIR}/doc/doxygen ${LAST_LETTER_DIR})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
	add_library(modelPlaneLib src/modelPlaneLib.cpp)

## Declare a cpp executable
	add_executable(uav_publisher src/state_publisher.cpp)
	add_executable(quat2euler src/quat2euler.cpp)
	add_executable(model src/model.cpp)
	add_executable(joy2chan src/joy2chan.cpp)
	add_executable(environment src/environment.cpp)
	add_executable(virtualJoy src/virtualJoy.cpp)
	add_executable(autopilot_ll src/controller_ll.cpp)
	add_executable(autopilot_HCUAV src/controller_HCUAV.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
	add_dependencies(uav_publisher ${catkin_EXPORTED_TARGETS})
	add_dependencies(quat2euler ${PROJECT_NAME}_generate_messages_cpp)
	add_dependencies(model ${PROJECT_NAME}_generate_messages_cpp)
	add_dependencies(joy2chan ${PROJECT_NAME}_generate_messages_cpp)
	add_dependencies(environment ${PROJECT_NAME}_generate_messages_cpp)
	add_dependencies(virtualJoy ${PROJECT_NAME}_generate_messages_cpp)
	add_dependencies(autopilot_ll ${PROJECT_NAME}_generate_messages_cpp)
	add_dependencies(autopilot_HCUAV ${PROJECT_NAME}_generate_messages_cpp)

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

## Generate Doxygen files
	# add_custom_command(OUTPUT rosdoc_ll
	# 	# COMMAND rosdoc_lite -o ${LAST_LETTER_DIR}/doc/doxygen ${LAST_LETTER_DIR}
	# 	COMMAND rosdoc_lite ${LAST_LETTER_DIR}
	# 	# echo ******************"${LAST_LETTER_DIR}"
	# 	)
	# add_custom_target(ROSDOC_LL
	# 	DEPENDS rosdoc_ll)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS last_letter last_letter_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_last_letter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
