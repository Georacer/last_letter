cmake_minimum_required(VERSION 2.8.3)
project(last_letter)

# activate c++ 11, taken from https://github.com/uzh-rpg/rpg_mpc/blob/master/CMakeLists.txt
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

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
	tf
	last_letter_lib
)

find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
# add_service_files(DIRECTORY srv FILES
# )

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
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${last_letter_lib_INCLUDE_DIRS}
)

## Add cmake target dependencies of the executable/library
# add_executable(uav_publisher src/state_publisher.cpp)
# add_executable(quat2euler src/quat2euler.cpp)
add_executable(model src/uav_model_ros.cpp src/model.cpp)
add_executable(joy2chan src/joy2chan.cpp)
add_executable(virtualJoy src/virtualJoy.cpp)
# add_executable(autopilot_ll src/controller_ll.cpp)
# add_executable(autopilot_HCUAV src/controller_HCUAV.cpp)
add_executable(simClock src/clock.cpp)
add_executable(ctrlInjector src/ctrlInjector.cpp)
# add_executable(cameraController src/cameraController.cpp)
add_executable(argumentHandler src/argumentHandler.cpp)

## Add cmake target dependencies of the executable/library
# add_dependencies(uav_publisher ${catkin_EXPORTED_TARGETS})
# add_dependencies(quat2euler ${catkin_EXPORTED_TARGETS} )
add_dependencies(model ${last_letter_lib_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
add_dependencies(joy2chan ${catkin_EXPORTED_TARGETS} )
# add_dependencies(virtualJoy )
# add_dependencies(autopilot_ll ${catkin_EXPORTED_TARGETS} )
# add_dependencies(autopilot_HCUAV ${catkin_EXPORTED_TARGETS} )
add_dependencies(simClock ${catkin_EXPORTED_TARGETS} )
add_dependencies(ctrlInjector ${catkin_EXPORTED_TARGETS} )
# add_dependencies(cameraController ${catkin_EXPORTED_TARGETS} )
add_dependencies(argumentHandler ${catkin_EXPORTED_TARGETS} )

## Specify libraries to link a library or executable target against
# target_link_libraries(uav_publisher ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
# target_link_libraries(quat2euler ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
target_link_libraries(model ${last_letter_lib_LIBRARIES} ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
target_link_libraries(joy2chan ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
target_link_libraries(virtualJoy ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
# target_link_libraries(autopilot_ll ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
# target_link_libraries(autopilot_HCUAV ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
# target_link_libraries(modelPlaneLib ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
target_link_libraries(simClock ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
target_link_libraries(ctrlInjector ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})
# target_link_libraries(cameraController ${catkin_LIBRARIES})
target_link_libraries(argumentHandler ${catkin_LIBRARIES} ${catkin_INCLUDE_DIRS})

## Generate package documentation with rosdoc_lite
execute_process(COMMAND rospack find last_letter
	OUTPUT_VARIABLE LAST_LETTER_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(COMMAND rosdoc_lite -o ${LAST_LETTER_DIR}/doc/doxygen_rosdoc_lite ${LAST_LETTER_DIR} )
