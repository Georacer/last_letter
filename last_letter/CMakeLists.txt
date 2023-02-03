cmake_minimum_required(VERSION 3.5)
project(last_letter)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(last_letter_msgs REQUIRED)
find_package(mavlink REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(iir REQUIRED)

# Declare and compile the last_letter_lib project
include(ExternalProject)
ExternalProject_Add(ext-last_letter_lib
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/external/last_letter_lib

  # CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=lib/${PROJECT_NAME}
  CMAKE_ARGS ""
  INSTALL_COMMAND "" # Do not install in system, library needed only locally
  USES_TERMINAL_BUILD true
)

# Construct include directories path
ExternalProject_Get_Property(ext-last_letter_lib SOURCE_DIR) # Access the source directory
list(APPEND LAST_LETTER_LIB_INCLUDE_DIR ${SOURCE_DIR}/include)
list(APPEND LAST_LETTER_LIB_INCLUDE_DIR ${SOURCE_DIR}/external/yaml-cpp/include)
message(WARNING "LAST_LETTER_LIB_INCLUDE_DIR set to ${LAST_LETTER_LIB_INCLUDE_DIR}")

# Expose built libraries
ExternalProject_Get_Property(ext-last_letter_lib BINARY_DIR) # Access the binaries location
message(WARNING "LAST_LETTER_LIB/BINARY_DIR set to ${BINARY_DIR}")
add_library(last_letter_lib SHARED IMPORTED) # Declare the dynamic library
set_target_properties(last_letter_lib PROPERTIES
  IMPORTED_LOCATION ${BINARY_DIR}/liblast_letter_lib.so # Attach the .so file to the library
  INTERFACE_INCLUDE_DIRECTORIES "${LAST_LETTER_LIB_INCLUDE_DIR}"
)

add_dependencies(last_letter_lib ext-last_letter_lib) # Declare a dependency on the 3rd party project, just in case
set(LAST_LETTER_LIB_LIBRARIES
  last_letter_lib
) # Aggregate all libraries to variable
message(WARNING "last_letter_lib libraries (LAST_LETTER_LIB_LIBRARIES) found= ${LAST_LETTER_LIB_LIBRARIES}")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}

  # ${iir_INCLUDE_DIRS}
  ${LAST_LETTER_LIB_INCLUDE_DIR}
)

add_library(gazebo_model_plugin SHARED src/gazebo_model_plugin.cpp)
target_link_libraries(gazebo_model_plugin
  ${LAST_LETTER_LIB_LIBRARIES}
)
ament_target_dependencies(gazebo_model_plugin
  last_letter_msgs
  geometry_msgs
  gazebo_dev
  gazebo_ros
  rclcpp
)
ament_export_libraries(gazebo_model_plugin)
install(TARGETS
  gazebo_model_plugin
  DESTINATION $ENV{HOME}/.gazebo/plugins
)

add_executable(px4_sitl_interface src/px4_sitl_interface.cpp src/mavlink_interface.cpp)
target_link_libraries(px4_sitl_interface
  ${LAST_LETTER_LIB_LIBRARIES}
)
ament_target_dependencies(px4_sitl_interface
  rclcpp
  std_msgs
  sensor_msgs
  last_letter_msgs
  gazebo_msgs
  Eigen3
  message_filters
)
install(TARGETS
  px4_sitl_interface
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(sensor_node src/sensor_node.cpp)
ament_target_dependencies(sensor_node
  rclcpp
  std_msgs
  gazebo_msgs
)
install(TARGETS
  sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(uav_model_ros src/uav_model_ros.cpp)
target_link_libraries(uav_model_ros
  iir::iir
  ${LAST_LETTER_LIB_LIBRARIES}
)
ament_target_dependencies(uav_model_ros
  rclcpp
  std_msgs
  std_srvs
  gazebo_msgs
  last_letter_msgs
)
install(TARGETS
  uav_model_ros
  DESTINATION lib/${PROJECT_NAME}
)

# Joy2chan node
add_executable(joy2chan src/joy2chan.cpp)
target_link_libraries(joy2chan
  ${LAST_LETTER_LIB_LIBRARIES}
)
ament_target_dependencies(joy2chan
  rclcpp
  std_msgs
  sensor_msgs
  last_letter_msgs
)
install(TARGETS
  joy2chan
  DESTINATION lib/${PROJECT_NAME}
)

# # Export the libraries
# ament_export_targets(
# export_last_letter_lib HAS_LIBRARY_TARGET
# )

# install (
# FILES ${LAST_LETTER_LIB_LIBRARIES}
# DESTINATION lib/${PROJECT_NAME}
# ARCHIVE DESTINATION lib
# RUNTIME DESTINATION bin
# INCLUDES DESTINATION include
# EXPORT last_letter_lib_targets
# )
foreach(shared_lib ${LAST_LETTER_LIB_LIBRARIES})
  install(
    FILES ${BINARY_DIR}/lib${shared_lib}.so
    DESTINATION lib
  )
endforeach()

# install (
# TARGETS ${LAST_LETTER_LIB_LIBRARIES}
# LIBRARY DESTINATION lib/${PROJECT_NAME}
# # ARCHIVE DESTINATION lib
# # RUNTIME DESTINATION bin
# # INCLUDES DESTINATION include
# # EXPORT last_letter_lib_targets
# )
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
