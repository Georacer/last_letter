cmake_minimum_required(VERSION 3.5)
project(last_letter_msgs)

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
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate message and service descriptions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vector4.msg"
  "msg/CommandPayload.msg"
  "msg/SimCommands.msg"
  "msg/Parameter.msg"
  "msg/SimPWM.msg"
  "msg/LinkStates.msg"
  "msg/Airdata.msg"
  "msg/Barometer.msg"
  "msg/Gnss.msg"
  "msg/Imu.msg"
  "msg/MavlinkHilStateQuaternion.msg"
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

ament_package()
