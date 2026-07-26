# 3.28 is required for the EXCLUDE_FROM_ALL keyword of FetchContent_Declare,
# which keeps the install rules of the fetched dependencies out of this project.
# Note this is newer than the cmake shipped by Ubuntu 22.04 (3.22).
cmake_minimum_required(VERSION 3.28)
project(last_letter_lib VERSION 2.1.0 LANGUAGES CXX)

# Default to a user-writable prefix instead of /usr/local, which needs root.
# Only applies when the caller did not pass -DCMAKE_INSTALL_PREFIX; in
# particular scikit-build-core sets it to the wheel staging directory, and that
# choice is preserved.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local"
      CACHE PATH "Installation prefix." FORCE)
endif()
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Generate compile_commands.json

message(STATUS "Building ${PROJECT_NAME}")

# Set compiler standards
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Whole project builds at C++17 (required by the DataTamer logging backend).
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependences
# find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED)

# find_package(NLopt REQUIRED) # Will enable later with trimmer

# Python extension (pybind11). OFF by default, so a bare `cmake` build produces
# only the C++ library and needs neither Python dev headers nor pybind11. The
# scikit-build-core / uv package build turns it ON (see [tool.scikit-build] in
# pyproject.toml); that build is the one whose extension actually gets imported.
option(LAST_LETTER_BUILD_PYTHON "Build the pybind11 Python extension (cpp_last_letter_lib)" OFF)
if(LAST_LETTER_BUILD_PYTHON)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  message(STATUS "Current Python site-packages dir: ${Python3_SITELIB}")
  set(PYBIND11_FINDPYTHON ON)
  find_package(pybind11 PATHS ${Python3_SITELIB}/pybind11/share/cmake/pybind11 CONFIG REQUIRED)
endif()

# add_subdirectory(external/pybind11)

# All fetched dependencies (yaml-cpp, DataTamer + its vendored mcap, googletest)
# are built STATIC and position-independent, so they embed directly into
# last_letter_lib.so and leave no side-car .so files to ship next to the Python
# extension. BUILD_SHARED_LIBS is left at its default (OFF); PIC must be enabled
# before the first FetchContent so every dependency is compiled with -fPIC.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Every FetchContent_Declare below passes EXCLUDE_FROM_ALL. Those projects carry
# their own install() rules targeting <prefix>/include and <prefix>/lib, which
# `cmake --install` would otherwise execute, scattering third-party headers into
# our prefix. EXCLUDE_FROM_ALL makes CMake ignore a subdirectory's install rules.
# The targets themselves are still built, because last_letter_lib links them.

# Import yaml-cpp
include(FetchContent)

FetchContent_Declare(
  yaml_cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG        yaml-cpp-0.9.0
  EXCLUDE_FROM_ALL
)

# Set options before the library is added
set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(yaml_cpp)

# Import DataTamer, the data-logging backend (MCAP output, PlotJuggler-native).
# The buildable CMake project lives in the data_tamer_cpp/ subdirectory, hence
# SOURCE_SUBDIR (honored by FetchContent_MakeAvailable on CMake >= 3.18).
# DATA_TAMER_BUILD_ROS must be OFF so it uses its vendored mcap (3rdparty/mcap)
# instead of requiring a ROS 2 / mcap_vendor environment.
FetchContent_Declare(
  data_tamer
  GIT_REPOSITORY https://github.com/PickNikRobotics/data_tamer.git
  GIT_TAG        1.0.3
  SOURCE_SUBDIR  data_tamer_cpp
  EXCLUDE_FROM_ALL
)
set(DATA_TAMER_BUILD_ROS OFF CACHE BOOL "" FORCE)
set(DATA_TAMER_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(DATA_TAMER_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(data_tamer)
# DataTamer compiles its own target with -Werror; drop it so a compiler version
# other than the one upstream tested can't fail our build on a stray warning.
if(TARGET data_tamer AND (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang"))
  target_compile_options(data_tamer PRIVATE -Wno-error)
endif()


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
  # "/usr/local/include" # Used to include nlopt
)

# Declare the project library
add_library(last_letter_lib SHARED
  src/math_utils.cpp
  src/uav_utils.cpp
  src/geo_mag_declination.cpp
  src/prog_utils.cpp
  src/systems.cpp
  src/environment.cpp
  src/gravity.cpp
  src/aerodynamics.cpp
  src/propulsion/propulsion.cpp
  src/propulsion/no_engine.cpp
  src/propulsion/thruster_simple.cpp
  src/propulsion/beard_engine.cpp
  src/propulsion/piston_engine.cpp
  src/propulsion/electric_engine.cpp
  src/propulsion/omega_control_engine.cpp
  src/ground_reaction/ground_reaction.cpp
  src/ground_reaction/no_ground_reactions.cpp
  src/ground_reaction/panos_contact_points.cpp
  src/ground_reaction/point_friction.cpp
  src/dynamics.cpp
  src/kinematics.cpp
  src/sensors.cpp
  src/uav_model.cpp
  src/logging/logging.cpp
)
target_link_libraries(last_letter_lib
  Eigen3::Eigen
  yaml-cpp
  Boost::boost
  data_tamer
)

# add_library(lib_trimmer src/trimmer.cpp)
# target_link_libraries(lib_trimmer
# ${NLOPT_LIBRARIES}
# lib_uav_model)
if(LAST_LETTER_BUILD_PYTHON)
  list(APPEND pybind11_sources
    src/python_api.cpp
  )
  pybind11_add_module(cpp_last_letter_lib ${pybind11_sources})
  set_target_properties(cpp_last_letter_lib PROPERTIES
      INSTALL_RPATH "$ORIGIN"
      BUILD_RPATH "$ORIGIN"
  )
  target_link_libraries(cpp_last_letter_lib PRIVATE last_letter_lib)
  install(TARGETS cpp_last_letter_lib LIBRARY DESTINATION ${PROJECT_NAME})
endif()

# Stage runtime aircraft-model data into the folder the library reads from at
# runtime. That location is hardcoded as $HOME/last_letter_models in both
# src/prog_utils.cpp (C++) and src/last_letter_lib/model_creator.py (Python), so
# the DESTINATION is an absolute path mirroring it, not CMAKE_INSTALL_PREFIX.
# These are install rules: they run at `cmake --install` time, NOT at configure,
# so plain `cmake`/`cmake --build` no longer writes to $HOME.
install(DIRECTORY last_letter_models DESTINATION $ENV{HOME})

# Export the header files for use by consumers of the library
# ament_export_include_directories(include)  # Unnecessary according to https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/#building-a-library
# Export the library binaries themselves
# ament_export_libraries(
# last_letter_lib
# )  # Unnecessary according to https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/#building-a-library

# # Export the libraries
# ament_export_targets(
# export_last_letterlib HAS_LIBRARY_TARGET
# )
# # Declare dependencies of this library to its consumers
# ament_export_dependencies(
# Eigen3
# )

# C++ unit tests (googletest). OFF by default, so an ordinary library / Python
# extension build neither fetches googletest nor compiles the test binary.
# Enable with -DLAST_LETTER_BUILD_TESTS=ON (see build command below).
option(LAST_LETTER_BUILD_TESTS "Build the C++ googletest suite (all_tests)" OFF)
if(LAST_LETTER_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip
    EXCLUDE_FROM_ALL
  )

  FetchContent_MakeAvailable(googletest)

  enable_testing()

  list(APPEND test_sources
    tests/cpp/test_utils.cpp
    tests/cpp/test_prog_utils.cpp
    tests/cpp/test_uav_utils.cpp
    tests/cpp/test_math_utils.cpp
    tests/cpp/test_systems.cpp
    tests/cpp/test_environment.cpp
    tests/cpp/test_aerodynamics.cpp
    tests/cpp/test_propulsion.cpp
    tests/cpp/test_ground_reaction.cpp
    tests/cpp/test_gravity.cpp
    tests/cpp/test_dynamics.cpp
    tests/cpp/test_kinematics.cpp
    tests/cpp/test_uav_model.cpp
    tests/cpp/test_loop_rate.cpp

    # tests/cpp/test_pybind.cpp
  )
  add_executable(all_tests ${test_sources})
  target_include_directories(all_tests PUBLIC
    tests/cpp
  )
  target_link_libraries(all_tests
    GTest::gtest_main
    yaml-cpp
    last_letter_lib
  )

  # Perhaps copy over the test data to the build folder?
  include(GoogleTest)
  gtest_discover_tests(all_tests)
endif()

# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)

# install(DIRECTORY tests/cpp/test_parameters DESTINATION share/${PROJECT_NAME})

# endif()
install(
  DIRECTORY include/ DESTINATION include
)

list(APPEND exported_libs
  last_letter_lib
)
install(
  TARGETS ${exported_libs}
  LIBRARY DESTINATION ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  EXPORT last_letter_lib_targets
)
