# 3.28 is required for the EXCLUDE_FROM_ALL keyword of FetchContent_Declare,
# which keeps the install rules of the fetched dependencies out of this project.
# Note this is newer than the cmake shipped by Ubuntu 22.04 (3.22).
cmake_minimum_required(VERSION 3.28)
project(last_letter VERSION 2.102.0 LANGUAGES CXX)

# This is the top level of a repository holding the simulation library (lib/)
# and the frontends that drive it (frontends/). Everything global lives here:
# toolchain settings, the dependencies found from the system, and the build
# options. The components themselves are added at the bottom of this file.

# Default to an optimized build.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type." FORCE)
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

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

# Keep our own binaries and shared libraries at the top of the build tree
# instead of letting them follow the source layout into build/lib/ and
# build/frontends/ardupilot/. Two things depend on this: the BUILD_RPATH of
# $ORIGIN, which is what lets the bridge and the Python extension find
# liblast_letter_lib.so before installation, and the documented ./build/all_tests
# invocation. ARCHIVE is deliberately left alone, so the static FetchContent
# dependencies stay in their own subdirectories.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

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

# Configure compile options.
add_compile_options(-Wall -Wextra -Wpedantic)

# Build options. All three default OFF, so a bare `cmake` build produces only
# the C++ library and needs neither Python dev headers nor a network fetch of
# googletest.
option(LAST_LETTER_BUILD_PYTHON "Build the pybind11 Python extension (cpp_last_letter)" OFF)
option(LAST_LETTER_BUILD_TESTS "Build the C++ googletest suite (all_tests)" OFF)
option(LAST_LETTER_BUILD_ARDUPILOT "Build the ArduPilot SITL JSON bridge (last_letter_ardupilot)" OFF)

# Find dependencies
find_package(Eigen3 REQUIRED NO_MODULE)

# Boost has shipped its own BoostConfig.cmake since 1.70, and CMake 3.30 removed
# its bundled FindBoost module behind this policy. Without the opt-in, every
# configure on CMake 3.30+ warns; with it, find_package() goes straight to config
# mode.
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

find_package(Boost REQUIRED)

# find_package(NLopt REQUIRED) # Will enable later with trimmer

# The scikit-build-core / uv package build turns LAST_LETTER_BUILD_PYTHON ON
# (see [tool.scikit-build] in pyproject.toml); that build is the one whose
# extension actually gets imported.
if(LAST_LETTER_BUILD_PYTHON)
  find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
  message(STATUS "Current Python site-packages dir: ${Python3_SITELIB}")
  set(PYBIND11_FINDPYTHON ON)
  find_package(pybind11 PATHS ${Python3_SITELIB}/pybind11/share/cmake/pybind11 CONFIG REQUIRED)
endif()

# All fetched dependencies (yaml-cpp, DataTamer + its vendored mcap, googletest)
# are built STATIC and position-independent, so they embed directly into
# last_letter_lib.so and leave no side-car .so files to ship next to the Python
# extension. BUILD_SHARED_LIBS is left at its default (OFF); PIC must be enabled
# here, before lib/ makes the first FetchContent call, so every dependency is
# compiled with -fPIC.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Where liblast_letter_lib.so and the Python extension are installed. When we
# are building the Python package they must end up in the Python package
# directory, since the extension's rpath is $ORIGIN and the two have to stay
# adjacent. The value is the *Python package* name, which happens to match
# PROJECT_NAME but is not the same thing - do not substitute one for the other.
#
# Note: this also relocates the library for a plain `cmake --install` that
# happens to have LAST_LETTER_BUILD_PYTHON on; enabling the option outside the
# pyproject.toml-driven build is only meant to check that the bindings still
# compile, so such an install is not expected.
#
# Set here rather than in lib/ because frontends/ardupilot needs it too, and a
# variable set in one subdirectory is not visible in its siblings.
if(LAST_LETTER_BUILD_PYTHON)
  set(LAST_LETTER_LIB_DESTINATION last_letter)
else()
  set(LAST_LETTER_LIB_DESTINATION lib)
endif()

# The simulation library, the Python bindings and the C++ test suite.
add_subdirectory(lib)

# Frontends that drive the library.
if(LAST_LETTER_BUILD_ARDUPILOT)
  add_subdirectory(frontends/ardupilot)
endif()

# Stage runtime aircraft-model data where the consumer of this build looks for
# it. For a plain build the destination is the absolute
# $HOME/last_letter_models, deliberately ignoring CMAKE_INSTALL_PREFIX.
#
# The wheel build cannot use that path. An absolute DESTINATION escapes
# scikit-build-core's staging directory. They go inside the Python package
# instead, next to the extension, where last_letter.paths.models_path() finds
# them.
if(LAST_LETTER_BUILD_PYTHON)
  install(DIRECTORY models/ DESTINATION ${LAST_LETTER_LIB_DESTINATION}/models)
else()
  install(DIRECTORY models/ DESTINATION $ENV{HOME}/last_letter_models)
endif()
