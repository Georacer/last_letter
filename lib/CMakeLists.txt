# The simulation library itself: the C++ target last_letter_lib, the pybind11
# extension that exposes it to Python, and the C++ test suite. Toolchain
# settings, the build options and the system dependencies come from the
# top-level CMakeLists.txt; this file deliberately calls no project().

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
if(TARGET data_tamer)
  target_compile_options(data_tamer PRIVATE -Wno-error)
endif()

# Force-depend against cstdint, since recent versions of data_tamer forgot to
# pass the dependency transitively.
#
# TODO: send a PR to https://github.com/facontidavide/data_tamer adding the
# missing #include <cstdint> to the vendored mcap headers, and delete this block
# once a release carries it. The fix belongs upstream -- every project that
# vendors this mcap snapshot breaks the same way on GCC 15.
foreach(mcap_consumer data_tamer mcap_lib)
  if(TARGET ${mcap_consumer})
    target_compile_options(${mcap_consumer} PUBLIC -include cstdint)
  endif()
endforeach()

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
  src/body/ardupilot_mixer.cpp
  src/body/body.cpp
  src/body/linear_copter.cpp
  src/control/evaluate.cpp
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

# include/ is carried on the target rather than through a directory-scoped
# include_directories(), so that everything linking last_letter_lib finds the
# public headers - including the frontends, which live in a sibling directory
# and would not inherit a directory-scoped setting.
# src/ is on the private include path so the implementation can reach internal
# headers that must not be installed and depended upon by downstream packages.
target_include_directories(last_letter_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    src
)

# Eigen and Boost are genuine PUBLIC dependencies: the installed headers include
# <Eigen/Eigen> almost everywhere and <boost/numeric/odeint.hpp> in systems.hpp,
# so a consumer needs them on its include path too.
#
# yaml-cpp and data_tamer are wrapped in $<BUILD_INTERFACE:> instead. They are
# FetchContent targets declared EXCLUDE_FROM_ALL, so they belong to no export
# set, and naming them in the link interface unguarded makes the
# install(EXPORT) below fail to generate. The genex keeps them for our own
# build - the library sources, all_tests and the bridge all still get their
# include directories - and drops them from the installed link interface.
target_link_libraries(last_letter_lib
  PUBLIC
    Eigen3::Eigen
    Boost::boost
    $<BUILD_INTERFACE:yaml-cpp>
    $<BUILD_INTERFACE:data_tamer>
)

# add_library(lib_trimmer src/trimmer.cpp)
# target_link_libraries(lib_trimmer
# ${NLOPT_LIBRARIES}
# lib_uav_model)

if(LAST_LETTER_BUILD_PYTHON)
  list(APPEND pybind11_sources
    src/python_api.cpp
  )
  pybind11_add_module(cpp_last_letter ${pybind11_sources})
  set_target_properties(cpp_last_letter PROPERTIES
      INSTALL_RPATH "$ORIGIN"
      BUILD_RPATH "$ORIGIN"
  )
  target_link_libraries(cpp_last_letter PRIVATE last_letter_lib)
  # Same destination as liblast_letter_lib.so, so the $ORIGIN rpath above always
  # resolves regardless of which install layout is in play.
  install(TARGETS cpp_last_letter LIBRARY DESTINATION ${LAST_LETTER_LIB_DESTINATION})
endif()

# C++ unit tests (googletest).
if(LAST_LETTER_BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
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
    tests/cpp/test_body.cpp
    tests/cpp/test_propulsion.cpp
    tests/cpp/test_ground_reaction.cpp
    tests/cpp/test_gravity.cpp
    tests/cpp/test_dynamics.cpp
    tests/cpp/test_kinematics.cpp
    tests/cpp/test_uav_model.cpp
    tests/cpp/test_loop_rate.cpp
    tests/cpp/test_control.cpp

    # tests/cpp/test_pybind.cpp
  )
  add_executable(all_tests ${test_sources})
  target_include_directories(all_tests PUBLIC
    tests/cpp
  )
  # The tests read YAML fixtures from tests/cpp. Bake in the absolute path of
  # that directory rather than resolving it against the working directory, so
  # all_tests can be run from anywhere.
  target_compile_definitions(all_tests PRIVATE
    LAST_LETTER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/cpp"
  )
  target_link_libraries(all_tests
    GTest::gtest_main
    yaml-cpp
    last_letter_lib
  )

  include(GoogleTest)
  gtest_discover_tests(all_tests)
endif()

# Export the header files for use by consumers of the library.
#
# Skipped for the Python build, as they are not useful to a Python project.
if(NOT LAST_LETTER_BUILD_PYTHON)
  install(
    DIRECTORY include/ DESTINATION include
  )
endif()

list(APPEND exported_libs
  last_letter_lib
)
install(
  TARGETS ${exported_libs}
  EXPORT last_letter_lib_targets
  LIBRARY DESTINATION ${LAST_LETTER_LIB_DESTINATION}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Turn the export set collected above into an installed CMake package.
# Guarded on the Python build because that layout relocates
# liblast_letter_lib.so into <prefix>/last_letter_lib/ for the wheel; a targets
# file generated from it would point consumers at a path that exists only
# inside a wheel.
if(NOT LAST_LETTER_BUILD_PYTHON)
  include(CMakePackageConfigHelpers)

  install(
    EXPORT last_letter_lib_targets
    FILE last_letter_libTargets.cmake
    NAMESPACE last_letter_lib::
    DESTINATION lib/cmake/last_letter_lib
  )

  configure_package_config_file(
    cmake/last_letter_libConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/last_letter_libConfig.cmake
    INSTALL_DESTINATION lib/cmake/last_letter_lib
  )

  write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/last_letter_libConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
  )

  install(
    FILES
      ${CMAKE_CURRENT_BINARY_DIR}/last_letter_libConfig.cmake
      ${CMAKE_CURRENT_BINARY_DIR}/last_letter_libConfigVersion.cmake
    DESTINATION lib/cmake/last_letter_lib
  )
endif()
