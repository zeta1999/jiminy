# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(exo_simu)

# Set various flags
set(default_build_type "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -D NDEBUG")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Release type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

# Add missing include & lib directory(ies)
include_directories(/opt/install/GCC/include/)
include_directories(/usr/include/python2.7/)
include_directories(/usr/local/lib/python2.7/dist-packages/numpy/core/include/)
link_directories(/opt/openrobots/lib)
link_directories(/opt/install/GCC/lib)

# Sub-project name(s)
add_subdirectory(core)
add_subdirectory(engine)
add_subdirectory(python)
add_subdirectory(scenarios/trajectory_tracking)