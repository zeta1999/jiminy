cmake_minimum_required (VERSION 3.10)
project(SimpleSimulation)

# Set various flags
set(default_build_type "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -DNDEBUG")

# Release type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Find library and headers
find_package(exo_simu++ CONFIG REQUIRED)

# Add sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/SimpleSimulation.cpp")

# Make executables
add_executable(SimpleSimulation ${SRC})

# Link with asif++ library
target_link_libraries(SimpleSimulation PRIVATE exo_simu++)

# Add c++11 flags
target_compile_features(SimpleSimulation PRIVATE cxx_std_11)
