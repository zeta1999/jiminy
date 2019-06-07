cmake_minimum_required (VERSION 3.10)

# Project name
project(trajectory_tracking)

# Set various flags
set(default_build_type "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -D NDEBUG")

# Update search path
#set(engine_DIR "$ENV{HOME}/wdc_workspace/build/pc/exo_simu")
#include_directories(${CMAKE_SOURCE_DIR}/engine)
link_directories(/opt/openrobots/lib)
#add_dependencies(trajectory_tracking engine)

# Release type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Find library and headers
find_package(urdfdom REQUIRED)

# Add sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/trajectory_tracking.cpp")

# Make executables
add_executable(trajectory_tracking ${SRC})

# Link with asif++ library
target_link_libraries(trajectory_tracking engine)

# Add c++11 flags
target_compile_features(trajectory_tracking PRIVATE cxx_std_11)

# Set installation options
# ----------------------------------------------
set_target_properties(engine PROPERTIES PUBLIC_HEADER "${HEADERS}")
install(TARGETS trajectory_tracking 
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")