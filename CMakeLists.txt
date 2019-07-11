# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(exo_simu)

# Set various flags
set(WARN_FULL "-Wall -Wextra -Weffc++ -pedantic -pedantic-errors \
               -Wcast-align -Wcast-qual -Wfloat-equal -Wformat=2 \
               -Wformat-nonliteral -Wformat-security -Wformat-y2k \
               -Wimport -Winit-self -Winvalid-pch -Wlong-long \
               -Wmissing-field-initializers -Wmissing-format-attribute \
               -Wmissing-include-dirs -Wmissing-noreturn -Wpacked \
               -Wpointer-arith -Wredundant-decls -Wshadow -Wstack-protector \
               -Wstrict-aliasing=2 -Wswitch-default -Wswitch-enum \
               -Wunreachable-code -Wunused -Wunused-parameter"
)
set(default_build_type "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wfatal-errors ${WARN_FULL}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG ${WARN_FULL} -Wno-unused-parameter")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Release type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()

# Add missing include & lib directory(ies)
link_directories(SYSTEM /opt/openrobots/lib)
link_directories(SYSTEM /opt/install/GCC/lib)
include_directories(SYSTEM /opt/install/GCC/include/)
include_directories(SYSTEM /opt/install/GCC/include/eigen3/)
include_directories(SYSTEM /usr/include/python2.7/)
include_directories(SYSTEM /usr/local/lib/python2.7/dist-packages/numpy/core/include/)

# Sub-project name(s)
add_subdirectory(core)
add_subdirectory(wdc)
add_subdirectory(python)
add_subdirectory(scenarios/trajectory_tracking)
add_subdirectory(scenarios/double_pendulum)
