# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(engine)

# Set various flags
set(default_build_type "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -D NDEBUG")

# Find library and headers
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(urdfdom REQUIRED)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Pinocchio specific stuff
SET(COMPILE_FLAGS "-D PINOCCHIO_WITH_URDFDOM")
add_definitions(${COMPILE_FLAGS})

# Release type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


# Add sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/ExoSimulator.cpp")

# List headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h*")

# Make library
add_library(engine STATIC ${SRC})

# Set include directory
target_include_directories(engine
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/engine>")

# Link with other libraries
target_link_libraries(engine PUBLIC Eigen3::Eigen urdfdom_model boost_system)

# Activate c++11
target_compile_features(engine PRIVATE cxx_std_11)

# Set installation options
# ----------------------------------------------
set_target_properties(engine PROPERTIES PUBLIC_HEADER "${HEADERS}")
install(TARGETS engine 
        EXPORT  ${PROJECT_NAME}
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/engine")