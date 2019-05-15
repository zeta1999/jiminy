# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(exo_simu++)

# Set various flags
set(default_build_type "Release")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -Wextra")
set(CMAKE_CXX_FLAGS_RELEASE "-Wall -O3 -DNDEBUG")

# Find library and headers
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(urdfdom REQUIRED)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Pinocchio specific stuff
SET(COMPILE_FLAGS "-DPINOCCHIO_WITH_URDFDOM -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR")
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
file(GLOB HEADERS "include/*.h*")

# Make library
add_library(exo_simu++ STATIC ${SRC})

# Set include directory
target_include_directories(exo_simu++
                           PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}/exo_simu++>")

# Link with other libraries
target_link_libraries(exo_simu++ PUBLIC Eigen3::Eigen urdfdom_model boost_system)

# Activate c++11
target_compile_features(exo_simu++ PRIVATE cxx_std_11)

# Set installation options
# ----------------------------------------------
set_target_properties(exo_simu++ PROPERTIES PUBLIC_HEADER "${HEADERS}")
install(TARGETS exo_simu++ 
        EXPORT  ${PROJECT_NAME}
        ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"
        PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/exo_simu++")

# Create CMake packages for the build directory
# ----------------------------------------------
export(EXPORT ${PROJECT_NAME}
       FILE "${CMAKE_CURRENT_BINARY_DIR}/exo_simu++-targets.cmake")

# if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/exo_simu++-config.cmake)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/exo_simu++-config.cmake "include(CMakeFindDependencyMacro)\nfind_dependency(Eigen3)\ninclude(\"\${CMAKE_CURRENT_LIST_DIR}/exo_simu++-targets.cmake\")\nSET(COMPILE_FLAGS \"-DPINOCCHIO_WITH_URDFDOM -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR\")\nadd_definitions(${COMPILE_FLAGS})\n")
# endif()

# Create CMake packages for the install directory
# ----------------------------------------------
set(ConfigPackageLocation lib/cmake/exo_simu++)

install(EXPORT ${PROJECT_NAME}
        FILE exo_simu++-targets.cmake
        DESTINATION ${ConfigPackageLocation})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/exo_simu++-config.cmake
        DESTINATION ${ConfigPackageLocation})