# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(engine)

# Find library and headers
find_package(Boost REQUIRED)
find_package(urdfdom REQUIRED)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Pinocchio specific stuff
SET(COMPILE_FLAGS "-D PINOCCHIO_WITH_URDFDOM")
add_definitions(${COMPILE_FLAGS})

# Add sources
set(SRC 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ExoSimulator.cc"
)

# Make library
add_library(${PROJECT_NAME} STATIC ${SRC})

# Set include directory
target_include_directories(${PROJECT_NAME} PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
)

# Link with other libraries
target_link_libraries(${PROJECT_NAME} PUBLIC urdfdom_model boost_system)

# Activate c++14
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Install
install(DIRECTORY "include/exo_simu" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(TARGETS ${PROJECT_NAME} 
    EXPORT  ${PROJECT_NAME}
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)
