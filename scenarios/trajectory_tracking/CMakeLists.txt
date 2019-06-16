# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(trajectory_tracking)

# Find library and headers
find_package(urdfdom REQUIRED)

# Add sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/trajectory_tracking.cc")

# Make executables
add_executable(${PROJECT_NAME} ${SRC})

# Link with other libraries
target_link_libraries(${PROJECT_NAME} engine)

# Add c++11 flags
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)

# Install
install(TARGETS ${PROJECT_NAME} 
        RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}")