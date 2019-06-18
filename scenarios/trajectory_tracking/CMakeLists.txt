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

# Activate c++14
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Install
install(TARGETS ${PROJECT_NAME} 
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
)