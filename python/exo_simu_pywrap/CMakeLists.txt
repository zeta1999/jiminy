# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(exo_simu_pywrap)

# Set the include directories.
include_directories("include/")

# Add cpp sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/Module.cc")

# Python sources
file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SOURCES "src/python/*.py")

# Make library
add_library(${PROJECT_NAME} SHARED ${SRC})

# Link with other libraries
target_link_libraries(${PROJECT_NAME} PUBLIC engine eigenpy boost_python)

# Activate c++14
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Install
install(TARGETS ${PROJECT_NAME}
    DESTINATION "${PYTHON_SITELIB}/exo_simu"
)
install(FILES ${${PROJECT_NAME}_PYTHON_SOURCES}
    DESTINATION "${PYTHON_SITELIB}/exo_simu"
)
