# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(jiminy_pywrap)

# Set the include directories.
include_directories("include/")

# Add cpp sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/Module.cc")

# Python sources
file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SOURCES "src/python/*.py")

# Make library
add_library(${PROJECT_NAME} SHARED ${SRC})

# Set include directory
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<INSTALL_INTERFACE:include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>"
)

# Link with other libraries
target_link_libraries(${PROJECT_NAME} PUBLIC core eigenpy ${BOOST_PYTHON_LIB})
target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${NUMPY_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

# Activate c++14
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 14
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# Install
install(DIRECTORY "include/jiminy" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}")
install(TARGETS ${PROJECT_NAME}
    DESTINATION "${PYTHON_SITELIB}/jiminy"
)
install(FILES ${${PROJECT_NAME}_PYTHON_SOURCES}
    DESTINATION "${PYTHON_SITELIB}/jiminy"
)