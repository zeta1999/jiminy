# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(jiminy_pywrap)

# Find library and headers
#set(Python2_FIND_VIRTUALENV FIRST)
#find_package(Python2 COMPONENTS Interpreter Development NumPy) # Available after V3.12

# Set the include directories.
include_directories("include/")

# Add cpp sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/Module.cc")

# Python sources
file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SOURCES "src/python/*.py")

# Make  shared and static libraries (built from the same object files)
add_library(${PROJECT_NAME} SHARED ${SRC})
#add_library("${PROJECT_NAME}_static" STATIC ${SRC})

# Set include directory
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<INSTALL_INTERFACE:include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>"
)

# Link with other libraries
target_link_libraries(${PROJECT_NAME} PUBLIC core eigenpy boost_python)
#target_link_libraries(${PROJECT_NAME} PUBLIC ${PYTHON_LIBRARIES} core eigenpy boost_python)
#target_include_directories(${PROJECT_NAME} PUBLIC ${PYTHON_INCLUDE_DIRS})

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