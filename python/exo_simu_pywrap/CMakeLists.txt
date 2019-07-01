# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(exo_simu_pywrap)

# Set the include directories.
include_directories("include/")

# Add cpp sources
set(SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/Module.cc")

# Make library
add_library(${PROJECT_NAME} SHARED ${SRC})

# Link with other libraries
target_link_libraries(${PROJECT_NAME} PUBLIC wdc_core engine eigenpy boost_python wdc_core)

# Python sources
file(GLOB_RECURSE ${PROJECT_NAME}_PYTHON_SOURCES "src/python/*.py")

# Add c++11 flags
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_11)

# Install
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION "${PYTHON_SITELIB}/exo_simu"
  )
install(
  FILES ${${PROJECT_NAME}_PYTHON_SOURCES}
  DESTINATION "${PYTHON_SITELIB}/exo_simu"
  )