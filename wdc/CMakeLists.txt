# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(wdc)

# Add sources
set(SRC 
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ExoModel.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ExoController.cc"
)

# Make library
add_library(${PROJECT_NAME} STATIC ${SRC})

# Set include directory
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<INSTALL_INTERFACE:include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>"
)

# Link with other libraries
target_link_libraries(${PROJECT_NAME} PUBLIC core)

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