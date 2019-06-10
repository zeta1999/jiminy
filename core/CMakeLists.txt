# Minimum version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(wdc_core)

# Source files.
set(SRC
        "${CMAKE_CURRENT_SOURCE_DIR}/src/Utilities.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/Logger.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/Error.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/os/SharedMemory.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/os/Time.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/os/Lock.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/io/NetFile.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/io/AbstractIODevice.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/io/AbstractSocket.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/io/LocalSocket.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/io/FileManager.cc"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/io/FileDevice.cc"
)

# Define target library name and create it.
add_library(${PROJECT_NAME} STATIC ${SRC})

# Link with other libraries
target_link_libraries(${PROJECT_NAME} PUBLIC rt pthread)

# Add sources to library.
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>)