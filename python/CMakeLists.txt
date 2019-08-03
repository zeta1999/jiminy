# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(python)

# Python install helper
function(deploy_python_package TARGET_NAME)
    install (CODE "EXECUTE_PROCESS (COMMAND pip install --no-index --upgrade .
                                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME})")
endfunction()

# Compile and install python bindings
add_subdirectory(jiminy_pywrap)

# Install native python packages
deploy_python_package(jiminy_py)