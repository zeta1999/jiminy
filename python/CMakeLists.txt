# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(python)

# Python install helper
function(deploy_python_package TARGET_NAME)
    install (CODE "EXECUTE_PROCESS (COMMAND pip install --upgrade .
                                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME})")
endfunction()

SET (PYTHON_VERSION_MAJOR 2)
SET (PYTHON_VERSION_MINOR 7)
SET (PYTHON_VERSION ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR})
SET (PYTHON_SITELIB lib/python${PYTHON_VERSION}/site-packages)

# Compile and install python bindings
add_subdirectory(exo_simu_pywrap)

# Install native python packages
deploy_python_package(exo_simu_py)