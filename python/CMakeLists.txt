# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(python)

# Python install helper
function(deploy_python_package TARGET_NAME)
    install (CODE "EXECUTE_PROCESS (COMMAND pip install --upgrade .
                                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${TARGET_NAME})")
endfunction()

# Set Python version
set(PYTHON_VERSION_MAJOR 2)
set(PYTHON_VERSION_MINOR 7)
set(PYTHON_VERSION ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR})
set(PYTHON_VERSION_NO_DOTS ${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
set(PYTHON_SITELIB lib/python${PYTHON_VERSION}/site-packages)

# Compile and install python bindings
add_subdirectory(exo_simu_pywrap)

# Install native python packages
deploy_python_package(exo_simu_py)