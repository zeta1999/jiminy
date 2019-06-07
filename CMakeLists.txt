# Minimum version required
cmake_minimum_required (VERSION 3.10)

# Project name
project(exo_simu)

#
include_directories(/opt/install/GCC/include/)

# Sub-project name(s)
add_subdirectory(engine)
add_subdirectory(scenarios/trajectory_tracking)