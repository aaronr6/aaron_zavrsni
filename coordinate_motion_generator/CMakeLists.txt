cmake_minimum_required(VERSION 3.4)
project(cmg_test_run)

# Specify the path to the Franka library
set(CMAKE_PREFIX_PATH "/home/crta/libfranka/lib/cmake/Franka" ${CMAKE_PREFIX_PATH})

# Find the required packages
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the source files
set(PROJECT_SOURCES
    cmg_test_run.cpp
    coordinate_motion_generator.cpp
)

# Create an executable from the source files
add_executable(cmg_test_run ${PROJECT_SOURCES})

# Specify the include directories
target_include_directories(cmg_test_run PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link the required libraries
target_link_libraries(cmg_test_run PRIVATE Franka::Franka)