cmake_minimum_required(VERSION 3.4)
project(hello_library)

set(CMAKE_PREFIX_PATH "/home/crta/libfranka/lib/cmake/Franka" ${CMAKE_PREFIX_PATH})
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(PROJECT_SOURCES
    src/robot_comm/joint_states.cpp
    src/movement_joint/joint_position.cpp
    src/examples_common.cpp
    # src/movement_cartesian/linear_motion.cpp
)


add_library(panda_library SHARED ${PROJECT_SOURCES})
target_include_directories(panda_library PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>/home/crta/libfranka/build/
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(panda_library PRIVATE Franka::Franka)


add_executable(joint_states src/robot_comm/joint_states_main.cpp)
target_link_libraries(joint_states PRIVATE panda_library Eigen3::Eigen Franka::Franka)

add_executable(joint_position src/movement_joint/joint_position_main.cpp)
target_link_libraries(joint_position PRIVATE panda_library Eigen3::Eigen Franka::Franka)

# add_executable(linear_motion src/movement_cartesian/linear_motion_main.cpp)
# target_link_libraries(linear_motion PRIVATE panda_library Eigen3::Eigen Franka::Franka)

