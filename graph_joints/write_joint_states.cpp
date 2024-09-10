#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>
#include <fstream>
#include <array>
#include "examples_common.h"
#include "franka_ik_He.hpp"


// Function to write joint states to a file
void writeJointStates(const franka::RobotState& state, std::ofstream& file) {
    for (double joint : state.q) {
        file << joint << " ";
    }
    file << "\n";
}

int main() {
    // Connect to the robot
    franka::Robot robot("192.168.40.45");

    std::array<double, 7> starting_position = {{ M_PI_4, -M_PI_4, -M_PI_4, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

    franka::RobotState initial_state = robot.readOnce();

    // grabbing the beginning robot pose
    std::array<double, 16> pose_array = initial_state.O_T_EE_d;
    // for now, changing the coordinates I want to change
    pose_array[12] = 0.5;
    pose_array[13] = 0.0;
    pose_array[14] = 0.5;

    std::array<double, 7> q_actual = initial_state.q;

    // initializing the desired joint positions which will be sent to the robot
    std::array<double, 7> q_j_final;
    q_j_final = franka_IK_EE_CC(pose_array, q_actual[6], q_actual);

    //mapping so that i can use it in the control law
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_j_final_map(q_j_final.data());

    // std::array<double, 7> final_position = {{ 0, M_PI_4, 0, - M_PI_2, 0, M_PI_2, M_PI_4}};

    // Create MotionGenerator objects for the starting and final positions
    MotionGenerator starting_motion_generator(0.1, starting_position);
    MotionGenerator final_motion_generator(0.1, q_j_final);

    // Open a file to write joint states
    std::ofstream outputFile("joint_states.txt");
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open file for writing." << std::endl;
        return -1;
    }

    try {
        // Move to the starting position
        std::cout << "Moving to starting position..." << std::endl;
        robot.control(starting_motion_generator);

        // Perform the motion to the final position while collecting joint states
        std::cout << "Moving to final position..." << std::endl;
        robot.control([&final_motion_generator, &outputFile](const franka::RobotState& state, franka::Duration dt) -> franka::JointPositions {
            writeJointStates(state, outputFile);
            return final_motion_generator(state, dt);
        });

        std::cout << "Motion completed successfully." << std::endl;

    } catch (const franka::Exception& e) {
        std::cerr << "Franka exception: " << e.what() << std::endl;
        outputFile.close();
        return -1;
    }

    // Close the output file
    outputFile.close();

    // Call the Python script to generate the graph
    system("python3 ../plot_joint_states.py");   

    return 0;
}