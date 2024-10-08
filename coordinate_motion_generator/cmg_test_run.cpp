#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>
#include <fstream>
#include <array>
#include "coordinate_motion_generator.h"

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

    std::array<double, 3> starting_position = {{0.3, 0.3, 0.3}};
    std::array<double, 3> final_position = {{0.4, 0.4, 0.4}};

    // Create MotionGenerator objects for the starting and final positions
    MotionGenerator starting_motion_generator(0.2, starting_position);
    MotionGenerator final_motion_generator(0.1, final_position);

    // Open a file to write joint states
    std::ofstream outputFile("cmg.txt");
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
        robot.control([&final_motion_generator, &outputFile](const franka::RobotState& state, franka::Duration dt) -> franka::CartesianPose {
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
    // system("python3 ../plot_joint_states.py");   

    return 0;
}