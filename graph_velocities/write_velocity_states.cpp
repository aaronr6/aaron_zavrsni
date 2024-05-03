#include <cmath>
#include <iostream>

#include <fstream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "examples_common.h"



void writeVelocityStates(double time, double v_x, double v_y, double v_z, std::ofstream& file) {
    file << time << " " << v_x << " " << v_y << " " << v_z << "\n";
}

int main() {
    // Connect to the robot
    franka::Robot robot("192.168.40.45");

    std::ofstream outputFile("velocity_states.txt");

  try {
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set the joint impedance.
    robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

    double time_max = 4.0;
    double v_max = 0.01;
    double angle = M_PI / 4.0;

    double a = 1.0;  // Semi-major axis
    double b = 1.0;  // Semi-minor axis

    double time = 0.0;
    robot.control([=, &time, &outputFile](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
        time += period.toSec();

        double cycle = std::floor(pow(-1.0, (time - std::fmod(time, time_max)) / time_max));
        double v = cycle * v_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / time_max * time));
        double v_x = std::cos(angle) * v;
        double v_y = std::sin(angle) * v;
        double v_z = -std::sin(angle) * v;

        // double v_x = a * std::cosh(time);
        // double v_y = 0.0;
        // double v_z = b * std::sinh(time);


        writeVelocityStates(time, v_x, v_y, v_z, outputFile);

        franka::CartesianVelocities output = {{v_x, v_y, v_z, 0.0, 0.0, 0.0}};
        if (time >= 2 * time_max) {
            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
            return franka::MotionFinished(output);
        }
        return output;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    outputFile.close();
    return -1;
  }

  outputFile.close();

  system("python3 ../plot_velocity_states.py");

  return 0;
}