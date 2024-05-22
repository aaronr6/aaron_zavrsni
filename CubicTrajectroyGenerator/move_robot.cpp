#include <iostream>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <Eigen/Dense>
#include "cubic_trajectory_generator.h"

void moveAlongTrajectory(franka::Robot& robot, const std::vector<Eigen::Vector3d>& trajectory_points) {
    size_t index = 0;
    robot.control([&index, &trajectory_points](const franka::RobotState&, franka::Duration period) -> franka::JointVelocities {
        franka::JointVelocities velocities = {0, 0, 0, 0, 0, 0, 0};
        if (index < trajectory_points.size()) {
            // Calculate desired velocities to move towards the next point
            Eigen::Vector3d desired_position = trajectory_points[index];
            // Assuming a simple proportional controller for demonstration
            double k_p = 0.1;  // Proportional gain
            Eigen::Vector3d current_position; // Obtain from the robot state
            // Here you should actually use the real current position from the state
            Eigen::Vector3d error = desired_position - current_position;
            Eigen::Vector3d velocity = k_p * error;

            // Convert to joint velocities (this is a simplification, actual implementation requires Jacobian etc.)
            // Set the desired velocity values
            velocities.dq[0] = velocity[0];
            velocities.dq[1] = velocity[1];
            velocities.dq[2] = velocity[2];

            index++;
        } else {
            // Stop the robot if the trajectory is completed
            return franka::MotionFinished(velocities);
        }
        return velocities;
    });
}

int main() {
    try {
        franka::Robot robot("192.168.1.1");  // Replace with the actual IP of your robot
        franka::Model model = robot.loadModel();

        // Get current robot pose
        franka::RobotState robot_state = robot.readOnce();
        Eigen::Vector3d current_pose(robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14]);

        // Set target pose
        Eigen::Vector3d target_pose(0.5, 0.5, 0.5);  // Example target pose

        // Create trajectory generator
        CubicTrajectoryGenerator trajectory_generator;
        trajectory_generator.setTargetPose(target_pose);
        trajectory_generator.calculateTrajectory(current_pose);

        // Get trajectory points
        std::vector<Eigen::Vector3d> trajectory_points = trajectory_generator.getTrajectoryPoints();

        // Move the robot along the calculated trajectory points
        moveAlongTrajectory(robot, trajectory_points);
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}

