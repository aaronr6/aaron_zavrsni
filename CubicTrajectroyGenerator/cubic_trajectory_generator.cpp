#include "cubic_trajectory_generator.h"
#include <cmath>
#include <iostream>

CubicTrajectoryGenerator::CubicTrajectoryGenerator() {}

void CubicTrajectoryGenerator::setTargetPose(const Eigen::Vector3d& target_pose) {
    target_pose_ = target_pose;
}

void CubicTrajectoryGenerator::calculateTrajectory(const Eigen::Vector3d& current_pose) {
    current_pose_ = current_pose;
    generateCubicCurve();
}

std::vector<Eigen::Vector3d> CubicTrajectoryGenerator::getTrajectoryPoints() const {
    return trajectory_points_;
}

void CubicTrajectoryGenerator::generateCubicCurve() {
    trajectory_points_.clear();

    const int num_points = 100;
    for (int i = 0; i <= num_points; ++i) {
        double t = static_cast<double>(i) / num_points;
        double t2 = t * t;
        double t3 = t2 * t;

        Eigen::Vector3d point;
        point = (2*t3 - 3*t2 + 1) * current_pose_ +
                (t3 - 2*t2 + t) * (target_pose_ - current_pose_) +
                (-2*t3 + 3*t2) * target_pose_ +
                (t3 - t2) * (target_pose_ - current_pose_);
        
        trajectory_points_.push_back(point);
    }
}
