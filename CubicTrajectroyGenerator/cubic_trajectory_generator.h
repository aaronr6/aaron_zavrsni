#ifndef CUBICTRAJECTORYGENERATOR_H
#define CUBICTRAJECTORYGENERATOR_H

#include <vector>
#include <Eigen/Dense>
#include <franka/robot.h>
#include <franka/model.h>

class CubicTrajectoryGenerator {
public:
    CubicTrajectoryGenerator();
    void setTargetPose(const Eigen::Vector3d& target_pose);
    void calculateTrajectory(const Eigen::Vector3d& current_pose);
    std::vector<Eigen::Vector3d> getTrajectoryPoints() const;

private:
    Eigen::Vector3d current_pose_;
    Eigen::Vector3d target_pose_;
    std::vector<Eigen::Vector3d> trajectory_points_;
    void generateCubicCurve();
};

#endif // CUBICTRAJECTORYGENERATOR_H
