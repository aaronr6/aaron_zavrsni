// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "coordinate_motion_generator.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 3> p_goal)
    : p_goal_(p_goal.data()) {
  dp_max_ *= speed_factor;
  ddp_max_start_ *= speed_factor;
  ddp_max_goal_ *= speed_factor;
  dp_max_sync_.setZero();
  p_start_.setZero();
  delta_p_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  p_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector3d* delta_p_d) const {
  Vector3i sign_delta_p;
  sign_delta_p << delta_p_.cwiseSign().cast<int>();
  Vector3d t_d = t_2_sync_ - t_1_sync_;
  Vector3d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 3> tcp_motion_finished{};

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_p_[i]) < kDeltaQMotionFinished) {
      (*delta_p_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_p_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dp_max_sync_[i] * sign_delta_p[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_p_d)[i] = p_1_[i] + (t - t_1_sync_[i]) * dp_max_sync_[i] * sign_delta_p[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_p_d)[i] =
            delta_p_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dp_max_sync_[i] * sign_delta_p[i];
      } else {
        (*delta_p_d)[i] = delta_p_[i];
        tcp_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(tcp_motion_finished.cbegin(), tcp_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector3d dp_max_reach(dp_max_);
  Vector3d t_f = Vector3d::Zero();
  Vector3d delta_t_2 = Vector3d::Zero();
  Vector3d t_1 = Vector3d::Zero();
  Vector3d delta_t_2_sync = Vector3d::Zero();
  Vector3i sign_delta_p;
  sign_delta_p << delta_p_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_p_[i]) > kDeltaQMotionFinished) {
      if (std::abs(delta_p_[i]) < (3.0 / 4.0 * (std::pow(dp_max_[i], 2.0) / ddp_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dp_max_[i], 2.0) / ddp_max_goal_[i]))) {
        dp_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_p_[i] * sign_delta_p[i] *
                                    (ddp_max_start_[i] * ddp_max_goal_[i]) /
                                    (ddp_max_start_[i] + ddp_max_goal_[i]));
      }
      t_1[i] = 1.5 * dp_max_reach[i] / ddp_max_start_[i];
      delta_t_2[i] = 1.5 * dp_max_reach[i] / ddp_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_p_[i]) / dp_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_p_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddp_max_goal_[i] + ddp_max_start_[i]);
      double b = -1.0 * max_t_f * ddp_max_goal_[i] * ddp_max_start_[i];
      double c = std::abs(delta_p_[i]) * ddp_max_goal_[i] * ddp_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dp_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dp_max_sync_[i] / ddp_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dp_max_sync_[i] / ddp_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_p_[i] / dp_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      p_1_[i] = (dp_max_sync_)[i] * sign_delta_p[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::CartesianPose MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    p_start_ = Vector3d(robot_state.O_T_EE_d[12], robot_state.O_T_EE_d[13], robot_state.O_T_EE_d[14]);
    delta_p_ = p_goal_ - p_start_;
    calculateSynchronizedValues();
  }

  Vector3d delta_p_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_p_d);

  std::array<double, 16> tcp_positions_extended = robot_state.O_T_EE; // Initialize with appropriate values
  
  std::array<double, 3> tcp_positions;
  Eigen::VectorXd::Map(&tcp_positions[0], 3) = (p_start_ + delta_p_d);
 
  tcp_positions_extended[12] = tcp_positions[0];
  tcp_positions_extended[13] = tcp_positions[1];
  tcp_positions_extended[14] = tcp_positions[2];
  franka::CartesianPose output(tcp_positions_extended);
  output.motion_finished = motion_finished;
  return output;
}
