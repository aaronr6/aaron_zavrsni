#include "tcp_mg.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  robot.setCollisionBehavior(
          {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}}, {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
          {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}}, {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}},
          {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}}, {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}},
          {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}}, {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 3> c_goal)
    : c_goal_(c_goal.data()) {
  dc_max_ *= speed_factor;
  ddc_max_start_ *= speed_factor;
  ddc_max_goal_ *= speed_factor; 
  dc_max_sync_.setZero();
  c_start_.setZero();
  delta_c_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  c_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector3d* delta_c_d) const {
  Vector3i sign_delta_c;
  sign_delta_c << delta_c_.cwiseSign().cast<int>();
  Vector3d t_d = t_2_sync_ - t_1_sync_;
  Vector3d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 3> cartesian_motion_finished{};

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_c_[i]) < kDeltaCMotionFinished) {
      (*delta_c_d)[i] = 0;
      cartesian_motion_finished[i] = true;
    } else {
      if (t < t_1_sync_[i]) {
        (*delta_c_d)[i] = (-1.0 / std::pow(t_1_sync_[i], 3.0) * dc_max_sync_[i] * sign_delta_c[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0));
        // std::cout << "delta_c_d: " << (*delta_c_d)[i] << std::endl;
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_c_d)[i] = (c_1_[i] + (t - t_1_sync_[i]) * dc_max_sync_[i] * sign_delta_c[i]);
        // std::cout << "delta_c_d: " << (*delta_c_d)[i] << std::endl;
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_c_d)[i] =
            (delta_c_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dc_max_sync_[i] * sign_delta_c[i]);
        // std::cout << "delta_c_d: " << (*delta_c_d)[i] << std::endl;
      } else {
        (*delta_c_d)[i] = delta_c_[i];
        cartesian_motion_finished[i] = true;
      }
    }
  }
  return std::all_of(cartesian_motion_finished.cbegin(), cartesian_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector3d dc_max_reach(dc_max_);
  Vector3d t_f = Vector3d::Zero();
  Vector3d delta_t_2 = Vector3d::Zero();
  Vector3d t_1 = Vector3d::Zero();
  Vector3d delta_t_2_sync = Vector3d::Zero();
  Vector3i sign_delta_c;
  sign_delta_c << delta_c_.cwiseSign().cast<int>();

  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_c_[i]) > kDeltaCMotionFinished) {
      if (std::abs(delta_c_[i]) < (3.0 / 4.0 * (std::pow(dc_max_[i], 2.0) / ddc_max_start_[i]) +
                                  3.0 / 4.0 * (std::pow(dc_max_[i], 2.0) / ddc_max_goal_[i]))) {
        dc_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_c_[i] * sign_delta_c[i] *
                                    (ddc_max_start_[i] * ddc_max_goal_[i]) /
                                    (ddc_max_start_[i] + ddc_max_goal_[i]));
      }
      t_1[i] = 1.5 * dc_max_reach[i] / ddc_max_start_[i];
      delta_t_2[i] = 1.5 * dc_max_reach[i] / ddc_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_c_[i]) / dc_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 3; i++) {
    if (std::abs(delta_c_[i]) > kDeltaCMotionFinished) {
      double a = 1.5 / 2.0 * (ddc_max_goal_[i] + ddc_max_start_[i]);
      double b = -1.0 * max_t_f * ddc_max_goal_[i] * ddc_max_start_[i];
      double c = std::abs(delta_c_[i]) * ddc_max_goal_[i] * ddc_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      dc_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      t_1_sync_[i] = 1.5 * dc_max_sync_[i] / ddc_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dc_max_sync_[i] / ddc_max_goal_[i];
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_c_[i] / dc_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      c_1_[i] = (dc_max_sync_)[i] * sign_delta_c[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

franka::CartesianPose MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    c_start_aux_ = Vector16d(robot_state.O_T_EE_d.data());
    // std::cout << "c_start_aux_: " << c_start_aux_ << std::endl;
    c_start_ = c_start_aux_.segment<3>(12);
    // std::cout << "c_start_: " << c_start_ << std::endl;
    delta_c_ = c_goal_ - c_start_;
    // std::cout << "delta_c_: " << delta_c_ << std::endl;
    calculateSynchronizedValues();
  }

  Vector3d delta_c_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_c_d);

  std::array<double, 16> cartesian_positions = {
    1.0, 0.0, 0.0, 0.0,  // First column
    0.0, 1.0, 0.0, 0.0,  // Second column
    0.0, 0.0, 1.0, 0.0,  // Third column
    0.0, 0.0, 0.0, 1.0   // Fourth column
  };
  std::copy_n(c_start_aux_.data(), 12, cartesian_positions.begin());
  std::array<double, 3> cartesian_positions_aux;
  Eigen::VectorXd::Map(&cartesian_positions_aux[0], 3) = (c_start_ + delta_c_d);
  std::copy(cartesian_positions_aux.begin(), cartesian_positions_aux.end(), cartesian_positions.begin() + 12);
  franka::CartesianPose output(cartesian_positions);
  output.motion_finished = motion_finished;
  return output;
}