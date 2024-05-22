// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

#include <Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

/**
 * @file examples_common.h
 * Contains common types and functions for the examples.
 */

/**
 * Sets a default collision behavior, joint impedance and Cartesian impedance.
 *
 * @param[in] robot Robot instance to | 1, 2, 3 |
| 4, 5, 6 |
| 7, 8, 9 |set behavior on.
 */
void setDefaultBehavior(franka::Robot& robot);

/**
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class MotionGenerator {
 public:
  /**
   * Creates a new MotionGenerator instance for a target q.
   *
   * @param[in] speed_factor General speed factor in range [0, 1].
   * @param[in] p_goal Target joint positions.
   */
  MotionGenerator(double speed_factor, const std::array<double, 3> p_goal);

  /**
   * Sends joint position calculations
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] period Duration of execution.
   *
   * @return Joint positions for use inside a control loop.
   */
  franka::CartesianPose operator()(const franka::RobotState& robot_state, franka::Duration period);

 private:
  using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
  using Vector3i = Eigen::Matrix<int, 3, 1, Eigen::ColMajor>;

  bool calculateDesiredValues(double t, Vector3d* delta_p_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  const Vector3d p_goal_;

  Vector3d p_start_;
  Vector3d delta_p_;

  Vector3d dp_max_sync_;
  Vector3d t_1_sync_;
  Vector3d t_2_sync_;
  Vector3d t_f_sync_;
  Vector3d p_1_;

  double time_ = 0.0;

  Vector3d dp_max_ = (Vector3d() << 2.0, 2.0, 2.0).finished();
  Vector3d ddp_max_start_ = (Vector3d() << 5, 5, 5).finished();
  Vector3d ddp_max_goal_ = (Vector3d() << 5, 5, 5).finished();
};
