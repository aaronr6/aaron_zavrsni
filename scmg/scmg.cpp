#include "scmg.h"

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
}

