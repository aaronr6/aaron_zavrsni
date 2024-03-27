#ifndef __JOINT_POSITION__
#define __JOINT_POSITION__

#include <eigen3/Eigen/Core>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include "shared/examples_common.h"
#include <array>
#include <cmath>

namespace joint_position
{

class Joint_position
{    
    public:
        Joint_position(const std::string& robot_ip);
        
    private:
        std::unique_ptr<franka::Robot> robot;
        std::shared_ptr<franka::RobotState> robot_state;
        
        static const std::array<double, 7> q_goal;
};

} //namespace joint_position
#endif 