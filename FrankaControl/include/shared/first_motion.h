#ifndef __FIRST_MOTION__
#define __FIRST_MOTION__

#include <eigen3/Eigen/Core>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include "shared/examples_common.h"
#include <array>
#include <cmath>

namespace first_motion
{

class First_motion
{    
    public:
        First_motion(const std::string& robot_ip);
        
    private:
        std::unique_ptr<franka::Robot> robot;
        std::shared_ptr<franka::RobotState> robot_state;
        
        static const std::array<double, 7> q_goal;
};

} //namespace first_motion
#endif 