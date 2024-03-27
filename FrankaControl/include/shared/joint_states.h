#ifndef __ROBOT_COMM_
#define __ROBOT_COMM_

#include <eigen3/Eigen/Core>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

namespace robot_comm
{

class Joint_states
{    
    public:
        Joint_states(const std::string& robot_ip);
        
    private:
        std::unique_ptr<franka::Robot> robot;
        std::shared_ptr<franka::RobotState> robot_state;
        std::array<double, 16> initial_pose;
        Eigen::Matrix4d initial_transform;
        std::array<double, 7> joint_state;    
};

} //namespace robot_comm
#endif 