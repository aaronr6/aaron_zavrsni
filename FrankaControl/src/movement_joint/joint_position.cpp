#include <iostream>
#include "shared/joint_position.h"


namespace joint_position
{
   Joint_position::Joint_position(const std::string& robot_ip) 
   {
        robot = std::make_unique<franka::Robot>(robot_ip);
        robot_state = std::make_shared<franka::RobotState>(robot->readOnce());

        robot->setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        const std::array<double, 7> q_goal = {{ 0, M_PI_4, 0, - M_PI_2, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        robot->control(motion_generator);
   }
} //namespace joint_position
