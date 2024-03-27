#include <iostream>
#include "shared/joint_states.h"


namespace robot_comm
{
   Joint_states::Joint_states(const std::string& robot_ip) 
   {
      robot = std::make_unique<franka::Robot>(robot_ip);
      robot_state = std::make_shared<franka::RobotState>(robot->readOnce());

      initial_pose = robot_state->O_T_EE_c;
      Eigen::Matrix4d initial_transform=(Eigen::Matrix4d::Map(robot_state->O_T_EE.data()));
      joint_state = robot_state->q;


      std::cout<<"-------------------------------------------------------------------------------"<<std::endl;
      std::cout<<"Current joint positions [rad]:"<<std::endl<<joint_state[0]<<", "<<joint_state[1]<<", "<<joint_state[2]<<", "<<joint_state[3]<<", "
              <<joint_state[4]<<", "<<joint_state[5]<<", "<<joint_state[6]<<std::endl;
      std::cout<<"Current joint positions [°]:"<<std::endl<<
         round((joint_state[0]*180/M_PI)*100)/100<<", "<<
         round((joint_state[1]*180/M_PI)*100)/100<<", "<<
         round((joint_state[2]*180/M_PI)*100)/100<<", "<<
         round((joint_state[3]*180/M_PI)*100)/100<<", "<<
         round((joint_state[4]*180/M_PI)*100)/100<<", "<<
         round((joint_state[5]*180/M_PI)*100)/100<<", "<<
         round((joint_state[6]*180/M_PI)*100)/100<<std::endl;
      std::cout<<"Current TCP transformation matrix:"<<std::endl<<initial_transform.matrix()<<std::endl;
      std::cout<<"-------------------------------------------------------------------------------"<<std::endl;
   }
} //namespace robot_comm