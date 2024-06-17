#include "shared/joint_states.h"

int main() {
    std::string robot_ip = "192.168.40.45";
    
    franka::Model model = robot.loadModel();
    robot_comm::Joint_states Joint_states(robot_ip);


    return 0;
}