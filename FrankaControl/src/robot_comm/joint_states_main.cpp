#include "shared/joint_states.h"

int main() {
    std::string robot_ip = "192.168.42.45";
    

    robot_comm::Joint_states Joint_states(robot_ip);


    return 0;
}