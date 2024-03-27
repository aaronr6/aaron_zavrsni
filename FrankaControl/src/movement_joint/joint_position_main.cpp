#include "shared/joint_position.h"

int main() {
    std::string robot_ip = "192.168.40.45";
    
    joint_position::Joint_position Joint_position(robot_ip);

    return 0;
}