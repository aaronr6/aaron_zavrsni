#include <cmath>
#include <array>
#include <iostream>

#include <Eigen/Core>

#include <fstream>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/duration.h>
#include <franka/robot.h>

#include "tcp_mg.h" // treba zamijeniti s example_common.h


void writeTCPPosition(double time, const Eigen::Vector3d& position, std::ofstream& file) {
  file << time << " " << position[0] << " " << position[1] << " " << position[2] << "\n";
}


int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  std::ofstream outputFile("tcp_positions.txt");

  // Connect to the robot
  franka::Robot robot("192.168.40.45");
  franka::Model model = robot.loadModel();

  // parameters for the conrollers

  constexpr double k_p{100.0};  // NOLINT (readability-identifier-naming)
  constexpr double k_i{0.0};  // NOLINT (readability-identifier-naming)
  constexpr double k_d{0.0};  // NOLINT (readability-identifier-naming)

  try {
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);


    // First move the robot to a suitable joint configuration
    std::array<double, 6> q_goal = {{0.5, 0.3, 0.5, -3 * M_PI_4, 0, M_PI_2}};  
    MotionGenerator motion_generator(0.1, q_goal);  /* !!!!!! ovo je moj motion_generator u koji mogu direktno unjeti pozu tcp, 
                                                      ali moguce ga je jednostavno zamijeniti s examples_common.h motion_generatorom */
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // set collision behavior

    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
    robot.setJointImpedance({{300, 300, 300, 250, 250, 200, 200}});
    robot.setCartesianImpedance({{300, 300, 300, 30, 30, 30}});


    franka::RobotState initial_state = robot.readOnce();

    Eigen::VectorXd initial_pose_ext(3), pose_error_integral(3), pose_error_derivative(3), pose_d(3), last_pose(3), next_pose(3);
    Eigen::Vector3d initial_position;
    double time = 0.0;
    auto get_position = [](const franka::RobotState& robot_state) {
      return Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                             robot_state.O_T_EE[14]);
    };

    pose_d << 0.5, 0.0, 0.5;
    pose_error_integral.setZero();


    std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)>
      motion_control_callback = [&](const franka::RobotState& robot_state,
                                      franka::Duration period) -> franka::JointVelocities {
      time += period.toSec();

      if (time == 0.0) {
          last_pose[0] = 0.0;
          last_pose[1] = 0.0;
          last_pose[2] = 0.0;
      }
      
      next_pose = get_position(robot_state);
      
      last_pose = next_pose;

      Eigen::Map<const Eigen::VectorXd> q_current(robot_state.q_d.data(), robot_state.q_d.size());

      std::array<double, 42> jacobian_array =
            model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());

      Eigen::VectorXd pose_ext(3), desired_joint_motion(7), control_output(3), pose_ext_prior(3);
      desired_joint_motion.setZero();
      pose_ext << pose_d - get_position(robot_state);

      // std::cout << "pose_ext: " << pose_ext << '\n' << pose_d << '\n' << "get_position(robot_state)" << get_position(robot_state) << std::endl;

      pose_error_integral +=  period.toSec() * (pose_d - get_position(robot_state)); // * period.toSec(); s ovim sam mnozio s lijeve strane ali ne kuzim bas zasto

      std::cout << "pose_error_integral: " << pose_error_integral << std::endl;

      pose_error_derivative = (pose_ext - pose_ext_prior) / period.toSec();
      
      if(time == 0.0){
        pose_error_derivative[0] = 0.0;
        pose_error_derivative[1] = 0.0;
        pose_error_derivative[2] = 0.0;
      }

      pose_ext_prior << pose_ext;

      Eigen::VectorXd result_coordinates(6);

      control_output << k_p * pose_ext + k_i * pose_error_integral + k_d * pose_error_derivative; //(next_pose - last_pose)/0.001 + ne znam zasto al to je stajalo jos tu pokraj

      std::cout << "control_output: " << control_output << std::endl;
      std::cout << "pose_ext: " << pose_ext << '\n' << "pose_error_integral: " << pose_error_integral << '\n' << "pose_error_derivative: " << pose_error_derivative << '\n' << std::endl;

      result_coordinates.head(3) = control_output.head(3);
      result_coordinates[3] = 0; //-3 * M_PI_4
      result_coordinates[4] = 0;
      result_coordinates[5] = 0; //M_PI_2

      std::cout << "result_coordinates: " << result_coordinates << '\n' << std::endl;

      desired_joint_motion = (jacobian.transpose() * result_coordinates)*0.1; 
      std::array<double, 7> desired_joint_motion_array;
      Eigen::VectorXd::Map(&desired_joint_motion_array[0], desired_joint_motion.size()) =
          desired_joint_motion;

      // Write TCP position to file
      writeTCPPosition(time, next_pose, outputFile);    

      return desired_joint_motion_array;
    };
  std::cout << "WARNING: Make sure sure that no endeffector is mounted and that the robot's last "
                  "joint is "
                  "in contact with a horizontal rigid surface before starting. Keep in mind that "
                  "collision thresholds are set to high values."
                << std::endl
                << "Press Enter to continue..." << std::endl;
  std::cin.ignore();
  // start real-time control loop
  robot.control(motion_control_callback);

  } catch (const std::exception& ex) {
    // print exception
    std::cout << ex.what() << std::endl;
  }

  outputFile.close();

  system("python3 ../tcp_positions.py");


  return 0;
}