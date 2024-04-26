#include <iostream>
#include "shared/first_motion.h"

namespace first_motion
{
    First_motion::First_motion(const std::string& robot_ip)
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
        MotionGenerator motion_generator(0.2, q_goal);
        robot->control(motion_generator);
    } 
}

namespace controller {

    class Controller {
    public:
    Controller(size_t dq_filter_size,
                const std::array<double, 7>& K_P,  // NOLINT(readability-identifier-naming)
                const std::array<double, 7>& K_D)  // NOLINT(readability-identifier-naming)
        : dq_current_filter_position_(0), dq_filter_size_(dq_filter_size), K_P_(K_P), K_D_(K_D) {
        std::fill(dq_d_.begin(), dq_d_.end(), 0);
        dq_buffer_ = std::make_unique<double[]>(dq_filter_size_ * 7);
        std::fill(&dq_buffer_.get()[0], &dq_buffer_.get()[dq_filter_size_ * 7], 0);
    }

    inline franka::Torques st ep(const franka::RobotState& state) {
        updateDQFilter(state);

        std::array<double, 7> tau_J_d;  // NOLINT(readability-identifier-naming)
        for (size_t i = 0; i < 7; i++) {
        tau_J_d[i] = K_P_[i] * (state.q_d[i] - state.q[i]) + K_D_[i] * (dq_d_[i] - getDQFiltered(i));
        }
        return tau_J_d;
    }

    void updateDQFilter(const franka::RobotState& state) {
        for (size_t i = 0; i < 7; i++) {a motion with smooth velocity and acceleration.
    // Squared sine is used for the acceleration/deceleration phase.sition_ * 7 + i] = state.dq[i];
        }
        dq_current_filter_position_ = (dq_current_filter_position_ + 1) % dq_filter_size_;
    }

    double getDQFiltered(size_t index) const {
        double value = 0;
        for (size_t i = index; i < 7 * dq_filter_size_; i += 7) {
        value += dq_buffer_.get()[i];
        }
        return value / dq_filter_size_;
    }

    private:
    size_t dq_current_filter_position_;
    size_t dq_filter_size_;

    const std::array<double, 7> K_P_;  // NOLINT(readability-identifier-naming)
    const std::array<double, 7> K_D_;  // NOLINT(readability-identifier-naming)

    std::array<double, 7> dq_d_;
    std::unique_ptr<double[]> dq_buffer_;
    };

    std::vector<double> generateTrajectory(double a_max) {
    // Generating a motion with smooth velocity and acceleration.
    // Squared sine is used for the acceleration/deceleration phase.
    std::vector<double> trajectory;
    constexpr double kTimeStep = 0.001;          // [s]
    constexpr double kAccelerationTime = 1;      // time spend accelerating and decelerating [s]
    constexpr double kConstantVelocityTime = 1;  // time spend with constant speed [s]
    // obtained during the speed up
    // and slow down [rad/s^2]
    double a = 0;  // [rad/s^2]
    double v = 0;  // [rad/s]
    double t = 0;  // [s]
    while (t < (2 * kAccelerationTime + kConstantVelocityTime)) {
        if (t <= kAccelerationTime) {
        a = pow(sin(t * M_PI / kAccelerationTime), 2) * a_max;
        } else if (t <= (kAccelerationTime + kConstantVelocityTime)) {
        a = 0;
        } else {
        const double deceleration_time =
            (kAccelerationTime + kConstantVelocityTime) - t;  // time spent in the deceleration phase
        a = -pow(sin(deceleration_time * M_PI / kAccelerationTime), 2) * a_max;
        }
        v += a * kTimeStep;
        t += kTimeStep;
        trajectory.push_back(v);
    }
    return trajectory;
    }

}