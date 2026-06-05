// MPC solvers
#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_planning_abstractions/ee_servo.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"
#include "ur_msgs/srv/set_io.hpp"

#include "ral_2026/human_to_robot_handover.hpp"

using namespace std::chrono_literals;

Eigen::Vector3d Handover::solve_mpc_velocity_setpoint(
    const Eigen::Vector3d& current_position,
    const Eigen::Vector3d& current_velocity,
    const Eigen::Vector3d& target_position,
    const Eigen::Vector3d& target_velocity,
    double dt
){
    const int N = 15;                 // MPC horizon
    const int max_iterations = 40;    // optimizer iterations
    const double max_velocity = 0.30;       // m/s
    const double max_acceleration = 0.60;   // m/s^2
    const double w_terminal_position = 80.0;
    const double w_terminal_velocity = 20.0;
    const double w_velocity_effort = 0.5;
    const double w_smoothness = 5.0;

    const double alpha = 0.015;       // gradient descent step size

    Eigen::Matrix<double, 3, Eigen::Dynamic> U(3, N);
    Eigen::Vector3d nominal_velocity=(target_position - current_position) / (N * dt);

    nominal_velocity = clamp_norm(nominal_velocity, max_velocity);

    for (int k = 0; k < N; k++){
        double blend = static_cast<double>(k) / static_cast<double>(N - 1);
        U.col(k) = (1.0 - blend) * nominal_velocity + blend * target_velocity;
        U.col(k) = clamp_norm(U.col(k), max_velocity);
    }

    U = project_velocity_constraints(U, current_velocity, dt);
    for (int iter = 0; iter < max_iterations; iter++){
        Eigen::Vector3d predicted_position = current_position;
        for (int k = 0; k < N; k++){
            predicted_position += U.col(k) * dt;
        }
        Eigen::Vector3d position_error =predicted_position - target_position;
        Eigen::Vector3d terminal_velocity_error =U.col(N - 1) - target_velocity;
        Eigen::Matrix<double, 3, Eigen::Dynamic> grad(3, N);
        grad.setZero();
        for (int k = 0; k < N; k++){
            grad.col(k) +=2.0 * w_terminal_position * dt * position_error;
            grad.col(k) +=2.0 * w_velocity_effort * U.col(k);
        }
        grad.col(N - 1) +=2.0 * w_terminal_velocity * terminal_velocity_error;

        Eigen::Vector3d first_diff = U.col(0) - current_velocity;
        grad.col(0) += 2.0 * w_smoothness * first_diff;

        for (int k = 1; k < N; k++){
            Eigen::Vector3d diff = U.col(k) - U.col(k - 1);
            grad.col(k) += 2.0 * w_smoothness * diff;
            grad.col(k - 1) -= 2.0 * w_smoothness * diff;
        }

        U = U - alpha * grad;

        for (int k = 0; k < N; k++){
            U.col(k) = clamp_norm(U.col(k), max_velocity);
        }

        U = project_velocity_constraints(U, current_velocity, dt);
    }
    return U.col(0);
}

Eigen::Vector3d Handover::clamp_norm(
    const Eigen::Vector3d& v,
    double max_norm
){
    double n = v.norm();
    if (n > max_norm && n > 1e-9){
        return v * (max_norm / n);
    }
    return v;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Handover::project_velocity_constraints(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& U_in,
    const Eigen::Vector3d& previous_velocity,
    double dt
){
    Eigen::Matrix<double, 3, Eigen::Dynamic> U = U_in;
    const double max_velocity = 0.30;
    const double max_acceleration = 0.60;
    Eigen::Vector3d prev = previous_velocity;
    for (int k = 0; k < U.cols(); k++){
        Eigen::Vector3d dv = U.col(k) - prev;
        Eigen::Vector3d dv_limited =
            clamp_norm(dv, max_acceleration * dt);
        U.col(k) = prev + dv_limited;
        U.col(k) = clamp_norm(U.col(k), max_velocity);
        prev = U.col(k);
    }
    return U;
}