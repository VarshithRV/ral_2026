// for the actual handover function
#ifndef HANDOVER_HPP
#define HANDOVER_HPP
#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_planning_abstractions/ee_servo.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"
#endif

#include "ral_2026/human_to_robot_handover.hpp"

using namespace std::chrono_literals;

bool Handover::handover()
{
    auto LOGGER = node_->get_logger();

    std::cout<<"preparing servo"<<std::endl;
    ee_servo_handle_->prepare_servo_();
    std::cout<<"starting servo"<< std::endl;
    ee_servo_handle_->start_servo_();
    std::cout<<"ready to move"<<std::endl;

    Eigen::Vector3d zero_velocity = Eigen::Vector3d::Zero();

    MPC_go_to_state(approach_phase_duration,via_point_position,via_point_linear_velocity,via_point_orientation);
    MPC_go_to_state(1.0,grasp_position,zero_velocity,grasp_orientation);
    MPC_go_to_state(1.0,via_point_position,zero_velocity,grasp_orientation);

    std::cout<<"Stopping servo now"<<std::endl;
    ee_servo_handle_->stop_servo_();
    std::cout<<"Unpreparing now"<<std::endl;
    ee_servo_handle_->unprepare_servo_();
    std::cout<<"Done"<<std::endl;

    return true;
}

void Handover::MPC_go_to_state(double approach_duration, Eigen::Vector3d final_position, Eigen::Vector3d final_velocity, Eigen::Quaterniond final_orientation){
    const double dt = 0.05;   // 20 Hz control loop
    auto rate = rclcpp::Rate(20.0);

    int i = 0;

    while (rclcpp::ok()){
        double elapsed_time = i * dt;
        double remaining_time = approach_duration - elapsed_time;

        // if (remaining_time <= 0.0){
        //     std::cout << "MPC_go_to_state timed out" << std::endl;
        //     set_velocity(Eigen::Vector3d::Zero(), Eigen::AngleAxisd());
        //     break;
        // }

        auto current_ee_pose=dual_arm_control_interface_->get_current_ee_pose("left");

        Eigen::Vector3d current_position(
            current_ee_pose->position.x,
            current_ee_pose->position.y,
            current_ee_pose->position.z
        );
        Eigen::Quaterniond current_orientation(
            current_ee_pose->orientation.w,
            current_ee_pose->orientation.x,
            current_ee_pose->orientation.y,
            current_ee_pose->orientation.z
        );

        Eigen::Vector3d current_velocity = ee_linear_velocity;
        Eigen::Vector3d target_position = final_position;
        Eigen::Vector3d target_velocity = final_velocity;
        Eigen::Quaterniond target_orientation = final_orientation;


        Eigen::Vector3d velocity_setpoint = solve_mpc_velocity_setpoint(
                current_position,
                current_velocity,
                target_position,
                target_velocity,
                dt
        );

        Eigen::Vector3d position_error = target_position - current_position;

        Eigen::Vector3d bias_velocity = 1.2 * position_error;
        bias_velocity = clamp_norm(bias_velocity, 0.15);
            
        Eigen::Vector3d raw_velocity_setpoint = velocity_setpoint + bias_velocity;
        raw_velocity_setpoint = clamp_norm(raw_velocity_setpoint, 0.20);
            
        Eigen::Vector3d dv = raw_velocity_setpoint - previous_velocity_command;
        dv = clamp_norm(dv, 0.60 * dt);
            
        velocity_setpoint = previous_velocity_command + dv;
        velocity_setpoint = previous_velocity_command + dv;
        previous_velocity_command = velocity_setpoint;
        Eigen::Vector3d angular_velocity = get_angular_vel(final_orientation,current_orientation);
        set_velocity(k(velocity_setpoint), Eigen::AngleAxisd(angular_velocity.norm(),angular_velocity/angular_velocity.norm()));
        double distance_to_target = (target_position - current_position).norm();
        if (distance_to_target < 0.015){
            std::cout << "Reached target region" << std::endl;
            break;
        }
        rate.sleep();
        i++;
    }
}

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


double Handover::f(double duration,int n, double delta_t){// duration over which the polynomial plans
    double remaining_duration = (duration -n*delta_t);
    return remaining_duration;
} 

double Handover::g(double delta_t){ // function over vel inference instant, if this increases, the controller will look ahead
    return delta_t;
}

Eigen::Vector3d Handover::k(Eigen::Vector3d vel_setpoint){ // function over velocity setpoint, output gain
    return vel_setpoint;
}