// phases of handover control 
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

void Handover::MPC_go_to_approach_state(double approach_duration){
    const double dt = 0.05;   // 20 Hz control loop
    auto rate = rclcpp::Rate(20.0);

    int i = 0;

    while (rclcpp::ok()){
        double elapsed_time = i * dt;
        double remaining_time = approach_duration - elapsed_time;

        auto current_ee_pose=dual_arm_control_interface_->get_current_ee_pose("right");

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
        Eigen::Vector3d target_position = via_point_position;
        Eigen::Vector3d target_velocity = via_point_linear_velocity;
        Eigen::Quaterniond target_orientation = via_point_orientation;


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
        Eigen::Vector3d angular_velocity = get_angular_vel(target_orientation,current_orientation);
        set_velocity(velocity_setpoint, Eigen::AngleAxisd(angular_velocity.norm(),angular_velocity/angular_velocity.norm()));
        double distance_to_target = (target_position - current_position).norm();
        if (distance_to_target < 0.015){
            std::cout << "Reached target region" << std::endl;
            break;
        }
        rate.sleep();
        i++;
    }
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

        auto current_ee_pose=dual_arm_control_interface_->get_current_ee_pose("right");

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
        set_velocity(velocity_setpoint, Eigen::AngleAxisd(angular_velocity.norm(),angular_velocity/angular_velocity.norm()));
        double distance_to_target = (target_position - current_position).norm();
        if (distance_to_target < 0.015){
            std::cout << "Reached target region" << std::endl;
            break;
        }
        rate.sleep();
        i++;
    }
}