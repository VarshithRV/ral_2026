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

// handover function
bool Handover::handover(){
    auto LOGGER = node_->get_logger();
    
    std::cout<<"preparing servo"<<std::endl;
    ee_servo_handle_->prepare_servo_();
    std::cout<<"starting servo"<<std::endl;
    ee_servo_handle_->start_servo_();
    std::cout<<"ready to move"<<std::endl;
    
    double approach_duration = approach_phase_duration;

    // modify this part for MPC
    MPC_go_to_state(approach_duration);
    
    std::cout<<"Stopping servo now"<<std::endl;
    ee_servo_handle_->stop_servo_();
    std::cout<<"Unpreparing now"<<std::endl;
    ee_servo_handle_->unprepare_servo_();
    std::cout<<"Done"<<std::endl;
}

void Handover::MPC_go_to_state(double approach_duration){
    // Modify this function to control the robot
    int i = 0;
    auto rate = rclcpp::Rate(50ms);
    while(rclcpp::ok()){
        auto current_ee_pose = dual_arm_control_interface_->get_current_ee_pose("left");
        
        Eigen::Vector3d current_state_position(current_ee_pose->position.x,current_ee_pose->position.y,current_ee_pose->position.z);
        Eigen::Vector3d current_state_velocity = ee_linear_velocity;
        Eigen::Vector3d vp_position(via_point_position);
        Eigen::Vector3d vp_velocity(via_point_linear_velocity);
        // Eigen::Vector3d vp_velocity(Eigen::Vector3d::Zero());
        
        ////////// change logic from here
        if((approach_duration-(i*0.05))<0.05){
            break;
        }

        // now get the parameters of the cubic polynomials
        auto instantaneous_traj_coeffs = get_coeffs(
            current_state_position,
            vp_position,
            current_state_velocity,
            vp_velocity,
            f(approach_duration,i,0.05)
        );
        
        // now get velocity 
        auto velocity_setpoint = get_linear_vel_from_coeffs(instantaneous_traj_coeffs,g(0.05));
        ////////// to here

    
        set_velocity(k(velocity_setpoint),Eigen::AngleAxisd());
        rate.sleep();
        i++;
    }
}

double Handover::f(double duration,int n, double delta_t){// duration over which the polynomial plans
    double remaining_duration = (duration -n*delta_t);
    return remaining_duration;
} 

double Handover::g(double delta_t){ // function over vel inference instant, if this increases, the controller will look ahead
    return delta_t;
}

Eigen::Vector3d Handover::k(Eigen::Vector3d vel_setpoint){ // function over velocity setpoint, output gain
    return 2*vel_setpoint;
}