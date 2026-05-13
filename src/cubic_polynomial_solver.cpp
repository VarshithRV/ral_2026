// get coeff matrix function
// get velocity function

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

Eigen::Matrix<double,4,3> Handover::get_coeffs(Eigen::Vector3d initial_position, 
            Eigen::Vector3d final_position, 
            Eigen::Vector3d initial_velocity, 
            Eigen::Vector3d final_velocity, 
            double duration
){
    Eigen::Matrix<double,4,3> B;
    
    B.row(0) = initial_position.transpose();
    B.row(1) = initial_velocity.transpose();
    B.row(2) = final_position.transpose();
    B.row(3) = final_velocity.transpose();

    Eigen::Matrix<double,4,4> A;
    A.col(0) = Eigen::Vector4d(1,0,1,0);
    A.col(1) = Eigen::Vector4d(0,1,duration,1);
    A.col(2) = Eigen::Vector4d(0,0,duration*duration,2*duration);
    A.col(3) = Eigen::Vector4d(0,0,duration*duration*duration,3*duration*duration);

    return A.inverse()*B;
}


Eigen::Vector3d Handover::get_vel_from_coeffs(Eigen::Matrix<double,4,3>coeff_matrix, double tau){
    Eigen::Vector4d vel_fn(0,1,2*tau,3*tau*tau);
    return coeff_matrix.transpose()*vel_fn;
}