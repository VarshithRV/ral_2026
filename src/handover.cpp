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
    
    auto rate = rclcpp::Rate(50ms);
    
    int i = 0;
    while(rclcpp::ok()){
        i++;
        set_velocity(Eigen::Vector<double,3>(0,0,-0.05),Eigen::AngleAxisd());
        auto current_ee_pose = dual_arm_control_interface_->get_current_ee_pose("left");
        rate.sleep();
        if(i>200){
            break;
        }
    }
    
    std::cout<<"Stopping servo now"<<std::endl;
    ee_servo_handle_->stop_servo_();
    std::cout<<"Unpreparing now"<<std::endl;
    ee_servo_handle_->unprepare_servo_();
    std::cout<<"Done"<<std::endl;
}