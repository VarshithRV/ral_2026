// contains the driver code, constructor and other general helper functions
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


Handover::Handover(rclcpp::Node::SharedPtr node){
    if(node == nullptr){
        std::cout<<"node passed is a nullptr!!, exiting"<<std::endl;
        return;
    }
    else
        node_ = node;

    // clock
    clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
    
    // servo and dual arm control inits
    ee_servo_handle_ = std::make_shared<EEServo>(node_);
    dual_arm_control_interface_ = std::make_shared<DualArmControlInterface>(node_);
    
    // callback groups
    parallel_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mex_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    // subscribers
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group=parallel_cb_group_;

    object0_filtered_pose_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/object0_filtered_pose",
        10,
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){
            process_object_pose(msg);
        },
        sub_opts
    );

    // publishers
    via_point_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("~/via_point_pose",10);
    grasp_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("~/grasp_pose",10);

    // handover server initialization
    handover_server_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/handover",
        [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
            res->success= handover();
        },
        rmw_qos_profile_services_default,
        parallel_cb_group_
    );

    // timer
    state_pose_publisher_timer_ = node_->create_wall_timer(50ms,[this](){state_pose_publisher_cb_();},parallel_cb_group_);
    ee_velocity_calculator_timer_ = node_->create_wall_timer(50ms,[this](){get_ee_linear_vel();},parallel_cb_group_);
}

// helper function to set velocity
void Handover::set_velocity(Eigen::Vector<double,3> linear_velocity, Eigen::AngleAxisd angular_velocity){
    auto vel = geometry_msgs::msg::TwistStamped();
    vel.header.frame_id="world";
    vel.header.stamp = node_->get_clock()->now();
    vel.twist.linear.x = linear_velocity[0];
    vel.twist.linear.y = linear_velocity[1];
    vel.twist.linear.z = linear_velocity[2];
    vel.twist.angular.x = angular_velocity.angle()*angular_velocity.axis().x();
    vel.twist.angular.y = angular_velocity.angle()*angular_velocity.axis().y();
    vel.twist.angular.z = angular_velocity.angle()*angular_velocity.axis().z();
    ee_servo_handle_->set_vel_setpoint_(vel);
    return;
}

Eigen::Vector3d Handover::filter_linear_velocity(Eigen::Vector<double,3> filtered_linear_velocity, Eigen::Vector<double,3> linear_velocity){
    Eigen::Vector3d new_filtered_linear_velocity = linear_velocity_filter_alpha*filtered_linear_velocity + (1-linear_velocity_filter_alpha)*linear_velocity;
    return new_filtered_linear_velocity;
}

int main(int argc, const char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("human_to_robot_handover_v3");

    auto handover_handle = std::make_shared<Handover>(node);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    
    rclcpp::shutdown();
}