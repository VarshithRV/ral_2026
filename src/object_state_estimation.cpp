// estimate via pose, via velocity, grasp pose, grasp velocity from the object input

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

// - [x] Get object position and orientation
// - [x] Get grasp position and orientation
// - [x] Get via point position and orientation
// - [ ] Update grasp and via point windows
// - [ ] Get grasp velocity
// - [ ] Get via point velocity
// - [ ] Filter grasp velocity and via point velocity
// - [x] Log everything
// - [x] Test
// - [x] Write a timer to publish the via pose and grasp pose

void Handover::process_object_pose(geometry_msgs::msg::PoseStamped::SharedPtr msg){
    auto logger = node_->get_logger();
    
    object_pose_received = true;
    
    Eigen::Vector3d object_position(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
    Eigen::Quaterniond object_orientation(msg->pose.orientation.w,msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);

    // grasp pose
    grasp_orientation = object_orientation*object_to_grasp_orientation_transform;
    grasp_position = object_position + object_orientation*object_to_grasp_position_transform;

    // via point pose
    via_point_orientation = grasp_orientation;
    via_point_position = grasp_orientation*grasp_to_via_point_position_transform + grasp_position;

    // get the via point velocity
    via_point_linear_velocity = via_point_speed*via_point_orientation.toRotationMatrix().col(2);
}

void Handover::state_pose_publisher_cb_(){
    geometry_msgs::msg::PoseStamped via_point_pose;
    geometry_msgs::msg::PoseStamped grasp_pose;
    
    via_point_pose.header.frame_id = "world";
    grasp_pose.header.frame_id = "world";
    
    via_point_pose.header.stamp = clock_.now();
    grasp_pose.header.stamp = clock_.now();

    via_point_pose.pose.position.x = via_point_position.x();
    via_point_pose.pose.position.y = via_point_position.y();
    via_point_pose.pose.position.z = via_point_position.z();
    via_point_pose.pose.orientation.x = via_point_orientation.x();
    via_point_pose.pose.orientation.y = via_point_orientation.y();
    via_point_pose.pose.orientation.z = via_point_orientation.z();
    via_point_pose.pose.orientation.w = via_point_orientation.w();

    grasp_pose.pose.position.x = grasp_position.x();
    grasp_pose.pose.position.y = grasp_position.y();
    grasp_pose.pose.position.z = grasp_position.z();
    grasp_pose.pose.orientation.x = grasp_orientation.x();
    grasp_pose.pose.orientation.y = grasp_orientation.y();
    grasp_pose.pose.orientation.z = grasp_orientation.z();
    grasp_pose.pose.orientation.w = grasp_orientation.w();

    via_point_pose_publisher_->publish(via_point_pose);
    grasp_pose_publisher_->publish(grasp_pose);
}