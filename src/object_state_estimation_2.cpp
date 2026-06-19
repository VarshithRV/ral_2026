// estimate via pose, via velocity, grasp pose, grasp velocity from the object input
#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_planning_abstractions/ee_servo.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "ral_2026/human_to_robot_handover_2.hpp"

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
    
    // camera optical frame -> world frame
    geometry_msgs::msg::PoseStamped camera_pose, world_pose;

    camera_pose.header = msg->header;
    camera_pose.pose = msg->pose;
    camera_pose.header.stamp = rclcpp::Time(0);
    Eigen::Vector3d object_position;
    Eigen::Quaterniond object_orientation;

    try{
        world_pose = tf_buffer_->transform(camera_pose, "world");

        object_position = {world_pose.pose.position.x,world_pose.pose.position.y,world_pose.pose.position.z};

        object_orientation = {world_pose.pose.orientation.w,world_pose.pose.orientation.x,world_pose.pose.orientation.y,world_pose.pose.orientation.z};

    }
    catch (tf2::TransformException &ex){
        RCLCPP_WARN(node_->get_logger(), "TF transform failed: %s", ex.what());
        return;
    }

    // grasp pose
    grasp_orientation = object_orientation*object_to_grasp_orientation_transform;
    grasp_position = object_position + object_orientation*object_to_grasp_position_transform;

    // via point pose
    via_point_orientation = grasp_orientation;
    via_point_position = grasp_orientation*grasp_to_via_point_position_transform + grasp_position;

    // get the via point velocity
    via_point_linear_velocity = via_point_speed*via_point_orientation.toRotationMatrix().col(2);
}

// EE Velocity 
void Handover::get_ee_linear_vel(){
    auto current_ee_pose = dual_arm_control_interface_->get_current_ee_pose("right");
    Eigen::Vector3d current_ee_position(
        current_ee_pose->position.x,
        current_ee_pose->position.y,
        current_ee_pose->position.z
    );
    
    //  write code to calculate the end effector linear velocity
    if(ee_position_window.size()<5){
        ee_position_window.push_back(current_ee_position);
    }
    else{
        ee_position_window.push_back(current_ee_position);
        ee_position_window.erase(ee_position_window.begin());
    }

    float elapsed_time = (ee_position_window.size() - 1)*0.05;
    Eigen::Vector3d avg_window_velocity;
    if(elapsed_time>0)
        avg_window_velocity=(ee_position_window.back() - ee_position_window.front())/elapsed_time;
    
    ee_linear_velocity = avg_window_velocity;
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
