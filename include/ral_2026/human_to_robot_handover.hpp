// header for the handover class
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

using namespace std::chrono_literals;

class Handover
{
    public:
        // constructor
        Handover(rclcpp::Node::SharedPtr node);

        // handover function
        bool handover();
    
    private:
        // node
        rclcpp::Node::SharedPtr node_;
        
        // helper objects
        std::shared_ptr<EEServo> ee_servo_handle_;
        std::shared_ptr<DualArmControlInterface> dual_arm_control_interface_;

        // cb groups
        rclcpp::CallbackGroup::SharedPtr parallel_cb_group_;
        rclcpp::CallbackGroup::SharedPtr mex_cb_group_;
        
        // services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handover_server_;

        // subscribers
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object0_filtered_pose_subscriber_;

        // publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr via_point_pose_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_publisher_;
        
        // timers
        rclcpp::TimerBase::SharedPtr state_pose_publisher_timer_;

        // clock
        rclcpp::Clock clock_;

        // // DATA

        // velocity filter coeff        
        double linear_velocity_filter_alpha = 0.5;
        
        // OBJECT STATE ESTIMATION DATA
        // grasp state, pose and velocity
        bool object_pose_received=false;
        Eigen::Vector3d grasp_position;
        std::vector<Eigen::Vector3d> grasp_position_window;
        std::vector<double> grasp_position_timestamp_window;
        Eigen::Quaterniond grasp_orientation = Eigen::Quaterniond(1,0,0,0);
        Eigen::Vector3d grasp_linear_velocity;
        Eigen::Vector3d grasp_filtered_linear_velocity;
        Eigen::AngleAxisd grasp_angular_velocity;
        // via point state, pose and velocity
        Eigen::Vector3d via_point_position;
        std::vector<Eigen::Vector3d> via_point_position_window;
        std::vector<double> via_point_position_timestamp_window;
        Eigen::Quaterniond via_point_orientation= Eigen::Quaterniond(1,0,0,0);
        Eigen::Vector3d via_point_linear_velocity;
        Eigen::Vector3d via_point_filtered_linear_velocity;
        Eigen::AngleAxisd via_point_angular_velocity;
        // state window size
        int state_window_size = 5;
        // object to grasp transform
        Eigen::Vector3d object_to_grasp_position_transform = Eigen::Vector3d(0.03,0.0,-0.03);
        Eigen::Quaterniond object_to_grasp_orientation_transform = Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d::UnitY()));
        Eigen::Vector3d grasp_to_via_point_position_transform = Eigen::Vector3d(0.0,0.0,-0.1);
        // via point speed 
        double via_point_speed = 0.1; //ms-1


        // helper function to set velocity
        void set_velocity(Eigen::Vector<double,3> linear_velocity, Eigen::AngleAxisd angular_velocity);
        void process_object_pose(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        Eigen::Vector3d filter_linear_velocity(Eigen::Vector<double,3> filtered_linear_velocity, Eigen::Vector<double,3> linear_velocity);
        void state_pose_publisher_cb_();
};
