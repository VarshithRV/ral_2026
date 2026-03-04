/*
This is a single node to do human to robot handover for the ral 2026 journal paper, the following node hosts a service which
controls the robotic arm by commanding velocity using the quintic polynomial replanning strategy, it is not a closed loop strategy
its feedforward.
*/

/*
Algorithm : 

The following servers :
1. human robot handover : callback should trigger the routine from outside

The following clients : 
1. left and right preaction servers
2. left pose tracker prepare, start, stop, unprepare

The following subscribers : 
1. apriltag grid detector output pose, callback should transform the pose to a equivalent grasp target pose \
2. lift sensor

The following publishers : 
1. grasp pose, grasp pose PoseStamped message
2. delta_twist_cmd command velocity

workflow : 
-   block until object pose is received
-   block till weight is detected on the sensor
-   start when weight is lifted
-   publish the velocity using the quintic polynomial replanning strategy, do blind reach after the via point
-   grasp
-   back up 10 cm in the opposite z direction and set it down
-   ungrasp
*/

#define THRESHOLD_HEIGHT 0.2
#define SENSOR_READING_THRESHOLD 500 

#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cmath>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Geometry>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class HumanRobotHandover{
    public:
        HumanRobotHandover():threshold_height_(THRESHOLD_HEIGHT){
            
            // node declaration
            node_ = std::make_shared<rclcpp::Node>("human_to_robot_handover");
            
            // lift sensor shit
            baud_flag_ = baudFromInt(serial_baud_);

            if(!openSerialPort()){
                RCLCPP_ERROR(node_->get_logger(), "Serial port open failed (%s). Sensor read will not update.", serial_port_.c_str());
            }
            else{
                RCLCPP_INFO(node_->get_logger(), "Serial opened: %s @ %d", serial_port_.c_str(), serial_baud_);
            }

            // initialize callback group
            callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            
            // tf buffer shit
            tf_buffer_ =std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // publishers
            grasp_pose_publisher_=node_->create_publisher<geometry_msgs::msg::PoseStamped>("~/grasp_pose",10);
            sensor_value_publisher_=node_->create_publisher<std_msgs::msg::Int32>("~/sensor_value",10);
            left_servo_node_main_delta_twist_cmds_publisher_=node_->create_publisher<geometry_msgs::msg::TwistStamped>("/left_servo_node_main/delta_twist_cmds",10);

            // move group interface setup
            rclcpp::NodeOptions node_options;
            node_options.automatically_declare_parameters_from_overrides(true);
            node_options.use_global_arguments(false);
            std::string moveit_node_name = std::string(node_->get_name()) + "_moveit";
            
            moveit_node_ = std::make_shared<rclcpp::Node>(moveit_node_name, node_options);
            move_group_interface_ = std::make_shared<MoveGroupInterface>(moveit_node_, planning_group_);
            move_group_interface_->setEndEffectorLink("left_tool0");
            move_group_interface_->setPlanningTime(10.0);
            move_group_interface_->setNumPlanningAttempts(15);
            move_group_interface_->setMaxVelocityScalingFactor(0.1);
            move_group_interface_->setMaxAccelerationScalingFactor(0.1);
            move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
            move_group_interface_->startStateMonitor();

            executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
            executor_->add_node(node_);
            moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            moveit_executor_->add_node(moveit_node_);

            // subscription for object pose stamped
            object_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/apriltag_grid_detector/object0_filtered_pose",10,
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
                    // need to learn rotation theory and Eigen C++ implementation
                    // tf to planning frame which is world
                    // then rotate wrt its own frame
                    geometry_msgs::msg::PoseStamped pose_cam = *msg;
                    Eigen::Vector3d p_in(pose_cam.pose.position.x, pose_cam.pose.position.y, pose_cam.pose.position.z);
                    Eigen::Quaterniond q_in(pose_cam.pose.orientation.w,pose_cam.pose.orientation.x,pose_cam.pose.orientation.y,pose_cam.pose.orientation.z);
                    q_in.normalize();

                    // transform object_to_grasp_linear_transform_ to object frame
                    Eigen::Quaterniond offset_in_object_frame =
                    q_in * Eigen::Quaterniond{0.0, object_to_grasp_linear_transform_.x(),object_to_grasp_linear_transform_.y(),object_to_grasp_linear_transform_.z()} * q_in.inverse();

                    Eigen::Vector3d p_out =
                    p_in + Eigen::Vector3d{offset_in_object_frame.x(), offset_in_object_frame.y(), offset_in_object_frame.z()};

                    pose_cam.pose.position.x = p_out.x();
                    pose_cam.pose.position.y = p_out.y();
                    pose_cam.pose.position.z = p_out.z();

                    Eigen::AngleAxisd rollAngle(object_to_grasp_euler_transform_.x(), Eigen::Vector3d::UnitX());
                    Eigen::AngleAxisd pitchAngle(object_to_grasp_euler_transform_.y(), Eigen::Vector3d::UnitY());
                    Eigen::AngleAxisd yawAngle(object_to_grasp_euler_transform_.z(), Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond q_offset = yawAngle * pitchAngle * rollAngle;

                    Eigen::Quaterniond q_out = q_in * q_offset;
                    q_out.normalize();

                    pose_cam.pose.orientation.w = q_out.w();
                    pose_cam.pose.orientation.x = q_out.x();
                    pose_cam.pose.orientation.y = q_out.y();
                    pose_cam.pose.orientation.z = q_out.z();

                    // Transform to world
                    geometry_msgs::msg::TransformStamped tf_world_cam;
                    try{
                        tf_world_cam = tf_buffer_->lookupTransform(world_frame_,pose_cam.header.frame_id,tf2::TimePointZero, tf2::durationFromSec(0.05));
                    } 
                    catch(const tf2::TransformException &ex){
                        RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
                        return;
                    }

                    geometry_msgs::msg::PoseStamped pose_world;
                    tf2::doTransform(pose_cam, pose_world, tf_world_cam);

                    latest_object_z_ = pose_world.pose.position.z;
                    object_seen_ = true;

                    current_setpoint_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
                    *current_setpoint_pose_ = pose_world.pose;

                    grasp_pose_publisher_->publish(pose_world);
                }
            );

            // clients
            switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller",rmw_qos_profile_services_default, callback_group_);
            left_preaction_client_ = node_->create_client<std_srvs::srv::Trigger>("/left_preaction_server/move_to_state", rmw_qos_profile_services_default, callback_group_);
            right_preaction_client_ =node_->create_client<std_srvs::srv::Trigger>("/right_preaction_server/move_to_state", rmw_qos_profile_services_default, callback_group_);
            std::string start_servo_service_name = servo_node_namespace_ + "/start_servo";
            start_servo_client_ = node_->create_client<std_srvs::srv::Trigger>(start_servo_service_name,rmw_qos_profile_services_default,callback_group_);
            std::string stop_servo_service_name = servo_node_namespace_ + "/stop_servo";
            stop_servo_client_ = node_->create_client<std_srvs::srv::Trigger>(stop_servo_service_name,rmw_qos_profile_services_default,callback_group_);
            std::string io_service_name = "left_io_and_status_controller/set_io";
            set_io_client_ = node_->create_client<ur_msgs::srv::SetIO>(io_service_name);
            if(!switch_controller_client_->wait_for_service(3s))
                RCLCPP_ERROR(node_->get_logger(),"Switch controller service is not connected!");
            if(!start_servo_client_->wait_for_service(3s))
                RCLCPP_ERROR(node_->get_logger(),"Start servo service is not connected!");
            if(!stop_servo_client_->wait_for_service(3s))
                RCLCPP_ERROR(node_->get_logger(),"Stop servo service is not connected!");
            if(!left_preaction_client_->wait_for_service(3s))
                RCLCPP_ERROR(node_->get_logger(),"Stop servo service is not connected!");
            if(!right_preaction_client_->wait_for_service(3s))
                RCLCPP_ERROR(node_->get_logger(),"Stop servo service is not connected!");
            if(!set_io_client_->wait_for_service(3s))
                RCLCPP_ERROR(node_->get_logger(),"Stop servo service is not connected!");

            // 50 Hz timer
            serial_timer_ = node_->create_wall_timer(std::chrono::milliseconds(20),[this](){pollSerial();});

            // handover server
            handover_server_ = node_->create_service<std_srvs::srv::Trigger>("~/handover",
                [this](const std_srvs::srv::Trigger_Request::SharedPtr req, std_srvs::srv::Trigger_Response::SharedPtr res){
                    // if(!object_seen_){
                    //     RCLCPP_INFO(node_->get_logger(),"Object pose not recieved yet");
                    //     return;
                    // }
                    // if(sensor_reading<SENSOR_READING_THRESHOLD){
                    //     RCLCPP_INFO(node_->get_logger(),"Nothing detected on the sensor, place something to begin");
                    //     return;
                    // }
                    // RCLCPP_INFO(node_->get_logger(),"Object pose received and object detected on the sensor, sensor value : %i",sensor_reading);
                    RCLCPP_INFO(node_->get_logger(),"Starting the handover routine");
                    auto success = handover();
                    std::string s2 = success ? "true" : "false";
                    res->message = "Handover finished with result = " + s2;
                    res->success = success;
                }
            );

            thread_ = std::thread([this](){moveit_executor_->spin();});
            executor_->spin();
        }

        // WERE HERE NOW
        bool handover(){
            RCLCPP_INFO(node_->get_logger(),"Calling left preaction");
            if(!call_trigger_client(left_preaction_client_,"left_preaction_client"))
                return false;
            
                RCLCPP_INFO(node_->get_logger(),"Calling right preaction");
            if(!call_trigger_client(right_preaction_client_,"left_preaction_client"))
                return false;
            
            RCLCPP_INFO(node_->get_logger(),"Preparing to servo");
            if(!prepare_servo()){
                RCLCPP_ERROR(node_->get_logger(),"Error preparing to start servo");
                return false;
            }
            RCLCPP_INFO(node_->get_logger(),"Preparation complete");
            
            // SERVO SHIT HERE
            grasp();

            RCLCPP_INFO(node_->get_logger(),"Switching to servo type controller");
            if(!unprepare_servo()){
                RCLCPP_ERROR(node_->get_logger(),"Error unpreparing to stop servo");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(),"Calling left preaction");
            if(!call_trigger_client(left_preaction_client_,"left_preaction_client"))
                return false;
            
                RCLCPP_INFO(node_->get_logger(),"Calling right preaction");
            if(!call_trigger_client(right_preaction_client_,"left_preaction_client"))
                return false;

            return true;
        }

        // need to estimate current velocity and acceleration at a high frequency


        // SERVO SHIT HERE
        bool grasp(){
            // approach till via point, grasp, then move back
            // Phases : 
            // reach : current to via point
            // grasp : via point to the object
            // pullback : object to via point
 
            Eigen::Vector3d grasp_position(current_setpoint_pose_->position.x,current_setpoint_pose_->position.y,current_setpoint_pose_->position.z);
            Eigen::Quaterniond grasp_orientation;
            grasp_orientation.w() = current_setpoint_pose_->orientation.w;
            grasp_orientation.x() = current_setpoint_pose_->orientation.x;
            grasp_orientation.y() = current_setpoint_pose_->orientation.y;
            grasp_orientation.z() = current_setpoint_pose_->orientation.z;
            grasp_orientation.normalize();

            auto current_pose = move_group_interface_->getCurrentPose();
            Eigen::Vector3d current_position(current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
            Eigen::Quaterniond current_orientation;
            current_orientation.w() = current_pose.pose.orientation.w;
            current_orientation.x() = current_pose.pose.orientation.x;
            current_orientation.y() = current_pose.pose.orientation.y;
            current_orientation.z() = current_pose.pose.orientation.z;
            current_orientation.normalize();

            // get the via point, in -0.1m in z direction
            auto via_point=grasp_position + grasp_orientation*(Eigen::Vector3d(0,0,-0.1));
            
            auto rate = rclcpp::Rate(50ms);
            auto twist_cmd = std::make_shared<geometry_msgs::msg::TwistStamped>();
            twist_cmd->header.frame_id="world";
            
            double reach_duration=1.2;
            double grasp_duration=0.5;
            double pullback_duration=0.5;
            double start_time = node_->get_clock()->now().seconds();
            double current_time = start_time;
            double time_elapsed = 0.0;
            
            Eigen::Matrix3d initial_conditions;
            initial_conditions.col(0) << current_position.x(), 0.0, 0.0;
            initial_conditions.col(1) << current_position.y(), 0.0, 0.0;
            initial_conditions.col(2) << current_position.z(), 0.0, 0.0;

            Eigen::Matrix3d grasp_conditions;
            grasp_conditions.col(0) << grasp_position.x(), 0.0, 0.0;
            grasp_conditions.col(1) << grasp_position.y(), 0.0, 0.0;
            grasp_conditions.col(2) << grasp_position.z(), 0.0, 0.0;

            Eigen::Vector3d via_velocity = grasp_orientation.toRotationMatrix().col(2) * 0.1;

            Eigen::Matrix3d via_point_conditions;
            via_point_conditions.col(0) << via_point.x(), via_velocity.x(), 0.0;
            via_point_conditions.col(1) << via_point.y(), via_velocity.y(), 0.0;
            via_point_conditions.col(2) << via_point.z(), via_velocity.z(), 0.0;

            Eigen::Matrix3d pullback_conditions;
            pullback_conditions.col(0) << via_point.x(), 0.0, 0.0;
            pullback_conditions.col(1) << via_point.y(), 0.0, 0.0;
            pullback_conditions.col(2) << via_point.z(), 0.0, 0.0;
            
            // reach
            Eigen::Vector<double,6> x_coeffs = compute_coeffs(time_elapsed,reach_duration,initial_conditions.col(0),via_point_conditions.col(0));
            Eigen::Vector<double,6> y_coeffs = compute_coeffs(time_elapsed,reach_duration,initial_conditions.col(1),via_point_conditions.col(1));
            Eigen::Vector<double,6> z_coeffs = compute_coeffs(time_elapsed,reach_duration,initial_conditions.col(2),via_point_conditions.col(2));

            while(rclcpp::ok() && time_elapsed<reach_duration){
                // get grasp position and orientation
                grasp_position = {current_setpoint_pose_->position.x,current_setpoint_pose_->position.y,current_setpoint_pose_->position.z};
                grasp_orientation.w() = current_setpoint_pose_->orientation.w;
                grasp_orientation.x() = current_setpoint_pose_->orientation.x;
                grasp_orientation.y() = current_setpoint_pose_->orientation.y;
                grasp_orientation.z() = current_setpoint_pose_->orientation.z;
                grasp_orientation.normalize();
                
                // get current position and orientaiton
                current_pose = move_group_interface_->getCurrentPose();
                current_position = {current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z};
                current_orientation.w() = current_pose.pose.orientation.w;
                current_orientation.x() = current_pose.pose.orientation.x;
                current_orientation.y() = current_pose.pose.orientation.y;
                current_orientation.z() = current_pose.pose.orientation.z;
                current_orientation.normalize();

                // get the via point
                auto via_point=grasp_position + grasp_orientation*(Eigen::Vector3d(0,0,-0.1));
                
                // simple pid on angular velocity
                auto angular_velocity=compute_angular_velocity(grasp_orientation,current_orientation);
                twist_cmd->twist.angular.x = angular_velocity[0];
                twist_cmd->twist.angular.y = angular_velocity[1];
                twist_cmd->twist.angular.z = angular_velocity[2];

                Eigen::Matrix3d current_conditions;
                current_conditions.col(0) << current_position.x(),compute_velocity(x_coeffs, time_elapsed),compute_acceleration(x_coeffs, time_elapsed);
                current_conditions.col(1) << current_position.y(),compute_velocity(y_coeffs, time_elapsed),compute_acceleration(y_coeffs, time_elapsed);
                current_conditions.col(2) << current_position.z(),compute_velocity(z_coeffs, time_elapsed),compute_acceleration(z_coeffs, time_elapsed);

                // get the linear velocity here
                x_coeffs = compute_coeffs(time_elapsed,reach_duration,current_conditions.col(0),via_point_conditions.col(0));
                y_coeffs = compute_coeffs(time_elapsed,reach_duration,current_conditions.col(1),via_point_conditions.col(1));
                z_coeffs = compute_coeffs(time_elapsed,reach_duration,current_conditions.col(2),via_point_conditions.col(2));
                
                rate.sleep();
                
                current_time = node_->get_clock()->now().seconds();
                time_elapsed = current_time - start_time;

                twist_cmd->twist.linear.x = compute_velocity(x_coeffs,time_elapsed);
                twist_cmd->twist.linear.y = compute_velocity(y_coeffs,time_elapsed);
                twist_cmd->twist.linear.z = compute_velocity(z_coeffs,time_elapsed);

                twist_cmd->header.stamp=node_->get_clock()->now();
                left_servo_node_main_delta_twist_cmds_publisher_->publish(*twist_cmd);
            }

            // grasp (via point to object)
            time_elapsed = 0.0;
            start_time = node_->get_clock()->now().seconds();

            x_coeffs = compute_coeffs(time_elapsed,grasp_duration,via_point_conditions.col(0),grasp_conditions.col(0));
            y_coeffs = compute_coeffs(time_elapsed,grasp_duration,via_point_conditions.col(1),grasp_conditions.col(1));
            z_coeffs = compute_coeffs(time_elapsed,grasp_duration,via_point_conditions.col(2),grasp_conditions.col(2));

            while(rclcpp::ok() && time_elapsed<grasp_duration){
                // get grasp position and orientation
                grasp_position = {current_setpoint_pose_->position.x,current_setpoint_pose_->position.y,current_setpoint_pose_->position.z};
                grasp_orientation.w() = current_setpoint_pose_->orientation.w;
                grasp_orientation.x() = current_setpoint_pose_->orientation.x;
                grasp_orientation.y() = current_setpoint_pose_->orientation.y;
                grasp_orientation.z() = current_setpoint_pose_->orientation.z;
                grasp_orientation.normalize();
                
                // get current position and orientaiton
                current_pose = move_group_interface_->getCurrentPose();
                current_position = {current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z};
                current_orientation.w() = current_pose.pose.orientation.w;
                current_orientation.x() = current_pose.pose.orientation.x;
                current_orientation.y() = current_pose.pose.orientation.y;
                current_orientation.z() = current_pose.pose.orientation.z;
                current_orientation.normalize();
                
                // simple pid on angular velocity
                auto angular_velocity=compute_angular_velocity(grasp_orientation,current_orientation);
                twist_cmd->twist.angular.x = angular_velocity[0];
                twist_cmd->twist.angular.y = angular_velocity[1];
                twist_cmd->twist.angular.z = angular_velocity[2];

                Eigen::Matrix3d current_conditions;
                current_conditions.col(0) << current_position.x(),compute_velocity(x_coeffs, time_elapsed),compute_acceleration(x_coeffs, time_elapsed);
                current_conditions.col(1) << current_position.y(),compute_velocity(y_coeffs, time_elapsed),compute_acceleration(y_coeffs, time_elapsed);
                current_conditions.col(2) << current_position.z(),compute_velocity(z_coeffs, time_elapsed),compute_acceleration(z_coeffs, time_elapsed);

                // get the linear velocity here
                x_coeffs = compute_coeffs(time_elapsed,grasp_duration,current_conditions.col(0),grasp_conditions.col(0));
                y_coeffs = compute_coeffs(time_elapsed,grasp_duration,current_conditions.col(1),grasp_conditions.col(1));
                z_coeffs = compute_coeffs(time_elapsed,grasp_duration,current_conditions.col(2),grasp_conditions.col(2));
                
                rate.sleep();
                
                current_time = node_->get_clock()->now().seconds();
                time_elapsed = current_time - start_time;

                twist_cmd->twist.linear.x = compute_velocity(x_coeffs,time_elapsed);
                twist_cmd->twist.linear.y = compute_velocity(y_coeffs,time_elapsed);
                twist_cmd->twist.linear.z = compute_velocity(z_coeffs,time_elapsed);

                twist_cmd->header.stamp=node_->get_clock()->now();
                left_servo_node_main_delta_twist_cmds_publisher_->publish(*twist_cmd);
            }

            // pull back (object to via point)
            start_time = node_->get_clock()->now().seconds();
            time_elapsed = 0.0;
            x_coeffs = compute_coeffs(time_elapsed,pullback_duration,grasp_conditions.col(0),pullback_conditions.col(0));
            y_coeffs = compute_coeffs(time_elapsed,pullback_duration,grasp_conditions.col(1),pullback_conditions.col(1));
            z_coeffs = compute_coeffs(time_elapsed,pullback_duration,grasp_conditions.col(2),pullback_conditions.col(2));

            while(rclcpp::ok() && time_elapsed<pullback_duration){
                // get grasp position and orientation
                grasp_position = {current_setpoint_pose_->position.x,current_setpoint_pose_->position.y,current_setpoint_pose_->position.z};
                grasp_orientation.w() = current_setpoint_pose_->orientation.w;
                grasp_orientation.x() = current_setpoint_pose_->orientation.x;
                grasp_orientation.y() = current_setpoint_pose_->orientation.y;
                grasp_orientation.z() = current_setpoint_pose_->orientation.z;
                grasp_orientation.normalize();
                
                // get current position and orientaiton
                current_pose = move_group_interface_->getCurrentPose();
                current_position = {current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z};
                current_orientation.w() = current_pose.pose.orientation.w;
                current_orientation.x() = current_pose.pose.orientation.x;
                current_orientation.y() = current_pose.pose.orientation.y;
                current_orientation.z() = current_pose.pose.orientation.z;
                current_orientation.normalize();
                
                // simple pid on angular velocity
                auto angular_velocity=compute_angular_velocity(grasp_orientation,current_orientation);
                twist_cmd->twist.angular.x = angular_velocity[0];
                twist_cmd->twist.angular.y = angular_velocity[1];
                twist_cmd->twist.angular.z = angular_velocity[2];

                Eigen::Matrix3d current_conditions;
                current_conditions.col(0) << current_position.x(),compute_velocity(x_coeffs, time_elapsed),compute_acceleration(x_coeffs, time_elapsed);
                current_conditions.col(1) << current_position.y(),compute_velocity(y_coeffs, time_elapsed),compute_acceleration(y_coeffs, time_elapsed);
                current_conditions.col(2) << current_position.z(),compute_velocity(z_coeffs, time_elapsed),compute_acceleration(z_coeffs, time_elapsed);

                // get the linear velocity here
                x_coeffs = compute_coeffs(time_elapsed,pullback_duration,current_conditions.col(0),pullback_conditions.col(0));
                y_coeffs = compute_coeffs(time_elapsed,pullback_duration,current_conditions.col(1),pullback_conditions.col(1));
                z_coeffs = compute_coeffs(time_elapsed,pullback_duration,current_conditions.col(2),pullback_conditions.col(2));
                
                rate.sleep();
                
                current_time = node_->get_clock()->now().seconds();
                time_elapsed = current_time - start_time;

                twist_cmd->twist.linear.x = compute_velocity(x_coeffs,time_elapsed);
                twist_cmd->twist.linear.y = compute_velocity(y_coeffs,time_elapsed);
                twist_cmd->twist.linear.z = compute_velocity(z_coeffs,time_elapsed);

                twist_cmd->header.stamp=node_->get_clock()->now();
                left_servo_node_main_delta_twist_cmds_publisher_->publish(*twist_cmd);
            }
            return true;
        }

        // need to measure velocity and acceleration better
        // feedforward for now, the velocity and acceleration are computed using the equation
        double compute_velocity(Eigen::Vector<double,6> coeffs, double current_time){
            return coeffs[1] + 2*coeffs[2]*current_time + 3*coeffs[3]*std::pow(current_time,2) + 4*coeffs[4]*std::pow(current_time,3) + 5*coeffs[5]*std::pow(current_time,4);
        }

        double compute_acceleration(Eigen::Vector<double,6> coeffs, double current_time){
            return 2*coeffs[2] + 6*coeffs[3]*current_time + 12*coeffs[4]*std::pow(current_time,2) + 20*coeffs[5]*std::pow(current_time,3);
        }

        // the conditions are in the format of position, velocity and acceleration for only one axis and not all 3
        Eigen::Vector<double,6> compute_coeffs(double start, double finish, Eigen::Vector3d initial_conditions, Eigen::Vector3d final_conditions){
            Eigen::Matrix<double,6,6> A;
            A << 
                1, start, std::pow(start,2), std::pow(start,3), std::pow(start,4), std::pow(start,5),
                0, 1, 2*start, 3*std::pow(start,2), 4*std::pow(start,3), 5*std::pow(start,4),
                0, 0, 2, 6*start, 12*std::pow(start,2), 20*std::pow(start,3),
                1, finish, std::pow(finish,2), std::pow(finish,3), std::pow(finish,4), std::pow(finish,5),
                0, 1, 2*finish, 3*std::pow(finish,2), 4*std::pow(finish,3), 5*std::pow(finish,4),
                0, 0, 2, 6*finish, 12*std::pow(finish,2), 20*std::pow(finish,3);
            Eigen::Vector<double,6> boundary_conditions;
            boundary_conditions << initial_conditions,final_conditions;
            return A.inverse()*boundary_conditions;
        }

        Eigen::Vector3d compute_angular_velocity(Eigen::Quaterniond target, Eigen::Quaterniond current){ 
            static float K = 0.5;
            static float P = 1.0;
            auto angular_distance = Eigen::AngleAxisd(target*current.inverse());
            return K*P*angular_distance.angle()*angular_distance.axis();
        }

        void gripper_on()
        {
            if(gripper_pin1_){
                auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
                request->fun = request->FUN_SET_DIGITAL_OUT;
                request->pin = gripper_pin1_;
                request->state = request->STATE_ON;
                (void)set_io_client_->async_send_request(request);
                RCLCPP_INFO(node_->get_logger(), "Gripper on (pin_out1)");
            }
            if(gripper_pin2_){
                auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
                request->fun = request->FUN_SET_DIGITAL_OUT;
                request->pin = gripper_pin2_;
                request->state = request->STATE_ON;
                (void)set_io_client_->async_send_request(request);
                RCLCPP_INFO(node_->get_logger(), "Gripper on (pin_out2)");
            }
        }

        void gripper_off()
        {
            if(gripper_pin1_){
                auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
                request->fun = request->FUN_SET_DIGITAL_OUT;
                request->pin = gripper_pin1_;
                request->state = request->STATE_OFF;
                (void)set_io_client_->async_send_request(request);
                RCLCPP_INFO(node_->get_logger(), "Gripper on (pin_out1)");
            }
            if(gripper_pin2_){
                auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
                request->fun = request->FUN_SET_DIGITAL_OUT;
                request->pin = gripper_pin2_;
                request->state = request->STATE_OFF;
                (void)set_io_client_->async_send_request(request);
                RCLCPP_INFO(node_->get_logger(), "Gripper on (pin_out2)");
            }
        }
    
        bool prepare_servo(){
            RCLCPP_INFO(node_->get_logger(),"Switching controller");
            auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            request->activate_controllers = std::vector<std::string>{servo_controller_};
            request->deactivate_controllers = std::vector<std::string>{non_servo_controller_};
            request->strictness = request->BEST_EFFORT;

            auto future = switch_controller_client_->async_send_request(request);

            if (future.wait_for(10s) != std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(), "Switch controller service call timed out!");
            }
            else{
                auto resp = future.get();
                if (resp->ok){
                    RCLCPP_INFO(node_->get_logger(),"Service successful");
                }
                else{
                    RCLCPP_ERROR(node_->get_logger(),"Service couldn't switch controller");
                    return false;
                }
            }

            auto start_servo_future = start_servo_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger_Request>());
            if(start_servo_future.wait_for(10s)!=std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(),"Start servo service call timed out");
            }
            else{
                auto resp = start_servo_future.get();
                if(resp->success){
                    RCLCPP_INFO(node_->get_logger(),"Service successful");
                    return true;
                }
                else{
                    RCLCPP_ERROR(node_->get_logger(),"Couldn't switch controller");
                    return false;
                }
            }
        }

        bool unprepare_servo(){
            auto stop_servo_future = stop_servo_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger_Request>());
            if(stop_servo_future.wait_for(10s)!=std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(),"stop_servo service timed out");
                return false;
            }
            else{
                auto stop_servo_res = stop_servo_future.get();
                if(!stop_servo_res->success){
                    RCLCPP_ERROR(node_->get_logger(),"stop_servo service unsuccessful");
                    return false;
                }
                else{
                    RCLCPP_INFO(node_->get_logger(),"stop_servo service successful");
                }
            }

            RCLCPP_INFO(node_->get_logger(),"Switching back controller");
            auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            request->activate_controllers = std::vector<std::string>{non_servo_controller_};
            request->deactivate_controllers = std::vector<std::string>{servo_controller_};
            request->strictness = request->BEST_EFFORT;

            auto future = switch_controller_client_->async_send_request(request);

            if (future.wait_for(10s) != std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(), "Switch controller service call timed out!");
            }
            else{
                auto resp = future.get();
                if (resp->ok){
                    RCLCPP_INFO(node_->get_logger(),"Service successful");
                    return true;
                }
                else{
                    RCLCPP_ERROR(node_->get_logger(),"Service couldn't swith controller");
                    return false;
                }
            }
        }

        bool call_trigger_client(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,const std::string &name){
            using namespace std::chrono_literals;
            if(!client->wait_for_service(5s)){
                RCLCPP_ERROR(node_->get_logger(), "Service %s not available", name.c_str());
                return false;
            }
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future  = client->async_send_request(request);
            if(future.wait_for(10s) != std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(), "Service %s call timed out", name.c_str());
                return false;
            }
            auto response = future.get();
            if(!response->success){
                RCLCPP_ERROR(node_->get_logger(), "Service %s responded with failure: %s",name.c_str(), response->message.c_str());
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Service %s succeeded: %s",name.c_str(), response->message.c_str());
            return true;
        }

        // serial shit
        bool openSerialPort(){
            serial_fd_ = ::open(serial_port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
            if(serial_fd_ < 0){
                RCLCPP_ERROR(node_->get_logger(), "open(%s) failed: %s", serial_port_.c_str(), std::strerror(errno));
                return false;
            }

            termios tty{};
            if(tcgetattr(serial_fd_, &tty) != 0){
                RCLCPP_ERROR(node_->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
                ::close(serial_fd_);
                serial_fd_ = -1;
                return false;
            }

            // Raw-ish config: 8N1, no flow control
            cfmakeraw(&tty);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~CRTSCTS;
            tty.c_cflag &= ~PARENB;
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;

            tty.c_cc[VMIN]  = 0;
            tty.c_cc[VTIME] = 0;

            cfsetispeed(&tty, baud_flag_);
            cfsetospeed(&tty, baud_flag_);

            if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0){
              RCLCPP_ERROR(node_->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
              ::close(serial_fd_);
              serial_fd_ = -1;
              return false;
            }
            return true;
        }

        void pollSerial(){
            if(serial_fd_<0){
              return;
            }
            char buf[256];
            // Drain everything available this tick
            while(true){
                const ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
                if (n > 0) {
                  serial_rx_buffer_.append(buf, buf + n);
                  processSerialLines();
                  continue;
                }
                if(n == 0){
                  // No data this time
                  break;
                }

                // n < 0
                if(errno == EAGAIN || errno == EWOULDBLOCK){
                  // No data available (expected in non-blocking mode)
                  break;
                }

                RCLCPP_WARN(node_->get_logger(), "serial read() error: %s", std::strerror(errno));
                break;
            }

            // Publish last value at 50 Hz (even if unchanged)
            std_msgs::msg::Int32 msg;
            msg.data = sensor_reading;
            sensor_value_publisher_->publish(msg);
        }

        void processSerialLines(){
            while (true) {
                const std::size_t pos = serial_rx_buffer_.find('\n');
                if(pos == std::string::npos){
                    return;
                }   
                std::string line = serial_rx_buffer_.substr(0, pos);
                serial_rx_buffer_.erase(0, pos + 1);    
                // Handle CRLF
                if(!line.empty() && line.back() == '\r'){
                    line.pop_back();
                }   
                if(line.empty()){
                    continue;
                }   
                // Convert ASCII digits to int
                try{
                    sensor_reading = std::stoi(line);
                } 
                catch(const std::exception &e){
                    RCLCPP_WARN(node_->get_logger(), "Failed to parse serial line '%s' as int: %s", line.c_str(), e.what());
                }
            }
        }

    private:
        static speed_t baudFromInt(int baud)
        {
            switch(baud){
                case 9600: return B9600;
                case 19200: return B19200;
                case 38400: return B38400;
                case 57600: return B57600;
                case 115200: return B115200;
                default: return B9600;
            }
        }

        std::thread thread_;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

        // data
        rclcpp::Node::SharedPtr node_;
        rclcpp::Node::SharedPtr moveit_node_;
        std::shared_ptr<MoveGroupInterface> move_group_interface_;
        geometry_msgs::msg::Pose::SharedPtr current_setpoint_pose_;
        int sensor_reading=0;
        int serial_fd_ = -1;
        std::string serial_rx_buffer_;

        // hardcoded shit
        double threshold_height_ = THRESHOLD_HEIGHT;
        Eigen::Vector3d object_to_grasp_linear_transform_{0.03, -0.015, -0.0375};
        Eigen::Vector3d object_to_grasp_euler_transform_{0.0, -M_PI/2, 0.0};
        float latest_object_z_=0;
        bool object_seen_=false;
        int gripper_pin1_ = 13;
        int gripper_pin2_ = 0;

        //  lift sensor shit
        std::string serial_port_="/dev/ttyACM0";
        int serial_baud_=9600;
        speed_t baud_flag_ = B9600;

        // initialization helpers
        std::string servo_controller_="left_forward_velocity_controller";
        std::string non_servo_controller_="left_scaled_joint_trajectory_controller";
        std::string planning_group_="left_ur16e";
        std::string servo_node_namespace_="left_servo_node_main";
        
        // subscriptions
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_subscription_;
        
        // clients
        rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_preaction_client_;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_preaction_client_;
        rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;
        
        // publishers
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr sensor_value_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_servo_node_main_delta_twist_cmds_publisher_;
        
        // services
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handover_server_;

        // timer
        rclcpp::TimerBase::SharedPtr serial_timer_;
        
        // this looks odd without a comment, so i commented it
        rclcpp::CallbackGroup::SharedPtr callback_group_;
        
        // dafuhhh
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        const std::string world_frame_ = "world";
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto human_robot_handover = HumanRobotHandover();
    rclcpp::shutdown();
    return 0;
}