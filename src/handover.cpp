// for the actual handover function
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

bool Handover::handover()
{
    auto LOGGER = node_->get_logger();

    // move to left preaction states
    dual_arm_control_interface_->move_to_joint_positions(
        std::vector<double>{
            -3.9176607767688196, 
            -1.245649353866913, 
            1.845574680958883, 
            -0.8931537431529541, 
            -0.46324092546571904, 
            0.24232390522956848
        },
        dual_arm_control_interface_->left_move_group_interface_
    );

    // move to right preaction states
    dual_arm_control_interface_->move_to_joint_positions(
        std::vector<double>{
            0.15968317804148882,
            -1.247802739692411,
            -1.9956461569496642,
            -1.1175868831372546,
            1.5685058669109873,
            -2.958524343639371,
        },
        dual_arm_control_interface_->right_move_group_interface_
    );

    right_gripper_off();

    std::cout<<"preparing servo"<<std::endl;
    std::this_thread::sleep_for(0.5s);
    ee_servo_handle_->prepare_servo_();
    std::cout<<"starting servo"<< std::endl;
    std::this_thread::sleep_for(0.5s);
    ee_servo_handle_->start_servo_();
    std::cout<<"ready to move"<<std::endl;

    Eigen::Vector3d zero_velocity = Eigen::Vector3d::Zero();
    
    // wait until the the object goes up by 10cm
    double obj_z = grasp_position.z();
    auto rate = rclcpp::Rate(50ms);
    RCLCPP_INFO(LOGGER,"Waiting for the object to be lifted up by 10cm");
    int i = 0;
    while(rclcpp::ok()){
        if(grasp_position.z()-obj_z>0.1)
            break;
        if(i>1000){
            RCLCPP_ERROR(LOGGER,"Timed out after waiting 50s");
        }
        i++;
        rate.sleep();
    }
    
    // approach
    MPC_go_to_approach_state(approach_phase_duration);

    auto freeze_grasp_position(grasp_position), freeze_via_point_position(via_point_position);
    auto freeze_grasp_orientation(grasp_orientation), freeze_via_point_orientation(via_point_orientation);

    // grasp
    MPC_go_to_state(1.0,freeze_grasp_position,zero_velocity,freeze_grasp_orientation);
    RCLCPP_INFO(LOGGER,"Graaaaaasp!!");
    right_gripper_on();
    std::this_thread::sleep_for(0.5s);
    
    std::cout<<"Stopping servo now"<<std::endl;
    ee_servo_handle_->stop_servo_();
    std::cout<<"Unpreparing now"<<std::endl;
    ee_servo_handle_->unprepare_servo_();

    std::this_thread::sleep_for(0.2s);

    auto current_pose = dual_arm_control_interface_->get_current_ee_pose("right");
    auto retract_pose = [this,current_pose](){
        geometry_msgs::msg::Pose retract_pose;
        // move 5cm in -y direction of the ee pose
        Eigen::Quaterniond ee_orientation(
            current_pose->orientation.w,
            current_pose->orientation.x,
            current_pose->orientation.y,
            current_pose->orientation.z
        );
        Eigen::Matrix<double,3,3> rmat = ee_orientation.toRotationMatrix();
        Eigen::Vector3d current_position(
            current_pose->position.x,
            current_pose->position.y,
            current_pose->position.z
        );
        Eigen::Quaterniond retract_orientation(0.,1.,0.0,0.);
        retract_orientation.normalize();

        auto target_position = current_position - rmat.col(1)*0.1;
        
        retract_pose.position.x = target_position.x();
        retract_pose.position.y = target_position.y();
        retract_pose.position.z = target_position.z();
        retract_pose.orientation.w = retract_orientation.w();
        retract_pose.orientation.x = retract_orientation.x();
        retract_pose.orientation.y = retract_orientation.y();
        retract_pose.orientation.z = retract_orientation.z();

        return retract_pose;
    }();

    geometry_msgs::msg::Pose right_handover_pose;
    right_handover_pose.position.x =0.1;
    right_handover_pose.position.y =0.;
    right_handover_pose.position.z =0.3;
    right_handover_pose.orientation.w = 0.0;
    right_handover_pose.orientation.x = -0.7071;
    right_handover_pose.orientation.y = 0.7071;
    right_handover_pose.orientation.z = 0.0;

    geometry_msgs::msg::Pose left_handover_via_point_pose,left_handover_pose;
    
    left_handover_via_point_pose.position.x = -0.20;
    left_handover_via_point_pose.position.y = 0.0;
    left_handover_via_point_pose.position.z = 0.218;
    left_handover_via_point_pose.orientation.w = 0.7071;
    left_handover_via_point_pose.orientation.x = 0.0;
    left_handover_via_point_pose.orientation.y = 0.7071;
    left_handover_via_point_pose.orientation.z = 0.0;
    
    left_handover_pose.position.x = 0.06;
    left_handover_pose.position.y = 0.0;
    left_handover_pose.position.z = 0.20;
    left_handover_pose.orientation.w = 0.7071;
    left_handover_pose.orientation.x = 0.0;
    left_handover_pose.orientation.y = 0.7071;
    left_handover_pose.orientation.z = 0.0;
    
    // right to handover
    auto right_to_robot_handover = dual_arm_control_interface_->async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{retract_pose,right_handover_pose},
        std::vector<double>{1,2},
        0.3,
        0.1,
        "right"
    );

    left_gripper_off();

    // left to handover
    auto left_to_robot_handover = dual_arm_control_interface_->async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{left_handover_via_point_pose,left_handover_pose},
        std::vector<double>{2,2},
        0.3,
        0.1,
        "left"
    );


    dual_arm_control_interface_->block_till_response_execute_cubic_trajectory(right_to_robot_handover,5s);
    dual_arm_control_interface_->block_till_response_execute_cubic_trajectory(left_to_robot_handover,5s);

    // right grasp and left let go
    left_gripper_on();
    std::this_thread::sleep_for(1s);
    right_gripper_off();
    std::this_thread::sleep_for(1s);

    // right +z 5cm and back to preaction, left move in -x 
    auto right_arm_current_pose = *dual_arm_control_interface_->get_current_ee_pose("right");
    auto left_arm_current_pose = *dual_arm_control_interface_->get_current_ee_pose("left");

    auto right_arm_after_robot_handover(right_arm_current_pose);
    right_arm_after_robot_handover.position.z +=0.1;
    auto right_arm_after_robot_handover_motion = dual_arm_control_interface_->async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{right_arm_after_robot_handover},
        std::vector<double>{1},
        0.3,
        0.0,
        "right"
    );

    // now left arm place and back to preaction
    geometry_msgs::msg::Pose left_after_handover_1(left_arm_current_pose),left_after_handover_2,left_after_handover_3_place;
    left_after_handover_1.position.x -= 0.25;
    left_after_handover_1.position.y -= 0.15;
    left_after_handover_1.orientation.w = 0.5;
    left_after_handover_1.orientation.x = 0.5;
    left_after_handover_1.orientation.y = 0.5;
    left_after_handover_1.orientation.z = -0.5;
    
    left_after_handover_2 = left_after_handover_1;
    left_after_handover_2.position.x -= 0.2;
    left_after_handover_2.position.y += 0.1;
    left_after_handover_2.position.z -= 0.05;

    left_after_handover_3_place = left_after_handover_2;
    left_after_handover_3_place.position.z = 0.1;

    auto left_to_place = dual_arm_control_interface_->async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{
            left_after_handover_1,
            left_after_handover_2,
            left_after_handover_3_place
        },
        std::vector<double>{3.0,2.0,1.5},
        0.3,
        0.1,
        "left"
    );

    dual_arm_control_interface_->block_till_response_execute_cubic_trajectory(right_arm_after_robot_handover_motion,5s);

    dual_arm_control_interface_->move_to_joint_positions(
        std::vector<double>{
            0.15968317804148882,
            -1.247802739692411,
            -1.9956461569496642,
            -1.1175868831372546,
            1.5685058669109873,
            -2.958524343639371,
        },
        dual_arm_control_interface_->right_move_group_interface_
    );

    dual_arm_control_interface_->block_till_response_execute_cubic_trajectory(left_to_place, 10s);

    std::this_thread::sleep_for(0.5s);
    left_gripper_off();
    std::this_thread::sleep_for(0.5s);
    RCLCPP_INFO(LOGGER,"PLAAAAAAAAAAAAAAAAAAAAAAAAACED!");


    // left +y,z 0.1,0.05 and then +z by 0.1cm
    geometry_msgs::msg::Pose left_after_placing_1(left_after_handover_3_place), left_after_placing_2;
    left_after_placing_1.position.y += 0.03;
    left_after_placing_1.position.z += 0.1;

    left_after_placing_2 = left_after_placing_1;
    left_after_placing_2.position.z += 0.1;

    auto left_after_placing_object = dual_arm_control_interface_->async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{
            left_after_placing_1,
            left_after_placing_2
        },
        std::vector<double>{1.0,1.0},
        0.3,
        0.1,
        "left"
    );

    dual_arm_control_interface_->block_till_response_execute_cubic_trajectory(left_after_placing_object, 10s);
    left_gripper_neutral();
    right_gripper_neutral();
    
    // move to left preaction states
    dual_arm_control_interface_->move_to_joint_positions(
        std::vector<double>{
            -3.9176607767688196, 
            -1.245649353866913, 
            1.845574680958883, 
            -0.8931537431529541, 
            -0.46324092546571904, 
            0.24232390522956848
        },
        dual_arm_control_interface_->left_move_group_interface_
    );

    std::this_thread::sleep_for(0.2s);
    return true;
}