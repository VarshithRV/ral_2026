// gripper functionality
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

void Handover::right_gripper_on(){
    auto gripper_on_msg1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT5;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT6;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT7;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
}

void Handover::right_gripper_off(){
    auto gripper_on_msg1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT5;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT6;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT7;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
}

void Handover::right_gripper_neutral(){
    auto gripper_on_msg1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT5;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT6;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT7;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
}


void Handover::left_gripper_on(){
    auto gripper_on_msg1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT3;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT5;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT6;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT7;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
}

void Handover::left_gripper_off(){
    auto gripper_on_msg1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT3;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT5;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT6;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT7;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
}

void Handover::left_gripper_neutral(){
    auto gripper_on_msg1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT3;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT5;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT6;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT7;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->left_set_io_client_->async_send_request(gripper_on_msg1);
}