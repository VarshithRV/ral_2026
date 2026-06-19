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

#include "ral_2026/human_to_robot_handover_2.hpp"

using namespace std::chrono_literals;

bool Handover::set_io_sync(
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr client,
    int8_t pin,
    float state)
{
    auto req = std::make_shared<ur_msgs::srv::SetIO::Request>();
    req->fun = req->FUN_SET_DIGITAL_OUT;
    req->pin = pin;
    req->state = state;

    if (!client->wait_for_service(1s)) {
        RCLCPP_ERROR(node_->get_logger(), "SetIO service not available");
        return false;
    }

    auto future = client->async_send_request(req);

    auto status = future.wait_for(2s);

    if (status != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "SetIO service call timed out");
        return false;
    }

    auto response = future.get();

    if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(), "SetIO request unsuccessful");
        return false;
    }

    return true;
}

void Handover::right_gripper_on(){
    auto client = dual_arm_control_interface_->right_set_io_client_;

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT5,
                ur_msgs::srv::SetIO::Request::STATE_ON);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT6,
                ur_msgs::srv::SetIO::Request::STATE_ON);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT7,
                ur_msgs::srv::SetIO::Request::STATE_OFF);
}

void Handover::right_gripper_off(){
    auto client = dual_arm_control_interface_->right_set_io_client_;

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT5,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT6,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT7,
                ur_msgs::srv::SetIO::Request::STATE_ON);
}

void Handover::right_gripper_neutral(){
    auto client = dual_arm_control_interface_->right_set_io_client_;

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT5,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT6,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT7,
                ur_msgs::srv::SetIO::Request::STATE_OFF);
}

void Handover::left_gripper_on(){
    auto client = dual_arm_control_interface_->left_set_io_client_;

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT3,
                ur_msgs::srv::SetIO::Request::STATE_ON);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT5,
                ur_msgs::srv::SetIO::Request::STATE_ON);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT6,
                ur_msgs::srv::SetIO::Request::STATE_ON);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT7,
                ur_msgs::srv::SetIO::Request::STATE_OFF);
}

void Handover::left_gripper_off(){
    auto client = dual_arm_control_interface_->left_set_io_client_;

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT3,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT5,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT6,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT7,
                ur_msgs::srv::SetIO::Request::STATE_ON);
}

void Handover::left_gripper_neutral(){
    auto client = dual_arm_control_interface_->left_set_io_client_;

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT3,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT5,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT6,
                ur_msgs::srv::SetIO::Request::STATE_OFF);

    set_io_sync(client,
                ur_msgs::srv::SetIO::Request::PIN_CONF_OUT7,
                ur_msgs::srv::SetIO::Request::STATE_OFF);
}