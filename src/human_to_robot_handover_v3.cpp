#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "motion_planning_abstractions/ee_servo.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"

using namespace std::chrono_literals;

class Handover
{
    public:
        Handover(rclcpp::Node::SharedPtr node){
            if(node == nullptr){
                std::cout<<"node passed is a nullptr!!, exiting"<<std::endl;
                return;
            }
            else
                node_ = node;

            // servo and dual arm control inits
            ee_servo_handle_ = std::make_shared<EEServo>(node_);
            dual_arm_control_interface_ = std::make_shared<DualArmControlInterface>(node_);
            
            // callback groups
            parallel_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            mex_cb_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // handover server initialization
            handover_server_ = node_->create_service<std_srvs::srv::Trigger>(
                "~/handover",
                [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
                    res->success= handover();
                },
                rmw_qos_profile_services_default,
                parallel_cb_group_
            );
        }

        // handover function
        bool handover(){
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
                set_velocity(Eigen::Vector<double,6>(0,0,-0.05,0,0,0));
                auto current_ee_pose = dual_arm_control_interface_->get_current_ee_pose("left");
                RCLCPP_INFO(LOGGER,"Current position : %.2f,%.2f,%.2f",current_ee_pose->position.x,current_ee_pose->position.y,current_ee_pose->position.z);
                rate.sleep();
                if(i>100){
                    break;
                }
            }

            std::cout<<"Stopping servo now"<<std::endl;
            ee_servo_handle_->stop_servo_();
            std::cout<<"Unpreparing now"<<std::endl;
            ee_servo_handle_->unprepare_servo_();
            std::cout<<"Done"<<std::endl;
        }
    
    private:
        rclcpp::Node::SharedPtr node_;
        
        std::shared_ptr<EEServo> ee_servo_handle_;
        std::shared_ptr<DualArmControlInterface> dual_arm_control_interface_;

        rclcpp::CallbackGroup::SharedPtr parallel_cb_group_;
        rclcpp::CallbackGroup::SharedPtr mex_cb_group_;
        
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr handover_server_;

        // helper function to set velocity
        void set_velocity(Eigen::Vector<double,6> velocity){
            auto vel = geometry_msgs::msg::TwistStamped();
            vel.header.frame_id="world";
            vel.header.stamp = node_->get_clock()->now();
            vel.twist.linear.x = velocity[0];
            vel.twist.linear.y = velocity[1];
            vel.twist.linear.z = velocity[2];
            vel.twist.angular.x = velocity[0];
            vel.twist.angular.y = velocity[1];
            vel.twist.angular.z = velocity[2];
            ee_servo_handle_->set_vel_setpoint_(vel);
            return;
        }
};


int main(int argc, const char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("human_to_robot_handover_v3");

    auto handover_handle = std::make_shared<Handover>(node);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    
    rclcpp::shutdown();
}