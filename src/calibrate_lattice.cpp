#include <functional>
#include <chrono>
#include <memory>
#include <cmath>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <motion_planning_abstractions/dual_arm_waypoint_programming.hpp>

using namespace std::chrono_literals;

// this hosts servers for step, ramp and sinusoidal z(t)

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("calibrate_lattice");
    
    rclcpp::executors::MultiThreadedExecutor::SharedPtr mex = 
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    mex->add_node(node);
    
    rclcpp::CallbackGroup::SharedPtr mex_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto dual_arm_control_interface  = std::make_shared<DualArmControlInterface>(node);

    auto step_input_callback = 
    [dual_arm_control_interface,node]
    (std_srvs::srv::Trigger_Request::SharedPtr req, std_srvs::srv::Trigger_Response::SharedPtr res)
    {
        auto start_pose = *(dual_arm_control_interface->get_current_ee_pose("left"));
        
        std::vector<double> step_sizes{0.001,0.0015,0.003};
        std::vector<double> movement_durations{0.5,1.0,1.5};
        std::vector<int> stepss{3,3,3};
        auto wait_duration = 10s;

        auto sub_calibration = [wait_duration,dual_arm_control_interface,start_pose](int steps, double step_size, double movement_duration ){
            geometry_msgs::msg::Pose setp(start_pose);
    
            for(int i=0; i<steps; i++){
                setp.position.z -= step_size;
                auto track_traj = dual_arm_control_interface->execute_waypoints_cubic(
                    std::vector<geometry_msgs::msg::Pose>{setp},std::vector<double>{movement_duration},0.3,0.0,"left"
                );
                std::this_thread::sleep_for(wait_duration);
            }
    
            for(int i=0; i<steps; i++){
                setp.position.z += step_size;
                auto track_traj = dual_arm_control_interface->execute_waypoints_cubic(
                    std::vector<geometry_msgs::msg::Pose>{setp},std::vector<double>{movement_duration},0.3,0.0,"left"
                );
                std::this_thread::sleep_for(wait_duration);
            }
        };

        for(int i=0; i<3; i++){
            double step_size = step_sizes[i];
            double steps = stepss[i];
            for(int j = 0; j<3; j++){
                
                sub_calibration(steps,step_size,movement_durations[j]);
                RCLCPP_INFO(node->get_logger(),"Movement duration : %f",movement_durations[j]);
                RCLCPP_INFO(node->get_logger(), "################################################################################################################################");
                std::this_thread::sleep_for(10s);
            }
            std::this_thread::sleep_for(0.5s);
            RCLCPP_INFO(node->get_logger(), "################################################################################################################################");
            RCLCPP_INFO(node->get_logger(),"Calibration for step_size : %f, steps: %f\n",step_size,steps);
            std::this_thread::sleep_for(10s);
        }


    };

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_input_server = 
    node->create_service<std_srvs::srv::Trigger>(
        "~/step_input_calibration",
        step_input_callback,
        rmw_qos_profile_services_default,
        mex_cb_group
    );

    mex->spin();
}