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

#include "ral_2026/human_to_robot_handover_2.hpp"

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
            -0.4291909376727503,
            -1.549779526223876,
            -2.1257333755493164,
            -2.6456037960448207,
            -2.003878895436422,
            0.054730068892240524,
        },
        dual_arm_control_interface_->right_move_group_interface_
    );

    right_gripper_off();

    std::cout<<"preparing servo"<<std::endl;
    ee_servo_handle_->prepare_servo_();
    std::this_thread::sleep_for(0.5s);
    std::cout<<"starting servo"<< std::endl;
    ee_servo_handle_->start_servo_();
    std::this_thread::sleep_for(0.5s);
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

    // retract
    MPC_go_to_state(1.0,freeze_via_point_position,zero_velocity,freeze_grasp_orientation);
    
    std::cout<<"Stopping servo now"<<std::endl;
    ee_servo_handle_->stop_servo_();
    std::cout<<"Unpreparing now"<<std::endl;
    ee_servo_handle_->unprepare_servo_();

    std::this_thread::sleep_for(0.2s);
    
    // need to place here
    dual_arm_control_interface_->execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{
            [this](){
                geometry_msgs::msg::Pose place_pose;
                place_pose.position.x = 0.386;
                place_pose.position.y = -0.029;
                place_pose.position.z = 0.206;
                place_pose.orientation.x = 0.720;
                place_pose.orientation.y = -0.024;
                place_pose.orientation.z = 0.027;
                place_pose.orientation.w = 0.694;
                return place_pose;
            }(),
            [this](){
                geometry_msgs::msg::Pose place_pose;
                place_pose.position.x = 0.386;
                place_pose.position.y = -0.029;
                place_pose.position.z = 0.065;
                place_pose.orientation.x = 0.720;
                place_pose.orientation.y = -0.024;
                place_pose.orientation.z = 0.027;
                place_pose.orientation.w = 0.694;
                return place_pose;
            }()
        },
        std::vector<double>{2.0,1.0},
        0.3,
        0.05,
        "right"
    );

    right_gripper_off();
    std::this_thread::sleep_for(1s);
    // move up a bit
    dual_arm_control_interface_->execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{
            [this](){
                geometry_msgs::msg::Pose place_pose;
                place_pose.position.x = 0.386;
                place_pose.position.y = -0.079;
                place_pose.position.z = 0.256;
                place_pose.orientation.x = 0.720;
                place_pose.orientation.y = -0.024;
                place_pose.orientation.z = 0.027;
                place_pose.orientation.w = 0.694;
                return place_pose;
            }()
        },
        std::vector<double>{2.0},
        0.3,
        0.0,
        "right"
    );

    // gripper neutral
    right_gripper_neutral();

    // move to right preaction
    // dual_arm_control_interface_->move_to_joint_positions(
    //     std::vector<double>{
    //         0.1587774157524109,
    //         -1.4757498067668458,
    //         -2.4635510444641113,
    //         -2.381132741967672,
    //         -1.4155743757831019,
    //         0.07609853893518448,
    //     },
    //     dual_arm_control_interface_->right_move_group_interface_
    // );
    // move to right preaction states
    dual_arm_control_interface_->move_to_joint_positions(
        std::vector<double>{
            -0.4291909376727503,
            -1.549779526223876,
            -2.1257333755493164,
            -2.6456037960448207,
            -2.003878895436422,
            0.054730068892240524,
        },
        dual_arm_control_interface_->right_move_group_interface_
    );

    return true;
}

void Handover::MPC_go_to_approach_state(double approach_duration){
    const double dt = 0.05;   // 20 Hz control loop
    auto rate = rclcpp::Rate(20.0);

    int i = 0;

    while (rclcpp::ok()){
        double elapsed_time = i * dt;
        double remaining_time = approach_duration - elapsed_time;

        auto current_ee_pose=dual_arm_control_interface_->get_current_ee_pose("right");

        Eigen::Vector3d current_position(
            current_ee_pose->position.x,
            current_ee_pose->position.y,
            current_ee_pose->position.z
        );
        Eigen::Quaterniond current_orientation(
            current_ee_pose->orientation.w,
            current_ee_pose->orientation.x,
            current_ee_pose->orientation.y,
            current_ee_pose->orientation.z
        );

        Eigen::Vector3d current_velocity = ee_linear_velocity;
        Eigen::Vector3d target_position = via_point_position;
        Eigen::Vector3d target_velocity = via_point_linear_velocity;
        Eigen::Quaterniond target_orientation = via_point_orientation;


        Eigen::Vector3d velocity_setpoint = solve_mpc_velocity_setpoint(
                current_position,
                current_velocity,
                target_position,
                target_velocity,
                dt
        );

        Eigen::Vector3d position_error = target_position - current_position;

        Eigen::Vector3d bias_velocity = 1.2 * position_error;
        bias_velocity = clamp_norm(bias_velocity, 0.15);
            
        Eigen::Vector3d raw_velocity_setpoint = velocity_setpoint + bias_velocity;
        raw_velocity_setpoint = clamp_norm(raw_velocity_setpoint, 0.20);
            
        Eigen::Vector3d dv = raw_velocity_setpoint - previous_velocity_command;
        dv = clamp_norm(dv, 0.60 * dt);
            
        velocity_setpoint = previous_velocity_command + dv;
        velocity_setpoint = previous_velocity_command + dv;
        previous_velocity_command = velocity_setpoint;
        Eigen::Vector3d angular_velocity = get_angular_vel(target_orientation,current_orientation);
        set_velocity(k(velocity_setpoint), Eigen::AngleAxisd(angular_velocity.norm(),angular_velocity/angular_velocity.norm()));
        double distance_to_target = (target_position - current_position).norm();
        if (distance_to_target < 0.015){
            std::cout << "Reached target region" << std::endl;
            break;
        }
        rate.sleep();
        i++;
    }
}


void Handover::MPC_go_to_state(double approach_duration, Eigen::Vector3d final_position, Eigen::Vector3d final_velocity, Eigen::Quaterniond final_orientation){
    const double dt = 0.05;   // 20 Hz control loop
    auto rate = rclcpp::Rate(20.0);

    int i = 0;

    while (rclcpp::ok()){
        double elapsed_time = i * dt;
        double remaining_time = approach_duration - elapsed_time;

        // if (remaining_time <= 0.0){
        //     std::cout << "MPC_go_to_state timed out" << std::endl;
        //     set_velocity(Eigen::Vector3d::Zero(), Eigen::AngleAxisd());
        //     break;
        // }

        auto current_ee_pose=dual_arm_control_interface_->get_current_ee_pose("right");

        Eigen::Vector3d current_position(
            current_ee_pose->position.x,
            current_ee_pose->position.y,
            current_ee_pose->position.z
        );
        Eigen::Quaterniond current_orientation(
            current_ee_pose->orientation.w,
            current_ee_pose->orientation.x,
            current_ee_pose->orientation.y,
            current_ee_pose->orientation.z
        );

        Eigen::Vector3d current_velocity = ee_linear_velocity;
        Eigen::Vector3d target_position = final_position;
        Eigen::Vector3d target_velocity = final_velocity;
        Eigen::Quaterniond target_orientation = final_orientation;


        Eigen::Vector3d velocity_setpoint = solve_mpc_velocity_setpoint(
                current_position,
                current_velocity,
                target_position,
                target_velocity,
                dt
        );

        Eigen::Vector3d position_error = target_position - current_position;

        Eigen::Vector3d bias_velocity = 1.2 * position_error;
        bias_velocity = clamp_norm(bias_velocity, 0.15);
            
        Eigen::Vector3d raw_velocity_setpoint = velocity_setpoint + bias_velocity;
        raw_velocity_setpoint = clamp_norm(raw_velocity_setpoint, 0.20);
            
        Eigen::Vector3d dv = raw_velocity_setpoint - previous_velocity_command;
        dv = clamp_norm(dv, 0.60 * dt);
            
        velocity_setpoint = previous_velocity_command + dv;
        velocity_setpoint = previous_velocity_command + dv;
        previous_velocity_command = velocity_setpoint;
        Eigen::Vector3d angular_velocity = get_angular_vel(final_orientation,current_orientation);
        set_velocity(k(velocity_setpoint), Eigen::AngleAxisd(angular_velocity.norm(),angular_velocity/angular_velocity.norm()));
        double distance_to_target = (target_position - current_position).norm();
        if (distance_to_target < 0.015){
            std::cout << "Reached target region" << std::endl;
            break;
        }
        rate.sleep();
        i++;
    }
}

Eigen::Vector3d Handover::solve_mpc_velocity_setpoint(
    const Eigen::Vector3d& current_position,
    const Eigen::Vector3d& current_velocity,
    const Eigen::Vector3d& target_position,
    const Eigen::Vector3d& target_velocity,
    double dt
){
    const int N = 15;                 // MPC horizon
    const int max_iterations = 40;    // optimizer iterations
    const double max_velocity = 0.30;       // m/s
    const double max_acceleration = 0.60;   // m/s^2
    const double w_terminal_position = 80.0;
    const double w_terminal_velocity = 20.0;
    const double w_velocity_effort = 0.5;
    const double w_smoothness = 5.0;

    const double alpha = 0.015;       // gradient descent step size

    Eigen::Matrix<double, 3, Eigen::Dynamic> U(3, N);
    Eigen::Vector3d nominal_velocity=(target_position - current_position) / (N * dt);

    nominal_velocity = clamp_norm(nominal_velocity, max_velocity);

    for (int k = 0; k < N; k++){
        double blend = static_cast<double>(k) / static_cast<double>(N - 1);
        U.col(k) = (1.0 - blend) * nominal_velocity + blend * target_velocity;
        U.col(k) = clamp_norm(U.col(k), max_velocity);
    }

    U = project_velocity_constraints(U, current_velocity, dt);
    for (int iter = 0; iter < max_iterations; iter++){
        Eigen::Vector3d predicted_position = current_position;
        for (int k = 0; k < N; k++){
            predicted_position += U.col(k) * dt;
        }
        Eigen::Vector3d position_error =predicted_position - target_position;
        Eigen::Vector3d terminal_velocity_error =U.col(N - 1) - target_velocity;
        Eigen::Matrix<double, 3, Eigen::Dynamic> grad(3, N);
        grad.setZero();
        for (int k = 0; k < N; k++){
            grad.col(k) +=2.0 * w_terminal_position * dt * position_error;
            grad.col(k) +=2.0 * w_velocity_effort * U.col(k);
        }
        grad.col(N - 1) +=2.0 * w_terminal_velocity * terminal_velocity_error;

        Eigen::Vector3d first_diff = U.col(0) - current_velocity;
        grad.col(0) += 2.0 * w_smoothness * first_diff;

        for (int k = 1; k < N; k++){
            Eigen::Vector3d diff = U.col(k) - U.col(k - 1);
            grad.col(k) += 2.0 * w_smoothness * diff;
            grad.col(k - 1) -= 2.0 * w_smoothness * diff;
        }

        U = U - alpha * grad;

        for (int k = 0; k < N; k++){
            U.col(k) = clamp_norm(U.col(k), max_velocity);
        }

        U = project_velocity_constraints(U, current_velocity, dt);
    }
    return U.col(0);
}

Eigen::Vector3d Handover::clamp_norm(
    const Eigen::Vector3d& v,
    double max_norm
){
    double n = v.norm();
    if (n > max_norm && n > 1e-9){
        return v * (max_norm / n);
    }
    return v;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> Handover::project_velocity_constraints(
    const Eigen::Matrix<double, 3, Eigen::Dynamic>& U_in,
    const Eigen::Vector3d& previous_velocity,
    double dt
){
    Eigen::Matrix<double, 3, Eigen::Dynamic> U = U_in;
    const double max_velocity = 0.30;
    const double max_acceleration = 0.60;
    Eigen::Vector3d prev = previous_velocity;
    for (int k = 0; k < U.cols(); k++){
        Eigen::Vector3d dv = U.col(k) - prev;
        Eigen::Vector3d dv_limited =
            clamp_norm(dv, max_acceleration * dt);
        U.col(k) = prev + dv_limited;
        U.col(k) = clamp_norm(U.col(k), max_velocity);
        prev = U.col(k);
    }
    return U;
}


double Handover::f(double duration,int n, double delta_t){// duration over which the polynomial plans
    double remaining_duration = (duration -n*delta_t);
    return remaining_duration;
} 

double Handover::g(double delta_t){ // function over vel inference instant, if this increases, the controller will look ahead
    return delta_t;
}

Eigen::Vector3d Handover::k(Eigen::Vector3d vel_setpoint){ // function over velocity setpoint, output gain
    return vel_setpoint;
}