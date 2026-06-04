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

    left_gripper_off();

    std::cout<<"preparing servo"<<std::endl;
    ee_servo_handle_->prepare_servo_();
    std::cout<<"starting servo"<< std::endl;
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
    left_gripper_on();
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
    left_handover_via_point_pose.position.y = 0.05;
    left_handover_via_point_pose.position.z = 0.218;
    left_handover_via_point_pose.orientation.w = 0.7071;
    left_handover_via_point_pose.orientation.x = 0.0;
    left_handover_via_point_pose.orientation.y = 0.7071;
    left_handover_via_point_pose.orientation.z = 0.0;
    
    left_handover_pose.position.x = -0.051;
    left_handover_pose.position.y = 0.0;
    left_handover_pose.position.z = 0.218;
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
    std::this_thread::sleep_for(0.5s);
    right_gripper_off();
    std::this_thread::sleep_for(0.5s);

    // right +z 5cm and back to preaction, left move in -x 
    auto right_arm_current_pose = *dual_arm_control_interface_->get_current_ee_pose("right");
    auto left_arm_current_pose = *dual_arm_control_interface_->get_current_ee_pose("left");

    auto right_arm_after_robot_handover(right_arm_current_pose);
    right_arm_after_robot_handover.position.z +=0.05;
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
    left_after_handover_2.position.z -= 0.05;

    left_after_handover_3_place = left_after_handover_2;
    left_after_handover_3_place.position.z = 0.15;

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
    left_after_placing_1.position.y += 0.1;
    left_after_placing_1.position.z += 0.05;

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

void Handover::left_gripper_on(){
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

void Handover::left_gripper_off(){
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

void Handover::left_gripper_neutral(){
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



void Handover::right_gripper_on(){
    auto gripper_on_msg1 = std::make_shared<ur_msgs::srv::SetIO::Request>();
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT2;
    gripper_on_msg1->state = gripper_on_msg1->STATE_ON;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT3;
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
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT2;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT3;
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
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT2;
    gripper_on_msg1->state = gripper_on_msg1->STATE_OFF;
    dual_arm_control_interface_->right_set_io_client_->async_send_request(gripper_on_msg1);
    gripper_on_msg1->fun = gripper_on_msg1->FUN_SET_DIGITAL_OUT;
    gripper_on_msg1->pin = gripper_on_msg1->PIN_CONF_OUT3;
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