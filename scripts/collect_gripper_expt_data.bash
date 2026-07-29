collect_data(){
ros2 bag record \
tf \
tf_static \
joint_states \
apriltag_grid_detector/object0_filtered_pose \
human_to_robot_handover_2/grasp_pose \
object_linear_encoder/float_array \
area_sensor/float_array \
-o expt_$1
}
