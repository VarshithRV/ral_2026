collect_data(){
ros2 bag record \
object_linear_encoder/float_array \
area_sensor/float_array \
-o expt_$1
}
