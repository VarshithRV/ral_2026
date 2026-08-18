from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

import os


def launch_setup(context, *args, **kwargs):
    serial_float_publisher1 = Node(
        package="ral_2026",
        executable="float_array_character_device_driver",
        name="object_linear_encoder",
        output="screen",
        parameters=[
            {
                "device_name": "/dev/ttyACM0",
                "channels": 2,
                "baud_rate": 9600,
            }
        ],
    )

    serial_float_publisher2 = Node(
            package="ral_2026",
            executable="float_array_character_device_driver",
            name="area_sensor",
            output="screen",
            parameters=[
                {
                    "device_name": "/dev/ttyACM1",
                    "channels": 8,
                    "baud_rate": 115200,
                }
            ],
        )

    nodes_to_start = [
        serial_float_publisher1,
        serial_float_publisher2,
    ]
    
    return nodes_to_start


def generate_launch_description():
    
    declared_arguments = []
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])