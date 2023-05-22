# Copyright 2023 Amadeusz Szymko
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # IMU
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('f1tenth_sensor_kit_launch'), 'launch', 'imu.launch.py'
            ]),
        )
    )

    # Vehicle Velocity Converter
    vehicle_velocity_converter_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('vehicle_velocity_converter'), 'launch', 'vehicle_velocity_converter.launch.xml'
            ]),
        ),
        launch_arguments={
            'input_vehicle_velocity_topic': '/vehicle/status/velocity_status',
            'output_twist_with_covariance': '/sensing/vehicle_velocity_converter/twist_with_covariance',
        }.items()
    )

    # Remote controller
    remote_controller_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('joy_controller'), 'launch', 'joy_controller.launch.xml'
            ]),
        ),
        launch_arguments={
            'config_file': PathJoinSubstitution([
                FindPackageShare('f1tenth_sensor_kit_launch'), 'config', 'joy/joy_controller.param.yaml'
            ]),
            'input_joy': '/sensing/joy',
            'joy_type': 'P65',
            'input_odometry': '/localization/kinematic_state',
        }.items()
    )

    return [
        imu_launch,
        vehicle_velocity_converter_launch,
        remote_controller_launch,
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
