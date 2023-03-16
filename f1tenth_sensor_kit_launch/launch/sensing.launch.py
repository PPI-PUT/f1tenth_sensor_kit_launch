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
    # Lidar
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('f1tenth_sensor_kit_launch'), 'launch', 'lidar.launch.py'
            ]),
        ),
        launch_arguments={
            "container_name": LaunchConfiguration("container_name"),
            "use_laser_container": LaunchConfiguration("use_laser_container"),
            "use_multithread": LaunchConfiguration("use_multithread"),
            "use_intra_process": LaunchConfiguration("use_intra_process")
        }.items()
    )

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
            'input_vehicle_velocity_topic': "/vehicle/status/velocity_status",
            'output_twist_with_covariance': "/sensing/vehicle_velocity_converter/twist_with_covariance",
        }.items()
    )
    
    return [
        lidar_launch,
        imu_launch,
        vehicle_velocity_converter_launch,
    ]


def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)
    
    declared_arguments = []

    declared_arguments.append(add_launch_arg("container_name", "hokuyo_node_container"))
    declared_arguments.append(add_launch_arg("use_laser_container", "false"))
    declared_arguments.append(add_launch_arg("use_multithread", "false"))
    declared_arguments.append(add_launch_arg("use_intra_process", "false"))

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
