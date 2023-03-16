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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare("f1tenth_sensor_kit_launch")
    imu_corrector_config = PathJoinSubstitution([pkg_prefix, "config/imu",
                                                 LaunchConfiguration('imu_corrector_param_file')])
    
    imu_corrector_node = Node(
        name='imu_corrector',
        namespace='',
        package='imu_corrector',
        executable='imu_corrector',
        parameters=[
                imu_corrector_config
        ],
        remappings=[
                ("input", "vesc/imu_raw"),
                ("output", "vesc/imu")
        ],
        output='screen',
    )

    return [
        imu_corrector_node
    ]


def generate_launch_description():
    declared_arguments = []
    
    def add_launch_arg(name: str, default_value=None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )
    
    add_launch_arg("imu_corrector_param_file", "imu_corrector.param.yaml")

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
