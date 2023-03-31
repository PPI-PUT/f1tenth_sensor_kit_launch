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
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):    
    #TODO: add urg_node for non-simulator mode
    
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_laser_container")),
        output="screen",
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_laser_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    component_loader = LoadComposableNodes(
        composable_node_descriptions=[],
        target_container=target_container,
    )

    return [
        container,
        component_loader
    ]


def generate_launch_description():
    declared_arguments = []
    
    def add_launch_arg(name: str, default_value=None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )
    
    add_launch_arg("container_name", "hokuyo_node_container")
    add_launch_arg("use_laser_container", "false")
    add_launch_arg("use_multithread", "false")
    add_launch_arg("use_intra_process", "false")
    add_launch_arg("input/pointcloud", "/sensing/lidar/pointcloud")
    add_launch_arg("output/laserscan", "/sensing/lidar/scan")
    add_launch_arg("output/pointcloud", "/sensing/lidar/debug/pointcloud")
    add_launch_arg("output/ray", "/sensing/lidar/debug/ray")
    add_launch_arg("output/stixel", "/sensing/lidar/debug/stixel")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return LaunchDescription([
        *declared_arguments,
        set_container_executable,
        set_container_mt_executable,
        OpaqueFunction(function=launch_setup)
    ])
