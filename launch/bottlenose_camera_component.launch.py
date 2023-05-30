"""
******************************************************************************
*  Copyright 2023 Labforge Inc.                                              *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License")            *
* you may not use this project except in compliance with the License.        *
* You may obtain a copy of the License at                                    *
*                                                                            *
*     http://www.apache.org/licenses/LICENSE-2.0                             *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
******************************************************************************
"""
__author__ = "Thomas Reidemeister <thomas@labforge.ca>"
__copyright__ = "Copyright 2023, Labforge Inc."

import launch
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import launch_ros.actions

NAMESPACE = '/camera'


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name="bottlenose_camera_driver_container",
            package='rclcpp_components',
            namespace=NAMESPACE,
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='bottlenose_camera_driver',
                    plugin='bottlenose_camera_driver::CameraDriver',
                    name='bottlenose_camera_driver_node',
                    parameters=[
                        
                    ])
            ],
            output='screen'
        )
    ])
