# Copyright 2023 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)
# @author Arash Sal Moslehian (arashsm79@yahoo.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_eddiebot_gazebo = get_package_share_directory(
        'eddiebot_gazebo')

    eddiebot_gz_sim_launch = PathJoinSubstitution(
        [pkg_eddiebot_gazebo, 'launch', 'eddiebot_gz_sim.launch.py'])

    eddiebot_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([eddiebot_gz_sim_launch]),
        launch_arguments={'world': 'maze_marked'}.items()
    )

    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        arguments=
            ['/kinect_rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/eddiebot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',],
        # output='screen'
    )

    navigation_node = Node(
        package="eddiebot_navigation",
        executable="eddiebot_navigation",
        name="eddiebot_navigation"
    )

    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/kinect_rgbd_camera/image'],
    )

    ld = LaunchDescription()
    ld.add_action(eddiebot_gz_sim)
    ld.add_action(bridges)
    ld.add_action(navigation_node)
    ld.add_action(rqt)

    return ld
