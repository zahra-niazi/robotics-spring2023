from launch_ros.actions import Node 
from launch.actions import (
    IncludeLaunchDescription
)
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_diff_drive = get_package_share_directory('diff_drive')

    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r /home/zahra/ws_moveit2/src/diff_drive/models/maze.sdf'}.items(),
        )
    )
    ld.add_action(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'vehicle_blue/chassis/gpu_lidar']
        )
    )

    ld.add_action(
        Node(
        package='rviz2',
        executable='rviz2',   
        # arguments=['-d', os.path.join(pkg_diff_drive, 'rviz', 'diff_drive_lidar.rviz')],
        )
    )

    ld.add_action(
        Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        'lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        )
    )

    # ld.add_action(
    #     Node(
    #         package="diff_drive",
    #         executable="diff_drive",
    #         name="diff_drive"
    #     )
    # )



    return ld



