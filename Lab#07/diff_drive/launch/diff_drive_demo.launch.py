from launch_ros.actions import Node 
from launch.actions import (
    IncludeLaunchDescription
)
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_diff_drive = get_package_share_directory('diff_drive')

    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-r /home/zahra/ws_moveit2/src/diff_drive/models/diff_drive.sdf'}.items(),
        )
    )
    ld.add_action(
        Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        )
    )

    ld.add_action(
        Node(
            package="diff_drive",
            executable="diff_drive",
            name="diff_drive"
        )
    )



    return ld



