from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch, generate_moveit_rviz_launch
from launch_ros.actions import Node 
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess
)
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    ws3 = get_package_share_directory('ws3')
    ld = LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
                    launch_arguments={'gz_args': '-r default.sdf'}.items(),
            ),
        )
    
    xacro_file = ("src/fumti/urdf/model.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description' : doc.toxml()}

    ld.add_action(
        Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        output='screen',
        respawn=True,
        parameters=[params]
        )
    )
    ld.add_action(
        Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/camera'],
        )
    )
    ld.add_action(
        ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configured',  'joint_state_broadcaster'], 
        output='screen'
        )
    )
    ld.add_action(
        ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'configured',  'robot_controller'], 

        )
    )
    ld.add_action(
        Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(ws3, 'config', 'camera.rviz')],
    )
    )

    ld.add_action(
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=[
                # {"world": "empty.sdf"},
                "-file", "src/fumti/urdf/model.urdf.xacro"
            ],
        )
    )
    ld.add_action(
        Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
        )
    )


    return ld



