from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import  (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

ARGUMENTS = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('world', default_value='maze',
                              description='Eddiebot World'),

        ]

def generate_launch_description():
    pkg_eddiebot_navigation = get_package_share_directory('eddiebot_navigation')
    pkg_eddiebot_gazebo = get_package_share_directory('eddiebot_gazebo')

    eddiebot_gz_sim_launch = PathJoinSubstitution(
        [pkg_eddiebot_gazebo, 'launch', 'eddiebot_gz_sim.launch.py'])
    rviz2_config = PathJoinSubstitution(
        [pkg_eddiebot_navigation, 'rviz', 'model.rviz'])




    bridges = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=
            ['/kinect_rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/kinect_rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/model/eddiebot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        remappings=[('/model/eddiebot/tf', 'tf'),]
    )


    d2l_param = PathJoinSubstitution([(get_package_share_directory('depthimage_to_laserscan')), 'cfg', 'param.yaml'])
    dimage_to_lscan_node = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan_node",
        remappings=[('depth','/kinect_rgbd_camera/depth_image'),
                    ( 'depth_camera_info','/kinect_rgbd_camera/camera_info')],
        parameters=[d2l_param]
    )



    static_t_pub1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'eddiebot/base_footprint', '--child-frame-id', 'base_footprint'],
    )
    static_t_pub2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'world', '--child-frame-id', 'eddiebot/odom'],
    )    
    static_t_pub3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'camera_link', '--child-frame-id', 'camera_depth_frame'],
    )





    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/kinect_rgbd_camera/image'],
    )





    rviz = Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', rviz2_config],
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
             remappings=[ ('/tf', 'tf'),('/tf_static', 'tf_static')],
             output='screen'
        )

    gz_sim = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([eddiebot_gz_sim_launch]),
                    launch_arguments=[('world', LaunchConfiguration('world')),
                        ('use_sim_time', LaunchConfiguration('use_sim_time'))])
        

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz)
    ld.add_action(gz_sim)
    ld.add_action(bridges)
    ld.add_action(dimage_to_lscan_node)

    ld.add_action(static_t_pub1)
    ld.add_action(static_t_pub2)
    ld.add_action(static_t_pub3)
    # ld.add_action(rqt)


    return ld
