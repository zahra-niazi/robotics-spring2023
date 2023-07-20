from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),

        # Turtle1 
        Node(
            package='turtlemania',
            executable='turtlemania_tf2_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),

        # LISTENER 1
        # Turtle2 (Source frame) will be evaluated
        #
        # in the past => compute transform from Turtle1 to world
        #   in world frame --> tf2 time travels to now
        # at current time => compute transform from world to Turtle2
        Node(
            package='turtlemania',
            executable='turtlemania_tf2_listener',
            name='listener1',
            parameters=[
                {'target_frame':'turtle1'},
                {'source_frame' : 'turtle2'}
            ]
        ),
        Node(
            package='turtlemania',
            executable='turtlemania_tf2_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'turtle2'}
            ]
        ),

        # LISTENER 2
        # Turtle3 (Source frame) will be evaluated
        #
        # in the past => compute transform from Turtle2 to world
        #   in world frame --> tf2 time travels to now
        # at current time => compute transform from world to Turtle3
        Node(
            package='turtlemania',
            executable='turtlemania_tf2_listener',
            name='listener2',
            parameters=[
                {'target_frame':  'turtle2'},
                {'source_frame' : 'turtle3'}
            ]
        ),        
        Node(
            package='turtlemania',
            executable='turtlemania_tf2_broadcaster',
            name='broadcaster3',
            parameters=[
                {'turtlename': 'turtle3'}
            ]
        ),

        # LISTENER 3
        # Turtle4 (Source frame) will be evaluated
        #
        # in the past => compute transform from Turtle3 to world
        #   in world frame --> tf2 time travels to now
        # at current time => compute transform from world to Turtle4
        Node(
            package='turtlemania',
            executable='turtlemania_tf2_listener',
            name='listener3',
            parameters=[
                {'target_frame': 'turtle3'},
                {'source_frame' : 'turtle4'}
            ]
        ),        
        Node(
            package='turtlemania',
            executable='turtlemania_tf2_broadcaster',
            name='broadcaster4',
            parameters=[
                {'turtlename': 'turtle4'}
            ]
        ),
    ])