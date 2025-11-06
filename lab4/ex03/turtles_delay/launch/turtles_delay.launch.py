from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    delay = LaunchConfiguration('delay', default='5.0')
    return LaunchDescription([
        DeclareLaunchArgument('turtle_delay', default_value='5.0',
                              description='Delay for turtle'),                 

        Node(
            package='turtlesim', 
            executable='turtlesim_node', 
            name='sim',
        ), 
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 2.0, y: 2.0, theta: 0.0, name: "turtle2"}'],
            shell=False
        ),
        Node(
            package='turtlesim', 
            executable='turtle_teleop_key', 
            name='teleop', 
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='turtles_delay', 
            executable='turtle_tf_broadcaster',
            parameters=[{'turtlename': 'turtle1'}],
        ),
        Node(
            package='turtles_delay', 
            executable='turtle_tf_broadcaster',
            name='turtle2_broadcaster',
            parameters=[{'turtlename': 'turtle2'}],
        ),
        Node(
            package='turtles_delay', 
            executable='turtle_tf_listener',
            parameters=[{'delay': delay}],
        ),
    ])
