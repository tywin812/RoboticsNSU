from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    radius = LaunchConfiguration('radius', default='2.0')
    direction = LaunchConfiguration('direction_of_rotation', default='1')

    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='2.0',
                              description='Radius of carrot rotation'),
        DeclareLaunchArgument('direction_of_rotation', default_value='1',
                              description='Rotation direction: 1 (CW) or -1 (CCW)'),

        Node(
            package='turtlesim', 
            executable='turtlesim_node', 
            name='sim',
        ),
        Node(
            package='turtlesim', 
            executable='turtle_teleop_key', 
            name='teleop', 
            output='screen',
            prefix='xterm -e',
        ),

        Node(
            package='turtle_tf2_carrot', 
            executable='turtle1_tf_broadcaster',
            parameters=[{'turtlename': 'turtle1'}],
        ),
        Node(
            package='turtle_tf2_carrot', 
            executable='turtle1_tf_broadcaster',
            parameters=[{'turtlename': 'turtle2'}],
        ),
        Node(
            package='turtle_tf2_carrot', 
            executable='carrot_tf_broadcaster',
            parameters=[{'radius': radius, 'direction_of_rotation': direction}],
        ),
        Node(
            package='turtle_tf2_carrot', 
            executable='turtle2_tf_listener',
        ),
    ])
