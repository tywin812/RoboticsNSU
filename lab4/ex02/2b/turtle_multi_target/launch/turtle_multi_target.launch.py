from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    radius1 = LaunchConfiguration('carrot1_radius', default='2.0')
    direction1 = LaunchConfiguration('carrot1_direction', default='1')
    radius2 = LaunchConfiguration('carrot2_radius', default='2.0')
    direction2 = LaunchConfiguration('carrot2_direction', default='1')
    return LaunchDescription([
        DeclareLaunchArgument('carrot1_radius', default_value='2.0',
                              description='Radius of carrot1 rotation'),
        DeclareLaunchArgument('carrot1_direction', default_value='1',
                              description='Rotation direction1 : 1 (CW) or -1 (CCW)'),
        DeclareLaunchArgument('carrot2_radius', default_value='2.0',
                              description='Radius of carrot2 rotation'),
        DeclareLaunchArgument('carrot2_direction', default_value='1',
                              description='Rotation direction2 : 1 (CW) or -1 (CCW)'),                      

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
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn',
                 '{x: 8.0, y: 8.0, theta: 0.0, name: "turtle3"}'],
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
            package='turtle_multi_target', 
            executable='turtle_tf_broadcaster',
            parameters=[{'turtlename': 'turtle1'}],
        ),
        Node(
            package='turtle_multi_target', 
            executable='turtle_tf_broadcaster',
            parameters=[{'turtlename': 'turtle2'}],
        ),
        Node(
            package='turtle_multi_target', 
            executable='turtle_tf_broadcaster',
            parameters=[{'turtlename': 'turtle3'}],
        ),
        Node(
            package='turtle_multi_target', 
            executable='target_switcher',
            parameters=[{'carrot1_radius': radius1, 'carrot1_direction': direction1, 'carrot2_radius' : radius2, 'carrot2_direction': direction2}],
        ),
        Node(
            package='turtle_multi_target', 
            executable='turtle_controller',
        ),
    ])
