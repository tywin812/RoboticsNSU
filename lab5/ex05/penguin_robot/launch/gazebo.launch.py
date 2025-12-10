import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    pkg_share = get_package_share_directory('penguin_robot')
    urdf_file = os.path.join(pkg_share, 'src', 'description', 'penguin_bot_description.urdf')
    fish_urdf_file = os.path.join(pkg_share, 'src', 'description', 'fish.urdf')

    robot_description_content = Command(['xacro ', urdf_file])
    fish_description_content = Command(['xacro ', fish_urdf_file])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )

    fish_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='fish_state_publisher',
        output='screen',
        parameters=[{'robot_description': fish_description_content, 'use_sim_time': True}],
        namespace='fish'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_entity = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/robot_description',
                        '-name', 'penguin_bot',
                        '-z', '0.1'],
                output='screen'
            ),
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/fish/robot_description',
                        '-name', 'fish_bot',
                        '-x', '1.0',
                        '-z', '0.01'],
                output='screen'
            )
        ]
    )

    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/fish/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/fish/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fish_random_drive = Node(
        package='penguin_robot',
        executable='random_fish_movement.py',
        name='random_fish_movement',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    fish_tf_broadcaster = Node(
        package='penguin_robot',
        executable='fish_tf_broadcaster.py',
        name='fish_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    penguin_tf_listener = Node(
        package='penguin_robot',
        executable='penguin_tf_listener.py',
        name='penguin_tf_listener',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        robot_state_publisher,
        fish_state_publisher,
        gazebo,
        spawn_entity,
        bridge_cmd_vel,
        bridge_odom,
        bridge_joint_states,
        # circle_movement,
        bridge_clock,
        fish_random_drive,
        fish_tf_broadcaster,
        penguin_tf_listener,
    ])
