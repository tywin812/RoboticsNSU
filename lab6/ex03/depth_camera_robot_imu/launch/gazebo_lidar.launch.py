import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    pkg_share = get_package_share_directory('depth_camera_robot')
    urdf_file = os.path.join(pkg_share, 'src', 'description', 'penguin_bot_description.urdf')

    robot_description_content = Command(['xacro ', urdf_file])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}]
    )    

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r gpu_lidar_sensor.sdf'
        }.items(),
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/robot_description',
                          '-name', 'penguin_bot',
                          '-x', '2.0',
                          '-z', '0.1'],
                output='screen'
            ),
        ]
    )
    
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
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
        arguments=['clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': False}], 
        output='screen'
    )

    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    bridge_depth_image = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[('/depth_camera', '/depth_camera/depth_image')],
        output='screen'
    )

    bridge_depth_points = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'  
        ],
        parameters=[{'use_sim_time': True}], 
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,    
        bridge_cmd_vel,
        bridge_odom,
        bridge_joint_states,
        bridge_clock,        
        bridge_camera_info,
        bridge_depth_image,
        bridge_depth_points,
        bridge_imu
    ])
