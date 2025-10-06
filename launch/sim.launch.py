#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    pkg_robo1 = get_package_share_directory('robo1_haje')
    config_path = os.path.join(pkg_robo1, 'config')

    # ------------------------------
    # Launch arguments
    # ------------------------------
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_robo1, 'worlds', 'simple_trees.sdf'),
        description='World file for Ignition Gazebo'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation time'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description='Launch RViz2'
    )

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ------------------------------
    # 1. Launch Ignition Gazebo
    # ------------------------------
    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', world, '-r', '-v', '4'],
        output='screen'
    )

    # ------------------------------
    # 2. Process URDF + spawn drone
    # ------------------------------
    urdf_file = os.path.join(pkg_robo1, 'urdf', 'drone.urdf.xacro')
    robot_description = xacro.process_file(urdf_file).toxml()

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-name', 'drone', '-topic', 'robot_description']
    )

    # ------------------------------
    # 3. Publish TF + robot_description
    # ------------------------------
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }],
        output='screen'
    )

    # ------------------------------
    # 4. Bridge Gazebo <-> ROS2
    # ------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(config_path, 'gazebo_bridge.yaml'),
            'use_sim_time': True
        }],
        output='screen'
    )

    # ------------------------------
    # 5. EKF localization
    # ------------------------------
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_tf': True,
            'two_d_mode': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'frequency': 30.0,
            'odom0': '/odometry',
            'imu0': '/imu',
            'odom0_config': [True, True, True,
                            False, False, True,
                            False, False, False,
                            False, False, False, False, False, False],
            'imu0_config': [False, False, False,
                            True, True, True,
                            False, False, False,
                            False, False, False, False, False, False],
        }]
    )

    # ------------------------------
    # 6. SLAM Toolbox
    # ------------------------------
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': PathJoinSubstitution([FindPackageShare('robo1_haje'),
                                                     'config', 'slam_params.yaml'])
        }.items()
    )

    # ------------------------------
    # 7. RViz launch
    # ------------------------------
    rviz_config = os.path.join(pkg_robo1, 'config', 'drone.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))  # uses your existing flag
    )

    # ------------------------------
    # Return everything
    # ------------------------------
    return LaunchDescription([
        declare_world,
        declare_use_sim_time,
        declare_rviz,
        ign_gazebo,
        spawn_entity,
        state_publisher,
        bridge,
        ekf_node,
        slam_toolbox,
        rviz_node
    ])
