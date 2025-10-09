# launch/movement.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robo1_haje',      # your package name
            executable='movement_node',         # console_scripts entry point
            name='movement_controller',
            output='screen',
            parameters=[{
                'drone_name': 'drone',
                # Or set explicit topics if different in your bridge config:
                # 'cmd_vel_topic': '/model/drone/cmd_vel',
                # 'odom_topic':    '/model/drone/odometry',
                'xy_tol': 0.35,
                'z_tol':  0.25
            }]
        )
    ])
