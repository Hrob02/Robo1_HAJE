# launch/movement.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='haje_drone_movement',      # your package name
            executable='movement_node',         # console_scripts entry point
            name='movement_controller',
            output='screen',
            parameters=[{
                'drone_name': 'parrot',
                # Or set explicit topics if different in your bridge config:
                # 'cmd_vel_topic': '/model/parrot/cmd_vel',
                # 'odom_topic':    '/model/parrot/odometry',
                'xy_tol': 0.35,
                'z_tol':  0.25
            }]
        )
    ])
