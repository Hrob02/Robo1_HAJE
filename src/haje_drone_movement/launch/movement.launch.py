# launch/movement.launch.py
#from launch import LaunchDescription
#from launch_ros.actions import Node
#
#def generate_launch_description():
#    return LaunchDescription([
#       Node(
 #           package='haje_drone_movement',      # your package name
  #          executable='movement_node',         # console_scripts entry point
   #         name='movement_controller',
    #        output='screen',
     #       parameters=[{
      #          'drone_name': 'parrot',
          #      # Or set explicit topics if different in your bridge config:
          #      # 'cmd_vel_topic': '/model/parrot/cmd_vel',
           #     # 'odom_topic':    '/model/parrot/odometry',
         #       'xy_tol': 0.35,
#                'z_tol':  0.25
#            }]
#        )
#    ])


#from launch import LaunchDescription
#from launch_ros.actions import Node

#def generate_launch_description():
#    return LaunchDescription([
#        Node(
 #           package='haje_drone_movement',
#            executable='movement_node',     # console_scripts entry point
#            name='movement_controller',
#            output='screen',
#            parameters=[{
#                # publish to the topic that actually moves your drone:
#                'cmd_vel_topic': '/cmd_vel',
#                # if your node reads odom, set it to the sim’s odom topic:
#                'odom_topic': '/model/parrot/odometry',  # change if yours differs
#                'xy_tol': 0.35,
#                'z_tol':  0.25
#            }]
#        )
#    ])
#from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration
#from launch_ros.actions import Node


#def generate_launch_description():
#    cmd_vel_topic   = LaunchConfiguration('cmd_vel_topic')
#    odom_topic      = LaunchConfiguration('odom_topic')
#    skip_takeoff    = LaunchConfiguration('skip_takeoff')
#    use_z_control   = LaunchConfiguration('use_z_control')
#    goal_source     = LaunchConfiguration('goal_source')
#    optimise_path   = LaunchConfiguration('optimise_path')

#    return LaunchDescription([
#        # ---- Defaults that match your sim wiring ----
#        DeclareLaunchArgument('cmd_vel_topic',   default_value='/cmd_vel'),
#        DeclareLaunchArgument('odom_topic',      default_value='/odometry/filtered'),
#
#        # ---- Behaviour switches (easy to override from CLI) ----
#        DeclareLaunchArgument('skip_takeoff',    default_value='true'),
#        DeclareLaunchArgument('use_z_control',   default_value='false'),
#        DeclareLaunchArgument('goal_source',     default_value='param'),  # 'param' or 'gui'
#        DeclareLaunchArgument('optimise_path',   default_value='false'),

#        Node(
#            package='haje_drone_movement',
#            executable='movement_node',
#            name='movement_controller',
#            output='screen',
#            parameters=[{
#                'cmd_vel_topic':   cmd_vel_topic,
#                'odom_topic':      odom_topic,
#                'skip_takeoff':    skip_takeoff,
#                'use_z_control':   use_z_control,
#                'goal_source':     goal_source,
#                'optimise_path':   optimise_path,
#                # (xy_tol, z_tol, etc. will use node defaults unless you override)
#            }]
#        )
#    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cmd_vel_topic   = LaunchConfiguration("cmd_vel_topic")
    odom_topic      = LaunchConfiguration("odom_topic")
    skip_takeoff    = LaunchConfiguration("skip_takeoff")   # <-- default now false
    use_z_control   = LaunchConfiguration("use_z_control")
    yaw_to_path     = LaunchConfiguration("yaw_to_path")
    lookahead_m     = LaunchConfiguration("lookahead_m")
    launch_altitude = LaunchConfiguration("launch_altitude")
    land_touchdown  = LaunchConfiguration("land_touchdown_z")
    params_file     = LaunchConfiguration("params_file")

    default_params = os.path.join(
        get_package_share_directory("haje_drone_movement"),
        "config", "movement_params.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument("cmd_vel_topic",   default_value="/cmd_vel"),
        DeclareLaunchArgument("odom_topic",      default_value="/odometry"),

        # changed from "true" -> "false" to skip z
        DeclareLaunchArgument("skip_takeoff",    default_value="false"),

        DeclareLaunchArgument("use_z_control",   default_value="false"),
        DeclareLaunchArgument("yaw_to_path",     default_value="true"),
        DeclareLaunchArgument("lookahead_m",     default_value="0.0"),
        DeclareLaunchArgument("launch_altitude", default_value="2.0"),
        DeclareLaunchArgument("land_touchdown_z",default_value="0.1"),
        DeclareLaunchArgument("params_file",     default_value=default_params),

        Node(
            package="haje_drone_movement",
            executable="movement_node",
            name="movement_controller",
            output="screen",
            parameters=[
                ParameterFile(params_file, allow_substs=True),
                {
                    "cmd_vel_topic":   cmd_vel_topic,
                    "odom_topic":      odom_topic,
                    "skip_takeoff":    skip_takeoff,
                    "use_z_control":   use_z_control,
                    "yaw_to_path":     yaw_to_path,
                    "lookahead_m":     lookahead_m,
                    "launch_altitude": launch_altitude,
                    "land_touchdown_z": land_touchdown,
                },
            ],
        ),
    ])

