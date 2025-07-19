from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory

import os

# Get shared directory of the package
pkg_share = get_package_share_directory('line_follower')

# Set Gazebo environment variables
os.environ["GAZEBO_MODEL_DATABASE_URI"] = ""
os.environ["GAZEBO_MODEL_PATH"] = os.path.join(pkg_share, 'models')


def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with your world
        ExecuteProcess(
            cmd=[
                'gzserver', '--verbose',
                os.path.join(pkg_share, 'worlds', 'line_following_monza.world'),
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Optional GUI
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
        
        # Launch rqt_image_view for camera
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
                    output='screen'
                )
            ]
        ),

        # Spawn the robot after a short delay
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'my_robot',
                        '-file', os.path.join(pkg_share, 'models', 'turtlebot3_waffle_pi', 'model.sdf'),
                        '-x', '3', '-y', '1', '-z', '0.1',
                        '-R', '0', '-P', '0', '-Y', '0.25'
                    ],
                    output='screen'
                )
            ]
        ),

        # Start your line follower node (update name if needed)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='line_follower',
                    executable='line_follower_node',  # update this to your actual node filename (without .py)
                    name='line_follower',
                    output='screen'
                )
            ]
        )
    ])
