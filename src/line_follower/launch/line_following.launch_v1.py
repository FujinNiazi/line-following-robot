from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

import os
os.environ["GAZEBO_MODEL_DATABASE_URI"] = ""
os.environ["GAZEBO_MODEL_PATH"] = "/mnt/d/Work/Coding/Navigation/line_follower/src/line_follower/models"


def generate_launch_description():
    return LaunchDescription([
        # Start Gazebo with your world (optional)
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '/mnt/d/Work/Coding/Navigation/line_follower/src/line_follower/worlds/line_following.world', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
         ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # Spawn your robot
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'my_robot',
                        '-file', '/mnt/d/Work/Coding/Navigation/line_follower/src/line_follower/models/turtlebot3_waffle_pi/model.sdf',
                        '-x', '4', '-y', '-2', '-z', '0.1',
                        '-R', '0', '-P', '0', '-Y', '1.57'
                    ],
                    output='screen'
                )
            ]
        ) 
    ])
