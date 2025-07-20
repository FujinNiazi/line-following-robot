# Line Following Robot
This package simualtes a turtlebot3 robot is gazebo using ROS 2 and OpenCV to follow any colored line or track.
<p float="demo">
  <img width="500" height="400" alt="spa" src="https://github.com/user-attachments/assets/424a3808-7438-4710-bb84-deba17f329be" />
  <img width="490" height="400" alt="monza" src="https://github.com/user-attachments/assets/5928fa39-cff3-4562-a5e0-c7f6b8acfcd1" />
</p>

## Working
In order to get the robot to follow the line, first a sample of the color of the line/track is taken using the robots camera feed. Then we threshold using the sample color instead of grayscale allowing the robot to follow a line of any color. Then we develop the contours and the center of mass to get the error. Using which we adjust the robots movement. The robot is also programmed to come to a stop once it reaches the start point.

A recent turning direction memory was also added in case the robot loses the line (when going too fast or when it reaches a blind sharp turn). If the line is lost the robot uses this direction memory to stop and keep turning in the direction it was intending to turn until it finds the line again. 

<p float="corner">
  <img width="500" height="400" alt="corner" src="https://github.com/user-attachments/assets/cda65102-98ee-4c58-8175-3cee8c752c52" />
  <img width="495" height="300" alt="corner_cam" src="https://github.com/user-attachments/assets/17b5c21a-1cb8-4a95-89ae-b4bdfce2909b" />
</p>

Whenever the robot loses the line it stops, checks for a new line first and if it detects a line/track like shape it samples a new color. Otherwise starts turning to find the line again.

Two tracks were simulated in gazebo. Spa in black and monza in blue.

<p float="tracks">
  <img width="500" height="400" alt="spa" src="https://github.com/user-attachments/assets/212238d5-c2c6-4f58-ad40-814b4f3259c5" />
  <img width="500" height="400" alt="monza" src="https://github.com/user-attachments/assets/807713c0-1baa-4432-b74a-ce5f48276ed5" />
</p>





## Requirements
- Ubuntu 22.04 (with WSL2 if on Windows)
- ROS 2 Humble
- Gazebo 11.10.2
- Python 3

## Installation
```
# Clone this repo into your ROS 2 workspace folder
cd ~/ros2_ws
git clone https://github.com/FujinNiazi/line_follower-robot

# Build the package
colcon build

# Source the environments
source /opt/ros/humble/setup.bash
source install/setup.bash

# Then any of the launch files can be directly run from the workspace e.g
ros2 launch line_follower line_following_spa.launch.py

# Or the directory can be switched to the package folder in order to run the script (which gives the option to select between the tracks)
cd ~/src/line_follower
python3 run_launcher.py
```
