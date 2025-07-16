# Robot Programming Assignment - CMP9767



![License](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)
![Course](https://img.shields.io/badge/Course-CMP9767-purple?style=for-the-badge)
![ROS](https://img.shields.io/badge/ROS-HUMBLE-blue?style=for-the-badge&logo=ROS)
[![VERSION](https://img.shields.io/badge/VERSION-0.1.0-COLOR.svg?style=for-the-badge&logo=LOGO)](<LINK>)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen?style=for-the-badge)
![Python](https://img.shields.io/badge/Script-python-blue?style=for-the-badge&logo=python)
![Container Size](https://img.shields.io/badge/Container%20Size->2GB-blue?style=for-the-badge&logo=docker)
![Flask](https://img.shields.io/badge/flask-%23000.svg?style=for-the-badge&logo=flask&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![HTML5](https://img.shields.io/badge/html5-%23E34F26.svg?style=for-the-badge&logo=html5&logoColor=white)
![JavaScript](https://img.shields.io/badge/javascript-%23323330.svg?style=for-the-badge&logo=javascript&logoColor=%23F7DF1E)
![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)
![Bootstrap](https://img.shields.io/badge/bootstrap-%238511FA.svg?style=for-the-badge&logo=bootstrap&logoColor=white)
![Edge](https://img.shields.io/badge/Edge-0078D7?style=for-the-badge&logo=Microsoft-edge&logoColor=white)
![RVIZ](https://img.shields.io/badge/RViz-0078D7?style=for-the-badge&logo=RViz&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-0078D7?style=for-the-badge&logo=Gazebo&logoColor=white)
![Robotics](https://img.shields.io/badge/Robotics-0078D7?style=for-the-badge&logo=Robotics&logoColor=white)
![AI](https://img.shields.io/badge/AI-0078D7?style=for-the-badge&logo=AI&logoColor=white)
 

## Steps to Run the Program


### Step 1: Build the workspace

Unzip or clone the [project file](https://github.com/Nditah/CMP9767-27455742)

Ensure that your Docker service is running, then open the project folder with VS Code.

VS Code will propmt you to reopen as a container. Accept and watch the logs on terminal

In the root of the workspace, run colcon build. 
This allows the installed files to be changed by changing the files in the source space for faster iteration.

```bash
# Install dependencies
pip install flask

sudo apt install ros-humble-nav2-* -y


bash: pytest

colcon build --symlink-install

colcon test-result --verbose

source install/setup.bash

# Alternatively, source it once and for all

echo "source ./install/setup.bash" >> ~/.bashrc
```


### Step 2: Show Dashboard / Control Panel

```bash

# Terminal 2: start web_server_node
ros2 run robot_assignment web_server

```

### Step 3: Launch Gazebo

For limo simulator with a custom world

```bash
ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/robot_assignment/worlds/custom_world.world
```


### Step 4: Inspect Sensors with  rviz2
Inspect the simulated robot and its sensors by running rviz2 in another terminal window

```bash
rviz2 -d /opt/ros/lcas/src/limo_ros2/src/limo_gazebosim/rviz/urdf.rviz
```


### Step 5: Launch the navigation
Launch the navigation stack by invoking the following command

```bash
ros2 launch limo_navigation limo_navigation.launch.py
```


### Step 6: Run the counter node
Run the counter node to prints out a full list of all objects in the terminal.

```bash
ros2 run robot_assignment counter_3d

ros2 run robot_assignment camera_classifier
```

### Step 7: Object detection in 3D
 The node outputs the detected objects in 3d as the `/limo/object_location` topic.

```bash
ros2 run robot_assignment color_3d_detection
```


### Step 8: BasicNavigator approach
 It contains a list of user defined waypoints that the robot will follow using a BasicNavigator approach.

```bash
ros2 run robot_assignment demo_inspection
```

### Debug

Teleoperation. Leave the RVIZ running. In a new terminal, start the keyboard teleoperation node

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

View Topics
```bash
ros2 topic echo /amcl_pose > output.txt
```

## Credit & References

Usefull resources:

- **CMP9767 Workshop**: Riccardo Polvara, Grzegor Cielniak [Wiki](https://github.com/LCAS/CMP9767/wiki).
- **ROS2 Humble**: ROS2 Humble Documentation [Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- **OpenCV with ROS2**: [Getting Started With OpenCV in ROS 2 Foxy Fitzroy (Python)](https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/)
- **ROS2 Nav Stack**: [Nav2 Documentation](https://docs.nav2.org/)
- **Python Flask**: [Flask Documentation](https://flask.palletsprojects.com/)
- **HTML5 Bootstrap**: [Bootstrap Documentation](https://getbootstrap.com/docs/5.3/getting-started/introduction/)
