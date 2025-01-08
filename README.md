# Robot Programming Assignment


## Steps to Run the Program


### Step 1: Build the workspace

Unzip or clone the [project file](https://github.com/Nditah/CMP9767-27455742)

Ensure that your Docker service is running, then open the project folder with VS Code.

VS Code will propmt you to reopen as a container. Accept and watch the logs on terminal

In the root of the workspace, run colcon build. 
This allows the installed files to be changed by changing the files in the source space for faster iteration.

```bash
colcon build --symlink-install
```



### Step 2: Source the environment

When colcon has completed building successfully, the output will be in the install directory. Source them to add all of the required elements to your path and library paths as well as provide any bash or shell commands exported by packages.

```bash
source install/setup.bash
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

### Step 6: Object detection in 3D
 The node outputs the detected objects in 3d as the `/limo/object_location` topic.

```bash
ros2 run robot_assignment detector_3d
```


### Step 7: BasicNavigator approach
 It contains a list of user defined waypoints that the robot will follow using a BasicNavigator approach.

```bash
ros2 run robot_assignment demo_inspection
```



### Step 8
Run the counter node by issuing ros2 run cmp9767_tutorial counter_3d. The node prints out a full list of all objects in the terminal.

```bash
ros2 run robot_assignment counter_3d
```


### Step 9 
Show count on a web page

```bash

```

## References


CMP9767 Workshop Riccardo Polvara, Grzegor Cielniak [Wiki](https://github.com/LCAS/CMP9767/wiki).

