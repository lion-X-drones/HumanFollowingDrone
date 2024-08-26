# HumanFollowingDrone

It maintains a fixed distance from an individual. It follows the individual around and can help in monitoring and as company for personnels in dangerous environment to keep track of their health status. 

The drone system is composed of the camera, the companion computer (jetson nano), and the flight controller (pixhawk). The camera was only used to capture a live video stream and send image bytes of the snapshots to the companion computer. Image processing is obtained by using Computer Vision (OpenCV) libraries and through local context for face detection. It is also capable of detecting humans farther in distance compared to the rapid object detection introduced by Viola et al., thus human following through drones is exceptionally safer with local context because of its capability to detect and operate in farther distance.

Code and simulation are given

## Prerequisites for simulation
- **ROS Noetic (Ubuntu 20.04)**: Robot Operating System (ROS or ros) is an [open-source](https://en.wikipedia.org/wiki/Open-source_software) [robotics middleware](https://en.wikipedia.org/wiki/Robotics_middleware) suite. Although ROS is not an operating system (OS) but a set of software frameworks for robot software development. ROS Noetic Ninjemys is primarily targeted at the Ubuntu 20.04 (Focal) release, though other systems are supported to varying degrees. Note that you should install ROS Noetic Full Desktop version.

- **Ardupilot**- The ArduPilot software suite consists of navigation software (typically referred to as firmware when it is compiled to binary form for microcontroller hardware targets) running on the vehicle (either Copter, Plane, Rover, AntennaTracker, or Sub), along with ground station controlling software including Mission Planner, APM Planner, QGroundControl, MavProxy, Tower and others.

- You also need to install plugins which you can refer from [ardupilot-installation](https://github.com/Bhaveshmeghwal21/AMC_Summer_Camp-2024/blob/main/Intermediate/ROS/Ardupilot-installation.md)

## Running the simulation
### Gazebo-ROS
Open one terminal and launch ROS integrated gazebo
```bash
#Make sure you have all the right environment, if you are not sure run the following first

source /opt/ros/noetic/setup.bash

export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=~/ardupilot_gazebo_roscam/src/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-9/plugins:$GAZEBO_PLUGIN_PATH 
export GAZEBO_PLUGIN_PATH=/opt/ros/melodic/lib:$GAZEBO_PLUGIN_PATH

#Launch ROS integrated Gazebo

source ~/ardupilot_gazebo_roscam/devel/setup.bash

roslaunch ardupilot_gazebo iris_with_roscam.launch
```
### Launch SITL Ardupilot
Open second terminal and launch SITL Ardupilot
```
cd ~/ardupilot/ArduCopter

sim_vehicle.py -f gazebo-iris --console --map
```

### Launch MAVROS
```bash
cd ~/ardupilot_ws/src/launch && roslaunch apm.launch
```
### Run the script
```bash
python3 human_follow.py
```
Result
![](https://drive.google.com/file/d/1Oh1V27VyK4wmFPlU0hcOi-F_y4q_wBrU/view?usp=drive_link)

