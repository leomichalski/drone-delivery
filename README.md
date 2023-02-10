## Table of contents

- [Table of contents](#table-of-contents)
- [Folder structure](#folder-structure)
- [Manual execution on a companion computer](#manual-execution-on-a-companion-computer)
  - [How to run the Raspberry Pi code](#how-to-run-the-raspberry-pi-code)
        - [terminal 1](#terminal-1)
        - [terminal 2](#terminal-2)
        - [terminal 3](#terminal-3)
- [Simulation](#simulation)
  - [Setup](#setup)
  - [How to run the simulation](#how-to-run-the-simulation)
        - [terminal 1](#terminal-1-1)
        - [terminal 2](#terminal-2-1)
        - [terminal 3](#terminal-3-1)
        - [terminal 4 (optional)](#terminal-4-optional)
  - [Useful coords](#useful-coords)


## Folder structure

* **drone-delivery**: root folder.
  * **[vision](./vision)**: code to detect ArUco markers and live stream drone footage.
  * **[flight](./flight)**: guidance, navigation and control (GNC) folder.
    * **[aruco_hunter](./flight/aruco_hunter)**: ROS package to find and land on an ArUco marker.
    * **[aruco_msgs](./flight/aruco_msgs)**: ROS package that define message(s) used to communicate ArUco detections from the ArUco detector node to the GNC nodes.
  * **[scripts](./scripts)**: useful instructions, scripts and recipes.


## Manual execution on a companion computer
WARNING: don't execute the following commands manually on a vehicle with propeller blades.

### How to run the Raspberry Pi code
The instructions suppose the board has 4 CPU cores. Tip: use tmux to run the instructions.

###### terminal 1

```
# launch roscore and mavros
taskset -c 3 roslaunch mavros px4.launch
```

###### terminal 2

```
# change the current directory
cd drone-delivery/vision

# run the computer vision code
taskset -c 0,1,2 python3 main.py --stream-video-to-app --detect-aruco --using-ros --frames-per-second 7

# for more info, run
python3 main.py --help
```

###### terminal 3

```
# run the guidance, navigation and control code
taskset -c 3 roslaunch aruco_hunter main.launch
```

## Simulation

### Setup
It's not my priority to write better setup instructions right now, but I'm using Ubuntu 20.04, ROS Noetic, Gazebo 11, MAVROS [2.2.0](https://github.com/mavlink/mavros/tree/2.2.0), PX4-Autopilot [1.13.0](https://github.com/PX4/PX4-Autopilot/tree/v1.13.0) and the latest [PX4-SITL_gazebo](https://github.com/PX4/PX4-SITL_gazebo).


### How to run the simulation

###### terminal 1

```
# set the vehicle startup location (see the next section for useful coords)
export PX4_HOME_LAT=52.171974
export PX4_HOME_LON=4.417091
export PX4_HOME_ALT=0

# launch gazebo, roscore, mavros and px4 sitl with a single command
roslaunch px4 mavros_posix_sitl.launch

# set some parameters to be able to simulate without a RC control
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
param set COM_RCL_EXCEPT 4
```

###### terminal 2

```
# launch qgroundcontrol
./QGroundControl.AppImage
```

###### terminal 3
Open the [main.launch](/flight/aruco_hunter/launch/main.launch) file for more info. Must open if changing the vehicle default startup location (52.171974, 4.417091).
```
# run the guidance, navigation and control code
roslaunch aruco_hunter main.launch
```

###### terminal 4 (optional)

```
# change the current working directory
cd drone-delivery/vision

# run the computer vision related code
python3 main.py --stream-video-to-app --detect-aruco --using-ros --receive-video-from-gazebo

# for more info on how to run the computer vision code, run
python3 main.py --help
```

### Useful coords
Tip: calculate altitude from latitude and longitude using https://www.maps.ie/coordinates.html

```
# default
export PX4_HOME_LAT=28.452386
export PX4_HOME_LON=-13.867138
export PX4_HOME_ALT=36
roslaunch px4 mavros_posix_sitl.launch

# my school soccer field (quadra de futebol da FGA)
export PX4_HOME_LAT=-15.989944
export PX4_HOME_LON=-48.044025
export PX4_HOME_ALT=1217
roslaunch px4 mavros_posix_sitl.launch
```
