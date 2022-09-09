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

```
# run the guidance, navigation and control code
rosrun aruco_hunter main
```

###### terminal 4 (optional)

```
# change the current working directory
cd drone-delivery/vision

# run the computer vision related code
python3 main.py --using-ros --using-gazebo --webstream-video --detect-aruco

# for more info on how to run the computer vision code, run
python3 main.py --help
```

### Useful coords
Tip: calculate altitude from (latitude, longitude) using https://www.maps.ie/coordinates.html

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
