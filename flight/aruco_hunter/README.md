# aruco_hunter
The objective of this ROS package is to find and land on an ArUco marker that may be some meters off from a given GPS coordinate. It's behavior is described with a [state transition diagram](/flight/aruco_hunter/transition_diagrams/mission00.png). It depends on the [vision](/vision) code to detect the ArUco marker.


### How to build
The "--no-deps" flag is optional, but it speeds up the build process if the only thing to be built is this package.

```
catkin build aruco_hunter --no-deps
```

### How to run
Open the "launch/main.launch" file for more info.

```
roslaunch aruco_hunter main.launch
```
