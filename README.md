
#### Folder structure

* **drone-delivery**: root folder.
  * **[vision](./vision)**: code to detect ArUco markers and live stream drone footage.
  * **[flight](./flight)**: guidance, navigation and control (GNC) folder.
    * **[aruco_hunter](./flight/aruco_hunter)**: ROS package to find and land on an ArUco marker.
    * **[aruco_msgs](./flight/aruco_msgs)**: ROS package that define message(s) used to communicate ArUco detections from the ArUco detector node to the GNC nodes.
  * **[scripts](./scripts)**: useful scripts and recipes.
