
#### Folder structure
* **drone-delivery**: root folder.
  * **[flight](./flight)**: guidance, navigation and control folder.
    * **[aruco_hunter](./flight/aruco_hunter)**: package to find and land on an ArUco marker.
    * **[aruco_msgs](./flight/aruco_msgs)**: useful ROS messages common to the vision folder and the flight folder.
  * **[vision](./vision)**: code to detect ArUco markers and live stream drone footage.
  * **[scripts](./scripts)**: useful scripts and recipes.
