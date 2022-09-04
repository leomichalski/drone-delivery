source /opt/ros/noetic/setup.bash


sudo apt install python3-catkin-tools python3-rosinstall-generator python3-osrf-pycommon -y

rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall

rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall

wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y

# 5. Install GeographicLib datasets: # MAYBE APT
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# 6. Build source
catkin build

# 7. Make sure that you use setup.bash or setup.zsh from workspace.
#    Else rosrun can't find nodes from this workspace.
source devel/setup.bash






git clone https://github.com/ros/urdf
git clone https://github.com/ros/urdfdom
git clone https://github.com/ros/urdfdom_headers; cd urdfdom_headers; wget https://raw.github.com/ros-gbp/urdfdom_headers-release/debian/indigo/precise/urdfdom_headers/package.xml

# https://github.com/ros-geographic-info/unique_identifier-release/
git clone https://github.com/ros-geographic-info/unique_identifier

git clone https://github.com/ros-controls/control_toolbox -b melodic-devel

git clone https://github.com/ros/bond_core.git -b kinetic-devel


git clone https://github.com/ros-controls/control_msgs/  -b kinetic-devel
git clone https://github.com/ros-controls/control_toolbox.git -b melodic-devel
git clone https://github.com/ros/dynamic_reconfigure.git --branch noetic-devel

git clone https://github.com/ros-controls/realtime_tools.git --branch noetic-devel








# to make mavros work with px4, you still need to:

# eigen
git clone eigen -b 3.1
cmake eigen (/usr/include/eigen3)

make install eigen
move signature from /usr/local/include/eigen3 to /usr/include/eigen3

add FindEigen3.cmake to ??/???/?/cmake-2.86/Modules
add FindEigen3.cmake as FindEigen.cmake to ??/???/?/cmake-2.86/Modules

# oros
cd ros_catkin_ws/src; git clone oros -b old_branch
