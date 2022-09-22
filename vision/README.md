#### File list

* [main.py](./main.py): code that glues everything together.

* [pivideosource.py](./pivideosource.py): capture camera images from the Raspberry Pi camera and publish them as a topic.
* [gazebovideosource.py](./gazebovideosource.py): capture camera images from Gazebo ROS topic ("/webcam/image_raw") and publish them as a topic.
* [videosaver.py](./videosaver.py): save the images as videos.
* [videowebstreaming.py](./videowebstreaming.py): host a site from which is possible to see the latest image the vehicle captured.
* [arucodetector.py](./arucodetector.py): detect ArUco markers, then publish the detections as a topic.

* [rosbridge.py](./rosbridge.py): bridge so ROS can also access some of the topics.

* [topics.py](./topics.py): topics.
* [msgs.py](./msgs.py): messages that are published to topics.
* [utils.py](./utils.py): a few out of place methods.

* [requirements.txt](./requirements.txt): Python libraries used both during simulations and field testing.
* [requirements_raspberry.txt](./requirements_raspberry.txt): Python libraries specific to the companion computer.

#### How to install

```
# create virtual environment
python3 -m venv --system-site-packages

# install libraries
pip3 install requirements.txt

# install libraries specific to raspberry pi boards
pip3 install requirements_raspberry.txt
```


#### How to run

```
# use the following command to check your options
python3 main.py --help


# e.g to live stream the drone footage, run
python3 main.py --webstream-video

# e.g to live stream the drone footage and detect aruco markers, run
python3 main.py --webstream-video --detect-aruco

# e.g to detect aruco markers and publish the detections to ROS, run
python3 main.py --detect-aruco --using-ros
```
