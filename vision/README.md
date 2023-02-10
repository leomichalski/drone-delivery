#### File list

* [main.py](./main.py): code that glues everything together.

* [videosourcepi.py](./videosourcepi.py): capture camera images from a Raspberry Pi camera and publish them as a topic.
* [videosourcegazebo.py](./videosourcegazebo.py): capture camera images from Gazebo ROS topic ("/webcam/image_raw") and publish them as a topic.
* [videosourcezmq.py](./videosourcezmq.py): receive images from a Raspberry Pi running ZeroMQ and publish them as a topic.
* [videosaver.py](./videosaver.py): save the images as videos.
* [videostreamingapp.py](./videostreamingapp.py): host a site from which is possible to see the latest image the vehicle captured.
* [videostreamingzmq.py](./videostreamingzmq.py): stream the images the vehicle captured to ZeroMQ.
* [arucodetector.py](./arucodetector.py): detect ArUco markers, then publish the detections as a topic.

* [bridgeros.py](./bridgeros.py): bridge so ROS can also access some of the topics.
* [bridgezmq.py](./bridgezmq.py): bridge to send/receive messages to/from ZeroMQ.

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
python3 main.py --stream-video-to-app

# e.g to live stream the drone footage and detect aruco markers, run
python3 main.py --stream-video-to-app --detect-aruco

# e.g to detect aruco markers and publish the detections to ROS, run
python3 main.py --detect-aruco --using-ros

# e.g to send images from the vehicle to a laptop, then detect aruco markers with the laptop
# vehicle side
python3 main.py --stream-video-to-zmq

# laptop side
python3 main.py --receive-video-from-zmq --detect-aruco
```
