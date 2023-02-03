#### File list

* [main.py](./main.py): code that glues everything together.

* [videosourcepi.py](./videosourcepi.py): capture camera images from a Raspberry Pi camera and publish them as a topic.
* [videosourcegazebo.py](./videosourcegazebo.py): capture camera images from Gazebo ROS topic ("/webcam/image_raw") and publish them as a topic.
* [videosourcekafka.py](./videosourcekafka.py): receive images from a Raspberry Pi running Kafka and publish them as a topic.
* [videosaver.py](./videosaver.py): save the images as videos.
* [videostreamingapp.py](./videostreamingapp.py): host a site from which is possible to see the latest image the vehicle captured.
* [videostreaming.py](./videostreaming.py): stream the images the vehicle captured to Kafka.
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
python3 main.py --stream-video-to-app

# e.g to live stream the drone footage and detect aruco markers, run
python3 main.py --stream-video-to-app --detect-aruco

# e.g to detect aruco markers and publish the detections to ROS, run
python3 main.py --detect-aruco --using-ros

# e.g to receive images from the vehicle on a laptop, then detect aruco markers
# vehicle side
python3 main.py --stream-video-to-kafka --detect-aruco

# laptop side
python3 main.py --using-kafka --detect-aruco
```
