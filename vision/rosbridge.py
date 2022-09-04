import time

import cv2
import numpy as np
import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image

import topics
import msgs


class RosBridge(object):
    def __init__(self):
        self.subscriber_list = []
        self.pub_aruco_detection = rospy.Publisher('aruco_detection', String, queue_size=10)
        self.pub_dont_detect_aruco = rospy.Publisher('dont_detect_aruco', Bool, queue_size=10)

    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def receive_msg(self, msg, topic):
        if rospy.is_shutdown():
            return

        if topic == topics.TOPIC_ARUCO_DETECTION:
            # msgs.ArucoDetection(corner_list, id_list, elapsed_time, image_creation_time)
            self.pub_aruco_detection.publish("elapsed_time:" + str(msg.elapsed_time))  # TODO: change to ArucoDetection msg
            return
        elif topic == topics.TOPIC_DONT_DETECT_ARUCO:
            # self.pub_dont_detect_aruco.publish(True)
            self.pub_dont_detect_aruco.publish(msg.boolean)
            return

    def start(self):
        rospy.init_node('rpi4_vision', anonymous=False, disable_signals=True)

    def stop(self):
        rospy.signal_shutdown("CLOSING ROS BRIDGE PROPERLY")


class GazeboVideoSource(object):
    def __init__(self):
        self.camera_ros_node = rospy.Subscriber("/webcam/image_raw", Image, self.analyze)
        self.subscriber_list = []

    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def analyze(self, data):

        img_array = np.fromstring(data.data, dtype=np.uint8).reshape((data.height, data.width, 3))
        _, img_jpeg = cv2.imencode('.jpg', img_array.copy())
        img_jpeg = img_jpeg.tostring()

        # for simulation purposes, consider the current time as the image creation time
        creation_time = time.time()

        self.publish(
            msg=msgs.Image(image=img_jpeg, creation_time=creation_time),
            topic=topics.TOPIC_IMAGE_JPEG
        )
        self.publish(
            msg=msgs.Image(image=img_array, creation_time=creation_time),
            topic=topics.TOPIC_IMAGE_ARRAY
        )

    # as the RosBridge node would be already running, we can't don't need to start another node
    # def start(self):
    #     rospy.init_node('gazebo_camera', anonymous=False, disable_signals=True)

    # def stop(self):
    #     rospy.signal_shutdown("CLOSING GAZEBO VIDEO SOURCE PROPERLY")
