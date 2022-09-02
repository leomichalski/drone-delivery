# import time

# import numpy as np
import rospy
from std_msgs.msg import String

import topics
import msgs


class RosBridge(object):
    def __init__(self):
        self.subscriber_list = []
        self.pub_aruco_detection = rospy.Publisher('aruco_detection', String, queue_size=10)
        self.pub_dont_detect_aruco = rospy.Publisher('dont_detect_aruco', String, queue_size=10)

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
            self.pub_dont_detect_aruco.publish("Dont detect aruco")  # TODO: change to Boolean msg
            return

    def start(self):
        rospy.init_node('rpi4_vision', anonymous=False, disable_signals=True)

    def stop(self):
        rospy.signal_shutdown("CLOSING ROS BRIDGE PROPERLY")


class GazeboVideoSource(object):
    def __init__(self):
        self.camera_ros_node = rospy.Subscriber("??????", String, analyze)  # TODO
        self.subscriber_list = []

    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def analyze(self, data):
        img_jpeg = data.data  # TODO
        img_array = data.data  # TODO
        creation_time = 0  # TODO
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
