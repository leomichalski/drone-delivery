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
        # elif topic == topics.TOPIC_IMAGE_ARRAY:
        #     return
        # elif topic == topics.TOPIC_IMAGE_JPEG:
        #     return

    def start(self):
        rospy.init_node('rpi4_vision', anonymous=False, disable_signals=True)

    def stop(self):
        rospy.signal_shutdown("CLOSING THE APP PROPERLY")
