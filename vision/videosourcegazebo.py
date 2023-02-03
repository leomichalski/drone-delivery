import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

import msgs
import topics


class VideoSourceGazebo(object):
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
        creation_time = int(time.time())

        self.publish(
            msg=msgs.Image(image=img_jpeg, creation_time=creation_time),
            topic=topics.TOPIC_IMAGE_JPEG
        )
        self.publish(
            msg=msgs.Image(image=img_array, creation_time=creation_time),
            topic=topics.TOPIC_IMAGE_ARRAY
        )
