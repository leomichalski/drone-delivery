import time

import rospy
from aruco_msgs.msg import ArucoDetection

import topics


class RosNode(object):
    def __init__(self, node_name='vision', anonymous=False, disable_signals=True):
        self.node_name = node_name
        self.anonymous = anonymous
        self.disable_signals = disable_signals

    def start(self):
        rospy.init_node(
            node_name,
            anonymous=self.anonymous,
            disable_signals=self.disable_signals
        )

    def stop(self):
        rospy.signal_shutdown("CLOSING ROS PROPERLY")


class RosBridge(object):
    def __init__(self):
        self.subscriber_list = []
        self.aruco_detection_publisher = rospy.Publisher('aruco_detection', ArucoDetection, queue_size=10)

    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def receive_msg(self, msg, topic):
        if rospy.is_shutdown():
            return

        if topic == topics.TOPIC_ARUCO_DETECTION:
            self.aruco_detection_publisher.publish(
                marker_id=msg.marker_id,
                marker_center=msg.marker_center,
                elapsed_time=msg.elapsed_time,
                image_creation_time=msg.image_creation_time,
            )
            return
        # elif topic == topics.TOPIC_DONT_DETECT_ARUCO:
