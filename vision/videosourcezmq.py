import time
from threading import Thread, Condition

import zmq
import cv2
import numpy as np

import msgs
import topics
from utils import ensure_loop_rate


class VideoSourceZmq(object):
    def __init__(self, consumer_zmq, frames_per_second):
        self.consumer_zmq = consumer_zmq
        self.frames_per_second = frames_per_second
        self.stopped = True
        self.subscriber_list = []

    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def start(self):
        self.stopped = False
        self.output_stream = Thread(
            target=self.analyze,
        )
        self.output_stream.daemon = True
        self.output_stream.start()

    def analyze(self):
        while not self.stopped:
            start_time = time.time()
            creation_time, img_jpeg = self.consumer_zmq.recv_jpeg()
            # TODO: use simplejpeg instead of cv2.imdecode, it'd be faster
            img_array = cv2.imdecode(
                np.frombuffer(img_jpeg, dtype='uint8'),
                cv2.IMREAD_UNCHANGED
            )
            self.publish(
                msg=msgs.Image(image=img_jpeg, creation_time=creation_time),
                topic=topics.TOPIC_IMAGE_JPEG
            )
            self.publish(
                msg=msgs.Image(image=img_array, creation_time=creation_time),
                topic=topics.TOPIC_IMAGE_ARRAY
            )
            ensure_loop_rate(
                rate=1/self.frames_per_second,
                loop_time=time.time() - start_time
            )

    def stop(self):
        self.stopped = True
        self.output_stream.join(5)
