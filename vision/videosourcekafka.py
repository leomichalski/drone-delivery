import time

from kafka import KafkaConsumer

import cv2
import numpy as np

import msgs
import topics


class KafkaVideoSource(object):
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.stopped = True
        self.subscriber_list = []

    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def start(self, **kwargs):
        self.stopped = False
        self.consumer = KafkaConsumer(
            'image-jpeg',
            bootstrap_servers=['{}:{}'.format(self.ip, self.port)],
        )
        self.output_stream = Thread(
            target=self.analyze,
            args=(),
        )
        self.output_stream.daemon = True
        self.output_stream.start()

    def analyze(self):
        while not self.stopped:
            message = self.consumer.poll(
                timeout_ms=10000  # 0.1 FPS
            )
            if message is None:
                continue
            if message.error():
                print('Error: {}'.format(message.error()))
                continue
            # TODO: check if the message is a jpeg image or a list of jpeg images
            img_jpeg_list = message.value()
            for img_jpeg in img_jpeg_list:
                # TODO: check if the next line of code really works and if it is the fastest way to convert the image
                # simplejpeg would probably be faster than opencv
                image_array = cv2.imdecode(np.fromstring(img_jpeg, np.uint8), cv2.IMREAD_COLOR)
                
                # TODO: send the creation time from the kafka producer
                creation_time = int(time.time())

                self.publish(
                    msg=msgs.Image(image=img_jpeg, creation_time=creation_time),
                    topic=topics.TOPIC_IMAGE_JPEG
                )
                self.publish(
                    msg=msgs.Image(image=img_array, creation_time=creation_time),
                    topic=topics.TOPIC_IMAGE_ARRAY
                )

    def stop(self):
        self.stopped = True
        self.consumer.close()
        self.output_stream.join(5)
