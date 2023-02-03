from threading import Thread, Condition

import topics
# from time_utils import ensure_loop_rate


class VideoStreaming(object):

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.stopped = True
        self.new_frame_condition = Condition()

    def receive_msg(self, msg, topic):
        if topic != topics.TOPIC_IMAGE_JPEG:
            return
        self.latest_frame = msg.image
        self.latest_creation_time = msg.creation_time
        with self.new_frame_condition:
            self.new_frame_condition.notify_all()

    def stream_in_parallel(self):
        while not self.stopped:
            with self.new_frame_condition:
                self.new_frame_condition.wait()
            # TODO: check if producer.send() is blocking. If not, the Thread is not necessary
            # TODO: check if producer.send() has a block parameter
            self.producer.send('image-jpeg', self.latest_frame)
            # TODO: send latest_creation_time associated with the image


    def start(self):
        self.stopped = False
        # TODO: detach the kafka producer from the video streaming, create a kafkabridge (like rosbridge)
        self.producer = KafkaProducer(
            bootstrap_servers=['{}:{}'.format(self.ip, self.port)],
        )
        self.streaming_thread = Thread(
            target=self.stream_in_parallel,
            args=()
        )
        self.streaming_thread.daemon = True
        self.streaming_thread.start()

    def stop(self):
        self.producer.flush()
        self.producer.close()
        self.stopped = True
