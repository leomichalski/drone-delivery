import topics


class VideoStreamingZmq(object):

    def __init__(self, producer_zmq):
        self.producer_zmq = producer_zmq

    def receive_msg(self, msg, topic):
        if topic != topics.TOPIC_IMAGE_JPEG:
            return
        # with current implementation (pub),
        # it won't block the main thread even if the internet connection is lost
        self.producer_zmq.send_jpeg(msg.creation_time, msg.image, copy=False)
