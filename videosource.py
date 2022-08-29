import io
import time

import picamera
from picamera.array import PiRGBAnalysis

import topics
import msgs


class JPEGOutput(object):
    def __init__(self, analyze):
        self.analyze = analyze
        self.buffer = io.BytesIO()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            self.analyze(self.buffer.getvalue())
            self.buffer.seek(0)
        self.buffer.write(buf)

    def flush(self):
        self.buffer.flush()

    def close(self):
        self.buffer.close()


class ArrayOutput(PiRGBAnalysis):
    def __init__(self, camera, analyze):
        super().__init__(camera)
        self.analyze = analyze


class VideoSource:

    def __init__(self, frame_height, frame_width, frames_per_second, **kwargs):
        self.frame_height = frame_height
        self.frame_width = frame_width
        self.frames_per_second = frames_per_second
        self.camera = picamera.PiCamera(
            resolution=(self.frame_width, self.frame_height),
            framerate=self.frames_per_second,
        )
        for (arg, value) in kwargs.items():
            setattr(self.camera, arg, value)
        self.subscriber_list = []

    def subscribe(self, subscriber):
        self.subscriber_list.append(subscriber)

    def publish(self, msg, topic):
        for subscriber in self.subscriber_list:
            subscriber.receive_msg(msg=msg, topic=topic)

    def analyze_array(self, img_array):
        self.publish(
            msg=msgs.Image(image=img_array, creation_time=time.time()),
            topic=topics.TOPIC_IMAGE_ARRAY
        )

    def analyze_jpeg(self, buf_value):
        self.publish(
            msg=msgs.Image(image=buf_value, creation_time=time.time()),
            topic=topics.TOPIC_IMAGE_JPEG
        )

    def start(self):
        self.array_output = ArrayOutput(
            camera=self.camera, analyze=self.analyze_array
        )
        self.jpeg_output = JPEGOutput(analyze=self.analyze_jpeg)

        self.camera.start_recording(
            self.array_output,
            splitter_port=1,
            format='rgb'
        )
        self.camera.start_recording(
            self.jpeg_output,
            splitter_port=2,
            format='mjpeg'
        )

    def stop(self):
        time.sleep(0.1)
        self.jpeg_output.close()
        self.array_output.close()
