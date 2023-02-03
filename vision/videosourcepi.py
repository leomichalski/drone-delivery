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


class VideoSourcePi(object):

    def __init__(self, frame_height, frame_width, streaming_frame_width, streaming_frame_height, frames_per_second, **kwargs):
        self.frame_height = frame_height
        self.frame_width = frame_width
        self.streaming_frame_height = streaming_frame_height
        self.streaming_frame_width = streaming_frame_width
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
        # numpy array (formato do opencv)
        self.publish(
            msg=msgs.Image(image=img_array, creation_time=int(time.time())),
            topic=topics.TOPIC_IMAGE_ARRAY
        )

    def analyze_jpeg(self, buf_value):
        # imagem binaria comprimida com JPEG
        self.publish(
            msg=msgs.Image(image=buf_value, creation_time=int(time.time())),
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
            format='rgb',
        )
        self.camera.start_recording(
            self.jpeg_output,
            splitter_port=2,
            format='mjpeg',
            resize=(self.streaming_frame_width, self.streaming_frame_height),
            # TODO: bitrate=,  # The bitrate at which video will be encoded. Defaults to 17000000 (17Mbps) if not specified. The maximum value depends on the selected H.264 level and profile. Bitrate 0 indicates the encoder should not use bitrate control (the encoder is limited by the quality only).
            # TODO: quality=,  # For the mjpeg format, use JPEG quality values between 1 and 100 (where higher values are higher quality). Quality 0 is special and seems to be a “reasonable quality” default.
        )
        # warmup the camera
        time.sleep(2)

    def stop(self):
        time.sleep(0.1)
        self.jpeg_output.close()
        self.array_output.close()
