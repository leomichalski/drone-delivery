from threading import Thread, Condition
from flask import Response, Flask, render_template_string

import topics
# import msgs

class EndpointAction(object):

    def __init__(self, action, mimetype=None, headers=None):
        self.action = action
        self.mimetype = mimetype
        self.headers = headers

    def __call__(self, *args):
        # Perform the action
        answer = self.action()
        # Create the answer (bundle it in a correctly formatted HTTP answer)
        self.response = Response(
            answer,
            status=200,
            mimetype=self.mimetype,
            headers=self.headers
        )
        # Send it
        return self.response


class VideoWebStreaming(object):

    app = None

    def __init__(self, ip, port,
                 frame_width, frame_height):
        self.ip = ip
        self.port = port
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.stopped = True
        self.latest_image_jpeg = None
        self.image_condition = Condition()
        self.app = Flask(__name__)  # initialize a flask object
        self.app.add_url_rule(
            '/', '/', EndpointAction(self.index, headers={})
        )
        self.app.add_url_rule(
            '/video_feed',
            '/video_feed',
            EndpointAction(
                self.generate_images,
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        )

    def receive_msg(self, msg, topic):
        if topic == topics.TOPIC_IMAGE_JPEG:
            self.latest_image_jpeg = msg.image
            with self.image_condition:
                self.image_condition.notify_all()
            return

    def start(self, **kwargs):
        self.output_stream = Thread(
            target=self.app.run,
            args=(self.ip, self.port),
            kwargs=kwargs
            )
        self.output_stream.daemon = True
        self.stopped = False
        self.output_stream.start()

    def index(self):
        ''' return the rendered template '''
        page = '''\
<html>
    <head>
        <title>UAV Monitoring</title>
    </head>
    <body>
        <center>
            <h1>UAV Monitoring</h1>
        </center>
        <center>
            <img src="{{ url_for('/video_feed') }}" >
        </center>
    </body>
</html>
    '''
        return render_template_string(page)
        # return render_template('index.html')

    def generate_images(self):
        """Image streaming generator function."""
        while not self.stopped:
            with self.image_condition:
                self.image_condition.wait()
            image = self.latest_image_jpeg
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + image + b'\r\n')

    def stop(self):
        self.stopped = True
        self.output_stream.join(5)
