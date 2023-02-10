# module based on ImageZMQ
import zmq


class ConsumerZmq(object):

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.address = 'tcp://{}:{}'.format(self.ip, self.port)
        self.zmq_context = SerializingContext()
        self.zmq_socket = self.zmq_context.socket(socket_type=zmq.SUB)
        self.zmq_socket.setsockopt(zmq.SUBSCRIBE, b'')
        self.zmq_socket.connect(self.address)

        # assign corresponding methods for convenience
        self.recv_array = self.zmq_socket.recv_array
        self.recv_jpeg = self.zmq_socket.recv_jpeg

    def stop(self):
        self.zmq_socket.close()
        self.zmq_context.term()


class ProducerZmq(object):

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.address = 'tcp://{}:{}'.format(self.ip, self.port)
        self.zmq_context = SerializingContext()
        self.zmq_socket = self.zmq_context.socket(socket_type=zmq.PUB)
        self.zmq_socket.bind(self.address)

        # assign corresponding methods for convenience
        self.send_array = self.zmq_socket.send_array
        self.send_jpeg = self.zmq_socket.send_jpeg

    def stop(self):
        self.zmq_socket.close()
        self.zmq_context.term()


class SerializingSocket(zmq.Socket):
    """Numpy array serialization methods.
    Modelled on PyZMQ serialization examples.
    Used for sending / receiving OpenCV images, which are Numpy arrays.
    Also used for sending / receiving jpeg compressed OpenCV images.
    """

    def send_array(self, A, msg='NoName', flags=0, copy=True, track=False):
        """Sends a numpy array with metadata and text message.
        Sends a numpy array with the metadata necessary for reconstructing
        the array (dtype,shape). Also sends a text msg, often the array or
        image name.
        Arguments:
          A: numpy array or OpenCV image.
          msg: (optional) array name, image name or text message.
          flags: (optional) zmq flags.
          copy: (optional) zmq copy flag.
          track: (optional) zmq track flag.
        """
        md = dict(
            msg=msg,
            dtype=str(A.dtype),
            shape=A.shape,
        )
        self.send_json(md, flags | zmq.SNDMORE)
        return self.send(A, flags, copy=copy, track=track)

    def send_jpeg(self,
                 msg='NoName',
                 jpeg_buffer=b'00',
                 flags=0,
                 copy=True,
                 track=False):
        """Send a jpeg buffer with a text message.
        Sends a jpeg bytestring of an OpenCV image.
        Also sends text msg, often the image name.
        Arguments:
          msg: image name or text message.
          jpeg_buffer: jpeg buffer of compressed image to be sent.
          flags: (optional) zmq flags.
          copy: (optional) zmq copy flag.
          track: (optional) zmq track flag.
        """
        md = dict(msg=msg, )
        self.send_json(md, flags | zmq.SNDMORE)
        return self.send(jpeg_buffer, flags, copy=copy, track=track)

    def recv_array(self, flags=0, copy=True, track=False):
        """Receives a numpy array with metadata and text message.
        Receives a numpy array with the metadata necessary
        for reconstructing the array (dtype,shape).
        Returns the array and a text msg, often the array or image name.
        Arguments:
          flags: (optional) zmq flags.
          copy: (optional) zmq copy flag.
          track: (optional) zmq track flag.
        Returns:
          msg: image name or text message.
          A: numpy array or OpenCV image reconstructed with dtype and shape.
        """
        md = self.recv_json(flags=flags)
        msg = self.recv(flags=flags, copy=copy, track=track)
        A = np.frombuffer(msg, dtype=md['dtype'])
        return (md['msg'], A.reshape(md['shape']))

    def recv_jpeg(self, flags=0, copy=True, track=False):
        """Receives a jpeg buffer and a text msg.
        Receives a jpeg bytestring of an OpenCV image.
        Also receives a text msg, often the image name.
        Arguments:
          flags: (optional) zmq flags.
          copy: (optional) zmq copy flag.
          track: (optional) zmq track flag.
        Returns:
          msg: image name or text message.
          jpeg_buffer: bytestring, containing jpeg image.
        """
        md = self.recv_json(flags=flags)  # metadata text
        jpeg_buffer = self.recv(flags=flags, copy=copy, track=track)
        return (md['msg'], jpeg_buffer)


class SerializingContext(zmq.Context):
    _socket_class = SerializingSocket
