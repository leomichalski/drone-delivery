import serial
import time

def test_loopback():
    input_ = b'hello world'
    output_ = ''
    try:
        ser = serial.Serial('/dev/serial0', 115200)
        # ser = serial.Serial('/dev/ttyACM0', 57600)
        ser.write(input_)
        time.sleep(0.001)
        output_ = ser.read(len(input_))
    finally:
        ser.close()
    assert output_ == input_


if __name__ == '__main__':
    test_loopback()
    print('it worked')
