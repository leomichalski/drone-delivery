import time
import argparse

from arucodetector import ArucoDetector
from rosbridge import RosBridge, GazeboVideoSource
from videowebstreaming import VideoWebStreaming
import utils


def stop_all_nodes(nodes_to_stop):
    for node in reversed(nodes_to_stop):
        node.stop()


def parse_args():
    ap = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    # what should the app do
    ap.add_argument(
        "--webstream-video",
        action='store_true', default=False,
        help=""
    )
    ap.add_argument(
        "--detect-aruco",
        action='store_true', default=False,
        help=""
    )
    ap.add_argument(
        "--using-ros",
        action='store_true', default=False,
        help="ros bridge to pass the computer vision information to the flight control code"
    )
    ap.add_argument(
        "--using-gazebo",
        action='store_true', default=False,
        help="ros bridge to pass the computer vision information to the flight control code"
    )

    ap.add_argument(
        "--mode-raspberry",
        action='store_true', default=False,
        help=""
    )
    ap.add_argument(
        "--mode-gazebo",
        action='store_true', default=False,
        help=""
    )
    ap.add_argument(
        "--using-vpn",
        action='store_true', default=False,
        help=""
    )

    # camera stuff
    ap.add_argument(
        "--frames-per-second",
        type=int, default=25,
        help="frames per second"
    )
    ap.add_argument(
        "-W", "--frame_width",
        type=int, default=640,
        help="frame width"
    )
    ap.add_argument(
        "-H", "--frame-height",
        type=int, default=480,
        help="frame height"
    )
    ap.add_argument(
        "--frame-rotation",
        type=int, default=180,
        help="camera rotation"

    )
    ap.add_argument(
        "--frame-channels",
        type=int, default=3,
        help="frame number of channels"
    )

    # web streaming stuff
    ap.add_argument(
        "--web-streaming-port",
        type=int, default=8090,
        help="ephemeral port number of the server (1024 to 65535)"
    )

    # other stuff
    ap.add_argument(
        "-t", "--time-out",
        type=int, default=360*24*60*60,
        help="time out in seconds (default: 360 days) or"
    )
    args = ap.parse_args()
    print(args)

    if args.using_gazebo:
        args.using_ros = True

    if args.mode_gazebo:
        args.webstream_video = True
        args.detect_aruco = True
        args.using_ros = True
        args.using_gazebo = True
        args.using_vpn = False

    if args.mode_raspberry:
        args.webstream_video = True
        args.detect_aruco = True
        args.using_ros = True
        args.using_gazebo = False
        # args.using_vpn = True  # it depends on your network choice

    if not args.mode_gazebo \
            and not args.mode_raspberry \
            and not args.webstream_video \
            and not args.detect_aruco:
        raise Exception('Please select at least one of the following options: '
                        '--' + 'mode_gazebo' + '; '
                        '--' + 'mode_raspberry' + '; '
                        '--' + 'webstream_video' + '; '
                        '--' + 'detect_aruco' + '. ')

    return args


def main(args):
    nodes_to_stop = []

    if args.using_vpn:
        ip = utils.get_rpi_vpn_ip()
    else:
        ip = utils.get_rpi_ip()


    if (args.webstream_video or args.detect_aruco) and (not args.using_gazebo):
        from videosource import VideoSource
        video_source = VideoSource(
            frame_width=args.frame_width,
            frame_height=args.frame_height,
            frames_per_second=args.frames_per_second,
            rotation=args.frame_rotation,
        )

    if args.using_gazebo:
        gazebo_video_source = GazeboVideoSource()

    if args.webstream_video:
        video_web_streaming = VideoWebStreaming(
            ip=ip,
            port=args.web_streaming_port,
            frame_width=args.frame_width,
            frame_height=args.frame_height,
        )

    if args.detect_aruco:
        # image_predictor = ImagePredictor(
        #     class_names=utils.categories_file_to_class_names("categories.txt"),
        #     threshold=0.5,
        # )
        aruco_detector = ArucoDetector()
        # wait for the image classifier model to load
        # time.sleep(1)

    if args.using_ros:
        ros_bridge = RosBridge()

    if 'video_source' in locals():
        print('STARTING CAMERA')
        video_source.start()
        nodes_to_stop.append(video_source)
        # warmup the camera
        time.sleep(2)

    if ('video_web_streaming' in locals()) and ('video_source' in locals()):
        video_source.subscribe(video_web_streaming)

    if ('video_web_streaming' in locals()) and ('gazebo_video_source' in locals()):
        gazebo_video_source.subscribe(video_web_streaming)

    if 'video_web_streaming' in locals():
        print('STARTING THE VIDEO WEB STREAMING')
        video_web_streaming.start(
            # threaded=True,
            use_reloader=False,
            # debug=True,
        )
        nodes_to_stop.append(video_web_streaming)

    if ('aruco_detector' in locals()) and ('video_source' in locals()):
        print('STARTING ARUCO DETECTOR')
        video_source.subscribe(aruco_detector)

    if ('aruco_detector' in locals()) and ('gazebo_video_source' in locals()):
        print('STARTING ARUCO DETECTOR')
        gazebo_video_source.subscribe(aruco_detector)


    if 'ros_bridge' in locals():
        print('STARTING ROS BRIDGE')
        ros_bridge.start()
        nodes_to_stop.append(ros_bridge)




    if ('ros_bridge' in locals()) and ('aruco_detector' in locals()):
        aruco_detector.subscribe(ros_bridge)

    print("ALL NODES STARTED")
    try:
        time.sleep(args.time_out)
    finally:
        print('\n\n\nCLOSING THE APP PROPERLY')
        stop_all_nodes(nodes_to_stop)

if __name__ == '__main__':
    # import os
    # os.nice(-19)
    args = parse_args()
    main(args)