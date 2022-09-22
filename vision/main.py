import time
import argparse

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
        "--save-video",
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
        help="enable ros bridge to pass the computer vision information to the flight control code"
    )
    ap.add_argument(
        "--using-gazebo",
        action='store_true', default=False,
        help="enable ros; enable gazebo video source to get the video from gazebo"
    )

    ap.add_argument(
        "--using-vpn",
        action='store_true', default=False,
        help=""
    )

    # camera stuff
    ap.add_argument(
        "--frames-per-second",
        type=int, default=7,
        help="frames per second"
    )
    ap.add_argument(
        "--frame-width",
        type=int, default=1920,
        help="frame width"
    )
    ap.add_argument(
        "--frame-height",
        type=int, default=1080,
        help="frame height"
    )
    ap.add_argument(
        "--streaming-frame-width",
        type=int, default=256,
        help="frame width"
    )
    ap.add_argument(
        "--streaming-frame-height",
        type=int, default=144,
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

    # video saving stuff
    ap.add_argument(
        "--video-output-time-limit",
        type=int, default=15*60,
        help="video time limit in seconds (default: 15min)"
    )
    ap.add_argument(
        "--video-output-folder",
        type=str, default='/home',
        help="folder path to store videos"
    )

    # other stuff
    ap.add_argument(
        "-t", "--time-out",
        type=int, default=360*24*60*60,
        help="time out in seconds (default: 360 days) or"
    )
    args = ap.parse_args()

    if args.using_gazebo:
        args.using_ros = True

    if not args.webstream_video \
            and not args.save_video \
            and not args.detect_aruco:
        raise Exception('Please select at least one of the following options: '
                        '--' + 'webstream-video' + '; '
                        '--' + 'save-video' + '; '
                        '--' + 'detect-aruco' + '. ')

    return args


def main(args):
    nodes_to_stop = []

    if args.using_vpn:
        ip = utils.get_rpi_vpn_ip()
    else:
        ip = utils.get_rpi_ip()

    if args.webstream_video:
        from videowebstreaming import VideoWebStreaming
        video_web_streaming = VideoWebStreaming(
            ip=ip,
            port=args.web_streaming_port,
        )

    if args.save_video:
        from videosaver import VideoSaver
        video_saver = VideoSaver(
            frames_per_second=args.frames_per_second,
            frame_width=args.frame_width,
            frame_height=args.frame_height,
            output_time_limit=args.video_output_time_limit,
            output_folder=args.video_output_folder,
        )

    if args.detect_aruco:
        from arucodetector import ArucoDetector
        aruco_detector = ArucoDetector()

    if args.using_ros:
        from rosbridge import RosBridge, RosNode
        ros_node = RosNode()
        ros_bridge = RosBridge()

    if args.using_gazebo:
        from gazebovideosource import GazeboVideoSource
        gazebo_video_source = GazeboVideoSource()

    if (args.save_video or args.webstream_video or args.detect_aruco) and (not args.using_gazebo):
        from pivideosource import PiVideoSource
        pi_video_source = PiVideoSource(
            frame_width=args.frame_width,
            frame_height=args.frame_height,
            streaming_frame_width=args.streaming_frame_width,
            streaming_frame_height=args.streaming_frame_height,
            frames_per_second=args.frames_per_second,
            rotation=args.frame_rotation,
        )

    if 'pi_video_source' in locals():
        print('STARTING RASPBERRY PI CAMERA')
        pi_video_source.start()
        nodes_to_stop.append(pi_video_source)
        # warmup the camera
        time.sleep(2)

    if ('video_web_streaming' in locals()) and ('pi_video_source' in locals()):
        pi_video_source.subscribe(video_web_streaming)

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

    if ('video_saver' in locals()) and ('pi_video_source' in locals()):
        pi_video_source.subscribe(video_saver)

    if ('video_saver' in locals()) and ('gazebo_video_source' in locals()):
        gazebo_video_source.subscribe(video_saver)

    if 'video_saver' in locals():
        print('STARTING THE VIDEO SAVER')
        video_saver.start()
        nodes_to_stop.append(video_saver)

    if ('aruco_detector' in locals()) and ('pi_video_source' in locals()):
        print('STARTING ARUCO DETECTOR')
        pi_video_source.subscribe(aruco_detector)

    if ('aruco_detector' in locals()) and ('gazebo_video_source' in locals()):
        print('STARTING ARUCO DETECTOR')
        gazebo_video_source.subscribe(aruco_detector)

    if 'ros_node' in locals():
        print('STARTING ROS NODE')
        ros_node.start()
        nodes_to_stop.append(ros_node)

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
