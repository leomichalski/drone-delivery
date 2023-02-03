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
        "--stream-video-to-app",
        action='store_true', default=False,
        help=""
    )
    ap.add_argument(
        "--stream-video-to-kafka",
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
        "--receive-video-from-kafka",
        action='store_true', default=False,
        help="enable kafka video source to get the video from a kafka producer"
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

    if not args.save_video \
            and not args.stream_video_to_app \
            and not args.stream_video_to_kafka \
            and not args.detect_aruco:
        raise Exception('Please select at least one of the following options: '
                        '--' + 'save-video' + '; '
                        '--' + 'stream-video-to-app' + '; '
                        '--' + 'stream-video-to-kafka' + '; '
                        '--' + 'detect-aruco' + '. ')

    return args


def main(args):
    nodes_to_stop = []

    if args.using_vpn:
        ip = utils.get_rpi_vpn_ip()
    else:
        ip = utils.get_rpi_ip()

    if args.stream_video_to_app:
        from videostreamingapp import VideoStreamingApp
        video_streaming_app = VideoStreamingApp(
            ip=ip,
            port=args.web_streaming_port,
        )

    if args.stream_video_to_kafka:
        from videostreaming import VideoStreaming
        video_streaming = VideoStreaming(
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
        from videosourcegazebo import VideoSourceGazebo
        video_source_gazebo = VideoSourceGazebo()

    if args.receive_video_from_kafka:
        from videosourcekafka import VideoSourceKafka
        video_source_kafka = VideoSourceKafka()

    if (args.save_video or args.stream_video_to_app or args.stream_video_to_kafka or args.detect_aruco) and (not args.using_gazebo):
        from videosourcepi import VideoSourcePi
        video_source_pi = VideoSourcePi(
            frame_width=args.frame_width,
            frame_height=args.frame_height,
            streaming_frame_width=args.streaming_frame_width,
            streaming_frame_height=args.streaming_frame_height,
            frames_per_second=args.frames_per_second,
            rotation=args.frame_rotation,
        )

    if 'video_source_pi' in locals():
        print('STARTING RASPBERRY PI CAMERA')
        video_source_pi.start()
        nodes_to_stop.append(video_source_pi)

    if ('video_streaming_app' in locals()) and ('video_source_pi' in locals()):
        video_source_pi.subscribe(video_streaming_app)

    if ('video_streaming_app' in locals()) and ('video_source_gazebo' in locals()):
        video_source_gazebo.subscribe(video_streaming_app)

    if ('video_streaming_app' in locals()) and ('video_source_kafka' in locals()):
        video_source_kafka.subscribe(video_streaming_app)


    if ('video_streaming' in locals()) and ('video_source_pi' in locals()):
        video_source_pi.subscribe(video_streaming)

    if ('video_streaming' in locals()) and ('video_source_gazebo' in locals()):
        video_source_gazebo.subscribe(video_streaming)

    # kafka can't be it's own video source
    # if ('video_streaming' in locals()) and ('video_source_kafka' in locals()):
    #     video_source_kafka.subscribe(video_streaming)


    if 'video_streaming_app' in locals():
        print('STARTING THE VIDEO STREAMING FLASK APPLICATION')
        video_streaming_app.start(
            # threaded=True,
            use_reloader=False,
            # debug=True,
        )
        nodes_to_stop.append(video_streaming_app)

    if 'video_streaming' in locals():
        print('STARTING TO STREAM VIDEO TO KAFKA')
        video_streaming.start()
        nodes_to_stop.append(video_streaming)

    if ('video_saver' in locals()) and ('video_source_pi' in locals()):
        video_source_pi.subscribe(video_saver)

    if ('video_saver' in locals()) and ('video_source_gazebo' in locals()):
        video_source_gazebo.subscribe(video_saver)

    if 'video_saver' in locals():
        print('STARTING THE VIDEO SAVER')
        video_saver.start()
        nodes_to_stop.append(video_saver)

    if ('aruco_detector' in locals()) and ('video_source_pi' in locals()):
        print('STARTING ARUCO DETECTOR')
        video_source_pi.subscribe(aruco_detector)

    if ('aruco_detector' in locals()) and ('video_source_gazebo' in locals()):
        print('STARTING ARUCO DETECTOR')
        video_source_gazebo.subscribe(aruco_detector)

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
