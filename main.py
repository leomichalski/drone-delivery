import time
import argparse

from imageclassifier import *
from topics import *
from videosource import *
from videowebstreaming import *
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
        "--webstream_video",
        action='store_true', default=False,
        help=""
    )
    ap.add_argument(
        "--predict_image",
        action='store_true', default=False,
        help=""
    )
    ap.add_argument(
        "--do_everything",
        action='store_true', default=False,
        help=""
    )

    # image detection stuff
    ap.add_argument(
        "--categories_file",
        type=str, default='categories.txt',
        help="file containing the image model categories"
    )
    ap.add_argument(
        "--image_classifier_threshold",
        type=float, default=0.6,
        help="image classifier threshold"
    )

    # camera stuff
    ap.add_argument(
        "--frames_per_second",
        type=int, default=25,
        help="frames per second"
    )
    ap.add_argument(
        "-W", "--frame_width",
        type=int, default=640,
        help="frame width"
    )
    ap.add_argument(
        "-H", "--frame_height",
        type=int, default=480,
        help="frame height"
    )
    ap.add_argument(
        "--frame_rotation",
        type=int, default=180,
        help="camera rotation"

    )
    ap.add_argument(
        "--frame_channels",
        type=int, default=3,
        help="frame number of channels"
    )

    # web streaming stuff
    ap.add_argument(
        "--web_streaming_port",
        type=int, default=8090,
        help="ephemeral port number of the server (1024 to 65535)"
    )

    # other stuff
    ap.add_argument(
        "-t", "--time_out",
        type=int, default=360*24*60*60,
        help="time out in seconds (default: 360 days) or"
    )
    args = ap.parse_args()

    if args.do_everything:
        args.predict_image = True
        args.webstream_video = True

    if not args.do_everything \
            and not args.predict_image \
            and not args.webstream_video:
        raise Exception('Please select at least one of the following options: '
                        '--' + 'do_everything' + '; '
                        '--' + 'predict_image' + '; '
                        '--' + 'webstream_video' + '. ')

    return args


def main(args):
    nodes_to_stop = []

    if args.webstream_video or args.predict_image:
        video_source = VideoSource(
            frame_width=args.frame_width,
            frame_height=args.frame_height,
            frames_per_second=args.frames_per_second,
            rotation=args.frame_rotation,
        )

    if args.webstream_video:
        video_web_streaming = VideoWebStreaming(
            ip=utils.get_rpi_ip(),
            port=args.web_streaming_port,
            frame_width=args.frame_width,
            frame_height=args.frame_height,
        )

    if args.predict_image:
        image_predictor = ImagePredictor(
            class_names=utils.categories_file_to_class_names("categories.txt"),
            threshold=0.5,
        )

    if 'video_source' in locals():
        print('STARTING CAMERA')
        video_source.start()
        nodes_to_stop.append(video_source)
        # warmup the camera
        time.sleep(2)

    if 'video_web_streaming' in locals():
        print('STARTING THE VIDEO WEB STREAMING')
        video_source.subscribe(video_web_streaming)
        video_web_streaming.start(
            # threaded=True,
            use_reloader=False,
            # debug=True,
        )
        nodes_to_stop.append(video_web_streaming)

    if 'image_predictor' in locals():
        print('STARTING IMAGE PREDICTOR')
        image_source.subscribe(image_predictor)
        # warmup the image predictor
        # time.sleep(1)

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
