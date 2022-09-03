import os
import time
import datetime
from threading import Thread, Condition

import cv2
# import imageio
import numpy as np

import topics
# from time_utils import ensure_loop_rate


def get_timestamp():
    timestamp = datetime.datetime.now()
    # timestamp = timestamp.replace(second=0, microsecond=0)
    timestamp = timestamp.strftime('%d_%b_%Y--%H:%M')
    return timestamp


def get_output_path(folder, base_filename, timestamp, index, ext):
    return (f'{os.path.join(folder, base_filename)}'
            f'_{timestamp}_{index}{ext}')


class VideoSaver(object):

    def __init__(self,
                 frames_per_second,
                 frame_width,
                 frame_height,
                 output_time_limit,
                 output_folder,
                 base_output_filename='',
                 output_extension='.mp4'):
        self.frames_per_second = frames_per_second
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.output_time_limit = output_time_limit
        self.output_folder = output_folder
        self.base_output_filename = base_output_filename
        self.output_extension = output_extension
        self.stopped = True
        self.saving_thread = None
        self.video_writer = None
        self.max_frame_count = self.frames_per_second * self.output_time_limit
        self.new_frame_condition = Condition()

    def receive_msg(self, msg, topic):
        if topic != topics.TOPIC_IMAGE_ARRAY:
            return
        if not self.video_writer:
            return
        self.latest_frame = msg.image
        with self.new_frame_condition:
            self.new_frame_condition.notify_all()

    def get_video_writer(self, path, fps, width, height):
        print(f'Writing video to {path}')
        return cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))
        # return imageio.get_writer(
        #     path,
        #     fps=fps,
        # )

    def save_video(self):
        print('Saving video')
        # print(f'Saving video to {self.video_writer._filename}')
        self.video_writer.release()
        # self.video_writer.close()

    def save_in_parallel(self):
        frame_count = 0
        video_count = 1
        video_path = get_output_path(
            folder=self.output_folder,
            base_filename=self.base_output_filename,
            ext=self.output_extension,
            timestamp=get_timestamp(),
            index=video_count,
        )
        self.video_writer = self.get_video_writer(
            video_path,
            # '/run/user/'+str(os.getuid())+'/video_path_'+str(video_count)+'.mp4',
            fps=self.frames_per_second,
            width=self.frame_width,
            height=self.frame_height
        )
        while not self.stopped:
            with self.new_frame_condition:
                self.new_frame_condition.wait()
            # start_time = time.time()
            self.video_writer.write(self.latest_frame)
            # self.video_writer.append_data(self.latest_frame)
            frame_count += 1
            if frame_count < self.max_frame_count:
                # print(time.time()-start_time)
                continue
            self.save_video()
            video_count += 1
            frame_count = 0
            video_path = get_output_path(
                folder=self.output_folder,
                base_filename=self.base_output_filename,
                ext=self.output_extension,
                timestamp=get_timestamp(),
                index=video_count,
            )
            self.video_writer = self.get_video_writer(
                video_path,
                # '/run/user/'+str(os.getuid())+'/video_path_'+str(video_count)+'.mp4',
                fps=self.frames_per_second,
                width=self.frame_width,
                height=self.frame_height,
            )
        else:
            self.save_video()

    def start(self):
        self.stopped = False
        self.saving_thread = Thread(
            target=self.save_in_parallel,
            args=()
        )
        self.saving_thread.daemon = True
        self.saving_thread.start()

    def stop(self):
        self.stopped = True
