from os import listdir
from os.path import join, abspath, dirname, basename, isdir, isfile
import os
import math
import json
from config import EXP_PATH, MAMAO_DATA_PATH, DATA_CONFIG_PATH, PBP_PATH
import numpy as np
import random


def mp4_to_gif(mp4_file, frame_folder='output'):
    import cv2
    def convert_mp4_to_jpgs(path):
        video_capture = cv2.VideoCapture(path)
        still_reading, image = video_capture.read()
        frame_count = 0
        while still_reading:
            cv2.imwrite(f"{frame_folder}/frame_{frame_count:03d}.jpg", image)

            # read next image
            still_reading, image = video_capture.read()
            frame_count += 1

    import glob
    from PIL import Image

    def make_gif():
        images = glob.glob(f"{frame_folder}/*.jpg")
        images.sort()
        frames = [Image.open(image) for image in images]
        frame_one = frames[0]
        output_file = mp4_file.replace('.mp4', '.gif')
        frame_one.save(output_file, format="GIF", append_images=frames,
                       save_all=True, duration=50, loop=0)
        return output_file

    convert_mp4_to_jpgs(mp4_file)
    output_file = make_gif()
    print('converted mp4 to', output_file)


##################################################################################


def get_envs_from_task(task_dir = join(MAMAO_DATA_PATH, 'tt_two_fridge_pick')):
    ori_dirs = [join(task_dir, f) for f in listdir(task_dir) if isdir(join(task_dir, f))]
    ori_dirs.sort()
    return ori_dirs