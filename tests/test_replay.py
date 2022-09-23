#!/usr/bin/env python

from __future__ import print_function
from ipaddress import v4_int_to_packed
import os
import json
import pickle
import shutil
from os import listdir
from os.path import join, abspath, dirname, isdir, isfile
from tabnanny import verbose
from config import EXP_PATH
import numpy as np
import random
import time
import sys

from pybullet_tools.pr2_utils import get_group_conf
from pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, wait_for_duration, safe_remove, ensure_dir, reset_simulation, \
    VideoSaver, wait_unlocked
from lisdf_tools.lisdf_loader import load_lisdf_pybullet, pddlstream_from_dir
from lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.actions import apply_actions

from mamao_tools.utils import get_feasibility_checker, get_plan

PARALLEL = False
SAVE_MP4 = False
AUTO_PLAY = False
EVALUATE_QUALITY = True

GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/one_fridge_pick_pr2/one_fridge_pr2_0921_220304'
TASK_NAME = 'one_fridge_pick_pr2'  ## 'one_fridge_pick_pr2_20_parallel_1'

TASK_NAME = 'mm_one_fridge_pick'
TASK_NAME = 'mm_one_fridge_table_in'
TASK_NAME = 'mm_one_fridge_table_on'
TASK_NAME = 'mm_one_fridge_table_pick'
TASK_NAME = 'mm_two_fridge_pick'
TASK_NAME = 'mm_two_fridge_in'

# TASK_NAME = 'tt_one_fridge_pick'
# TASK_NAME = 'tt_one_fridge_table_in'
# TASK_NAME = 'tt_two_fridge_in'

# TASK_NAME = '_examples'
# TASK_NAME = 'elsewhere'
TASK_NAME = 'discarded'

DATABASE_DIR = join('..', '..', 'fastamp-data')
# DATABASE_DIR = join('..', '..', 'mamao-data')


#####################################


def query_yes_no(question, default="no"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
            It must be "yes" (the default), "no" or None (meaning
            an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' " "(or 'y' or 'n').\n")


def run_one(run_dir, task_name=TASK_NAME, save_mp4=SAVE_MP4, width=1440, height=1120):
    ori_dir = run_dir  ## join(DATABASE_DIR, run_dir)

    print(f'\n\n\n--------------------------\n    replay {ori_dir} \n------------------------\n\n\n')
    run_name = os.path.basename(ori_dir)
    exp_dir = join(EXP_PATH, f"{task_name}_{run_name}")
    if isdir(exp_dir):
        shutil.rmtree(exp_dir)
    if not isdir(exp_dir):
        shutil.copytree(ori_dir, exp_dir)
    plan = get_plan(run_dir)

    world = load_lisdf_pybullet(exp_dir, width=width, height=height, verbose=True)
    problem = Problem(world)
    wait_unlocked()

    commands = pickle.load(open(join(exp_dir, 'commands.pkl'), "rb"))

    if save_mp4:
        video_path = join(ori_dir, 'replay.mp4')
        with VideoSaver(video_path):
            apply_actions(problem, commands, time_step=0.025, verbose=False, plan=plan)
        print('saved to', abspath(video_path))
    else:
        # wait_if_gui(f'start replay {run_name}?')
        answer = True
        if not AUTO_PLAY:
            answer = query_yes_no(f"start replay {run_name}?", default='yes')
        if answer:
            apply_actions(problem, commands, time_step=0.05, verbose=False, plan=plan)

        if EVALUATE_QUALITY:
            answer = query_yes_no(f"delete this run {run_name}?", default='no')
            if answer:
                new_dir = join(DATABASE_DIR, 'impossible', f"{task_name}_{run_name}")
                shutil.move(run_dir, new_dir)
                print(f"moved {run_dir} to {new_dir}")

        # wait_if_gui('replay next run?')

    # disconnect()
    reset_simulation()
    shutil.rmtree(exp_dir)


def process(index):
    np.random.seed(int(time.time()))
    random.seed(time.time())
    return run_one(str(index))


def main(parallel=True, cases=None, path=None):
    if isdir('visualizations'):
        shutil.rmtree('visualizations')

    start_time = time.time()
    dataset_dir = join(DATABASE_DIR, TASK_NAME)
    if cases is None:
        cases = [join(dataset_dir, f) for f in listdir(dataset_dir) if isdir(join(dataset_dir, f))]
        cases.sort()
    else:
        cases = [join(dataset_dir, f) for f in cases if isdir(join(dataset_dir, f))]
    if path is not None:
        cases = [path]

    num_cases = len(cases)

    if parallel:
        import multiprocessing
        from multiprocessing import Pool

        max_cpus = 24
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus')
        with Pool(processes=num_cpus) as pool:
            pool.map(process, cases)

    else:
        for i in range(num_cases):
            process(cases[i])

    print(f'solved {num_cases} problems (parallel={parallel}) in {round(time.time() - start_time, 3)} sec')


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


if __name__ == '__main__':
    main(parallel=PARALLEL, cases=['0']) ##
    # main(parallel=PARALLEL, path=GIVEN_PATH)
