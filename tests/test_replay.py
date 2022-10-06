#!/usr/bin/env python

from __future__ import print_function
from ipaddress import v4_int_to_packed
import os
import json
import pickle
import shutil
from os import listdir
from os.path import join, abspath, dirname, isdir, isfile, basename
from tabnanny import verbose
from config import EXP_PATH, MAMAO_DATA_PATH
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

from world_builder.actions import adapt_action, apply_actions
from world_builder.world import State
from world_builder.actions import Action, AttachObjectAction
from pybullet_tools.pr2_primitives import Trajectory, Command

from mamao_tools.utils import get_feasibility_checker, get_plan

from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser

USE_GYM = False
SAVE_MP4 = False
AUTO_PLAY = True
EVALUATE_QUALITY = True

GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/one_fridge_pick_pr2/one_fridge_pick_pr2_1004_01:29_1'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data/tt_one_fridge_table_pick/0/rerun_2/diverse_commands_rerun_fc=oracle.pkl'
TASK_NAME = 'one_fridge_pick_pr2'

# TASK_NAME = 'mm_one_fridge_table_in'
# TASK_NAME = 'mm_one_fridge_table_on'
# TASK_NAME = 'mm_one_fridge_table_pick'
# TASK_NAME = 'mm_two_fridge_pick'
# TASK_NAME = 'mm_two_fridge_in'

# TASK_NAME = 'tt_one_fridge_pick'
# TASK_NAME = 'tt_one_fridge_table_in'
TASK_NAME = 'tt_two_fridge_pick'
# TASK_NAME = 'tt_two_fridge_in'

# TASK_NAME = 'ff_one_fridge_table_pick'
# TASK_NAME = 'ff_two_fridge_pick'

# TASK_NAME = '_examples'
# TASK_NAME = 'elsewhere'
# TASK_NAME = 'discarded'

CASES = None
CASES = ['5']

parser = get_base_parser(task_name=TASK_NAME, parallel=False, use_viewer=True)
args = parser.parse_args()

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
    pkl_file = 'commands.pkl'
    if run_dir.endswith('.pkl'):
        pkl_file = basename(run_dir)
        run_dir = run_dir[:-len(pkl_file)-1]
        rerun_dir = basename(run_dir)
        run_dir = run_dir[:-len(rerun_dir)-1]
        pkl_file = join(rerun_dir, pkl_file)
    run_name = basename(run_dir)
    exp_dir = copy_dir_for_process(run_dir, tag='replaying')
    plan = get_plan(run_dir)

    world = load_lisdf_pybullet(exp_dir, use_gui=not USE_GYM, width=width, height=height, verbose=False)
    problem = Problem(world)

    commands = pickle.load(open(join(exp_dir, pkl_file), "rb"))

    if USE_GYM:
        from test_gym import load_lisdf_isaacgym
        gym_world = load_lisdf_isaacgym(os.path.abspath(exp_dir),
                                        camera_width=1280, camera_height=800)
        img_dir = join(exp_dir, 'gym_images')
        gif_name = 'gym_replay.gif'
        os.mkdir(img_dir)
        gif_name = record_actions_in_gym(problem, commands, gym_world, img_dir=img_dir,
                                         gif_name=gif_name, time_step=0, verbose=False, plan=plan)
        shutil.copy(join(exp_dir, gif_name), join(run_dir, gif_name))
        print('moved gif to {}'.format(join(run_dir, gif_name)))

    elif save_mp4:
        video_path = join(run_dir, 'replay.mp4')
        with VideoSaver(video_path):
            apply_actions(problem, commands, time_step=0.025, verbose=False, plan=plan)
        print('saved to', abspath(video_path))

    else:
        if not AUTO_PLAY:
            wait_unlocked()
            # wait_if_gui(f'start replay {run_name}?')
        answer = True
        if not AUTO_PLAY:
            answer = query_yes_no(f"start replay {run_name}?", default='yes')
        if answer:
            apply_actions(problem, commands, time_step=0.02, verbose=False, plan=plan)

        if EVALUATE_QUALITY:
            answer = query_yes_no(f"delete this run {run_name}?", default='no')
            if answer:
                new_dir = join(MAMAO_DATA_PATH, 'impossible', f"{task_name}_{run_name}")
                shutil.move(run_dir, new_dir)
                print(f"moved {run_dir} to {new_dir}")

        # wait_if_gui('replay next run?')

    # disconnect()
    reset_simulation()
    shutil.rmtree(exp_dir)


def record_actions_in_gym(problem, actions, gym_world, img_dir=None, gif_name='gym_replay.gif',
                          time_step=0.5, verbose=False, plan=None):
    """ act out the whole plan and event in the world without observation/replanning """
    from test_gym import update_gym_world
    if actions is None:
        return
    state_event = State(problem.world)
    camera = gym_world.cameras[0]
    filenames = []
    frame_gap = 3
    for i, action in enumerate(actions):
        if verbose:
            print(i, action)
        action = adapt_action(action, problem, plan)
        if action is None:
            continue
        state_event = action.transition(state_event.copy())
        if isinstance(action, AttachObjectAction):
            print(action.grasp)
        wait_for_duration(time_step)

        """ update gym world """
        update_gym_world(gym_world, problem.world)
        if img_dir is not None and i % frame_gap == 0:
            # img_file = join(img_dir, f'{i}.png')
            # gym_world.get_rgba_image(camera, image_type='rgb', filename=img_file)  ##
            img_file = gym_world.get_rgba_image(camera)
            filenames.append(img_file)

    import imageio
    start = time.time()
    gif_file = join(img_dir, '..', gif_name)
    print(f'saving to {abspath(gif_file)} with {len(filenames)} frames')
    with imageio.get_writer(gif_file, mode='I') as writer:
        for filename in filenames:
            # image = imageio.imread(filename)
            writer.append_data(filename)

    print(f'saved to {abspath(gif_file)} with {len(filenames)} frames in {round(time.time() - start, 2)} seconds')
    return gif_name


def process(index):
    np.random.seed(int(time.time()))
    random.seed(time.time())
    return run_one(str(index))


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
    process_all_tasks(process, args.t, cases=CASES)
    # process_all_tasks(process, args.t, path=GIVEN_PATH)
