#!/usr/bin/env python

from __future__ import print_function
from ipaddress import v4_int_to_packed
import os
import json
import math
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

from pybullet_tools.bullet_utils import get_datetime
from pybullet_tools.utils import reset_simulation, VideoSaver, wait_unlocked
from lisdf_tools.lisdf_loader import load_lisdf_pybullet, pddlstream_from_dir
from lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem
from world_builder.actions import apply_actions

from mamao_tools.utils import get_plan

from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser, \
    query_yes_no, get_body_map

USE_GYM = False
SAVE_MP4 = False
STEP_BY_STEP = False
AUTO_PLAY = True
EVALUATE_QUALITY = True

GIVEN_PATH = None
# GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/one_fridge_pick_pr2/one_fridge_pick_pr2_1004_01:29_1'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data/_examples/5/rerun_2/diverse_commands_rerun_fc=pvt-all.pkl'
GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/0104_094417_original_1'
GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_storage/397'

GIVEN_DIR = None
# GIVEN_DIR = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen_100'

TASK_NAME = 'one_fridge_pick_pr2'

# TASK_NAME = 'mm_one_fridge_table_in'
# TASK_NAME = 'mm_one_fridge_table_on'
# TASK_NAME = 'mm_one_fridge_table_pick'
# TASK_NAME = 'mm_two_fridge_pick'
TASK_NAME = 'mm_two_fridge_in'

# TASK_NAME = 'tt_one_fridge_pick'
# TASK_NAME = 'tt_one_fridge_table_in'
# TASK_NAME = 'tt_two_fridge_pick'
# TASK_NAME = 'tt_two_fridge_in'

# TASK_NAME = 'ff_one_fridge_table_pick'
# TASK_NAME = 'ff_two_fridge_pick'

# TASK_NAME = '_examples'
# TASK_NAME = 'elsewhere'
# TASK_NAME = 'discarded'

CASES = None
CASES = ['104', '186']

parser = get_base_parser(task_name=TASK_NAME, parallel=False, use_viewer=True)
args = parser.parse_args()

#####################################


def get_pkl_run(run_dir, verbose=True):
    pkl_file = 'commands.pkl'
    if run_dir.endswith('.pkl'):
        pkl_file = basename(run_dir)
        run_dir = run_dir[:-len(pkl_file) - 1]
        rerun_dir = basename(run_dir)
        run_dir = run_dir[:-len(rerun_dir) - 1]
        pkl_file = join(rerun_dir, pkl_file)

    exp_dir = copy_dir_for_process(run_dir, tag='replaying', verbose=verbose)
    if 'rerun' in pkl_file:
        plan_json = join(run_dir, pkl_file).replace('commands', 'plan').replace('.pkl', '.json')
        plan = get_plan(run_dir, plan_json=plan_json)
    else:
        plan = get_plan(run_dir)
    commands = pickle.load(open(join(exp_dir, pkl_file), "rb"))
    return exp_dir, run_dir, commands, plan


def run_one(run_dir, task_name=TASK_NAME, save_mp4=SAVE_MP4, width=1440, height=1120,
            camera_point=(8.5, 2.5, 3), camera_target=(0, 2.5, 0)):

    if 'full_kitchen' in run_dir:
        camera_point = (4, 4, 8)
        camera_target = (0, 4, 0)

    exp_dir, run_dir, commands, plan = get_pkl_run(run_dir)

    world = load_lisdf_pybullet(exp_dir, use_gui=not USE_GYM, width=width, height=height, verbose=False)
    problem = Problem(world)
    world.summarize_all_objects()
    body_map = get_body_map(run_dir, world)

    if USE_GYM:
        from isaac_tools.gym_utils import load_lisdf_isaacgym, record_actions_in_gym
        gym_world = load_lisdf_isaacgym(abspath(exp_dir), camera_width=1280, camera_height=800,
                                        camera_point=camera_point, camera_target=camera_target)
        img_dir = join(exp_dir, 'gym_images')
        gif_name = 'gym_replay.gif'
        os.mkdir(img_dir)
        gif_name = record_actions_in_gym(problem, commands, gym_world, img_dir=img_dir, body_map=body_map,
                                         gif_name=gif_name, time_step=0, verbose=False, plan=plan)
        gym_world.wait_if_gui()
        shutil.copy(join(exp_dir, gif_name), join(run_dir, gif_name))
        print('moved gif to {}'.format(join(run_dir, gif_name)))

    elif save_mp4:
        video_path = join(run_dir, 'replay.mp4')
        with VideoSaver(video_path):
            apply_actions(problem, commands, time_step=0.025, verbose=False,
                          plan=plan)
        print('saved to', abspath(video_path))

    else:

        run_name = basename(run_dir)
        if not AUTO_PLAY:
            wait_unlocked()
            # wait_if_gui(f'start replay {run_name}?')
        answer = True
        if not AUTO_PLAY:
            answer = query_yes_no(f"start replay {run_name}?", default='yes')
        if answer:
            time_step = 0.02 if not STEP_BY_STEP else None
            apply_actions(problem, commands, time_step=time_step, verbose=True,
                          plan=plan, body_map=body_map)

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


def process(index):
    np.random.seed(int(time.time()))
    random.seed(time.time())
    return run_one(str(index))


def merge_all_wconfs(all_wconfs):
    longest_command = max([len(wconf) for wconf in all_wconfs])
    whole_wconfs = []
    for t in range(longest_command):
        whole_wconf = {}
        for num in range(len(all_wconfs)):
            if t < len(all_wconfs[num]):
                whole_wconf.update(all_wconfs[num][t])
        whole_wconfs.append(whole_wconf)
    return whole_wconfs


def replay_all_in_gym(width=1440, height=1120, num_rows=5, num_cols=5, world_size=(6, 6), verbose=False,
                      frame_gap=6, debug=False, loading_effect=False, save_gif=True, save_mp4=False,
                      camera_motion=None):
    from test_gym import get_dirs_camera
    from isaac_tools.gym_utils import load_envs_isaacgym, record_actions_in_gym, \
        update_gym_world_by_wconf, images_to_gif, images_to_mp4
    from tqdm import tqdm

    img_dir = join('gym_images')
    gif_name = 'gym_replay_batch_gym.gif'
    # if isdir(img_dir):
    #     shutil.rmtree(img_dir)
    # os.mkdir(img_dir)

    data_dir = 'test_full_kitchen_100' if loading_effect else 'test_full_kitchen_sink'
    ori_dirs, camera_point_begin, camera_point_final, camera_target = get_dirs_camera(
        num_rows, num_cols, world_size, data_dir=data_dir, camera_motion=camera_motion)
    lisdf_dirs = [copy_dir_for_process(ori_dir, verbose=verbose) for ori_dir in ori_dirs]
    num_worlds = min([len(lisdf_dirs), num_rows * num_cols])

    ## translate commands into world_confs
    all_wconfs = []

    if loading_effect:
        ### load all gym_worlds and return all wconfs
        gym_world, offsets, all_wconfs = load_envs_isaacgym(lisdf_dirs, num_rows=num_rows, num_cols=num_cols,
                                                            world_size=world_size, loading_effect=True, verbose=verbose,
                                                            camera_point=camera_point_begin, camera_target=camera_target)
    else:
        for i in tqdm(range(num_worlds)):
            exp_dir, run_dir, commands, plan = get_pkl_run(lisdf_dirs[i], verbose=verbose)
            world = load_lisdf_pybullet(exp_dir, use_gui=not USE_GYM or debug,
                                        width=width, height=height, verbose=False)
            body_map = get_body_map(run_dir, world)
            problem = Problem(world)
            wconfs = record_actions_in_gym(problem, commands, plan=plan, return_wconf=True,
                                           world_index=i, body_map=body_map)
            all_wconfs.append(wconfs)
            reset_simulation()
            if debug:
                wait_unlocked()

        ## load all scenes in gym
        gym_world, offsets = load_envs_isaacgym(lisdf_dirs, num_rows=num_rows, num_cols=num_cols, world_size=world_size,
                                                camera_point=camera_point_begin, camera_target=camera_target, verbose=verbose)

    all_wconfs = merge_all_wconfs(all_wconfs)
    print(f'\n\nrendering all {len(all_wconfs)} frames')

    ## update all scenes for each frame
    filenames = []
    for i in tqdm(range(len(all_wconfs))):
        if camera_point_final != camera_point_begin:
            if isinstance(camera_point_final, tuple) and isinstance(camera_point_final, tuple):
                camera_point = tuple([camera_point_begin[j] + (camera_point_final[j] - camera_point_begin[j]) * i / len(all_wconfs)
                                      for j in range(3)])
            else:
                ## rotate camera around point begin by radius of camera_point_final
                dx = camera_point_final * math.sin(2 * math.pi * i / len(all_wconfs))
                dy = camera_point_final * math.cos(2 * math.pi * i / len(all_wconfs))
                offset = [dx, dy, 0]
                camera_point = tuple([camera_point_begin[j] + offset[j] for j in range(3)])
            gym_world.set_camera_target(gym_world.cameras[0], camera_point, camera_target)
        update_gym_world_by_wconf(gym_world, all_wconfs[i], offsets=offsets)
        if i % frame_gap == 0:
            img_file = gym_world.get_rgba_image(gym_world.cameras[0])
            filenames.append(img_file)

    if save_gif:
        images_to_gif(img_dir, gif_name, filenames)
        print('created gif {}'.format(join(img_dir, gif_name)))
    if save_mp4:
        mp4_name = join(img_dir, gif_name.replace('.gif', f'_{get_datetime()}.mp4'))
        images_to_mp4(filenames, mp4_name=mp4_name)
        print('created mp4 {} with {} frames'.format(mp4_name, len(filenames)))


if __name__ == '__main__':
    # replay_all_in_gym(num_rows=14, num_cols=14, world_size=(6, 6), save_gif=True)
    process_all_tasks(process, args.t, cases=CASES, path=GIVEN_PATH, dir=GIVEN_DIR)

    ## record 1 : 250+ worlds
    # replay_all_in_gym(num_rows=32, num_cols=8, world_size=(4, 8), loading_effect=True,
    #                   frame_gap=1, save_mp4=True, save_gif=False, verbose=False, camera_motion='zoom')

    ## record 2 : robot execution
    # replay_all_in_gym(num_rows=8, num_cols=3, world_size=(4, 8), loading_effect=False,
    #                   frame_gap=2, save_mp4=True, save_gif=False, verbose=False, camera_motion='splotlight')
