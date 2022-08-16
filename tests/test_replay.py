#!/usr/bin/env python

from __future__ import print_function
import os
import json
import pickle
import shutil
from os import listdir
from os.path import join, abspath, dirname, isdir, isfile
from config import EXP_PATH
import numpy as np
import random
import time

from pybullet_tools.pr2_utils import get_group_conf
from pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, wait_for_duration, safe_remove, ensure_dir, reset_simulation, \
    VideoSaver
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, get_datetime
from pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn, \
    get_stream_map # , solve_multiple, solve_one
from pybullet_tools.logging import TXT_FILE

from pybullet_tools.pr2_primitives import get_group_joints, Conf, get_base_custom_limits, Pose, Conf, \
    get_ik_ir_gen, get_motion_gen, get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, Command, \
    get_gripper_joints, GripperCommand, State

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.algorithms.meta import solve, DEFAULT_ALGORITHM
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.algorithm import reset_globals
from pddlstream.algorithms.common import SOLUTIONS
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, TmpCWD

from lisdf_tools.lisdf_loader import load_lisdf_pybullet, pddlstream_from_dir
from lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.actions import apply_actions

from mamao_tools.utils import get_feasibility_checker

PARALLEL = False
TASK_NAME = 'one_fridge_pick_pr2'  ## 'one_fridge_pick_pr2_20_parallel_1'
DATABASE_DIR = join('..', '..', 'mamao-data', TASK_NAME)


#####################################


def run_one(run_dir, task_name=TASK_NAME, save_mp4=False):
    ori_dir = run_dir ## join(DATABASE_DIR, run_dir)

    print(f'\n\n\n--------------------------\n    replay {ori_dir} \n------------------------\n\n\n')
    run_name = os.path.basename(ori_dir)
    exp_dir = join(EXP_PATH, f"{task_name}_{run_name}")
    if not isdir(exp_dir):
        shutil.copytree(ori_dir, exp_dir)

    world = load_lisdf_pybullet(exp_dir, width=720, height=560)
    problem = Problem(world)

    commands = pickle.load(open(join(exp_dir, 'commands.pkl'), "rb"))

    if save_mp4:
        video_path = join(ori_dir, 'replay.mp4')
        with VideoSaver(video_path):
            apply_actions(problem, commands, time_step=0.025)
        print('saved to', abspath(video_path))
    else:
        wait_if_gui('start replay?')
        apply_actions(problem, commands, time_step=0.05)
        wait_if_gui('replay next run?')

    # disconnect()
    reset_simulation()
    shutil.rmtree(exp_dir)


def process(index):
    np.random.seed(int(time.time()))
    random.seed(time.time())
    return run_one(str(index))


def main(PARALLEL=True, cases=None):
    if isdir('visualizations'):
        shutil.rmtree('visualizations')

    start_time = time.time()
    if cases is None:
        cases = [join(DATABASE_DIR, f) for f in listdir(DATABASE_DIR) if isdir(join(DATABASE_DIR, f))]
    else:
        cases = [join(DATABASE_DIR, f) for f in cases if isdir(join(DATABASE_DIR, f))]
    num_cases = len(cases)

    if PARALLEL:
        import multiprocessing
        from multiprocessing import Pool

        max_cpus = 24
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus')
        with Pool(processes=num_cpus) as pool:
            pool.map(process, cases)

    else:
        for i in range(num_cases):
            # if i in [0, 1]: continue
            process(cases[i])

    print(f'solved {num_cases} problems (parallel={PARALLEL}) in {round(time.time() - start_time, 3)} sec')


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
    main(PARALLEL=PARALLEL, cases=['2137'])
