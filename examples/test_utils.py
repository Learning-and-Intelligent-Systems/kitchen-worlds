from __future__ import print_function
import os
import json
import csv
from collections import defaultdict
import argparse
from os.path import join, abspath, basename, isdir, isfile
from os import listdir
import shutil

from pybullet_tools.utils import connect, draw_pose, unit_pose, link_from_name, load_pybullet, load_model, \
    sample_aabb, AABB, set_pose, get_aabb, get_aabb_center, quat_from_euler, Euler, HideOutput, get_aabb_extent, \
    set_camera_pose, wait_unlocked, disconnect, wait_if_gui, create_box, wait_for_duration, \
    SEPARATOR, get_aabb, get_pose, approximate_as_prism, draw_aabb, multiply, unit_quat, remove_body, invert, \
    Pose, get_link_pose, get_joint_limits, WHITE, RGBA, set_all_color, RED, GREEN, set_renderer, clone_body, \
    add_text, joint_from_name, set_caching, Point, set_random_seed, set_numpy_seed, reset_simulation, \
    get_joint_name, get_link_name, dump_joint, set_joint_position, ConfSaver, pairwise_link_collision
from pybullet_tools.bullet_utils import nice
from pybullet_tools.pr2_problems import create_floor

from world_builder.robot_builders import build_skill_domain_robot

from tutorials.config import EXP_PATH, modify_file_by_project

from pddlstream.algorithms.meta import solve, create_parser


def init_experiment(exp_dir):
    from pybullet_tools.logging import TXT_FILE
    if isfile(TXT_FILE):
        os.remove(TXT_FILE)


def get_test_world(robot='feg', semantic_world=False, draw_origin=False,
                   width=1980, height=1238, **kwargs):
    connect(use_gui=True, shadows=False, width=width, height=height)  ##  , width=360, height=270
    if draw_origin:
        draw_pose(unit_pose(), length=.5)
        create_floor()
    set_caching(cache=False)
    if semantic_world:
        from world_builder.world import World
        world = World()
    else:
        from lisdf_tools.lisdf_loader import World
        world = World()
    build_skill_domain_robot(world, robot, **kwargs)
    return world


def get_test_base_parser(task_name=None, parallel=False, use_viewer=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', type=str, default=task_name)
    parser.add_argument('-p', action='store_true', default=parallel)
    parser.add_argument('-v', '--viewer', action='store_true', default=use_viewer,
                        help='When enabled, enables the PyBullet viewer.')
    return parser


def get_parser(exp_name=None):
    parser = create_parser()
    parser.add_argument('-test', type=str, default=exp_name, help='Name of the test case')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions during planning')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    return parser


def get_args(exp_name=None):
    parser = get_parser(exp_name=exp_name)
    args = parser.parse_args()
    print('Arguments:', args)
    return args


###########################################################################


def save_csv(csv_file, data):
    csv_file = modify_file_by_project(csv_file)
    col_names = list(data.keys())
    col_data = list(data.values())

    file_exists = isfile(csv_file)
    with open(csv_file, 'a') as csvfile:
        writer = csv.writer(csvfile)
        if not file_exists:
            writer.writerow(col_names)
        for row in zip(*col_data):
            writer.writerow(row)


def read_csv(csv_file, summarize=True):
    from tabulate import tabulate
    csv_file = modify_file_by_project(csv_file)
    keys = None
    data = {}
    with open(csv_file, 'r') as csvfile:
        reader = csv.reader(csvfile)
        for j, row in enumerate(reader):
            if j == 0:
                keys = row
                data = defaultdict(list)
            else:
                for i, elem in enumerate(row):
                    data[keys[i]].append(elem if i == 0 else eval(elem))
                    if i == 1 and elem == 0:  ## failed run
                        break

    ## summarize the average, min, max, and count of each column
    if summarize:
        stats = [["name", "avg", "min", "max", "count"]]
        for key, value in data.items():
            if key in ["date"]: continue
            numbers = [sum(value) / len(value), min(value), max(value), len(value)]
            stats.append([key] + [nice(n) for n in numbers])
        print(tabulate(stats, headers="firstrow"))

    return data


########################################################################


def has_srl_stream():
    try:
        import srl_stream
    except ImportError:
        print('Unfortunately, you cant use the library unless you are part of NVIDIA Seattle Robotics lab')
        return False
    return True


def has_getch():
    try:
        import getch
    except ImportError:
        print('Please install has_getch in order to use `step_by_step`: ```pip install getch```\n')
        return False
    return True
