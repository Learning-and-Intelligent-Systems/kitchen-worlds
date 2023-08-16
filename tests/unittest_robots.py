#!/usr/bin/env python

from __future__ import print_function
import shutil
import pickle
import os
import time
import random
import copy
import json
from os.path import join, abspath, dirname, isdir, isfile, basename
from config import EXP_PATH, OUTPUT_PATH

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.algorithms.meta import solve, create_parser

from pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, wait_for_duration, has_gui, reset_simulation, set_random_seed, \
    set_numpy_seed, set_renderer
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, get_datetime
from pybullet_tools.pr2_agent import solve_multiple, post_process, pddlstream_from_state_goal, \
    create_cwd_saver, solve_one
from pybullet_tools.pr2_primitives import control_commands, apply_commands
from pybullet_tools.logging import parallel_print, myprint

from lisdf_tools.lisdf_loader import pddl_files_from_dir

from world_builder.world import State
from world_builder.actions import apply_actions
from world_builder.world_generator import save_to_outputs_folder

from test_world_builder import create_pybullet_world

# additional imports
import argparse
from world_builder.builders import initialize_pybullet
from world_builder.utils import get_domain_constants, parse_yaml
from world_builder.world import World
from world_builder.robot_builders import get_robot_builder, create_gripper_robot, create_pr2_robot
from pybullet_tools.utils import set_all_color, GREEN, RED
from pybullet_tools.flying_gripper_utils import set_se3_conf, get_se3_conf

#####################################

def get_config(config_name):
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, default=config_name)
    args = parser.parse_args()
    config = parse_yaml(args.config)
    return config


def main():

    DEFAULT_YAML = "/home/weiyu/Research/nsplan/original/kitchen-worlds/configs/test/feg.yaml"
    # DEFAULT_YAML = "/home/weiyu/Research/nsplan/original/kitchen-worlds/configs/test/pr2.yaml"
    config = get_config(DEFAULT_YAML)

    """ ============== initiate simulator ==================== """
    initialize_pybullet(config)
    constants = get_domain_constants(config.planner.domain_pddl)
    world = World(time_step=config.time_step, camera=config.camera, segment=config.segment,
                  constants=constants)

    robot_builder = get_robot_builder(config.robot.builder_name)
    robot = robot_builder(world, config.robot.robot_name, **config.robot.builder_kwargs)

    if config.robot.robot_name == "feg":
        print(robot.get_gripper_joints())
        print(robot.get_positions())
        set_all_color(robot, RED)
        print(get_se3_conf(robot))
        # first three numbers are xyz, last three are rotations
        set_se3_conf(robot, (0, 0, 0, 1.57, -1.57, 0.0))

    input("next")


if __name__ == '__main__':
    main()



