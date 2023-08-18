#!/usr/bin/env python

from __future__ import print_function

import itertools
import shutil
import pickle
import os
import time
import random
import copy
import json
from os.path import join, abspath, dirname, isdir, isfile, basename
from config import EXP_PATH, OUTPUT_PATH
from itertools import product
from PIL import Image
import math


from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.algorithms.meta import solve, create_parser

from pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, wait_for_duration, has_gui, reset_simulation, set_random_seed, \
    set_numpy_seed, set_renderer, pairwise_collision
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, get_datetime, collided
from pybullet_tools.pr2_agent import solve_multiple, post_process, pddlstream_from_state_goal, \
    create_cwd_saver, solve_one
from pybullet_tools.pr2_primitives import control_commands, apply_commands
from pybullet_tools.logging import parallel_print, myprint

from lisdf_tools.lisdf_loader import pddl_files_from_dir

from world_builder.world import State
from world_builder.actions import apply_actions
from world_builder.world_generator import save_to_outputs_folder

from test_utils import parallel_processing, get_config
from test_world_builder import create_pybullet_world

from nsplan_tools.generate_semantic_specification import get_semantic_specs

# additional dependencies for using streams
from pybullet_tools.bullet_utils import set_camera_target_body, visualize_camera_image, get_readable_list
from pybullet_planning.pybullet_tools.utils import get_image_at_pose, get_image, unit_pose, get_camera_matrix
import matplotlib.pyplot as plt
from world_builder.entities import StaticCamera
from world_builder.utils import parse_yaml
from pybullet_tools.pr2_primitives import Pose, Conf
from pybullet_tools.utils import get_pose, multiply, quat_from_euler, dump_world, get_bodies, remove_body, invert, Euler, Point
from pybullet_tools.flying_gripper_utils import get_se3_joints, se3_from_pose
from world_builder.actions import get_primitive_actions
from pybullet_tools.flying_gripper_utils import se3_ik, Grasp

# gym-related
import gymnasium as gym
from gymnasium import error, spaces, utils
from gymnasium.utils import seeding
import numpy as np

from collect_clean_dish_rollouts import CleanDishEnvV1


DEFAULT_YAML = '../configs/clean_dish_feg_collect_rollouts_cluster.yaml'
config = parse_yaml(DEFAULT_YAML)

#####################################

def main(config):

    seed = config.seed
    semantic_spec_seed = config.semantic_spec_seed

    new_config = copy.deepcopy(config)
    new_config.seed = seed
    set_random_seed(seed)
    set_numpy_seed(seed)

    exp_dir = abspath(join(config.data.out_dir, "test_grasp_in_world"))  # + get_datetime(TO_LISDF=True)))
    print(exp_dir)
    os.makedirs(exp_dir, exist_ok=True)
    new_config.data.out_dir = exp_dir

    new_config.world.builder_kwargs["semantic_spec_file"] = os.path.join(config.semantic_specs_dir,
                                                                         f"{semantic_spec_seed}.json")

    """ STEP 1 -- GENERATE SCENES """
    world, goal = create_pybullet_world(new_config, SAVE_LISDF=False, SAVE_TESTCASE=True)

    ## get movable objects
    moveable_bodies = world.cat_to_bodies('moveable')
    object_names = [world.body_to_name(b) for b in moveable_bodies]
    print("moveable object names", object_names)
    print("moveable bodies", moveable_bodies)

    saver = WorldSaver()
    state = State(world)
    robot = state.robot
    problem = state
    domain_path = abspath(config.planner.domain_pddl)
    stream_map = robot.get_stream_map(problem, collisions=not config.cfree,
                                                custom_limits=world.robot.custom_limits,
                                                teleport=config.teleport, domain_pddl=domain_path,
                                                num_grasp_samples=30)

    """ STEP 2 -- PLACE OBJECTS IN THE WORLD """

    # ----------------------------------
    # option 1
    # important: the placement streams do not check collision between moveable objects.
    # option one is to add movable objects

    # object_name = "mug#1"
    # # object_name = "bottle#1"
    # obj_body = world.name_to_body(object_name)
    #
    # surface_name = "sink_bottom"
    # surface = world.name_to_body(surface_name)
    #
    # if surface_name == "cabinettop_storage":
    #     placement_stream = stream_map["sample-pose-in"]
    # else:
    #     placement_stream = stream_map["sample-pose-on"]
    #
    # # important: this is how obstacles are obtained in the streams. So the streams do not check collision between
    # #            moveable objects.
    # old_obstacles = problem.fixed
    #
    # # important: our fix is add movable objects
    # fixed_obstacles = old_obstacles + moveable_bodies
    #
    # # this is in general_streams.get_stable_gen()
    # old_obs = [obst for obst in old_obstacles if obst not in {obj_body, surface}]
    # obs = [obst for obst in fixed_obstacles if obst not in {obj_body, surface}]
    # print("old obstacles", old_obs)
    # print("fixed obstacles", obs)
    #
    # for placement_pose in placement_stream(obj_body, surface):
    #     placement_pose = placement_pose[0][0]
    #     print(placement_pose)
    #
    #     p0 = Pose(obj_body, get_pose(obj_body), surface)
    #
    #     placement_pose.assign()
    #     result = collided(obj_body, obs, verbose=True, visualize=False, tag='stable_gen', world=world)
    #     print("collision checking result", result)
    #
    #     input("try next placement pose?")
    #     # restore the original pose
    #     p0.assign()
    # ----------------------------------


    # ----------------------------------
    # option 2
    # option 2 is to use cfree_pose_pose_stream to check collisions between moveable objects
    object_name = "mug#1"
    # object_name = "bottle#1"
    obj_body = world.name_to_body(object_name)

    surface_name = "sink_bottom"
    surface = world.name_to_body(surface_name)

    if surface_name == "cabinettop_storage":
        placement_stream = stream_map["sample-pose-in"]
    else:
        placement_stream = stream_map["sample-pose-on"]

    cfree_pose_pose_stream = stream_map["test-cfree-pose-pose"]

    obstacles = problem.fixed

    obs = [obst for obst in obstacles if obst not in {obj_body, surface}]
    print("fixed obstacles", obs)

    for placement_pose in placement_stream(obj_body, surface):
        placement_pose = placement_pose[0][0]
        print(placement_pose)

        p0 = Pose(obj_body, get_pose(obj_body), surface)

        placement_pose.assign()
        result = collided(obj_body, obs, verbose=True, visualize=False, tag='stable_gen', world=world)
        print("collision checking result", result)

        moveable_collision = False
        for mb in moveable_bodies:
            if mb == obj_body:
                continue
            mbp = Pose(mb, get_pose(mb))
            result = pairwise_collision(obj_body, mb)
            print(f"cfree_pose_pose_stream result between {obj_body} and {mb}: {result}")
            moveable_collision = moveable_collision or result
        print("moveable collsion result", moveable_collision)

        input("try next placement pose?")
        # restore the original pose
        p0.assign()

if __name__ == '__main__':

    config_file = "/home/weiyu/Research/nsplan/original/kitchen-worlds/configs/test/feg_collision_in_world.yaml"
    config = parse_yaml(config_file)
    main(config)



