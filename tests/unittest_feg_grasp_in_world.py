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
    print("moveable objects", object_names)

    """ STEP 2 -- Initialize the world and streams """
    # important
    saver = WorldSaver()

    state = State(world)
    robot = state.robot
    problem = state

    domain_path = abspath(config.planner.domain_pddl)
    stream_map = robot.get_stream_map(problem, collisions=not config.cfree,
                                                custom_limits=world.robot.custom_limits,
                                                teleport=config.teleport, domain_pddl=domain_path,
                                                num_grasp_samples=30)

    moveable_bodies = world.cat_to_bodies('moveable')
    object_names = [world.body_to_name(b) for b in moveable_bodies]
    print("moveable objects", object_names)

    """ STEP 3 -- Sample a grasp for each object """
    current_q = Conf(robot, get_se3_joints(robot))

    object_name = np.random.choice(object_names)

    instance_name = world.get_instance_name(object_name)
    print("instance name", instance_name)

    obj_body = world.name_to_body(object_name)
    world_obj = world.name_to_object(object_name)

    obj_pose = world_obj.get_link_pose(link=-1)
    obj_pose_2 = get_pose(obj_body)
    obj_Pose = Pose(obj_body, get_pose(obj_body))
    print("\n")
    print(f"world_obj.get_link_pose(link=-1): {obj_pose}")
    print(f"get_pose(obj_body): {obj_pose_2}")
    print(f"Pose(obj_body, get_pose(obj_body)): {obj_Pose}")
    input("confirm obj pose")

    grasp_list = next(stream_map["sample-grasp"](body=obj_body))
    print("sample {} grasps for {}: {}".format(len(grasp_list), world_obj, grasp_list))

    def clean_gripper(robot):
        """helper function to remove cloned gripper for visualizing grasps"""
        if "hand" in robot.grippers:
            gripper_body = robot.grippers["hand"]
            remove_body(gripper_body)
            robot.remove_gripper()

    # # ----------------------------------
    # # test the grasp
    # for grasp in grasp_list:
    #
    #     clean_gripper(robot)
    #
    #     grasp = grasp[0]
    #     gripper_grasp = robot.visualize_grasp(obj_pose, grasp.value, body=grasp.body, width=grasp.grasp_width)
    #     # gripper_approach = robot.visualize_grasp(body_pose, grasp.approach, color=RED)
    #     input("next grasp?")
    # # ----------------------------------

    # # ----------------------------------
    # # test ik stream
    # for grasp in grasp_list:
    #     print("find ik for grasp", grasp)
    #     for ik in stream_map["inverse-kinematics-hand"](a=None, o=obj_body, p=obj_Pose, g=grasp[0]):
    #         if len(ik) == 0:
    #             continue
    #         print("ik", ik)
    #         input("confirm ik found")
    # # ----------------------------------

    # # ----------------------------------
    # # test ik function which ik stream calls
    # # the problem is that the grasp pose is computed as grasp_pose = multiply(body_pose, grasp.value)
    # # however, in robots.visualize_grasp(), body_pose = robot.get_body_pose(body_pose, body=body, verbose=verbose)
    # # in ik function, flying_gripper_utils.get_approach_path(), body_pose = robot.get_body_pose(o, verbose=False)
    # # when body is none, body_pose is multiplied with robot.tool_from_hand, which is Pose(euler=Euler(math.pi / 2, 0, -math.pi / 2)). i.e., multiply(body_pose, r)
    #
    #
    # for grasp in grasp_list:
    #
    #     # # =================================
    #     # # before fixing the issue
    #     # clean_gripper(robot)
    #     #
    #     # grasp = grasp[0]
    #     # gripper_grasp = robot.visualize_grasp(obj_pose, grasp.value, body=grasp.body, width=grasp.grasp_width)
    #     # input("confirm visualized grasp pose")
    #     #
    #     # clean_gripper(robot)
    #     #
    #     # print("find ik for grasp", grasp)
    #     #
    #     # body_pose = robot.get_body_pose(obj_body, verbose=False)
    #     # print(f"robot.get_body_pose(obj_body, verbose=False): {body_pose}")
    #     # approach_pose = multiply(body_pose, grasp.approach)
    #     # grasp_pose = multiply(body_pose, grasp.value)
    #     # print(f"multiply(body_pose, grasp.value): {grasp_pose}")
    #     #
    #     # seconf1 = se3_ik(robot, grasp_pose)
    #     # input("confirm visualized ik for grasp")
    #     #
    #     # # =================================
    #
    #     # =================================
    #     # after fixing the issue
    #     clean_gripper(robot)
    #
    #     grasp = grasp[0]
    #     robot.visualize_grasp(obj_pose, grasp.value, body=grasp.body, width=grasp.grasp_width)
    #     input("confirm visualized grasp pose")
    #
    #     clean_gripper(robot)
    #
    #     print("find ik for grasp", grasp)
    #
    #     # let's adjust the grasp pose
    #     grasp_type = 'hand'
    #     tool_from_hand = (Point(), quat_from_euler(Euler(math.pi / 2, 0, -math.pi / 2)))
    #     g = multiply(invert(tool_from_hand), grasp.value)
    #     approach = multiply(invert(tool_from_hand), grasp.approach)
    #     adjusted_grasp = Grasp(grasp_type, obj_body, g, approach, robot.get_carry_conf(robot.arms[0], grasp_type, g))
    #
    #     body_pose = robot.get_body_pose(obj_body, verbose=False)
    #     print(f"robot.get_body_pose(obj_body, verbose=False): {body_pose}")
    #     approach_pose = multiply(body_pose, adjusted_grasp.approach)
    #     grasp_pose = multiply(body_pose, adjusted_grasp.value)
    #     print(f"multiply(body_pose, grasp.value): {grasp_pose}")
    #
    #     seconf1 = se3_ik(robot, grasp_pose)
    #     print("grasp ik solution", seconf1)
    #     input("confirm visualized ik for grasp")
    #
    #     # =================================
    #
    # # ----------------------------------

    def yield_grasp():
        for grasp in grasp_list:
            print("find ik for grasp", grasp)

            grasp = grasp[0]

            # let's adjust the grasp pose
            grasp_type = 'hand'
            tool_from_hand = (Point(), quat_from_euler(Euler(math.pi / 2, 0, -math.pi / 2)))
            g = multiply(invert(tool_from_hand), grasp.value)
            approach = multiply(invert(tool_from_hand), grasp.approach)
            adjusted_grasp = Grasp(grasp_type, obj_body, g, approach,
                                   robot.get_carry_conf(robot.arms[0], grasp_type, g))

            ## find ik
            for ik in stream_map["inverse-kinematics-hand"](a=None, o=obj_body, p=obj_Pose, g=adjusted_grasp):
                if len(ik) == 0:
                    continue

                print("ik", ik)

                ## now we want to sample a trajectory from current position to next
                q2 = ik[0][0]
                for move_cmd in stream_map["plan-free-motion-hand"](q1=current_q, q2=q2):
                    print(move_cmd)
                    if len(move_cmd) == 0:
                        continue

                    ## computes actions to step the world
                    pick_action = ('pick_hand', ('hand', obj_body, obj_Pose, adjusted_grasp, None, ik[0][1]))
                    move_action = ('move_cartesian', (current_q, ik[0][0], move_cmd[0][0]))
                    commands = []
                    for action in [move_action, pick_action]:
                        commands += get_primitive_actions(action, world)

                    ## update world state
                    current_g = adjusted_grasp
                    return commands

    commands = yield_grasp()
    print(commands)

    state.remove_gripper()
    saver.restore()
    input("confirm that state has been restored")

    apply_actions(state, commands, time_step=0, verbose=False)

    input("exit?")





if __name__ == '__main__':

    config_file = "/home/weiyu/Research/nsplan/original/kitchen-worlds/configs/test/feg_grasp_in_world.yaml"
    config = parse_yaml(config_file)
    main(config)



