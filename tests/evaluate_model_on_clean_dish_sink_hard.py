#!/usr/bin/env python

from __future__ import print_function

import sys
import math
import argparse
import itertools
import shutil
import pickle
import os
import time
import random
import copy
import json
from os.path import join, abspath, dirname, isdir, isfile, basename

import einops

from config import EXP_PATH, OUTPUT_PATH
from itertools import product
from PIL import Image


# from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
# from pddlstream.algorithms.meta import solve, create_parser

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

from nsplan_tools.generate_semantic_specification import get_semantic_specs, get_semantic_spec
from nsplan_tools.utils.file import print_data_types

# additional dependencies for using streams
from pybullet_tools.bullet_utils import set_camera_target_body, visualize_camera_image, get_readable_list
from pybullet_planning.pybullet_tools.utils import get_image_at_pose, get_image, unit_pose, get_camera_matrix
import matplotlib.pyplot as plt
from world_builder.entities import StaticCamera
from world_builder.utils import parse_yaml
from pybullet_tools.pr2_primitives import Pose, Conf
from pybullet_tools.utils import get_pose, multiply, quat_from_euler, dump_world, get_bodies, remove_body, get_bodies, remove_body, invert, Euler, Point, pairwise_collisions, tform_from_pose
from pybullet_tools.flying_gripper_utils import get_se3_joints, se3_from_pose, Grasp, se3_ik
from world_builder.actions import get_primitive_actions

# gym-related
import gymnasium as gym
from gymnasium import error, spaces, utils
from gymnasium.utils import seeding
import numpy as np


# import env
from collect_clean_dish_rollouts import plot_images, CleanDishEnvV1

# import from nsplan
import torch
import pytorch_lightning as pl
from omegaconf import OmegaConf
import trimesh

# sys.path.append("../../nsplan/src")  # adjust the path as necessary
import nsplan.data.vocab2 as vocab
from nsplan.data.dataset_sequence_multiview import MultiviewSequenceDataset
from nsplan.models.pl_models import DynamicsFeasibilityModel

from nsplan.utils.point_cloud import to_pc, extract_scene_xyzrgb, extract_multiview_img
from nsplan.utils.pointnet import pc_normalize
from nsplan.utils.data_conversions import move_to_gpu, repeat_batch, move_to_numpy
from nsplan.utils.files import get_checkpoint_path_from_dir
from nsplan.utils.visualization import adjust_scene_camera, scene_to_img, visualize_image_text_pairs, generate_html, visualize_xyzrgbs


# ===============================================================================================
# nsplan model related

def filter_null_actions(candidate_actions, feasibility_matrix, debug=False):
    query_actions_concept_idxs = []
    query_actions = []
    for action, concept_idx in zip(candidate_actions, feasibility_matrix):
        if debug:
            print(action, concept_idx)
        if "null" in action:
            continue
        query_actions_concept_idxs.append(concept_idx)
        query_actions.append(action)
    return np.array(query_actions_concept_idxs), query_actions


# def concept_idx_to_action_name(ordered_vocabs):
#     id_to_concept_maps = []
#     for vocab in ordered_vocabs:
#         id_to_concept_map = {vocab[concept]: concept for concept in vocab}
#         id_to_concept_maps.append(id_to_concept_map)
#     # how to use:
#     # action = []
#     # for i, c in enumerate(concept_idx):
#     #     action.append(id_to_concept_maps[i][c])
#     return id_to_concept_maps

def get_text_action_from_concept_action(concept_action, color_object_to_obj_name):
    # concept action is ('place', 'brown', 'bottle', 'sink_counter_right')
    # text action is (manip_name, obj_name, loc_name)
    (manipulation, color, object_class, location) = concept_action
    obj_name = color_object_to_obj_name[(color, object_class)]
    return (manipulation, obj_name, location)


def get_concept_action_from_text_action(text_action, obj_name_to_info):
    # concept action is ('place', 'brown', 'bottle', 'sink_counter_right')
    # text action is (manip_name, obj_name, loc_name)
    (manipulation, object_name, location) = text_action
    color = obj_name_to_info[object_name]["color"]
    object_class = obj_name_to_info[object_name]["class"]
    return (manipulation, color, object_class, location)


def get_color_object_to_obj_name(obj_name_to_info):
    color_object_to_obj_name = {}
    for obj_name in obj_name_to_info:
        color = obj_name_to_info[obj_name]["color"]
        object_class = obj_name_to_info[obj_name]["class"]
        color_object_to_obj_name[(color, object_class)] = obj_name
    return color_object_to_obj_name


# def check_reach_symbolic_goal(symbolic_goal, symbolic_state):
#     # symbolic_goal = env.symbolic_goal
#     at_goal = True
#     for obj_name in symbolic_goal:
#         if symbolic_goal[obj_name] != symbolic_state[obj_name]["location"]:
#             at_goal = False
#             break
#     return at_goal

def get_goal_actions(symbolic_goal):
    # symbolic_goal = env.symbolic_goal
    goal_actions = []
    for obj_name in symbolic_goal:
        goal_actions.append(("place", obj_name, symbolic_goal[obj_name]))
    return goal_actions


def load_model_and_cfg():

    # create a new argparser args object to store the model arguments
    args = argparse.Namespace()
    args.base_config_file = "/home/weiyu/Research/nsplan/nsplan/configs/base.yaml"

    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTFiLM1DDynamicsFeasibilityModel_subsample_trajectory.yaml"
    # args.checkpoint_id = "gq6ui8m9"

    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTFiLM1DDynamicsFeasibilityModel.yaml"
    # args.checkpoint_id = "026qc2hq"

    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTFiLM1DDynamicsFeasibilityModel_subsample_trajectory_irrelevant_actions.yaml"
    # args.checkpoint_id = "0yjlzwp2"

    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTTransformerDynamicsFeasibilityModel_subsample_trajectory_irrelevant_actions_XL_lr.yaml"
    # args.checkpoint_id = "6ixjer9q"

    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTFiLM1DDynamicsFeasibilityModel_subsample_trajectory_irrelevant_actions.yaml"
    # args.checkpoint_id = "w6lzkv52"

    # trained on 0919 data
    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTTransformerDynamicsFeasibilityModel_subsample_trajectory_irrelevant_actions_XL_lr.yaml"
    # args.checkpoint_id = "k6sxfz7r"

    # # trained on 0828 data witheld red bowl
    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTTransformerDynamicsFeasibilityModel_subsample_trajectory_irrelevant_actions_XL_lr.yaml"
    # args.checkpoint_id = "c7ycj2eh"

    # trained on 0828 data witheld red bowl
    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTFiLM1DDynamicsFeasibilityModel_subsample_trajectory_irrelevant_actions_large_lr.yaml"
    # args.checkpoint_id = "ztgus1dc"

    # trained on 0924 constrained placing parallel data
    args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTTransformerDynamicsFeasibilityModel_subsample_trajectory_irrelevant_actions_XL_lr_v2.yaml"
    args.checkpoint_id = "9mc72ohx"

    base_cfg = OmegaConf.load(args.base_config_file)
    cfg = OmegaConf.load(args.config_file)
    cfg = OmegaConf.merge(base_cfg, cfg)

    # pl.seed_everything(cfg.random_seed)
    device = (torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu"))

    checkpoint_dir = os.path.join(cfg.WANDB.save_dir, cfg.WANDB.project, args.checkpoint_id, "checkpoints")
    checkpoint_path = get_checkpoint_path_from_dir(checkpoint_dir)

    model = DynamicsFeasibilityModel.load_from_checkpoint(checkpoint_path)
    model.to(device)
    model.eval()

    return model, cfg


# ===============================================================================================
# env related

def run_evaluation(args):
    env_config = parse_yaml(args.config_file)

    # override
    env_config.seed = args.seed
    env_config.semantic_spec_seed = args.semantic_spec_seed

    # change data.out_dir for storing evaluation data
    env_config.data.out_dir = os.path.join(os.path.split(env_config.data.out_dir)[0], "evaluation")
    print(f"Saving evaluation data to {abspath(env_config.data.out_dir)}")

    process(env_config)


def process(config):
    """ exist a version in cognitive-architectures for generating mini-datasets (single process),
        run in kitchen-worlds for parallelization, but no reliable planning time data

        inside each data folder, to be generated:
        - before planning:
            [x] scene.lisdf
            [x] problem.pddl
            [x] planning_config.json
            [x] log.txt (generated before planning)
        - after planning:
            [x] plan.json
            [x] commands.pkl
            [x] log.json (generated by pddlstream)
    """

    seed = config.seed
    semantic_spec_seed = config.semantic_spec_seed

    new_config = copy.deepcopy(config)
    new_config.seed = seed
    set_random_seed(seed)
    set_numpy_seed(seed)

    exp_dir = abspath(join(config.data.out_dir, "semantic_spec_{}_seed_{}".format(semantic_spec_seed, seed))) #+ get_datetime(TO_LISDF=True)))
    print(exp_dir)
    os.makedirs(exp_dir, exist_ok=True)
    new_config.data.out_dir = exp_dir

    new_config.world.builder_kwargs["semantic_spec_file"] = os.path.join(config.semantic_specs_dir, f"{semantic_spec_seed}.json")

    """ STEP 1 -- GENERATE SCENES """
    world, goal = create_pybullet_world(new_config, SAVE_LISDF=False, SAVE_TESTCASE=True)

    env = CleanDishEnvV1(world, goal, config, render_mode="bot")
    # input("env initialized, next?")
    # play(env)
    run_high_level_policy(env, exp_dir)


def play(env):
    try:
        done = False
        obs = env.reset()
        num_moves = 0
        while not done:
            print("-"*10)
            print(f"move {num_moves}")
            env.print_admissible_actions()
            manip_name = input("manipulation name > ")
            obj_name = input("object name > ")
            loc_name = input("location name > ")
            action = env.convert_text_to_action(manip_name, obj_name, loc_name)
            obs, score, done, _, info = env.step(action)
            print(f"\nscore {score}, done {done}")
            num_moves += 1
    except KeyboardInterrupt:
        pass # Press the stop button in the toolbar to quit the game.

    print("\n" + "*" * 100)
    print("Played {} steps, scoring {} points.".format(num_moves, score))
    print("*" * 100 + "\n")


# ===============================================================================================
# rollouts

def sort_query_actions(query_actions, query_actions_concept_idxs, query_actions_scores):
    # sort query_actions and query_action_scores by query_action_scores
    ranking_idx = np.argsort(query_actions_scores)[::-1]
    sorted_query_actions = [query_actions[idx] for idx in ranking_idx]
    sorted_query_actions_scores = query_actions_scores[ranking_idx]
    sorted_query_actions_concept_idxs = query_actions_concept_idxs[ranking_idx]
    return sorted_query_actions, sorted_query_actions_concept_idxs, sorted_query_actions_scores

def print_scored_query_actions(query_actions, query_actions_scores, admissible_concept_actions):
    if admissible_concept_actions is not None:
        print(f"Only consider admissible_concept_actions")
    for query_action, query_action_score in zip(query_actions, query_actions_scores):
        if admissible_concept_actions is None or query_action in admissible_concept_actions:
            print(f"{query_action}: {query_action_score}")

def plan_single_step(model,
                     scene_xyzrgb, query_actions_concept_idxs, grasped,
                     query_actions,
                     num_scene_pts, device,
                     admissible_concept_actions=None, debug=False):
    # --------------------------------
    # build current data for model
    # pcs: B, T (sequence length), P (number of pts), D (number of channels)
    # next_action_concept_idxs: B, T, L (number of concepts)
    # query_actions_concept_idxs: B, T, K (number of query actions), L
    # gripper_states: B, T
    B = 1
    # debug: make one step inference
    T = 1
    P = num_scene_pts
    D = 6
    L = 4
    # K = ?

    # prepare single datum
    if grasped is None:
        gripper_states = np.array(vocab.gripper_state_to_id["empty"], dtype=np.int32)  # 1
    else:
        gripper_states = np.array(vocab.gripper_state_to_id["grasped"], dtype=np.int32)  # 1
    next_action_concept_idxs = np.zeros((B, T, L), dtype=np.int32)
    pcs = scene_xyzrgb.astype(np.float32)  # P, D

    # repeat and convert to tensor
    pcs = einops.repeat(pcs, "P D -> B T P D", B=B, T=T, P=P, D=D)
    query_actions_concept_idxs = einops.repeat(query_actions_concept_idxs, "K L -> B T K L", B=B, T=T, L=L)
    gripper_states = einops.repeat(gripper_states, " -> B T", B=B, T=T)

    batch = {"pcs": pcs,
             "query_actions_concept_idxs": query_actions_concept_idxs,
             "next_action_concept_idxs": next_action_concept_idxs,
             "gripper_states": gripper_states}

    move_to_gpu(batch, device=device)

    # the policy model should predict the next action given the current observation
    with torch.no_grad():
        query_actions_logits = model.model(batch["pcs"],
                                           batch["gripper_states"],
                                           batch["next_action_concept_idxs"],
                                           batch["query_actions_concept_idxs"])  # B, T, K
    query_actions_scores = model.model.convert_logit_to_binary(query_actions_logits).cpu().numpy()  # B, T, K
    query_actions_scores = query_actions_scores[0][0]
    if debug:
        print(f"query_actions_scores shape: {query_actions_scores.shape}")

    # sort query_actions and query_action_scores by query_action_scores
    ranking_idx = np.argsort(query_actions_scores)[::-1]
    sorted_query_actions = [query_actions[idx] for idx in ranking_idx]
    sorted_query_actions_scores = query_actions_scores[ranking_idx]

    if debug:
        print("\n" + "-" * 100 + "\nranked query actions")
        if admissible_concept_actions is not None:
            print(f"Only consider admissible_concept_actions")
        for query_action, query_action_score in zip(sorted_query_actions, sorted_query_actions_scores):
            if admissible_concept_actions is None or query_action in admissible_concept_actions:
                print(f"{query_action}: {query_action_score}")
        trimesh.PointCloud(scene_xyzrgb[:, :3], scene_xyzrgb[:, 3:]).show()

    return sorted_query_actions, sorted_query_actions_scores


def plan_multi_step_debug(model,
                    # model input
                     scene_xyzrgb, query_actions_concept_idxs, grasped,
                     # for logging
                     query_actions,
                    # hyperparameters
                     num_scene_pts, device,
                     admissible_concept_actions=None, debug=False,
                    action_score_threshold=0.3):

    # --------------------------------
    # first step
    print("\n\n" + "~" * 100)
    print(f"planning timestep 0")

    # build current data for model
    # pcs: B, T (sequence length), P (number of pts), D (number of channels)
    # next_action_concept_idxs: B, T, L (number of concepts)
    # query_actions_concept_idxs: B, T, K (number of query actions), L
    # gripper_states: B, T
    B = 1
    # debug: make one step inference
    T = 1
    P = num_scene_pts
    D = 6
    L = 4
    # K = ?

    # prepare single datum
    if grasped is None:
        gripper_states = np.array(vocab.gripper_state_to_id["empty"], dtype=np.int32)  # 1
    else:
        gripper_states = np.array(vocab.gripper_state_to_id["grasped"], dtype=np.int32)  # 1
    batch_next_action_concept_idxs = np.zeros((B, T, L), dtype=np.int32)
    pcs = scene_xyzrgb.astype(np.float32)  # P, D
    # query_actions_concept_idxs: K, L

    # repeat and convert to tensor
    batch_pcs = einops.repeat(pcs, "P D -> B T P D", B=B, T=T, P=P, D=D)
    batch_query_actions_concept_idxs = einops.repeat(query_actions_concept_idxs, "K L -> B T K L", B=B, T=T, L=L)
    batch_gripper_states = einops.repeat(gripper_states, " -> B T", B=B, T=T)

    batch = {"pcs": batch_pcs,
             "query_actions_concept_idxs": batch_query_actions_concept_idxs,
             "next_action_concept_idxs": batch_next_action_concept_idxs,
             "gripper_states": batch_gripper_states}

    move_to_gpu(batch, device=device)

    # the policy model should predict the next action given the current observation
    with torch.no_grad():
        query_actions_logits = model.model(batch["pcs"],
                                           batch["gripper_states"],
                                           batch["next_action_concept_idxs"],
                                           batch["query_actions_concept_idxs"])  # B, T, K
    query_actions_scores = model.model.convert_logit_to_binary(query_actions_logits).cpu().numpy()  # B, T, K
    query_actions_scores = query_actions_scores[0][0]
    print(f"query_actions_scores shape: {query_actions_scores.shape}")

    sorted_query_actions, sorted_query_actions_concept_idxs, sorted_query_actions_scores = sort_query_actions(query_actions, query_actions_concept_idxs, query_actions_scores)

    print("\n" + "-" * 100 + "\nranked query actions")
    print_scored_query_actions(sorted_query_actions, sorted_query_actions_scores, admissible_concept_actions)

    # --------------------------------
    # second step
    print("\n\n" + "~" * 100)
    print(f"planning timestep 1")

    # find next actions we want to consider
    next_action_concept_idxs = []
    next_actions = []
    next_action_scores = []
    for query_action, query_action_score, query_action_concept_idxs in zip(sorted_query_actions,
                                                                            sorted_query_actions_scores,
                                                                            sorted_query_actions_concept_idxs):
        if admissible_concept_actions is None or query_action in admissible_concept_actions:
            if query_action_score > action_score_threshold:
                next_actions.append(query_action)
                next_action_concept_idxs.append(query_action_concept_idxs)
                next_action_scores.append(query_action_score)

    print(f"step 2: {len(next_action_concept_idxs)} next actions to consider with score threshold {action_score_threshold}: {next_actions}")

    # build current data for model
    B = len(next_action_concept_idxs)
    T = 2

    batch_next_action_concept_idxs = np.zeros((B, T, L), dtype=np.int32)
    # the second action is zero
    batch_next_action_concept_idxs[:, 0, :] = next_action_concept_idxs

    # important: since the model will only use the current observation
    # i.e., pcs_0 = pcs[:, 0, :, :]  # B, P, D
    # i.e., gripper_states_0 = gripper_states[:, 0]  # B
    # we will set T = 1 for batch_pcs and batch_gripper_states to save memory
    batch_pcs = einops.repeat(pcs, "P D -> B T P D", B=B, T=1, P=P, D=D)
    batch_gripper_states = einops.repeat(gripper_states, " -> B T", B=B, T=1)
    batch_query_actions_concept_idxs = einops.repeat(query_actions_concept_idxs, "K L -> B T K L", B=B, T=T, L=L)

    batch = {"pcs": batch_pcs,
             "query_actions_concept_idxs": batch_query_actions_concept_idxs,
             "next_action_concept_idxs": batch_next_action_concept_idxs,
             "gripper_states": batch_gripper_states}

    move_to_gpu(batch, device=device)

    # the policy model should predict the next action given the current observation
    with torch.no_grad():
        query_actions_logits = model.model(batch["pcs"],
                                           batch["gripper_states"],
                                           batch["next_action_concept_idxs"],
                                           batch["query_actions_concept_idxs"])  # B, T, K
    query_actions_scores = model.model.convert_logit_to_binary(query_actions_logits).cpu().numpy()  # B, T, K
    print(f"query_actions_scores shape: {query_actions_scores.shape}")

    for nai in range(len(next_actions)):
        print("\n" + "-" * 100)
        # # This is only sanity check, the prediction of feasibility for first actions should be the same as first step
        # print(f"\nnext_action: {next_actions[nai]} step: {0}")
        # this_query_actions_scores = query_actions_scores[nai][0]
        # this_sorted_query_actions, this_sorted_query_actions_concept_idxs, this_sorted_query_actions_scores = sort_query_actions(query_actions, query_actions_concept_idxs, this_query_actions_scores)
        # print_scored_query_actions(this_sorted_query_actions, this_sorted_query_actions_scores, admissible_concept_actions)

        print(f"\nnext_action: {next_actions[nai]} step: {1}")
        this_query_actions_scores = query_actions_scores[nai][1]
        this_sorted_query_actions, this_sorted_query_actions_concept_idxs, this_sorted_query_actions_scores = sort_query_actions(query_actions, query_actions_concept_idxs, this_query_actions_scores)
        print_scored_query_actions(this_sorted_query_actions, this_sorted_query_actions_scores, admissible_concept_actions)

    return


def build_model_input(observation_mode, grasped, observation, query_actions_concept_idxs,
                      B, T, D, L, device, num_obj_pts=None, num_scene_pts=None,
                      next_action_concept_idxs=None, observation_mask=None):
    """
    :param grasped:
    :param observation: P, D or N, P, D
    :param query_actions_concept_idxs: K, L
    :param B:
    :param T:
    :param P:
    :param D:
    :param L:
    :param device:
    :param next_action_concept_idxs: B, T-1, L
    :param observation_mask: N
    :return:
    """

    # build current data for model
    # pcs: B, T (sequence length), P (number of pts), D (number of channels) or B, T, N, P, D
    # next_action_concept_idxs: B, T, L (number of concepts)
    # query_actions_concept_idxs: B, T, K (number of query actions), L
    # gripper_states: B, T

    # prepare single datum
    if grasped is None:
        gripper_states = np.array(vocab.gripper_state_to_id["empty"], dtype=np.int32)  # 1
    else:
        gripper_states = np.array(vocab.gripper_state_to_id["grasped"], dtype=np.int32)  # 1
    pcs = observation.astype(np.float32)  # P, D or N, P, D
    # query_actions_concept_idxs: K, L

    batch_next_action_concept_idxs = np.zeros((B, T, L), dtype=np.int32)
    if next_action_concept_idxs is not None:
        # e.g., if T = 2, we set the first next action so that we can get action feasibility for t=2
        batch_next_action_concept_idxs[:, :T-1, :] = next_action_concept_idxs

    # repeat and convert to tensor
    # important: since the model will only use the current observation
    # i.e., pcs_0 = pcs[:, 0, :, :]  # B, P, D
    # i.e., gripper_states_0 = gripper_states[:, 0]  # B
    # we will set T = 1 for batch_pcs and batch_gripper_states to save memory
    if observation_mode == "pc":
        batch_pcs = einops.repeat(pcs, "P D -> B T P D", B=B, T=1, P=num_scene_pts, D=D)
    elif observation_mode == "pcs":
        batch_pcs = einops.repeat(pcs, "N P D -> B T N P D", B=B, T=1, N=observation.shape[0], P=num_obj_pts, D=D)
        batch_observation_masks = einops.repeat(observation_mask, "N -> B T N", B=B, T=T)
    else:
        raise KeyError
    batch_gripper_states = einops.repeat(gripper_states, " -> B T", B=B, T=1)

    batch_query_actions_concept_idxs = einops.repeat(query_actions_concept_idxs, "K L -> B T K L", B=B, T=T, L=L)

    batch = {"pcs": batch_pcs,
             "query_actions_concept_idxs": batch_query_actions_concept_idxs,
             "next_action_concept_idxs": batch_next_action_concept_idxs,
             "gripper_states": batch_gripper_states}

    if observation_mode == "pcs":
        batch["pc_masks"] = batch_observation_masks

    move_to_gpu(batch, device=device)

    # # print shape for element in batch
    # for k, v in batch.items():
    #     print(f"{k}: {v.shape}")
    # input("next")

    return batch


def mask_list(list, mask):
    assert len(list) == len(mask)
    return [list[i] for i in range(len(list)) if mask[i]]


def plan_multi_step(model,
                    # model input
                    observation_mode, observation, query_actions_concept_idxs, grasped,
                     # for logging
                     query_actions, goal_action,
                    # hyperparameters
                     num_obj_pts, num_scene_pts, device,
                    observation_mask=None,
                     admissible_concept_actions=None, debug=False,
                    action_score_threshold=0.3, planning_horizon=1, max_beam_size=20, rank_method="min"):

    assert rank_method in ["product", "min", "max"]

    # query_actions_concept_idxs: K, L

    # we want to find a sequence of actions that leads to the goal action
    goal_action_sequence = []

    # if we can't find a sequence of actions that lead to the goal, we just execute an action that is feasible
    scored_first_feasible_actions = []

    # to keep track of the beam search
    sampled_action_idx_sequences = None
    sampled_action_score_sequences = None

    all_action_idx_sequences = []
    all_action_score_sequences = []

    all_action_idxs = np.arange(len(query_actions))  # K
    admissible_action_mask = []
    for action_idx, action in enumerate(query_actions):
        if admissible_concept_actions is None or action in admissible_concept_actions:
            admissible_action_mask.append(True)
        else:
            admissible_action_mask.append(False)
    admissible_action_mask = np.array(admissible_action_mask, dtype=bool)  # K
    goal_action_idx = query_actions.index(goal_action)

    texts = []

    for t in range(planning_horizon):

        print("\n\n" + "~" * 100)
        print(f"planning timestep {t}")
        texts.append("\n\n" + "~" * 50)
        texts.append(f"planning timestep {t}")

        # sampled_action_concept_idx_sequences: B, t, L
        if t == 0:
            batch = build_model_input(observation_mode, grasped, observation, query_actions_concept_idxs,
                                      B=1, T=t+1, D=6, L=4, device=device, num_obj_pts=num_obj_pts, num_scene_pts=num_scene_pts,
                                      next_action_concept_idxs=None, observation_mask=observation_mask)
        else:
            current_beam_size = len(sampled_action_idx_sequences)

            # sampled_action_idx_sequences: B, T
            # query_actions_concept_idxs: K, L
            # next_action_concept_idxs: B, T, L
            next_action_concept_idxs = query_actions_concept_idxs[sampled_action_idx_sequences]  # B, T, L

            batch = build_model_input(observation_mode, grasped, observation, query_actions_concept_idxs,
                                      B=current_beam_size, T=t + 1, D=6, L=4, device=device, num_obj_pts=num_obj_pts, num_scene_pts=num_scene_pts,
                                      next_action_concept_idxs=next_action_concept_idxs, observation_mask=observation_mask)

        with torch.no_grad():
            if model.model_cfg.name == "PCTTransformer":
                # debug: we are using training mode because there is a bug in pytorch for transformer.
                #        we can upgrade pytorch version.
                #        https://discuss.pytorch.org/t/model-evaluation-fails-when-using-gpus-but-works-well-on-cpu/158796
                model.train()
                query_actions_logits = model.model(batch["pcs"],
                                                   batch["pc_masks"],
                                                   batch["gripper_states"],
                                                   batch["next_action_concept_idxs"],
                                                   batch["query_actions_concept_idxs"])  # B, T, K
                model.eval()
            else:
                query_actions_logits = model.model(batch["pcs"],
                                                   batch["gripper_states"],
                                                   batch["next_action_concept_idxs"],
                                                   batch["query_actions_concept_idxs"])  # B, T, K
        query_actions_scores = model.model.convert_logit_to_binary(query_actions_logits).cpu().numpy()  # B, T, K
        # since we are decoding autoreressively, we only care about the last timestep
        query_actions_scores = query_actions_scores[:, t]  # B, K
        print(f"query_actions_scores shape: {query_actions_scores.shape}")

        # # debug
        # # print("\n" + "-" * 100 + "\nranked query actions")
        # sorted_query_actions, sorted_query_actions_concept_idxs, sorted_query_actions_scores = sort_query_actions(query_actions, query_actions_concept_idxs, query_actions_scores)
        # print_scored_query_actions(sorted_query_actions, sorted_query_actions_scores, admissible_concept_actions)

        # # filter actions based on admissible actions and single-step feasibility score
        # candidate_actions = []  # C
        # candidate_action_concept_idxs = []   # C, L
        # candidate_action_scores = []  # C
        # for query_action, query_action_score, query_action_concept_idxs in zip(query_actions,
        #                                                                        query_actions_scores,
        #                                                                        query_actions_concept_idxs):
        #     if admissible_concept_actions is None or query_action in admissible_concept_actions:
        #         if query_action_score > action_score_threshold:
        #             candidate_actions.append(query_action)
        #             candidate_action_concept_idxs.append(query_action_concept_idxs)
        #             candidate_action_scores.append(query_action_score)

        if t == 0:

            index_matrix = einops.rearrange(all_action_idxs, 'K -> 1 K')  # B, K

            # TODO: perform more complicated filtering
            score_threshold_mask = query_actions_scores > action_score_threshold
            # Bool and admissible_action_mask and score_threshold_mask
            keep_mask = np.logical_and(score_threshold_mask, einops.rearrange(admissible_action_mask, 'K -> 1 K'))
            keep_mask = keep_mask.flatten()  # 1 * K

            # find topk index from query_actions_scores with dim [B, K]
            topk_idxs = np.argsort(query_actions_scores.flatten()[keep_mask])[::-1][:max_beam_size]  # B * beam_size

            sampled_action_idxs = index_matrix.flatten()[keep_mask][topk_idxs]  # beam_size
            sampled_action_scores = query_actions_scores.flatten()[keep_mask][topk_idxs]  # beam_size

            sampled_actions = [mask_list(query_actions, keep_mask)[ki] for ki in topk_idxs]

            for sa, sas in zip(sampled_actions, sampled_action_scores):
                print(f"{sa}: {sas}")
                texts.append(f"{sa}: {sas}")

            sampled_action_idx_sequences = einops.rearrange(sampled_action_idxs, 'B -> B 1')  # B, T
            sampled_action_score_sequences = einops.rearrange(sampled_action_scores, 'B -> B 1')  # B, T

            scored_first_feasible_actions = [(query_actions[ai], score) for ai, score in zip(sampled_action_idxs, sampled_action_scores)]

        else:

            sequence_index_matrix = []  # B * K,
            for bi in range(current_beam_size):
                for ki in range(len(query_actions)):
                    sequence_index_matrix.append([bi, ki])

            if rank_method == "min":
                sampled_sequence_scores = np.min(sampled_action_score_sequences, axis=1)  # B
                sampled_sequence_scores = einops.repeat(sampled_sequence_scores, 'B -> B K', K=len(query_actions))  # B, K
                # give two matrices of size B, K, return a matrix of size B, K, where each element is the min of the two
                accumulated_scores = np.minimum(sampled_sequence_scores, query_actions_scores)  # B, K
            elif rank_method == "max":
                sampled_sequence_scores = np.max(sampled_action_score_sequences, axis=1)  # B
                sampled_sequence_scores = einops.repeat(sampled_sequence_scores, 'B -> B K', K=len(query_actions))  # B, K
                # give two matrices of size B, K, return a matrix of size B, K, where each element is the min of the two
                accumulated_scores = np.maximum(sampled_sequence_scores, query_actions_scores)  # B, K
            elif rank_method == "product":
                sampled_sequence_scores = np.product(sampled_action_score_sequences, axis=1)  # B
                sampled_sequence_scores = einops.repeat(sampled_sequence_scores, 'B -> B K', K=len(query_actions))  # B, K
                accumulated_scores = sampled_sequence_scores * query_actions_scores  # B, K

            # TODO: perform more complicated filtering
            score_threshold_mask = query_actions_scores > action_score_threshold
            # Bool and admissible_action_mask and score_threshold_mask
            keep_mask = np.logical_and(score_threshold_mask, einops.repeat(admissible_action_mask, 'K -> B K', B=current_beam_size))
            keep_mask = keep_mask.flatten()  # B * K
            print("keep_mask.shape", keep_mask.shape)
            print("np.sum(keep_mask)", np.sum(keep_mask))

            topk_idxs = np.argsort(accumulated_scores.flatten()[keep_mask])[::-1][:max_beam_size]  # B * beam_size
            print("topk_idxs", topk_idxs)

            new_beam_size = len(topk_idxs)

            sampled_accumulated_scores = accumulated_scores.flatten()[keep_mask][topk_idxs]  # beam_size
            sampled_query_actions_scores = query_actions_scores.flatten()[keep_mask][topk_idxs]  # beam_size
            keep_sequence_index_matrix = mask_list(sequence_index_matrix, keep_mask)
            print("keep_sequence_index_matrix", keep_sequence_index_matrix)
            sampled_action_idxs = [keep_sequence_index_matrix[ti][1] for ti in topk_idxs]
            sampled_sequence_idxs = [keep_sequence_index_matrix[ti][0] for ti in topk_idxs]

            # sampled_action_idx_sequences: B, T
            # sampled_action_score_sequences: B, T

            for bi in range(new_beam_size):
                action_sequence = [query_actions[ai] for ai in sampled_action_idx_sequences[sampled_sequence_idxs[bi]]]
                print(f"action sequence: {action_sequence}")
                action = query_actions[sampled_action_idxs[bi]]
                print(f"action: {action}")
                score = sampled_accumulated_scores[bi]
                print(f"score: {score}")
                texts.append(f"action sequence: {action_sequence}")
                texts.append(f"action: {action}")
                texts.append(f"score: {score}")

            # debug: sequence assignment is wrong
            new_sampled_action_idx_sequences = np.zeros((new_beam_size, t + 1), dtype=np.int32)
            new_sampled_action_score_sequences = np.zeros((new_beam_size, t + 1), dtype=np.float32)
            new_sampled_action_idx_sequences[:, :-1] = np.array([sampled_action_idx_sequences[ssi] for ssi in sampled_sequence_idxs], dtype=np.int32)
            new_sampled_action_score_sequences[:, :-1] = np.array([sampled_action_score_sequences[ssi] for ssi in sampled_sequence_idxs], dtype=np.float32)
            new_sampled_action_idx_sequences[:, -1] = np.array(sampled_action_idxs, dtype=np.int32)
            new_sampled_action_score_sequences[:, -1] = np.array(sampled_query_actions_scores, dtype=np.float32)

            sampled_action_idx_sequences = new_sampled_action_idx_sequences
            sampled_action_score_sequences = new_sampled_action_score_sequences

        # # check for goal
        # # action sequences are already ranked
        # for bi in range(len(sampled_action_idx_sequences)):
        #     action_idx_sequence = sampled_action_idx_sequences[bi]
        #     if goal_action_idx in action_idx_sequence:
        #         print("\n" + "*"*20)
        #         print(f"plan for goal {goal_action} found!")
        #         goal_action_sequence = [query_actions[ai] for ai in action_idx_sequence]
        #         print(f"goal action sequence: {goal_action_sequence}")
        #         print(f"goal action score sequence: {sampled_action_score_sequences[bi]}")
        #         print("*" * 20 + "\n")
        #         texts.append(f"plan for goal {goal_action} found!")
        #         texts.append(f"goal action sequence: {goal_action_sequence}")
        #         texts.append(f"goal action score sequence: {sampled_action_score_sequences[bi]}")
        #         break
        #
        # # if goal action sequence is found, break from planning
        # if goal_action_sequence:
        #     break

            all_action_idx_sequences.extend(sampled_action_idx_sequences)
            all_action_score_sequences.extend(sampled_action_score_sequences)

    # sort all action sequences by score
    if rank_method == "min":
        all_action_accumulated_scores = np.array([np.min(a) for a in all_action_score_sequences])
    elif rank_method == "max":
        all_action_accumulated_scores = np.array([np.max(a) for a in all_action_score_sequences])
    elif rank_method == "product":
        all_action_accumulated_scores = np.array([np.product(a) for a in all_action_score_sequences])
    sorted_idxs = np.argsort(all_action_accumulated_scores)[::-1]
    all_action_idx_sequences = [all_action_idx_sequences[si] for si in sorted_idxs]
    all_action_score_sequences = [all_action_score_sequences[si] for si in sorted_idxs]
    all_action_accumulated_scores = [all_action_accumulated_scores[si] for si in sorted_idxs]

    goal_action_sequence = None
    print("\n\n" + "~" * 100)
    for bi in range(len(all_action_idx_sequences)):
        action_idx_sequence = all_action_idx_sequences[bi]
        if goal_action_idx in action_idx_sequence:
            action_sequence = [query_actions[ai] for ai in action_idx_sequence]
            if goal_action_sequence is None:
                goal_action_sequence = action_sequence
            # print(f"{all_action_accumulated_scores[bi]}: {action_sequence}")
            print(f"{all_action_score_sequences[bi]} ({all_action_accumulated_scores[bi]}): {action_sequence}")
    input("here")

    if goal_action_sequence:
        print(f"Planning complete. Goal action sequence found: {len(goal_action_sequence) != 0}")
        texts.append(f"Planning complete. Goal action sequence found: {len(goal_action_sequence) != 0}")
    else:
        print(f"Scored scored first feasible actions: {scored_first_feasible_actions}")
        first_feasible_actions = [a[0] for a in scored_first_feasible_actions if a[1] > 0.5]
        if len(first_feasible_actions) == 0:
            first_feasible_actions = [a[0] for a in scored_first_feasible_actions]
        random_action = first_feasible_actions[np.random.choice(len(first_feasible_actions))]
        print(f"Planning reached max horizon. Goal action sequence not found")
        print(f"Taking random action: {random_action}")
        texts.append(f"Scored scored first feasible actions: {scored_first_feasible_actions}")
        texts.append(f"Planning reached max horizon. Goal action sequence not found")
        texts.append(f"Taking random action: {random_action}")
        # if debug:
        #     trimesh.PointCloud(scene_xyzrgb[:, :3], scene_xyzrgb[:, 3:]).show()
        goal_action_sequence = [random_action]

    if debug:
        if observation_mode == "pc":
            trimesh.PointCloud(observation[:, :3], observation[:, 3:]).show()
        elif observation_mode == "pcs":
            visualize_xyzrgbs(observation, show_instance_seg=False).show()

    if observation_mode == "pc":
        scene = trimesh.PointCloud(observation[:, :3], observation[:, 3:]).scene()
    elif observation_mode == "pcs":
        scene = visualize_xyzrgbs(observation, show_instance_seg=False)
    adjust_scene_camera(scene, camera_location=[1.5, 0.75, 0.75], look_at=[0, 0, 0])
    img = scene_to_img(scene, resolution=[1080, 1080])
    log_img_text_pair = (img, "\n".join(texts))

    return goal_action_sequence, log_img_text_pair


def run_high_level_policy(env: CleanDishEnvV1, exp_dir, max_depth=5, debug=True, only_consider_feasible=True):

    # TODO: load from model or data config

    model, cfg = load_model_and_cfg()
    device = model.device

    # ----------------------------------
    # hyperparams:
    num_obj_pts = cfg.DATASET.num_obj_pts
    num_scene_pts = cfg.DATASET.num_scene_pts if "num_scene_pts" in cfg.DATASET else None
    observation_mode = cfg.DATASET.observation_mode
    max_num_objs = cfg.DATASET.max_num_objs if "max_num_objs" in cfg.DATASET else None

    # ----------------------------------
    # meta data
    obj_dict = env.world.obj_dict
    for obj in obj_dict:
        obj_dict[obj]["name"] = str(obj_dict[obj]["name"])
    # for mapping from text_action to concept actions
    obj_name_to_info = MultiviewSequenceDataset.get_obj_name_to_info(obj_dict)
    # for mapping from concept actions to text_action
    color_object_to_obj_name = get_color_object_to_obj_name(obj_name_to_info)

    # {1: floor1, 2: sinkbase, 3: sink#1, 4: faucet, 7: sink_counter_front, 8: sink_counter_back, 9: dishwasherbox, 10: cabinetlower, 11: wall, 6: sink_counter_right, 5: sink_counter_left, 12: counter_back#1, 13: cabinettop, 14: cabinettop_filler, 15: shelf_lower, (3, None, 2): sink#1::sink_bottom, (13, 2): cabinettop::dagger_door_left_joint, (13, 6): cabinettop::dagger_door_right_joint, (13, None, 0): cabinettop::cabinettop_storage, 16: mug#1, 17: bowl#1}
    pybullet_idx_to_name = {b: env.world.BODY_TO_OBJECT[b].name for b in env.world.BODY_TO_OBJECT}
    obj_name_to_pybullet_idx = {pybullet_idx_to_name[pbid]: pbid for pbid in pybullet_idx_to_name}
    obj_name_to_pybullet_idx["robot"] = 0

    cached_candidate_actions, cached_feasibility_matrix, orderer_vocabs = MultiviewSequenceDataset.cache_candidate_actions_and_feasibility_matrix(vocab)
    # important: augment state representation with gripper state because it's hard to see from img
    orderer_vocabs += [vocab.gripper_state_to_id]

    # query_actions_concept_idxs = np.copy(cached_feasibility_matrix)  # K, L
    # filter "null" actions
    query_actions_concept_idxs, query_actions = filter_null_actions(cached_candidate_actions, cached_feasibility_matrix)
    # important: ignore gripper state
    query_actions_concept_idxs = query_actions_concept_idxs[:, :4].astype(np.int32)
    print(f"Action space: {len(query_actions_concept_idxs)}")

    if only_consider_feasible:
        admissible_text_actions = sorted(env.get_admissible_text_actions())
        admissible_concept_actions = [get_concept_action_from_text_action(text_action, obj_name_to_info) for text_action in admissible_text_actions]
    else:
        admissible_concept_actions = None

    goal_actions = get_goal_actions(env.symbolic_goal)
    assert len(goal_actions) == 1, "We currently assume one goal action."
    goal_action = goal_actions[0]
    goal_concept_action = get_concept_action_from_text_action(goal_action, obj_name_to_info)
    print(f"Goal action: {goal_action} | Goal concept action: {goal_concept_action}")

    # ----------------------------------
    # start acting
    cur_obs, info = env.reset()
    cur_symbolic_state = env.symbolic_state
    cur_done = False
    cur_score = 0

    log_img_text_pairs = []
    for t in range(max_depth):

        if t == 0:
            before_texts = [f"Goal action: {goal_action} | Goal concept action: {goal_concept_action}"]
        else:
            before_texts = []
        after_texts = []

        # --------------------------------
        # build current observation
        if observation_mode == "pc":
            furniture_names = ["sink#1", "sink_counter_left", "sink_counter_right", "cabinettop", "shelf_lower"]
            obj_names = ["scene"] + ["robot"] + furniture_names + sorted(obj_name_to_info.keys())
            scene_xyzrgb = extract_scene_xyzrgb(cur_obs, obj_names, obj_name_to_pybullet_idx,
                                                num_pts_object=num_obj_pts,
                                                num_pts_scene=num_scene_pts, visualize=False)
            scene_xyzrgb = scene_xyzrgb[:, :6]  # ignore alpha
            scene_xyzrgb[:, :3] = pc_normalize(scene_xyzrgb[:, :3])
            observation = scene_xyzrgb
            observation_mask = None
        elif observation_mode == "pcs":
            furniture_names = ["sink#1", "sink_counter_left", "sink_counter_right", "cabinettop",
                               "shelf_lower"]
            obj_names = ["scene"] + ["robot"] + furniture_names + sorted(obj_name_to_info.keys())
            obj_name_to_xyzrgb = extract_scene_xyzrgb(cur_obs, obj_names, obj_name_to_pybullet_idx,
                                                      num_pts_object=num_obj_pts, visualize=False,
                                                      return_obj_name_to_xyzrgb=True)
            obj_xyzrgbs = []
            # important: the key is ordered in a specific order
            for obj in obj_names:
                if obj not in obj_name_to_xyzrgb:
                    continue
                obj_xyzrgbs.append(obj_name_to_xyzrgb[obj][:, :6])
            scene_xyzrgb = np.concatenate(obj_xyzrgbs, axis=0)
            xyz_centroid = np.mean(scene_xyzrgb[:, :3], axis=0)
            xyz_scale = 1.0 / np.max(np.sqrt(np.sum((scene_xyzrgb[:, :3] - xyz_centroid) ** 2, axis=1)))
            xyz_scale_translation = np.array([xyz_scale, xyz_centroid[0], xyz_centroid[1], xyz_centroid[2]])
            for obj_xyzrgb in obj_xyzrgbs:
                obj_xyzrgb[:, :3] = (obj_xyzrgb[:, :3] - xyz_centroid) * xyz_scale
            assert len(
                obj_name_to_xyzrgb) <= max_num_objs, "Too many objects in the scene. Max num obj is set to {}".format(max_num_objs)
            for _ in range(max_num_objs - len(obj_name_to_xyzrgb)):
                obj_xyzrgbs.append(np.zeros([num_obj_pts, 6]))
            observation = np.stack(obj_xyzrgbs, axis=0)  # O, P, 6
            observation_mask = np.concatenate([np.zeros([len(obj_name_to_xyzrgb)], dtype=np.int32),
                                               np.ones([max_num_objs - len(obj_name_to_xyzrgb)], dtype=np.int32)], axis=0)
            # visualize_xyzrgbs(observation, show_instance_seg=True).show()
            # visualize_xyzrgbs(observation, show_instance_seg=False).show()

            # for obs, obs_mask in zip(observation, observation_mask):
            #     print(obs)
            #     print(obs_mask)
            #     # trimesh.PointCloud(obs[:, :3]).show()

        else:
            raise KeyError

        grasped = cur_symbolic_state["grasped"]

        if debug:
            print("\n\n" + "=" * 100)
            print(f"timestep {t}")
            print(f"symbolic state: {cur_symbolic_state}")
            # plot_images({view_name: Image.fromarray(cur_obs[view_name][0], 'RGBA') for view_name in cur_obs})
            before_texts.append("\n\n" + "=" * 100)
            before_texts.append(f"timestep {t}")
            before_texts.append(f"symbolic state: {cur_symbolic_state}")

        # sorted_query_actions, sorted_query_actions_scores = plan_single_step(model,
        #                  scene_xyzrgb, query_actions_concept_idxs, grasped,
        #                  query_actions,
        #                  num_scene_pts, device,
        #                  admissible_concept_actions=admissible_concept_actions, debug=True)

        goal_concept_action_sequence, log_img_text_pair = plan_multi_step(model,
                                                                          observation_mode,
                        observation, query_actions_concept_idxs, grasped,
                        query_actions, goal_concept_action,
                        num_obj_pts, num_scene_pts, device,
                        observation_mask=observation_mask,
                        admissible_concept_actions=admissible_concept_actions, debug=True,
                        action_score_threshold=0.0, planning_horizon=4, max_beam_size=200)

        # --------------------------------
        # step the environment
        text_action = get_text_action_from_concept_action(goal_concept_action_sequence[0], color_object_to_obj_name)
        if debug:
            print(f"Take action: {text_action} | concept action: {goal_concept_action_sequence[0]}")
            after_texts.append(f"Take action: {text_action} | concept action: {goal_concept_action_sequence[0]}")
            # input("Press Enter to continue...")

        action = env.convert_text_to_action(text_action[0], text_action[1], text_action[2])
        cur_obs, cur_score, cur_done, _, _ = env.step(action)
        cur_symbolic_state = env.symbolic_state

        if cur_done:
            print(f"\nscore {cur_score}, done {cur_done}")
            after_texts.append(f"\nscore {cur_score}, done {cur_done}")
            log_img_text_pairs += [(None, "\n".join(before_texts)), log_img_text_pair, (None, "\n".join(after_texts))]
            break

        # debug: need to fix goal check in env.step(action). This is a temporary fix.
        # print(text_action)
        # print(goal_action)
        # input("here")
        if text_action == goal_action and cur_score != -1:
            cur_score = 100
            cur_done = True
            print(f"\nscore {cur_score}, done {cur_done}")
            after_texts.append(f"\nscore {cur_score}, done {cur_done}")
            log_img_text_pairs += [(None, "\n".join(before_texts)), log_img_text_pair, (None, "\n".join(after_texts))]
            break

        after_texts.append(f"\nscore {cur_score}, done {cur_done}")
        log_img_text_pairs += [(None, "\n".join(before_texts)), log_img_text_pair, (None, "\n".join(after_texts))]

    generate_html(log_img_text_pairs, save_dir=exp_dir, filename="rollout.html")

    print(f"Task completed: {cur_done}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="collect rollouts")
    parser.add_argument("--seed", default=12, type=int)
    parser.add_argument("--semantic_spec_seed", default=853, type=int)
    parser.add_argument("--config_file", default='../configs/evaluate_clean_dish_feg_collect_rollouts_0922.yaml', type=str)
    args = parser.parse_args()

    run_evaluation(args)

