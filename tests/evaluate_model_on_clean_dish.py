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

def load_model_and_cfg():

    # create a new argparser args object to store the model arguments
    args = argparse.Namespace()
    args.base_config_file = "/home/weiyu/Research/nsplan/nsplan/configs/base.yaml"
    # args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTFiLM1DDynamicsFeasibilityModel_subsample_trajectory.yaml"
    # args.checkpoint_id = "gq6ui8m9"

    args.config_file = "/home/weiyu/Research/nsplan/nsplan/configs/PCTFiLM1DDynamicsFeasibilityModel.yaml"
    args.checkpoint_id = "026qc2hq"

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
    run_high_level_policy(env)


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


def run_high_level_policy(env: CleanDishEnvV1, max_depth=10, debug=True):

    # TODO: load from model or data config

    model, cfg = load_model_and_cfg()
    device = model.device

    # ----------------------------------
    # hyperparams:
    num_obj_pts = cfg.DATASET.num_obj_pts
    num_scene_pts = cfg.DATASET.num_scene_pts

    # ----------------------------------
    # meta data
    obj_dict = env.world.obj_dict
    for obj in obj_dict:
        obj_dict[obj]["name"] = str(obj_dict[obj]["name"])
    obj_name_to_info = MultiviewSequenceDataset.get_obj_name_to_info(obj_dict)

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

    # ----------------------------------
    # start acting
    cur_obs, info = env.reset()

    symbolic_state = env.symbolic_state

    for t in range(max_depth):

        # --------------------------------
        # build current observation
        furniture_names = ["sink#1", "sink_counter_left", "sink_counter_right", "cabinettop", "shelf_lower"]
        obj_names = ["scene"] + ["robot"] + furniture_names + sorted(obj_name_to_info.keys())
        scene_xyzrgb = extract_scene_xyzrgb(cur_obs, obj_names, obj_name_to_pybullet_idx,
                                            num_pts_object=num_obj_pts,
                                            num_pts_scene=num_scene_pts, visualize=False)
        scene_xyzrgb = scene_xyzrgb[:, :6]  # ignore alpha
        scene_xyzrgb[:, :3] = pc_normalize(scene_xyzrgb[:, :3])

        if debug:
            print("\n\n" + "=" * 100)
            print(f"timestep {t}")
            print(f"symbolic state: {symbolic_state}")
            # plot_images({view_name: Image.fromarray(cur_obs[view_name][0], 'RGBA') for view_name in cur_obs})
            trimesh.PointCloud(scene_xyzrgb[:, :3], scene_xyzrgb[:, 3:]).show()

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
        if symbolic_state["grasped"] is None:
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
        print(query_actions_scores.shape)

        # sort query_actions and query_action_scores by query_action_scores
        ranking_idx = np.argsort(query_actions_scores)[::-1]
        sorted_query_actions = [query_actions[idx] for idx in ranking_idx]
        sorted_query_actions_scores = query_actions_scores[ranking_idx]

        for query_action, query_action_score in zip(sorted_query_actions, sorted_query_actions_scores):
            print(query_action, query_action_score)

        input("next")

        # --------------------------------
        # step the environment
        # action = env.convert_text_to_action(text_action[0], text_action[1], text_action[2])
        # new_obs, new_score, new_done, _, _ = env.step(action)

def run_evaluation(env_config_file, env_seed, semantic_spec_seed):
    env_config = parse_yaml(env_config_file)

    # override
    env_config.seed = env_seed
    env_config.semantic_spec_seed = semantic_spec_seed

    # change data.out_dir for storing evaluation data
    env_config.data.out_dir = os.path.join(os.path.split(env_config.data.out_dir)[0], "evaluation")
    print(f"Saving evaluation data to {abspath(env_config.data.out_dir)}")

    process(env_config)


if __name__ == '__main__':
    run_evaluation(env_config_file='../configs/clean_dish_feg_collect_rollouts.yaml',
                   env_seed=0,
                   semantic_spec_seed=0)

