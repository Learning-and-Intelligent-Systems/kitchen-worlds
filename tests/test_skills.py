#!/usr/bin/env python

from __future__ import print_function
import os
import json
from os.path import join, abspath, dirname, isdir, isfile
from config import EXP_PATH

from pybullet_planning.pybullet_tools.pr2_utils import get_group_conf
from pybullet_planning.pybullet_tools.pr2_primitives import get_base_custom_limits, control_commands, apply_commands
from pybullet_planning.pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb
from pybullet_planning.pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, set_camera_target_body
from pybullet_planning.pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn, \
    visualize_grasps_by_quat, visualize_grasps
from pybullet_planning.pybullet_tools.logging import TXT_FILE

## custom stream_map
from pybullet_planning.pybullet_tools.pr2_streams import get_stable_gen, get_contain_gen, get_position_gen, \
    Position, get_handle_grasp_gen, LinkPose, get_ik_ir_grasp_handle_gen, get_pull_drawer_handle_motion_gen, \
    get_joint_position_test, get_marker_grasp_gen, get_bconf_in_region_test, get_pull_door_handle_motion_gen, \
    get_bconf_in_region_gen, get_pose_in_region_gen, visualize_grasp, get_motion_wconf_gen, get_update_wconf_p_two_gen, \
    get_marker_pose_gen, get_pull_marker_to_pose_motion_gen, get_pull_marker_to_bconf_motion_gen,  \
    get_pull_marker_random_motion_gen, get_ik_ungrasp_handle_gen, get_pose_in_region_test, \
    get_cfree_btraj_pose_test, get_joint_position_open_gen, get_ik_ungrasp_mark_gen, get_handle_pose, \
    sample_joint_position_open_list_gen, get_update_wconf_pst_gen, get_ik_ir_wconf_gen, \
    get_update_wconf_p_gen, get_ik_ir_wconf_gen, get_pose_in_space_test, get_turn_knob_handle_motion_gen
from pybullet_planning.pybullet_tools.pr2_primitives import get_group_joints, Conf, get_base_custom_limits, Pose, Conf, \
    get_ik_ir_gen, get_motion_gen, get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen, from_test

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object
from pddlstream.algorithms.meta import solve, create_parser

from pybullet_planning.lisdf_tools.lisdf_loader import load_lisdf_pybullet
from pybullet_planning.lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.world import State

from test_pddlstream import get_args

DEFAULT_TEST = 'kitchen' ## 'blocks_pick'

def run_tests(p, init, l, c=True, t=True):
     test_handle_grasp_gen(p, init)


def test_handle_grasp_gen(p, init, visualize=True):
    from pybullet_tools.pr2_streams import get_handle_pose
    joints = [f[1] for f in init if f[0] == 'joint']
    funk = get_handle_grasp_gen(p, visualize=False)
    for j in joints:
        if 'faucet' not in p.world.body_to_name[j]: continue
        outputs = funk(j)
        if visualize:
            body_pose = get_handle_pose(j)
            set_camera_target_body(j[0], dx=0.5, dy=0.5, dz=0.8)
            visualize_grasps(p, outputs, body_pose, RETAIN_ALL=True)
            set_camera_target_body(j[0], dx=0.5, dy=0.5, dz=0.8)

    wait_if_gui('Finish?')

# ####################################

ASSET_PATH = join('..', 'assets')
models = {
    'Fridge': {
        # '10144': 1.09,
        '10905': 1,
        '11299': 1,
        '11846': 1,
        '12036': 1,
        '12248': 1
    }
}
custom_limits = {0: (-5, 5), 1: (-5, 5), 2: (0, 3)}

from pybullet_tools.pr2_problems import create_floor
from pybullet_tools.utils import connect, draw_pose, unit_pose, link_from_name, load_pybullet, load_model, \
    sample_aabb, AABB, set_pose, get_aabb, get_aabb_center, quat_from_euler, Euler, HideOutput, get_aabb_extent, \
    set_camera_pose
from pybullet_tools.flying_gripper_utils import create_fe_gripper, set_se3_conf
from lisdf_tools.lisdf_loader import World
import math

def get_z_on_floor(body):
    return get_aabb_extent(get_aabb(body))[-1]/2

def get_floor_aabb(custom_limits):
    x_min, x_max = custom_limits[0]
    y_min, y_max = custom_limits[1]
    return AABB(lower=(x_min, y_min), upper=(x_max, y_max))

def sample_pose_on_floor(body, custom_limits):
    x, y = sample_aabb(get_floor_aabb(custom_limits))
    z = get_z_on_floor(body)
    return ((x, y, z), quat_from_euler((0, 0, math.pi)))

def pose_from_2d(body, xy):
    z = get_z_on_floor(body)
    return ((xy[0], xy[1], z), quat_from_euler((0, 0, math.pi)))

def test_fridges(world, custom_limits):
    problem = State(world)
    funk = get_handle_grasp_gen(problem, visualize=False)

    ## load fridge
    n = len(models['Fridge'])
    i = 0
    locations = [(0, 2*n) for n in range(0, n)]
    set_camera_pose((4, 3, 2), (0, 3, 0.5))
    for id, scale in models['Fridge'].items():

        path = join(ASSET_PATH, 'models', 'Fridge', id)

        ## load objects
        file = join(path, 'mobility.urdf')
        print('loading', file)
        with HideOutput(True):
            body = load_model(file, scale=scale)
            if isinstance(body, tuple): body = body[0]
        pose = pose_from_2d(body, locations[i])
        set_pose(body, pose)
        world.add_body(body, f'fridge#{id}')
        # set_camera_target_body(body, dx=1, dy=1, dz=1)

        ## color links corresponding to semantic labels
        file = join(path, 'semantics.txt')
        body_joints = world.add_semantic_label(body, file)
        for body_joint in body_joints:
            outputs = funk(body_joint)
            body_pose = get_handle_pose(body_joint)
            # set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.8)
            visualize_grasps(problem, outputs, body_pose, RETAIN_ALL=True)
            # set_camera_target_body(body, dx=0.5, dy=0.5, dz=0.8)

        i += 1

    ## sample grasp
    set_camera_pose((4, 3, 2), (0, 3, 0.5))
    print()

    ## sample traj

    ## execute traj

def main(exp_name, verbose=True):
    args = get_args(exp_name)

    connect(use_gui=True, shadows=False, width=1980, height=1238)
    draw_pose(unit_pose(), length=2.)
    create_floor()

    init_q = [3, 1, 1, 0, 0, 0]
    robot = create_fe_gripper(init_q=init_q)

    world = World(args)
    world.add_robot(robot, 'feg')

    test_fridges(world, custom_limits)

    wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main(exp_name=DEFAULT_TEST)
