#!/usr/bin/env python

from __future__ import print_function
import os
import json
from os.path import join, abspath, dirname, isdir, isfile
from config import EXP_PATH

from pybullet_planning.pybullet_tools.pr2_utils import get_group_conf
from pybullet_planning.pybullet_tools.pr2_primitives import get_base_custom_limits, control_commands, apply_commands, State
from pybullet_planning.pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb
from pybullet_planning.pybullet_tools.bullet_utils import summarize_facts, print_goal, nice
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
    get_cfree_btraj_pose_test, get_joint_position_open_gen, get_ik_ungrasp_mark_gen, \
    sample_joint_position_open_list_gen, get_update_wconf_pst_gen, get_ik_ir_wconf_gen, \
    get_update_wconf_p_gen, get_ik_ir_wconf_gen, get_pose_in_space_test, get_turn_knob_handle_motion_gen
from pybullet_planning.pybullet_tools.pr2_primitives import get_group_joints, Conf, get_base_custom_limits, Pose, Conf, \
    get_ik_ir_gen, get_motion_gen, get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, \
    get_gripper_joints, GripperCommand, apply_commands, State
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen, from_test

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object
from pddlstream.algorithms.meta import solve, create_parser

from pybullet_planning.lisdf_tools.lisdf_loader import load_lisdf_pybullet
from pybullet_planning.lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from test_pddlstream import get_args

DEFAULT_TEST = 'kitchen' ## 'blocks_pick'

def run_tests(p, init, l, c=True, t=True):
    test_handle_grasp_gen(p, init)


def test_handle_grasp_gen(p, init, visualize=True):
    from pybullet_tools.pr2_streams import get_handle_pose
    joints = [f[1] for f in init if f[0] == 'joint']
    funk = get_handle_grasp_gen(p, visualize=False)
    for j in joints:
        if 'knob' not in p.world.body_to_name[j]: continue
        outputs = funk(j)
        if visualize:
            body_pose = get_handle_pose(j)
            visualize_grasps(p, outputs, body_pose, RETAIN_ALL=True)
    wait_if_gui('Finish?')

# ####################################

def main(exp_name, verbose=True):
    args = get_args(exp_name)

    exp_dir = join(EXP_PATH, args.test)
    world = load_lisdf_pybullet(join(exp_dir, 'scene.lisdf'))

    problem = Problem(world)
    init = pddl_to_init_goal(exp_dir, world)[0]
    world.summarize_all_objects()
    summarize_facts(init)

    planning_config = json.load(open(join(exp_dir, 'planning_config.json')))
    custom_limits = get_base_custom_limits(world.robot, planning_config['base_limits'])

    run_tests(problem, init, custom_limits)

    wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main(exp_name=DEFAULT_TEST)
