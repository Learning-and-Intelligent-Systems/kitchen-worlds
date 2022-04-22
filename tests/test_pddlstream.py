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
from pybullet_planning.pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn ## , get_stream_map
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

def get_stream_map(p, c, l, t):
    # p = problem
    # c = collisions
    # l = custom_limits
    # t = teleport
    stream_map = {
        'sample-pose': from_gen_fn(get_stable_gen(p, collisions=c)),
        'sample-pose-inside': from_gen_fn(get_contain_gen(p, collisions=c)),  ##
        'sample-grasp': from_list_fn(get_grasp_gen(p, collisions=True)),
        'inverse-kinematics': from_gen_fn(get_ik_ir_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                        learned=False, max_attempts=60, verbose=False)),
        'inverse-kinematics-wconf': from_gen_fn(get_ik_ir_wconf_gen(p, collisions=c, teleport=t, custom_limits=l,
                                                                    learned=False, max_attempts=60, verbose=False,
                                                                    visualize=False)),
        'plan-base-motion': from_fn(get_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-base-motion-wconf': from_fn(get_motion_wconf_gen(p, collisions=c, teleport=t, custom_limits=l)),

        'test-cfree-pose-pose': from_test(get_cfree_pose_pose_test(collisions=c)),
        'test-cfree-approach-pose': from_test(get_cfree_approach_pose_test(p, collisions=c)),
        'test-cfree-traj-pose': from_test(get_cfree_traj_pose_test(p.robot, collisions=c)),

        'test-cfree-btraj-pose': from_test(get_cfree_btraj_pose_test(p.robot, collisions=c)),

        # 'get-joint-position-open': from_fn(get_joint_position_open_gen(p)),
        'get-joint-position-open': from_list_fn(sample_joint_position_open_list_gen(p)),
        # 'sample-joint-position-open': from_fn(get_position_gen(p, collisions=c, extent='max')),
        # 'sample-joint-position-closed': from_fn(get_position_gen(p, collisions=c, extent='min')),
        # 'test-joint-position-open': from_test(get_joint_position_test(extent='max')),
        # 'test-joint-position-closed': from_test(get_joint_position_test(extent='min')),

        'sample-handle-grasp': from_list_fn(get_handle_grasp_gen(p, collisions=c)),

        'inverse-kinematics-grasp-handle': from_gen_fn(
            get_ik_ir_grasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                       learned=False, verbose=False, ACONF=True, WCONF=False)),
        'inverse-kinematics-ungrasp-handle': from_gen_fn(
            get_ik_ungrasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                      verbose=False, WCONF=False)),
        'inverse-kinematics-grasp-handle-wconf': from_gen_fn(
            get_ik_ir_grasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                       learned=False, verbose=False, ACONF=True, WCONF=True)),
        'inverse-kinematics-ungrasp-handle-wconf': from_gen_fn(
            get_ik_ungrasp_handle_gen(p, collisions=c, teleport=t, custom_limits=l,
                                      verbose=False, WCONF=True)),

        'plan-base-pull-drawer-handle': from_fn(
            get_pull_drawer_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-base-pull-door-handle': from_fn(
            get_pull_door_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),
        'plan-arm-turn-knob-handle': from_fn(
            get_turn_knob_handle_motion_gen(p, collisions=c, teleport=t, custom_limits=l)),

        'sample-marker-grasp': from_list_fn(get_marker_grasp_gen(p, collisions=c)),
        'inverse-kinematics-grasp-marker': from_gen_fn(
            get_ik_ir_grasp_handle_gen(p, collisions=True, teleport=t, custom_limits=l,
                                       learned=False, verbose=False)),
        'inverse-kinematics-ungrasp-marker': from_fn(
            get_ik_ungrasp_mark_gen(p, collisions=True, teleport=t, custom_limits=l)),
        'plan-base-pull-marker-random': from_gen_fn(
            get_pull_marker_random_motion_gen(p, collisions=c, teleport=t, custom_limits=l,
                                              learned=False)),

        'sample-marker-pose': from_list_fn(get_marker_pose_gen(p, collisions=c)),
        'plan-base-pull-marker-to-bconf': from_fn(get_pull_marker_to_bconf_motion_gen(p, collisions=c, teleport=t)),
        'plan-base-pull-marker-to-pose': from_fn(get_pull_marker_to_pose_motion_gen(p, collisions=c, teleport=t)),
        'test-bconf-in-region': from_test(get_bconf_in_region_test(p.robot)),
        'test-pose-in-region': from_test(get_pose_in_region_test()),
        'test-pose-in-space': from_test(get_pose_in_space_test()),  ##

        # 'sample-bconf-in-region': from_gen_fn(get_bconf_in_region_gen(p, collisions=c, visualize=False)),
        'sample-bconf-in-region': from_list_fn(get_bconf_in_region_gen(p, collisions=c, visualize=False)),
        'sample-pose-in-region': from_list_fn(get_pose_in_region_gen(p, collisions=c, visualize=False)),

        'update-wconf-p': from_fn(get_update_wconf_p_gen()),
        'update-wconf-p-two': from_fn(get_update_wconf_p_two_gen()),
        'update-wconf-pst': from_fn(get_update_wconf_pst_gen()),

        'MoveCost': move_cost_fn,

        # 'TrajPoseCollision': fn_from_constant(False),
        # 'TrajArmCollision': fn_from_constant(False),
        # 'TrajGraspCollision': fn_from_constant(False),
    }
    return stream_map

def pddlstream_from_dir(problem, exp_dir, collisions=True, teleport=False):

    world = problem.world

    domain_pddl = read(join(exp_dir, 'domain_full.pddl'))
    stream_pddl = read(join(exp_dir, 'stream.pddl'))
    planning_config = json.load(open(join(exp_dir, 'planning_config.json')))
    constant_map = {}

    init, goal = pddl_to_init_goal(exp_dir, world)
    goal = [AND] + goal
    problem.add_init(init)

    base_limits = planning_config['base_limits']
    custom_limits = get_base_custom_limits(world.robot, base_limits)
    stream_map = get_stream_map(problem, collisions, custom_limits, teleport)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def init_experiment(exp_dir):
    if isfile(TXT_FILE):
        os.remove(TXT_FILE)

###################
# ####################################

def main(exp_dir, partial=False, defer=False, verbose=True):
    parser = create_parser()
    parser.add_argument('-cfree', action='store_true', help='Disables collisions during planning')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()
    print('Arguments:', args)

    world = load_lisdf_pybullet(join(exp_dir, 'scene.lisdf'))
    world.summarize_all_objects()
    saver = WorldSaver()
    problem = Problem(world)

    pddlstream_problem = pddlstream_from_dir(problem, exp_dir=exp_dir, collisions=not args.cfree,
                                             teleport=args.teleport)

    stream_info = get_stream_info(partial, defer)
    _, _, _, stream_map, init, goal = pddlstream_problem
    summarize_facts(init)
    print_goal(goal)
    print(SEPARATOR)
    init_experiment(exp_dir)

    with Profiler():
        with LockRenderer(lock=not args.enable):
            solution = solve(pddlstream_problem, algorithm=args.algorithm, unit_costs=args.unit,
                             stream_info=stream_info, success_cost=INF, verbose=True, debug=False)
            saver.restore()

    print_solution(solution)
    plan, cost, evaluations = solution
    if (plan is None) or not has_gui():
        disconnect()
        return

    print(SEPARATOR)
    with LockRenderer(lock=not args.enable):
        commands = post_process(problem, plan)
        problem.remove_gripper()
        saver.restore()

    #restore_state(state_id)
    saver.restore()
    wait_if_gui('Execute?')
    if args.simulate:
        control_commands(commands)
    else:
        apply_commands(State(), commands, time_step=0.01)
    wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    exp_name = 'test_pick' ## 'blocks_kitchen'
    exp_dir = join(EXP_PATH, exp_name)
    main(exp_dir=exp_dir)
