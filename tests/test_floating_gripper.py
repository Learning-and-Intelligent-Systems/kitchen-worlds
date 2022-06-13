#!/usr/bin/env python

from __future__ import print_function
import os
import json
from os.path import join, abspath, dirname, isdir, isfile
from config import EXP_PATH

from pybullet_tools.pr2_utils import get_group_conf
from pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, wait_for_duration
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice
from pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn, stream_info
from pybullet_tools.logging import TXT_FILE

from pybullet_tools.pr2_primitives import get_group_joints, Conf, get_base_custom_limits, Pose, Conf, \
    get_ik_ir_gen, get_motion_gen, get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, Command, \
    get_gripper_joints, GripperCommand, State
from pybullet_tools.flying_gripper_agent import get_stream_map

from pddlstream.language.generator import from_gen_fn, from_list_fn, from_fn, fn_from_constant, empty_gen, from_test
from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object
from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.algorithms.focused import solve_focused

from pybullet_planning.lisdf_tools.lisdf_loader import load_lisdf_pybullet
from pybullet_planning.lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.actions import apply_actions


DEFAULT_TEST = 'test_feg_cabinets_rearrange'  ## success
# DEFAULT_TEST = 'test_feg_clean_only'  ## success
# DEFAULT_TEST = 'test_feg_clean_after_open'  ## fail


def pddlstream_from_dir(problem, exp_dir, collisions=True, teleport=False):

    world = problem.world

    domain_pddl = read(join(exp_dir, 'domain_full.pddl'))
    stream_pddl = read(join(exp_dir, 'stream.pddl'))
    planning_config = json.load(open(join(exp_dir, 'planning_config.json')))

    init, goal, constant_map = pddl_to_init_goal(exp_dir, world)
    goal = [AND] + goal
    problem.add_init(init)

    custom_limits = planning_config['base_limits']
    stream_map = get_stream_map(problem, collisions, custom_limits, teleport)

    return PDDLProblem(domain_pddl, constant_map, stream_pddl, stream_map, init, goal)

def init_experiment(exp_dir):
    if isfile(TXT_FILE):
        os.remove(TXT_FILE)

def get_args(exp_name):
    parser = create_parser()
    parser.add_argument('-test', type=str, default=exp_name, help='Name of the test case')
    parser.add_argument('-cfree', action='store_true', help='Disables collisions during planning')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()
    print('Arguments:', args)
    return args

#####################################

def main(exp_name, verbose=True):

    args = get_args(exp_name)

    exp_dir = join(EXP_PATH, args.test)
    world = load_lisdf_pybullet(exp_dir, width=1280, height=960) ## , width=720, height=560)
    saver = WorldSaver()
    problem = Problem(world)

    pddlstream_problem = pddlstream_from_dir(problem, exp_dir=exp_dir, collisions=not args.cfree,
                                             teleport=args.teleport)
    _, _, _, stream_map, init, goal = pddlstream_problem
    world.summarize_all_objects(init)

    # stream_info = get_stream_info(partial=False, defer=False)  ## problem
    summarize_facts(init, world=world)
    print_goal(goal)
    print(SEPARATOR)
    init_experiment(exp_dir)

    with Profiler():
        with LockRenderer(lock=not args.enable):
            solution = solve_focused(pddlstream_problem, stream_info=stream_info,
                                     planner='ff-astar1', max_planner_time=10, debug=False,
                                     unit_costs=True, success_cost=INF,
                                     max_time=INF, verbose=True, visualize=False,
                                     unit_efforts=True, effort_weight=1,
                                     bind=True, max_skeletons=INF,
                                     search_sample_ratio=0)
            # solution = solve(pddlstream_problem, algorithm=args.algorithm, unit_costs=args.unit,
            #                  stream_info=stream_info, success_cost=INF, verbose=True, debug=False)
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

    saver.restore()
    wait_if_gui('Execute?')
    if args.simulate:  ## real physics
        control_commands(commands)
    else:
        # apply_commands(State(), commands, time_step=0.01)
        apply_actions(problem, commands, time_step=0.1)
    wait_if_gui('Finish?')
    disconnect()



if __name__ == '__main__':
    main(exp_name=DEFAULT_TEST)
