#!/usr/bin/env python

from __future__ import print_function
import os
import json
from os.path import join, abspath, dirname, isdir, isfile
from config import EXP_PATH

from pybullet_planning.pybullet_tools.pr2_primitives import get_base_custom_limits, control_commands, apply_commands, State
from pybullet_planning.pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, SEPARATOR
from pybullet_planning.pybullet_tools.bullet_utils import summarize_facts, print_goal
from pybullet_planning.pybullet_tools.pr2_agent import get_stream_info, get_stream_map, post_process
from pybullet_planning.pybullet_tools.logging import TXT_FILE

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object
from pddlstream.algorithms.meta import solve, create_parser

from pybullet_planning.lisdf_tools.lisdf_loader import load_lisdf_pybullet
from pybullet_planning.lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

def pddlstream_from_dir(problem, exp_dir, collisions=True, teleport=False):

    world = problem.world

    domain_pddl = read(join(exp_dir, 'domain_full.pddl'))
    stream_pddl = read(join(exp_dir, 'stream.pddl'))
    planning_config = json.load(open('planning_config.json'))
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

#######################################################

def main(exp_dir, partial=False, defer=False, verbose=True):
    parser = create_parser()
    parser.add_argument('-cfree', action='store_true', help='Disables collisions during planning')
    parser.add_argument('-enable', action='store_true', help='Enables rendering during planning')
    parser.add_argument('-teleport', action='store_true', help='Teleports between configurations')
    parser.add_argument('-simulate', action='store_true', help='Simulates the system')
    args = parser.parse_args()
    print('Arguments:', args)

    world = load_lisdf_pybullet(join(exp_dir, 'scene.lisdf'))
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
