#!/usr/bin/env python

from __future__ import print_function
import os
import json
import pickle
import shutil
from os import listdir
from os.path import join, abspath, dirname, isdir, isfile
from tabnanny import verbose
from config import EXP_PATH, MAMAO_DATA_PATH
import numpy as np
import random
import time
import argparse

from pybullet_tools.pr2_utils import get_group_conf
from pybullet_tools.utils import disconnect, LockRenderer, has_gui, WorldSaver, wait_if_gui, \
    SEPARATOR, get_aabb, wait_for_duration, safe_remove, ensure_dir, reset_simulation
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, get_datetime
from pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn, \
    get_stream_map, solve_multiple, solve_one
from pybullet_tools.logging import TXT_FILE

from pybullet_tools.pr2_primitives import get_group_joints, Conf, get_base_custom_limits, Pose, Conf, \
    get_ik_ir_gen, get_motion_gen, get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    get_grasp_gen, Attach, Detach, Clean, Cook, control_commands, Command, \
    get_gripper_joints, GripperCommand, State, apply_commands

from pddlstream.language.constants import Equal, AND, print_solution, PDDLProblem
from pddlstream.algorithms.meta import solve, DEFAULT_ALGORITHM
from pddlstream.algorithms.constraints import PlanConstraints
from pddlstream.algorithms.algorithm import reset_globals
from pddlstream.algorithms.common import SOLUTIONS
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, TmpCWD

from lisdf_tools.lisdf_loader import load_lisdf_pybullet, pddlstream_from_dir
from lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.actions import apply_actions

from mamao_tools.utils import get_feasibility_checker
from mamao_tools.feasibility_checkers import Shuffler
from mamao_tools.data_utils import get_instance_info, exist_instance

from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser


GENERATE_SKELETONS = False
USE_VIEWER = False
DIVERSE = True
PREFIX = 'diverse_' if DIVERSE else ''
RERUN_SUBDIR = 'rerun_2'

SKIP_IF_SOLVED = False and not GENERATE_SKELETONS
SKIP_IF_SOLVED_RECENTLY = True and not GENERATE_SKELETONS
RETRY_IF_FAILED = True
check_time = 1665010453  ## 1664908130, 1664976972 for pick, 1664750094 for in

# TASK_NAME = 'tt_one_fridge_pick'
TASK_NAME = 'tt_one_fridge_table_pick'
# TASK_NAME = 'tt_one_fridge_table_in'
TASK_NAME = 'tt_two_fridge_pick'
# TASK_NAME = 'tt_two_fridge_in'
TASK_NAME = 'tt'

# TASK_NAME = 'mm_two_fridge_in'
# TASK_NAME = 'mm'

CASES = None
# CASES = ['11']
if CASES is not None:
    SKIP_IF_SOLVED = False
    SKIP_IF_SOLVED_RECENTLY = False

PARALLEL = GENERATE_SKELETONS ## and False
FEASIBILITY_CHECKER = 'pvt-124'  ## None | oracle | pvt | pvt* | binary | shuffle
if GENERATE_SKELETONS:
    FEASIBILITY_CHECKER = 'oracle'

## =========================================

parser = get_base_parser(task_name=TASK_NAME, parallel=PARALLEL, use_viewer=USE_VIEWER)
parser.add_argument('-d', type=str, default=DIVERSE)
parser.add_argument('-f', type=str, default=FEASIBILITY_CHECKER)
parser.add_argument('-u', '--unlock', action='store_true',
                    help='When enabled, unlocks the PyBullet viewer.')
parser.add_argument('-c', '--cfree', action='store_true',
                    help='When enabled, disables collision checking.')
parser.add_argument('-i', '--index', type=int, default=0,
                    help='The index of the first problem.')
args = parser.parse_args()

TASK_NAME = args.t
DIVERSE = args.d
PARALLEL = args.p
FEASIBILITY_CHECKER = args.f

# DATABASE_DIR = abspath(join(MAMAO_DATA_PATH, TASK_NAME))


def init_experiment(exp_dir):
    if isfile(TXT_FILE):
        os.remove(TXT_FILE)

#####################################


def clear_all_rerun_results(run_dir, **kwargs):
    files = [f for f in listdir(run_dir) if '_fc' in f or 'fc_' in f]
    if len(files) > 0:
        print('\n'+run_dir)
        print('clearing files', '\n'.join(files))
        rerun_dir = join(run_dir, RERUN_SUBDIR)
        if not isdir(rerun_dir):
            os.mkdir(rerun_dir)
        for f in files:
            shutil.move(join(run_dir, f), join(rerun_dir, f))
        seg_images_dir = join(run_dir, 'seg_images')
        if isdir(seg_images_dir):
            shutil.rmtree(seg_images_dir)

    ## delete those with problematic meshes
    # model_instances = get_instance_info(run_dir)
    # if exist_instance(model_instances, '10849') or exist_instance(model_instances, '11178'):
    #     print('clearing files', run_dir)
    #     shutil.rmtree(run_dir)


def run_one(run_dir, parallel=False, SKIP_IF_SOLVED=SKIP_IF_SOLVED):
    if GENERATE_SKELETONS:
        file = join(run_dir, f'diverse_plans.json')
        if isfile(file):
            return

    else:
        ori_dir = join(run_dir, RERUN_SUBDIR)  ## join(DATABASE_DIR, run_dir)
        if not isdir(ori_dir):
            os.mkdir(ori_dir)
        file = join(ori_dir, f'{PREFIX}plan_rerun_fc={FEASIBILITY_CHECKER}.json')
        if isfile(file):  ## and not '/11' in ori_dir
            failed = False
            if RETRY_IF_FAILED:
                failed = json.load(open(file, 'r'))['plan'] is None

            if not RETRY_IF_FAILED or not failed:
                if SKIP_IF_SOLVED:
                    print('skipping solved problem', run_dir)
                    return
                elif SKIP_IF_SOLVED_RECENTLY:
                    last_modified = os.path.getmtime(file)
                    if last_modified > check_time:
                        print('skipping recently solved problem', run_dir)
                        return

    exp_dir = copy_dir_for_process(run_dir, tag='replaying')

    if False:
        from utils import load_lisdf_synthesizer
        scene = load_lisdf_synthesizer(exp_dir)

    world = load_lisdf_pybullet(exp_dir, verbose=False, use_gui=args.viewer) ## , width=720, height=560
    saver = WorldSaver()
    problem = Problem(world)

    if False:
        from utils import load_lisdf_nvisii
        scene = load_lisdf_nvisii(exp_dir)

    update_fn = None
    if False:
        #from test_gym import load_lisdf_isaacgym
        from test_gym import update_gym_world
        gym_world = load_lisdf_isaacgym(exp_dir) #, skip=['meatturkeyleg'])
        #world.gym_world = gym_world
        update_fn = lambda pause=False: update_gym_world(gym_world, pb_world=world, pause=pause)
        update_fn(pause=True)
        #gym_world.wait_if_gui()

    pddlstream_problem = pddlstream_from_dir(problem, exp_dir=exp_dir, replace_pddl=True,
                                             collisions=not args.cfree, teleport=False)
    world.summarize_all_objects(pddlstream_problem.init)

    stream_info = world.robot.get_stream_info(partial=False, defer=False)
    _, _, _, stream_map, init, goal = pddlstream_problem
    summarize_facts(init, world=world)
    print_goal(goal)
    print(SEPARATOR)
    init_experiment(exp_dir)

    fc = get_feasibility_checker(run_dir, mode=FEASIBILITY_CHECKER, diverse=DIVERSE)
    # fc = Shuffler()

    start = time.time()
    if DIVERSE:
        kwargs = dict(
            diverse=DIVERSE,
            downward_time=10,  ## max time to get 100, 10 sec, 30 sec for 300
            evaluation_time=60,  ## on each skeleton
            max_plans=100,  ## number of skeletons
        )
        if GENERATE_SKELETONS:
            kwargs['evaluation_time'] = -0.5
        if '_in/' in run_dir:
            kwargs['downward_time'] = 10
            kwargs['max_plans'] = 100
    else:
        kwargs = dict()

    if parallel:
        solution = solve_multiple(pddlstream_problem, stream_info, fc=fc, lock=not args.unlock, **kwargs)
    else:
        solution = solve_one(pddlstream_problem, stream_info, fc=fc, lock=not args.unlock, **kwargs)

    ## just to get all diverse plans as labels
    if GENERATE_SKELETONS:
        ori_dir = join(run_dir, 'rerun_1')  ## join(DATABASE_DIR, run_dir)
        if isdir(ori_dir) and len(listdir(ori_dir)) == 0:
            shutil.rmtree(ori_dir)

        fc.dump_log(join(run_dir, f'diverse_plans.json'), plans_only=True)
        reset_simulation()
        shutil.rmtree(exp_dir)
        return

    planning_time = time.time() - start
    saver.restore()

    print_solution(solution)
    plan, cost, evaluations = solution
    # if (plan is None) or not has_gui():
    #     disconnect()
    #     return

    """ log plan, planning stats, commands, and fc stats """
    with open(join(ori_dir, f'{PREFIX}plan_rerun_fc={FEASIBILITY_CHECKER}.json'), 'w') as f:
        data = {
            'planning_time': planning_time,
            'plan': [[str(a.name)]+[str(v) for v in a.args] for a in plan] if plan is not None else None,
            'datatime': get_datetime(),
        }
        json.dump(data, f, indent=3)

    fc.dump_log(join(ori_dir, f'{PREFIX}fc_log={FEASIBILITY_CHECKER}.json'))

    if plan is not None:
        print(SEPARATOR)
        with LockRenderer(lock=True):
            commands = post_process(problem, plan)
            print('Commands:', commands)
            problem.remove_gripper()
            saver.restore()
        with open(join(ori_dir, f'{PREFIX}commands_rerun_fc={FEASIBILITY_CHECKER}.pkl'), 'wb') as f:
            pickle.dump(commands, f)
        if has_gui():
            saver.restore()
            input('Begin?')
            apply_actions(problem, commands, time_step=5e-2, verbose=False)
            input('End?')

    # disconnect()
    reset_simulation()
    shutil.rmtree(exp_dir)


def process(index):
    t = int(time.time())
    print('current time', t)
    np.random.seed(t)
    random.seed(t)
    return run_one(str(index), parallel=PARALLEL)


# def main(parallel=True, cases=None):
#     if isdir('visualizations'):
#         shutil.rmtree('visualizations')
#
#     start_time = time.time()
#     if cases is None:
#         cases = [join(DATABASE_DIR, f) for f in listdir(DATABASE_DIR) if isdir(join(DATABASE_DIR, f))]
#         cases.sort()
#     else:
#         cases = [join(DATABASE_DIR, c) for c in cases]
#     print('Cases:', cases)
#
#     num_cases = len(cases)
#     if parallel:
#         import multiprocessing
#         from multiprocessing import Pool
#
#         max_cpus = 24
#         num_cpus = min(multiprocessing.cpu_count(), max_cpus)
#         print(f'using {num_cpus} cpus')
#         with Pool(processes=num_cpus) as pool:
#             # for result in pool.imap_unordered(process, range(num_cases)):
#             #     pass
#             pool.map(process, cases)
#             # pool.map(process, range(num_cases))
#
#     else:
#         for i in range(num_cases):
#             # if i in [0, 1]: continue
#             # if '/11' not in cases[i]: continue
#             process(cases[i], parallel=False)
#
#     print(f'solved {num_cases} problems (parallel={parallel}) in {round(time.time() - start_time, 3)} sec')


if __name__ == '__main__':
    process_all_tasks(process, args.t, parallel=PARALLEL, cases=CASES)
    # process_all_tasks(clear_all_rerun_results, args.t, parallel=False)

