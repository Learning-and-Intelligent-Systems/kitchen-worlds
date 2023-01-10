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
    SEPARATOR, get_aabb, wait_for_duration, safe_remove, ensure_dir, reset_simulation, timeout
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, get_datetime, \
    initialize_logs
from pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn, \
    get_stream_map, solve_multiple, solve_one

from pddlstream.language.constants import Equal, AND, print_solution

from lisdf_tools.lisdf_loader import load_lisdf_pybullet, pddlstream_from_dir
from lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.actions import apply_actions

from mamao_tools.feasibility_checkers import Shuffler
from mamao_tools.data_utils import get_instance_info, exist_instance, get_indices, \
    get_plan_skeleton, get_successful_plan, get_feasibility_checker, get_plan

from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser, get_body_map, \
    modify_plan_with_body_map

## special modes
GENERATE_MULTIPLE_SOLUTIONS = False
GENERATE_SKELETONS = False

USE_VIEWER = False
LOCK_VIEWER = True
DIVERSE = True
PREFIX = 'diverse_' if DIVERSE else ''
RERUN_SUBDIR = 'rerun'

SKIP_IF_SOLVED = True and not GENERATE_SKELETONS
SKIP_IF_SOLVED_RECENTLY = True and not GENERATE_SKELETONS
RETRY_IF_FAILED = True
check_time = 1666297068  ## 1665768219 for goals, 1664750094 for in, 1666297068 for goals

#########################################

# TASK_NAME = 'tt_one_fridge_pick'
# TASK_NAME = 'tt_one_fridge_table_pick'
# TASK_NAME = 'tt_one_fridge_table_in'
# TASK_NAME = 'tt_two_fridge_pick'
# TASK_NAME = 'tt_two_fridge_in'
# TASK_NAME = 'tt_two_fridge_goals'
# TASK_NAME = 'tt'

# TASK_NAME = '_examples'
# TASK_NAME = 'zz'
# TASK_NAME = 'ss_two_fridge_pick'
# TASK_NAME = 'ss_two_fridge_in'
# TASK_NAME = 'mm_two_fridge_goals'
# TASK_NAME = 'mm_test'

# TASK_NAME = 'mm_two_fridge_in'
# TASK_NAME = 'mm'

##########################################

TASK_NAME = 'mm_storage'
# TASK_NAME = 'mm_sink'
# TASK_NAME = 'mm_braiser'
# TASK_NAME = '_test'

CASES = None
CASES = ['150', '395', '399', '404', '406', '418', '424', '428', '430', '435', '438', '439', '444', '453', '455', '466', '475', '479', '484', '489', '494', '539', '540', '547', '548', '553', '802', '804', '810', '815', '818', '823', '831', '833', '838', '839', '848', '858', '860', '862']
# CASES = ['1514', '1566', '1612', '1649', '1812', '2053', '2110', '2125', '2456', '2534', '2535', '2576', '2613']
# CASES = ['688', '810', '813', '814', '816', '824', '825', '830', '831', '915', '917', '927', '931', '939', '948', '949', '950', '1099', '1100', '1101', '1102', '1107', '1108', '1109', '1110', '1115', '1116', '1118', '1120', '1125', '1127', '1132', '1143', '1144', '1151', '1152']

if CASES is not None:
    SKIP_IF_SOLVED = False
    SKIP_IF_SOLVED_RECENTLY = False

PARALLEL = GENERATE_SKELETONS # and False
FEASIBILITY_CHECKER = 'None'
## None | oracle | pvt | pvt* | pvt-task | pvt-all | binary | shuffle
if GENERATE_SKELETONS:
    FEASIBILITY_CHECKER = 'oracle'

## =========================================

parser = get_base_parser(task_name=TASK_NAME, parallel=PARALLEL, use_viewer=USE_VIEWER)
parser.add_argument('-d', type=str, default=DIVERSE)
parser.add_argument('-f', type=str, default=FEASIBILITY_CHECKER)
parser.add_argument('-l', '--lock', action='store_true',
                    help='When enabled, locks the PyBullet viewer.', default=LOCK_VIEWER)
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


def check_if_skip(run_dir, **kwargs):
    skip = False
    if GENERATE_SKELETONS:
        file = join(run_dir, f'diverse_plans.json')
        MORE_PLANS = False
        if isfile(file):
            skeletons = [p[0] for p in json.load(open(file, 'r'))['checks']]
            indices = get_indices(run_dir)
            successful_plan = get_successful_plan(run_dir, indices)[0]
            successful_skeleton = get_plan_skeleton(successful_plan, indices)
            if successful_skeleton in skeletons:
                skip = True
            MORE_PLANS = True

    elif GENERATE_MULTIPLE_SOLUTIONS:
        file = join(run_dir, f'multiple_solutions.json')
        if (SKIP_IF_SOLVED or SKIP_IF_SOLVED_RECENTLY) and isfile(file):
            skip = True

    else:
        ori_dir = join(run_dir, RERUN_SUBDIR)
        file = join(ori_dir, f'{PREFIX}plan_rerun_fc={FEASIBILITY_CHECKER}.json')
        if isfile(file):  ## and not '/11' in ori_dir
            failed = False
            if RETRY_IF_FAILED:
                failed = json.load(open(file, 'r'))['plan'] is None

            if not RETRY_IF_FAILED or not failed:
                if SKIP_IF_SOLVED:
                    print('skipping solved problem', run_dir)
                    skip = True
                elif SKIP_IF_SOLVED_RECENTLY:
                    last_modified = os.path.getmtime(file)
                    if last_modified > check_time:
                        print('skipping recently solved problem', run_dir)
                        skip = True
    return skip


def run_one(run_dir, parallel=False, SKIP_IF_SOLVED=SKIP_IF_SOLVED):
    from pybullet_tools.logging import myprint as print
    ori_dir = join(run_dir, RERUN_SUBDIR)
    if not isdir(ori_dir):
        os.mkdir(ori_dir)
    if check_if_skip(run_dir):
        return

    initialize_logs()
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

    pddlstream_problem = pddlstream_from_dir(problem, exp_dir=exp_dir, replace_pddl=True,
                                             collisions=not args.cfree, teleport=False)

    stream_info = world.robot.get_stream_info(partial=False, defer=False)
    _, _, _, stream_map, init, goal = pddlstream_problem
    summarize_facts(init, world=world)
    print_goal(goal)
    print(SEPARATOR)

    fc = get_feasibility_checker(run_dir, mode=FEASIBILITY_CHECKER, diverse=DIVERSE)
    # fc = Shuffler()

    start = time.time()
    collect_dataset = False
    kwargs = dict(fc=fc, lock=args.lock)
    if DIVERSE:
        kwargs.update(dict(
            diverse=DIVERSE,
            downward_time=10,  ## max time to get 100, 10 sec, 30 sec for 300
            evaluation_time=60,  ## on each skeleton
            max_plans=100,  ## number of skeletons
        ))
        if GENERATE_SKELETONS:
            kwargs['evaluation_time'] = -0.5
            if MORE_PLANS:
                kwargs['downward_time'] = 30
                # kwargs['max_plans'] = 300
        if GENERATE_MULTIPLE_SOLUTIONS:
            kwargs['max_solutions'] = 4
            kwargs['collect_dataset'] = True

    cwd = os.getcwd()
    max_time = 8 * 60
    solution = 'failed'
    with timeout(duration=max_time):
        if parallel:
            solution = solve_multiple(pddlstream_problem, stream_info, **kwargs)
            solution, cwd = solution
        else:
            solution = solve_one(pddlstream_problem, stream_info, **kwargs)
    if solution == 'failed':
        reset_simulation()
        shutil.rmtree(exp_dir)

    if GENERATE_MULTIPLE_SOLUTIONS:
        from mamao_tools.data_utils import save_multiple_solutions
        solution, plan_dataset = solution
        file_path = join(run_dir, 'multiple_solutions.json')
        solution = save_multiple_solutions(plan_dataset, run_dir=run_dir, file_path=file_path)

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
            apply_actions(problem, commands, time_step=5e-3, verbose=False)
            input('End?')

        ## maybe generate a multiple_solutions.json file
        if 'fastamp-data' and '/mm_' in run_dir:
            old_plan = get_plan(run_dir)[0]
            indices = get_indices(run_dir)
            # indices.update({eval(k): v for k, v in indices.items()})
            skeleton_kargs = dict(indices=indices, include_movable=True, include_joint=True)
            rerun_dir = join(run_dir, f"rerun_{get_datetime(TO_LISDF=True)}")
            shutil.move(join(run_dir, ori_dir), rerun_dir)
            if len(old_plan) > len(plan):
                new_plan = modify_plan_with_body_map(plan, get_body_map(run_dir, world, inv=True))
                new_plan = [[a.name] + [str(s) for s in a.args] for a in new_plan]
                multiple_solutions = [{
                    'plan': new_plan,
                    'skeleton': get_plan_skeleton(new_plan, **skeleton_kargs),
                    'score': 1.0,
                    'rerun_dir': rerun_dir
                }, {
                    'plan': old_plan,
                    'skeleton': get_plan_skeleton(old_plan, **skeleton_kargs),
                    'score': len(plan)/len(old_plan)
                }]
                solutions_file = join(run_dir, 'multiple_solutions.json')
                json.dump(multiple_solutions, open(solutions_file, 'w'), indent=3)
                print('Saved multiple solutions to', solutions_file)

    # disconnect()
    reset_simulation()
    shutil.rmtree(exp_dir)


def process(index):
    t = int(time.time())
    print('current time', t)
    np.random.seed(t)
    random.seed(t)
    return run_one(str(index), parallel=PARALLEL)


if __name__ == '__main__':
    process_all_tasks(process, args.t, parallel=PARALLEL, cases=CASES)
    # process_all_tasks(clear_all_rerun_results, args.t, parallel=False)

