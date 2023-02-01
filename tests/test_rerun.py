#!/usr/bin/env python

from __future__ import print_function
import os
import json
import pickle
import shutil
import copy
import sys
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
    SEPARATOR, get_aabb, wait_for_duration, safe_remove, ensure_dir, reset_simulation, timeout, wait_unlocked
from pybullet_tools.bullet_utils import summarize_facts, print_goal, nice, get_datetime, \
    initialize_logs
from pybullet_tools.pr2_agent import get_stream_info, post_process, move_cost_fn, \
    get_stream_map, solve_multiple, solve_one

from pddlstream.language.constants import Equal, AND, print_solution

from lisdf_tools.lisdf_loader import load_lisdf_pybullet, pddlstream_from_dir
from lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem

from world_builder.actions import apply_actions

from mamao_tools.data_utils import get_instance_info, exist_instance, get_indices, \
    get_plan_skeleton, get_successful_plan, get_feasibility_checker, get_plan, get_body_map, \
    modify_plan_with_body_map, add_to_planning_config, load_planning_config, \
    add_objects_and_facts

from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser

## special modes
GENERATE_MULTIPLE_SOLUTIONS = False
GENERATE_SKELETONS = False
GENERATE_NEW_PROBLEM = False
GENERATE_NEW_LABELS = False
USE_LARGE_WORLD = True

USE_VIEWER = False
LOCK_VIEWER = True
DIVERSE = True
PREFIX = 'diverse_' if DIVERSE else ''
RERUN_SUBDIR = 'rerun_2'

SKIP_IF_SOLVED = True and not GENERATE_SKELETONS
SKIP_IF_SOLVED_RECENTLY = True and not GENERATE_SKELETONS
RETRY_IF_FAILED = True
check_time = 1675220260  ## for 12 sec of FD

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

# TASK_NAME = 'mm_storage'
# TASK_NAME = 'mm_sink'
# TASK_NAME = 'mm_braiser'
# TASK_NAME = 'mm_braiser_to_storage'
# TASK_NAME = '_test'

TASK_NAME = 'tt_storage'
# TASK_NAME = 'tt_sink'
TASK_NAME = 'tt_braiser'
# TASK_NAME = 'tt_storage_to_storage'
# TASK_NAME = 'tt_sink_to_storage'
TASK_NAME = 'tt_braiser_to_storage'

# TASK_NAME = 'hh_braiser'

# TASK_NAME = 'hh_storage'
# TASK_NAME = 'hh_braiser'

evaluation_time = {
    'tt_storage': 60,
    'tt_sink': 20,
    'tt_braiser': 60,
    'tt_sink_to_storage': 30,
    'tt_braiser_to_storage': 60,
}
evaluation_time.update({n.replace('tt', 'mm'): v for n, v in evaluation_time.items()})
evaluation_time.update({n.replace('tt', 'hh'): v for n, v in evaluation_time.items()})
evaluation_time = evaluation_time[TASK_NAME]

downward_time = 3
if 'braiser_to_storage' in TASK_NAME:
    downward_time = 120

CASES = None  ##
# CASES = ['0']
# CASES = ['45', '340', '387', '467'] ## mm_storage
# CASES = ['150', '395', '399', '404', '406', '418', '424', '428', '430', '435', '438', '439', '444', '453', '455', '466', '475', '479', '484', '489', '494', '539', '540', '547', '548', '553', '802', '804', '810', '815', '818', '823', '831', '833', '838', '839', '848', '858', '860', '862']
# CASES = ['1514', '1566', '1612', '1649', '1812', '2053', '2110', '2125', '2456', '2534', '2535', '2576', '2613']
# CASES = ['688', '810', '813', '814', '816', '824', '825', '830', '831', '915', '917', '927', '931', '939', '948', '949', '950', '1099', '1100', '1101', '1102', '1107', '1108', '1109', '1110', '1115', '1116', '1118', '1120', '1125', '1127', '1132', '1143', '1144', '1151', '1152']

if CASES is not None:
    SKIP_IF_SOLVED = False
    SKIP_IF_SOLVED_RECENTLY = False

PARALLEL = GENERATE_SKELETONS and False
FEASIBILITY_CHECKER = 'oracle'
## None | oracle | pvt | pvt* | pvt-task | pvt-all | binary | shuffle | heuristic
if GENERATE_SKELETONS:
    FEASIBILITY_CHECKER = 'oracle'
if GENERATE_NEW_PROBLEM:
    GENERATE_NEW_LABELS = False
    USE_LARGE_WORLD = False
if GENERATE_NEW_LABELS:
    FEASIBILITY_CHECKER = 'larger_world'
    GENERATE_NEW_PROBLEM = False
    downward_time = 5

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
    run_num = eval(run_dir.split('/')[-1])
    # return skip
    if GENERATE_NEW_PROBLEM:
        return False
        file = join(run_dir, f'problem_larger.pddl')
        return isfile(file)

    elif GENERATE_NEW_LABELS:
        return False
        file = join(run_dir, f'diverse_plans_larger.json')
        return isfile(file)

    elif GENERATE_SKELETONS:
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
        # if run_num < 35:
        #     return True
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
    #######################################################
    # if 'tt_' in run_dir:
    #     diverse_larger = join(run_dir, 'diverse_plans_larger.json')
    #     if isfile(diverse_larger):
    #         os.remove(diverse_larger)
    #######################################################
    # if isdir(ori_dir):
    #     shutil.rmtree(ori_dir)
    # return
    #######################################################
    # if isdir(ori_dir):
    #     rem_files = [f for f in listdir(ori_dir) if f'={FEASIBILITY_CHECKER}' in f]
    #     for f in rem_files:
    #         os.remove(join(ori_dir, f))
    # return
    #######################################################
    # if isdir(ori_dir):
    #     des_dir = join(run_dir, RERUN_SUBDIR+'_old')
    #     if not isdir(des_dir):
    #         os.mkdir(des_dir)
    #     rem_files = [f for f in listdir(ori_dir) if f'={FEASIBILITY_CHECKER}' in f]
    #     for f in rem_files:
    #         shutil.move(join(ori_dir, f), join(des_dir, f))
    # return
    #######################################################
    if not isdir(ori_dir):
        os.mkdir(ori_dir)
    if check_if_skip(run_dir):
        return

    larger_world = USE_LARGE_WORLD or GENERATE_NEW_LABELS

    initialize_logs()
    exp_dir = copy_dir_for_process(run_dir, tag='rerunning')

    if False:
        from isaac_tools.urdf_utils import load_lisdf_synthesizer
        scene = load_lisdf_synthesizer(exp_dir)

    world = load_lisdf_pybullet(exp_dir, verbose=False, use_gui=args.viewer,
                                larger_world=larger_world) ## , width=720, height=560

    if not GENERATE_NEW_PROBLEM:
        inv_body_map = get_body_map(run_dir, world, inv=True)
        pc_file = join(ori_dir, 'planning_config.json')
        if not isfile(pc_file):
            with open(join(ori_dir, 'planning_config.json'), 'w') as f:
                json.dump({'inv_body_map': {str(k): v for k, v in inv_body_map.items()}}, f, indent=3)

    if isfile(join(ori_dir, 'diverse_runlog_fc=None.log')):
        shutil.move(join(ori_dir, 'diverse_runlog_fc=None.log'),
                    join(ori_dir, 'diverse_runlog_fc=None.json'))
    if isfile(join(ori_dir, 'diverse_runlog_fc=None.pkl')):
        shutil.move(join(ori_dir, 'diverse_runlog_fc=None.pkl'),
                    join(ori_dir, 'diverse_runlog_fc=None.json'))

    # reset_simulation()
    # shutil.rmtree(exp_dir)
    # return

    saver = WorldSaver()
    problem = Problem(world)

    if False:
        from isaac_tools.urdf_utils import load_lisdf_nvisii
        scene = load_lisdf_nvisii(exp_dir)

    ## because there can be a gap in body indexing due to reachability checking created gripper
    pddlstream_problem = pddlstream_from_dir(problem, exp_dir=exp_dir, replace_pddl=True,
                                             collisions=not args.cfree, teleport=False,
                                             larger_world=larger_world)
    _, _, _, stream_map, init, goal = pddlstream_problem
    world.summarize_facts(init)
    print_goal(goal)

    ######################################################
    if GENERATE_NEW_PROBLEM:
        from world_builder.world_generator import generate_problem_pddl

        out_path = join(run_dir, 'problem_larger.pddl')

        ## add new objects and facts according to key
        added_obj, added_init = add_objects_and_facts(world, init, run_dir)

        ## generate a new problem
        generate_problem_pddl(world, init, goal, out_path=out_path,
                              added_obj=added_obj, added_init=added_init)
        body_to_name = load_planning_config(run_dir)['body_to_name']
        added_body_to_name = {
            str(world.name_to_body[k]): k for k in list(body_to_name.values())+added_obj
        }
        add_to_planning_config(run_dir, {'body_to_name_new': added_body_to_name})

        reset_simulation()
        shutil.rmtree(exp_dir)
        return

    ######################################################

    stream_info = world.robot.get_stream_info(partial=False, defer=False)
    print(SEPARATOR)

    ######################################################

    if FEASIBILITY_CHECKER == 'heuristic':
        fc = get_feasibility_checker([copy.deepcopy(problem), goal, init], mode='heuristic')
    else:
        fc = get_feasibility_checker(run_dir, mode=FEASIBILITY_CHECKER, diverse=DIVERSE, world=world)
    # fc = Shuffler()

    start = time.time()
    collect_dataset = False
    kwargs = dict(fc=fc, lock=args.lock)
    if DIVERSE:
        kwargs.update(dict(
            diverse=DIVERSE,
            downward_time=downward_time,  ## max time to get 100, 10 sec, 30 sec for 300
            evaluation_time=evaluation_time,  ## on each skeleton
            max_plans=100,  ## number of skeletons
            visualize=True,
        ))
        # if FEASIBILITY_CHECKER == 'larger' and '_braiser' in run_dir:
        #     kwargs['downward_time'] = 6
        #     kwargs['max_plans'] = 200

        if GENERATE_SKELETONS or GENERATE_NEW_LABELS:
            kwargs['evaluation_time'] = -0.5
            # if MORE_PLANS:
            #     kwargs['downward_time'] = 30
            #     # kwargs['max_plans'] = 300
        if GENERATE_MULTIPLE_SOLUTIONS:
            kwargs['max_solutions'] = 4
            kwargs['collect_dataset'] = True

    cwd = os.getcwd()
    max_time = 6 * 60 + downward_time
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
        return

    if GENERATE_MULTIPLE_SOLUTIONS:
        from mamao_tools.data_utils import save_multiple_solutions
        solution, plan_dataset = solution
        file_path = join(run_dir, 'multiple_solutions.json')
        solution = save_multiple_solutions(plan_dataset, run_dir=run_dir, file_path=file_path)

    ## just to get all diverse plans as labels
    if GENERATE_SKELETONS or GENERATE_NEW_LABELS:
        # ori_dir = join(run_dir, 'rerun_1')  ## join(DATABASE_DIR, run_dir)
        # if isdir(ori_dir) and len(listdir(ori_dir)) == 0:
        #     shutil.rmtree(ori_dir)

        file_name = f'diverse_plans_larger.json' if GENERATE_NEW_LABELS else 'diverse_plans.json'
        fc.dump_log(join(run_dir, file_name), plans_only=True)
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
    commands_file = join(ori_dir, f'{PREFIX}commands_rerun_fc={FEASIBILITY_CHECKER}.pkl')

    if plan is not None:
        print(SEPARATOR)
        with LockRenderer(lock=True):
            commands = post_process(problem, plan)
            print('Commands:', commands)
            problem.remove_gripper()
            saver.restore()
        with open(commands_file, 'wb') as f:
            pickle.dump(commands, f)
        if has_gui():
            saver.restore()
            input('Begin?')
            apply_actions(problem, commands, time_step=5e-3, verbose=False)
            input('End?')

        ## maybe generate a multiple_solutions.json file
        if 'fastamp-data-rss/' in run_dir:
            old_plan = get_plan(run_dir)[0][0]
            indices = get_indices(run_dir)
            # indices.update({eval(k): v for k, v in indices.items()})
            skeleton_kargs = dict(indices=indices, include_movable=True, include_joint=True)
            if 'fastamp-data-rss/mm_' in run_dir:
                rerun_dir = join(run_dir, f"rerun_{get_datetime(TO_LISDF=True)}")
                shutil.move(join(run_dir, ori_dir), rerun_dir)
                commands_name = 'commands.pkl'
                log_name = 'log.json'
            else:
                rerun_dir = ori_dir
                commands_name = f'{PREFIX}commands_rerun_fc={FEASIBILITY_CHECKER}.pkl'
                log_name = f'{PREFIX}runlog_fc={FEASIBILITY_CHECKER}.json'

            inv_body_map = get_body_map(run_dir, world, inv=True)
            new_plan = modify_plan_with_body_map(plan, inv_body_map)
            with open(join(rerun_dir, commands_name), 'wb') as f:
                pickle.dump(post_process(problem, new_plan), f)

            shutil.move(join('visualizations', 'log.json'), join(rerun_dir, log_name))

            if 'fastamp-data-rss/mm_' in run_dir and len(old_plan) > len(plan):
                with open(join(rerun_dir, 'planning_config.json'), 'w') as f:
                    json.dump({'body_map': {str(k): v for k, v in inv_body_map.items()}}, f, indent=3)

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

