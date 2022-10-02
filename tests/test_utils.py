from os import listdir
from os.path import join, abspath, dirname, basename, isdir, isfile
from tabnanny import verbose
import os
import json
from config import EXP_PATH, MAMAO_DATA_PATH
import numpy as np
import random
import time
import sys
import pickle
import shutil
import argparse

# TASK_NAME = 'tt_one_fridge_pick'
TASK_NAME = 'tt_one_fridge_table_pick'
# TASK_NAME = 'tt_one_fridge_table_in'
# TASK_NAME = 'tt_two_fridge_pick'
# TASK_NAME = 'tt_two_fridge_in'
# TASK_NAME = 'mm_two_fridge_in'


def get_base_parser(task_name=TASK_NAME, parallel=False, use_viewer=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', type=str, default=task_name)
    parser.add_argument('-p', action='store_true', default=parallel)
    parser.add_argument('-v', '--viewer', action='store_true', default=use_viewer,
                        help='When enabled, enables the PyBullet viewer.')
    return parser


def clear_pddlstream_cache():
    if isdir('visualizations'):
        shutil.rmtree('visualizations')
    if isdir('statistics'):
        shutil.rmtree('statistics')
    if isdir('temp'):
        shutil.rmtree('temp')


def copy_dir_for_process(viz_dir, tag=None):
    subdir = basename(viz_dir)
    task_name = basename(viz_dir.replace(f"/{subdir}", ''))

    ## temporarily move the dir to the test_cases folder for asset paths to be found
    test_dir = join(EXP_PATH, f"{task_name}_{subdir}")
    if isdir(test_dir):
        shutil.rmtree(test_dir)
    if not isdir(test_dir):
        shutil.copytree(viz_dir, test_dir)
    if tag is None:
        print(viz_dir, end='\r')
    else:
        print(f'\n\n\n--------------------------\n    {tag} {viz_dir} \n------------------------\n\n\n')

    return test_dir


def get_task_names(task_name):
    if task_name == 'mm':
        task_names = ['mm_one_fridge_pick',
                      'mm_one_fridge_table_pick', 'mm_one_fridge_table_in', 'mm_one_fridge_table_on',
                      'mm_two_fridge_in', 'mm_two_fridge_pick']
    elif task_name == 'tt':
        task_names = ['tt_one_fridge_table_pick', 'tt_one_fridge_table_in',
                      'tt_two_fridge_pick', 'tt_two_fridge_in']  ##
    elif task_name == 'ff':
        task_names = ['ff_one_fridge_table_pick', 'ff_one_fridge_table_in',
                      'ff_two_fridge_in', 'ff_two_fridge_pick']
    elif task_name == 'bb':
        task_names = ['bb_one_fridge_pick',
                      'bb_one_fridge_table_pick', 'bb_one_fridge_table_in', 'bb_one_fridge_table_on',
                      'bb_two_fridge_in', 'bb_two_fridge_pick']
    else:
        task_names = [task_name]
    return task_names


def get_run_dirs(task_name):
    task_names = get_task_names(task_name)
    all_subdirs = []
    for task_name in task_names:
        dataset_dir = join('/home/yang/Documents/fastamp-data/', task_name)
        # organize_dataset(task_name)
        subdirs = listdir(dataset_dir)
        subdirs.sort()
        subdirs = [join(dataset_dir, s) for s in subdirs if isdir(join(dataset_dir, s))]
        all_subdirs += subdirs
    return all_subdirs


def process_all_tasks(process, task_name, parallel=False, cases=None, path=None):
    clear_pddlstream_cache()
    start_time = time.time()

    if path is not None:
        cases = [path]
    elif cases is not None and len(cases) > 0:
        cases = [join(MAMAO_DATA_PATH, task_name, case) for case in cases]
    else:
        cases = get_run_dirs(task_name)

    num_cases = len(cases)

    if parallel:
        import multiprocessing
        from multiprocessing import Pool

        max_cpus = 12
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus')
        with Pool(processes=num_cpus) as pool:
            pool.map(process, cases)

    else:
        for i in range(num_cases):
            process(cases[i])

    print(f'went through {num_cases} run_dirs (parallel={parallel}) in {round(time.time() - start_time, 3)} sec')
