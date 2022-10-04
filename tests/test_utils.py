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
    elif task_name == 'ww':
        task_names = ['ww_one_fridge_table_pick', 'ww_one_fridge_table_in',
                      'ww_two_fridge_in', 'ww_two_fridge_pick']
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


def parallel_processing(process, inputs, parallel):
    start_time = time.time()
    num_cases = len(inputs)

    if parallel:
        import multiprocessing
        from multiprocessing import Pool

        max_cpus = 12
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus')
        with Pool(processes=num_cpus) as pool:
            pool.map(process, inputs)

    else:
        for i in range(num_cases):
            process(inputs[i])

    print(f'went through {num_cases} run_dirs (parallel={parallel}) in {round(time.time() - start_time, 3)} sec')


def process_all_tasks(process, task_name, parallel=False, cases=None, path=None):
    clear_pddlstream_cache()

    if path is not None:
        cases = [path]
    elif cases is not None and len(cases) > 0:
        cases = [join(MAMAO_DATA_PATH, task_name, case) for case in cases]
    else:
        cases = get_run_dirs(task_name)

    parallel_processing(process, cases, parallel)


def find_duplicate_worlds(d1, d2):
    from config import MAMAO_DATA_PATH

    def find_duplicate(found):
        dup1 = len(set(found)) != len(found)
        if dup1:
            seen = []
            dup = []
            for f in found:
                if f not in seen:
                    seen.append(f)
                else:
                    dup.append(f)
            print(f'{t} has duplicate {(len(dup))}, {dup}')

    t1 = get_task_names(d1)
    t2 = get_task_names(d2)
    n1 = []
    for t in t1:
        found = open(join(MAMAO_DATA_PATH, f'{t}.txt')).read().splitlines()
        find_duplicate(found)
        n1.extend(found)
    n2 = []
    for t in t2:
        found = open(join(MAMAO_DATA_PATH, f'{t}.txt')).read().splitlines()
        find_duplicate(found)
        n2.extend(found)
    dup1 = len(set(n1)) != len(n1)
    dup2 = len(set(n2)) != len(n2)
    inter = len(list(set(n1) & set(n2))) > 0
    print(f'\nt1: {d1}, t2: {d2}, dup1: {dup1}, dup2: {dup2}, inter: {inter}'
          f'\n----------------------------------------------------------\n')


if __name__ == '__main__':
    find_duplicate_worlds('mm', 'ww')
    find_duplicate_worlds('mm', 'ff')
    find_duplicate_worlds('ww', 'ff')