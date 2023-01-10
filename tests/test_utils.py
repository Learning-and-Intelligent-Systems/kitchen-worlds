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


def copy_dir_for_process(viz_dir, tag=None, verbose=True, print_fn=None):
    if print_fn is None:
        from pybullet_tools.logging import myprint as print_fn
    subdir = basename(viz_dir)
    task_name = basename(viz_dir.replace(f"/{subdir}", ''))

    ## temporarily move the dir to the test_cases folder for asset paths to be found
    test_dir = join(EXP_PATH, f"temp_{task_name}_{subdir}")
    if isdir(test_dir):
        if verbose:
            print_fn('copy_dir_for_process | removing', test_dir)
        shutil.rmtree(test_dir)
    if not isdir(test_dir):
        shutil.copytree(viz_dir, test_dir)
    if verbose:
        if tag is None:
            print_fn(viz_dir, end='\r')
        else:
            print_fn(f'\n\n\n--------------------------\n    {tag} {viz_dir} \n------------------------\n\n\n')

    return test_dir


def get_task_names(task_name):
    if task_name == 'mm':
        task_names = ['mm_one_fridge_pick',
                      'mm_one_fridge_table_pick', 'mm_one_fridge_table_in', 'mm_one_fridge_table_on',
                      'mm_two_fridge_in', 'mm_two_fridge_pick'] ## , 'mm_two_fridge_goals'
    elif task_name == 'tt':
        task_names = ['tt_one_fridge_table_pick', 'tt_one_fridge_table_in',
                      'tt_two_fridge_pick', 'tt_two_fridge_in', 'tt_two_fridge_goals']  ##
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
    elif task_name == 'zz':
        task_names = ['zz_three_fridge', 'ss_two_fridge_pick', 'ss_two_fridge_in']
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


def process_all_tasks(process, task_name, parallel=False, cases=None, path=None, dir=None):
    clear_pddlstream_cache()

    if dir is not None:
        cases = [join(dir, c) for c in listdir(dir)]
        cases = [c for c in cases if isdir(c) and not isfile(join(c, 'gym_replay.gif'))]
    elif path is not None:
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


def mp4_to_gif(mp4_file, frame_folder='output'):
    import cv2
    def convert_mp4_to_jpgs(path):
        video_capture = cv2.VideoCapture(path)
        still_reading, image = video_capture.read()
        frame_count = 0
        while still_reading:
            cv2.imwrite(f"{frame_folder}/frame_{frame_count:03d}.jpg", image)

            # read next image
            still_reading, image = video_capture.read()
            frame_count += 1

    import glob
    from PIL import Image

    def make_gif():
        images = glob.glob(f"{frame_folder}/*.jpg")
        images.sort()
        frames = [Image.open(image) for image in images]
        frame_one = frames[0]
        output_file = mp4_file.replace('.mp4', '.gif')
        frame_one.save(output_file, format="GIF", append_images=frames,
                       save_all=True, duration=50, loop=0)
        return output_file

    convert_mp4_to_jpgs(mp4_file)
    output_file = make_gif()
    print('converted mp4 to', output_file)


def query_yes_no(question, default="no"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
            It must be "yes" (the default), "no" or None (meaning
            an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True, "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = input().lower()
        if default is not None and choice == "":
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' " "(or 'y' or 'n').\n")


def get_body_map(run_dir, world, inv=False):
    body_to_name = json.load(open(join(run_dir, 'planning_config.json'), 'r'))['body_to_name']
    body_to_new = {eval(k): world.name_to_body[v] for k, v in body_to_name.items()}
    if inv:
        return {v: k for k, v in body_to_new.items()}
    return body_to_new


def modify_plan_with_body_map(plan, body_map):
    from pddlstream.language.constants import Action
    new_plan = []
    for action in plan:
        new_args = []
        for a in action.args:
            if a in body_map:
                new_args.append(body_map[a])
            else:
                if hasattr(a, 'body') and a in body_map:
                    a.body = body_map[a]
                new_args.append(a)
        new_plan.append(Action(action.name, new_args))
    return new_plan



if __name__ == '__main__':
    find_duplicate_worlds('mm', 'ww')
    find_duplicate_worlds('mm', 'ff')
    find_duplicate_worlds('ww', 'ff')