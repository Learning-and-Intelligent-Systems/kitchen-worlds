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
    if not verbose:
        print_fn = print
    elif print_fn is None:
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

    if not verbose:
        print_fn(viz_dir)
    else:
        if tag is None:
            print_fn(viz_dir, end='\r')
        elif verbose:
            print_fn(f'\n\n\n--------------------------\n    {tag} {viz_dir} \n------------------------\n\n\n')

    return test_dir


def get_task_names(task_name):
    # if task_name == 'mm':
    #     task_names = ['mm_one_fridge_pick',
    #                   'mm_one_fridge_table_pick', 'mm_one_fridge_table_in', 'mm_one_fridge_table_on',
    #                   'mm_two_fridge_in', 'mm_two_fridge_pick'] ## , 'mm_two_fridge_goals'
    # elif task_name == 'tt':
    #     task_names = ['tt_one_fridge_table_pick', 'tt_one_fridge_table_in',
    #                   'tt_two_fridge_pick', 'tt_two_fridge_in', 'tt_two_fridge_goals']  ##
    # elif task_name == 'ff':
    #     task_names = ['ff_one_fridge_table_pick', 'ff_one_fridge_table_in',
    #                   'ff_two_fridge_in', 'ff_two_fridge_pick']
    # elif task_name == 'ww':
    #     task_names = ['ww_one_fridge_table_pick', 'ww_one_fridge_table_in',
    #                   'ww_two_fridge_in', 'ww_two_fridge_pick']
    # elif task_name == 'bb':
    #     task_names = ['bb_one_fridge_pick',
    #                   'bb_one_fridge_table_pick', 'bb_one_fridge_table_in', 'bb_one_fridge_table_on',
    #                   'bb_two_fridge_in', 'bb_two_fridge_pick']
    # elif task_name == 'zz':
    #     task_names = ['zz_three_fridge', 'ss_two_fridge_pick', 'ss_two_fridge_in']

    mm_task_names = ['mm_storage', 'mm_sink', 'mm_braiser',
                     'mm_sink_to_storage', 'mm_braiser_to_storage']
    if task_name == 'mm':
        task_names = mm_task_names
    elif task_name == 'tt':
        task_names = [t.replace('mm_', 'tt_') for t in mm_task_names]
    else:
        task_names = [task_name]
    return task_names


def get_run_dirs(task_name):
    task_names = get_task_names(task_name)
    all_subdirs = []
    for task_name in task_names:
        dataset_dir = join('/home/yang/Documents/fastamp-data-rss/', task_name)
        # organize_dataset(task_name)
        if not isdir(dataset_dir):
            print('get_run_dirs | no directory', dataset_dir)
            continue
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

        max_cpus = 11
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus')
        with Pool(processes=num_cpus) as pool:
            pool.map(process, inputs)

    else:
        for i in range(num_cases):
            process(inputs[i])

    print(f'went through {num_cases} run_dirs (parallel={parallel}) in {round(time.time() - start_time, 3)} sec')


def process_all_tasks(process, task_name, parallel=False, cases=None, path=None,
                      dir=None, case_filter=None, return_dirs=False):
    clear_pddlstream_cache()

    if dir is not None:
        cases = [join(dir, c) for c in listdir(dir)]
        cases = [c for c in cases if isdir(c) and not isfile(join(c, 'gym_replay.gif'))]
        # cases = cases[:1]
    elif path is not None:
        cases = [path]
    elif cases is not None and len(cases) > 0:
        cases = [join(MAMAO_DATA_PATH, task_name, case) for case in cases]
    else:
        cases = get_run_dirs(task_name)

    if len(cases) > 1:
        cases = sorted(cases, key=lambda x: eval(x.split('/')[-1]))

    if case_filter is not None:
        cases = [c for c in cases if case_filter(c)]

    if return_dirs:
        return cases

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


##########################################################################


def get_dirs_camera(num_rows=5, num_cols=5, world_size=(6, 6), data_dir=None, camera_motion=None):
    ## load all dirs
    ori_dirs = []
    if world_size == (4, 8):
        ori_dirs = get_sample_envs_full_kitchen(num_rows * num_cols, data_dir=data_dir)

    if num_rows == 1 and num_cols == 1:

        if world_size == (4, 8):
            camera_point = (6, 4, 6)
            camera_target = (0, 4, 0)
            camera_point_begin = (6, 4, 6)
            camera_point_final = (12, 4, 12)

    elif num_rows == 2 and num_cols == 1:

        if world_size == (4, 8):
            camera_point = (24, 4, 10)
            camera_target = (0, 4, 0)
            camera_point_begin = (16, 4, 6)
            camera_point_final = (24, 4, 10)

    elif num_rows == 4 and num_cols == 4:

        if world_size == (4, 8):
            camera_point = (32, 8, 10)
            camera_target = (0, 24, 0)
            camera_point_begin = (16, 16, 2)
            camera_point_final = (32, 16, 10)

    elif num_rows == 5 and num_cols == 5:
        ori_dirs = get_sample_envs_for_corl()

        if world_size == (6, 6):
            mid = num_rows * 6 // 2
            camera_point = (45, 15, 10)
            camera_target = (0, 15, 0)
            camera_point_final = (mid + 35, 15, 14)
            camera_point_begin = (mid + 9, 15, 4)
            camera_target = (mid, 15, 0)

    elif num_rows == 8 and num_cols == 3:

        if world_size == (4, 8):
            if camera_motion == 'zoom':
                camera_target = (4*6, 8*2, 0)
                camera_point_begin = (4*8-1.5, 8*3-4, 2)
                camera_point_final = (4*8+3, 8*3, 4)
            elif camera_motion == 'spotlight':
                camera_target = (4*4, 8*1.5, 1)
                camera_point_begin = (4*4, 8*1.5, 2.5)
                camera_point_final = 3.5

    elif num_rows == 10 and num_cols == 10:

        if world_size == (4, 8):
            ## bad
            camera_target = (0, 48, 0)
            camera_point_begin = (16, 40, 6)
            camera_point_final = (40, 32, 16)

    elif num_rows == 14 and num_cols == 14:
        ori_dirs = get_sample_envs_200()

        if world_size == (6, 6):
            y = 42 + 3
            camera_point_begin = (67, y, 3)
            camera_point_final = (102, y, 24)
            camera_target = (62, y, 0)

    elif num_rows == 16 and num_cols == 16:

        if world_size == (4, 8):
            camera_target = (5*11, 8*4-4, 0)
            camera_point_begin = (5*13-1, 8*4-6, 2)
            camera_point_final = (5*16, 8*2, 12)

    elif num_rows == 32 and num_cols == 8:

        if world_size == (4, 8):
            camera_target = (5*(11+16), 8*4-4, 0)
            camera_point_begin = (5*(12+16), 8*4-6, 2)
            camera_point_final = (5*(16+16), 8*2, 12)

    return ori_dirs, camera_point_begin, camera_point_final, camera_target


##################################################################################


def get_envs_from_task(task_dir = join(MAMAO_DATA_PATH, 'tt_two_fridge_pick')):
    ori_dirs = [join(task_dir, f) for f in listdir(task_dir) if isdir(join(task_dir, f))]
    ori_dirs.sort()
    return ori_dirs


def get_sample_envs_200():
    dirs = get_sample_envs_for_corl()
    new_dirs = []
    for subdir in get_task_names('mm'):
        if subdir == 'mm_one_fridge_pick':
            continue
        path = join(MAMAO_DATA_PATH, subdir)
        names = [join(path, f) for f in listdir(path) if isdir(join(path, f))]
        new_dirs.extend(random.choices(names, k=40))
    new_dirs = [n for n in new_dirs if n not in dirs]
    random.shuffle(new_dirs)
    dirs.extend(new_dirs[:200-len(dirs)])
    return dirs


def get_sample_envs_for_corl():
    task_scenes = {
        'mm_one_fridge_pick': ['5', '226', '229', '232', '288', '295', '311'],
        'mm_one_fridge_table_on': ['48', '69', '168', '248', '296', '325', '313'],
        'mm_one_fridge_table_in': ['7', '88', '97', '202', '305', '394', '419', '466'],
        'mm_two_fridge_in': ['36', '104', '186', '294', '346', '405', '473', '493', '498', '502'],
        'mm_two_fridge_pick': ['222', '347', '472']
    }
    dirs = []
    for k, v in task_scenes.items():
        for i in v:
            dirs.append(join(MAMAO_DATA_PATH, k, i))
    random.shuffle(dirs)
    return dirs


def get_sample_envs_for_rss(task_name=None, count=None):
    task_scenes = {
        'mm_storage': ['1085', '1097', '1098', '1105', '1110', '1182', '1218', '1232', '1234', '1253', '1257'],
        'mm_sink': ['1514', '1566', '1612', '1649', '1812', '2053', '2110', '2125', '2456', '2534', '2535', '2576', '2613'],
        'mm_braiser': ['688', '810', '813', '814', '816', '824', '825', '830', '831', '915', '917', '927', '931', '939', '948', '949', '950', '1099', '1100', '1101', '1102', '1107', '1108', '1109', '1110', '1115', '1116', '1118', '1120', '1125', '1127', '1132', '1143', '1144', '1151', '1152'],
        'mm_storage_long': ['0', '2', '3', '9', '19', '26', '31', '40', '42', '44', '47'],
    }
    dirs = []
    if task_name is not None:
        task_scenes = {task_name: task_scenes[task_name]}
    for k, v in task_scenes.items():
        for i in v:
            dirs.append(join(MAMAO_DATA_PATH, k, i))
    return make_count(dirs, count)


def get_sample_envs_full_kitchen(count=4, data_dir='test_full_kitchen_100'):
    data_dir = join('/home/yang/Documents/kitchen-worlds/outputs', data_dir)
    dirs = [join(data_dir, f) for f in listdir(data_dir) if isdir(join(data_dir, f))]
    return make_count(dirs, count)


def make_count(dirs, count=None):
    random.shuffle(dirs)
    if count is None:
        return dirs
    if count <= len(dirs):
        dirs = dirs[:count]
    else:
        dirs = random.choices(dirs, k=count)
    return dirs


if __name__ == '__main__':
    find_duplicate_worlds('mm', 'ww')
    find_duplicate_worlds('mm', 'ff')
    find_duplicate_worlds('ww', 'ff')