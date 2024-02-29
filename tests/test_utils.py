from os import listdir
from os.path import join, abspath, dirname, basename, isdir, isfile
from tabnanny import verbose
import os
import math
import json
from config import EXP_PATH, MAMAO_DATA_PATH, SCENE_CONFIG_PATH
import numpy as np
import random
import time
import sys
import pickle
import shutil
import argparse

from world_builder.world_utils import parse_yaml


def get_config(config_name):
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, default=config_name)
    args = parser.parse_args()
    config = parse_yaml(join(SCENE_CONFIG_PATH, args.config))
    if isinstance(config.seed, str) and config.seed.lower() == 'none':
        config.seed = None
    return config


def get_base_parser(task_name=None, parallel=False, use_viewer=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', type=str, default=task_name)
    parser.add_argument('-p', action='store_true', default=parallel)
    parser.add_argument('-v', '--viewer', action='store_true', default=use_viewer,
                        help='When enabled, enables the PyBullet viewer.')
    return parser


def clear_failed_out_dirs(out_dir):
    exp_dirs = [join(out_dir, f) for f in listdir(out_dir) if isdir(join(out_dir, f))]
    for exp_dir in exp_dirs:
        solution_file = join(exp_dir, 'plan.json')
        if not isfile(solution_file):
            shutil.rmtree(exp_dir)
        else:
            solution = json.load(open(solution_file, 'r'))[0]['plan']
            if solution is None:
                shutil.rmtree(exp_dir)

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
    from pigi_tools.data_utils import load_planning_config
    # data_dir = join('/home/yang/Documents/kitchen-worlds/outputs', data_dir)
    # dirs = [join(data_dir, f) for f in listdir(data_dir) if isdir(join(data_dir, f))]
    task_names = ['mm_storage', 'mm_sink', 'mm_braiser',
                  'mm_sink_to_storage', 'mm_braiser_to_storage']
    num_per_problem = math.ceil(count / len(task_names))
    dirs = []
    for t in task_names:
        task_dir = join(MAMAO_DATA_PATH, t)
        run_dirs = [join(task_dir, f) for f in listdir(task_dir) if isdir(join(task_dir, f))]
        cleaned_dirs = []
        for rr in run_dirs:
            conf = load_planning_config(rr)
            if 'cfree' in conf and isinstance(conf['cfree'], float):
                cleaned_dirs.append(rr)
        dirs.extend(cleaned_dirs)
        # dirs.extend(random.sample(run_dirs, num_per_problem))
    random.shuffle(dirs)
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