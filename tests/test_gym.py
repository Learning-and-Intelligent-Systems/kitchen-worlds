from os.path import join, isdir
from os import listdir
import shutil
import random
from config import *
from test_utils import copy_dir_for_process


def get_envs_from_task(task_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_pick'):
    ori_dirs = [join(task_dir, f) for f in listdir(task_dir) if isdir(join(task_dir, f))]
    ori_dirs.sort()
    return ori_dirs


def get_sample_envs_for_corl():
    task_scenes = {
        'mm_one_fridge_pick': ['5', '226', '229', '232', '311'],
        'mm_one_fridge_table_in': ['7', '88', '97', '202', '305', '394', '419', '466'],
        'mm_two_fridge_in': ['36', '104', '186', '294', '346', '405', '473', '493', '498', '502'],
        'mm_two_fridge_pick': ['222', '347', '472']
    }
    dirs = []
    for k, v in task_scenes.items():
        for i in v:
            dirs.append(join('/home/yang/Documents/fastamp-data', k, i))
    random.shuffle(dirs)
    return dirs


def test_load_lisdf():
    from isaac_tools.gym_utils import load_lisdf
    ori_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_in/4'
    lisdf_dir = copy_dir_for_process(ori_dir)
    for name, path, scale, is_fixed, pose, positions in load_lisdf(lisdf_dir, robots=True):
        print(name, positions)


def test_load_one():
    from isaac_tools.gym_utils import load_lisdf_isaacgym
    ori_dir = '/home/caelan/Programs/interns/yang/kitchen-worlds/test_cases/tt_one_fridge_pick_2'
    ori_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_in/4'
    lisdf_dir = copy_dir_for_process(ori_dir)
    world = load_lisdf_isaacgym(abspath(lisdf_dir), pause=True)
    shutil.rmtree(lisdf_dir)


def test_load_multiple():
    from isaac_tools.gym_utils import load_envs_isaacgym
    # ori_dirs = get_envs_from_task()
    ori_dirs = get_sample_envs_for_corl()
    lisdf_dirs = [copy_dir_for_process(ori_dir) for ori_dir in ori_dirs]
    world = load_envs_isaacgym(lisdf_dirs, pause=True)
    print('test_load_multiple | to remove', len(lisdf_dirs))
    for lisdf_dir in lisdf_dirs:
        shutil.rmtree(lisdf_dir)


if __name__ == "__main__":
    # test_load_lisdf()
    # test_load_one()
    test_load_multiple()


