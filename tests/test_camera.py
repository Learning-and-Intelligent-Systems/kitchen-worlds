from config import EXP_PATH
from pybullet_tools.utils import quat_from_euler, reset_simulation
from lisdf_tools.lisdf_loader import load_lisdf_pybullet
import json
import shutil
from os import listdir
from os.path import join, isdir
import pybullet as p
from tqdm import tqdm


def render_rgb_image(viz_dir):
    ## need to temporarily move the dir to the test_cases folder for asset paths to be found
    test_dir = join(EXP_PATH, f"{task_name}_{subdir}")
    if not isdir(test_dir):
        shutil.copytree(viz_dir, test_dir)

    world = load_lisdf_pybullet(test_dir, width=720, height=560)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, True)
    camera_pose = json.load(open(join(viz_dir, 'planning_config.json')))["obs_camera_pose"]
    if len(camera_pose) == 6:
        point = camera_pose[:3]
        euler = camera_pose[3:]
        camera_pose = (point, quat_from_euler(euler))
    world.add_camera(camera_pose, viz_dir)
    world.visualize_image(index='scene', rgb=True)

    shutil.rmtree(test_dir)


if __name__ == "__main__":
    dataset_dir = '/home/zhutiany/Documents/mamao-data/one_fridge_pick_pr2_400'
    task_name = dataset_dir[dataset_dir.rfind('/')+1:]
    for subdir in listdir(dataset_dir):
        if not isdir(join(dataset_dir, subdir)): continue
        viz_dir = join(dataset_dir, subdir)
        render_rgb_image(viz_dir)
        reset_simulation()

