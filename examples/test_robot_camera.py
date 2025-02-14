import os
from os.path import join, isfile, abspath
import sys
from config import ASSET_PATH, EXP_PATH, DATA_CONFIG_PATH
import time
import copy
import tqdm
import pybullet as p
import random
import numpy as np
import argparse

from pybullet_tools.utils import set_random_seed, disconnect, set_numpy_seed, unit_pose
from pybullet_tools.bullet_utils import get_datetime
from pybullet_tools.pr2_utils import EYE_FRAME

from world_builder.builders import sample_world_and_goal, test_feg_kitchen_mini
from world_builder.world_utils import parse_yaml
from world_builder.paths import TEMP_PATH

from data_generator.run_utils import get_config_from_argparse, parallel_processing


DEFAULT_YAML = 'kitchen_full_pr2.yaml'
config = get_config_from_argparse(DEFAULT_YAML)


def set_cameras_and_save_images():
    world, goal = sample_world_and_goal(config)
    robot = world.robot
    # robot.img_dir = TEMP_PATH
    for camera_name in ['head', 'right_wrist', 'left_wrist']:
        robot.visualize_image_by_name(camera_name, index=camera_name, img_dir=TEMP_PATH)


if __name__ == '__main__':
    set_cameras_and_save_images()
