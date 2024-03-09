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

from pybullet_tools.utils import set_random_seed, disconnect, set_numpy_seed
from pybullet_tools.bullet_utils import get_datetime

from world_builder.builders import sample_world_and_goal, test_feg_kitchen_mini
from world_builder.world_utils import parse_yaml

from data_generator.run_utils import get_config_from_argparse, parallel_processing


DEFAULT_YAML = 'kitchen_full_feg.yaml'
config = get_config_from_argparse(DEFAULT_YAML)


def process(index):
    set_random_seed(index)
    set_numpy_seed(index)
    new_config = copy.deepcopy(config)
    new_config.seed = index
    new_config.data.out_dir = join(EXP_PATH, config.data.out_dir, str(index))
    return sample_world_and_goal(new_config, save_testcase=True, reset_sim=True)


if __name__ == '__main__':
    parallel_processing(process, range(config.n_data), parallel=config.parallel)
