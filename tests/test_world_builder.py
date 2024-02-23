import os
from os.path import join, isfile, abspath
import sys
from config import ASSET_PATH, EXP_PATH, SCENE_CONFIG_PATH
import time
import copy
import tqdm
import pybullet as p
import random
import numpy as np
import argparse

from pybullet_tools.utils import set_random_seed, disconnect
from pybullet_tools.bullet_utils import get_datetime
from world_builder.builders import create_pybullet_world, test_feg_kitchen_mini
from world_builder.world_utils import parse_yaml
from test_utils import get_config


DEFAULT_YAML = 'clean_dish_feg.yaml'


def main():
    config = get_config(config_name=DEFAULT_YAML)
    parallel = False
    num_cases = 4

    def process(index):
        np.random.seed(index)
        random.seed(index)
        new_config = copy.deepcopy(config)
        new_config.data.out_dir = join(EXP_PATH, config.data.out_dir, str(index))
        return create_pybullet_world(config, SAVE_TESTCASE=True, RESET=True)

    start_time = time.time()
    if parallel:
        import multiprocessing
        from multiprocessing import Pool

        max_cpus = 14
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus')
        with Pool(processes=num_cpus) as pool:
            for result in tqdm.tqdm(pool.imap_unordered(process, range(num_cases)), total=num_cases):
                pass
            # pool.map(process, range(num_cases))

    else:
        for i in range(num_cases):
            process(i)

    print(f'generated {num_cases} problems (parallel={parallel}) in {round(time.time() - start_time, 3)} sec')
    if config.viewer: disconnect()


if __name__ == '__main__':
    main()
    # parse_yaml(join(SCENE_CONFIG_PATH, DEFAULT_YAML))