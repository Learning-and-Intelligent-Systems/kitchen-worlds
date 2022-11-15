import os
from os.path import join, isfile, abspath
import sys
from config import ASSET_PATH, EXP_PATH, SCENE_CONFIG_PATH
import time
import tqdm
import pybullet as p
import random
import numpy as np

from pybullet_tools.utils import set_random_seed, disconnect
from pybullet_tools.bullet_utils import get_datetime
from world_builder.builders import create_pybullet_world, test_feg_kitchen_mini, test_kitchen_clean

import argparse

DEFAULT_TEST = test_kitchen_clean
## test_feg_kitchen_mini  ## test_one_fridge | test_feg_pick | test_kitchen_oven | test_exist_omelette
USE_GUI = True
DEFAULT_YAML = 'kitchen_mini_feg.yaml'


def parse_yaml(path=DEFAULT_YAML):
    import yaml
    from pprint import pprint
    from pathlib import Path
    conf = yaml.safe_load(Path(path).read_text())
    print(f'-------------- {abspath(path)} --------------')
    pprint(conf)
    print('------------------------------------\n')
    return conf


def get_parser(use_gui=USE_GUI, config_name=DEFAULT_YAML):
    parser = argparse.ArgumentParser()

    ## -------- simulation related
    parser.add_argument('-v', '--viewer', action='store_true', default=use_gui, help='')
    parser.add_argument('-d', '--drive', action='store_true', help='')
    parser.add_argument('-t', '--time_step', type=float, default=4e-0)
    parser.add_argument('--teleport', action='store_true', help='')
    parser.add_argument('-s', '--seed', type=int, default=None, help='')
    parser.add_argument('-c', '--config_name', type=str, default=config_name)
    parser.add_argument('-cam', '--camera', action='store_true', default=True, help='')
    parser.add_argument('-seg', '--segment', action='store_true', default=False, help='')
    parser.add_argument('-mon', '--monitoring', action='store_true', default=False)

    args = parser.parse_args()
    set_random_seed(args.seed)
    args.config = parse_yaml(join(SCENE_CONFIG_PATH, args.config))
    return args


def main():
    args = get_parser()
    parallel = False
    num_cases = 4
    builder = DEFAULT_TEST
    out_name = f'{builder.__name__}_{get_datetime()}'
    os.mkdir(join(EXP_PATH, out_name))

    def process(index):
        np.random.seed(index)
        random.seed(index)
        out_dir = join(EXP_PATH, out_name, f"{out_name}_{index}")
        return create_pybullet_world(args, builder, out_dir=out_dir, verbose=False,
                                     SAVE_TESTCASE=True, SAVE_RGB=True, RESET=True)

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
    if USE_GUI: disconnect()


if __name__ == '__main__':
    # main()
    parse_yaml(join(SCENE_CONFIG_PATH, DEFAULT_YAML))