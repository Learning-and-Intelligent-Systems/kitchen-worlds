import os
from os.path import join, isfile
import sys
from config import ASSET_PATH, EXP_PATH
import time
import tqdm
import pybullet as p
import random
import numpy as np

from pybullet_tools.utils import set_random_seed, connect, enable_preview, \
    disconnect, draw_pose, set_all_static, wait_if_gui, remove_handles, unit_pose, get_sample_fn, pairwise_collision, \
    set_camera_pose, add_line, get_point, BLACK, get_name, CLIENTS, get_client, link_from_name, \
    get_link_subtree, clone_body, set_all_color, GREEN, BROWN, invert, multiply, set_pose, VideoSaver, reset_simulation
from pybullet_tools.bullet_utils import get_datetime

from world_builder.builders import create_pybullet_world, test_pick, test_exist_omelette, test_kitchen_oven, \
    test_feg_pick, test_one_fridge

import argparse
from datetime import datetime

DEFAULT_TEST = test_one_fridge  ## test_one_fridge | test_feg_pick | test_kitchen_oven | test_exist_omelette
USE_GUI = True


def get_parser():
    parser = argparse.ArgumentParser()

    ## -------- simulation related
    parser.add_argument('-v', '--viewer', action='store_true', help='')
    parser.add_argument('-d', '--drive', action='store_true', help='')
    parser.add_argument('-t', '--time_step', type=float, default=4e-0)
    parser.add_argument('--teleport', action='store_true', help='')
    parser.add_argument('-s', '--seed', type=int, default=None, help='')
    parser.add_argument('-cam', '--camera', action='store_true', default=True, help='')
    parser.add_argument('-seg', '--segment', action='store_true', default=False, help='')
    parser.add_argument('-mon', '--monitoring', action='store_true', default=False)

    args = parser.parse_args()  # TODO: flag to save a video
    set_random_seed(args.seed)
    return args


if __name__ == '__main__':
    args = get_parser()
    parallel = False
    num_cases = 4
    builder = DEFAULT_TEST
    out_dir = f'{builder.__name__}_{get_datetime()}'
    os.makedirs(out_dir, exist_ok=True)

    def process(index):
        np.random.seed(index)
        random.seed(index)
        return create_pybullet_world(args, builder, out_dir=out_dir, SAVE_TESTCASE=True,
                                     EXIT=False, USE_GUI=USE_GUI, verbose=False)

    start_time = time.time()
    if parallel:
        import multiprocessing
        from multiprocessing import Pool

        # def process(index):
        #     np.random.seed(index)
        #     random.seed(index)
        #     return create_pybullet_world(builder, out_dir=out_dir, SAVE_TESTCASE=True, EXIT=False)

        max_cpus = 24
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus')
        with Pool(processes=num_cpus) as pool:
            for result in tqdm.tqdm(pool.imap_unordered(process, range(num_cases)), total=num_cases):
                pass
            # pool.map(process, range(num_cases))

    else:
        for i in range(num_cases):
            process(i)

    print(f'generated {num_cases} problems (parallel={parallel}) in {round(time.time()-start_time, 3)} sec')
    if USE_GUI: disconnect()