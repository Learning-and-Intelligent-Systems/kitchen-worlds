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
from pybullet_tools.utils import apply_alpha, get_camera_matrix, LockRenderer, HideOutput, load_model, TURTLEBOT_URDF, \
    set_all_color, dump_body, draw_base_limits, multiply, Pose, Euler, PI, draw_pose, unit_pose, create_box, TAN, Point, \
    GREEN, create_cylinder, INF, BLACK, WHITE, RGBA, GREY, YELLOW, BLUE, BROWN, RED, stable_z, set_point, set_camera_pose, \
    set_all_static, get_model_info, load_pybullet, remove_body, get_aabb, set_pose, wait_if_gui, get_joint_names, \
    get_min_limit, get_max_limit, set_joint_position, set_joint_position, get_joints, get_joint_info, get_moving_links, \
    get_pose, get_joint_position, enable_gravity, enable_real_time, get_links, set_color, dump_link, draw_link_name, \
    get_link_pose, get_aabb, get_link_name, sample_aabb, aabb_contains_aabb, aabb2d_from_aabb, sample_placement, \
    aabb_overlap, get_links, get_collision_data, get_visual_data, link_from_name, body_collision, get_closest_points, \
    load_pybullet, FLOOR_URDF, pairwise_collision, is_movable, get_bodies, get_aabb_center, draw_aabb
from pybullet_tools.pr2_primitives import get_group_joints, Conf

from world_builder.world import World, State
from world_builder.entities import Object, Region, Environment, Robot, Camera, Floor, Stove,\
    Surface, Moveable, Supporter, Steerable, Door
from world_builder.world_generator import to_lisdf, save_to_test_cases

from world_builder.builders import test_pick, test_exist_omelette, test_kitchen_oven, test_feg_pick, test_one_fridge

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


def create_pybullet_world(args, builder, world_name='test_scene', SAVE_LISDF=False, EXIT=True,
                          SAVE_TESTCASE=False, template_name=None, out_dir=None, verbose=False):
    if template_name is None:
        template_name = builder.__name__

    """ ============== initiate simulator ==================== """

    ## for viewing, not the size of depth image
    connect(use_gui=USE_GUI, shadows=False, width=1980, height=1238)

    # set_camera_pose(camera_point=[2.5, 0., 3.5], target_point=[1., 0, 1.])
    if args.camera:
        enable_preview()
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, False)
    draw_pose(unit_pose(), length=1.)

    """ ============== sample world configuration ==================== """

    world = World(args, time_step=args.time_step)
    floorplan, goal = builder(world, verbose=verbose)

    ## no gravity once simulation starts
    set_all_static()
    if verbose: world.summarize_all_objects()

    """ ============== save world configuration ==================== """

    state = State(world)
    file = None
    if SAVE_LISDF:   ## only lisdf files
        init = state.get_facts(verbose=verbose)
        file = to_lisdf(state.world, init, floorplan=floorplan, world_name=world_name, verbose=verbose)
    if SAVE_TESTCASE and out_dir is not None:
        file = save_to_test_cases(state, goal, template_name, floorplan, out_dir, verbose=verbose, DEPTH_IMAGES=True)

    if EXIT:
        wait_if_gui('exit?')
    reset_simulation()
    return file


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
        return create_pybullet_world(args, builder, out_dir=out_dir, SAVE_TESTCASE=True, EXIT=False, verbose=False)

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