#!/usr/bin/env python

from __future__ import print_function
from ipaddress import v4_int_to_packed
import os
import json
import math
import pickle
import shutil
from os import listdir
from os.path import join, abspath, dirname, isdir, isfile, basename
from tabnanny import verbose
from config import EXP_PATH, MAMAO_DATA_PATH
import numpy as np
import random
import time
import sys
from PIL import Image

from pybullet_tools.bullet_utils import query_yes_no, get_datetime
from pybullet_tools.utils import reset_simulation, VideoSaver, wait_unlocked, draw_aabb, get_aabb
from lisdf_tools.lisdf_loader import load_lisdf_pybullet, pddlstream_from_dir
from lisdf_tools.lisdf_planning import pddl_to_init_goal, Problem
from lisdf_tools.image_utils import make_composed_image_multiple_episodes, images_to_gif
from isaac_tools.gym_utils import save_gym_run
from world_builder.actions import apply_actions

from mamao_tools.data_utils import get_plan, get_body_map, get_multiple_solutions, \
    add_to_planning_config, load_planning_config, exist_instance, \
    check_unrealistic_placement_z

from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser, \
    get_sample_envs_for_rss

USE_GYM = False
SAVE_COMPOSED_JPG = False
SAVE_GIF = True
SAVE_JPG = True or SAVE_COMPOSED_JPG or SAVE_GIF
PREVIEW_SCENE = False

CHECK_COLLISIONS = False
CFREE_RANGE = 0.1
VISUALIZE_COLLISIONS = False

SAVE_MP4 = False
STEP_BY_STEP = False
AUTO_PLAY = True
EVALUATE_QUALITY = False
PARALLEL = SAVE_JPG and not PREVIEW_SCENE and False  ## and not CHECK_COLLISIONS

SKIP_IF_PROCESSED_RECENTLY = False
CHECK_TIME = 1674417578

GIVEN_PATH = None
# GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/one_fridge_pick_pr2/one_fridge_pick_pr2_1004_01:29_1'
# GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/0104_094417_original_1'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_storage/528'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_sink/10'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_braiser/563'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_sink/1998/' + 'rerun/diverse_commands_rerun_fc=None.pkl'
# GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/230115_115113_original_0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/mm_sink_to_storage/43'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/tt_sink/1'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/mm_braiser_to_storage/1'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/_gmm/902'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_storage/45' + '/rerun_230120_000551/commands.pkl'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'tt_braiser/0' + '/rerun/diverse_commands_rerun_fc=None.pkl'

GIVEN_DIR = None
# GIVEN_DIR = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen_100'
# GIVEN_DIR ='/home/yang/Documents/fastamp-data-rss/' + 'mm_storage_long'

#####################################################################

# TASK_NAME = 'one_fridge_pick_pr2'

# TASK_NAME = 'mm_one_fridge_table_in'
# TASK_NAME = 'mm_one_fridge_table_on'
# TASK_NAME = 'mm_one_fridge_table_pick'
# TASK_NAME = 'mm_two_fridge_pick'
# TASK_NAME = 'mm_two_fridge_in'

# TASK_NAME = 'tt_one_fridge_pick'
# TASK_NAME = 'tt_one_fridge_table_in'
# TASK_NAME = 'tt_two_fridge_pick'
# TASK_NAME = 'tt_two_fridge_in'

# TASK_NAME = 'ff_one_fridge_table_pick'
# TASK_NAME = 'ff_two_fridge_pick'

# TASK_NAME = '_examples'
# TASK_NAME = 'elsewhere'
# TASK_NAME = 'discarded'

#####################################################################

# TASK_NAME = 'mm'
# TASK_NAME = 'mm_storage'
# TASK_NAME = 'mm_sink'
# TASK_NAME = 'mm_braiser'
# TASK_NAME = 'mm_sink_to_storage'
# TASK_NAME = 'mm_braiser_to_storage'

# TASK_NAME = 'tt'
TASK_NAME = 'tt_storage'

CASES = None
# CASES = ['45','340', '387', '467']  ##
# CASES = get_sample_envs_for_rss(task_name=TASK_NAME, count=None)

if GIVEN_PATH:
    VISUALIZE_COLLISIONS = True
    PARALLEL = False
if GIVEN_PATH is not None and 'rerun' in GIVEN_PATH:
    SAVE_JPG = False
    SAVE_COMPOSED_JPG = False
    SAVE_GIF = False

parser = get_base_parser(task_name=TASK_NAME, parallel=PARALLEL, use_viewer=True)
args = parser.parse_args()

#####################################


def get_pkl_run(run_dir, verbose=True):
    pkl_file = 'commands.pkl'
    if run_dir.endswith('.pkl'):
        pkl_file = basename(run_dir)
        run_dir = run_dir[:-len(pkl_file) - 1]
        rerun_dir = basename(run_dir)
        run_dir = run_dir[:-len(rerun_dir) - 1]
        pkl_file = join(rerun_dir, pkl_file)

    exp_dir = copy_dir_for_process(run_dir, tag='replaying', verbose=verbose)
    if basename(pkl_file) != 'commands.pkl':
        plan_json = join(run_dir, pkl_file).replace('commands', 'plan').replace('.pkl', '.json')
        plan = get_plan(run_dir, plan_json=plan_json)
    else:
        ## if there are reran versions
        plan = get_plan(run_dir, skip_multiple_plans=True)
    commands = pickle.load(open(join(exp_dir, pkl_file), "rb"))
    return exp_dir, run_dir, commands, plan


def check_if_exist_rerun(run_dir, world, commands, plan):
    indices = world.get_indices()
    multiple_solutions = get_multiple_solutions(run_dir, indices=indices, commands_too=True)
    if len(multiple_solutions) > 1:
        plan, path = multiple_solutions[0]
        commands_file = join(path, 'commands.pkl')
        if not isfile(commands_file):
            return None
        commands = pickle.load(open(commands_file, "rb"))
    return commands, plan


def swap_microwave(run_dir, verbose=False):
    exp_dir = copy_dir_for_process(run_dir, tag='swapping microwave', verbose=verbose)
    world = load_lisdf_pybullet(exp_dir, use_gui=not USE_GYM, verbose=False)


def run_one(run_dir_ori, task_name=TASK_NAME, save_mp4=SAVE_MP4, width=1440, height=1120, fx=600,
            camera_point=(8.5, 2.5, 3), target_point=(0, 2.5, 0)):

    verbose = not SAVE_JPG

    exp_dir, run_dir, commands, plan = get_pkl_run(run_dir_ori, verbose=verbose)

    # load_lisdf_synthesizer(exp_dir)
    world = load_lisdf_pybullet(exp_dir, use_gui=not USE_GYM, width=width, height=height,
                                verbose=False) ## , clear_for_topdown_camera=True
    # wait_unlocked()
    problem = Problem(world)
    if verbose:
        world.summarize_all_objects()
    body_map = get_body_map(run_dir, world)
    result = check_if_exist_rerun(run_dir, world, commands, plan)
    if result is None:
        print(run_dir_ori, 'does not have rerun commands.pkl')
        reset_simulation()
        shutil.rmtree(exp_dir)
        return
    commands, plan = result

    ## -----------------------------------------------------------
    # artichoke = world.safely_get_body_from_name('veggiepotato')
    # draw_aabb(get_aabb(artichoke))
    # microwave = world.safely_get_body_from_name('microwave')
    # draw_aabb(get_aabb(microwave))
    # wait_unlocked()
    ## -----------------------------------------------------------

    ## save the initial scene image in pybullet
    if not CHECK_COLLISIONS and SAVE_JPG:
        viz_dir = join(run_dir_ori, 'zoomin')
        world.add_camera(viz_dir, width=width//4, height=height//4, fx=fx//2, img_dir=viz_dir,
                         **world.camera_kwargs)
        world.visualize_image(index='initial', rgb=True)
        if SAVE_COMPOSED_JPG or SAVE_GIF:
            world.add_camera(viz_dir, width=width//2, height=height//2, fx=fx//2, img_dir=viz_dir,
                             camera_point=(6, 4, 2), target_point=(0, 4, 1))
            world.make_transparent(world.robot.body, transparency=0)

    if USE_GYM:
        from isaac_tools.gym_utils import load_lisdf_isaacgym, record_actions_in_gym, set_camera_target_body
        gym_world = load_lisdf_isaacgym(abspath(exp_dir), camera_width=1280, camera_height=800,
                                        camera_point=camera_point, target_point=target_point)
        set_camera_target_body(gym_world, run_dir)
        img_dir = join(exp_dir, 'gym_images')
        gif_name = 'gym_replay.gif'
        os.mkdir(img_dir)
        gif_name = record_actions_in_gym(problem, commands, gym_world, img_dir=img_dir, body_map=body_map,
                                         gif_name=gif_name, time_step=0, verbose=False, plan=plan)
        # gym_world.wait_if_gui()
        shutil.copy(join(exp_dir, gif_name), join(run_dir, gif_name))
        print('moved gif to {}'.format(join(run_dir, gif_name)))

        new_file = join('gym_images', basename(exp_dir).replace('temp_', '')+'.gif')
        shutil.move(join(exp_dir, gif_name), new_file)
        del(gym_world.simulator)

    elif save_mp4:
        video_path = join(run_dir, 'replay.mp4')
        with VideoSaver(video_path):
            apply_actions(problem, commands, time_step=0.025, verbose=False, plan=plan)
        print('saved to', abspath(video_path))

    else:

        run_name = basename(run_dir)
        if not AUTO_PLAY:
            wait_unlocked()
            # wait_if_gui(f'start replay {run_name}?')
        answer = True
        if not AUTO_PLAY:
            answer = query_yes_no(f"start replay {run_name}?", default='yes')
        if answer:
            time_step = 2e-5 if SAVE_JPG else 0.02
            time_step = None if STEP_BY_STEP else time_step
            results = apply_actions(problem, commands, time_step=time_step, verbose=verbose, plan=plan,
                                    body_map=body_map, SAVE_COMPOSED_JPG=SAVE_COMPOSED_JPG, SAVE_GIF=SAVE_GIF,
                                    CHECK_COLLISIONS=CHECK_COLLISIONS, cfree_range=CFREE_RANGE,
                                    VISUALIZE_COLLISIONS=VISUALIZE_COLLISIONS)

        if CHECK_COLLISIONS:
            new_data = {'cfree': results} if results else {'cfree': CFREE_RANGE}
            ## another way to be bad data is if the place pose is not realistic
            if new_data['cfree'] == CFREE_RANGE:
                result = check_unrealistic_placement_z(world, run_dir)
                if result:
                    new_data['cfree'] = result
            add_to_planning_config(run_dir, new_data)
            if results:
                print('COLLIDED', run_dir)

        else:

            if SAVE_COMPOSED_JPG:
                episodes = results
                h, w, _ = episodes[0][0][0][0].shape
                crop = (0, h//3-h//30, w, 2*h//3-h//30)
                make_composed_image_multiple_episodes(episodes, join(world.img_dir, 'composed.jpg'),
                                                      verbose=verbose, crop=crop)

            if SAVE_GIF:
                episodes = results
                h, w, _ = episodes[0].shape
                crop = (0, h//3-h//30, w, 2*h//3-h//30)
                gif_name = 'replay.gif'
                images_to_gif(world.img_dir, gif_name, episodes, crop=crop)

            if SAVE_COMPOSED_JPG or SAVE_GIF:
                world.camera = world.cameras[0]

            if SAVE_JPG:
                world.visualize_image(index='final', rgb=True, **world.camera_kwargs)

        if EVALUATE_QUALITY:
            answer = query_yes_no(f"delete this run {run_name}?", default='no')
            if answer:
                new_dir = join(MAMAO_DATA_PATH, 'impossible', f"{task_name}_{run_name}")
                shutil.move(run_dir, new_dir)
                print(f"moved {run_dir} to {new_dir}")

        # wait_if_gui('replay next run?')

    # disconnect()
    reset_simulation()
    shutil.rmtree(exp_dir)


def merge_all_wconfs(all_wconfs):
    longest_command = max([len(wconf) for wconf in all_wconfs])
    whole_wconfs = []
    for t in range(longest_command):
        whole_wconf = {}
        for num in range(len(all_wconfs)):
            if t < len(all_wconfs[num]):
                whole_wconf.update(all_wconfs[num][t])
        whole_wconfs.append(whole_wconf)
    return whole_wconfs


def replay_all_in_gym(width=1440, height=1120, num_rows=5, num_cols=5, world_size=(6, 6), verbose=False,
                      frame_gap=6, debug=False, loading_effect=False, save_gif=True, save_mp4=False,
                      camera_motion=None):
    from test_utils import get_dirs_camera
    from isaac_tools.gym_utils import load_envs_isaacgym, record_actions_in_gym, \
        update_gym_world_by_wconf, images_to_gif, images_to_mp4
    from tqdm import tqdm

    img_dir = join('gym_images')
    gif_name = 'gym_replay_batch_gym.gif'
    # if isdir(img_dir):
    #     shutil.rmtree(img_dir)
    # os.mkdir(img_dir)

    data_dir = 'test_full_kitchen_100' if loading_effect else 'test_full_kitchen_sink'
    ori_dirs, camera_point_begin, camera_point_final, target_point = get_dirs_camera(
        num_rows, num_cols, world_size, data_dir=data_dir, camera_motion=camera_motion)
    lisdf_dirs = [copy_dir_for_process(ori_dir, verbose=verbose) for ori_dir in ori_dirs]
    num_worlds = min([len(lisdf_dirs), num_rows * num_cols])

    ## translate commands into world_confs
    all_wconfs = []

    if loading_effect:
        ### load all gym_worlds and return all wconfs
        gym_world, offsets, all_wconfs = load_envs_isaacgym(lisdf_dirs, num_rows=num_rows, num_cols=num_cols,
                                                            world_size=world_size, loading_effect=True, verbose=verbose,
                                                            camera_point=camera_point_begin, target_point=target_point)
    else:
        for i in tqdm(range(num_worlds)):
            exp_dir, run_dir, commands, plan = get_pkl_run(lisdf_dirs[i], verbose=verbose)
            world = load_lisdf_pybullet(exp_dir, use_gui=not USE_GYM or debug,
                                        width=width, height=height, verbose=False)
            body_map = get_body_map(run_dir, world)
            problem = Problem(world)
            wconfs = record_actions_in_gym(problem, commands, plan=plan, return_wconf=True,
                                           world_index=i, body_map=body_map)
            all_wconfs.append(wconfs)
            reset_simulation()
            if debug:
                wait_unlocked()

        ## load all scenes in gym
        gym_world, offsets = load_envs_isaacgym(lisdf_dirs, num_rows=num_rows, num_cols=num_cols, world_size=world_size,
                                                camera_point=camera_point_begin, target_point=target_point, verbose=verbose)

    all_wconfs = merge_all_wconfs(all_wconfs)
    print(f'\n\nrendering all {len(all_wconfs)} frames')

    ## update all scenes for each frame
    filenames = []
    for i in tqdm(range(len(all_wconfs))):
        if camera_point_final != camera_point_begin:
            if isinstance(camera_point_final, tuple) and isinstance(camera_point_final, tuple):
                camera_point = tuple([camera_point_begin[j] + (camera_point_final[j] - camera_point_begin[j]) * i / len(all_wconfs)
                                      for j in range(3)])
            else:
                ## rotate camera around point begin by radius of camera_point_final
                dx = camera_point_final * math.sin(2 * math.pi * i / len(all_wconfs))
                dy = camera_point_final * math.cos(2 * math.pi * i / len(all_wconfs))
                offset = [dx, dy, 0]
                camera_point = tuple([camera_point_begin[j] + offset[j] for j in range(3)])
            gym_world.set_camera_target(gym_world.cameras[0], camera_point, target_point)
        update_gym_world_by_wconf(gym_world, all_wconfs[i], offsets=offsets)
        if i % frame_gap == 0:
            img_file = gym_world.get_rgba_image(gym_world.cameras[0])
            filenames.append(img_file)

    save_gym_run(img_dir, gif_name, filenames, save_gif=save_gif, save_mp4=save_mp4)


def generated_recentely(file):
    result = False
    if isfile(file):
        if SKIP_IF_PROCESSED_RECENTLY:
            last_modified = os.path.getmtime(file)
            if last_modified > CHECK_TIME:
                result = True
        else:
            result = True
    return result


def case_filter(run_dir_ori):
    """ whether to process this run """
    if CASES is not None or GIVEN_PATH is not None:
        return True

    result = True
    if CHECK_COLLISIONS:
        return True
        # config, mod_time = load_planning_config(run_dir_ori, return_mod_time=True)
        if 'cfree' in config: ##  and config['cfree']:
            if mod_time > 1674746024:
                return False
            if isinstance(config['cfree'], float):  ## check if unrealistic
                return True
            # if isinstance(config['cfree'], str) and exist_instance(run_dir_ori, '100015') \
            #         and 'braiser' in config['cfree']:
            #     return True
            result = False
        return result

    if SAVE_JPG:
        viz_dir = join(run_dir_ori, 'zoomin')
        if isdir(viz_dir):
            enough = len([a for a in listdir(viz_dir) if '.png' in a]) > 1
            file = join(viz_dir, 'rgb_image_final.png')
            result = not generated_recentely(file) or not enough
    if result:
        return result

    if SAVE_GIF:
        file = join(run_dir_ori, 'replay.gif')
        result = not generated_recentely(file)
    multiple_solutions_file = join(run_dir_ori, 'multiple_solutions.json')

    # if not result and isfile(multiple_solutions_file):
    #     plans = json.load(open(multiple_solutions_file, 'r'))
    #     if len(plans) == 2 and 'rerun_dir' in plans[0]:
    #         print('dont skip multiple solutions', run_dir_ori)
    #         result = True
    return result


if __name__ == '__main__':
    process = run_one  ## run_one | swap_microwave
    # case_filter = None
    process_all_tasks(process, args.t, parallel=args.p, cases=CASES, path=GIVEN_PATH, dir=GIVEN_DIR,
                      case_filter=case_filter)

    # replay_all_in_gym(num_rows=14, num_cols=14, world_size=(6, 6), save_gif=True)

    ## record 1 : 250+ worlds
    # replay_all_in_gym(num_rows=32, num_cols=8, world_size=(4, 8), loading_effect=True,
    #                   frame_gap=1, save_mp4=True, save_gif=False, verbose=False, camera_motion='zoom')

    ## record 2 : robot execution
    # replay_all_in_gym(num_rows=8, num_cols=3, world_size=(4, 8), loading_effect=False,
    #                   frame_gap=2, save_mp4=True, save_gif=False, verbose=False, camera_motion='splotlight')
