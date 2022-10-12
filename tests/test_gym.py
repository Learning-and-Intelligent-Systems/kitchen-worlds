import os.path
import sys
from os.path import join, isdir
from os import listdir

import numpy as np

from config import ASSET_PATH  ## necessaty

from pybullet_tools.utils import pose_from_tform, get_pose, get_joint_name, get_joint_position, get_movable_joints
from utils import load_lisdf, test_is_robot
from test_utils import copy_dir_for_process

"""
# Note to myself on setting up slr_stream $ IsaacGym
1. clone git@gitlab.com:nvidia_srl/caelan/srl_stream.git
2. download isaac gym from https://developer.nvidia.com/isaac-gym/download, 
    a. follow instruction in docs/install.html to install isaacgym 
        `(cd ~/Documents/isaacgym/python; pip install -e .)`
3. add python path to srl_stream (srl_stream/src)
4. `pip install setuptools_scm trimesh`
5. run from terminal. It will hang if ran in pycharm
    `cd tests; python test_gym.py`
"""


def load_one_world(gym_world, ori_dir, offset=None, robots=True, **kwargs):
    lisdf_dir = copy_dir_for_process(ori_dir)
    for name, path, scale, is_fixed, pose, positions in load_lisdf(lisdf_dir, robots=robots, **kwargs):
        if 'veggiegreenpepper' in name or 'meatturkeyleg' in name or 'veggietomato' in name:
            print('!!! skipping', name)
            continue
        is_robot = test_is_robot(name)
        asset = gym_world.simulator.load_asset(
            asset_file=path, root=None, fixed_base=is_fixed or is_robot,  # y_up=is_robot,
            gravity_comp=is_robot, collapse=False, vhacd=False)
        actor = gym_world.create_actor(asset, name=name, scale=scale)
        pose = pose_from_tform(pose)
        if offset is not None:
            pose = (pose[0] + offset, pose[1])
        gym_world.set_pose(actor, pose)
        if positions is not None and 'pr2' not in name:
            joint_names = gym_world.get_joint_names(actor)
            joint_positions = gym_world.get_joint_positions(actor)
            joint_positions = {joint_names[i]: joint_positions[i] for i in range(len(joint_names))}
            joint_positions.update(positions)
            gym_world.set_joint_positions(actor, list(joint_positions.values()))
        gym_world.simulator.update_viewer()


def load_lisdf_isaacgym(ori_dir, robots=True, pause=False,
                        camera_width=2560, camera_height=1600, **kwargs):
    sys.path.append('/home/yang/Documents/playground/srl_stream/src')
    from srl_stream.gym_world import create_single_world, default_arguments

    # TODO: Segmentation fault - possibly cylinders & mimic joints
    gym_world = create_single_world(args=default_arguments(use_gpu=False), spacing=5.)

    point_from = (8.5, 2.5, 3)
    point_to = (0, 2.5, 0)
    gym_world.set_viewer_target(point_from, target=point_to)

    load_one_world(gym_world, ori_dir, robots=robots, **kwargs)

    img_file = os.path.join(ori_dir, 'gym_scene.png')
    camera = gym_world.create_camera(width=camera_width, height=camera_height, fov=60)
    gym_world.set_camera_target(camera, point_from, point_to)
    gym_world.save_image(camera, image_type='rgb', filename=img_file)

    if pause:
        gym_world.wait_if_gui()
    return gym_world


def update_gym_world(gym_world, pb_world, pause=False, verbose=False):
    for actor in gym_world.get_actors():
        name = gym_world.get_actor_name(actor)
        body = pb_world.name_to_body[name] # TODO: lookup if pb_world is None
        pose = get_pose(body)
        gym_world.set_pose(actor, pose)
        if verbose:
            print(f'Name: {name} | Actor: {actor} | Body: {body}')

        joint_state = {}
        for joint in get_movable_joints(body):
            joint_name = get_joint_name(body, joint)
            position = get_joint_position(body, joint)
            joint_state[joint_name] = position
            if verbose:
                print(f'Joint: {joint_name} | Position: {position}')
        joints = gym_world.get_joint_names(actor)
        positions = list(map(joint_state.get, joints))
        gym_world.set_joint_positions(actor, positions)
    gym_world.simulator.update_viewer()
    if pause:
        gym_world.wait_if_gui()


def load_task_isaacgym(task_dir, robots=True, pause=False,
                        camera_width=2560, camera_height=1600, **kwargs):
    sys.path.append('/home/yang/Documents/playground/srl_stream/src')
    from srl_stream.gym_world import create_single_world, default_arguments

    gym_world = create_single_world(args=default_arguments(use_gpu=False), spacing=5.)

    point_from = (34, 12, 10)
    point_to = (0, 12, 0)
    gym_world.set_viewer_target(point_from, target=point_to)

    num_rows = 5
    num_cols = 5
    world_size = 6
    num_worlds = num_rows * num_cols - 1  ## 24 is the limit

    ori_dirs = [join(task_dir, f) for f in listdir(task_dir) if isdir(join(task_dir, f))]
    ori_dirs.sort()
    for i in range(num_worlds):
        ori_dir = ori_dirs[i]
        print('------------------\n', ori_dir, '\n\n')
        row = i // num_rows
        col = i % num_rows
        print(row, col)
        offset = np.asarray([row * world_size, col * world_size, 0])
        load_one_world(gym_world, ori_dir, offset=offset, robots=robots, **kwargs)

    # task_name = os.path.basename(task_dir)
    # img_file = os.path.join(task_dir, '..', f'{task_name}_gym_scene.png')
    # camera = gym_world.create_camera(width=camera_width, height=camera_height, fov=60)
    # gym_world.set_camera_target(camera, point_from, point_to)
    # gym_world.save_image(camera, image_type='rgb', filename=img_file)

    if pause:
        gym_world.wait_if_gui()
    return gym_world


if __name__ == "__main__":
    # lisdf_dir = '/home/caelan/Programs/interns/yang/kitchen-worlds/test_cases/tt_one_fridge_pick_2'
    # lisdf_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_in/4'
    # world = load_lisdf_isaacgym(os.path.abspath(lisdf_dir), pause=True)

    lisdf_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_pick'
    world = load_task_isaacgym(os.path.abspath(lisdf_dir), pause=True)

    # for name, path, scale, is_fixed, pose, positions in load_lisdf(lisdf_dir, robots=True):
    #     print(name, positions)

