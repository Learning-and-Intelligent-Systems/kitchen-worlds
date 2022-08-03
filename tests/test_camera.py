import os

import PIL.Image

from config import EXP_PATH
from pybullet_tools.utils import quat_from_euler, reset_simulation
from lisdf_tools.lisdf_loader import load_lisdf_pybullet
import json
import shutil
from os import listdir
from os.path import join, isdir
import pybullet as p
from tqdm import tqdm
from lisdf_tools.lisdf_loader import get_depth_images


def get_camera_pose(viz_dir):
    camera_pose = json.load(open(join(viz_dir, 'planning_config.json')))["obs_camera_pose"]
    if len(camera_pose) == 6:
        point = camera_pose[:3]
        euler = camera_pose[3:]
        camera_pose = (point, quat_from_euler(euler))
    return camera_pose


def render_rgb_image(test_dir, viz_dir, camera_pose):
    world = load_lisdf_pybullet(test_dir, width=720, height=560)
    world.add_camera(camera_pose, viz_dir)
    world.visualize_image(index='scene', rgb=True)


def render_segmented_rgb_images(test_dir, viz_dir, camera_pose, robot=False):
    get_depth_images(test_dir, camera_pose=camera_pose, rgb=True, robot=robot,
                     img_dir=join(viz_dir, 'rgb_images'))


def render_segmented_rgbd_images(test_dir, viz_dir, camera_pose, robot=False):
    get_depth_images(test_dir, camera_pose=camera_pose, rgbd=True, robot=robot,
                     img_dir=join(viz_dir))


def render_masked_rgb_images(viz_dir):
    import numpy as np

    new_key = 'masked_rgb'
    out_dir = join(viz_dir, f"{new_key}s")
    if not isdir(out_dir): os.mkdir(out_dir)

    # crops = { 'rgb': (50, 50, 562, 413), 'depth': (49, 30, 446, 398) }

    def load_rgbd(rgb_image_name):
        depth_image_name = rgb_image_name.replace('rgb', 'depth')
        rgb_img = np.asarray(PIL.Image.open(rgb_image_name).convert('RGB')) ## .crop(crops['rgb']))
        depth_img = np.asarray(PIL.Image.open(depth_image_name)) ## .crop(crops['rgb']))
        return rgb_img, depth_img

    def expand_mask(mask):
        y = np.expand_dims(mask, axis=2)
        return np.concatenate((y, y, y), axis=2)

    def mask_from_rgb_image(arr):
        background = make_image_background(arr)
        i = np.where(np.sum(np.abs(arr - background), axis=2) > 5)
        mask = np.zeros_like(arr[:, :, 0])
        mask[i] = 1
        return expand_mask(mask)

    def mask_from_depth_image(arr):
        i = np.where(arr != 0)
        mask = np.zeros_like(arr)
        mask[i] = 1
        return mask

    def get_visibility_mask(obj, scene, mask):
        visibility_mask = np.zeros_like(obj)
        i = np.where(obj*mask == scene*mask)
        visibility_mask[i] = 1
        return expand_mask(visibility_mask*mask)

    def make_image_background(old_arr):
        new_arr = np.ones_like(old_arr)
        new_arr[:, :, 0] = 178
        new_arr[:, :, 1] = 178
        new_arr[:, :, 2] = 204
        return new_arr

    ## ----- step 1: read the instance rgb and depth images
    rgb_dir = join(viz_dir, 'rgbs')
    rgb_files = [f for f in listdir(rgb_dir)]
    rgb_scene = [join(rgb_dir, f) for f in rgb_files if 'scene' in f][0]
    rgb_scene_img, depth_scene_img = load_rgbd(rgb_scene)
    shutil.copy(rgb_scene, rgb_scene.replace('rgb', new_key))

    ## ----- step 2: render the masked out version of the scene rgb if the depth values are the same
    rgb_files.sort()
    for f in rgb_files:
        if 'scene' in f: continue
        rgb_obj = join(rgb_dir, f)
        rgb_obj_img, depth_obj_img = load_rgbd(rgb_obj)

        rgb_mask = mask_from_rgb_image(rgb_obj_img)
        depth_mask = mask_from_depth_image(depth_obj_img)

        visibility_mask = get_visibility_mask(depth_obj_img, depth_scene_img, depth_mask)

        background = make_image_background(rgb_obj_img)  ## background color [178, 178, 204] in png image
        foreground = np.zeros_like(background) + rgb_scene_img * rgb_mask * visibility_mask
        background[np.where(foreground!=0)] = 0
        background += foreground
        im = PIL.Image.fromarray(background)
        im.save(rgb_obj.replace('rgb', new_key))


if __name__ == "__main__":
    dataset_dir = '/home/zhutiany/Documents/mamao-data/one_fridge_pick_pr2'
    # dataset_dir = '/Users/z/Documents/simulators/PyBullet/kitchen-worlds/outputs/one_fridge_pick_pr2'
    task_name = dataset_dir[dataset_dir.rfind('/')+1:]
    subdirs = listdir(dataset_dir)
    subdirs.sort()
    for subdir in subdirs:
        if not isdir(join(dataset_dir, subdir)): continue
        viz_dir = join(dataset_dir, subdir)

        if not isdir(join(viz_dir, 'rgbs')):
            camera_pose = get_camera_pose(viz_dir)

            ## need to temporarily move the dir to the test_cases folder for asset paths to be found
            test_dir = join(EXP_PATH, f"{task_name}_{subdir}")
            if not isdir(test_dir):
                shutil.copytree(viz_dir, test_dir)

            ## ------------- visualization function to test -------------------
            # render_rgb_image(test_dir, viz_dir, camera_pose)
            # if not isdir(join(viz_dir, 'rgb_images')):
            #     render_segmented_rgb_images(test_dir, viz_dir, camera_pose, robot=True)

            ## without robot
            render_segmented_rgbd_images(test_dir, viz_dir, camera_pose)

            shutil.rmtree(test_dir)
            reset_simulation()

        print(viz_dir)
        if not isdir(join(viz_dir, 'masked_rgbs')):
            render_masked_rgb_images(viz_dir)
        ## ----------------------------------------------------------------
