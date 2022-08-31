import copy
import os

import PIL.Image
import numpy as np
import argparse

from config import EXP_PATH
from pybullet_tools.utils import quat_from_euler, reset_simulation, remove_body, AABB, \
    get_aabb_extent, get_aabb_center, get_joint_name, get_link_name, euler_from_quat, \
    set_color, apply_alpha, YELLOW, WHITE
from pybullet_tools.bullet_utils import get_segmask, get_door_links, nice, \
    get_partnet_doors

from mamao_tools.data_utils import get_indices
from lisdf_tools.lisdf_loader import load_lisdf_pybullet
import json
import shutil
from os import listdir
from os.path import join, isdir, isfile, dirname, getmtime, basename
import time
import pybullet as p
from tqdm import tqdm
from lisdf_tools.lisdf_loader import get_depth_images
from mamao_tools.utils import organize_dataset

from utils import load_lisdf_synthesizer

N_PX = 224
NEW_KEY = 'meraki'
ACCEPTED_KEYS = [NEW_KEY, 'crop_fix', 'rgb']
DEFAULT_TASK = 'tt_one_fridge_pick'
# DEFAULT_TASK = 'fault'


parser = argparse.ArgumentParser()
parser.add_argument('-p', action='store_true', default=True)
parser.add_argument('-t', type=str, default=DEFAULT_TASK)  ## 'one_fridge_pick_pr2_tmp'
args = parser.parse_args()


def get_camera_pose(viz_dir):
    camera_pose = json.load(open(join(viz_dir, 'planning_config.json')))["obs_camera_pose"]
    if len(camera_pose) == 6:
        point = camera_pose[:3]
        euler = camera_pose[3:]
        camera_pose = (point, quat_from_euler(euler))
    return camera_pose


def render_transparent_doors(test_dir, viz_dir, camera_pose):
    world = load_lisdf_pybullet(test_dir, width=720, height=560)

    paths = {}
    for m in world.lisdf.models:
        if m.name in ['minifridge', 'cabinet']:
            path = m.uri.replace('../../', '').replace('/mobility.urdf', '')
            paths[m.name] = path

    count = 0
    bodies = copy.deepcopy(world.body_to_name)
    for b, name in bodies.items():
        if name in['minifridge', 'cabinet']:
            doors = world.add_joints_by_keyword(name)
            for _, d in doors:
                for l in get_door_links(b, d):
                    set_color(b, link=l, color=apply_alpha(WHITE, alpha=0.2))
                    count += 1
    print(f'changed {count} doors to transparent')
    world.add_camera(camera_pose, viz_dir)
    world.visualize_image(index='trans', rgb=True)


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


def fix_planning_config(viz_dir):
    config_file = join(viz_dir, 'planning_config.json')
    config = json.load(open(config_file, 'r'))
    if 'body_to_name' in config:
        body_to_name = config['body_to_name']
        new_body_to_name = {}
        changed = False
        for k, v in body_to_name.items():
            k = eval(k)
            if isinstance(k, tuple) and not ('link' in v or 'joint' in v):
                name = body_to_name[str(k[0])] + '::'
                if len(k) == 2:
                    name += get_joint_name(k[0], k[-1])
                elif len(k) == 3:
                    name += get_link_name(k[0], k[-1])
                v = name
                changed = True
            new_body_to_name[str(k)] = v
        if changed:
            config['body_to_name'] = new_body_to_name
            tmp_config_file = join(viz_dir, 'planning_config_tmp.json')
            shutil.move(config_file, tmp_config_file)
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=3)


def render_segmentation_mask(test_dir, viz_dir, camera_pose,
                             width=1280, height=960, fx=800, crop=False):
    world = load_lisdf_pybullet(test_dir, width=width, height=height, verbose=False)
    remove_body(world.robot.body)
    # width = 1960
    # height = 1470
    # fx = 800
    if crop:
        world.add_camera(camera_pose, viz_dir, width=width, height=height, fx=fx)
    else:
        world.add_camera(camera_pose, viz_dir, width=width, height=height, fx=fx)
    imgs = world.camera.get_image(segment=True, segment_links=True)
    rgb = imgs.rgbPixels[:, :, :3]

    ## a fix for previous wrong lisdf names in planning_config[name_to_body]
    fix_planning_config(viz_dir)

    new_key = 'seg_image' if not crop else 'crop_image'
    rgb_dir = join(viz_dir, f"{new_key}s")
    os.makedirs(rgb_dir, exist_ok=True)

    im = PIL.Image.fromarray(rgb)
    im.save(join(rgb_dir, f'{new_key}_scene.png'))
    im_name = new_key+"_[{index}]_{name}.png"

    seg = imgs.segmentationMaskBuffer
    # seg = imgs.segmentationMaskBuffer[:, :, 0].astype('int32')
    unique = get_segmask(seg)
    indices = get_indices(viz_dir)
    for k, v in indices.items():
        file_name = join(rgb_dir, im_name.format(index=str(k), name=v))
        if isfile(file_name): continue
        k = eval(k)
        keys = []
        if isinstance(k, int): ##  and (k, 0) in unique
            keys = [u for u in unique if u[0] == k]
            # print('viz_dir', viz_dir, k, keys)
        elif isinstance(k, tuple) and len(k) == 3:
            keys = [(k[0], k[2])]
        elif isinstance(k, tuple) and len(k) == 2:
            keys = [(k[0], l) for l in get_door_links(k[0], k[1])]
        # else:
        #     print('theres nothing to do with this key:', k)

        mask = np.zeros_like(rgb[:, :, 0])
        background = make_image_background(rgb)
        for k in keys:
            if k in unique:
                c, r = zip(*unique[k])
                mask[(np.asarray(c), np.asarray(r))] = 1
            # else:
            #     print('key not found', k)
        foreground = rgb * expand_mask(mask)
        background[np.where(mask!= 0)] = 0
        new_image = foreground + background

        im = PIL.Image.fromarray(new_image)
        if crop:
            bb = get_mask_bb(mask)
            # if bb is not None:
            #     draw_bb(new_image, bb)
            im = crop_image(im, bb, width, height)

        # im.show()
        im.save(file_name)
    #     print(v)
    # print()


def draw_bb(im, bb):
    from PIL import ImageOps
    im2 = np.array(ImageOps.grayscale(im))
    for j in range(bb.lower[0], bb.upper[0]+1):
        for i in [bb.lower[1], bb.upper[1]]:
            im2[i, j] = 255
    for i in range(bb.lower[1], bb.upper[1]+1):
        for j in [bb.lower[0], bb.upper[0]]:
            im2[i, j] = 255
    im.show()
    PIL.Image.fromarray(im2).show()


def crop_image(im, bb, width, height):
    if bb is None:
        # crop the center of the blank image
        left = int((width - N_PX) / 2)
        top = int((height - N_PX) / 2)
        right = left + N_PX
        bottom = top + N_PX
        cp = (left, top, right, bottom)
        im = im.crop(cp)
        return im

    # draw_bb(im, bb)
    need_resizing = False
    size = N_PX
    padding = 30
    dx, dy = get_aabb_extent(bb)
    cx, cy = get_aabb_center(bb)
    dmax = max(dx, dy)
    if dmax > N_PX:
        dmax += padding * 2
        if dmax > height:
            dmax = height
            cy = height / 2
        need_resizing = True
        size = dmax
    left = max(0, int(cx - size / 2))
    top = max(0, int(cy - size / 2))
    right = left + size
    bottom = top + size
    if right > width:
        right = width
        left = width - size
    if bottom > height:
        bottom = height
        top = height - size
    cp = (left, top, right, bottom)

    im = im.crop(cp)
    if need_resizing:
        im = im.resize((N_PX, N_PX))
    return im


def get_mask_bb(mask):
    if np.all(mask == 0):
        return None
    col = np.max(mask, axis=0)  ## 1280
    row = np.max(mask, axis=1)  ## 960
    col = np.where(col == 1)[0]
    row = np.where(row == 1)[0]
    return AABB(lower=(col[0], row[0]), upper=(col[-1], row[-1]))


def expand_mask(mask):
    y = np.expand_dims(mask, axis=2)
    return np.concatenate((y, y, y), axis=2)


def make_image_background(old_arr):
    new_arr = np.ones_like(old_arr)
    new_arr[:, :, 0] = 178
    new_arr[:, :, 1] = 178
    new_arr[:, :, 2] = 204
    return new_arr


# def render_masked_rgb_images(viz_dir):
#     import numpy as np
#
#     new_key = 'masked_rgb'
#     out_dir = join(viz_dir, f"{new_key}s")
#     if not isdir(out_dir): os.mkdir(out_dir)
#
#     # crops = { 'rgb': (50, 50, 562, 413), 'depth': (49, 30, 446, 398) }
#
#     def load_rgbd(rgb_image_name):
#         depth_image_name = rgb_image_name.replace('rgb', 'depth')
#         rgb_img = np.asarray(PIL.Image.open(rgb_image_name).convert('RGB')) ## .crop(crops['rgb']))
#         depth_img = np.asarray(PIL.Image.open(depth_image_name)) ## .crop(crops['rgb']))
#         return rgb_img, depth_img
#
#     def mask_from_rgb_image(arr):
#         background = make_image_background(arr)
#         i = np.where(np.sum(np.abs(arr - background), axis=2) > 5)
#         mask = np.zeros_like(arr[:, :, 0])
#         mask[i] = 1
#         return expand_mask(mask)
#
#     def mask_from_depth_image(arr):
#         i = np.where(arr != 0)
#         mask = np.zeros_like(arr)
#         mask[i] = 1
#         return mask
#
#     def get_visibility_mask(obj, scene, mask):
#         visibility_mask = np.zeros_like(obj)
#         i = np.where(obj*mask == scene*mask)
#         visibility_mask[i] = 1
#         return expand_mask(visibility_mask*mask)
#
#     ## ----- step 1: read the instance rgb and depth images
#     rgb_dir = join(viz_dir, 'rgbs')
#     rgb_files = [f for f in listdir(rgb_dir)]
#     rgb_scene = [join(rgb_dir, f) for f in rgb_files if 'scene' in f][0]
#     rgb_scene_img, depth_scene_img = load_rgbd(rgb_scene)
#     shutil.copy(rgb_scene, rgb_scene.replace('rgb', new_key))
#
#     ## ----- step 2: render the masked out version of the scene rgb if the depth values are the same
#     rgb_files.sort()
#     for f in rgb_files:
#         if 'scene' in f: continue
#         rgb_obj = join(rgb_dir, f)
#         rgb_obj_img, depth_obj_img = load_rgbd(rgb_obj)
#
#         rgb_mask = mask_from_rgb_image(rgb_obj_img)
#         depth_mask = mask_from_depth_image(depth_obj_img)
#
#         visibility_mask = get_visibility_mask(depth_obj_img, depth_scene_img, depth_mask)
#
#         background = make_image_background(rgb_obj_img)  ## background color [178, 178, 204] in png image
#         foreground = np.zeros_like(background) + rgb_scene_img * rgb_mask * visibility_mask
#         background[np.where(foreground!=0)] = 0
#         background += foreground
#         im = PIL.Image.fromarray(background)
#         im.save(rgb_obj.replace('rgb', new_key))


def add_key(viz_dir):
    config_file = join(viz_dir, 'planning_config.json')
    config = json.load(open(config_file, 'r'))
    if 'version_key' not in config or config['version_key'] != NEW_KEY:
        config['version_key'] = NEW_KEY
        tmp_config_file = join(viz_dir, 'planning_config_tmp.json')
        if isfile(tmp_config_file):
            os.remove(tmp_config_file)
        shutil.move(config_file, tmp_config_file)
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=3)


def check_key_same(viz_dir):
    config_file = join(viz_dir, 'planning_config.json')
    config = json.load(open(config_file, 'r'))
    if 'version_key' not in config:
        crop_dir = join(viz_dir, 'crop_images')
        if isdir(crop_dir):
            imgs = [join(crop_dir, f) for f in listdir(crop_dir) if 'png' in f]
            if len(imgs) > 0:
                image_time = getmtime(imgs[0])
                now = time.time()
                since_generated = now - image_time
                print('found recently generated images')
                return since_generated < 6000
            return False
        return False
    return config['version_key'] in ACCEPTED_KEYS


def process(viz_dir):
    # if not isdir(join(dataset_dir, subdir)): return
    # viz_dir = join(dataset_dir, subdir)
    subdir = basename(viz_dir)

    ## need to temporarily move the dir to the test_cases folder for asset paths to be found
    test_dir = join(EXP_PATH, f"{task_name}_{subdir}")
    if isdir(test_dir):
        shutil.rmtree(test_dir)
    if not isdir(test_dir):
        shutil.copytree(viz_dir, test_dir)
    print(viz_dir, end='\r')

    # load_lisdf_synthesizer(test_dir)

    constraint_dir = join(viz_dir, 'constraint_networks')
    stream_dir = join(viz_dir, 'stream_plans')
    if isdir(constraint_dir) and len(listdir(constraint_dir)) == 0:
        shutil.rmtree(constraint_dir)
    if isdir(stream_dir) and len(listdir(stream_dir)) == 0:
        shutil.rmtree(stream_dir)

    if isdir(join(viz_dir, 'rgbs')):
        shutil.rmtree(join(viz_dir, 'rgbs'))
    if isdir(join(viz_dir, 'masked_rgbs')):
        shutil.rmtree(join(viz_dir, 'masked_rgbs'))
    seg_dir = join(viz_dir, 'seg_images')
    rgb_dir = join(viz_dir, 'rgb_images')
    crop_dir = join(viz_dir, 'crop_images')
    tmp_file = join(viz_dir, 'planning_config_tmp.json')

    if isfile(tmp_file):
        os.remove(tmp_file)

    redo = False
    camera_pose = get_camera_pose(viz_dir)
    (x, y, z), quat = camera_pose
    (r, p, w) = euler_from_quat(quat)
    if x < 6.5:
        x = np.random.normal(7, 0.2)
        # redo = True
    camera_pose = (x, y, z + 1), quat_from_euler((r - 0.3, p, w))
    # print('camera_pose', nice(camera_pose))

    # redo = True
    if not check_key_same(viz_dir) or redo:
        # if isdir(rgb_dir):
        #     shutil.rmtree(rgb_dir)
        if isdir(seg_dir):
            shutil.rmtree(seg_dir)
        if isdir(crop_dir):
            shutil.rmtree(crop_dir)

    ## ------------- visualization function to test -------------------
    # render_rgb_image(test_dir, viz_dir, camera_pose)
    # render_transparent_doors(test_dir, viz_dir, camera_pose)

    # if not isdir(rgb_dir):
    #     print(viz_dir, 'rgbing ...')
    #     render_segmented_rgb_images(test_dir, viz_dir, camera_pose, robot=False)
    #     reset_simulation()

    ## Pybullet segmentation mask
    num_imgs = len(get_indices(viz_dir)) + 1
    # if not isdir(seg_dir) or len(listdir(seg_dir)) < num_imgs:
    #     print(viz_dir, 'segmenting ...')
    #     render_segmentation_mask(test_dir, viz_dir, camera_pose)
    #     reset_simulation()

    if not isdir(crop_dir) or len(listdir(crop_dir)) < num_imgs:
        print(viz_dir, 'cropping ...')
        render_segmentation_mask(test_dir, viz_dir, camera_pose, crop=True)
        reset_simulation()

    ## ----------------------------------------------------------------
    add_key(viz_dir)
    shutil.rmtree(test_dir)


if __name__ == "__main__":

    task_name = args.t
    if task_name == 'tt':
        task_names = ['tt_one_fridge_pick', 'tt_one_fridge_table_in', 'tt_two_fridge_in']
    else:
        task_names = [task_name]

    all_subdirs = []
    for task_name in task_names:
        dataset_dir = join('/home/zhutiany/Documents/mamao-data/', task_name)
        # organize_dataset(task_name)
        subdirs = listdir(dataset_dir)
        subdirs.sort()
        # subdirs = ['2102']
        subdirs = [join(dataset_dir, s) for s in subdirs if isdir(join(dataset_dir, s))]
        all_subdirs += subdirs
    
    if args.p:
        import multiprocessing
        from multiprocessing import Pool

        max_cpus = 24
        num_cpus = min(multiprocessing.cpu_count(), max_cpus)
        print(f'using {num_cpus} cpus for {len(all_subdirs)} subdirs')
        with Pool(processes=num_cpus) as pool:
            for result in pool.imap_unordered(process, all_subdirs):
                pass

    else:
        for subdir in all_subdirs:
            process(subdir)
            # break
