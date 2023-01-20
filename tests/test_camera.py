import copy
import os
from tqdm import tqdm
import PIL.Image
import numpy as np
import argparse
import sys
import time
from config import EXP_PATH
from pybullet_tools.utils import quat_from_euler, reset_simulation, remove_body, AABB, \
    get_aabb_extent, get_aabb_center, get_joint_name, get_link_name, euler_from_quat, \
    set_color, apply_alpha, YELLOW, WHITE, get_aabb, get_point, wait_unlocked, \
    get_joint_positions, GREEN, get_pose, unit_pose
from pybullet_tools.bullet_utils import get_segmask, get_door_links, adjust_segmask, \
    get_obj_keys_for_segmentation

from lisdf_tools.lisdf_loader import load_lisdf_pybullet, get_depth_images, create_gripper_robot, \
    make_furniture_transparent
from lisdf_tools.image_utils import draw_bb, crop_image, get_mask_bb, expand_mask, \
    make_image_background
import json
import shutil
from os import listdir
from os.path import join, isdir, isfile, dirname, getmtime, basename

# from utils import load_lisdf_synthesizer
from mamao_tools.data_utils import get_indices, exist_instance, get_init_tuples, \
    get_body_map, get_world_center, add_to_planning_config
from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser

N_PX = 224
NEW_KEY = 'meraki'
ACCEPTED_KEYS = [NEW_KEY, 'crop_fix', 'rgb', 'meraki']

#################################################################

# DEFAULT_TASK = 'tt_two_fridge_pick'
# DEFAULT_TASK = 'tt_two_fridge_in'
# DEFAULT_TASK = 'tt'
# DEFAULT_TASK = 'mm'
# DEFAULT_TASK = 'mm_two_fridge_pick'
# DEFAULT_TASK = 'ff'
# DEFAULT_TASK = 'ww_two_fridge_in'
# DEFAULT_TASK = 'ww'
# DEFAULT_TASK = 'zz'
# DEFAULT_TASK = '_examples'
# DEFAULT_TASK = 'ff_two_fridge_goals'

# DEFAULT_TASK = 'mm_storage'
# DEFAULT_TASK = 'mm_sink'
# DEFAULT_TASK = 'mm_braiser'
# DEFAULT_TASK = 'mm_storage_long'

DEFAULT_TASK = 'tt'

#################################################################

# GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/230115_115113_original_0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_sink/0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_storage/0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_braiser/0'
GIVEN_PATH = None

MODIFIED_TIME = 1663895681
PARALLEL = True
USE_VIEWER = True
REDO = False


parser = get_base_parser(task_name=DEFAULT_TASK, parallel=PARALLEL, use_viewer=USE_VIEWER)
args = parser.parse_args()


def get_camera_pose(viz_dir, key="obs_camera_pose"):
    config = json.load(open(join(viz_dir, 'planning_config.json')))
    if key not in config:
        return None
    camera_pose = config[key]
    if len(camera_pose) == 6:
        point = camera_pose[:3]
        euler = camera_pose[3:]
        camera_pose = (point, quat_from_euler(euler))
    return camera_pose


def create_doorless_lisdf(test_dir):
    lisdf_file = join(test_dir, 'scene.lisdf')
    text = open(lisdf_file).read().replace('MiniFridge', 'MiniFridgeDoorless')
    doorless_lisdf = join(test_dir, 'scene_dooless.lisdf')
    with open(doorless_lisdf, 'w') as f:
        f.write(text)
    return doorless_lisdf


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
            # tmp_config_file = join(viz_dir, 'planning_config_tmp.json')
            # shutil.move(config_file, tmp_config_file)
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=3)


def adjust_indices_for_full_kitchen(indices):
    return {k: v for k, v in indices.items() if not \
            'pr2' in v or 'braiser_bottom' in v or 'sink_bottom' in v}


def render_segmentation_mask(test_dir, viz_dir, camera_poses, camera_kwargs, crop=False,
                             transparent=False, pairs=None, width=1280, height=960, fx=800):
    ## width = 1960, height = 1470, fx = 800
    world = load_lisdf_pybullet(test_dir, width=width, height=height, verbose=False,
                                transparent=transparent)
    remove_body(world.robot.body)
    if transparent:
        world.make_doors_transparent()
        doorless_lisdf = create_doorless_lisdf(test_dir)

    """ find the door links """
    indices = get_indices(viz_dir, body_map=get_body_map(viz_dir, world))
    indices = adjust_indices_for_full_kitchen(indices)

    for i in range(len(camera_poses)):
        common = dict(img_dir=viz_dir, width=width, height=height, fx=fx)
        world.add_camera(camera_poses[i], **common, **camera_kwargs[i])

        ## a fix for previous wrong lisdf names in planning_config[name_to_body]
        # fix_planning_config(viz_dir)

        new_key = 'seg_image' if not crop else 'crop_image'
        new_key = 'transp_image' if transparent else new_key
        new_key = f"{new_key}s"
        if len(camera_poses) > 1:
            new_key = f"{new_key}_{i}"
        rgb_dir = join(viz_dir, new_key)
        os.makedirs(rgb_dir, exist_ok=True)

        ## get the scene image
        imgs = world.camera.get_image(segment=True, segment_links=True)
        rgb = imgs.rgbPixels[:, :, :3]
        im = PIL.Image.fromarray(rgb)
        im.save(join(rgb_dir, f'{new_key}_scene.png'))
        im_name = new_key+"_[{index}]_{name}.png"

        """ get segmask with opaque doors """
        seg = imgs.segmentationMaskBuffer
        # seg = imgs.segmentationMaskBuffer[:, :, 0].astype('int32')
        unique = get_segmask(seg)
        obj_keys = get_obj_keys_for_segmentation(indices, unique)

        """ get segmask with transparent doors """
        if transparent:
            reset_simulation()
            world = load_lisdf_pybullet(doorless_lisdf, width=width, height=height,
                                        verbose=False, jointless=True)
            remove_body(world.robot.body)
            # world.add_camera(camera_pose, viz_dir, width=width, height=height, fx=fx)
            unique = adjust_segmask(unique, world)

        """ get pairs of objects to render """
        if pairs is not None:
            inv_indices = {v: k for k, v in indices.items()}
            indices.update({'+'.join([str(inv_indices[n]) for n in p]): p for p in pairs})

        """ render cropped images """
        for k, v in indices.items():
            if '+' not in k:  ## single object/part
                keys = obj_keys[v]
            else:
                keys = []
                for vv in v:
                    keys.extend(obj_keys[vv])
                v = '+'.join([n for n in v])

            ## skip generation if already exists
            file_name = join(rgb_dir, im_name.format(index=str(k), name=v))
            if isfile(file_name): continue

            ## generate image
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

            ## crop image with object bounding box centered
            if crop:
                bb = get_mask_bb(mask)
                # if bb is not None:
                #     draw_bb(new_image, bb)
                im = crop_image(im, bb, width, height, N_PX=N_PX)

            # im.show()
            im.save(file_name)
        #     print(v)

        if not transparent and len(camera_poses) > 1 and i == 0:
            make_furniture_transparent(world, viz_dir, lower_tpy=1, upper_tpy=0)


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
        os.remove(tmp_config_file)


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


def get_num_images(viz_dir, pairwise=False):
    indices = get_indices(viz_dir)
    indices = adjust_indices_for_full_kitchen(indices)
    objs = list(indices.values())
    num_images = len(indices) + 1
    pairs = []
    if pairwise:
        init = get_init_tuples(viz_dir)
        for f in init:
            oo = [i for i in f if i in objs]
            if len(oo) >= 2:
                # print(f)
                pairs.append(oo)
    num_images += len(pairs)
    return num_images, pairs


def process(viz_dir, redo=REDO):
    test_dir = copy_dir_for_process(viz_dir)

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
    rgb_dir = join(viz_dir, 'rgb_images')
    seg_dirs = [join(viz_dir, f'seg_images_{i}') for i in range(4)]
    crop_dirs = [join(viz_dir, f'crop_images_{i}') for i in range(4)]
    transp_dirs = [join(viz_dir, f'transp_images_{i}') for i in range(4)]
    tmp_file = join(viz_dir, 'planning_config_tmp.json')

    if isdir(rgb_dir):
        shutil.rmtree(rgb_dir)
    if isfile(tmp_file):
        os.remove(tmp_file)

    """ just one rgb image """
    camera_pose = get_camera_pose(viz_dir)
    if camera_pose is None:  ## RSS
        cx, cy, lx, ly = get_world_center(viz_dir)
        camera_kwargs = [
            {'camera_point': (cx+5, cy, 2.5), 'target_point': (0, cy, 1)},
        ]
        for y in [cy-3*ly/8, cy-ly/8, cy+ly/8, cy+3*ly/8]: ## [cy-ly/3, cy, cy+ly/3]:
            camera_kwargs.append(
                {'camera_point': (cx+1, y, 2.2), 'target_point': (0, y, 0.5)}
            )
        camera_poses = [unit_pose()] * len(camera_kwargs)
        add_to_planning_config(viz_dir, 'camera_kwargs', camera_kwargs)

    else:  ## CoRL
        (x, y, z), quat = camera_pose
        (r, p, w) = euler_from_quat(quat)
        if x < 6.5:
            x = np.random.normal(7, 0.2)
            # redo = True
        camera_pose = [(x, y, z + 1), quat_from_euler((r - 0.3, p, w))]
        camera_poses = [camera_pose]
        camera_kwargs = [dict()]
        add_to_planning_config(viz_dir, 'img_camera_pose', camera_pose)

    # check_file = join(seg_dirs[0], 'crop_image_scene.png')
    # if isfile(check_file) and os.path.getmtime(check_file) > MODIFIED_TIME:
    #     redo = False

    """ other types of image """
    redo = False
    if not check_key_same(viz_dir) or redo:
        # if isdir(rgb_dir):
        #     shutil.rmtree(rgb_dir)
        # for crop_dir in crop_dirs:
        #     if isdir(crop_dir):
        #         shutil.rmtree(crop_dir)
        for seg_dir in seg_dirs:
            if isdir(seg_dir):
                shutil.rmtree(seg_dir)
        for transp_dir in transp_dirs:
            if isdir(transp_dir):
                shutil.rmtree(transp_dir)

    ## ------------- visualization function to test -------------------
    # render_rgb_image(test_dir, viz_dir, camera_pose)
    # render_transparent_doors(test_dir, viz_dir, camera_pose)

    # if not isdir(rgb_dir):
    #     print(viz_dir, 'rgbing ...')
    #     render_segmented_rgb_images(test_dir, viz_dir, camera_pose, robot=False)
    #     reset_simulation()

    pairwise = False

    ## Pybullet segmentation mask
    num_imgs, pairs = get_num_images(viz_dir, pairwise=pairwise)

    ## choose configuration
    name, dirs, kwargs = 'segmenting', seg_dirs, dict()
    # name, dirs, kwargs = 'cropping', crop_dirs, dict(crop=True)
    # name, dirs, kwargs = 'transparent doors', transp_dirs, dict(crop=True, transparent=True)

    ## skip if all done
    done = True
    for img_dir in dirs:
        done = done and isdir(img_dir) and len(listdir(img_dir)) >= num_imgs
    if not done or redo:
        print(viz_dir, f'{name} ...')
        render_segmentation_mask(test_dir, viz_dir, camera_poses, camera_kwargs, **kwargs)
        reset_simulation()
    else:
        print('skipping', viz_dir, f'{name}')


    ## ----------------------------------------------------------------
    add_key(viz_dir)
    shutil.rmtree(test_dir)


if __name__ == "__main__":
    process_all_tasks(process, args.t, parallel=args.p, path=GIVEN_PATH)
