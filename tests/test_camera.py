import copy
import os
from tqdm import tqdm
import PIL.Image
import numpy as np
import json
import shutil
from os import listdir
from os.path import join, isdir, isfile, dirname, getmtime, basename
import time
from config import EXP_PATH

from pybullet_tools.utils import quat_from_euler, reset_simulation, remove_body, AABB, \
    get_aabb_extent, get_aabb_center, get_joint_name, get_link_name, euler_from_quat, \
    set_color, apply_alpha, YELLOW, WHITE, get_aabb, get_point, wait_unlocked, \
    get_joint_positions, GREEN, get_pose, unit_pose
from pybullet_tools.bullet_utils import get_segmask, get_door_links, adjust_segmask, \
    get_obj_keys_for_segmentation

from lisdf_tools.lisdf_loader import load_lisdf_pybullet, get_depth_images, create_gripper_robot, \
    make_furniture_transparent, get_camera_kwargs
from lisdf_tools.image_utils import draw_bb, crop_image, get_mask_bb, save_seg_image_given_obj_keys

# from utils import load_lisdf_synthesizer
from mamao_tools.data_utils import get_indices, exist_instance, get_init_tuples, \
    get_body_map, get_world_center, add_to_planning_config, get_worlds_aabb
from test_utils import process_all_tasks, copy_dir_for_process, get_base_parser

N_PX = 224
NEW_KEY = 'channel7'
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

#################################################################

# DEFAULT_TASK = 'mm'
# DEFAULT_TASK = 'mm_storage'  ## done
# DEFAULT_TASK = 'mm_braiser'
DEFAULT_TASK = 'mm_sink'
# DEFAULT_TASK = 'mm_braiser_to_storage'
# DEFAULT_TASK = 'mm_sink_to_storage'
# DEFAULT_TASK = 'mm_storage_long'

# DEFAULT_TASK = 'tt_storage'  ## done
# DEFAULT_TASK = 'tt_storage'  ## done
# DEFAULT_TASK = 'tt_braiser'
# DEFAULT_TASK = 'tt_storage_long'
# DEFAULT_TASK = 'tt_braiser_to_storage'
# DEFAULT_TASK = 'tt'

# DEFAULT_TASK = 'hh_storage'  ## done
# DEFAULT_TASK = 'hh_braiser'  ## done

LARGER_WORLD = 'mm_' in DEFAULT_TASK or 'tt_' in DEFAULT_TASK

#################################################################

GIVEN_PATH = None
# GIVEN_PATH = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/230115_115113_original_0'
GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_sink/0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_storage/0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_braiser_to_storage/1'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'mm_sink_to_storage/84'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'tt_storage/0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'tt_braiser/0'
# GIVEN_PATH = '/home/yang/Documents/fastamp-data-rss/' + 'tt_storage/2'

MODIFIED_TIME = 1663895681
PARALLEL = False and (GIVEN_PATH is None)
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
    return {k: v for k, v in indices.items() if not 'pr2' in v}


def render_segmentation_mask(test_dir, viz_dir, camera_poses, camera_kwargs, camera_zoomins=[], crop=False,
                             transparent=False, pairs=None, width=1280, height=960, fx=800, done=None):
    ## width = 1960, height = 1470, fx = 800
    world = load_lisdf_pybullet(test_dir, width=width, height=height, verbose=False,
                                transparent=transparent, larger_world=LARGER_WORLD)
    remove_body(world.robot.body)
    if transparent:
        world.make_doors_transparent()
        doorless_lisdf = create_doorless_lisdf(test_dir)

    """ find the door links """
    body_map = get_body_map(viz_dir, world) if 'mm_' in viz_dir else None
    indices = get_indices(viz_dir, larger=True, body_map=body_map)
    indices = adjust_indices_for_full_kitchen(indices)

    ## pointing at goal regions: initial and final
    camera_poses.extend([unit_pose()] * len(camera_zoomins))
    camera_kwargs.extend([get_camera_kwargs(world, d) for d in camera_zoomins])

    common = dict(img_dir=viz_dir, width=width//2, height=height//2, fx=fx//2)
    crop_kwargs = dict(crop=crop, center=crop, width=width//2, height=height//2, N_PX=N_PX)

    ## a fix for previous wrong lisdf names in planning_config[name_to_body]
    # fix_planning_config(viz_dir)

    for i in range(len(camera_poses)):
        if done is not None and done[i]:
            continue

        # ---------- make furniture disappear
        if not transparent and len(camera_poses) > 1 and i == len(camera_poses) - 1:
            make_furniture_transparent(world, viz_dir, lower_tpy=1, upper_tpy=0,
                                       remove_upper_furnitures=True)

        world.add_camera(camera_poses[i], **common, **camera_kwargs[i])

        if not crop:
            if 0 < i < len(camera_poses)-len(camera_zoomins):
                crop_kwargs = dict(crop=True, center=False, width=width//2, height=height//2,
                                   N_PX=height//2, align_vertical='top', keep_ratio=True)
            elif i >= len(camera_poses)-len(camera_zoomins):
                ## more zoomed-in on braiser
                n_px = int(height//2 * 0.6) if (i == len(camera_poses)-1) else int(height//2)
                crop_kwargs = dict(crop=True, center=False, width=width//2, height=height//2,
                                   N_PX=n_px, align_vertical='center', keep_ratio=True)

        new_key = 'seg_image' if not crop else 'crop_image'
        new_key = 'transp_image' if transparent else new_key
        new_key = f"{new_key}s"
        if len(camera_poses) > 1:
            new_key = f"{new_key}_{i}"
        rgb_dir = join(viz_dir, new_key)
        os.makedirs(rgb_dir, exist_ok=True)
        # print(f'    ..... generating in {new_key}')

        ## get the scene image
        imgs = world.camera.get_image(segment=True, segment_links=True)
        rgb = imgs.rgbPixels[:, :, :3]
        im = PIL.Image.fromarray(rgb)
        if not crop and crop_kwargs['crop']:
            im = crop_image(im, **{k: v for k, v in crop_kwargs.items() if k not in ['crop', 'center']})
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
        files = []
        for k, v in indices.items():
            ## single object/part
            if '+' not in k:
                keys = obj_keys[v]

            ## pairs of objects/parts
            else:
                keys = []
                for vv in v:
                    keys.extend(obj_keys[vv])
                v = '+'.join([n for n in v])

            ## skip generation if already exists
            file_name = join(rgb_dir, im_name.format(index=str(k), name=v))
            if isfile(file_name): continue
            files.append(file_name)

            ## save the cropped image
            save_seg_image_given_obj_keys(rgb, keys, unique, file_name, **crop_kwargs)

        files = [f for f in files if 'braiserbody' in f]
        if len(files) == 2:
            bottom_file = [f for f in files if 'braiser_bottom' in f][0]
            braiser_file = [f for f in files if f not in bottom_file][0]
            shutil.copy(braiser_file, bottom_file)


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


def generate_images(viz_dir, redo=REDO):

    # A = join(viz_dir, 'seg_images_5')
    # B = join(viz_dir, 'seg_images_6')
    # C = join(viz_dir, 'seg_images_9')
    # shutil.move(A, C)
    # shutil.move(B, A)
    # shutil.move(C, B)
    # # return
    # ## -----------------------------------------------
    # files = [join(A, f) for f in listdir(A) if 'png' in f]
    # for f in files:
    #     shutil.move(f, f.replace('/seg_images_6', '/seg_images_5'))
    # files = [join(B, f) for f in listdir(B) if 'png' in f]
    # for f in files:
    #     shutil.move(f, f.replace('/seg_images_5', '/seg_images_6'))
    # return

    ##################################################
    # file = join(viz_dir, 'seg_images_0', 'seg_images_0_[25]_braiserbody#1.png')
    # if not isfile(file):
    #     print(file)
    # return

    # run_num = eval(viz_dir.split('/')[-1])
    # if run_num < 80:
    #     return

    """ get the camera poses """
    camera_pose = get_camera_pose(viz_dir)
    camera_zoomins = []
    if camera_pose is None:  ## RSS
        cx, cy, lx, ly = get_world_center(viz_dir)
        camera_kwargs = [
            {'camera_point': (cx+5, cy, 2.5), 'target_point': (0, cy, 1)},
        ]
        for y in [cy-3*ly/8, cy-ly/8, cy+ly/8, cy+3*ly/8]: ## [cy-ly/3, cy, cy+ly/3]:
            camera_kwargs.append(
                # {'camera_point': (cx+1, y, 2.2), 'target_point': (0, y, 0.5)}
                {'camera_point': (cx+1.8, y, 2.8), 'target_point': (0, y, 0.5)}
            )

        camera_poses = [unit_pose()] * len(camera_kwargs)
        config = add_to_planning_config(viz_dir, {'camera_kwargs': camera_kwargs})
        camera_zoomins += config['camera_zoomins']

    else:  ## CoRL
        (x, y, z), quat = camera_pose
        (r, p, w) = euler_from_quat(quat)
        if x < 6.5:
            x = np.random.normal(7, 0.2)
            # redo = True
        camera_pose = [(x, y, z + 1), quat_from_euler((r - 0.3, p, w))]
        camera_poses = [camera_pose]
        camera_kwargs = [dict()]
        add_to_planning_config(viz_dir, {'img_camera_pose': camera_pose})

    num_dirs = len(camera_kwargs) + len(camera_zoomins)
    test_dir = copy_dir_for_process(viz_dir)

    rgb_dir = join(viz_dir, 'rgb_images')
    seg_dirs = [join(viz_dir, f'seg_images_{i}') for i in range(num_dirs)]
    crop_dirs = [join(viz_dir, f'crop_images_{i}') for i in range(num_dirs)]
    transp_dirs = [join(viz_dir, f'transp_images_{i}') for i in range(num_dirs)]

    # check_file = join(seg_dirs[0], 'crop_image_scene.png')
    # if isfile(check_file) and os.path.getmtime(check_file) > MODIFIED_TIME:
    #     redo = False

    """ other types of image """
    redo = True ## or GIVEN_PATH is not None
    if not check_key_same(viz_dir) or redo:
        # if isdir(rgb_dir):
        #     shutil.rmtree(rgb_dir)
        # for crop_dir in crop_dirs:
        #     if isdir(crop_dir):
        #         shutil.rmtree(crop_dir)
        for seg_dir in seg_dirs:
            if isdir(seg_dir) and ('/seg_images_5' in seg_dir or '/seg_images_6' in seg_dir):
                shutil.rmtree(seg_dir)
        # for transp_dir in transp_dirs:
        #     if isdir(transp_dir):
        #         shutil.rmtree(transp_dir)

    ## ----------------------------------------------------
    # for seg_dir in seg_dirs:
    #     files = [join(seg_dir, f) for f in listdir(seg_dir) if 'braiserbody' in f]
    #     if len(files) == 2:
    #         bottom_file = [f for f in files if 'braiser_bottom' in f][0]
    #         braiser_file = [f for f in files if f not in bottom_file][0]
    #         shutil.copy(braiser_file, bottom_file)

    ## ----------------------------------------------------
    # if exist_instance(viz_dir, '100015') and isdir(seg_dirs[0]):
    #     braiser_files = [join(seg_dirs[0], f) for f in listdir(seg_dirs[0]) \
    #                      if f.endswith('.png') and 'braiser' in f]
    #     if len(braiser_files) > 0:
    #         if 1674437043 > os.path.getmtime(braiser_files[0]) > 1674351569:
    #             print('braiser problem', viz_dir)
    #             for f in braiser_files:
    #                 os.remove(f)

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
    done = []
    for img_dir in dirs:
        done.append(isdir(img_dir) and len(listdir(img_dir)) >= num_imgs)

        # if isdir(img_dir):
        #     files = [join(img_dir, f) for f in listdir(img_dir) if 'braiserbody' in f]
        #     if len(files) == 2:
        #         bottom_file = [f for f in files if 'braiser_bottom' in f][0]
        #         braiser_file = [f for f in files if f not in bottom_file][0]
        #         shutil.copy(braiser_file, bottom_file)

    if (False in done) or redo:
        print(viz_dir, f'{name} ...')
        render_segmentation_mask(test_dir, viz_dir, camera_poses, camera_kwargs, camera_zoomins,
                                 done=done, **kwargs)
        reset_simulation()
    else:
        print('skipping', viz_dir, f'{name}')

    ## ----------------------------------------------------------------
    add_key(viz_dir)
    shutil.rmtree(test_dir)


def process_worlds_aabb():
    """ ((-0.879, -2.56, -0.002), (1.15, 9.477, 2.841)) """
    run_dirs = process_all_tasks(None, args.t, parallel=False, return_dirs=True)
    aabb = get_worlds_aabb(run_dirs)


if __name__ == "__main__":
    process = generate_images
    process_all_tasks(process, args.t, parallel=args.p, path=GIVEN_PATH)

    # process_worlds_aabb()
