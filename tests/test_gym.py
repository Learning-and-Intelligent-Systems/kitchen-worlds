from os.path import join, isdir, getsize, abspath
import sys
from os import listdir
import numpy as np
import shutil
import random
from config import MAMAO_DATA_PATH, ASSET_PATH
from test_utils import copy_dir_for_process, get_task_names
from pybullet_tools.utils import connect
from isaac_tools.gym_utils import images_to_gif
from test_utils import get_sample_envs_200


###########################################################################


def test_load_lisdf():
    from isaac_tools.gym_utils import load_lisdf
    ori_dir = join(MAMAO_DATA_PATH, 'mm_sink_to_storage/22')
    lisdf_dir = copy_dir_for_process(ori_dir)
    for name, path, scale, is_fixed, pose, positions in load_lisdf(lisdf_dir, robots=True):
        print(name, positions)


def test_load_one(loading_effect=False, **kwargs):
    from isaac_tools.gym_utils import load_lisdf_isaacgym
    ori_dir = '/home/caelan/Programs/interns/yang/kitchen-worlds/test_cases/tt_one_fridge_pick_2'
    ori_dir = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/1231-093847_original_2'
    ori_dir = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/1230-134950_original_1'
    ori_dir = join(MAMAO_DATA_PATH, 'mm_sink_to_storage/84')
    lisdf_dir = copy_dir_for_process(ori_dir)
    world = load_lisdf_isaacgym(abspath(lisdf_dir), pause=True, loading_effect=loading_effect, **kwargs)
    if loading_effect:
        shutil.move(join(lisdf_dir, world), join(ori_dir, world))
    shutil.rmtree(lisdf_dir)


def test_load_multiple(test_camera_pose=False):
    from isaac_tools.gym_utils import load_envs_isaacgym
    # ori_dirs = get_envs_from_task()
    # ori_dirs = get_sample_envs_for_corl()
    ori_dirs = get_sample_envs_200()
    lisdf_dirs = [copy_dir_for_process(ori_dir) for ori_dir in ori_dirs]
    kwargs = dict()
    if len(ori_dirs) == 25:
        camera_point_begin = (34, 15, 10)
        camera_point_final = (34, 15, 10)
        camera_target = (0, 15, 0)
        kwargs.update(dict(
            pause=True
        ))
    elif len(ori_dirs) == 200:
        ## 14 * 6 = 84
        y = 42 + 3
        camera_point_begin = (67, y, 3)
        camera_point_final = (102, y, 24)
        camera_target = (62, y, 0)
        kwargs.update(dict(
            num_rows=14, num_cols=14, test_camera_pose=test_camera_pose
        ))

    world, _ = load_envs_isaacgym(lisdf_dirs, camera_point=camera_point_begin, camera_target=camera_target, **kwargs)

    imgs = []
    imgs.append(world.get_rgba_image(world.cameras[0]))
    if camera_point_final != camera_point_begin:
        world.set_camera_target(world.cameras[0], camera_point_final, camera_target)
        imgs.append(world.get_rgba_image(world.cameras[0]))

    img_dir = join('gym_images')
    gif_name = 'test_gym_camera_pose.gif'
    images_to_gif(img_dir, gif_name, imgs)

    print('test_load_multiple | to remove', len(lisdf_dirs))
    for lisdf_dir in lisdf_dirs:
        shutil.rmtree(lisdf_dir)


def test_load_objects():
    sys.path.append('/home/yang/Documents/playground/srl_stream/src')
    from srl_stream.gym_world import create_single_world, default_arguments
    from pybullet_tools.utils import pose_from_tform
    from trimesh import transformations
    from world_builder.partnet_scales import MODEL_SCALES, MODEL_HEIGHTS
    from world_builder.utils import get_instances, get_scale_by_category

    connect(use_gui=False, shadows=False, width=1980, height=1238)
    gym_world = create_single_world(args=default_arguments(use_gpu=True), spacing=5.)
    gym_world.set_viewer_target((3, 3, 3), target=(0, 0, 0))

    ## test all categories
    wanted = ['Food']  ## None
    unwanted = ['Cart']
    skip_till = None ## 'OvenCounter'
    problematic = []
    categories = list(MODEL_SCALES.keys()) + list(MODEL_HEIGHTS.keys())
    categories = [c for c in categories if c not in unwanted and c != c.lower()]
    if skip_till is not None:
        categories = categories[categories.index(skip_till):]

    ## test specific categories
    ids = None
    # categories = ['Food']
    # ids = ['MeatTurkeyLeg', 'VeggieGreenPepper', 'VeggieTomato']  ## 'VeggieSweetPotato', 'MeatTurkeyLeg', 'VeggieTomato',

    assets = {}
    count = 0
    for k in range(2):
        for i in range(len(categories)):
            cat = categories[i]
            if cat in unwanted or (wanted is not None and cat not in wanted):
                continue
            instances = list(get_instances(category=cat)) if ids is None else ids
            print('test_load_objects | category:', cat, 'instances:', instances)
            for j in range(len(instances)):
                count += 1
                idx = instances[j]
                if idx in problematic:
                    continue
                path = join(ASSET_PATH, 'models', cat, idx, 'mobility.urdf')
                print(f'    {count} | isfile({(cat, idx)})', f"{round(getsize(path)/(1024**2), 4)} mb")
                if (cat, idx) not in assets:
                    asset = gym_world.simulator.load_asset(
                        asset_file=path, root=None, fixed_base=True,
                        gravity_comp=False, collapse=False, vhacd=False)
                    assets[(cat, idx)] = asset
                else:
                    asset = assets[(cat, idx)]
                scale = get_scale_by_category(file=path, category=cat)
                actor = gym_world.create_actor(asset, name=idx, scale=scale)
                pose = transformations.translation_matrix([i, j+12*k, 0.1]) @ \
                       transformations.euler_matrix(*[0, 0, np.pi,])
                pose = pose_from_tform(pose)
                gym_world.set_pose(actor, pose)
            gym_world.simulator.update_viewer()
            gym_world.set_viewer_target((3+i, 3+j+12*k, 3), target=(i, j+12*k, 0))
        # gym_world.wait_if_gui()
    gym_world.wait_if_gui()


if __name__ == "__main__":
    # test_load_lisdf()
    test_load_one(loading_effect=False, load_cameras=True, save_obj_shots=True)
    # test_load_multiple(test_camera_pose=True)
    # test_load_objects()


