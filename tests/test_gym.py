from os.path import join, isdir, getsize
from os import listdir
import shutil
import random
from config import *
from test_utils import copy_dir_for_process


def get_envs_from_task(task_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_pick'):
    ori_dirs = [join(task_dir, f) for f in listdir(task_dir) if isdir(join(task_dir, f))]
    ori_dirs.sort()
    return ori_dirs


def get_sample_envs_for_corl():
    task_scenes = {
        'mm_one_fridge_pick': ['5', '226', '229', '232', '288', '295', '311'],
        'mm_one_fridge_table_on': ['48', '69', '168', '248', '296', '325', '313'],
        'mm_one_fridge_table_in': ['7', '88', '97', '202', '305', '394', '419', '466'],
        'mm_two_fridge_in': ['36', '104', '186', '294', '346', '405', '473', '493', '498', '502'],
        'mm_two_fridge_pick': ['222', '347', '472']
    }
    dirs = []
    for k, v in task_scenes.items():
        for i in v:
            dirs.append(join('/home/yang/Documents/fastamp-data', k, i))
    random.shuffle(dirs)
    return dirs


def test_load_lisdf():
    from isaac_tools.gym_utils import load_lisdf
    ori_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_in/4'
    lisdf_dir = copy_dir_for_process(ori_dir)
    for name, path, scale, is_fixed, pose, positions in load_lisdf(lisdf_dir, robots=True):
        print(name, positions)


def test_load_one():
    from isaac_tools.gym_utils import load_lisdf_isaacgym
    ori_dir = '/home/caelan/Programs/interns/yang/kitchen-worlds/test_cases/tt_one_fridge_pick_2'
    ori_dir = '/home/yang/Documents/fastamp-data/tt_two_fridge_in/4'
    lisdf_dir = copy_dir_for_process(ori_dir)
    world = load_lisdf_isaacgym(abspath(lisdf_dir), pause=True)
    shutil.rmtree(lisdf_dir)


def test_load_multiple():
    from isaac_tools.gym_utils import load_envs_isaacgym
    # ori_dirs = get_envs_from_task()
    ori_dirs = get_sample_envs_for_corl()
    lisdf_dirs = [copy_dir_for_process(ori_dir) for ori_dir in ori_dirs]
    world = load_envs_isaacgym(lisdf_dirs, camera_point=(34, 15, 10), camera_target=(0, 15, 0), pause=True)
    print('test_load_multiple | to remove', len(lisdf_dirs))
    for lisdf_dir in lisdf_dirs:
        shutil.rmtree(lisdf_dir)


def test_load_objects():
    sys.path.append('/home/yang/Documents/playground/srl_stream/src')
    from srl_stream.gym_world import create_single_world, default_arguments
    from pybullet_tools.bullet_utils import get_scale_by_category
    from pybullet_tools.utils import pose_from_tform
    from trimesh import transformations

    gym_world = create_single_world(args=default_arguments(use_gpu=False), spacing=5.)
    gym_world.set_viewer_target((1, 1, 1), target=(0, 0, 0))

    ids = ['MeatTurkeyLeg', 'VeggieGreenPepper', 'VeggieTomato']  ## 'VeggieSweetPotato', 'MeatTurkeyLeg', 'VeggieTomato',
    for i in range(len(ids)):
        idx = ids[i]
        path = f'/home/yang/Documents/jupyter-worlds/assets/models/Food/{idx}/mobility.urdf'
        print('isfile', isfile(path), f"{round(getsize(path), 1)/1024} kb")
        asset = gym_world.simulator.load_asset(
            asset_file=path, root=None, fixed_base=True,
            gravity_comp=False, collapse=False, vhacd=False)
        scale = get_scale_by_category(file=path, category='Food')
        actor = gym_world.create_actor(asset, name=idx, scale=scale)
        pose = transformations.translation_matrix([0, 0.5*i, 0.1]) @ \
               transformations.euler_matrix(*[0, 0, 0,])
        pose = pose_from_tform(pose)
        gym_world.set_pose(actor, pose)
        gym_world.simulator.update_viewer()
    gym_world.wait_if_gui()


if __name__ == "__main__":
    # test_load_lisdf()
    # test_load_one()
    # test_load_multiple()
    test_load_objects()


