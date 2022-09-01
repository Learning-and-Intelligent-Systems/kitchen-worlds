import os.path

from srl_stream.gym_world import create_single_world, default_arguments

from pybullet_planning.pybullet_tools.utils import pose_from_tform
from utils import load_lisdf, test_is_robot


def load_lisdf_isaacgym(lisdf_dir, robots=False, **kwargs):
    # TODO: Segmentation fault - possibly cylinders & mimic joints
    gym_world = create_single_world(args=default_arguments(use_gpu=False), spacing=5.)
    for name, path, scale, is_fixed, pose in load_lisdf(lisdf_dir, robots=robots, **kwargs):
        is_robot = test_is_robot(name)
        asset = gym_world.simulator.load_asset(
            asset_file=path, root=None, fixed_base=is_fixed or is_robot, y_up=is_robot,
            gravity_comp=is_robot, collapse=False, vhacd=False)
        actor = gym_world.create_actor(asset, name=name, scale=scale)
        gym_world.set_pose(actor, pose_from_tform(pose))
    gym_world.wait_if_gui()
    return gym_world

if __name__ == "__main__":
    lisdf_dir = '/home/caelan/Programs/interns/yang/kitchen-worlds/test_cases/tt_one_fridge_pick_2'
    load_lisdf_isaacgym(os.path.abspath(lisdf_dir))
