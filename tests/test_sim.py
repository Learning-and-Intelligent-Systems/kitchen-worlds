import sys
import os

sys.path.extend([
    '/home/caelan/Programs/SRL/srl/src',
    '/home/caelan/Programs/srl_stream/src',
    '/home/caelan/Programs/SRL/scene_synthesizer/src',
])
from omni.isaac.kit import SimulationApp

SIMULATION_APP = SimulationApp(launch_config={
    "headless": False,
})

from pybullet_planning.pybullet_tools.utils import pose_from_tform
from utils import load_lisdf, test_is_robot
from srl_stream.sim_world import SimEnv
from srl_stream.sim_utils import convert_urdf, set_prim_pose
from srl.math.transform import Transform
from scene_synthesizer.exchange.usd_export import add_light


def load_lisdf_isaacgym(lisdf_dir, robots=True, **kwargs):
    env = SimEnv(SIMULATION_APP, usd_path=None)
    env.add_ground(z=0.)
    add_light(env.stage, scene_path='/Light',
              transform=Transform.from_translation([0, 0, 10]),
              radius=1, intensity=300000)

    for name, path, scale, is_fixed, tform in load_lisdf(lisdf_dir, robots=robots, **kwargs):
        prim_path = f'/{name}'
        prim_path = convert_urdf(path, dest_path=prim_path, scale=scale, merge=False, decomposition=False)
        print(prim_path)
        set_prim_pose(prim_path, pose_from_tform(tform))
    env.wait_if_gui()

if __name__ == "__main__":
    lisdf_dir = '/home/caelan/Programs/interns/yang/kitchen-worlds/test_cases/tt_one_fridge_pick_2'
    load_lisdf_isaacgym(os.path.abspath(lisdf_dir))
