from config import ASSET_PATH, EXP_PATH
from os.path import join, abspath
from lisdf_loader import load_lisdf_pybullet
from pybullet_planning.pybullet_tools.utils import wait_if_gui, disconnect

lisdf_paths = []

# scene_names = ['test_scene', 'kitchen_lunch'] ## 'm0m_joint_test', 'kitchen_basics', 'kitchen_counter',
# lisdf_paths.extend([join(ASSET_PATH, 'scenes', f'{n}.lisdf') for n in scene_names])

exp_names = ['kitchen']
lisdf_paths.extend([join(EXP_PATH, n, 'scene.lisdf') for n in exp_names])

if __name__ == "__main__":

    for lisdf_path in lisdf_paths:
        world = load_lisdf_pybullet(lisdf_path)
        wait_if_gui('load next test scene?')
        disconnect()
