from config import ASSET_PATH
from os.path import join, abspath
from lisdf_loader import load_lisdf_pybullet
from pybullet_planning.pybullet_tools.utils import wait_if_gui, disconnect

if __name__ == "__main__":

    for lisdf_test in ['test_scene', 'kitchen_lunch']: ## 'm0m_joint_test', 'kitchen_basics', 'kitchen_counter',
        lisdf_path = join(ASSET_PATH, 'scenes', f'{lisdf_test}.lisdf')
        world = load_lisdf_pybullet(lisdf_path)
        wait_if_gui('load next test scene?')
        disconnect()
