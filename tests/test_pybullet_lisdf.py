import shutil

from config import ASSET_PATH, EXP_PATH, DATA_PATH
from os.path import join, abspath
from os import listdir
from pybullet_planning.lisdf_tools.lisdf_loader import load_lisdf_pybullet
from pybullet_planning.pybullet_tools.utils import wait_if_gui, disconnect, reset_simulation




def load_test_cases():
    # scene_names = ['test_scene', 'kitchen_lunch'] ## 'm0m_joint_test', 'kitchen_basics', 'kitchen_counter',
    # lisdf_paths = [join(ASSET_PATH, 'scenes', f'{n}.lisdf') for n in scene_names]

    exp_names = ['test_pr2_kitchen', 'test_feg_pick']  ##
    lisdf_paths = [join(EXP_PATH, n) for n in exp_names]

    for lisdf_path in lisdf_paths:
        world = load_lisdf_pybullet(lisdf_path)
        wait_if_gui('load next test scene?')
        reset_simulation()


def load_dataset_cases(task_name = 'one_fridge_pick_pr2'):
    task_dir = join(DATA_PATH, task_name)
    lisdf_paths = [f for f in listdir(task_dir)]

    for f in lisdf_paths:
        old_path = join(task_dir, f)
        new_path = join(DATA_PATH, f)
        shutil.copytree(old_path, new_path)

        world = load_lisdf_pybullet(new_path)
        world.open_all_doors()
        wait_if_gui('load next test scene?')
        reset_simulation()

        shutil.rmtree(new_path)


if __name__ == "__main__":
    load_dataset_cases(task_name='one_fridge_pick_pr2')
