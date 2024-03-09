import os

from config import ASSET_PATH
from os.path import join, dirname
import shutil
from lisdf.parsing.sdf_j import load_sdf

test_cases = ['m0m_0_test', 'm0m_joint_test', 'kitchen_counter']  ## 'counter'

if __name__ == "__main__":

    scene_paths = [join(ASSET_PATH, 'scenes', f'{l}.lisdf') for l in test_cases]
    scene_paths = ['/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/1230-140406_original_3/scene.lisdf']

    for lisdf_path in scene_paths:
        temp_file = '/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen_1230-140406_original_3/scene.lisdf'
        temp_dir = dirname(temp_file)
        os.makedirs(temp_dir, exist_ok=True)
        shutil.copyfile(lisdf_path, temp_file)
        lisdf_path = temp_file
        lissdf_results = load_sdf(lisdf_path)
        models = lissdf_results.worlds[0].models
        print(f"{lisdf_path} has {len(models)} models\n", end='\r')
        shutil.rmtree(temp_dir)
    print(f'finished parsing {len(test_cases)} scene files')
