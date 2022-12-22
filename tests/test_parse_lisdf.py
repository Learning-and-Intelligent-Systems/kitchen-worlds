from config import ASSET_PATH
from os.path import join
from lisdf.parsing.sdf_j import load_sdf

test_cases = ['m0m_0_test', 'm0m_joint_test', 'kitchen_counter']  ## 'counter'

if __name__ == "__main__":

    scene_paths = [join(ASSET_PATH, 'scenes', f'{l}.lisdf') for l in test_cases]
    scene_paths = ['/home/yang/Documents/jupyter-worlds/test_cases/temp_test_full_kitchen_1222-105144_original/scene.lisdf']

    for lisdf_path in scene_paths:
        lissdf_results = load_sdf(lisdf_path)
        models = lissdf_results.worlds[0].models
        print(f"{lisdf_path} has {len(models)} models\n", end='\r')
    print(f'finished parsing {len(test_cases)} scene files')
