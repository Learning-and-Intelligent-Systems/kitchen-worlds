from config import ASSET_PATH
from os.path import join, dirname
from lisdf.parsing.sdf_j import load_sdf

test_cases = ['kitchen_counter', 'm0m_0_test', 'm0m_joint_test'] 
scene_paths = [join(ASSET_PATH, 'scenes', f'{l}.lisdf') for l in test_cases]
# scene_paths = ['/home/yang/Documents/kitchen-worlds/outputs/test_full_kitchen/1230-140406_original_3/scene.lisdf']

if __name__ == "__main__":
    for lisdf_path in scene_paths:
        lissdf_results = load_sdf(lisdf_path)
        models = lissdf_results.worlds[0].models
        print(f"{lisdf_path} has {len(models)} models\n", end='\r')
    print(f'finished parsing {len(test_cases)} scene files')
