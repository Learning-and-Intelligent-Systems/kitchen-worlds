from config import ASSET_PATH
from os.path import join
from lisdf.parsing.sdf_j import load_sdf

test_cases = ['m0m_0_test', 'm0m_joint_test', 'kitchen_counter'] ## 'counter'

if __name__ == "__main__":

    for lisdf_test in test_cases:
        lisdf_path = join(ASSET_PATH, 'scenes', f'{lisdf_test}.lisdf')
        lissdf_results = load_sdf(lisdf_path)
        models = lissdf_results.worlds[0].models
        print(f"{lisdf_test} has {len(models)} models\n", end='\r')
    print(f'finished parsing {len(test_cases)} scene files')
