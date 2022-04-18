import sys
from os.path import join, abspath, dirname, isdir, isfile
sys.path.append('lisdf')
from lisdf.parsing.sdf_j import load_sdf

if __name__ == "__main__":

    for lisdf_test in ['m0m_0_test', 'm0m_joint_test', 'kitchen_counter']:  ## 'counter'
        print(f"... paring {lisdf_test}.lisdf")
        lisdf_path = join('assets', 'scenes', f'{lisdf_test}.lisdf')
        lissdf_results = load_sdf(lisdf_path)
        models = lissdf_results.worlds[0].models
        print(f"{lisdf_test} has {len(models)} models\n")
