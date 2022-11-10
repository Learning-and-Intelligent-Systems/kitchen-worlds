import sys
from os.path import join, abspath, dirname, isdir, isfile


def absjoin(*args):
    return abspath(join(*args))


sys.path.append(absjoin('..'))
sys.path.append(absjoin('..', 'lisdf'))
sys.path.append(absjoin('..', 'pddlstream'))
sys.path.append(absjoin('..', 'pybullet_planning'))

import warnings
warnings.filterwarnings('ignore')

PROJECT_DIR = absjoin(dirname(__file__), '..')
ASSET_PATH = absjoin(PROJECT_DIR, 'assets')
EXP_PATH = absjoin(PROJECT_DIR, 'test_cases')
OUTPUT_PATH = absjoin(PROJECT_DIR, 'outputs')
SCENE_CONFIG_PATH = absjoin('..', 'pybullet_planning', 'scene_configs')
MAMAO_DATA_PATH = absjoin(PROJECT_DIR, '..', 'fastamp-data')