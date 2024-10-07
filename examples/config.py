import sys
from os.path import join, abspath, dirname, isdir, isfile

abs_join = lambda *args, **kwargs: abspath(join(*args, **kwargs))

PROJECT_DIR = abspath(join(dirname(__file__), '..'))
PBP_PATH = abs_join(PROJECT_DIR, 'pybullet_planning')

sys.path.append(PROJECT_DIR)
sys.path.append(PBP_PATH)
sys.path.append(join(PROJECT_DIR, 'lisdf'))
sys.path.append(join(PROJECT_DIR, 'pddlstream'))

ASSET_PATH = abs_join(PROJECT_DIR, 'assets')
EXP_PATH = abs_join(PROJECT_DIR, 'test_cases')
OUTPUT_PATH = abs_join(PROJECT_DIR, 'outputs')
TEMP_PATH = abs_join(PROJECT_DIR, 'temp')
DATA_CONFIG_PATH = abs_join(PBP_PATH, 'data_generator', 'configs')
# MAMAO_DATA_PATH = join(PROJECT_DIR, '..', 'fastamp-data')
MAMAO_DATA_PATH = abs_join(PROJECT_DIR, '..', 'fastamp-data-rss')

import warnings
warnings.filterwarnings('ignore')
