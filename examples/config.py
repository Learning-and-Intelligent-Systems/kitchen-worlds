import sys
from os.path import join, abspath, dirname, isdir, isfile

PROJECT_DIR = abspath(join(dirname(__file__), '..'))
PBP_PATH = join(PROJECT_DIR, 'pybullet_planning')

sys.path.append(PROJECT_DIR)
sys.path.append(PBP_PATH)
sys.path.append(join(PROJECT_DIR, 'lisdf'))
sys.path.append(join(PROJECT_DIR, 'pddlstream'))

ASSET_PATH = join(PROJECT_DIR, 'assets')
EXP_PATH = join(PROJECT_DIR, 'test_cases')
OUTPUT_PATH = join(PROJECT_DIR, 'outputs')
DATA_CONFIG_PATH = join(PBP_PATH, 'data_generator', 'configs')
# MAMAO_DATA_PATH = join(PROJECT_DIR, '..', 'fastamp-data')
MAMAO_DATA_PATH = join(PROJECT_DIR, '..', 'fastamp-data-rss')

import warnings
warnings.filterwarnings('ignore')
