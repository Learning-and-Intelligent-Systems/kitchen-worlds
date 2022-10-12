import sys
from os.path import join, abspath, dirname, isdir, isfile
sys.path.append(join('..'))
sys.path.append(join('..', 'lisdf'))
sys.path.append(join('..', 'pddlstream'))
sys.path.append(join('..', 'pybullet_planning'))
sys.path.append(join('..', 'pybullet_planning', 'fastamp'))

import warnings
warnings.filterwarnings('ignore')

ASSET_PATH = join(dirname(__file__), '..', 'assets')
EXP_PATH = join(dirname(__file__), '..', 'test_cases')
OUTPUT_PATH = join(dirname(__file__), '..', 'outputs')
MAMAO_DATA_PATH = join(dirname(__file__), '..', '..', 'fastamp-data')