from os.path import join, isdir, isfile, dirname, getmtime

import config

from pigi_tools.data_utils import  get_worlds_aabb

from test_utils import process_all_tasks

from data_generator.run_utils import copy_dir_for_process, get_data_processing_parser
from data_generator.image_generation import generate_segmented_images, generate_color_images

DATASET_ROOT = join(dirname(__file__), '..', '..', 'fastamp-data-rss')
DATASET_ROOT = join(dirname(__file__), '..', 'outputs')

DEFAULT_TASK = 'test_feg_kitchen_full'
GIVEN_PATH = None
PARALLEL = False and (GIVEN_PATH is None)
USE_VIEWER = True
GENERATE_SEG = False  ## generate RGB only

N_PX = 224
REDO = True
MODIFIED_TIME = 1675535195
LARGER_WORLD = 'mm_' in DEFAULT_TASK or 'tt_' in DEFAULT_TASK
NEW_KEY = 'channel7'
ACCEPTED_KEYS = [NEW_KEY, 'crop_fix', 'rgb', 'meraki']


parser = get_data_processing_parser(task_name=DEFAULT_TASK, parallel=PARALLEL, use_viewer=USE_VIEWER)
parser.add_argument('--seg', action='store_true', default=GENERATE_SEG)
parser.add_argument('--redo', action='store_true', default=REDO)
parser.add_argument('--larger_world', action='store_true', default=LARGER_WORLD)
parser.add_argument('--modified_time', type=int, default=MODIFIED_TIME)
parser.add_argument('--crop_px', type=int, default=N_PX)
parser.add_argument('--new_key', type=str, default=NEW_KEY)
parser.add_argument('--accepted_keys', type=list, default=ACCEPTED_KEYS)
args = parser.parse_args()


def process_worlds_aabb():
    """ bounds ((-0.879, -2.56, -0.002), (1.15, 9.477, 2.841)) """
    run_dirs = process_all_tasks(None, args.t, DATASET_ROOT, parallel=False, return_dirs=True, input_args=args)
    aabb = get_worlds_aabb(run_dirs)


if __name__ == "__main__":
    kwargs = dict(task_name=args.t, dataset_root=DATASET_ROOT, parallel=args.p, path=GIVEN_PATH, input_args=args)
    process_all_tasks(generate_segmented_images, **kwargs)

    # process_worlds_aabb()
