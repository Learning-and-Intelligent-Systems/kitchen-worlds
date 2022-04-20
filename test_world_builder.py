import os
from os.path import join, isfile
import sys
sys.path.append(join('pybullet_planning'))

from pybullet_tools.utils import set_random_seed



import argparse

def get_parser():
    parser = argparse.ArgumentParser()
    ## -------- simulation related
    parser.add_argument('-v', '--viewer', action='store_true', help='')
    parser.add_argument('-d', '--drive', action='store_true', help='')
    parser.add_argument('-t', '--time_step', type=float, default=4e-0)
    parser.add_argument('--teleport', action='store_true', help='')
    parser.add_argument('-s', '--seed', type=int, default=None, help='')
    parser.add_argument('-cam', '--camera', action='store_true', default=True, help='')
    parser.add_argument('-seg', '--segment', action='store_true', help='')
    parser.add_argument('-mon', '--monitoring', action='store_true', default=False)

    args = parser.parse_args()  # TODO: flag to save a video
    set_random_seed(args.seed)
    return args

def create_pybullet_world():
    pass

def build_world_from_shapes():
    pass

def build_world_from_floorplan():
    pass

if __name__ == '__main__':
    world = create_pybullet_world()
    build_world_from_shapes()
    # build_world_from_floorplan()
