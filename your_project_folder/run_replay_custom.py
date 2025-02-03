#!/usr/bin/env python

from __future__ import print_function

import sys
import argparse
from os.path import join, abspath, dirname, isdir, isfile
from os import listdir, pardir
RD = abspath(join(dirname(__file__), pardir))
sys.path.extend([join(RD), join(RD, 'pddlstream'), join(RD, 'pybullet_planning'), join(RD, 'lisdf')])

from config_custom import OUTPUT_PATH, DATA_CONFIG_PATH
from pigi_tools.replay_utils import run_replay, load_pigi_data

## change to whatever is printed at the end of planning
given_subpath = 'custom_piginet_data/241007_213032_original'
given_subpath = 'custom_pr2_kitchen_full/241007_233942'

parser = argparse.ArgumentParser()
parser.add_argument('-p', '--path', type=str, default=given_subpath)
parser.add_argument('--width', type=int, default=1440)
parser.add_argument('--height', type=int, default=1080)
parser.add_argument('--timestep', type=float, default=0.05)  ## increase this to slow down robot motion
args = parser.parse_args()

if __name__ == '__main__':

    run_replay(join(DATA_CONFIG_PATH, 'config_replay.yaml'), load_pigi_data,
               given_path=join(OUTPUT_PATH, args.path),
               save_mp4=True, time_step=args.timestep, width=args.width, height=args.height)
