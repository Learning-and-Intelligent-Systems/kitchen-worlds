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


if __name__ == '__main__':

    debug_kwargs = dict(width=1440, height=1080)

    run_replay(join(DATA_CONFIG_PATH, 'config_replay.yaml'), load_pigi_data,
               given_path=join(OUTPUT_PATH, parser.parse_args().path), save_mp4=True, **debug_kwargs)
