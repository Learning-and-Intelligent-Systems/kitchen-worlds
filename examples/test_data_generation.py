#!/usr/bin/env python

from __future__ import print_function

import config
from data_generator.data_generation_run import data_generation_process
from data_generator.run_utils import get_config_from_argparse, parallel_processing

#####################################

default_config_name, default_config_path, simulate = 'kitchen_full_feg.yaml', None, False
default_config_name, default_config_path, simulate = 'kitchen_full_pr2.yaml', None, False
# default_config_name, default_config_path, simulate = None, join(root, 'config_pigi.yaml'), False

config = get_config_from_argparse(default_config_name, default_config_path)
config.sim.simulate = simulate

#####################################


def process(index):
    """ exist a version in cognitive-architectures for generating mini-datasets (single process),
        run in kitchen-worlds for parallelization, but no reliable planning time data

        inside each data folder, to be generated:
        - before planning:
            [x] scene.lisdf
            [x] problem.pddl
            [x] planning_config.json
            [x] log.txt (generated before planning)
        - after planning:
            [x] plan.json
            [x] commands.pkl
            [x] log.json (updated by pddlstream)
    """
    data_generation_process(config)


if __name__ == '__main__':
    parallel_processing(process, range(config.n_data), parallel=config.parallel)
