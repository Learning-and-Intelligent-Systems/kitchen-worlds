from __future__ import print_function

from config_custom import DATA_CONFIG_PATH
from data_generator.data_generation_run import data_generation_process
from data_generator.run_utils import get_config_from_argparse, parallel_processing

config = get_config_from_argparse(default_config_name='config_generation.yaml', default_config_dir=DATA_CONFIG_PATH)


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
    """ output will be in outputs/custom_pr2_kitchen_full/{timestamped_run_dir} """
    parallel_processing(process, range(config.n_data), parallel=config.parallel)
