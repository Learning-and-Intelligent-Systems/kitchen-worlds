#!/usr/bin/env python

from __future__ import print_function
from os.path import join

from config import PBP_PATH
from data_generator.run_utils import get_config_file_from_argparse, process_all_tasks
from pigi_tools.replay_utils import load_replay_conf, run_one, case_filter

REPLAY_CONFIG_PATH = join(PBP_PATH, 'pigi_tools', 'config')
DEFAULT_CONFIG_NAME = 'replay_rss.yaml'
DEFAULT_CONFIG_PATH = None

## or replace with your path to config yaml file
DEFAULT_CONFIG_NAME = None
DEFAULT_CONFIG_PATH = join(REPLAY_CONFIG_PATH, DEFAULT_CONFIG_NAME)

config_file = get_config_file_from_argparse(default_config_name=DEFAULT_CONFIG_NAME,
                                            default_config_path=DEFAULT_CONFIG_PATH,
                                            default_config_dir=REPLAY_CONFIG_PATH)


def run_replay(config_yaml_file, load_data_fn):
    c = load_replay_conf(config_yaml_file)

    def process(run_dir_ori):
        return run_one(run_dir_ori, load_data_fn=load_data_fn, **c)

    def _case_filter(run_dir_ori):
        case_kwargs = dict(given_path=c['given_path'], cases=c['cases'], check_collisions=c['check_collisions'],
                           save_jpg=c['save_jpg'], save_gif=c['save_gif'],
                           skip_if_processed_recently=c['skip_if_processed_recently'], check_time=c['check_time'])
        return case_filter(run_dir_ori, **case_kwargs)

    process_all_tasks(process, c['task_name'], parallel=c['parallel'], cases=c['cases'],
                      path=c['given_path'], dir=c['given_dir'], case_filter=_case_filter)


if __name__ == '__main__':
    from pigi_tools.replay_utils import load_pigi_data_complex

    run_replay(config_file, load_pigi_data_complex)

    # replay_all_in_gym(num_rows=14, num_cols=14, world_size=(6, 6), save_gif=True)

    ## ------------- record 1 : 250+ worlds
    # replay_all_in_gym(num_rows=2, num_cols=1, world_size=(4, 8), loading_effect=False,
    #                   frame_gap=1, save_mp4=True, save_gif=False, verbose=False, camera_motion='zoom')
    # replay_all_in_gym(num_rows=32, num_cols=8, world_size=(4, 8), loading_effect=True,
    #                   frame_gap=1, save_mp4=True, save_gif=False, verbose=False, camera_motion='zoom')

    ## ------------- record 1 : 96+ worlds
    # replay_all_in_gym(num_rows=32, num_cols=8, world_size=(4, 8), loading_effect=False,
    #                   frame_gap=1, save_mp4=True, save_gif=False, verbose=False, camera_motion='pan')

    ## ------------- record 2 : robot execution
    # replay_all_in_gym(num_rows=8, num_cols=3, world_size=(4, 8), loading_effect=False,
    #                   frame_gap=2, save_mp4=True, save_gif=False, verbose=False, camera_motion='spotlight')
