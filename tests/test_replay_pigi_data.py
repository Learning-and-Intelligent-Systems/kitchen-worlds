#!/usr/bin/env python

from __future__ import print_function
from os.path import join
from world_builder.paths import pbp_path
from pigi_tools.replay_utils import run_replay

CONFIG_YAML_PATH = join(pbp_path, 'pigi_tools', 'config', 'replay_rss.yaml')


if __name__ == '__main__':
    from pigi_tools.replay_utils import load_pigi_data_complex

    run_replay(CONFIG_YAML_PATH, load_pigi_data_complex)

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
