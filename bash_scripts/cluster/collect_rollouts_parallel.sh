#!/usr/bin/env bash

base_dir="/svl/u/weiyul/Research/nsplan/kitchen-worlds"
config_filename=$5

cd /svl/u/weiyul/Research/nsplan/kitchen-worlds/tests
python collect_clean_dish_rollouts_cache_parallel.py \
    --config_file $base_dir/configs/$config_filename \
    --semantic_spec_seed_start $1 \
    --semantic_spec_seed_end $2 \
    --seed_start $3 \
    --seed_end $4
