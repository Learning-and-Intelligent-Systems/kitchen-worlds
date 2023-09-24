#!/usr/bin/env bash

base_dir="/svl/u/weiyul/Research/nsplan/kitchen-worlds"

# conda info
# export PYTHONPATH="/svl/u/weiyul/Research/nsplan/kitchen_worlds:$PYTHONPATH"
# export PYTHONPATH="/svl/u/weiyul/Research/nsplan/kitchen_worlds/pybullet_planning/pybullet_tools:$PYTHONPATH"
# echo $PYTHONPATH

# conda env is set up this way
cd /svl/u/weiyul/Research/nsplan/kitchen-worlds/tests
python collect_clean_dish_rollouts_cache_parallel.py \
    --config_file $base_dir/configs/clean_dish_feg_collect_rollouts_cluster_parallel.yaml \
