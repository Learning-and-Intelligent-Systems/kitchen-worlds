#!/usr/bin/env bash

semantic_spec_seed_start=$1
semantic_spec_seed_end=$2
seed_start=$3
seed_end=$4

base_dir="/svl/u/weiyul/Research/nsplan/kitchen-worlds"

# conda info
# export PYTHONPATH="/svl/u/weiyul/Research/nsplan/kitchen_worlds:$PYTHONPATH"
# export PYTHONPATH="/svl/u/weiyul/Research/nsplan/kitchen_worlds/pybullet_planning/pybullet_tools:$PYTHONPATH"
# echo $PYTHONPATH

# conda env is set up this way
cd /svl/u/weiyul/Research/nsplan/kitchen-worlds/tests
# pwd
for si in $(eval echo "{$semantic_spec_seed_start..$semantic_spec_seed_end}"); do
  for ei in $(eval echo "{$seed_start..$seed_end}"); do
    echo "Run semantic spec ${si}, env ${ei}"
    python collect_clean_dish_rollouts.py \
        --config_file $base_dir/configs/clean_dish_feg_collect_rollouts_cluster.yaml \
        --seed $ei \
        --semantic_spec_seed $si
  done
done
