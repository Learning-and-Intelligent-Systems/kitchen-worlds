#!/usr/bin/env bash

base_dir="/svl/u/weiyul/Research/nsplan/kitchen-worlds"

# eval "$(conda shell.bash hook)"
# conda activate kitchen
# not needed
#export PYTHONPATH="/home/weiyu/Research/intern2/part_grounding/src:$PYTHONPATH"
#echo $PYTHONPATH

for si in {0..1}; do
  for ei in {0..1}; do
    echo "Run semantic spec ${si}, env ${ei}"
    python $base_dir/tests/collect_clean_dish_rollouts.py \
        --config_file $base_dir/configs/clean_dish_feg_collect_rollouts_cluster.yaml \
        --seed $ei \
        --semantic_spec_seed $si
  done
done