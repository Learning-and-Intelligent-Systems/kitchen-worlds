#!/usr/bin/env bash
eval "$(conda shell.bash hook)"
conda activate kitchen
# not needed
export PYTHONPATH="/home/weiyu/Research/nsplan/nsplan/src:$PYTHONPATH"
echo $PYTHONPATH

export QT_QPA_PLATFORM=offscreen

cd /home/weiyu/Research/nsplan/original/kitchen-worlds/tests

for si in {0..20}; do
  for ei in {20..25}; do
    echo "Run semantic spec ${si}, env ${ei}"
    python evaluate_model_on_clean_dish_bc.py \
        --config_file /home/weiyu/Research/nsplan/original/kitchen-worlds/configs/evaluate_clean_dish_feg_collect_rollouts_0828.yaml \
        --seed $ei \
        --semantic_spec_seed $si
  done
done