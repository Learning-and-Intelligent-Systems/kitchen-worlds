#!/usr/bin/env bash
eval "$(conda shell.bash hook)"
conda activate kitchen
# not needed
export PYTHONPATH="/home/weiyu/Research/nsplan/nsplan/src:$PYTHONPATH"
echo $PYTHONPATH

export QT_QPA_PLATFORM=offscreen

cd /home/weiyu/Research/nsplan/kitchen-worlds/tests

for si in {800..809}; do
  for ei in {0..4}; do
    echo "Run semantic spec ${si}, env ${ei}"
    python evaluate_model_on_clean_dish.py \
        --config_file /home/weiyu/Research/nsplan/kitchen-worlds/configs/clean_dish_feg_collect_rollouts.yaml \
        --seed $ei \
        --semantic_spec_seed $si
  done
done