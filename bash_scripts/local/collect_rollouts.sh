#!/usr/bin/env bash
# eval "$(conda shell.bash hook)"
# conda activate kitchen
# not needed
#export PYTHONPATH="/home/weiyu/Research/intern2/part_grounding/src:$PYTHONPATH"
#echo $PYTHONPATH

cd /home/weiyu/Research/nsplan/original/kitchen-worlds/tests
export PYTHONPATH="/home/weiyu/Research/nsplan/nsplan/src:$PYTHONPATH"

# export QT_PLUGIN_PATH=/home/weiyu/anaconda3/envs/kitchen/lib/python3.8/site-packages/cv2/qt/plugins

for si in {0..10}; do
  for ei in {0..10}; do
    echo "Run semantic spec ${si}, env ${ei}"
    python /home/weiyu/Research/nsplan/original/kitchen-worlds/tests/collect_clean_dish_rollouts_cache.py \
        --config_file /home/weiyu/Research/nsplan/original/kitchen-worlds/configs/clean_dish_feg_collect_rollouts.yaml \
        --seed $ei \
        --semantic_spec_seed $si
  done
done