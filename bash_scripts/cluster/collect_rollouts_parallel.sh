#!/usr/bin/env bash

# Check if the number of arguments is 4
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <arg1> <arg2> <arg3> <arg4>"
    exit 1
fi

base_dir="/svl/u/weiyul/Research/nsplan/kitchen-worlds"
config_filename=$4
cd /svl/u/weiyul/Research/nsplan/kitchen-worlds/tests

# Calculate N
N=$(echo "($2 - $1) / 10" | bc)

# Check if N is an integer
if ! [[ "$N" =~ ^[0-9]+$ ]]; then
    echo "Error: N is not an integer."
    exit 1
fi

# Loop to call the python script
for ((i=0; i<N; i++)); do
    arg1=$(echo "$1 + 10 * $i" | bc)
    arg2=10
    echo "Calling script with semantic_spec_seed_start=$arg1 and num_semantic_spec_seed=$arg2 and num_seed=$3 and config_file=$base_dir/configs/$config_filename"
    python collect_clean_dish_rollouts_cache_parallel.py \
      --semantic_spec_seed_start $1 \
      --num_semantic_spec_seed $2 \
      --num_seed $3 \
      --config_file $base_dir/configs/$config_filename
done