#!/bin/bash

# 0923_constrained_placing_parallel
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 0 99 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 100 199 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 200 299 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 300 399 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 400 499 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 500 599 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 600 699 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 700 799 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 800 899 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh 900 999 0 9 clean_dish_feg_collect_rollouts_cluster_parallel_sink.yaml'

# 0923
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 0 99 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 100 199 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 200 299 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 300 399 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 400 499 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 500 599 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 600 699 0 10'

# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh'