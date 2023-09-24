#!/bin/bash

sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 0 99 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 100 199 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 200 299 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 300 399 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 400 499 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 500 599 0 10'
# sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 600 699 0 10'

sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts_parallel.sh'