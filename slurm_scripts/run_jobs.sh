#!/bin/bash

sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 0 199 0 10'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 200 399 0 10'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 400 599 0 10'
sbatch start_sbatch_job.sh 'bash bash_scripts/cluster/collect_rollouts.sh 600 799 0 10'
