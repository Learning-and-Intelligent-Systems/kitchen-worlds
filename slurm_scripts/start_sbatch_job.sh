#!/bin/bash
#SBATCH --partition=svl
#SBATCH --cpus-per-task=8
#SBATCH --nodes=1
#SBATCH --mem=32G
#SBATCH --account=viscam

# only use the following on partition with GPUs
#SBATCH --gres=gpu:1

#SBATCH --job-name=ns_plan
#SBATCH --output=/svl/u/weiyul/test_output/%j.out
#SBATCH --error=/svl/u/weiyul/test_output/%j.err


# list out some useful information (optional)
echo "SLURM_JOBID="$SLURM_JOBID
echo "SLURM_JOB_NODELIST"=$SLURM_JOB_NODELIST
echo "SLURM_NNODES"=$SLURM_NNODES
echo "SLURMTMPDIR="$SLURMTMPDIR
echo "working directory = "$SLURM_SUBMIT_DIR

# sample process (list hostnames of the nodes you've requested)
NPROCS=`srun --nodes=${SLURM_NNODES} bash -c 'hostname' |wc -l`
echo NPROCS=$NPROCS

# can try the following to list out which GPU you have access to
#srun /usr/local/cuda/samples/1_Utilities/deviceQuery/deviceQuery

source /sailhome/weiyul/.bashrc
conda activate kitchen

export PYTHONPATH="/svl/u/weiyul/Research/nsplan/nsplan/src:$PYTHONPATH"

cd /svl/u/weiyul/Research/nsplan/kitchen-worlds
$1

# done
echo "Done"
