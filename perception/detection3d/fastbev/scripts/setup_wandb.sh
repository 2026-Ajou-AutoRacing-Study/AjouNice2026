#!/usr/bin/env bash
set -euo pipefail

CONDA_ENV="${CONDA_ENV:-fastbev_rtx50}"

source "$HOME/miniconda3/etc/profile.d/conda.sh"
set +u
conda activate "$CONDA_ENV"
set -u

python -m pip install 'wandb==0.28.2'
python -c 'import wandb; print("wandb ready:", wandb.__version__)'

printf '\nRun `wandb login` once on this machine before online training.\n'
printf 'The API key must not be stored in this repository.\n'
