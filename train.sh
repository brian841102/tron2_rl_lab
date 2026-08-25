#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

python scripts/rsl_rl/train.py \
    --task Isaac-Booster-K1-Wheelfoot-Blind-Flat-v0 \
    --run_name v0 \
    --device cuda:0 \
    --headless \
#     --resume True \
#     --checkpoint_path logs/rsl_rl/k1_wheelfoot_flat/2026-08-21_18-42-46_v0/model_15000.pt \
#     --max_iterations 2000
