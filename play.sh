#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"

python scripts/rsl_rl/play.py \
    --task Isaac-Booster-K1-Wheelfoot-Blind-Flat-Play-v0 \
    --num_envs 64 \
    --checkpoint_path logs/rsl_rl/k1_wheelfoot_flat/2026-08-21_18-42-46_v0/model_15000.pt



