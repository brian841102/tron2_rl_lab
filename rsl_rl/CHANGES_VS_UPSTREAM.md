# Changes vs upstream

This document records how the vendored `rsl_rl/` in this repository
differs from its upstream, per the licensing requirements of the
BSD-3-Clause license inherited from
[leggedrobotics/rsl_rl](https://github.com/leggedrobotics/rsl_rl).

## Upstream reference

| Field | Value |
|-------|-------|
| Upstream project | [leggedrobotics/rsl_rl](https://github.com/leggedrobotics/rsl_rl) |
| Upstream license | BSD-3-Clause |
| Fork base tag | **`v2.0.1`** |
| Fork base commit | `73fd7c621bf63104a8a7eb0c168df16c0ee65908` |
| Vendored version string | `2.0.2` (in `setup.py`), `2.0.1` (in `rsl_rl/__init__.py`) |
| Vendored license | BSD-3-Clause (unchanged from upstream) |

The `2.0.2` version string in `setup.py` is a **local** version bump
that has no corresponding upstream tag — upstream jumped from `v2.0.1`
directly to `v2.1.1`. Treat the vendored code as *derived from `v2.0.1`*
for license and attribution purposes.

Copyright notices (`Copyright 2021 ETH Zurich, NVIDIA CORPORATION`,
`SPDX-License-Identifier: BSD-3-Clause`) in individual files are
preserved as-is; no upstream copyright headers were removed.

## Summary of divergence

### Directory renames

| Upstream (`v2.0.1`) | Vendored | Notes |
|---------------------|----------|-------|
| `rsl_rl/algorithms/` | `rsl_rl/algorithm/` | Renamed to singular. Contents (`__init__.py`, `ppo.py`) match the upstream path structure. |
| `rsl_rl/runners/`    | `rsl_rl/runner/`    | Renamed to singular. Contents (`__init__.py`, `on_policy_runner.py`) match the upstream path structure. |

Downstream imports in this repository use the singular form
(`from rsl_rl.algorithm.ppo import …`, `from rsl_rl.runner.on_policy_runner import …`).

### Files removed from the vendored copy

| Upstream path | Reason |
|---------------|--------|
| `rsl_rl/utils/` (entire directory) | Utilities not needed by the biped-locomotion training pipeline. |
| `rsl_rl/modules/actor_critic_recurrent.py` | Recurrent variant unused by our configs. |
| `rsl_rl/modules/normalizer.py` | Replaced by the MLP-encoder normalisation baked into `rsl_rl/modules/mlp_encoder.py`. |

### Files added by LimX

| Vendored path | Purpose |
|---------------|---------|
| `rsl_rl/modules/mlp_encoder.py` | MLP-encoder architecture used by our proprioceptive-encoder training recipe. Not present upstream. |

### Files modified

Line counts below are `diff` output line counts against the same
upstream path at `v2.0.1` (or the pre-rename upstream path where the
directory was renamed). They are a coarse size indicator, not a
semantic measure.

| Vendored path | Diff-line count vs upstream `v2.0.1` |
|---------------|--------------------------------------|
| `rsl_rl/__init__.py` | small |
| `rsl_rl/env/__init__.py` | small |
| `rsl_rl/modules/__init__.py` | small (re-export list adjusted for removed / added modules) |
| `rsl_rl/modules/actor_critic.py` | modified |
| `rsl_rl/storage/__init__.py` | small |
| `rsl_rl/storage/rollout_storage.py` | modified |
| `rsl_rl/algorithm/ppo.py` | ~292 diff lines vs upstream `algorithms/ppo.py` |
| `rsl_rl/runner/on_policy_runner.py` | ~499 diff lines vs upstream `runners/on_policy_runner.py` |

## How to reproduce this diff

```bash
# 1. Fetch the upstream fork base.
git clone --branch v2.0.1 --depth 1 \
    https://github.com/leggedrobotics/rsl_rl.git /tmp/rsl_rl_v2.0.1

# 2. Structural summary.
diff -rq rsl_rl/rsl_rl /tmp/rsl_rl_v2.0.1/rsl_rl

# 3. Full per-file diff (rename-aware).
diff -ru \
    --exclude=__pycache__ \
    rsl_rl/rsl_rl /tmp/rsl_rl_v2.0.1/rsl_rl \
    > /tmp/rsl_rl-vs-v2.0.1.patch

# 4. Rename-aware inspection for the two singular/plural pairs:
diff -u /tmp/rsl_rl_v2.0.1/rsl_rl/algorithms/ppo.py             rsl_rl/rsl_rl/algorithm/ppo.py
diff -u /tmp/rsl_rl_v2.0.1/rsl_rl/runners/on_policy_runner.py   rsl_rl/rsl_rl/runner/on_policy_runner.py
```

## Update policy

Do **not** silently rebase the vendored copy onto a newer upstream tag.
Any upstream re-sync must:

1. Re-run the reproduce steps above at the new upstream tag and record
   the resulting patch in this file (append a new section).
2. Update the *Fork base tag* and *Fork base commit* rows above.
3. Bump the local `setup.py` version to a value that clearly encodes
   both the upstream base and the local suffix (e.g. `2.0.1+limx.1`).
4. Preserve every upstream copyright header. Do not remove BSD-3-Clause
   SPDX identifiers.
5. Verify `rsl_rl/licenses/` is still consistent with the new upstream
   tree (in particular the `codespell-license.txt` classification —
   see `../THIRD_PARTY_NOTICES.md`).

CI does **not** rebuild this diff automatically; a rebase without a
paired update to this file will be rejected in review (see
`.github/PULL_REQUEST_TEMPLATE.md`).
