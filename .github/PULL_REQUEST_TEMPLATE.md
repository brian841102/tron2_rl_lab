<!--
Thanks for contributing to tron2_rl_lab!
Please fill in the sections below. Delete any that are not applicable.
-->

## Summary

<!-- One paragraph: what and why. -->

## Type of change

- [ ] `fix`     — corrects a defect in env / MDP / actuator / cfg
- [ ] `feat`    — new task, new reward term, or new capability
- [ ] `asset`   — bundled USD / STL update
- [ ] `docs`    — README, THIRD_PARTY_NOTICES, CONTRIBUTING
- [ ] `ci`      — GitHub Actions or verification tooling
- [ ] `chore`   — repo maintenance (deps, formatting, cleanup)
- [ ] `rsl_rl`  — vendored fork sync or fix (see policy below)

## Affected components

- [ ] `exts/bipedal_locomotion/`
- [ ] `rsl_rl/` (vendored fork)
- [ ] `scripts/rsl_rl/`
- [ ] Bundled assets (`assets/usd/…`)
- [ ] Docs / meta

## Affected task IDs

<!-- e.g. Isaac-Limx-SF-TRON2A-Blind-Flat-v0 -->

- Training:
- Play:

## Verification

Paste the output (or a summary) of the local verification steps from
`CONTRIBUTING.md#verification-before-opening-a-pr`:

```text
python -m py_compile: ...
ruff check: ...
pip install --dry-run: ...
training-artifact scan: ...
license / TODO scan: ...
```

If this PR changes reward / termination / action-manager code
(safety-adjacent — see
`CONTRIBUTING.md#reward--termination-changes-safety-adjacent`), attach
a `play.py` run of the affected task or a reward-curve plot showing
the intended effect.

## Vendored `rsl_rl` (if touched)

<!-- Fill this section only if any file under rsl_rl/ changed. -->

- [ ] This is an explicit upstream sync or bug fix, **not** a silent
      rebase.
- [ ] `rsl_rl/CHANGES_VS_UPSTREAM.md` is updated with:
  - upstream ref before the sync,
  - upstream ref after the sync,
  - summary of the LimX-side diff carried across the sync.
- [ ] `THIRD_PARTY_NOTICES.md` §2 is updated if the upstream URL,
      license, or fork commit changed.
- [ ] Every touched source file still carries the ETH Zurich / NVIDIA
      BSD-3-Clause copyright header.

## Excluded artifacts

- [ ] This PR does **not** add any training artifact: no checkpoints,
      no logs, no wandb runs, no `events.out.tfevents.*`, no
      `*.onnx` / `*.pt` / `*.pth` / `*.ckpt`.
- [ ] This PR does **not** add SDK binaries (`.so`, `.dll`, `.dylib`,
      `.lib`), vendor CAD, calibration values, firmware, or customer
      data.

## Provenance & licensing

<!-- Required if the PR touches bundled assets or third-party code. -->

- [ ] All new / modified STL / USD files are LimX-authored or already
      cleared for redistribution.
- [ ] STL headers and USD `customLayerData` / `comment` / `author`
      fields contain no internal paths, usernames, or serial numbers.
- [ ] `doc/*.gif` (if changed) have been metadata-stripped and
      reviewed for identifiable individuals / office / non-public
      products.
- [ ] `THIRD_PARTY_NOTICES.md` is up to date.

## Checklist

- [ ] `CHANGELOG.md` has an entry under `## [Unreleased]`.
- [ ] All commits are DCO-signed (`git commit -s`).
- [ ] CI is expected to pass.

## Related issues

<!-- Fixes #123 / Refs #456 -->
