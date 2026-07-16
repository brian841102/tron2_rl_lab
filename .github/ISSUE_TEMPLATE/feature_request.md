---
name: Feature request
about: Suggest a new env, task, reward, or tooling improvement
title: "[feat] <short summary>"
labels: enhancement
assignees: ''
---

## Problem

<!-- What is missing or awkward in the current training stack? -->

## Proposed change

<!-- Which files / configs / task IDs you propose adding or editing. -->

## Alternatives considered

## Downstream impact

- New task IDs (if any):
- Reward / termination changes (safety-adjacent — see
  `CONTRIBUTING.md#reward--termination-changes-safety-adjacent`):
- Vendored `rsl_rl/` touched? If so, note the upstream sync
  implications:
- Bundled STL / USD assets added or replaced?

## Checklist

- [ ] This request does **not** require shipping trained weights
      (`*.pt`, `*.pth`, `*.ckpt`, `*.onnx`), training logs, or
      Weights & Biases artifacts.
- [ ] This request does **not** require shipping SDK binaries, vendor
      CAD, calibration values, firmware, or customer data.
- [ ] I have skimmed `CONTRIBUTING.md` for the env / vendored-fork
      conventions.
