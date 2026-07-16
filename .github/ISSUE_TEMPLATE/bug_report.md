---
name: Bug report
about: Environment / MDP / actuator / robot cfg / vendored rsl_rl defect
title: "[bug] <short summary>"
labels: bug
assignees: ''
---

## Affected component

- [ ] `exts/bipedal_locomotion/` — env / MDP / actuator / robot cfg
- [ ] `rsl_rl/` — vendored PPO trainer / on-policy runner
- [ ] `scripts/rsl_rl/` — `train.py` / `play.py`
- [ ] Bundled asset (`assets/usd/…`)
- [ ] Documentation / README

## Task ID(s)

<!-- e.g. Isaac-Limx-SF-TRON2A-Blind-Flat-v0 -->

- Training task id:
- Play task id:

## Environment

- Isaac Sim version (must be 5.1 for the current release):
- Isaac Lab version (must be 2.3.1 for the current release):
- Python version:
- GPU + driver:
- OS + arch:
- Commit / tag:

## Expected behavior

<!-- What the training / play run should do, or what the cfg should describe. -->

## Actual behavior

<!-- Observed behavior: log lines, screenshots, reward curves, error tracebacks. -->

## Minimal reproduction

```bash
# Commands that reproduce the issue starting from a fresh clone.
```

## Additional context

<!-- Cross-links to related issues, upstream Isaac Lab / rsl_rl bugs, etc. -->

## Checklist

- [ ] I have searched existing issues.
- [ ] I have included the exact commit / tag.
- [ ] I am **not** attaching a trained checkpoint / `.pt` / `.onnx`
      to this issue.
- [ ] I am **not** reporting a security issue (those go to
      `contact@limxdynamics.com` per `SECURITY.md`).
- [ ] I am **not** reporting a hardware / real-robot safety incident
      (those go to LimX product support, not this repo).
