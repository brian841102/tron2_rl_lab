# Development-tool license notices

The `.txt` files in this directory are **verbatim license notices for
development-time tools only**. They exist so that anyone auditing the
repository can see the license terms of each linting / typing / testing
tool that a developer might install locally when hacking on `rsl_rl`.

## Classification

**None of these tools ship inside this repository or inside any
release artifact produced from it.** They are consumed only via
`pip install` at development time; the vendored `rsl_rl/` package
itself has no import of any of them, and installing the package
(`pip install -e rsl_rl`) does not pull them in.

Concretely, this directory contains license text for:

| File | Tool | Role |
|------|------|------|
| `black-license.txt`             | [`black`](https://pypi.org/project/black/)          | Formatter — dev-time |
| `codespell-license.txt`         | [`codespell`](https://pypi.org/project/codespell/)  | Spelling linter — dev-time. **License is GPL-2.0**; see note below. |
| `flake8-license.txt`            | [`flake8`](https://pypi.org/project/flake8/)        | Linter — dev-time |
| `isort-license.txt`             | [`isort`](https://pypi.org/project/isort/)          | Import sorter — dev-time |
| `numpy_license.txt`             | [`numpy`](https://pypi.org/project/numpy/)          | Runtime dep (rsl_rl imports numpy); attribution retained here as a courtesy — see `../../THIRD_PARTY_NOTICES.md` for the authoritative runtime-dep list |
| `onnx-license.txt`              | [`onnx`](https://pypi.org/project/onnx/)            | Optional (policy export); runtime-adjacent — see `../../THIRD_PARTY_NOTICES.md` |
| `pre-commit-hooks-license.txt`  | [`pre-commit-hooks`](https://pypi.org/project/pre-commit-hooks/) | Dev-time |
| `pre-commit-license.txt`        | [`pre-commit`](https://pypi.org/project/pre-commit/) | Dev-time |
| `pyright-license.txt`           | [`pyright`](https://pypi.org/project/pyright/)      | Type checker — dev-time |
| `pyupgrade-license.txt`         | [`pyupgrade`](https://pypi.org/project/pyupgrade/)  | Dev-time |
| `torch_license.txt`             | [`torch`](https://pytorch.org/)                     | Runtime dep — see `../../THIRD_PARTY_NOTICES.md` |

## Note on `codespell-license.txt` (GPL-2.0)

`codespell` is a **development-time spell checker**. Its GPL-2.0
license terms:

- Do **not** apply to any code in this repository.
- Do **not** apply to any release artifact built from this repository.
- Do **not** propagate to end users of `rsl_rl`, because `rsl_rl` neither
  imports nor bundles `codespell`.

The GPL-2.0 license text is retained here **only** as an audit-time
courtesy so that a reviewer can confirm the source of the notice
without having to fetch upstream `codespell`. This is a NOTICE, not a
license grant, and its presence does **not** subject the surrounding
BSD-3-Clause `rsl_rl` sources or the Apache-2.0 outer `tron2_rl_lab`
repository to GPL terms.

If `codespell` is ever added as a runtime dependency of `rsl_rl` (it
is not, today), this classification must be revisited before the next
release.

## Repository-level policy

The repository's CI includes a GPL-text scan (see
`.github/workflows/ci.yml`). That scan **allow-lists exactly this one
file** (`rsl_rl/licenses/dependencies/codespell-license.txt`) as a
dev-tool notice. Any additional GPL-licensed file anywhere else in the
tree will fail CI and must either be removed or given its own
`⚠ TO CONFIRM` classification in
[`THIRD_PARTY_NOTICES.md`](../../../THIRD_PARTY_NOTICES.md).

## Update procedure

Whenever a dev tool is added, removed, or its license changes:

1. Add / update / remove the corresponding `*-license.txt` file here.
2. Add / update / remove the corresponding row in the table above.
3. If the new tool is copyleft (GPL / AGPL / LGPL / SSPL / …), file a
   `⚠ TO CONFIRM` in `THIRD_PARTY_NOTICES.md` and update the CI
   allow-list; a copyleft dev tool is fine only if it is genuinely
   dev-only and never enters the runtime import graph.
