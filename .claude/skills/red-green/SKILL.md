---
name: red-green
description: Test-first procedure for bug fixes and features in this repository -- write the test, prove it fails for the right reason with scripts/tdd.py, implement, prove it passes, and put both verdicts in the PR. Use before writing the test for any fix or feature; not for pure refactorings.
---

# Red/green for a bug fix or feature

The team convention (the cycle, the evidence a PR carries, choosing a map) is
in `autoware_lanelet2_to_opendrive/docs/development.md`, section "Red/green
development", and what `tdd.py` accepts and rejects is in `docs/docker.md`,
section "Red/green runs for test-driven development". This skill is how to
carry them out without fooling yourself; it restates neither.

Run `tdd.py` with the host's `python`, never `uv run python`. If Docker is not
available, say so: there is no red or green to check, and verification is left
to CI.

## 0. Decide whether there is a red step

- **Bug fix or feature**: yes. Continue.
- **Refactoring** (no behaviour change): no. Verify unchanged output instead,
  write `none (refactoring)` in the PR's evidence block with how you verified
  it, and stop here.

## 1. Write the test first

- State the **missing behaviour** as an assertion, not the implementation.
- Put the thing under test behind an import that already exists, or import it
  inside the test body. A module-level `ImportError` of a name you have not
  written yet is a collection error, and `tdd.py red` rejects it.
- Choose the map as `development.md` describes. If the defect only shows on
  Nishishinjuku, say so in the PR.

## 2. Prove the red

```bash
python scripts/tdd.py red <node id>
```

Only `RED` counts. Then **read every failure reason it prints**. `RED` means
every selected test failed inside its body; it does not say why:

| reason looks like | meaning | what to do |
| --- | --- | --- |
| an `assert` on the behaviour | a genuine red | continue |
| `ImportError` / `AttributeError` / `NameError` for the thing under test | only the name is missing | add a stub that exists and returns a wrong value, rerun |
| `NotImplementedError` from your own stub | only the stub is reached, not the behaviour | make the stub return a wrong value instead of raising, rerun |
| any other exception (`TypeError`, `KeyError`, `CalledProcessError`, ...) | either the defect itself or a mistake in the test | decide which; if it is the defect, say so in the PR |

An error in a fixture or in setup never appears under `RED`: `tdd.py` reports
it as `NOT RED`.

If the verdict is `NOT RED`, **fix the test, not the verdict**:

- `passes already` -- the test does not constrain the change. Tighten it.
- `error outside the test body` -- a fixture or setup failed before the
  assertion.
- `skipped` -- a skip condition fired; the test proves nothing until it runs.
- `interrupted` -- collection failed (usually an import at module level).
- `usage error` / `no tests were collected` -- the node id is wrong, or `-k` /
  `-m` deselected every test.
- `no tests ran` / `internal pytest error` -- read the pytest output above the
  verdict.

Never pass `--testmon`: it deselects tests and turns a red into "nothing ran".

## 3. Implement, then prove the green

```bash
python scripts/tdd.py green <node id>
```

Only `GREEN` counts. Then run the full suite with the bounded worker count
`development.md` gives, and stop the resident container:

```bash
python scripts/tdd.py stop
```

## 4. Put the evidence in the PR

Fill the template's "Red/green evidence" block with both verdicts as printed,
including the failure reasons under `RED`. It is the only record of the red
that survives the squash merge. If the red needed Nishishinjuku, or a stub,
say which and why.
