---
name: red-green
description: Test-first procedure for bug fixes and features in this repository -- write the test, prove it fails for the right reason with scripts/tdd.py, implement, prove it passes, and record both in the commits and the PR. Use before writing the test for any fix or feature; not for pure refactorings.
---

# Red/green for a bug fix or feature

The team convention (cycle, commit shape, choosing a map) is in
`autoware_lanelet2_to_opendrive/docs/development.md`, section "Red/green
development". This skill is how to carry it out without fooling yourself.

## 0. Decide whether there is a red step

- **Bug fix or feature**: yes. Continue.
- **Refactoring** (no behaviour change): no. Verify unchanged output instead
  (e.g. before/after `.xodr` diff, ignoring `<header date>`), say so in the PR,
  and stop here.

## 1. Write the test first

- State the **missing behaviour** as an assertion, not the implementation.
- Put the thing under test behind an import that already exists, or import it
  inside the test body. A module-level `ImportError` of a name you have not
  written yet is a collection error, and `tdd.py red` rejects it.
- Prefer a small map. A test that converts `nishishinjuku.osm` costs about 160 s
  per run; a `*_mini.osm` about 5 s. If the defect only shows on
  Nishishinjuku, say so in the PR.

## 2. Prove the red

```bash
python scripts/tdd.py red <node id>
```

Only `RED` counts. Then **read every failure reason it prints**:

| reason looks like | meaning | what to do |
| --- | --- | --- |
| an `assert` on the behaviour | a genuine red | continue |
| `ImportError` / `AttributeError` / `NameError` for the thing under test | only the name is missing | give it a stub that exists and returns the wrong thing, rerun |
| `NotImplementedError` from your own stub | acceptable only if the assertion would otherwise reach the behaviour | prefer a stub that returns a wrong value |
| an error in a fixture or in setup | `NOT RED` -- the test never reached its assertion | fix the test |

If the verdict is `NOT RED`, **fix the test, not the verdict**:

- `passes already` -- the test does not constrain the change. Tighten it.
- `skipped` -- a skip condition fired; the test proves nothing until it runs.
- `usage error` / `no tests were collected` -- the node id is wrong.
- `interrupted` -- collection failed (usually an import at module level).

Never use `--testmon` for this: it deselects tests and turns a red into
"nothing ran".

## 3. Commit the red

```python
@pytest.mark.xfail(strict=True, reason="#<issue>")
def test_...():
```

Commit as `test: ...`. The suite stays green; `tdd.py red` still checks the
test as failing because it passes `--runxfail`. Run it once more after adding
the marker to confirm.

## 4. Implement, then prove the green

Remove the marker in the same change as the implementation.

```bash
python scripts/tdd.py green <node id>
```

Only `GREEN` counts. `NOT GREEN: ... [XPASS(strict)]` means the marker is still
there. Then run the full suite:

```bash
docker compose --profile test run --rm pytest
```

Commit as `fix: ...` / `feat: ...`.

## 5. Put the evidence in the PR

Fill the template's "Red/green evidence" block with both verdicts as printed,
including the failure reasons under `RED`. If the red needed Nishishinjuku, or
a stub, say which and why.
