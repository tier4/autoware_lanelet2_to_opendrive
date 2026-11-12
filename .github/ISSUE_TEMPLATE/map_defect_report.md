---
name: 🗺️ Map-Specific Defect (map cannot be shared)
about: Report a conversion defect seen on a map you may not upload
title: "[MAP] "
labels: bug
assignees: ""
---

<!--
Use this when the defect shows on a map you cannot attach. Do NOT paste
element ids, coordinates, file paths, screenshots or excerpts of that map.

`python scripts/map_issue.py draft` writes this whole body for you from a
fingerprint, and refuses to if its leak check fails. The procedure is in
.claude/skills/map-issue-report/SKILL.md.
-->

## 🐛 Finding

<!-- One `instrument:code` from the fingerprint table, e.g.
`invariant:linked_endpoints_apart`. One issue per finding. -->

---

## 📊 Fingerprint of the source map (L0: codes and counts only)

<!-- Paste from public/issue.md, or fill in by hand. -->

- **Converter**: <!-- version and commit -->
- **Overrides**: <!-- e.g. `map=junction_cluster target=carla`; withhold path values -->
- **Conversion**: <!-- completed / did not complete, seconds, whether preprocessing ran -->
- **Map**: <!-- lanelet count by subtype; no name, no location -->
- **qc checkers**: <!-- total / completed / skipped / errored. An error count is
  a lower bound unless every checker completed. -->

| finding | count | example (ids and numbers withheld) |
|---|---:|---|
|  |  |  |

---

## 🔁 Reproducer

<!-- Either attach `repro.osm` (a synthetic rebuild: renumbered, relocated,
rotated, perturbed) with the exact command and the findings on it, or say
"L0 only" and why no reproducer could be made:
  - the finding needs the map's preprocessing,
  - it did not survive reduction, or
  - it did not survive the rebuild.
Each of those tells the maintainer something. -->

```bash
convert input_map_path=repro.osm map=junction_cluster
```

---

## ✅ Expected / ❌ Actual

<!-- What the output should have been, and what it was. -->

---

## 🌍 Environment

- **OS / container**: <!-- e.g. the repository's Docker image, Ubuntu 22.04 -->
- **Consumer, if the defect shows there**: <!-- e.g. CARLA 0.10.0 `carla.Map()` -->

---

## ✅ Checklist

- [ ] `python scripts/map_issue.py leak-check` passed on everything attached or pasted here
- [ ] No element id, coordinate, file path or screenshot of the source map appears in this issue
- [ ] The owner of the source map agreed to publishing the fingerprint, and separately to any attached `repro.osm`
- [ ] I have searched for an existing issue with the same finding code
