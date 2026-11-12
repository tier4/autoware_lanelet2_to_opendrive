# Checks that pass while verifying nothing

A check is vacuous when its pass can be reached **even though the property is
false**. Every entry below was hit in practice on this project; each has a
cheap test you can run *before* believing the result.

The one line to keep:

> **Ask what a pass would look like if the property were false.**

If the answer is "exactly like this pass", you have not measured anything.

## A. The instrument cannot report what is being asked

| trap | what happened | test before believing |
| --- | --- | --- |
| A filter that excludes the very thing sought | A 1:1 seam filter removed forks and merges -- the topology the defect lived in. A probe that starts 5 cm before a road's end skipped every road shorter than that and reported "no change". | Inject one instance of what you are looking for and confirm the instrument returns it. |
| A signal that fires only on success | Waiting for `passed\|failed\|error` waits forever on a killed container, and looks like progress. | Would this wait print anything if the process died right now? |
| The wrapper's exit code read as the work's result | "completed (exit code 0)" over a run that wrote no output. | Does the output artifact exist, is it non-empty, does it close (`</OpenDRIVE>`)? |
| A stale artifact read as evidence of a later run | A log written 12 s before the container started was read as that container's failure. | Compare the artifact's mtime with the run's **start**, not with now. |
| A checker that was skipped or crashed | qc reported PASS with 5 of 26 checkers completed. | Read the `Checkers :` tally. Count issues, not `xml_location`s (one issue has several). |

## B. The comparison cannot come out any other way

| trap | what happened | test before believing |
| --- | --- | --- |
| An identity presented as a measurement | "The last geometry ends exactly at `road@length` on all 810 roads" -- `road@length` is computed *as* the sum of the geometry lengths. | Construct an input on which the two sides should differ. If you cannot, it is a tautology. |
| A "truth" that is the candidate restated | A reference defined through the candidate's own parametrisation scored the candidate at exactly 0 error everywhere. | Score the favoured candidate symbolically under the proposed truth, before measuring. |
| Docstring prose taken as a measurement | "22 violations, worst 0.071 m" was stale; the test beside it pinned 0. | Run the check. Read the constant the neighbouring test asserts. |

## C. The reference is real but cannot decide

| trap | what happened | test before believing |
| --- | --- | --- |
| Independent but too coarse | The reference's own uncertainty was ~400 µm against a ~4 µm effect. | Estimate the reference's uncertainty first and compare with the effect size. |
| A correlation where the predictor has no range | `r = -0.060` on a subset where the predictor was nearly constant; `+0.496` on the whole population. | Report the spread of the predictor next to every `r`. |
| Aggregates in different units | "52/52 mixed" by site and "52/52 constant" by namer were both true. | Say what one item is. |
| Medians agreeing | Two distributions with equal medians and no per-item relation. | Correlate per item, not per summary. |
| Three arc lengths | Polyline chord length, fitted-spline length and emitted `sum(geometry.length)` differ (up to 0.35 m on one road). | Say which `s` a figure is written against. |

## D. A positive has more than one cause

| trap | what happened | test before believing |
| --- | --- | --- |
| Naming the test after one cause | A "fails because of X" predicate also failed for Y, and a map reduction drifted onto Y. | List every cause that produces the positive before naming it; make the predicate ask for the same code *and* message. |
| A 1:1 reference asked a 1:many question | `<road><link>` names one predecessor, so at a convergence the comparison subtracted one merging road from the whole carriageway: 136 "lane drops", of which 1 was real. | Use contact groups, and report the component-size histogram (pairs vs three or more). |
| Complementary counts that sum exactly | Two categories that add up to the total can be one category with the sign flipped. | Check one member of each by hand. |
| A minimum over candidates | The best of N candidates always looks good. | Report N and the single-candidate control. |

## E. The zero belongs to the test data or to the procedure

| trap | what happened | test before believing |
| --- | --- | --- |
| Zero as a property of the map | 0 lane drops on Nishishinjuku: the map has none. | Does this map contain a structure that could produce a non-zero? |
| Only your own signal moved | The metric added with the fix improved; nothing independent did. | Did an existing quantity nobody wrote for this change move the corresponding way? |
| Ids looked up after preprocessing | Lanelets 112 and 114 "do not exist" -- preprocessing merged them into a new id. | Which stage's map are the ids from: the input or `.preprocessed.osm`? |
| Road ids matched across conversions | "597 of 693 roads changed" -- the ids had been reassigned, and unrelated roads were being compared. | Pair roads by their source lanelets, never by id. |
| The first run after a dependency change | It tested the previous venv and produced two failures pointing the wrong way. | Read the second run. |
| Two instructions that cannot both hold | A gate that needs condition A and a setup that forbids A, each verified separately. | Can both conditions be satisfied in the place where the thing actually runs? |
