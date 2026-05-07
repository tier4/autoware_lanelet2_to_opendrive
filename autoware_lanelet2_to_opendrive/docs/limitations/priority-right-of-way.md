# Priority-Based Right-of-Way Control Not Used by CARLA

## Issue

Priority tags (`priority` attribute on the `<junction>` element of the
OpenDRIVE 1.4 schema) for controlling intersection entry order are
**emitted by this converter but ignored by CARLA**.

## What the converter currently does

The converter exports `<junction><priority>` records derived from
Lanelet2 `right_of_way` regulatory elements (issue #438). For each
`right_of_way` element, the road carrying the *yielding* lanelets is
written as `low` and the road carrying the *right_of_way* lanelets as
`high`. The implementation lives in
[`opendrive/junction.py:Junction.build_priorities_from_regulatory_elements`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/junction.py)
and is wired into the main converter in `main.py`.

This means downstream tools that *do* consume `<junction><priority>`
(e.g. RoadRunner-style simulators, esmini, custom planners) get correct
right-of-way information.

## Why CARLA still ignores it

While the OpenDRIVE 1.4 schema defines `<junction><priority>`, CARLA's
parser and traffic manager simply do not consume it.

### CARLA's Complete Ignorance of Priority Attributes

CARLA simulator completely ignores priority information at every processing stage:

| Processing Stage | Result |
|------------------|--------|
| **① Parsing** | ❌ Priority attributes are not read from OpenDRIVE files |
| **② Data Storage** | ❌ No internal fields exist to store priority values |
| **③ Traffic Control** | ❌ Uses FIFO queue only for junction management; priority values are never consulted |

### Technical Analysis with Code References

**1. Parsing Stage: Priority Not Read**

CARLA's OpenDRIVE parser does not extract `priority` attributes from regulatory elements:

- **OpenDRIVE Parser**: [`LibCarla/source/carla/opendrive/parser/SignalParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp)
  - Only parses signal `type`, `subtype`, `value`, `orientation`, and `position`
  - **No code exists** to parse `priority` attributes from `<signal>` or `<controller>` elements

**2. Data Storage: No Priority Field**

CARLA's internal traffic signal representation lacks priority storage:

- **Signal Class**: [`LibCarla/source/carla/road/element/RoadInfoSignal.h`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/element/RoadInfoSignal.h)
  - Contains: `type`, `value`, `orientation`, `heading`, `pitch`, `roll`, `width`, `height`, `text`
  - **Does not contain**: `priority` field

- **Controller Class**: [`LibCarla/source/carla/road/SignalController.h`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/SignalController.h)
  - Contains: `signal_ids`, `junction_id`
  - **Does not contain**: `priority` field

**3. Traffic Control: FIFO Queue Only**

CARLA's Traffic Manager uses a simple FIFO (First-In-First-Out) queue for junction management:

- **Traffic Manager Junction Logic**: [`LibCarla/source/carla/trafficmanager/`](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/trafficmanager)
  - Junction entry order is determined **solely by arrival time**
  - **No priority-based decision making** exists in the codebase
  - All vehicles are treated equally regardless of any potential priority values

## Impact

When converting Lanelet2 maps with `right_of_way` regulatory elements:

- Priority information **is preserved** in the OpenDRIVE output as
  `<junction><priority>` records
- Intersection right-of-way behavior **will not match** the original
  Lanelet2 specification when running in CARLA — all vehicles follow
  FIFO queue behavior regardless of the emitted priorities
- Other OpenDRIVE consumers that read `<junction><priority>` will get
  the correct right-of-way information

## Workaround

If priority-based right-of-way control is needed in CARLA:

### Custom CARLA Source Modification

<details>
<summary><strong>Advanced: Modify CARLA source code</strong> (click to expand)</summary>

!!! warning "Out of Scope"
    The following information is provided for reference only. **Modifying CARLA's source code is not within the scope of this converter project**. This is a CARLA-side limitation that requires changes to the CARLA simulator itself.

If priority support is critical:

1. Fork CARLA repository
2. Modify `SignalParser.cpp` to parse priority attributes
3. Add `priority` field to `RoadInfoSignal` and `SignalController` classes
4. Implement priority-based logic in Traffic Manager
5. Rebuild CARLA from source

**Reference**: [CARLA Build Documentation](https://carla.readthedocs.io/en/latest/build_linux/)

!!! danger "CARLA Source Modification Required"
    Implementing priority-based right-of-way control requires **modifying CARLA's C++ source code**. This is a non-trivial undertaking requiring deep understanding of CARLA's architecture.

</details>

## Requesting Support

If CARLA priority support is important for your use case:

1. **Open a CARLA Issue**: [CARLA GitHub Issues](https://github.com/carla-simulator/carla/issues)
2. **Describe the use case**:
    - Why priority-based right-of-way is needed
    - Expected behavior at intersections
    - Real-world traffic scenarios requiring priority
3. **Propose implementation**:
    - Suggest changes to data structures
    - Outline Traffic Manager modifications
4. **Engage CARLA community**: Discuss with CARLA maintainers

!!! info "Future Support"
    If CARLA adds priority attribute support in the future, this converter can be updated to include priority information in the generated OpenDRIVE files.

---

[← Back to Limitations Overview](index.md)
