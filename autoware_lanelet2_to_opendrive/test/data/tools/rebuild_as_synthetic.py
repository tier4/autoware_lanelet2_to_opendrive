"""Rebuild a Lanelet2 map excerpt as a synthetic fixture.

An excerpt cut out of a real map carries that map's node ids, its
coordinates and its authoring metadata.  A fixture in a public repository
should carry none of those, so this rewrites the excerpt into a map that
keeps what a test needs -- the *topology* and the *shape* of the road
network -- and drops what identifies it:

* every node, way and relation is renumbered from a fresh counter, in a
  deterministic order, so no id survives;
* every coordinate goes through a rigid transform (rotation about the
  excerpt's centroid, then placement at a new centre) and is then perturbed,
  so neither the position nor the surveyed geometry survives;
* the authoring metadata (``MetaInfo``, ``generator``) is dropped, and so is
  every tag whose value is an id in the source map -- by name and count, so
  the drop is never silent. Every other tag passes through unchanged: the
  fixture has to behave exactly like the excerpt, and an allowlist would
  quietly lose any tag the converter learns to read later.

What this deliberately does **not** change is the *character* of the
geometry: the irregular vertex spacing, boundaries that are coarse in one
place and dense in another, and fork branches that nearly but do not exactly
meet.  Those are the properties a hand-drawn fixture cannot reproduce, and
they are the whole reason a real-derived fixture is worth having.  The
perturbation is sized to leave them intact while making the coordinates no
longer the surveyed ones -- see :class:`Perturbation` for how its two
components divide that work, and ``data/README.md`` for what was measured
before and after.

The network's *layout* necessarily survives: a perturbation large enough to
change which road meets which would destroy the thing the fixture is for.
What does not survive is any particular coordinate.

Usage::

    python rebuild_as_synthetic.py excerpt.osm synthetic.osm \\
        --rotate 37 --to-lat 35.0 --to-lon 139.0 \\
        --warp 0.25 --wavelength 150 --jitter 0.02 --seed 20260920

``--to-lat`` defaults to the source centroid's latitude, which makes the
transform exactly rigid in the converter's projected frame: the
metres-per-degree-of-longitude scale is a function of latitude alone, so
holding the latitude fixed and moving only in longitude preserves distances
exactly.  Moving in latitude as well is allowed and costs a scale error of
about 1 part in 10^4 per degree, which is below the fitting error of the
conversion and irrelevant to a fixture -- but the default avoids it.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import struct
from collections import Counter
from pathlib import Path
from typing import Dict, List, Tuple
import xml.etree.ElementTree as ET

#: WGS84 semi-major axis (m).  A sphere of this radius is accurate enough
#: for a local frame a few hundred metres across, and the same constant is
#: used in both directions so the round trip is exact by construction.
_EARTH_RADIUS_M = 6378137.0

#: Tags whose *value* is an id in the source map, so keeping one would keep a
#: piece of the source. Everything else passes through: the fixture has to
#: behave exactly like the excerpt it came from, and an allowlist would
#: silently drop any tag the converter learns to read later. Anything matching
#: this is reported by name and count when the tool runs, so a drop is never
#: silent either.
_ID_BEARING_TAGS = ("intersection_area",)


def _is_id_bearing(key: str) -> bool:
    """True when a tag's value is an id from the source map."""
    return key.endswith("_id") or key in _ID_BEARING_TAGS


def _local_frame(
    lat: float, lon: float, lat0: float, lon0: float
) -> Tuple[float, float]:
    """Project (lat, lon) to metres east/north of (lat0, lon0)."""
    east = math.radians(lon - lon0) * _EARTH_RADIUS_M * math.cos(math.radians(lat0))
    north = math.radians(lat - lat0) * _EARTH_RADIUS_M
    return east, north


def _geographic(
    east: float, north: float, lat0: float, lon0: float
) -> Tuple[float, float]:
    """Inverse of :func:`_local_frame`."""
    lat = lat0 + math.degrees(north / _EARTH_RADIUS_M)
    lon = lon0 + math.degrees(east / (_EARTH_RADIUS_M * math.cos(math.radians(lat0))))
    return lat, lon


class RigidTransform:
    """Rotation about one centre followed by placement at another.

    Attributes:
        lat0, lon0: Centre the source is rotated about.
        lat1, lon1: Centre the result is placed at.
        theta: Rotation angle (radians), counter-clockwise.
    """

    def __init__(
        self,
        lat0: float,
        lon0: float,
        lat1: float,
        lon1: float,
        degrees: float,
    ) -> None:
        self.lat0, self.lon0 = lat0, lon0
        self.lat1, self.lon1 = lat1, lon1
        self.theta = math.radians(degrees)
        self._cos, self._sin = math.cos(self.theta), math.sin(self.theta)

    def rotate(self, east: float, north: float) -> Tuple[float, float]:
        """Rotate a vector in the metric frame."""
        return (
            east * self._cos - north * self._sin,
            east * self._sin + north * self._cos,
        )

    def to_metric(self, lat: float, lon: float) -> Tuple[float, float]:
        """Project (lat, lon) into the rotated metric frame."""
        return self.rotate(*_local_frame(lat, lon, self.lat0, self.lon0))

    def to_geographic(self, east: float, north: float) -> Tuple[float, float]:
        """Place a point of the rotated metric frame at the new centre."""
        return _geographic(east, north, self.lat1, self.lon1)


class Perturbation:
    """Displaces every node so the result is no longer the surveyed geometry.

    Two components, because they break different things:

    * a **smooth warp** -- a sum of long-wavelength sinusoids -- displaces the
      whole network by up to ``warp`` metres.  Because its wavelength is far
      longer than a lane is wide, it moves each neighbourhood almost as a
      unit: local shape, curvature and lane width survive, while absolute
      positions and the distances between features hundreds of metres apart
      no longer match the source.
    * **per-node jitter** -- independent Gaussian noise of standard deviation
      ``jitter`` -- breaks the correspondence at the level of the individual
      surveyed vertex, which the warp alone would leave locally intact.

    Both components are a function of the vertex's **position**, not of its
    index or its id.  That is load-bearing, and the first version of this
    class got it wrong.  Keyed on position:

    * two vertices at the same point get the same displacement, so the 33
      pairs of distinct-but-exactly-coincident vertices in the source stay
      exactly coincident.  Real maps contain that degeneracy and the
      converter has to cope with it; keying on identity destroyed all 33.
    * a vertex shared between two ways is one point, so everything built on
      shared boundaries -- routing, lateral adjacency, junction endpoints
      meeting exactly -- is untouched.
    * distinct vertices a few centimetres apart, such as the two branches of
      a fork that nearly meet, are displaced independently.  Their separation
      changes while staying at the same scale, which is the point: the
      values stop being the surveyed ones, the character does not.

    Attributes:
        warp: Peak displacement per axis from the smooth component (m).
        jitter: Standard deviation of the per-position component (m).
        wavelength: Wavelength of the smooth component (m).
    """

    #: Resolution (m) at which a position is hashed.  Two vertices closer
    #: together than this are treated as the same point and displaced
    #: identically; the source's exact coincidences are exact, so any
    #: sub-millimetre value preserves them.
    _POSITION_RESOLUTION_M = 1e-3

    def __init__(
        self, warp: float, jitter: float, wavelength: float, seed: int
    ) -> None:
        self.warp = warp
        self.jitter = jitter
        self.wavelength = wavelength
        self.seed = seed
        rng = random.Random(seed)
        self._phases = [rng.uniform(0.0, 2.0 * math.pi) for _ in range(4)]

    def _jitter_at(self, east: float, north: float) -> Tuple[float, float, float]:
        """Return the position-keyed jitter, the same for the same point."""
        if self.jitter <= 0.0:
            return 0.0, 0.0, 0.0
        scale = 1.0 / self._POSITION_RESOLUTION_M
        key = struct.pack("<qqq", self.seed, round(east * scale), round(north * scale))
        digest = hashlib.blake2b(key, digest_size=16).digest()
        rng = random.Random(int.from_bytes(digest, "big"))
        return (
            rng.gauss(0.0, self.jitter),
            rng.gauss(0.0, self.jitter),
            rng.gauss(0.0, self.jitter),
        )

    def displace(self, east: float, north: float) -> Tuple[float, float, float]:
        """Return the (east, north, up) displacement for a point."""
        k = 2.0 * math.pi / self.wavelength
        p = self._phases
        de = (math.sin(k * east + p[0]) + math.sin(k * north + p[1])) * self.warp / 2
        dn = (math.sin(k * east + p[2]) + math.sin(k * north + p[3])) * self.warp / 2
        je, jn, ju = self._jitter_at(east, north)
        return de + je, dn + jn, ju

    @property
    def max_length_distortion(self) -> float:
        """Upper bound on the fractional length change the warp can cause.

        The warp's gradient is bounded by ``warp * k / 2`` per axis with
        ``k = 2*pi/wavelength``, and a segment's length changes by at most the
        gradient along it.  Reported so a reader can check the amplitude
        against the vertex-spacing distribution rather than take it on trust.
        """
        return self.warp * math.pi / self.wavelength


def _parse(source: Path) -> ET.Element:
    """Parse *source*, refusing a document type declaration.

    The tool deliberately depends on nothing outside the standard library so
    a fixture can be regenerated without the project's container.  That means
    the stdlib parser, which resolves entities, so a ``<!DOCTYPE>`` -- the
    only way a Lanelet2 file could carry one -- is refused outright rather
    than trusted.  A map written by any Lanelet2 tool has none.
    """
    with open(source, "rb") as handle:
        head = handle.read(4096)
    if b"<!DOCTYPE" in head or b"<!ENTITY" in head:
        raise ValueError(
            f"{source} carries a DOCTYPE or ENTITY declaration; refusing to "
            "parse it. Lanelet2 maps do not have one."
        )
    return ET.parse(str(source)).getroot()


def _attr(element: ET.Element, name: str) -> str:
    """Return a required attribute, or say which element is missing it.

    ``Element.get`` is typed as returning ``str | None`` and an OSM primitive
    without its ``id``, ``lat`` or ``ref`` is not something to carry forward
    as ``None`` -- it is a malformed input worth naming.
    """
    value = element.get(name)
    if value is None:
        raise ValueError(f"<{element.tag}> is missing the required attribute {name!r}")
    return value


def _tags(element: ET.Element) -> Dict[str, str]:
    """Return an OSM element's ``<tag k= v=>`` children as a dict."""
    return {_attr(t, "k"): _attr(t, "v") for t in element.findall("tag")}


def _append_tags(parent: ET.Element, tags: Dict[str, str], dropped: Counter) -> None:
    """Append every tag that does not carry a source id, counting the rest."""
    for key, value in tags.items():
        if _is_id_bearing(key):
            dropped[key] += 1
        else:
            ET.SubElement(parent, "tag", k=key, v=value)


def rebuild(
    source: Path,
    destination: Path,
    rotate_degrees: float,
    to_lat: float | None,
    to_lon: float,
    warp: float,
    jitter: float,
    wavelength: float,
    seed: int,
) -> Dict[str, object]:
    """Write the synthetic rebuild of *source* and return a census of it.

    Args:
        source: The excerpt to rebuild.
        destination: Where to write the synthetic map.
        rotate_degrees: Rotation about the excerpt's centroid.
        to_lat: Latitude of the new centre; ``None`` keeps the source's,
            which makes the transform exactly rigid (see the module
            docstring).
        to_lon: Longitude of the new centre.
        warp: Peak displacement per axis of the smooth component (m).
        jitter: Standard deviation of the per-node component (m).
        wavelength: Wavelength of the smooth component (m).
        seed: Seed for the perturbation, so the fixture is reproducible.

    Returns:
        A dict with the counts and the transform, for the caller to print.
    """
    root = _parse(source)
    nodes = [e for e in root if e.tag == "node"]
    ways = [e for e in root if e.tag == "way"]
    relations = [e for e in root if e.tag == "relation"]

    lat0 = sum(float(_attr(n, "lat")) for n in nodes) / len(nodes)
    lon0 = sum(float(_attr(n, "lon")) for n in nodes) / len(nodes)
    transform = RigidTransform(
        lat0, lon0, lat0 if to_lat is None else to_lat, to_lon, rotate_degrees
    )
    perturbation = Perturbation(warp, jitter, wavelength, seed)

    locals_present = [
        (float(t["local_x"]), float(t["local_y"]))
        for t in (_tags(n) for n in nodes)
        if "local_x" in t and "local_y" in t
    ]
    if locals_present:
        cx = sum(p[0] for p in locals_present) / len(locals_present)
        cy = sum(p[1] for p in locals_present) / len(locals_present)
    else:
        cx = cy = 0.0

    # One global id space: lanelet2 keeps node, way and relation ids in the
    # same space in practice, and a fixture that assumes otherwise is a trap.
    next_id = iter(range(1, 1 << 30))
    new_id: Dict[Tuple[str, str], str] = {}
    for element in nodes + ways + relations:
        new_id[(element.tag, _attr(element, "id"))] = str(next(next_id))

    dropped: Counter = Counter()
    out = ET.Element("osm", generator="rebuild_as_synthetic")

    displacements = []
    for node in nodes:
        tags = _tags(node)
        east, north = transform.to_metric(
            float(_attr(node, "lat")), float(_attr(node, "lon"))
        )
        de, dn, du = perturbation.displace(east, north)
        displacements.append(math.hypot(de, dn))
        lat, lon = transform.to_geographic(east + de, north + dn)
        new = ET.SubElement(
            out,
            "node",
            id=new_id[("node", _attr(node, "id"))],
            lat=f"{lat:.11f}",
            lon=f"{lon:.11f}",
        )
        if "local_x" in tags and "local_y" in tags:
            # The same displacement, so Autoware's metric mirror of lat/lon
            # stays consistent with it rather than describing the map as it
            # was before the perturbation.
            x, y = transform.rotate(
                float(tags["local_x"]) - cx, float(tags["local_y"]) - cy
            )
            tags["local_x"] = f"{x + de:.4f}"
            tags["local_y"] = f"{y + dn:.4f}"
        if "ele" in tags:
            tags["ele"] = f"{float(tags['ele']) + du:.3f}"
        _append_tags(new, tags, dropped)

    for way in ways:
        new = ET.SubElement(out, "way", id=new_id[("way", _attr(way, "id"))])
        for nd in way.findall("nd"):
            ET.SubElement(new, "nd", ref=new_id[("node", _attr(nd, "ref"))])
        _append_tags(new, _tags(way), dropped)

    for relation in relations:
        new = ET.SubElement(
            out, "relation", id=new_id[("relation", _attr(relation, "id"))]
        )
        for member in relation.findall("member"):
            key = (_attr(member, "type"), _attr(member, "ref"))
            if key not in new_id:
                # A member pointing outside the file would make lanelet2
                # refuse the result; the crop tool already guarantees there
                # are none, so this is a loud failure rather than a silent drop.
                raise ValueError(
                    f"relation {relation.get('id')} refers to {key[0]} "
                    f"{key[1]}, which is not in {source}"
                )
            ET.SubElement(
                new,
                "member",
                type=key[0],
                ref=new_id[key],
                role=member.get("role", ""),
            )
        _append_tags(new, _tags(relation), dropped)

    ET.indent(out, space="  ")
    # Explicit newline: ElementTree's own filename path opens the file in
    # text mode, so regenerating the fixture on Windows would rewrite every
    # line ending and show the whole file as changed.
    with open(destination, "w", encoding="utf-8", newline="\n") as handle:
        handle.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        handle.write(ET.tostring(out, encoding="unicode"))
        handle.write("\n")

    return {
        "nodes": len(nodes),
        "ways": len(ways),
        "relations": len(relations),
        "rotation_degrees": rotate_degrees,
        "centre_from": [round(lat0, 6), round(lon0, 6)],
        "centre_to": [round(transform.lat1, 6), round(transform.lon1, 6)],
        "dropped_id_bearing_tags": dict(sorted(dropped.items())),
        "perturbation": {
            "warp_m": warp,
            "wavelength_m": wavelength,
            "jitter_sigma_m": jitter,
            "seed": seed,
            "displacement_max_m": round(max(displacements), 4),
            "displacement_mean_m": round(sum(displacements) / len(displacements), 4),
            "max_length_distortion": round(perturbation.max_length_distortion, 6),
        },
    }


def main(argv: List[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("source", type=Path, help="excerpt .osm to rebuild")
    parser.add_argument("destination", type=Path, help="synthetic .osm to write")
    parser.add_argument(
        "--rotate",
        type=float,
        default=0.0,
        help="rotation about the excerpt centroid, in degrees",
    )
    parser.add_argument(
        "--to-lat",
        type=float,
        default=None,
        help="latitude of the new centre (default: the source's, which keeps "
        "the transform exactly rigid)",
    )
    parser.add_argument(
        "--to-lon", type=float, required=True, help="longitude of the new centre"
    )
    parser.add_argument(
        "--warp",
        type=float,
        default=0.25,
        help="peak displacement per axis of the smooth component (m)",
    )
    parser.add_argument(
        "--wavelength",
        type=float,
        default=150.0,
        help="wavelength of the smooth component (m)",
    )
    parser.add_argument(
        "--jitter",
        type=float,
        default=0.02,
        help="standard deviation of the per-node component (m)",
    )
    parser.add_argument(
        "--seed", type=int, default=20260920, help="seed for the perturbation"
    )
    args = parser.parse_args(argv)

    census = rebuild(
        args.source,
        args.destination,
        args.rotate,
        args.to_lat,
        args.to_lon,
        args.warp,
        args.jitter,
        args.wavelength,
        args.seed,
    )
    print(f"wrote {args.destination}")
    print(json.dumps(census, indent=2))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
