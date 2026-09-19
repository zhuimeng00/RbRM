"""
Build a CAD-native GT engineering-feature manifest for RbRM evaluation.

The builder:
- reads existing GT feature partitions and source STEP models;
- associates feature instances with source CAD analytic faces;
- exports provenance information and metric eligibility.

Point clouds are used only for instance provenance and finite extent
definition of Cylinder/Cone features. Intrinsic analytic parameters are
read from source CAD/B-Rep geometry.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

# -----------------------------------------------------------------------------
# PythonOCC imports
# -----------------------------------------------------------------------------
try:
    from OCC.Core.STEPControl import STEPControl_Reader
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_VERTEX
    from OCC.Core.TopoDS import topods
    from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
    from OCC.Core.GeomAbs import (
        GeomAbs_Plane,
        GeomAbs_Cylinder,
        GeomAbs_Cone,
        GeomAbs_Sphere,
        GeomAbs_Torus,
    )
    from OCC.Core.Bnd import Bnd_Box
    from OCC.Core.BRepBndLib import brepbndlib
    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.GProp import GProp_GProps
    from OCC.Core.BRepGProp import brepgprop
except ImportError as exc:  # pragma: no cover - checked in user's OCC env
    print(
        "ERROR: pythonocc-core is required. Run this script in the same conda "
        "environment used by RbRM/OpenCascade.\n"
        f"Original import error: {exc}",
        file=sys.stderr,
    )
    raise


# -----------------------------------------------------------------------------
# Constants / schema
# -----------------------------------------------------------------------------
SCHEMA_VERSION = "RbRM-EV2-GTManifest-1.4"

# GT-only provenance/parameter-uniqueness policy.
# These tolerances are used only for source-CAD GT parameter equivalence.
PARAMETER_UNIQUENESS_POLICY = "zero_marginal_prune_then_numerical_equivalence_v1"
AXIS_UNIQUENESS_DEG_TOL = 1e-7
RADIUS_UNIQUENESS_REL_TOL = 1e-9
CENTER_UNIQUENESS_NORMALIZED_TOL = 1e-9
SUPPORTED_ANALYTIC = {"plane", "cylinder", "cone", "sphere", "torus"}
DIRECT_ANALYTIC_INSTANCE_TYPES = {"cylinder", "cone", "sphere", "torus"}
KNOWN_INSTANCE_TYPES = {"plane", "cylinder", "cone", "sphere", "torus", "prism"}

TOPOLOGY_ROLE = {
    0: "new/base/fuse",
    1: "intersect",
    2: "fuse",
    3: "cut",
    4: "fillet",
    5: "chamfer",
}

INSTANCE_RE = re.compile(
    r"^(?P<feature>[A-Za-z_]+)-(?P<index>\d+)-(?P<topology>-?\d+)$"
)

CSV_COLUMNS = [
    "schema_version",
    "dataset",
    "sample_id",
    "instance_id",
    "instance_uid",
    "feature_type",
    "instance_index",
    "topology_code",
    "boolean_role",
    "source_pointcloud",
    "gt_step",
    "point_count",
    "has_normals",
    "model_bbox_diag",
    "model_supported_area_ratio",
    "model_unsupported_area_ratio",
    "primary_face_id",
    "support_face_ids",
    "accepted_face_count",
    "multiface_provenance",
    "preprune_support_face_ids",
    "preprune_accepted_face_count",
    "zero_marginal_pruned_face_ids",
    "zero_marginal_pruned_count",
    "match_status",
    "best_support_fraction",
    "second_support_fraction",
    "support_margin",
    "analytic_union_support_fraction",
    "best_median_residual",
    "normalized_best_median_residual",
    "max_axis_spread_deg",
    "max_radius_rel_spread",
    "centerline_spread_normalized",
    "center_spread_normalized",
    "apex_spread_normalized",
    "semi_angle_spread_deg",
    "support_plane_count",
    "parameter_uniqueness_policy",
    "unique_axis_gt",
    "unique_radius_gt",
    "unique_center_gt",
    "cad_surface_type",
    "axis_x",
    "axis_y",
    "axis_z",
    "center_x",
    "center_y",
    "center_z",
    "radius",
    "major_radius",
    "minor_radius",
    "reference_radius",
    "semi_angle_deg",
    "height",
    "finite_extent_source",
    "feature_pointcloud_axial_span",
    "eligible_fr",
    "eligible_aae",
    "eligible_rre",
    "eligible_ecd",
    "parameter_source",
    "notes",
]


# -----------------------------------------------------------------------------
# Data classes
# -----------------------------------------------------------------------------
@dataclass
class FaceRecord:
    face_id: int
    surface_type: str
    area: float
    bbox: Tuple[float, float, float, float, float, float]
    shape: object
    params: Dict[str, object]


@dataclass
class MatchRecord:
    face_id: int
    support_count: int
    support_fraction: float
    median_residual: float
    p95_residual: float
    normal_agreement_mean: Optional[float]


# -----------------------------------------------------------------------------
# Small helpers
# -----------------------------------------------------------------------------
def _vec3(v) -> np.ndarray:
    return np.asarray([v.X(), v.Y(), v.Z()], dtype=float)


def _pnt3(p) -> np.ndarray:
    return np.asarray([p.X(), p.Y(), p.Z()], dtype=float)


def _unit(v: Sequence[float], eps: float = 1e-15) -> np.ndarray:
    a = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(a))
    if n <= eps:
        return np.zeros(3, dtype=float)
    return a / n


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, tuple):
        return [_jsonable(x) for x in value]
    if isinstance(value, list):
        return [_jsonable(x) for x in value]
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    return value


def _safe_float(x) -> Optional[float]:
    if x is None:
        return None
    try:
        y = float(x)
    except Exception:
        return None
    return y if math.isfinite(y) else None


def _bbox_tuple(shape) -> Tuple[float, float, float, float, float, float]:
    box = Bnd_Box()
    brepbndlib.Add(shape, box)
    if box.IsVoid():
        return (math.nan,) * 6
    return tuple(float(v) for v in box.Get())


def _bbox_diag(bbox: Sequence[float]) -> float:
    xmin, ymin, zmin, xmax, ymax, zmax = bbox
    if not np.all(np.isfinite(bbox)):
        return 0.0
    return float(np.linalg.norm([xmax - xmin, ymax - ymin, zmax - zmin]))


def _inside_bbox(points: np.ndarray, bbox: Sequence[float], pad: float) -> np.ndarray:
    xmin, ymin, zmin, xmax, ymax, zmax = bbox
    lo = np.asarray([xmin - pad, ymin - pad, zmin - pad], dtype=float)
    hi = np.asarray([xmax + pad, ymax + pad, zmax + pad], dtype=float)
    return np.all((points >= lo) & (points <= hi), axis=1)


def _surface_area(face) -> float:
    props = GProp_GProps()
    try:
        brepgprop.SurfaceProperties(face, props)
        return float(props.Mass())
    except Exception:
        return 0.0


def _iter_vertices(shape) -> np.ndarray:
    pts: List[List[float]] = []
    exp = TopExp_Explorer(shape, TopAbs_VERTEX)
    while exp.More():
        vtx = topods.Vertex(exp.Current())
        p = BRep_Tool.Pnt(vtx)
        pts.append([p.X(), p.Y(), p.Z()])
        exp.Next()
    if not pts:
        return np.zeros((0, 3), dtype=float)
    arr = np.asarray(pts, dtype=float)
    # STEP topology can repeat vertices; stable approximate deduplication is enough
    return np.unique(np.round(arr, decimals=12), axis=0)


def _load_step(path: Path):
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone and int(status) != 1:
        raise RuntimeError(f"Cannot read STEP file: {path}; status={status}")
    reader.TransferRoots()
    shape = reader.OneShape()
    if shape.IsNull():
        raise RuntimeError(f"STEP produced a null shape: {path}")
    return shape


def _load_instance_cloud(path: Path) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Load x y z [nx ny nz] text without depending on Open3D."""
    try:
        arr = np.loadtxt(str(path), dtype=float)
    except Exception as exc:
        raise RuntimeError(f"Cannot read point cloud {path}: {exc}") from exc

    if arr.ndim == 1:
        arr = arr[None, :]
    if arr.shape[1] < 3:
        raise ValueError(f"{path} must contain at least x y z columns")

    points = np.asarray(arr[:, :3], dtype=float)
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    normals = None

    if arr.shape[1] >= 6:
        n = np.asarray(arr[:, 3:6], dtype=float)[finite]
        norm = np.linalg.norm(n, axis=1)
        good = np.isfinite(n).all(axis=1) & (norm > 1e-12)
        if np.any(good):
            n_out = np.zeros_like(n)
            n_out[good] = n[good] / norm[good, None]
            normals = n_out

    return points, normals


def _deterministic_subset(
    points: np.ndarray,
    normals: Optional[np.ndarray],
    max_points: int,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    n = len(points)
    if n <= max_points:
        return points, normals
    # Deterministic evenly spaced indices: no hidden evaluation randomness.
    idx = np.linspace(0, n - 1, max_points, dtype=int)
    return points[idx], None if normals is None else normals[idx]


def _parse_instance(path: Path) -> Dict[str, object]:
    stem = path.stem
    m = INSTANCE_RE.match(stem)
    if not m:
        raise ValueError(
            f"Unsupported instance filename '{path.name}'. Expected e.g. "
            "cylinder-0000-0.asc"
        )
    feature_from_name = m.group("feature").lower()
    feature_from_dir = path.parent.name.lower()
    feature_type = feature_from_name
    if feature_from_dir in KNOWN_INSTANCE_TYPES and feature_from_dir != feature_from_name:
        # Keep filename as authoritative but record mismatch later.
        feature_type = feature_from_name
    topo = int(m.group("topology"))
    return {
        "instance_id": stem,
        "feature_type": feature_type,
        "instance_index": int(m.group("index")),
        "topology_code": topo,
        "boolean_role": TOPOLOGY_ROLE.get(topo, f"unknown({topo})"),
        "parent_type": feature_from_dir,
    }


def _find_step(gt_step_dir: Path, sample_id: str) -> Optional[Path]:
    candidates = [
        gt_step_dir / f"{sample_id}.step",
        gt_step_dir / f"{sample_id}.stp",
        gt_step_dir / f"{sample_id}.STEP",
        gt_step_dir / f"{sample_id}.STP",
    ]
    for p in candidates:
        if p.is_file():
            return p
    return None


def _collect_instance_files(sample_dir: Path) -> List[Path]:
    out: List[Path] = []
    for ext in ("*.asc", "*.xyz", "*.txt", "*.xyzn"):
        for p in sample_dir.rglob(ext):
            # Ignore whole-model point cloud such as 00003_index_1.asc at sample root.
            if p.parent == sample_dir:
                continue
            out.append(p)
    return sorted(set(out), key=lambda p: str(p).lower())


# -----------------------------------------------------------------------------
# STEP face extraction
# -----------------------------------------------------------------------------
def _extract_face_params(face, surface_type: str) -> Dict[str, object]:
    surf = BRepAdaptor_Surface(face)
    vertices = _iter_vertices(face)
    params: Dict[str, object] = {}

    if surface_type == "plane":
        obj = surf.Plane()
        params["location"] = _pnt3(obj.Location())
        params["normal"] = _unit(_vec3(obj.Axis().Direction()))

    elif surface_type == "cylinder":
        obj = surf.Cylinder()
        loc = _pnt3(obj.Location())
        axis = _unit(_vec3(obj.Axis().Direction()))
        params.update(location=loc, axis=axis, radius=float(obj.Radius()))
        if len(vertices):
            t = (vertices - loc) @ axis
            t0, t1 = float(np.min(t)), float(np.max(t))
            params["height"] = abs(t1 - t0)
            params["center"] = loc + 0.5 * (t0 + t1) * axis
            params["bottom_center"] = loc + t0 * axis
            params["top_center"] = loc + t1 * axis

    elif surface_type == "cone":
        obj = surf.Cone()
        loc = _pnt3(obj.Location())
        axis = _unit(_vec3(obj.Axis().Direction()))
        params.update(
            location=loc,
            axis=axis,
            reference_radius=float(obj.RefRadius()),
            semi_angle_rad=float(obj.SemiAngle()),
            semi_angle_deg=float(math.degrees(obj.SemiAngle())),
        )
        try:
            params["apex"] = _pnt3(obj.Apex())
        except Exception:
            pass
        if len(vertices):
            t = (vertices - loc) @ axis
            t0, t1 = float(np.min(t)), float(np.max(t))
            params["height"] = abs(t1 - t0)
            params["center"] = loc + 0.5 * (t0 + t1) * axis
            params["bottom_center"] = loc + t0 * axis
            params["top_center"] = loc + t1 * axis

    elif surface_type == "sphere":
        obj = surf.Sphere()
        params.update(
            center=_pnt3(obj.Location()),
            radius=float(obj.Radius()),
        )

    elif surface_type == "torus":
        obj = surf.Torus()
        params.update(
            center=_pnt3(obj.Location()),
            axis=_unit(_vec3(obj.Axis().Direction())),
            major_radius=float(obj.MajorRadius()),
            minor_radius=float(obj.MinorRadius()),
        )

    return params


def _enumerate_faces(shape) -> Tuple[List[FaceRecord], Dict[str, float]]:
    records: List[FaceRecord] = []
    area_by_type: Dict[str, float] = defaultdict(float)

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    face_id = 0
    while exp.More():
        face_id += 1
        face = topods.Face(exp.Current())
        surf = BRepAdaptor_Surface(face)
        st = surf.GetType()

        if st == GeomAbs_Plane:
            name = "plane"
        elif st == GeomAbs_Cylinder:
            name = "cylinder"
        elif st == GeomAbs_Cone:
            name = "cone"
        elif st == GeomAbs_Sphere:
            name = "sphere"
        elif st == GeomAbs_Torus:
            name = "torus"
        else:
            # Keep all other surface types as unsupported rather than guessing.
            name = f"unsupported_{int(st)}"

        area = _surface_area(face)
        area_by_type[name] += area
        params = _extract_face_params(face, name) if name in SUPPORTED_ANALYTIC else {}
        records.append(
            FaceRecord(
                face_id=face_id,
                surface_type=name,
                area=area,
                bbox=_bbox_tuple(face),
                shape=face,
                params=params,
            )
        )
        exp.Next()

    return records, dict(area_by_type)


# -----------------------------------------------------------------------------
# Analytic residual / normal models
# -----------------------------------------------------------------------------
def _analytic_residual_and_normals(
    face: FaceRecord,
    points: np.ndarray,
) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Distance-like residual to the underlying exact analytic surface.

    This is used only for identifying which source-CAD face supports an already
    known GT feature instance; it is NOT a prediction-vs-GT evaluation metric.
    """
    p = points
    q = face.params
    st = face.surface_type

    if st == "plane":
        loc = np.asarray(q["location"], dtype=float)
        n = _unit(q["normal"])
        signed = (p - loc) @ n
        residual = np.abs(signed)
        expected = np.repeat(n[None, :], len(p), axis=0)
        return residual, expected

    if st == "cylinder":
        loc = np.asarray(q["location"], dtype=float)
        a = _unit(q["axis"])
        d = p - loc
        t = d @ a
        radial = d - t[:, None] * a[None, :]
        rr = np.linalg.norm(radial, axis=1)
        residual = np.abs(rr - float(q["radius"]))
        expected = np.zeros_like(radial)
        good = rr > 1e-15
        expected[good] = radial[good] / rr[good, None]
        return residual, expected

    if st == "sphere":
        c = np.asarray(q["center"], dtype=float)
        d = p - c
        rr = np.linalg.norm(d, axis=1)
        residual = np.abs(rr - float(q["radius"]))
        expected = np.zeros_like(d)
        good = rr > 1e-15
        expected[good] = d[good] / rr[good, None]
        return residual, expected

    if st == "torus":
        c = np.asarray(q["center"], dtype=float)
        a = _unit(q["axis"])
        major = float(q["major_radius"])
        minor = float(q["minor_radius"])
        d = p - c
        z = d @ a
        in_plane = d - z[:, None] * a[None, :]
        rxy = np.linalg.norm(in_plane, axis=1)
        # Standard torus implicit radial cross-section residual.
        tube_dist = np.sqrt((rxy - major) ** 2 + z**2)
        residual = np.abs(tube_dist - minor)

        expected = np.zeros_like(d)
        good = rxy > 1e-15
        ring_dir = np.zeros_like(d)
        ring_dir[good] = in_plane[good] / rxy[good, None]
        tube_center = c[None, :] + major * ring_dir
        nvec = p - tube_center
        nn = np.linalg.norm(nvec, axis=1)
        ok = nn > 1e-15
        expected[ok] = nvec[ok] / nn[ok, None]
        return residual, expected

    if st == "cone":
        loc = np.asarray(q["location"], dtype=float)
        a = _unit(q["axis"])
        r0 = float(q["reference_radius"])
        alpha = float(q["semi_angle_rad"])
        tan_a = math.tan(alpha)
        d = p - loc
        t = d @ a
        radial = d - t[:, None] * a[None, :]
        rr = np.linalg.norm(radial, axis=1)
        expected_radius = r0 + t * tan_a
        residual = np.abs(rr - expected_radius)

        # Gradient of radial - (r0 + t tan(alpha)).
        expected = np.zeros_like(radial)
        good = rr > 1e-15
        radial_u = np.zeros_like(radial)
        radial_u[good] = radial[good] / rr[good, None]
        nvec = radial_u - tan_a * a[None, :]
        nn = np.linalg.norm(nvec, axis=1)
        ok = nn > 1e-15
        expected[ok] = nvec[ok] / nn[ok, None]
        return residual, expected

    raise ValueError(f"Unsupported residual type: {st}")


def _score_face(
    face: FaceRecord,
    points: np.ndarray,
    normals: Optional[np.ndarray],
    distance_tol: float,
    normal_angle_deg: float,
    bbox_pad: float,
) -> MatchRecord:
    in_box = _inside_bbox(points, face.bbox, bbox_pad)
    residual, expected_normals = _analytic_residual_and_normals(face, points)
    mask = in_box & np.isfinite(residual) & (residual <= distance_tol)

    normal_mean = None
    if normals is not None and expected_normals is not None:
        good_n = (
            np.linalg.norm(normals, axis=1) > 1e-12
        ) & (
            np.linalg.norm(expected_normals, axis=1) > 1e-12
        )
        dot = np.zeros(len(points), dtype=float)
        dot[good_n] = np.abs(np.sum(normals[good_n] * expected_normals[good_n], axis=1))
        cos_thr = math.cos(math.radians(normal_angle_deg))
        mask &= (~good_n) | (dot >= cos_thr)
        if np.any(mask & good_n):
            normal_mean = float(np.mean(dot[mask & good_n]))

    count = int(np.count_nonzero(mask))
    frac = count / max(len(points), 1)
    if count:
        med = float(np.median(residual[mask]))
        p95 = float(np.percentile(residual[mask], 95))
    else:
        med = math.inf
        p95 = math.inf

    return MatchRecord(
        face_id=face.face_id,
        support_count=count,
        support_fraction=float(frac),
        median_residual=med,
        p95_residual=p95,
        normal_agreement_mean=normal_mean,
    )


def _match_direct_analytic_instance(
    feature_type: str,
    points: np.ndarray,
    normals: Optional[np.ndarray],
    faces: List[FaceRecord],
    distance_tol: float,
    normal_angle_deg: float,
    bbox_pad: float,
    min_support_fraction: float,
    min_support_points: int,
) -> Tuple[List[MatchRecord], str]:
    candidates = [f for f in faces if f.surface_type == feature_type]
    if not candidates:
        return [], "NO_SOURCE_FACE_OF_REQUIRED_TYPE"

    scored = [
        _score_face(f, points, normals, distance_tol, normal_angle_deg, bbox_pad)
        for f in candidates
    ]
    scored.sort(key=lambda r: (-r.support_fraction, r.median_residual, r.face_id))

    required = max(min_support_points, int(math.ceil(min_support_fraction * len(points))))
    accepted = [r for r in scored if r.support_count >= required]

    if not accepted:
        return scored[:2], "UNMATCHED_LOW_SUPPORT"
    return accepted, "MATCHED"


def _match_prism_planes(
    points: np.ndarray,
    normals: Optional[np.ndarray],
    faces: List[FaceRecord],
    distance_tol: float,
    normal_angle_deg: float,
    bbox_pad: float,
    min_support_fraction: float,
    min_support_points: int,
) -> Tuple[List[MatchRecord], str]:
    planes = [f for f in faces if f.surface_type == "plane"]
    if not planes:
        return [], "NO_PLANE_FACES"

    scored = [
        _score_face(f, points, normals, distance_tol, normal_angle_deg, bbox_pad)
        for f in planes
    ]
    scored.sort(key=lambda r: (-r.support_fraction, r.median_residual, r.face_id))

    # A prism instance is intentionally a multi-face entity; each constituent
    # plane can occupy only a modest fraction of all instance points.
    required = max(min_support_points, int(math.ceil(min_support_fraction * len(points))))
    accepted = [r for r in scored if r.support_count >= required]
    if len(accepted) < 2:
        return scored[: max(3, len(accepted))], "PRISM_SUPPORT_INCOMPLETE"
    return accepted, "MATCHED_PROVENANCE_ONLY"


# -----------------------------------------------------------------------------
# Provenance diagnostics (do NOT alter matching decisions)
# -----------------------------------------------------------------------------
def _diagnostic_support_mask(
    face: FaceRecord,
    points: np.ndarray,
    normals: Optional[np.ndarray],
    distance_tol: float,
    normal_angle_deg: float,
    bbox_pad: float,
) -> np.ndarray:
    """Reproduce the existing face-support test only for diagnostics.

    This helper intentionally mirrors `_score_face` but is not used to accept or
    reject a match.  It exists solely to measure the union of points explained
    by all already-accepted constituent faces of one engineering instance.
    """
    in_box = _inside_bbox(points, face.bbox, bbox_pad)
    residual, expected_normals = _analytic_residual_and_normals(face, points)
    mask = in_box & np.isfinite(residual) & (residual <= distance_tol)

    if normals is not None and expected_normals is not None:
        good_n = (
            np.linalg.norm(normals, axis=1) > 1e-12
        ) & (
            np.linalg.norm(expected_normals, axis=1) > 1e-12
        )
        dot = np.zeros(len(points), dtype=float)
        dot[good_n] = np.abs(np.sum(normals[good_n] * expected_normals[good_n], axis=1))
        cos_thr = math.cos(math.radians(normal_angle_deg))
        mask &= (~good_n) | (dot >= cos_thr)
    return mask



def _prune_zero_marginal_direct_matches(
    feature_type: str,
    matches: List[MatchRecord],
    match_status: str,
    faces: List[FaceRecord],
    points: np.ndarray,
    normals: Optional[np.ndarray],
    distance_tol: float,
    normal_angle_deg: float,
    bbox_pad: float,
) -> Tuple[List[MatchRecord], Dict[str, object]]:
    """Prune redundant direct-analytic provenance faces deterministically.

    The direct matcher can accept a second analytic face when a small set of
    boundary/intersection samples also happens to satisfy that face's analytic
    surface and trimmed-face bounding box.  Such a face must not participate in
    GT parameter consolidation if it explains no sample not already explained
    by higher-ranked retained faces.

    Policy:
      * only direct analytic MATCHED instances are pruned;
      * matches are traversed in the matcher's existing deterministic order;
      * the first accepted face is retained;
      * a later face is pruned iff its marginal supported-point count is exactly 0.

    Therefore no new percentage/distance threshold is introduced here.
    """

    accepted_status = (
        match_status == "MATCHED"
        and feature_type in DIRECT_ANALYTIC_INSTANCE_TYPES
        and bool(matches)
    )

    if not accepted_status:
        accepted = list(matches) if match_status in {"MATCHED", "MATCHED_PROVENANCE_ONLY"} else []
        return list(matches), {
            "preprune_support_face_ids": [m.face_id for m in accepted],
            "preprune_accepted_face_count": len(accepted),
            "zero_marginal_pruned_face_ids": [],
            "zero_marginal_pruned_count": 0,
        }

    union_mask = np.zeros(len(points), dtype=bool)
    retained: List[MatchRecord] = []
    pruned: List[MatchRecord] = []

    for match in matches:
        face = _face_by_id(faces, match.face_id)
        mask = _diagnostic_support_mask(
            face,
            points,
            normals,
            distance_tol,
            normal_angle_deg,
            bbox_pad,
        )

        marginal_count = int(np.count_nonzero(mask & ~union_mask))

        # Always retain the first accepted face.  For later faces, exact
        # zero-marginal contribution is the only pruning criterion.
        if retained and marginal_count == 0:
            pruned.append(match)
            continue

        retained.append(match)
        union_mask |= mask

    return retained, {
        "preprune_support_face_ids": [m.face_id for m in matches],
        "preprune_accepted_face_count": len(matches),
        "zero_marginal_pruned_face_ids": [m.face_id for m in pruned],
        "zero_marginal_pruned_count": len(pruned),
    }


def _line_shortest_distance(
    p1: Sequence[float],
    a1: Sequence[float],
    p2: Sequence[float],
    a2: Sequence[float],
) -> float:
    """Shortest distance between two infinite 3-D lines."""
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    a1 = _unit(a1)
    a2 = _unit(a2)
    cross = np.cross(a1, a2)
    cn = float(np.linalg.norm(cross))
    delta = p2 - p1
    if cn > 1e-12:
        return float(abs(np.dot(delta, cross)) / cn)
    # Parallel / anti-parallel lines.
    return float(np.linalg.norm(delta - np.dot(delta, a1) * a1))


def _max_pairwise_axis_spread_deg(face_records: List[FaceRecord]) -> Optional[float]:
    axes = []
    for f in face_records:
        axis = f.params.get("axis")
        if axis is not None:
            axes.append(_unit(axis))
    if len(axes) < 2:
        return None

    worst = 0.0
    for i in range(len(axes)):
        for j in range(i + 1, len(axes)):
            dot = float(np.clip(abs(np.dot(axes[i], axes[j])), -1.0, 1.0))
            worst = max(worst, math.degrees(math.acos(dot)))
    return float(worst)


def _max_pairwise_radius_rel_spread(
    feature_type: str,
    face_records: List[FaceRecord],
) -> Optional[float]:
    """Type-safe radius spread for invariant radius parameters.

    Cone reference radius is deliberately excluded because it depends on the
    chosen axial reference location even for the same infinite conical surface.
    """
    if len(face_records) < 2:
        return None

    def rel_spread(values: List[float]) -> Optional[float]:
        vals = [abs(float(v)) for v in values if v is not None and math.isfinite(float(v))]
        if len(vals) < 2:
            return None
        denom = max(max(vals), 1e-15)
        return float((max(vals) - min(vals)) / denom)

    if feature_type in {"cylinder", "sphere"}:
        return rel_spread([f.params.get("radius") for f in face_records])
    if feature_type == "torus":
        major = rel_spread([f.params.get("major_radius") for f in face_records])
        minor = rel_spread([f.params.get("minor_radius") for f in face_records])
        vals = [v for v in (major, minor) if v is not None]
        return max(vals) if vals else None
    return None



def _max_pairwise_point_spread_normalized(
    points: List[Sequence[float]],
    model_diag: float,
) -> Optional[float]:
    """Maximum Euclidean separation of a set of CAD-native points, normalized by model scale."""
    if len(points) < 2 or model_diag <= 0:
        return None
    pts = [np.asarray(p, dtype=float) for p in points]
    worst = 0.0
    for i in range(len(pts)):
        for j in range(i + 1, len(pts)):
            worst = max(worst, float(np.linalg.norm(pts[i] - pts[j])))
    return float(worst / model_diag)


def _max_pairwise_center_spread_normalized(
    feature_type: str,
    face_records: List[FaceRecord],
    model_diag: float,
) -> Optional[float]:
    """Center consistency for surface types whose analytic center is intrinsic."""
    if feature_type not in {"sphere", "torus"}:
        return None
    centers = [f.params.get("center") for f in face_records if f.params.get("center") is not None]
    return _max_pairwise_point_spread_normalized(centers, model_diag)


def _max_pairwise_apex_spread_normalized(
    feature_type: str,
    face_records: List[FaceRecord],
    model_diag: float,
) -> Optional[float]:
    if feature_type != "cone":
        return None
    apexes = [f.params.get("apex") for f in face_records if f.params.get("apex") is not None]
    return _max_pairwise_point_spread_normalized(apexes, model_diag)


def _max_pairwise_semi_angle_spread_deg(
    feature_type: str,
    face_records: List[FaceRecord],
) -> Optional[float]:
    if feature_type != "cone" or len(face_records) < 2:
        return None
    vals = []
    for f in face_records:
        v = f.params.get("semi_angle_deg")
        if v is not None and math.isfinite(float(v)):
            vals.append(float(v))
    if len(vals) < 2:
        return None
    return float(max(vals) - min(vals))



def _parameter_uniqueness(
    feature_type: str,
    face_records: List[FaceRecord],
    model_diag: float,
) -> Dict[str, Optional[bool]]:
    """Determine whether retained source-CAD faces define unique GT parameters.

    This is a GT-internal numerical-equivalence test, not a prediction matching
    rule.  Tolerances are intentionally near numerical precision and are exposed
    in every dataset manifest.
    """

    out: Dict[str, Optional[bool]] = {
        "unique_axis_gt": None,
        "unique_radius_gt": None,
        "unique_center_gt": None,
    }

    if not face_records:
        return out

    n = len(face_records)

    # Axis-bearing metric types.
    if feature_type in {"cylinder", "cone", "torus"}:
        if n == 1:
            out["unique_axis_gt"] = face_records[0].params.get("axis") is not None
        else:
            spread = _max_pairwise_axis_spread_deg(face_records)
            out["unique_axis_gt"] = (
                spread is not None
                and spread <= AXIS_UNIQUENESS_DEG_TOL
            )

    # A generic single-radius GT is currently used only by Cylinder/Sphere
    # metrics.  Torus uniqueness is still tracked for provenance completeness.
    if feature_type in {"cylinder", "sphere", "torus"}:
        if n == 1:
            p = face_records[0].params
            if feature_type == "torus":
                vals = (p.get("major_radius"), p.get("minor_radius"))
                out["unique_radius_gt"] = all(
                    _safe_float(v) is not None and float(v) > 0
                    for v in vals
                )
            else:
                r = _safe_float(p.get("radius"))
                out["unique_radius_gt"] = r is not None and r > 0
        else:
            spread = _max_pairwise_radius_rel_spread(feature_type, face_records)
            out["unique_radius_gt"] = (
                spread is not None
                and spread <= RADIUS_UNIQUENESS_REL_TOL
            )

    # Center semantics:
    #   Cylinder/Cone -> finite center is defined on a unique analytic axis line
    #                    using the holistic GT feature cloud axial span.
    #   Sphere/Torus  -> CAD-native analytic center is intrinsic.
    if feature_type in {"cylinder", "cone"}:
        if n == 1:
            out["unique_center_gt"] = (
                face_records[0].params.get("axis") is not None
                and (
                    face_records[0].params.get("location") is not None
                    or face_records[0].params.get("center") is not None
                    or face_records[0].params.get("apex") is not None
                )
            )
        else:
            spread = _max_pairwise_centerline_spread_normalized(
                feature_type,
                face_records,
                model_diag,
            )
            out["unique_center_gt"] = (
                out["unique_axis_gt"] is True
                and spread is not None
                and spread <= CENTER_UNIQUENESS_NORMALIZED_TOL
            )

    elif feature_type in {"sphere", "torus"}:
        if n == 1:
            out["unique_center_gt"] = face_records[0].params.get("center") is not None
        else:
            spread = _max_pairwise_center_spread_normalized(
                feature_type,
                face_records,
                model_diag,
            )
            out["unique_center_gt"] = (
                spread is not None
                and spread <= CENTER_UNIQUENESS_NORMALIZED_TOL
            )

    return out


def _parameter_source_label(
    feature_type: str,
    face_records: List[FaceRecord],
    uniqueness: Dict[str, Optional[bool]],
) -> str:
    if not face_records:
        return "provenance_only"

    if len(face_records) == 1:
        return "source_cad_exact_analytic_intrinsics"

    relevant: List[Optional[bool]] = []

    if feature_type in {"cylinder", "cone", "torus"}:
        relevant.append(uniqueness.get("unique_axis_gt"))

    if feature_type in {"cylinder", "sphere", "torus"}:
        relevant.append(uniqueness.get("unique_radius_gt"))

    if feature_type in {"cylinder", "cone", "sphere", "torus"}:
        relevant.append(uniqueness.get("unique_center_gt"))

    if relevant and all(v is True for v in relevant):
        return "source_cad_multiface_equivalent_intrinsics"

    return "source_cad_multiface_parameter_specific_intrinsics"


def _aligned_mean_axis(face_records: List[FaceRecord], fallback: Sequence[float]) -> np.ndarray:
    """Unweighted sign-aligned mean of equivalent CAD analytic axes.

    Support fractions are deliberately not used as parameter weights.  They are
    provenance evidence only.
    """
    ref = _unit(fallback)
    axes = []
    for f in face_records:
        axis = f.params.get("axis")
        if axis is None:
            continue
        a = _unit(axis)
        if float(np.dot(a, ref)) < 0:
            a = -a
        axes.append(a)
    if not axes:
        return ref
    mean = np.sum(np.asarray(axes), axis=0)
    if float(np.linalg.norm(mean)) <= 1e-15:
        return ref
    return _unit(mean)


def _median_scalar(values: Iterable[object]) -> Optional[float]:
    vals = []
    for value in values:
        try:
            x = float(value)
        except Exception:
            continue
        if math.isfinite(x):
            vals.append(x)
    if not vals:
        return None
    return float(np.median(np.asarray(vals, dtype=float)))


def _mean_point(values: Iterable[object]) -> Optional[np.ndarray]:
    pts = []
    for value in values:
        if value is None:
            continue
        arr = np.asarray(value, dtype=float)
        if arr.shape == (3,) and np.isfinite(arr).all():
            pts.append(arr)
    if not pts:
        return None
    return np.mean(np.asarray(pts), axis=0)


def _union_axial_extent(
    face_records: List[FaceRecord],
    axis: Sequence[float],
    origin: Sequence[float],
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float]]:
    """Finite extent of the union of all accepted trimmed faces along a canonical axis."""
    a = _unit(axis)
    o = np.asarray(origin, dtype=float)
    projected = []
    for f in face_records:
        vertices = _iter_vertices(f.shape)
        if len(vertices):
            projected.append((vertices - o) @ a)
    if not projected:
        return None
    t = np.concatenate(projected)
    if not len(t) or not np.isfinite(t).all():
        return None
    t0 = float(np.min(t))
    t1 = float(np.max(t))
    bottom = o + t0 * a
    top = o + t1 * a
    center = o + 0.5 * (t0 + t1) * a
    return bottom, top, center, abs(t1 - t0)



def _pointcloud_axial_extent(
    points: np.ndarray,
    axis: Sequence[float],
    origin: Sequence[float],
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, float, float, float]]:
    """Finite engineering-instance extent from the holistic GT feature cloud.

    The analytic axis is supplied by the source CAD primitive.  No primitive is
    re-fitted to the point cloud.  For the CAD-derived DeepCAD/CADParser GT
    partitions, the cloud is sampled directly from source CAD and therefore the
    exact min/max axial projections are the most direct definition of the
    finite engineering-instance range.

    Returns
    -------
    bottom, top, center, height, t_min, t_max
        All points lie on the source-CAD analytic axis except that t_min/t_max
        are scalar coordinates relative to ``origin``.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] < 3 or len(pts) == 0:
        return None

    pts = pts[:, :3]
    finite = np.isfinite(pts).all(axis=1)
    pts = pts[finite]
    if len(pts) == 0:
        return None

    a = _unit(axis)
    o = np.asarray(origin, dtype=float)
    if o.shape != (3,) or not np.isfinite(o).all():
        return None

    t = (pts - o) @ a
    t = t[np.isfinite(t)]
    if len(t) == 0:
        return None

    t0 = float(np.min(t))
    t1 = float(np.max(t))
    bottom = o + t0 * a
    top = o + t1 * a
    center = o + 0.5 * (t0 + t1) * a
    height = abs(t1 - t0)
    return bottom, top, center, height, t0, t1


def _canonical_direct_parameters(
    feature_type: str,
    accepted_faces: List[FaceRecord],
    primary: FaceRecord,
    instance_points: np.ndarray,
    model_diag: float,
    uniqueness: Dict[str, Optional[bool]],
) -> Dict[str, object]:
    """Construct only those GT parameters that are unique after pruning.

    No parameter is support-weighted.  A retained multiface instance contributes
    a metric parameter only when the corresponding source-CAD parameter is
    numerically equivalent across all retained constituent faces.

    For finite Cylinder/Cone instances, height and (when the axis line is unique)
    center are obtained from the holistic GT feature cloud projected onto the
    source-CAD analytic axis.  No primitive is re-fitted to the point cloud.
    """

    if not accepted_faces:
        return {}

    p0 = primary.params
    out: Dict[str, object] = {}

    axis_unique = uniqueness.get("unique_axis_gt") is True
    radius_unique = uniqueness.get("unique_radius_gt") is True
    center_unique = uniqueness.get("unique_center_gt") is True

    if feature_type == "cylinder":
        axis = (
            _aligned_mean_axis(accepted_faces, p0["axis"])
            if axis_unique
            else None
        )
        if axis is not None:
            out["axis"] = axis

        if radius_unique:
            out["radius"] = _median_scalar(
                f.params.get("radius")
                for f in accepted_faces
            )

        # Height is axis-direction dependent but translation-invariant along the
        # chosen axis line.  Expose center only when the retained CAD faces define
        # one unique axis line.
        if axis is not None:
            origin = np.asarray(p0.get("location", p0.get("center")), dtype=float)
            if center_unique:
                locations = [f.params.get("location") for f in accepted_faces]
                mean_origin = _mean_point(locations)
                if mean_origin is not None:
                    origin = mean_origin

            extent = _pointcloud_axial_extent(instance_points, axis, origin)
            if extent is not None:
                (
                    bottom,
                    top,
                    center,
                    height,
                    t_min,
                    t_max,
                ) = extent
                out["height"] = height
                out["feature_pointcloud_t_min"] = t_min
                out["feature_pointcloud_t_max"] = t_max
                out["finite_extent_source"] = "gt_feature_pointcloud_axis_projection"
                out["feature_pointcloud_axial_span"] = height
                if center_unique:
                    out["location"] = origin
                    out["bottom_center"] = bottom
                    out["top_center"] = top
                    out["center"] = center

        return out

    if feature_type == "cone":
        axis = (
            _aligned_mean_axis(accepted_faces, p0["axis"])
            if axis_unique
            else None
        )
        if axis is not None:
            out["axis"] = axis

        # Cone RefRadius is axial-reference dependent and is not a RRE
        # quantity.  Preserve the primary value only for traceability.
        out["reference_radius"] = p0.get("reference_radius")

        # Semi-angle is intrinsic.  Only consolidate it when retained faces are
        # numerically equivalent; otherwise leave it absent rather than inventing
        # an average.  It is currently not a reported metric.
        semi_spread = _max_pairwise_semi_angle_spread_deg(feature_type, accepted_faces)
        if len(accepted_faces) == 1 or (
            semi_spread is not None
            and semi_spread <= AXIS_UNIQUENESS_DEG_TOL
        ):
            out["semi_angle_deg"] = _median_scalar(
                f.params.get("semi_angle_deg")
                for f in accepted_faces
            )
            out["semi_angle_rad"] = _median_scalar(
                f.params.get("semi_angle_rad")
                for f in accepted_faces
            )

        if axis is not None:
            origin = np.asarray(
                p0.get("location", p0.get("apex", p0.get("center"))),
                dtype=float,
            )
            if center_unique:
                locations = [f.params.get("location") for f in accepted_faces]
                mean_origin = _mean_point(locations)
                if mean_origin is not None:
                    origin = mean_origin

                apex = _mean_point(
                    f.params.get("apex")
                    for f in accepted_faces
                )
                if apex is not None:
                    out["apex"] = apex

            extent = _pointcloud_axial_extent(instance_points, axis, origin)
            if extent is not None:
                (
                    bottom,
                    top,
                    center,
                    height,
                    t_min,
                    t_max,
                ) = extent
                out["height"] = height
                out["feature_pointcloud_t_min"] = t_min
                out["feature_pointcloud_t_max"] = t_max
                out["finite_extent_source"] = "gt_feature_pointcloud_axis_projection"
                out["feature_pointcloud_axial_span"] = height
                if center_unique:
                    out["location"] = origin
                    out["bottom_center"] = bottom
                    out["top_center"] = top
                    out["center"] = center

        return out

    if feature_type == "sphere":
        if center_unique:
            center = _mean_point(
                f.params.get("center")
                for f in accepted_faces
            )
            if center is not None:
                out["center"] = center

        if radius_unique:
            out["radius"] = _median_scalar(
                f.params.get("radius")
                for f in accepted_faces
            )

        return out

    if feature_type == "torus":
        if axis_unique:
            out["axis"] = _aligned_mean_axis(
                accepted_faces,
                p0["axis"],
            )

        if center_unique:
            center = _mean_point(
                f.params.get("center")
                for f in accepted_faces
            )
            if center is not None:
                out["center"] = center

        if radius_unique:
            out["major_radius"] = _median_scalar(
                f.params.get("major_radius")
                for f in accepted_faces
            )
            out["minor_radius"] = _median_scalar(
                f.params.get("minor_radius")
                for f in accepted_faces
            )

        return out

    return dict(p0)


def _max_pairwise_centerline_spread_normalized(
    feature_type: str,
    face_records: List[FaceRecord],
    model_diag: float,
) -> Optional[float]:
    if len(face_records) < 2 or model_diag <= 0:
        return None

    lines: List[Tuple[np.ndarray, np.ndarray]] = []
    for f in face_records:
        p = f.params
        axis = p.get("axis")
        if axis is None:
            continue

        if feature_type == "torus":
            origin = p.get("center")
        elif feature_type == "cone":
            origin = p.get("apex", p.get("location"))
        else:  # cylinder and any future axis-bearing analytic type
            origin = p.get("location", p.get("center"))

        if origin is not None:
            lines.append((np.asarray(origin, dtype=float), _unit(axis)))

    if len(lines) < 2:
        return None

    worst = 0.0
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            d = _line_shortest_distance(lines[i][0], lines[i][1], lines[j][0], lines[j][1])
            worst = max(worst, d)
    return float(worst / model_diag)


def _build_provenance_diagnostics(
    feature_type: str,
    points: np.ndarray,
    normals: Optional[np.ndarray],
    faces: List[FaceRecord],
    matches: List[MatchRecord],
    match_status: str,
    model_diag: float,
    distance_tol: float,
    normal_angle_deg: float,
    bbox_pad: float,
) -> Dict[str, object]:
    accepted = match_status in {"MATCHED", "MATCHED_PROVENANCE_ONLY"} and bool(matches)
    if not accepted:
        return {
            "accepted_face_count": 0,
            "multiface_provenance": False,
            "analytic_union_support_fraction": None,
            "normalized_best_median_residual": None,
            "max_axis_spread_deg": None,
            "max_radius_rel_spread": None,
            "centerline_spread_normalized": None,
            "center_spread_normalized": None,
            "apex_spread_normalized": None,
            "semi_angle_spread_deg": None,
            "support_plane_count": 0 if feature_type == "prism" else None,
        }

    accepted_faces = [_face_by_id(faces, m.face_id) for m in matches]
    union_mask = np.zeros(len(points), dtype=bool)
    for f in accepted_faces:
        union_mask |= _diagnostic_support_mask(
            f, points, normals, distance_tol, normal_angle_deg, bbox_pad
        )

    union_fraction = float(np.mean(union_mask)) if len(points) else None
    best_residual = matches[0].median_residual if matches else None
    normalized_residual = (
        float(best_residual / model_diag)
        if best_residual is not None and math.isfinite(best_residual) and model_diag > 0
        else None
    )

    return {
        "accepted_face_count": len(accepted_faces),
        "multiface_provenance": len(accepted_faces) > 1,
        "analytic_union_support_fraction": union_fraction,
        "normalized_best_median_residual": normalized_residual,
        "max_axis_spread_deg": _max_pairwise_axis_spread_deg(accepted_faces),
        "max_radius_rel_spread": _max_pairwise_radius_rel_spread(feature_type, accepted_faces),
        "centerline_spread_normalized": _max_pairwise_centerline_spread_normalized(
            feature_type, accepted_faces, model_diag
        ),
        "center_spread_normalized": _max_pairwise_center_spread_normalized(
            feature_type, accepted_faces, model_diag
        ),
        "apex_spread_normalized": _max_pairwise_apex_spread_normalized(
            feature_type, accepted_faces, model_diag
        ),
        "semi_angle_spread_deg": _max_pairwise_semi_angle_spread_deg(
            feature_type, accepted_faces
        ),
        "support_plane_count": len(accepted_faces) if feature_type == "prism" else None,
    }


# -----------------------------------------------------------------------------
# Manifest row construction
# -----------------------------------------------------------------------------
def _face_by_id(faces: List[FaceRecord], face_id: int) -> FaceRecord:
    for f in faces:
        if f.face_id == face_id:
            return f
    raise KeyError(face_id)


def _finite_triplet(row: Dict[str, object], prefix: str) -> bool:
    vals = [row.get(f"{prefix}_{k}") for k in ("x", "y", "z")]
    try:
        return all(v is not None and math.isfinite(float(v)) for v in vals)
    except Exception:
        return False


def _metric_eligibility(
    feature_type: str,
    matched: bool,
    row: Dict[str, object],
) -> Dict[str, bool]:
    """Metric-specific GT eligibility after parameter-uniqueness audit."""

    out = {
        "eligible_fr": feature_type in KNOWN_INSTANCE_TYPES,
        "eligible_aae": False,
        "eligible_rre": False,
        "eligible_ecd": False,
    }

    if not matched:
        return out

    has_axis = _finite_triplet(row, "axis")
    has_center = _finite_triplet(row, "center")
    radius = _safe_float(row.get("radius"))
    has_radius = radius is not None and radius > 0

    unique_axis = row.get("unique_axis_gt") is True
    unique_radius = row.get("unique_radius_gt") is True
    unique_center = row.get("unique_center_gt") is True

    if feature_type == "cylinder":
        out.update(
            eligible_aae=has_axis and unique_axis,
            eligible_rre=has_radius and unique_radius,
            eligible_ecd=has_center and unique_center,
        )
    elif feature_type == "cone":
        out.update(
            eligible_aae=has_axis and unique_axis,
            eligible_ecd=has_center and unique_center,
        )
    elif feature_type == "torus":
        out.update(
            eligible_aae=has_axis and unique_axis,
            eligible_ecd=has_center and unique_center,
        )
    elif feature_type == "sphere":
        out.update(
            eligible_rre=has_radius and unique_radius,
            eligible_ecd=has_center and unique_center,
        )

    # Prism remains FR-only.
    return out



def _build_row(
    dataset: str,
    sample_id: str,
    instance_file: Path,
    gt_seg_root: Path,
    step_path: Path,
    points: np.ndarray,
    normals: Optional[np.ndarray],
    model_diag: float,
    supported_area_ratio: float,
    unsupported_area_ratio: float,
    faces: List[FaceRecord],
    matches: List[MatchRecord],
    match_status: str,
    provenance_diagnostics: Dict[str, object],
    pruning_diagnostics: Dict[str, object],
) -> Dict[str, object]:
    meta = _parse_instance(instance_file)
    feature_type = str(meta["feature_type"])
    accepted_match = match_status in {"MATCHED", "MATCHED_PROVENANCE_ONLY"} and bool(matches)

    primary: Optional[FaceRecord] = None
    retained_faces: List[FaceRecord] = []

    if feature_type in DIRECT_ANALYTIC_INSTANCE_TYPES and accepted_match:
        retained_faces = [_face_by_id(faces, m.face_id) for m in matches]
        primary = retained_faces[0]

    uniqueness = (
        _parameter_uniqueness(feature_type, retained_faces, model_diag)
        if primary is not None
        else {
            "unique_axis_gt": None,
            "unique_radius_gt": None,
            "unique_center_gt": None,
        }
    )

    parameter_source = (
        _parameter_source_label(feature_type, retained_faces, uniqueness)
        if primary is not None
        else "provenance_only"
    )

    row: Dict[str, object] = {
        "schema_version": SCHEMA_VERSION,
        "dataset": dataset,
        "sample_id": sample_id,
        "instance_id": meta["instance_id"],
        "instance_uid": f"{dataset}::{sample_id}::{meta['instance_id']}",
        "feature_type": feature_type,
        "instance_index": meta["instance_index"],
        "topology_code": meta["topology_code"],
        "boolean_role": meta["boolean_role"],
        "source_pointcloud": str(instance_file.relative_to(gt_seg_root)).replace("\\", "/"),
        "gt_step": str(step_path),
        "point_count": int(len(points)),
        "has_normals": normals is not None,
        "model_bbox_diag": model_diag,
        "model_supported_area_ratio": supported_area_ratio,
        "model_unsupported_area_ratio": unsupported_area_ratio,
        "primary_face_id": primary.face_id if primary else None,
        "support_face_ids": [m.face_id for m in matches] if accepted_match else [],
        "accepted_face_count": provenance_diagnostics.get("accepted_face_count"),
        "multiface_provenance": provenance_diagnostics.get("multiface_provenance"),
        "preprune_support_face_ids": pruning_diagnostics.get("preprune_support_face_ids", []),
        "preprune_accepted_face_count": pruning_diagnostics.get("preprune_accepted_face_count", 0),
        "zero_marginal_pruned_face_ids": pruning_diagnostics.get(
            "zero_marginal_pruned_face_ids", []
        ),
        "zero_marginal_pruned_count": pruning_diagnostics.get(
            "zero_marginal_pruned_count", 0
        ),
        "match_status": match_status,
        "best_support_fraction": matches[0].support_fraction if matches else None,
        "second_support_fraction": matches[1].support_fraction if len(matches) > 1 else None,
        "support_margin": (
            matches[0].support_fraction - matches[1].support_fraction
            if len(matches) > 1
            else None
        ),
        "analytic_union_support_fraction": provenance_diagnostics.get(
            "analytic_union_support_fraction"
        ),
        "best_median_residual": matches[0].median_residual if matches else None,
        "normalized_best_median_residual": provenance_diagnostics.get(
            "normalized_best_median_residual"
        ),
        "max_axis_spread_deg": provenance_diagnostics.get("max_axis_spread_deg"),
        "max_radius_rel_spread": provenance_diagnostics.get("max_radius_rel_spread"),
        "centerline_spread_normalized": provenance_diagnostics.get(
            "centerline_spread_normalized"
        ),
        "center_spread_normalized": provenance_diagnostics.get(
            "center_spread_normalized"
        ),
        "apex_spread_normalized": provenance_diagnostics.get(
            "apex_spread_normalized"
        ),
        "semi_angle_spread_deg": provenance_diagnostics.get(
            "semi_angle_spread_deg"
        ),
        "support_plane_count": provenance_diagnostics.get("support_plane_count"),
        "parameter_uniqueness_policy": (
            PARAMETER_UNIQUENESS_POLICY
            if feature_type in DIRECT_ANALYTIC_INSTANCE_TYPES and primary is not None
            else None
        ),
        "unique_axis_gt": uniqueness.get("unique_axis_gt"),
        "unique_radius_gt": uniqueness.get("unique_radius_gt"),
        "unique_center_gt": uniqueness.get("unique_center_gt"),
        "cad_surface_type": primary.surface_type if primary else None,
        "axis_x": None,
        "axis_y": None,
        "axis_z": None,
        "center_x": None,
        "center_y": None,
        "center_z": None,
        "radius": None,
        "major_radius": None,
        "minor_radius": None,
        "reference_radius": None,
        "semi_angle_deg": None,
        "height": None,
        "finite_extent_source": None,
        "feature_pointcloud_axial_span": None,
        "parameter_source": parameter_source,
        "notes": "",
    }

    if meta["parent_type"] != feature_type:
        row["notes"] = (
            f"parent_dir_type={meta['parent_type']} differs from filename_type={feature_type}"
        )

    if primary:
        p = _canonical_direct_parameters(
            feature_type,
            retained_faces,
            primary,
            points,
            model_diag,
            uniqueness,
        )

        axis = p.get("axis")
        center = p.get("center")

        if axis is not None:
            axis = np.asarray(axis, dtype=float)
            row.update(
                axis_x=axis[0],
                axis_y=axis[1],
                axis_z=axis[2],
            )

        if center is not None:
            center = np.asarray(center, dtype=float)
            row.update(
                center_x=center[0],
                center_y=center[1],
                center_z=center[2],
            )

        if feature_type in {"cylinder", "sphere"}:
            row["radius"] = _safe_float(p.get("radius"))

        if feature_type == "torus":
            row["major_radius"] = _safe_float(p.get("major_radius"))
            row["minor_radius"] = _safe_float(p.get("minor_radius"))

        if feature_type == "cone":
            row["reference_radius"] = _safe_float(p.get("reference_radius"))
            row["semi_angle_deg"] = _safe_float(p.get("semi_angle_deg"))

        row["height"] = _safe_float(p.get("height"))
        row["finite_extent_source"] = p.get("finite_extent_source")
        row["feature_pointcloud_axial_span"] = _safe_float(
            p.get("feature_pointcloud_axial_span")
        )

    row.update(
        _metric_eligibility(
            feature_type,
            primary is not None,
            row,
        )
    )

    return row


# -----------------------------------------------------------------------------
# Sample / dataset processing
# -----------------------------------------------------------------------------
def _process_sample(
    dataset: str,
    sample_dir: Path,
    gt_seg_root: Path,
    gt_step_root: Path,
    out_root: Path,
    args,
) -> Tuple[List[Dict[str, object]], Dict[str, object]]:
    sample_id = sample_dir.name
    step_path = _find_step(gt_step_root, sample_id)
    if step_path is None:
        raise FileNotFoundError(f"No STEP file found for sample {sample_id}")

    shape = _load_step(step_path)
    model_bbox = _bbox_tuple(shape)
    model_diag = _bbox_diag(model_bbox)
    if model_diag <= 0:
        raise RuntimeError(f"Invalid model bounding box for {sample_id}: {model_bbox}")

    faces, area_by_type = _enumerate_faces(shape)
    total_area = float(sum(area_by_type.values()))
    supported_area = float(
        sum(v for k, v in area_by_type.items() if k in SUPPORTED_ANALYTIC)
    )
    unsupported_area = max(total_area - supported_area, 0.0)
    supported_area_ratio = supported_area / total_area if total_area > 0 else 0.0
    unsupported_area_ratio = unsupported_area / total_area if total_area > 0 else 0.0

    # Scale-aware matching tolerance. This is ONLY for GT provenance construction.
    # It never participates in prediction-vs-GT Evaluation metrics.
    distance_tol = max(args.abs_match_tol, args.rel_match_tol * model_diag)
    bbox_pad = max(args.bbox_pad_multiplier * distance_tol, args.abs_match_tol)

    instance_files = _collect_instance_files(sample_dir)
    rows: List[Dict[str, object]] = []
    json_features: List[Dict[str, object]] = []

    for instance_file in instance_files:
        meta = _parse_instance(instance_file)
        feature_type = str(meta["feature_type"])
        points, normals = _load_instance_cloud(instance_file)
        points_sub, normals_sub = _deterministic_subset(
            points, normals, args.max_match_points
        )

        if feature_type in DIRECT_ANALYTIC_INSTANCE_TYPES:
            raw_matches, status = _match_direct_analytic_instance(
                feature_type,
                points_sub,
                normals_sub,
                faces,
                distance_tol,
                args.normal_angle_deg,
                bbox_pad,
                args.min_support_fraction,
                args.min_support_points,
            )
        elif feature_type == "prism":
            raw_matches, status = _match_prism_planes(
                points_sub,
                normals_sub,
                faces,
                distance_tol,
                args.normal_angle_deg,
                bbox_pad,
                args.prism_min_plane_fraction,
                args.prism_min_plane_points,
            )
        elif feature_type == "plane":
            raw_matches, status = _match_direct_analytic_instance(
                "plane",
                points_sub,
                normals_sub,
                faces,
                distance_tol,
                args.normal_angle_deg,
                bbox_pad,
                args.min_support_fraction,
                args.min_support_points,
            )
        else:
            raw_matches, status = [], "UNKNOWN_INSTANCE_TYPE"

        matches, pruning_diagnostics = _prune_zero_marginal_direct_matches(
            feature_type,
            raw_matches,
            status,
            faces,
            points_sub,
            normals_sub,
            distance_tol,
            args.normal_angle_deg,
            bbox_pad,
        )

        provenance_diagnostics = _build_provenance_diagnostics(
            feature_type,
            points_sub,
            normals_sub,
            faces,
            matches,
            status,
            model_diag,
            distance_tol,
            args.normal_angle_deg,
            bbox_pad,
        )

        row = _build_row(
            dataset,
            sample_id,
            instance_file,
            gt_seg_root,
            step_path,
            points,
            normals,
            model_diag,
            supported_area_ratio,
            unsupported_area_ratio,
            faces,
            matches,
            status,
            provenance_diagnostics,
            pruning_diagnostics,
        )
        rows.append(row)

        json_features.append(
            {
                **{k: _jsonable(v) for k, v in row.items()},
                # Preserve candidate and retained provenance records.
                "match_candidates": [
                    asdict(m)
                    for m in raw_matches[: args.max_saved_candidates]
                ],
                "retained_matches": [
                    asdict(m)
                    for m in matches[: args.max_saved_candidates]
                ],
                "retained_source_faces": [
                    {
                        "face_id": m.face_id,
                        "surface_type": _face_by_id(faces, m.face_id).surface_type,
                        "params": _jsonable(_face_by_id(faces, m.face_id).params),
                    }
                    for m in matches[: args.max_saved_candidates]
                    if status in {"MATCHED", "MATCHED_PROVENANCE_ONLY"}
                ],
                "zero_marginal_pruned_source_faces": [
                    {
                        "face_id": fid,
                        "surface_type": _face_by_id(faces, fid).surface_type,
                        "params": _jsonable(_face_by_id(faces, fid).params),
                    }
                    for fid in pruning_diagnostics.get(
                        "zero_marginal_pruned_face_ids", []
                    )
                ],
            }
        )

    sample_manifest = {
        "schema_version": SCHEMA_VERSION,
        "dataset": dataset,
        "sample_id": sample_id,
        "gt_step": str(step_path),
        "model_bbox": list(model_bbox),
        "model_bbox_diag": model_diag,
        "matching_provenance_config": {
            "rel_match_tol": args.rel_match_tol,
            "abs_match_tol": args.abs_match_tol,
            "effective_distance_tol": distance_tol,
            "normal_angle_deg": args.normal_angle_deg,
            "bbox_pad": bbox_pad,
            "max_match_points": args.max_match_points,
            "min_support_fraction": args.min_support_fraction,
            "min_support_points": args.min_support_points,
            "prism_min_plane_fraction": args.prism_min_plane_fraction,
            "prism_min_plane_points": args.prism_min_plane_points,
            "note": (
                "These thresholds are used only to associate existing GT instance partitions "
                "with exact source-CAD faces. They are not feature-metric matching thresholds. "
                "Different instance IDs are never merged based on geometric similarity."
            ),
        },
        "gt_parameter_policy": {
            "parameter_uniqueness_policy": PARAMETER_UNIQUENESS_POLICY,
            "zero_marginal_pruning_rule": (
                "For direct analytic MATCHED instances, process accepted faces in "
                "deterministic support-ranked order and prune any later face whose "
                "marginal supported-point count is exactly zero."
            ),
            "axis_uniqueness_deg_tol": AXIS_UNIQUENESS_DEG_TOL,
            "radius_uniqueness_rel_tol": RADIUS_UNIQUENESS_REL_TOL,
            "center_uniqueness_normalized_tol": CENTER_UNIQUENESS_NORMALIZED_TOL,
            "note": (
                "These are GT-internal provenance/numerical-equivalence rules. "
                "They are never used to match predictions to GT instances."
            ),
        },
        "surface_summary": {
            "face_count": len(faces),
            "area_by_surface_type": area_by_type,
            "total_area": total_area,
            "supported_analytic_area": supported_area,
            "unsupported_area": unsupported_area,
            "supported_analytic_area_ratio": supported_area_ratio,
            "unsupported_area_ratio": unsupported_area_ratio,
            "contains_unsupported_surface": bool(unsupported_area > 0),
        },
        "features": json_features,
    }

    sample_out = out_root / "samples"
    sample_out.mkdir(parents=True, exist_ok=True)
    with (sample_out / f"{sample_id}.json").open("w", encoding="utf-8") as f:
        json.dump(_jsonable(sample_manifest), f, indent=2, ensure_ascii=False)

    return rows, sample_manifest


def _write_csv(rows: List[Dict[str, object]], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        w.writeheader()
        for row in rows:
            out = dict(row)
            for key in (
                "support_face_ids",
                "preprune_support_face_ids",
                "zero_marginal_pruned_face_ids",
            ):
                out[key] = ";".join(map(str, row.get(key, [])))
            w.writerow({k: _jsonable(out.get(k)) for k in CSV_COLUMNS})


def _print_summary(rows: List[Dict[str, object]], manifests: List[Dict[str, object]]) -> None:
    type_count = Counter(str(r["feature_type"]) for r in rows)
    role_count = Counter(str(r["boolean_role"]) for r in rows)
    status_count = Counter(str(r["match_status"]) for r in rows)

    print("\n" + "=" * 78)
    print("GT MANIFEST BUILD SUMMARY")
    print("=" * 78)
    print(f"samples        : {len(manifests)}")
    print(f"instances      : {len(rows)}")
    print(f"feature types  : {dict(type_count)}")
    print(f"Boolean roles  : {dict(role_count)}")
    print(f"match status   : {dict(status_count)}")
    print(
        "metric eligible: "
        f"FR={sum(bool(r['eligible_fr']) for r in rows)}, "
        f"AAE={sum(bool(r['eligible_aae']) for r in rows)}, "
        f"RRE={sum(bool(r['eligible_rre']) for r in rows)}, "
        f"ECD={sum(bool(r['eligible_ecd']) for r in rows)}"
    )
    pruned_instances = sum(
        int(r.get("zero_marginal_pruned_count") or 0) > 0
        for r in rows
    )
    pruned_faces = sum(
        int(r.get("zero_marginal_pruned_count") or 0)
        for r in rows
    )
    nonunique_axis = sum(r.get("unique_axis_gt") is False for r in rows)
    nonunique_radius = sum(r.get("unique_radius_gt") is False for r in rows)
    nonunique_center = sum(r.get("unique_center_gt") is False for r in rows)

    print(
        "GT provenance   : "
        f"zero-marginal-pruned instances={pruned_instances}, "
        f"faces={pruned_faces}"
    )
    print(
        "GT uniqueness   : "
        f"axis_nonunique={nonunique_axis}, "
        f"radius_nonunique={nonunique_radius}, "
        f"center_nonunique={nonunique_center}"
    )

    unsupported_models = sum(
        bool(m["surface_summary"]["contains_unsupported_surface"]) for m in manifests
    )
    print(f"models containing unsupported surfaces: {unsupported_models}")
    print("=" * 78)


# -----------------------------------------------------------------------------
# CLI
# -----------------------------------------------------------------------------
def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Build CAD-native GT engineering-feature manifest for RbRM Evaluation V2."
    )
    p.add_argument("--dataset", required=True, help="Dataset label, e.g. DeepCAD or CADParser")
    p.add_argument("--gt-seg", required=True, type=Path, help="Existing GT feature-instance root")
    p.add_argument("--gt-step", required=True, type=Path, help="Source GT STEP directory")
    p.add_argument("--out", required=True, type=Path, help="Output directory")
    p.add_argument(
        "--sample",
        action="append",
        default=[],
        help="Optional sample ID; may be repeated. If omitted, process all sample directories.",
    )

    # Provenance association thresholds: deliberately scale-aware and isolated
    # from the final evaluation metrics.
    p.add_argument("--rel-match-tol", type=float, default=1e-5)
    p.add_argument("--abs-match-tol", type=float, default=1e-7)
    p.add_argument("--normal-angle-deg", type=float, default=12.0)
    p.add_argument("--bbox-pad-multiplier", type=float, default=5.0)
    p.add_argument("--max-match-points", type=int, default=4000)
    p.add_argument("--min-support-fraction", type=float, default=0.01)
    p.add_argument("--min-support-points", type=int, default=25)
    p.add_argument("--prism-min-plane-fraction", type=float, default=0.005)
    p.add_argument("--prism-min-plane-points", type=int, default=15)
    p.add_argument("--max-saved-candidates", type=int, default=12)
    return p


def main() -> int:
    args = build_arg_parser().parse_args()
    gt_seg_root = args.gt_seg.resolve()
    gt_step_root = args.gt_step.resolve()
    out_root = args.out.resolve()

    if not gt_seg_root.is_dir():
        raise FileNotFoundError(f"gt_seg directory not found: {gt_seg_root}")
    if not gt_step_root.is_dir():
        raise FileNotFoundError(f"gt_step directory not found: {gt_step_root}")
    out_root.mkdir(parents=True, exist_ok=True)

    if args.sample:
        sample_dirs = [gt_seg_root / s for s in args.sample]
    else:
        sample_dirs = sorted([p for p in gt_seg_root.iterdir() if p.is_dir()])

    all_rows: List[Dict[str, object]] = []
    all_manifests: List[Dict[str, object]] = []
    failures: List[Dict[str, str]] = []

    for idx, sample_dir in enumerate(sample_dirs, 1):
        print(f"[{idx}/{len(sample_dirs)}] {sample_dir.name}")
        if not sample_dir.is_dir():
            failures.append({"sample_id": sample_dir.name, "error": "sample directory missing"})
            print("  [FAIL] sample directory missing")
            continue
        try:
            rows, manifest = _process_sample(
                args.dataset,
                sample_dir,
                gt_seg_root,
                gt_step_root,
                out_root,
                args,
            )
            all_rows.extend(rows)
            all_manifests.append(manifest)
            matched = sum(r["match_status"] in {"MATCHED", "MATCHED_PROVENANCE_ONLY"} for r in rows)
            print(f"  instances={len(rows)}, matched/provenance={matched}")
        except Exception as exc:
            failures.append({"sample_id": sample_dir.name, "error": repr(exc)})
            print(f"  [FAIL] {exc}")

    csv_path = out_root / "gt_feature_manifest.csv"
    _write_csv(all_rows, csv_path)

    dataset_manifest = {
        "schema_version": SCHEMA_VERSION,
        "dataset": args.dataset,
        "sample_count_requested": len(sample_dirs),
        "sample_count_completed": len(all_manifests),
        "instance_count": len(all_rows),
        "failures": failures,
        "gt_parameter_policy": {
            "parameter_uniqueness_policy": PARAMETER_UNIQUENESS_POLICY,
            "zero_marginal_pruning_rule": (
                "later retained candidate is pruned iff marginal supported-point count == 0"
            ),
            "axis_uniqueness_deg_tol": AXIS_UNIQUENESS_DEG_TOL,
            "radius_uniqueness_rel_tol": RADIUS_UNIQUENESS_REL_TOL,
            "center_uniqueness_normalized_tol": CENTER_UNIQUENESS_NORMALIZED_TOL,
            "prediction_matching_use": False,
        },
        "samples": [
            {
                "sample_id": m["sample_id"],
                "model_bbox_diag": m["model_bbox_diag"],
                "surface_summary": m["surface_summary"],
                "feature_count": len(m["features"]),
            }
            for m in all_manifests
        ],
    }
    with (out_root / "dataset_manifest.json").open("w", encoding="utf-8") as f:
        json.dump(_jsonable(dataset_manifest), f, indent=2, ensure_ascii=False)

    _print_summary(all_rows, all_manifests)
    print(f"CSV : {csv_path}")
    print(f"JSON: {out_root / 'dataset_manifest.json'}")

    if failures:
        print(f"WARNING: {len(failures)} sample(s) failed. See dataset_manifest.json")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
