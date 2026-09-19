"""
Adapter for converting RbRM structured feature XLSX outputs into
evaluation records.

The adapter performs:
- XLSX field parsing;
- feature identity and type normalization;
- structured parameter extraction;
- CSV/JSON record generation.

Metric computation and GT matching are handled by separate evaluation modules.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from openpyxl import load_workbook


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

ADAPTER_PROTOCOL = "RbRM-EV2-PredictionAdapter-1.4"
DEFAULT_SHEET = "PartFeatures"

KNOWN_TYPES = {"cylinder", "cone", "sphere", "torus", "prism"}

TOPOLOGY_ROLE_MAP = {
    "0": "new/base/fuse",
    "1": "intersection",
    "2": "union",
    "3": "cut",
    "4": "fillet",
    "5": "chamfer",
}

FLOAT_RE = re.compile(
    r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
)


# ---------------------------------------------------------------------------
# Basic parsing helpers
# ---------------------------------------------------------------------------

def is_blank(value: Any) -> bool:
    if value is None:
        return True
    s = str(value).strip()
    return s == "" or s.lower() in {"none", "nan", "null"}


def parse_float(value: Any) -> Optional[float]:
    if is_blank(value):
        return None

    if isinstance(value, bool):
        return float(value)

    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except Exception:
        pass

    match = FLOAT_RE.search(str(value))
    if not match:
        return None

    try:
        x = float(match.group(0))
    except Exception:
        return None

    return x if math.isfinite(x) else None


def parse_vector3(value: Any) -> Optional[Tuple[float, float, float]]:
    """
    支持：
      [1.0, 2.0, 3.0]
      (1.0, 2.0, 3.0)
      numpy-style XLSX string: "[ 1.  0. -2.3e-4]"
      comma-separated string
    """
    if is_blank(value):
        return None

    if isinstance(value, (list, tuple)):
        if len(value) != 3:
            return None
        vals = [parse_float(x) for x in value]
        if any(x is None for x in vals):
            return None
        return (float(vals[0]), float(vals[1]), float(vals[2]))

    vals = FLOAT_RE.findall(str(value))
    if len(vals) != 3:
        return None

    out = tuple(float(x) for x in vals)
    if not all(math.isfinite(x) for x in out):
        return None
    return out


def normalize_feature_type(value: Any) -> str:
    if is_blank(value):
        return ""
    return str(value).strip().lower()


def normalize_instance_id(value: Any) -> str:
    if is_blank(value):
        return ""
    return str(value).strip()


def normalize_topology_code(value: Any) -> str:
    if is_blank(value):
        return ""

    s = str(value).strip()
    try:
        x = float(s)
        if math.isfinite(x) and abs(x - round(x)) < 1e-12:
            return str(int(round(x)))
    except Exception:
        pass
    return s


# ---------------------------------------------------------------------------
# Vector helpers
# ---------------------------------------------------------------------------

def vec_norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


def vec_unit(
    v: Optional[Sequence[float]],
) -> Optional[Tuple[float, float, float]]:
    if v is None:
        return None

    n = vec_norm(v)
    if not math.isfinite(n) or n <= 1e-15:
        return None

    return (
        float(v[0]) / n,
        float(v[1]) / n,
        float(v[2]) / n,
    )


def vec_add(
    a: Sequence[float],
    b: Sequence[float],
) -> Tuple[float, float, float]:
    return (
        float(a[0]) + float(b[0]),
        float(a[1]) + float(b[1]),
        float(a[2]) + float(b[2]),
    )


def vec_sub(
    a: Sequence[float],
    b: Sequence[float],
) -> Tuple[float, float, float]:
    return (
        float(a[0]) - float(b[0]),
        float(a[1]) - float(b[1]),
        float(a[2]) - float(b[2]),
    )


def vec_scale(
    v: Sequence[float],
    s: float,
) -> Tuple[float, float, float]:
    return (
        float(v[0]) * float(s),
        float(v[1]) * float(s),
        float(v[2]) * float(s),
    )


def vec_dot(
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    return (
        float(a[0]) * float(b[0])
        + float(a[1]) * float(b[1])
        + float(a[2]) * float(b[2])
    )


def vec_distance(
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    return vec_norm(vec_sub(a, b))


def sign_invariant_angle_deg(
    a: Sequence[float],
    b: Sequence[float],
) -> Optional[float]:
    ua = vec_unit(a)
    ub = vec_unit(b)
    if ua is None or ub is None:
        return None

    c = abs(vec_dot(ua, ub))
    c = min(1.0, max(-1.0, c))
    return math.degrees(math.acos(c))


# ---------------------------------------------------------------------------
# Record schema
# ---------------------------------------------------------------------------

@dataclass
class PredictionFeatureRecord:
    protocol: str

    dataset: str
    sample_id: str
    instance_id: str
    instance_uid: str

    feature_type: str
    topology_code: str
    boolean_role: str

    # Canonical Evaluation V2 geometric attributes
    axis_x: Optional[float] = None
    axis_y: Optional[float] = None
    axis_z: Optional[float] = None

    center_x: Optional[float] = None
    center_y: Optional[float] = None
    center_z: Optional[float] = None
    center_semantics: str = ""

    radius: Optional[float] = None
    height: Optional[float] = None

    # Explicit finite-extent information
    bottom_center_x: Optional[float] = None
    bottom_center_y: Optional[float] = None
    bottom_center_z: Optional[float] = None

    stored_top_center_x: Optional[float] = None
    stored_top_center_y: Optional[float] = None
    stored_top_center_z: Optional[float] = None

    # Cone-specific
    cone_apex_x: Optional[float] = None
    cone_apex_y: Optional[float] = None
    cone_apex_z: Optional[float] = None
    cone_angle: Optional[float] = None
    cone_radius_bottom: Optional[float] = None
    cone_radius_top: Optional[float] = None

    # Torus-specific
    torus_major_radius: Optional[float] = None
    torus_minor_radius: Optional[float] = None

    # Prism-specific / passthrough
    prism_type: str = ""
    length: Optional[float] = None
    width: Optional[float] = None
    side_n: Optional[float] = None
    side_plane_length: Optional[float] = None
    side_plane_width: Optional[float] = None

    # Other RbRM structured metadata
    point_on_axis_x: Optional[float] = None
    point_on_axis_y: Optional[float] = None
    point_on_axis_z: Optional[float] = None

    bottom_visible: Optional[float] = None
    top_visible: Optional[float] = None

    # Adapter-level structural status only.
    # This is NOT Feature Recall.
    adapter_valid: bool = False
    adapter_errors: str = ""
    adapter_warnings: str = ""

    # Diagnostic: stored top_center vs bottom_center + height * axis
    top_axis_checked: bool = False
    top_axis_error: Optional[float] = None
    top_axis_error_over_height: Optional[float] = None
    top_bottom_distance: Optional[float] = None
    top_bottom_height_abs_error: Optional[float] = None
    top_axis_alignment_deg: Optional[float] = None


# ---------------------------------------------------------------------------
# XLSX I/O
# ---------------------------------------------------------------------------

def read_partfeatures_xlsx(
    xlsx_path: Path,
    sheet_name: str = DEFAULT_SHEET,
) -> Tuple[List[str], List[Dict[str, Any]]]:
    wb = load_workbook(
        xlsx_path,
        read_only=True,
        data_only=True,
    )

    if sheet_name not in wb.sheetnames:
        raise RuntimeError(
            f"Worksheet '{sheet_name}' not found. "
            f"Available sheets: {wb.sheetnames}"
        )

    ws = wb[sheet_name]
    it = ws.iter_rows(values_only=True)

    try:
        header_row = next(it)
    except StopIteration:
        raise RuntimeError("The worksheet is empty.")

    headers = [
        str(x).strip() if x is not None else ""
        for x in header_row
    ]

    rows: List[Dict[str, Any]] = []

    for excel_row, values in enumerate(it, start=2):
        row: Dict[str, Any] = {}

        for i, header in enumerate(headers):
            if not header:
                continue
            row[header] = values[i] if i < len(values) else None

        if all(is_blank(v) for v in row.values()):
            continue

        row["_excel_row"] = excel_row
        rows.append(row)

    return headers, rows


# ---------------------------------------------------------------------------
# Canonical parameter construction
# ---------------------------------------------------------------------------

def canonical_center_from_bottom_axis_height(
    bottom: Optional[Tuple[float, float, float]],
    axis: Optional[Tuple[float, float, float]],
    height: Optional[float],
) -> Optional[Tuple[float, float, float]]:
    if bottom is None or axis is None or height is None:
        return None

    if not math.isfinite(height) or height <= 0:
        return None

    u = vec_unit(axis)
    if u is None:
        return None

    return vec_add(
        bottom,
        vec_scale(u, 0.5 * height),
    )


def top_axis_diagnostic(
    bottom: Optional[Tuple[float, float, float]],
    stored_top: Optional[Tuple[float, float, float]],
    axis: Optional[Tuple[float, float, float]],
    height: Optional[float],
) -> Dict[str, Any]:
    result = {
        "checked": False,
        "top_axis_error": None,
        "top_axis_error_over_height": None,
        "top_bottom_distance": None,
        "top_bottom_height_abs_error": None,
        "top_axis_alignment_deg": None,
    }

    if (
        bottom is None
        or stored_top is None
        or axis is None
        or height is None
        or height <= 0
    ):
        return result

    u = vec_unit(axis)
    if u is None:
        return result

    # RbRM 的 axis_vector 对 cylinder/cone/prism 按当前结构表示应从 bottom 指向 top。
    expected_top = vec_add(
        bottom,
        vec_scale(u, height),
    )

    delta = vec_sub(stored_top, bottom)
    top_bottom_distance = vec_norm(delta)

    result.update({
        "checked": True,
        "top_axis_error": vec_distance(stored_top, expected_top),
        "top_axis_error_over_height": (
            vec_distance(stored_top, expected_top)
            / max(abs(height), 1e-15)
        ),
        "top_bottom_distance": top_bottom_distance,
        "top_bottom_height_abs_error": abs(top_bottom_distance - height),
        "top_axis_alignment_deg": (
            sign_invariant_angle_deg(delta, u)
            if top_bottom_distance > 1e-15
            else None
        ),
    })

    return result


def validate_core_fields(
    feature_type: str,
    axis: Optional[Tuple[float, float, float]],
    bottom: Optional[Tuple[float, float, float]],
    center: Optional[Tuple[float, float, float]],
    radius: Optional[float],
    height: Optional[float],
    cone_angle: Optional[float],
    cone_radius_top: Optional[float],
    torus_major_radius: Optional[float],
    torus_minor_radius: Optional[float],
) -> Tuple[List[str], List[str]]:
    errors: List[str] = []
    warnings: List[str] = []

    def require_axis():
        if axis is None:
            errors.append("missing_or_invalid_axis")

    def require_center():
        if center is None:
            errors.append("missing_or_invalid_center")

    def require_positive_radius(value: Optional[float], name: str):
        if value is None or not math.isfinite(value) or value <= 0:
            errors.append(f"missing_or_invalid_{name}")

    def require_positive_height():
        if height is None or not math.isfinite(height) or height <= 0:
            errors.append("missing_or_invalid_height")

    if feature_type == "cylinder":
        require_axis()
        require_center()
        require_positive_radius(radius, "radius")
        require_positive_height()

    elif feature_type == "cone":
        require_axis()
        require_center()
        require_positive_radius(radius, "bottom_radius")
        require_positive_radius(cone_radius_top, "top_radius")
        require_positive_height()

        if cone_angle is None or not math.isfinite(cone_angle):
            errors.append("missing_or_invalid_cone_angle")

    elif feature_type == "sphere":
        require_center()
        require_positive_radius(radius, "radius")

    elif feature_type == "torus":
        require_axis()
        require_center()
        require_positive_radius(torus_major_radius, "torus_major_radius")
        require_positive_radius(torus_minor_radius, "torus_minor_radius")

        if (
            torus_major_radius is not None
            and torus_minor_radius is not None
            and torus_major_radius <= torus_minor_radius
        ):
            warnings.append("torus_major_radius_not_greater_than_minor_radius")

    elif feature_type == "prism":
        # 仅检查当前 RbRM structured prism record 的基础完整性。
        # 是否进入 FR/AAE/ECD 由后续 metric protocol 决定。
        require_axis()
        require_center()
        require_positive_height()

    else:
        errors.append(f"unsupported_feature_type:{feature_type}")

    return errors, warnings


def adapt_one_row(
    dataset: str,
    sample_id: str,
    row: Dict[str, Any],
) -> PredictionFeatureRecord:
    feature_type = normalize_feature_type(row.get("type"))
    instance_id = normalize_instance_id(row.get("name"))
    topology_code = normalize_topology_code(row.get("topology"))
    boolean_role = TOPOLOGY_ROLE_MAP.get(
        topology_code,
        "unknown" if topology_code else "",
    )

    axis_raw = parse_vector3(row.get("axis_vector"))
    axis = vec_unit(axis_raw)

    bottom = parse_vector3(row.get("bottom_center"))
    stored_top = parse_vector3(row.get("top_center"))

    radius = parse_float(row.get("radius"))
    height = parse_float(row.get("height"))

    cone_apex = parse_vector3(row.get("cone_vertex"))
    cone_angle = parse_float(row.get("cone_angle"))
    cone_radius_top = parse_float(row.get("cone_r_top"))

    torus_minor_radius = parse_float(row.get("torus_minor_r"))

    warnings: List[str] = []
    errors: List[str] = []

    if not instance_id:
        errors.append("missing_instance_id")

    if not feature_type:
        errors.append("missing_feature_type")

    if feature_type and feature_type not in KNOWN_TYPES:
        errors.append(f"unsupported_feature_type:{feature_type}")

    if instance_id and feature_type:
        name_prefix = instance_id.split("-", 1)[0].lower()
        if name_prefix != feature_type:
            warnings.append(
                f"name_type_prefix_mismatch:{name_prefix}!={feature_type}"
            )

    if topology_code and topology_code not in TOPOLOGY_ROLE_MAP:
        warnings.append(f"unknown_topology_code:{topology_code}")

    # ------------------------------------------------------------------
    # Evaluation V2 canonical center semantics
    # ------------------------------------------------------------------

    center: Optional[Tuple[float, float, float]] = None
    center_semantics = ""

    if feature_type in {"cylinder", "cone", "prism"}:
        center = canonical_center_from_bottom_axis_height(
            bottom,
            axis,
            height,
        )
        center_semantics = (
            "finite_extent_midpoint="
            "bottom_center+0.5*height*unit(axis_vector)"
        )

    elif feature_type == "sphere":
        center = bottom
        center_semantics = "analytic_sphere_center=bottom_center"

    elif feature_type == "torus":
        center = bottom
        center_semantics = "analytic_torus_center=bottom_center"

    torus_major_radius = (
        radius if feature_type == "torus" else None
    )

    cone_radius_bottom = (
        radius if feature_type == "cone" else None
    )

    core_errors, core_warnings = validate_core_fields(
        feature_type=feature_type,
        axis=axis,
        bottom=bottom,
        center=center,
        radius=radius,
        height=height,
        cone_angle=cone_angle,
        cone_radius_top=cone_radius_top,
        torus_major_radius=torus_major_radius,
        torus_minor_radius=(
            torus_minor_radius if feature_type == "torus" else None
        ),
    )

    errors.extend(core_errors)
    warnings.extend(core_warnings)

    diagnostic = top_axis_diagnostic(
        bottom=bottom,
        stored_top=stored_top,
        axis=axis,
        height=height,
    )

    point_on_axis = parse_vector3(row.get("point_on_axis"))

    record = PredictionFeatureRecord(
        protocol=ADAPTER_PROTOCOL,
        dataset=dataset,
        sample_id=sample_id,
        instance_id=instance_id,
        instance_uid=(
            f"{dataset}::{sample_id}::{instance_id}"
            if instance_id
            else ""
        ),
        feature_type=feature_type,
        topology_code=topology_code,
        boolean_role=boolean_role,

        axis_x=axis[0] if axis else None,
        axis_y=axis[1] if axis else None,
        axis_z=axis[2] if axis else None,

        center_x=center[0] if center else None,
        center_y=center[1] if center else None,
        center_z=center[2] if center else None,
        center_semantics=center_semantics,

        radius=radius,
        height=height,

        bottom_center_x=bottom[0] if bottom else None,
        bottom_center_y=bottom[1] if bottom else None,
        bottom_center_z=bottom[2] if bottom else None,

        stored_top_center_x=stored_top[0] if stored_top else None,
        stored_top_center_y=stored_top[1] if stored_top else None,
        stored_top_center_z=stored_top[2] if stored_top else None,

        cone_apex_x=cone_apex[0] if cone_apex else None,
        cone_apex_y=cone_apex[1] if cone_apex else None,
        cone_apex_z=cone_apex[2] if cone_apex else None,
        cone_angle=cone_angle,
        cone_radius_bottom=cone_radius_bottom,
        cone_radius_top=(
            cone_radius_top if feature_type == "cone" else None
        ),

        torus_major_radius=torus_major_radius,
        torus_minor_radius=(
            torus_minor_radius if feature_type == "torus" else None
        ),

        prism_type=(
            "" if is_blank(row.get("prism_type"))
            else str(row.get("prism_type")).strip()
        ),
        length=parse_float(row.get("length")),
        width=parse_float(row.get("width")),
        side_n=parse_float(row.get("side_n")),
        side_plane_length=parse_float(row.get("side_plane_length")),
        side_plane_width=parse_float(row.get("side_plane_width")),

        point_on_axis_x=point_on_axis[0] if point_on_axis else None,
        point_on_axis_y=point_on_axis[1] if point_on_axis else None,
        point_on_axis_z=point_on_axis[2] if point_on_axis else None,

        bottom_visible=parse_float(row.get("bottom_visible")),
        top_visible=parse_float(row.get("top_visible")),

        adapter_valid=(len(errors) == 0),
        adapter_errors=";".join(errors),
        adapter_warnings=";".join(warnings),

        top_axis_checked=bool(diagnostic["checked"]),
        top_axis_error=diagnostic["top_axis_error"],
        top_axis_error_over_height=(
            diagnostic["top_axis_error_over_height"]
        ),
        top_bottom_distance=diagnostic["top_bottom_distance"],
        top_bottom_height_abs_error=(
            diagnostic["top_bottom_height_abs_error"]
        ),
        top_axis_alignment_deg=(
            diagnostic["top_axis_alignment_deg"]
        ),
    )

    return record


def adapt_prediction_xlsx(
    dataset: str,
    xlsx_path: Path,
    sample_id: Optional[str] = None,
    sheet_name: str = DEFAULT_SHEET,
) -> Tuple[List[str], List[PredictionFeatureRecord]]:
    headers, rows = read_partfeatures_xlsx(
        xlsx_path=xlsx_path,
        sheet_name=sheet_name,
    )

    resolved_sample_id = (
        str(sample_id).strip()
        if sample_id is not None and str(sample_id).strip()
        else xlsx_path.stem
    )

    records = [
        adapt_one_row(
            dataset=dataset,
            sample_id=resolved_sample_id,
            row=row,
        )
        for row in rows
    ]

    # Sample 内 instance_id 必须唯一。
    ids = [r.instance_id for r in records if r.instance_id]
    seen = set()
    duplicates = set()

    for instance_id in ids:
        if instance_id in seen:
            duplicates.add(instance_id)
        seen.add(instance_id)

    if duplicates:
        duplicate_msg = (
            "duplicate_instance_id_within_sample:"
            + ",".join(sorted(duplicates))
        )

        for r in records:
            if r.instance_id in duplicates:
                r.adapter_valid = False
                r.adapter_errors = (
                    duplicate_msg
                    if not r.adapter_errors
                    else r.adapter_errors + ";" + duplicate_msg
                )

    # dataset + sample_id + instance_id 的 UID 也必须唯一。
    uids = [r.instance_uid for r in records if r.instance_uid]
    if len(uids) != len(set(uids)):
        raise RuntimeError(
            "Duplicate instance_uid detected after adaptation."
        )

    return headers, records


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_json(
    path: Path,
    payload: Dict[str, Any],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    # ensure_ascii=True：
    # 输出纯 ASCII JSON，避免 Windows PowerShell 5.1 默认 ANSI 解码 UTF-8
    # 中文路径时造成非法转义的问题。
    text = json.dumps(
        payload,
        ensure_ascii=True,
        indent=2,
        allow_nan=False,
    )

    path.write_text(
        text,
        encoding="ascii",
    )


def write_csv(
    path: Path,
    records: Sequence[PredictionFeatureRecord],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = list(
        asdict(PredictionFeatureRecord(
            protocol="",
            dataset="",
            sample_id="",
            instance_id="",
            instance_uid="",
            feature_type="",
            topology_code="",
            boolean_role="",
        )).keys()
    )

    with path.open(
        "w",
        encoding="utf-8-sig",
        newline="",
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames,
        )
        writer.writeheader()

        for record in records:
            writer.writerow(asdict(record))


def make_summary(
    headers: Sequence[str],
    records: Sequence[PredictionFeatureRecord],
) -> Dict[str, Any]:
    type_counts: Dict[str, int] = {}
    role_counts: Dict[str, int] = {}

    for r in records:
        type_counts[r.feature_type] = (
            type_counts.get(r.feature_type, 0) + 1
        )
        role_counts[r.boolean_role] = (
            role_counts.get(r.boolean_role, 0) + 1
        )

    valid_count = sum(r.adapter_valid for r in records)
    invalid_count = len(records) - valid_count

    checked = [
        r for r in records
        if r.top_axis_checked
    ]

    top_errors = [
        r.top_axis_error
        for r in checked
        if r.top_axis_error is not None
    ]

    top_rel_errors = [
        r.top_axis_error_over_height
        for r in checked
        if r.top_axis_error_over_height is not None
    ]

    return {
        "protocol": ADAPTER_PROTOCOL,
        "headers": list(headers),
        "record_count": len(records),
        "type_counts": type_counts,
        "role_counts": role_counts,
        "adapter_valid_count": valid_count,
        "adapter_invalid_count": invalid_count,
        "top_axis_checked_count": len(checked),
        "top_axis_error_max": (
            max(top_errors) if top_errors else None
        ),
        "top_axis_error_mean": (
            sum(top_errors) / len(top_errors)
            if top_errors else None
        ),
        "top_axis_error_over_height_max": (
            max(top_rel_errors)
            if top_rel_errors
            else None
        ),
        "top_axis_error_over_height_mean": (
            sum(top_rel_errors) / len(top_rel_errors)
            if top_rel_errors
            else None
        ),
    }


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "RbRM Evaluation V2 prediction XLSX adapter v1.0"
        )
    )

    parser.add_argument(
        "--dataset",
        required=True,
        choices=["CADParser", "DeepCAD"],
    )

    parser.add_argument(
        "--xlsx",
        required=True,
        help="Path to one RbRM PartFeatures XLSX",
    )

    parser.add_argument(
        "--sample-id",
        default=None,
        help=(
            "Optional explicit sample ID. "
            "Default: XLSX filename stem."
        ),
    )

    parser.add_argument(
        "--sheet",
        default=DEFAULT_SHEET,
    )

    parser.add_argument(
        "--out-dir",
        required=True,
    )

    args = parser.parse_args()

    xlsx_path = Path(args.xlsx)
    if not xlsx_path.is_file():
        raise FileNotFoundError(
            f"Prediction XLSX not found: {xlsx_path}"
        )

    out_dir = Path(args.out_dir)
    out_dir.mkdir(
        parents=True,
        exist_ok=True,
    )

    headers, records = adapt_prediction_xlsx(
        dataset=args.dataset,
        xlsx_path=xlsx_path,
        sample_id=args.sample_id,
        sheet_name=args.sheet,
    )

    summary = make_summary(
        headers=headers,
        records=records,
    )

    payload = {
        "protocol": ADAPTER_PROTOCOL,
        "dataset": args.dataset,
        "sample_id": (
            args.sample_id.strip()
            if args.sample_id
            else xlsx_path.stem
        ),
        "source_xlsx": str(xlsx_path.resolve()),
        "summary": summary,
        "records": [
            asdict(r)
            for r in records
        ],
    }

    json_path = out_dir / "prediction_records.json"
    csv_path = out_dir / "prediction_records.csv"

    write_json(
        json_path,
        payload,
    )

    write_csv(
        csv_path,
        records,
    )

    print()
    print("=" * 78)
    print("RbRM Evaluation V2 - PREDICTION ADAPTER v1.0")
    print("=" * 78)
    print(f"dataset               : {args.dataset}")
    print(
        "sample_id             : "
        f"{payload['sample_id']}"
    )
    print(f"source XLSX           : {xlsx_path}")
    print(f"records               : {summary['record_count']}")
    print(f"type counts           : {summary['type_counts']}")
    print(f"role counts           : {summary['role_counts']}")
    print(
        "adapter valid/invalid : "
        f"{summary['adapter_valid_count']}/"
        f"{summary['adapter_invalid_count']}"
    )
    print(
        "top/axis checked      : "
        f"{summary['top_axis_checked_count']}"
    )
    print(
        "max top-axis error    : "
        f"{summary['top_axis_error_max']}"
    )
    print(
        "max rel top-axis err  : "
        f"{summary['top_axis_error_over_height_max']}"
    )
    print("=" * 78)

    bad_records = [
        r for r in records
        if not r.adapter_valid
    ]

    warning_records = [
        r for r in records
        if r.adapter_warnings
    ]

    if bad_records:
        print("\nINVALID RECORDS:")
        for r in bad_records:
            print(
                f"  {r.instance_uid}: "
                f"{r.adapter_errors}"
            )

    if warning_records:
        print("\nWARNING RECORDS:")
        for r in warning_records:
            print(
                f"  {r.instance_uid}: "
                f"{r.adapter_warnings}"
            )

    print(f"\nJSON: {json_path}")
    print(f"CSV : {csv_path}")

    return 1 if bad_records else 0


if __name__ == "__main__":
    raise SystemExit(main())