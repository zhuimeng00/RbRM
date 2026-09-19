"""
Metric definitions for engineering feature evaluation.

Defines FR, AAE, RRE, ECD and nECD eligibility rules and formulas.
The evaluator uses persistent feature identities and GT manifest
eligibility information.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Set, Tuple


PROTOCOL_VERSION = "RbRM-EV2-MetricDefinitions-1.4"

EV2_FEATURE_TYPES: Set[str] = {
    "cylinder",
    "cone",
    "sphere",
    "torus",
    "prism",
}

AAE_TYPES: Set[str] = {
    "cylinder",
    "cone",
    "torus",
}

RRE_TYPES: Set[str] = {
    "cylinder",
    "sphere",
}

ECD_TYPES: Set[str] = {
    "cylinder",
    "cone",
    "sphere",
    "torus",
}

FR_TYPES: Set[str] = set(EV2_FEATURE_TYPES)


# Cross-task methods whose final output does not preserve persistent
# engineering-feature-instance identity. Their feature-instance metrics
# are N/A in Evaluation.
CROSS_TASK_FEATURE_METRICS_NA = {
    "DeepCAD",
    "CAD-Recode",
    "ComplexGen",
    "Point2CAD",
}


@dataclass(frozen=True)
class MetricSpec:
    name: str
    eligible_types: Tuple[str, ...]
    unit: str
    aggregation: str
    definition: str


METRIC_SPECS: Dict[str, MetricSpec] = {
    "AAE": MetricSpec(
        name="AAE",
        eligible_types=tuple(sorted(AAE_TYPES)),
        unit="degree",
        aggregation="mean_over_recovered_metric_eligible_instances",
        definition="acos(abs(dot(unit(axis_pred), unit(axis_gt)))) * 180/pi",
    ),
    "RRE": MetricSpec(
        name="RRE",
        eligible_types=tuple(sorted(RRE_TYPES)),
        unit="dimensionless_ratio",
        aggregation="mean_over_recovered_metric_eligible_instances",
        definition="abs(radius_pred-radius_gt)/radius_gt",
    ),
    "ECD": MetricSpec(
        name="ECD",
        eligible_types=tuple(sorted(ECD_TYPES)),
        unit="dataset_native_coordinate_unit",
        aggregation="mean_over_recovered_metric_eligible_instances",
        definition="euclidean_distance(center_pred, center_gt)",
    ),
    "nECD": MetricSpec(
        name="nECD",
        eligible_types=tuple(sorted(ECD_TYPES)),
        unit="dimensionless",
        aggregation="mean_over_recovered_metric_eligible_instances",
        definition="ECD/model_bbox_diag_gt",
    ),
    "FR": MetricSpec(
        name="FR",
        eligible_types=tuple(sorted(FR_TYPES)),
        unit="percent",
        aggregation="100*N_recovered/N_gt_eligible",
        definition=(
            "recovered iff exact identity join + adapter_valid + type_match "
            "+ type-required finite structured parameters"
        ),
    ),
}


# ---------------------------------------------------------------------------
# Generic parsing
# ---------------------------------------------------------------------------

def normalize_type(value: Any) -> str:
    if value is None:
        return ""
    return str(value).strip().lower()


def parse_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value

    if value is None:
        return None

    s = str(value).strip().lower()
    if s in {"true", "1", "yes", "y"}:
        return True
    if s in {"false", "0", "no", "n"}:
        return False
    return None


def parse_float(value: Any) -> Optional[float]:
    if value is None:
        return None

    s = str(value).strip()
    if s == "" or s.lower() in {"nan", "none", "null"}:
        return None

    try:
        x = float(s)
    except Exception:
        return None

    return x if math.isfinite(x) else None


def vector3_from_prefixed_row(
    row: Mapping[str, Any],
    prefix: str,
    stem: str,
) -> Optional[Tuple[float, float, float]]:
    """
    Example:
      prefix="gt_", stem="axis"   -> gt_axis_x/y/z
      prefix="pred_", stem="center" -> pred_center_x/y/z
    """
    vals = tuple(
        parse_float(row.get(f"{prefix}{stem}_{axis}"))
        for axis in ("x", "y", "z")
    )

    if any(v is None for v in vals):
        return None

    return (float(vals[0]), float(vals[1]), float(vals[2]))


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def vec_norm(v: Sequence[float]) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


def unit_vector(
    v: Sequence[float],
) -> Optional[Tuple[float, float, float]]:
    n = vec_norm(v)

    if not math.isfinite(n) or n <= 1e-15:
        return None

    return (
        float(v[0]) / n,
        float(v[1]) / n,
        float(v[2]) / n,
    )


def dot(
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    return sum(float(a[i]) * float(b[i]) for i in range(3))


def distance3(
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    return math.sqrt(
        sum(
            (float(a[i]) - float(b[i])) ** 2
            for i in range(3)
        )
    )


# ---------------------------------------------------------------------------
# Metric formulas
# ---------------------------------------------------------------------------

def aae_deg(
    axis_pred: Sequence[float],
    axis_gt: Sequence[float],
) -> Optional[float]:
    """
    Sign-invariant Axis Angular Error in degrees.
    """
    up = unit_vector(axis_pred)
    ug = unit_vector(axis_gt)

    if up is None or ug is None:
        return None

    c = abs(dot(up, ug))
    c = min(1.0, max(-1.0, c))

    value = math.degrees(math.acos(c))
    return value if math.isfinite(value) else None


def rre(
    radius_pred: float,
    radius_gt: float,
) -> Optional[float]:
    """
    Relative Radius Error as a dimensionless ratio.
    """
    rp = parse_float(radius_pred)
    rg = parse_float(radius_gt)

    if rp is None or rg is None:
        return None

    if rg <= 0:
        return None

    value = abs(rp - rg) / rg
    return value if math.isfinite(value) else None


def ecd(
    center_pred: Sequence[float],
    center_gt: Sequence[float],
) -> Optional[float]:
    value = distance3(center_pred, center_gt)
    return value if math.isfinite(value) else None


def normalized_ecd(
    ecd_value: float,
    model_bbox_diag_gt: float,
) -> Optional[float]:
    e = parse_float(ecd_value)
    diag = parse_float(model_bbox_diag_gt)

    if e is None or diag is None:
        return None

    if diag <= 0:
        return None

    value = e / diag
    return value if math.isfinite(value) else None


# ---------------------------------------------------------------------------
# Eligibility
# ---------------------------------------------------------------------------

def metric_type_eligible(
    metric_name: str,
    feature_type: str,
) -> bool:
    feature_type = normalize_type(feature_type)

    if metric_name == "AAE":
        return feature_type in AAE_TYPES
    if metric_name == "RRE":
        return feature_type in RRE_TYPES
    if metric_name in {"ECD", "nECD"}:
        return feature_type in ECD_TYPES
    if metric_name == "FR":
        return feature_type in FR_TYPES

    raise KeyError(f"Unknown metric: {metric_name}")


def gt_manifest_metric_eligible(
    row: Mapping[str, Any],
    metric_name: str,
) -> bool:
    """
    GT manifest v1.4 提供 eligible_fr/aae/rre/ecd 字段。
    Evaluation 同时要求：
      1) feature type 属于冻结的 metric type set；
      2) manifest 中对应参数具有唯一 CAD-native GT；
      3) manifest eligibility 为 True。

    对 nECD 使用 ECD eligibility。
    """
    feature_type = normalize_type(
        row.get("gt_feature_type", row.get("feature_type"))
    )

    if not metric_type_eligible(metric_name, feature_type):
        return False

    if metric_name == "AAE":
        field = "eligible_aae"
    elif metric_name == "RRE":
        field = "eligible_rre"
    elif metric_name in {"ECD", "nECD"}:
        field = "eligible_ecd"
    elif metric_name == "FR":
        field = "eligible_fr"
    else:
        raise KeyError(metric_name)

    value = row.get(f"gt_{field}", row.get(field))
    parsed = parse_bool(value)

    # 若 manifest eligibility 字段缺失，不静默推断。
    return parsed is True


# ---------------------------------------------------------------------------
# Recovery semantics
# ---------------------------------------------------------------------------

def prediction_recovered(
    join_row: Mapping[str, Any],
) -> bool:
    """
    FR recovery gate.

    Required:
      - prediction_present == True
      - prediction_adapter_valid == True
      - feature_type_match == True

    boolean_role_match is intentionally NOT part of this gate.
    """
    present = parse_bool(join_row.get("prediction_present"))
    adapter_valid = parse_bool(
        join_row.get("prediction_adapter_valid")
    )
    type_match = parse_bool(
        join_row.get("feature_type_match")
    )

    return (
        present is True
        and adapter_valid is True
        and type_match is True
    )


def role_mismatch_is_metric_failure(
    join_row: Mapping[str, Any],
) -> bool:
    """
    Frozen Evaluation policy:
    Boolean role/topology is metadata, not a feature-metric recovery gate.
    """
    return False


# ---------------------------------------------------------------------------
# Metric value extraction from identity_join.csv
# ---------------------------------------------------------------------------

def compute_instance_metric(
    join_row: Mapping[str, Any],
    metric_name: str,
) -> Optional[float]:
    """
    Compute one metric value from one direct-identity-join row.

    Returns None when:
      - GT instance is not eligible;
      - prediction is not recovered;
      - required values are missing/invalid.

    FR is intentionally NOT returned here as a per-instance numeric error.
    Use prediction_recovered + FR denominator accounting in the evaluator.
    """
    if metric_name == "FR":
        raise ValueError(
            "FR is an aggregate recovery rate; "
            "use prediction_recovered() for each GT instance."
        )

    if not gt_manifest_metric_eligible(join_row, metric_name):
        return None

    if not prediction_recovered(join_row):
        return None

    if metric_name == "AAE":
        gt_axis = vector3_from_prefixed_row(
            join_row,
            "gt_",
            "axis",
        )
        pred_axis = vector3_from_prefixed_row(
            join_row,
            "pred_",
            "axis",
        )

        if gt_axis is None or pred_axis is None:
            return None

        return aae_deg(
            axis_pred=pred_axis,
            axis_gt=gt_axis,
        )

    if metric_name == "RRE":
        gt_radius = parse_float(
            join_row.get("gt_radius")
        )
        pred_radius = parse_float(
            join_row.get("pred_radius")
        )

        if gt_radius is None or pred_radius is None:
            return None

        return rre(
            radius_pred=pred_radius,
            radius_gt=gt_radius,
        )

    if metric_name in {"ECD", "nECD"}:
        gt_center = vector3_from_prefixed_row(
            join_row,
            "gt_",
            "center",
        )
        pred_center = vector3_from_prefixed_row(
            join_row,
            "pred_",
            "center",
        )

        if gt_center is None or pred_center is None:
            return None

        raw_ecd = ecd(
            center_pred=pred_center,
            center_gt=gt_center,
        )

        if raw_ecd is None:
            return None

        if metric_name == "ECD":
            return raw_ecd

        bbox_diag = parse_float(
            join_row.get("gt_model_bbox_diag")
        )
        if bbox_diag is None:
            return None

        return normalized_ecd(
            ecd_value=raw_ecd,
            model_bbox_diag_gt=bbox_diag,
        )

    raise KeyError(f"Unknown metric: {metric_name}")


# ---------------------------------------------------------------------------
# Protocol metadata for paper / evaluator audit
# ---------------------------------------------------------------------------

def raw_ecd_unit_label(dataset: str) -> str:
    """
    Conservative unit labels.

    CADParser:
      Use "mm" in the paper ONLY after dataset/source STEP units have been
      explicitly verified. The evaluator itself treats ECD as raw native unit.

    DeepCAD:
      normalized/native coordinate unit.
    """
    dataset = str(dataset).strip()

    if dataset == "DeepCAD":
        return "normalized/native coordinate unit"

    if dataset == "CADParser":
        return "native coordinate unit (mm only after unit verification)"

    return "dataset native coordinate unit"


def protocol_dict() -> Dict[str, Any]:
    return {
        "protocol": PROTOCOL_VERSION,
        "evaluated_entity": "engineering_feature_instance",
        "correspondence": {
            "key": [
                "dataset",
                "sample_id",
                "instance_id",
            ],
            "equivalent_key": "instance_uid",
            "uses_geometric_matching": False,
            "uses_hungarian": False,
            "uses_fixed_distance_threshold": False,
        },
        "feature_types": sorted(EV2_FEATURE_TYPES),
        "gt_parameter_eligibility": {
            "manifest_schema": "RbRM-EV2-GTManifest-1.4",
            "zero_marginal_face_pruning": (
                "Direct source-CAD faces that add exactly zero new supported "
                "feature points are removed before GT parameter consolidation."
            ),
            "parameter_specific_uniqueness": {
                "AAE": "requires a unique source-CAD axis",
                "RRE": "requires a unique source-CAD single radius",
                "ECD_nECD": "requires a unique source-CAD center/axis-line center",
            },
            "prediction_matching_use": False,
        },
        "metric_specs": {
            name: {
                "eligible_types": list(spec.eligible_types),
                "unit": spec.unit,
                "aggregation": spec.aggregation,
                "definition": spec.definition,
            }
            for name, spec in METRIC_SPECS.items()
        },
        "recovery_gate": [
            "exact identity join/prediction present",
            "prediction_adapter_valid == True",
            "feature_type_match == True",
        ],
        "boolean_role_gates_feature_metrics": False,
        "ecd_normalization": {
            "nECD": "ECD / source_GT_model_bbox_diagonal",
            "scale_independent": True,
        },
        "cross_task_baseline_feature_metrics": {
            method: {
                "AAE": "N/A",
                "RRE": "N/A",
                "ECD": "N/A",
                "nECD": "N/A",
                "FR": "N/A",
            }
            for method in sorted(CROSS_TASK_FEATURE_METRICS_NA)
        },
    }


# ---------------------------------------------------------------------------
# Self tests
# ---------------------------------------------------------------------------

def _assert_close(
    a: float,
    b: float,
    tol: float = 1e-10,
) -> None:
    if abs(a - b) > tol:
        raise AssertionError(
            f"{a} != {b} within tol={tol}"
        )


def run_self_tests() -> None:
    # AAE sign invariance
    _assert_close(
        aae_deg((1, 0, 0), (-1, 0, 0)),
        0.0,
    )
    _assert_close(
        aae_deg((1, 0, 0), (0, 1, 0)),
        90.0,
    )

    # RRE
    _assert_close(
        rre(11.0, 10.0),
        0.1,
    )

    # ECD / nECD
    _assert_close(
        ecd((0, 0, 0), (3, 4, 0)),
        5.0,
    )
    _assert_close(
        normalized_ecd(5.0, 10.0),
        0.5,
    )

    # Eligibility sets
    assert metric_type_eligible("AAE", "cylinder")
    assert metric_type_eligible("AAE", "cone")
    assert metric_type_eligible("AAE", "torus")
    assert not metric_type_eligible("AAE", "sphere")
    assert not metric_type_eligible("AAE", "prism")

    assert metric_type_eligible("RRE", "cylinder")
    assert metric_type_eligible("RRE", "sphere")
    assert not metric_type_eligible("RRE", "cone")
    assert not metric_type_eligible("RRE", "torus")
    assert not metric_type_eligible("RRE", "prism")

    assert metric_type_eligible("ECD", "cylinder")
    assert metric_type_eligible("ECD", "cone")
    assert metric_type_eligible("ECD", "sphere")
    assert metric_type_eligible("ECD", "torus")
    assert not metric_type_eligible("ECD", "prism")

    for feature_type in EV2_FEATURE_TYPES:
        assert metric_type_eligible("FR", feature_type)

    # Recovery gate: role mismatch must not invalidate feature recovery.
    row = {
        "prediction_present": "True",
        "prediction_adapter_valid": "True",
        "feature_type_match": "True",
        "boolean_role_match": "False",
    }
    assert prediction_recovered(row)
    assert not role_mismatch_is_metric_failure(row)

    # Manifest parameter-specific ineligibility must gate a metric even when
    # the feature type itself belongs to that metric.
    row_ineligible = {
        "gt_feature_type": "cylinder",
        "gt_eligible_rre": "False",
    }
    assert not gt_manifest_metric_eligible(row_ineligible, "RRE")

    # Missing prediction must fail FR recovery.
    row_missing = dict(row)
    row_missing["prediction_present"] = "False"
    assert not prediction_recovered(row_missing)

    print("=" * 78)
    print("RbRM Evaluation - METRIC DEFINITIONS SELF TEST")
    print("=" * 78)
    print("AAE sign invariance                 : PASS")
    print("RRE definition                      : PASS")
    print("ECD / nECD definition               : PASS")
    print("metric type eligibility             : PASS")
    print("FR recovery gate                    : PASS")
    print("Boolean role non-gating policy      : PASS")
    print("RESULT                              : PASS")
    print("=" * 78)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="RbRM Evaluation metric definitions v1.1"
    )

    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Run deterministic metric-definition tests.",
    )

    parser.add_argument(
        "--print-protocol",
        action="store_true",
        help="Print frozen protocol as JSON.",
    )

    parser.add_argument(
        "--write-protocol",
        default=None,
        help="Optional path to write ASCII-only protocol JSON.",
    )

    args = parser.parse_args()

    if not (
        args.self_test
        or args.print_protocol
        or args.write_protocol
    ):
        parser.print_help()
        return 0

    if args.self_test:
        run_self_tests()

    payload = protocol_dict()

    if args.print_protocol:
        print(
            json.dumps(
                payload,
                ensure_ascii=True,
                indent=2,
                allow_nan=False,
            )
        )

    if args.write_protocol:
        from pathlib import Path

        path = Path(args.write_protocol)
        path.parent.mkdir(
            parents=True,
            exist_ok=True,
        )
        path.write_text(
            json.dumps(
                payload,
                ensure_ascii=True,
                indent=2,
                allow_nan=False,
            ),
            encoding="ascii",
        )
        print(f"protocol: {path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
