"""
Evaluate engineering feature instances from identity-joined records.

The evaluator uses:
- exact instance identity;
- GT manifest metric eligibility;
- structured feature parameter comparison.

No geometric matching is performed.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from metric_definitions import (
    PROTOCOL_VERSION as METRIC_PROTOCOL_VERSION,
    compute_instance_metric,
    gt_manifest_metric_eligible,
    prediction_recovered,
    raw_ecd_unit_label,
)


EVALUATOR_PROTOCOL = "RbRM-EV2-FeatureInstanceEvaluator-1.4"
METRICS = ("AAE", "RRE", "ECD", "nECD")


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    if not path.is_file():
        raise FileNotFoundError(f"CSV not found: {path}")

    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"CSV has no header: {path}")
        fields = [str(x).strip() for x in reader.fieldnames]
        rows = [dict(r) for r in reader]

    return fields, rows


def write_csv(
    path: Path,
    rows: Sequence[Dict[str, Any]],
    fieldnames: Sequence[str],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=list(fieldnames),
            extrasaction="ignore",
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json_ascii(path: Path, payload: Mapping[str, Any]) -> None:
    """
    ASCII-only JSON，避免 Windows PowerShell 5.1 对中文路径默认编码错误。
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            payload,
            ensure_ascii=True,
            indent=2,
            allow_nan=False,
        ),
        encoding="ascii",
    )


# ---------------------------------------------------------------------------
# Generic helpers
# ---------------------------------------------------------------------------

def norm_str(value: Any) -> str:
    if value is None:
        return ""
    return str(value).strip()


def parse_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value

    s = norm_str(value).lower()
    if s in {"true", "1", "yes", "y"}:
        return True
    if s in {"false", "0", "no", "n"}:
        return False
    return None


def parse_float(value: Any) -> Optional[float]:
    s = norm_str(value)
    if not s or s.lower() in {"nan", "none", "null"}:
        return None

    try:
        x = float(s)
    except Exception:
        return None

    return x if math.isfinite(x) else None


def mean_or_none(values: Sequence[float]) -> Optional[float]:
    return sum(values) / len(values) if values else None


def median_or_none(values: Sequence[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def min_or_none(values: Sequence[float]) -> Optional[float]:
    return min(values) if values else None


def max_or_none(values: Sequence[float]) -> Optional[float]:
    return max(values) if values else None


def sample_std_or_none(values: Sequence[float]) -> Optional[float]:
    if len(values) < 2:
        return 0.0 if len(values) == 1 else None
    return statistics.stdev(values)


# ---------------------------------------------------------------------------
# Audit / validation
# ---------------------------------------------------------------------------

def validate_identity_join_rows(
    rows: Sequence[Mapping[str, Any]],
    expected_dataset: Optional[str],
) -> List[str]:
    errors: List[str] = []

    seen_uids = set()

    for i, row in enumerate(rows, start=2):
        uid = norm_str(row.get("instance_uid"))
        dataset = norm_str(row.get("dataset"))
        sample_id = norm_str(row.get("sample_id"))
        instance_id = norm_str(row.get("instance_id"))

        if not uid:
            errors.append(f"row {i}: missing instance_uid")
            continue

        if uid in seen_uids:
            errors.append(f"row {i}: duplicate instance_uid={uid}")
        seen_uids.add(uid)

        if expected_dataset and dataset != expected_dataset:
            errors.append(
                f"row {i}: dataset mismatch: {dataset} != {expected_dataset}"
            )

        expected_uid = f"{dataset}::{sample_id}::{instance_id}"
        if uid != expected_uid:
            errors.append(
                f"row {i}: UID component mismatch: {uid} != {expected_uid}"
            )

        if norm_str(row.get("protocol")) != "RbRM-EV2-DirectIdentityJoin-1.0":
            errors.append(
                f"row {i}: unexpected identity-join protocol="
                f"{norm_str(row.get('protocol'))}"
            )

    return errors


def metric_failure_reason(
    row: Mapping[str, Any],
    metric_name: str,
    recovered: bool,
    metric_value: Optional[float],
) -> str:
    if not gt_manifest_metric_eligible(row, metric_name):
        return "GT_NOT_ELIGIBLE"

    if not recovered:
        if parse_bool(row.get("prediction_present")) is not True:
            return "MISSING_PREDICTION"
        if parse_bool(row.get("prediction_adapter_valid")) is not True:
            return "PREDICTION_INVALID"
        if parse_bool(row.get("feature_type_match")) is not True:
            return "FEATURE_TYPE_MISMATCH"
        return "NOT_RECOVERED"

    if metric_value is None:
        return "REQUIRED_PARAMETER_MISSING_OR_INVALID"

    return "EVALUATED"


def fr_failure_reason(
    row: Mapping[str, Any],
    recovered: bool,
) -> str:
    if not gt_manifest_metric_eligible(row, "FR"):
        return "GT_NOT_ELIGIBLE"

    if recovered:
        return "RECOVERED"

    if parse_bool(row.get("prediction_present")) is not True:
        return "MISSING_PREDICTION"

    if parse_bool(row.get("prediction_adapter_valid")) is not True:
        return "PREDICTION_INVALID"

    if parse_bool(row.get("feature_type_match")) is not True:
        return "FEATURE_TYPE_MISMATCH"

    return "NOT_RECOVERED"


# ---------------------------------------------------------------------------
# Per-instance evaluation
# ---------------------------------------------------------------------------

def evaluate_rows(
    rows: Sequence[Mapping[str, Any]],
) -> Tuple[List[Dict[str, Any]], Dict[str, Any], Dict[str, Any]]:
    per_instance: List[Dict[str, Any]] = []

    metric_values: Dict[str, List[float]] = {
        metric: []
        for metric in METRICS
    }

    metric_eligible_counts: Counter = Counter()
    metric_evaluated_counts: Counter = Counter()
    metric_failed_counts: Counter = Counter()

    metric_values_by_type: Dict[str, Dict[str, List[float]]] = {
        metric: defaultdict(list)
        for metric in METRICS
    }

    fr_gt_eligible = 0
    fr_recovered = 0
    fr_failed = 0

    fr_by_type: Dict[str, Dict[str, int]] = defaultdict(
        lambda: {
            "gt_eligible_count": 0,
            "recovered_count": 0,
            "failed_count": 0,
        }
    )

    join_status_counts: Counter = Counter()
    role_mismatch_count = 0

    audit_errors: List[str] = []
    audit_warnings: List[str] = []

    for row in rows:
        uid = norm_str(row.get("instance_uid"))
        feature_type = norm_str(row.get("gt_feature_type")).lower()
        boolean_role = norm_str(row.get("gt_boolean_role"))
        join_status = norm_str(row.get("join_status"))

        join_status_counts[join_status] += 1

        recovered = prediction_recovered(row)

        role_match = parse_bool(row.get("boolean_role_match"))
        if (
            parse_bool(row.get("prediction_present")) is True
            and role_match is False
        ):
            role_mismatch_count += 1
            audit_warnings.append(
                f"{uid}: Boolean role mismatch is audit-only and does not gate metrics."
            )

        # ----------------------------
        # FR
        # ----------------------------
        fr_eligible = gt_manifest_metric_eligible(row, "FR")
        fr_reason = fr_failure_reason(row, recovered)

        if fr_eligible:
            fr_gt_eligible += 1
            fr_by_type[feature_type]["gt_eligible_count"] += 1

            if recovered:
                fr_recovered += 1
                fr_by_type[feature_type]["recovered_count"] += 1
            else:
                fr_failed += 1
                fr_by_type[feature_type]["failed_count"] += 1

        # ----------------------------
        # AAE/RRE/ECD/nECD
        # ----------------------------
        metric_result: Dict[str, Optional[float]] = {}
        metric_reason: Dict[str, str] = {}
        metric_eligible: Dict[str, bool] = {}

        for metric_name in METRICS:
            eligible = gt_manifest_metric_eligible(row, metric_name)
            metric_eligible[metric_name] = eligible

            if eligible:
                metric_eligible_counts[metric_name] += 1

            value = compute_instance_metric(row, metric_name)
            metric_result[metric_name] = value

            reason = metric_failure_reason(
                row=row,
                metric_name=metric_name,
                recovered=recovered,
                metric_value=value,
            )
            metric_reason[metric_name] = reason

            if eligible:
                if value is not None:
                    metric_evaluated_counts[metric_name] += 1
                    metric_values[metric_name].append(float(value))
                    metric_values_by_type[metric_name][feature_type].append(
                        float(value)
                    )
                else:
                    metric_failed_counts[metric_name] += 1

                    # 如果实例已经 recovered，但 metric 值仍然算不出来，
                    # 说明 GT eligibility / adapter / evaluator 之间存在协议矛盾。
                    # production evaluation 不应静默忽略。
                    if recovered:
                        audit_errors.append(
                            f"{uid}: {metric_name} is GT-eligible and recovered "
                            "but required metric parameters are missing/invalid."
                        )

        per_instance.append({
            "evaluator_protocol": EVALUATOR_PROTOCOL,
            "metric_protocol": METRIC_PROTOCOL_VERSION,
            "instance_uid": uid,
            "dataset": norm_str(row.get("dataset")),
            "sample_id": norm_str(row.get("sample_id")),
            "instance_id": norm_str(row.get("instance_id")),
            "feature_type": feature_type,
            "boolean_role": boolean_role,

            "join_status": join_status,
            "prediction_present": norm_str(row.get("prediction_present")),
            "prediction_adapter_valid": norm_str(
                row.get("prediction_adapter_valid")
            ),
            "feature_type_match": norm_str(row.get("feature_type_match")),
            "boolean_role_match": norm_str(row.get("boolean_role_match")),

            "recovered_for_fr": recovered if fr_eligible else "",
            "fr_eligible": fr_eligible,
            "fr_status": fr_reason,

            "aae_eligible": metric_eligible["AAE"],
            "aae_deg": metric_result["AAE"],
            "aae_status": metric_reason["AAE"],

            "rre_eligible": metric_eligible["RRE"],
            "rre_ratio": metric_result["RRE"],
            "rre_percent": (
                100.0 * metric_result["RRE"]
                if metric_result["RRE"] is not None
                else None
            ),
            "rre_status": metric_reason["RRE"],

            "ecd_eligible": metric_eligible["ECD"],
            "ecd_native": metric_result["ECD"],
            "ecd_status": metric_reason["ECD"],

            "necd_eligible": metric_eligible["nECD"],
            "necd": metric_result["nECD"],
            "necd_status": metric_reason["nECD"],

            "gt_model_bbox_diag": parse_float(
                row.get("gt_model_bbox_diag")
            ),

            "gt_multiface_provenance": norm_str(
                row.get("gt_multiface_provenance")
            ),
            "gt_accepted_face_count": norm_str(
                row.get("gt_accepted_face_count")
            ),
            "gt_parameter_source": norm_str(
                row.get("gt_parameter_source")
            ),

            "pred_center_semantics": norm_str(
                row.get("pred_center_semantics")
            ),
            "pred_top_axis_error": parse_float(
                row.get("pred_top_axis_error")
            ),
            "pred_top_axis_error_over_height": parse_float(
                row.get("pred_top_axis_error_over_height")
            ),
            "pred_adapter_errors": norm_str(
                row.get("pred_adapter_errors")
            ),
            "pred_adapter_warnings": norm_str(
                row.get("pred_adapter_warnings")
            ),
        })

    # -----------------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------------

    if fr_gt_eligible > 0:
        fr_percent = 100.0 * fr_recovered / fr_gt_eligible
    else:
        fr_percent = None

    metric_summary: Dict[str, Any] = {}

    for metric_name in METRICS:
        values = metric_values[metric_name]

        metric_summary[metric_name] = {
            "gt_eligible_count": int(metric_eligible_counts[metric_name]),
            "evaluated_count": int(metric_evaluated_counts[metric_name]),
            "skipped_or_failed_count": int(metric_failed_counts[metric_name]),
            "mean": mean_or_none(values),
            "median": median_or_none(values),
            "std_sample": sample_std_or_none(values),
            "min": min_or_none(values),
            "max": max_or_none(values),
            "by_type": {
                feature_type: {
                    "evaluated_count": len(type_values),
                    "mean": mean_or_none(type_values),
                    "median": median_or_none(type_values),
                    "min": min_or_none(type_values),
                    "max": max_or_none(type_values),
                }
                for feature_type, type_values in sorted(
                    metric_values_by_type[metric_name].items()
                )
            },
        }

    fr_by_type_summary = {}

    for feature_type, counts in sorted(fr_by_type.items()):
        denom = counts["gt_eligible_count"]
        recovered_count = counts["recovered_count"]
        fr_by_type_summary[feature_type] = {
            **counts,
            "fr_percent": (
                100.0 * recovered_count / denom
                if denom > 0
                else None
            ),
        }

    datasets = sorted({
        norm_str(row.get("dataset"))
        for row in rows
        if norm_str(row.get("dataset"))
    })

    sample_ids = sorted({
        norm_str(row.get("sample_id"))
        for row in rows
        if norm_str(row.get("sample_id"))
    })

    if len(datasets) == 1:
        dataset_name = datasets[0]
        ecd_unit = raw_ecd_unit_label(dataset_name)
    else:
        dataset_name = "MULTI_DATASET"
        ecd_unit = (
            "mixed dataset-native units; raw ECD must not be pooled "
            "across datasets"
        )

    summary = {
        "evaluator_protocol": EVALUATOR_PROTOCOL,
        "metric_protocol": METRIC_PROTOCOL_VERSION,
        "dataset": dataset_name,
        "sample_count": len(sample_ids),
        "instance_count": len(rows),
        "sample_ids": sample_ids,
        "join_status_counts": dict(join_status_counts),
        "feature_reconstruction": {
            "gt_eligible_count": fr_gt_eligible,
            "recovered_count": fr_recovered,
            "failed_count": fr_failed,
            "FR_percent": fr_percent,
            "by_type": fr_by_type_summary,
        },
        "metrics": metric_summary,
        "units": {
            "AAE": "degree",
            "RRE": "dimensionless ratio",
            "RRE_percent": "percent representation of the same ratio",
            "ECD": ecd_unit,
            "nECD": "dimensionless",
            "FR": "percent",
        },
        "aggregation_policy": {
            "AAE": (
                "arithmetic mean over recovered AAE-eligible engineering "
                "feature instances whose GT manifest provides a unique CAD-native axis"
            ),
            "RRE": (
                "arithmetic mean over recovered RRE-eligible engineering "
                "feature instances whose GT manifest provides a unique single radius"
            ),
            "ECD": (
                "arithmetic mean over recovered ECD-eligible engineering "
                "feature instances with unique CAD-native center semantics, "
                "within the same dataset only"
            ),
            "nECD": (
                "arithmetic mean over recovered ECD-eligible engineering "
                "feature instances with unique CAD-native center semantics, "
                "after normalization by each GT model bounding-box diagonal"
            ),
            "FR": (
                "100 * recovered FR-eligible GT instances / all "
                "FR-eligible GT instances"
            ),
        },
    }

    audit = {
        "evaluator_protocol": EVALUATOR_PROTOCOL,
        "metric_protocol": METRIC_PROTOCOL_VERSION,
        "uses_geometric_matching": False,
        "uses_hungarian": False,
        "uses_fixed_distance_threshold": False,
        "evaluated_entity": "engineering_feature_instance",
        "identity_key": "(dataset, sample_id, instance_id) / instance_uid",
        "gt_parameter_eligibility_source": "RbRM-EV2-GTManifest-1.4",
        "gt_parameter_uniqueness_gates_parameter_metrics": True,
        "boolean_role_gates_metrics": False,
        "role_mismatch_count": role_mismatch_count,
        "errors": audit_errors,
        "warnings": audit_warnings,
        "result": "PASS" if not audit_errors else "FAIL",
    }

    return per_instance, summary, audit


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Engineering feature instance evaluator"
        )
    )

    parser.add_argument(
        "--identity-join",
        required=True,
        help="identity_join.csv",
    )

    parser.add_argument(
        "--dataset",
        default=None,
        choices=["CADParser", "DeepCAD"],
        help="Optional expected dataset name for strict validation.",
    )

    parser.add_argument(
        "--out-dir",
        required=True,
    )

    parser.add_argument(
        "--fail-on-audit-error",
        action="store_true",
        help="Return non-zero if protocol/metric audit errors are found.",
    )

    args = parser.parse_args()

    join_path = Path(args.identity_join)
    out_dir = Path(args.out_dir)

    _, rows = read_csv_rows(join_path)

    if not rows:
        raise RuntimeError("identity_join.csv contains no instances.")

    identity_errors = validate_identity_join_rows(
        rows=rows,
        expected_dataset=args.dataset,
    )

    per_instance, summary, audit = evaluate_rows(rows)

    if identity_errors:
        audit["errors"] = identity_errors + list(audit["errors"])
        audit["result"] = "FAIL"

    out_dir.mkdir(parents=True, exist_ok=True)

    per_instance_path = out_dir / "per_instance_metrics.csv"
    summary_path = out_dir / "dataset_summary.json"
    audit_path = out_dir / "evaluation_audit.json"

    per_instance_fields = [
        "evaluator_protocol",
        "metric_protocol",
        "instance_uid",
        "dataset",
        "sample_id",
        "instance_id",
        "feature_type",
        "boolean_role",

        "join_status",
        "prediction_present",
        "prediction_adapter_valid",
        "feature_type_match",
        "boolean_role_match",

        "fr_eligible",
        "recovered_for_fr",
        "fr_status",

        "aae_eligible",
        "aae_deg",
        "aae_status",

        "rre_eligible",
        "rre_ratio",
        "rre_percent",
        "rre_status",

        "ecd_eligible",
        "ecd_native",
        "ecd_status",

        "necd_eligible",
        "necd",
        "necd_status",

        "gt_model_bbox_diag",
        "gt_multiface_provenance",
        "gt_accepted_face_count",
        "gt_parameter_source",

        "pred_center_semantics",
        "pred_top_axis_error",
        "pred_top_axis_error_over_height",
        "pred_adapter_errors",
        "pred_adapter_warnings",
    ]

    write_csv(
        per_instance_path,
        per_instance,
        per_instance_fields,
    )

    write_json_ascii(
        summary_path,
        summary,
    )

    write_json_ascii(
        audit_path,
        audit,
    )

    print()
    print("=" * 78)
    print("RbRM ENGINEERING FEATURE INSTANCE EVALUATION")
    print("=" * 78)
    print(f"dataset               : {summary['dataset']}")
    print(f"samples               : {summary['sample_count']}")
    print(f"instances             : {summary['instance_count']}")
    print(f"join status           : {summary['join_status_counts']}")
    print()

    fr = summary["feature_reconstruction"]
    print(
        "FR                    : "
        f"{fr['FR_percent']}% "
        f"({fr['recovered_count']}/{fr['gt_eligible_count']})"
    )

    for metric_name in METRICS:
        m = summary["metrics"][metric_name]
        print(
            f"{metric_name:<22}: "
            f"mean={m['mean']} "
            f"eligible={m['gt_eligible_count']} "
            f"evaluated={m['evaluated_count']} "
            f"failed/skipped={m['skipped_or_failed_count']}"
        )

    print()
    print(f"ECD unit              : {summary['units']['ECD']}")
    print(f"audit                 : {audit['result']}")
    print(f"audit errors          : {len(audit['errors'])}")
    print(f"audit warnings        : {len(audit['warnings'])}")
    print("=" * 78)

    if audit["errors"]:
        print("\nAUDIT ERRORS:")
        for item in audit["errors"]:
            print("  -", item)

    if audit["warnings"]:
        print("\nAUDIT WARNINGS:")
        for item in audit["warnings"]:
            print("  -", item)

    print(f"\nPER INSTANCE: {per_instance_path}")
    print(f"SUMMARY     : {summary_path}")
    print(f"AUDIT       : {audit_path}")

    if args.fail_on_audit_error and audit["errors"]:
        return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
