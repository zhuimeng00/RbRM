"""
Join GT feature instances and prediction records by persistent identity.

The join stage:
- uses instance_uid as the primary key;
- keeps GT instances with missing predictions;
- reports unknown predictions and consistency checks.

No geometric matching is performed.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


PROTOCOL = "RbRM-EV2-DirectIdentityJoin-1.0"
EXPECTED_GT_MANIFEST_SCHEMA = "RbRM-EV2-GTManifest-1.4"


# ---------------------------------------------------------------------------
# Basic helpers
# ---------------------------------------------------------------------------

def norm_str(value: object) -> str:
    if value is None:
        return ""
    return str(value).strip()


def norm_bool(value: object) -> str:
    s = norm_str(value).lower()
    if s in {"true", "1", "yes", "y"}:
        return "True"
    if s in {"false", "0", "no", "n"}:
        return "False"
    return norm_str(value)


def read_csv_rows(path: Path) -> Tuple[List[str], List[Dict[str, str]]]:
    if not path.is_file():
        raise FileNotFoundError(f"CSV not found: {path}")

    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"CSV has no header: {path}")
        fieldnames = [str(x).strip() for x in reader.fieldnames]
        rows = [dict(r) for r in reader]

    return fieldnames, rows


def validate_gt_manifest_schema(rows: Sequence[Dict[str, str]]) -> None:
    """Require the GT manifest schema used by the released evaluation protocol."""
    if not rows:
        raise RuntimeError("GT manifest is empty.")

    versions = {norm_str(row.get("schema_version")) for row in rows}
    if "" in versions:
        raise RuntimeError("GT manifest contains rows without schema_version.")

    if versions != {EXPECTED_GT_MANIFEST_SCHEMA}:
        raise RuntimeError(
            "Unexpected GT manifest schema version(s): "
            f"{sorted(versions)}. "
            f"Expected only {EXPECTED_GT_MANIFEST_SCHEMA}."
        )


def make_uid(dataset: str, sample_id: str, instance_id: str) -> str:
    return f"{dataset}::{sample_id}::{instance_id}"


def row_uid(row: Dict[str, str]) -> str:
    uid = norm_str(row.get("instance_uid"))
    if uid:
        return uid

    dataset = norm_str(row.get("dataset"))
    sample_id = norm_str(row.get("sample_id"))
    instance_id = norm_str(row.get("instance_id"))
    if dataset and sample_id and instance_id:
        return make_uid(dataset, sample_id, instance_id)

    return ""


def validate_uid_components(
    uid: str,
    dataset: str,
    sample_id: str,
    instance_id: str,
) -> bool:
    if not uid or not dataset or not sample_id or not instance_id:
        return False
    return uid == make_uid(dataset, sample_id, instance_id)


def index_unique(
    rows: Sequence[Dict[str, str]],
    label: str,
) -> Tuple[Dict[str, Dict[str, str]], Dict[str, List[int]], List[int]]:
    """
    Returns:
      unique_index: uid -> row
      duplicate_positions: uid -> row indices (0-based)
      missing_uid_positions: row indices
    """
    positions: Dict[str, List[int]] = defaultdict(list)
    missing_uid_positions: List[int] = []

    for i, row in enumerate(rows):
        uid = row_uid(row)
        if not uid:
            missing_uid_positions.append(i)
            continue
        positions[uid].append(i)

    duplicate_positions = {
        uid: idxs
        for uid, idxs in positions.items()
        if len(idxs) > 1
    }

    if duplicate_positions:
        details = "; ".join(
            f"{uid} -> rows {[i + 2 for i in idxs]}"
            for uid, idxs in sorted(duplicate_positions.items())
        )
        raise RuntimeError(
            f"Duplicate {label} instance_uid detected. "
            f"Direct identity join requires unique keys. {details}"
        )

    if missing_uid_positions:
        excel_rows = [i + 2 for i in missing_uid_positions]
        raise RuntimeError(
            f"{label} records with missing instance_uid/key at CSV rows "
            f"{excel_rows}"
        )

    unique_index = {
        uid: rows[idxs[0]]
        for uid, idxs in positions.items()
    }
    return unique_index, duplicate_positions, missing_uid_positions


def filter_gt_rows(
    rows: Sequence[Dict[str, str]],
    dataset: Optional[str],
    sample_id: Optional[str],
) -> List[Dict[str, str]]:
    out: List[Dict[str, str]] = []

    for row in rows:
        if dataset is not None:
            if norm_str(row.get("dataset")) != dataset:
                continue
        if sample_id is not None:
            if norm_str(row.get("sample_id")) != sample_id:
                continue
        out.append(dict(row))

    return out


def filter_pred_rows(
    rows: Sequence[Dict[str, str]],
    dataset: Optional[str],
    sample_id: Optional[str],
) -> List[Dict[str, str]]:
    out: List[Dict[str, str]] = []

    for row in rows:
        if dataset is not None:
            if norm_str(row.get("dataset")) != dataset:
                continue
        if sample_id is not None:
            if norm_str(row.get("sample_id")) != sample_id:
                continue
        out.append(dict(row))

    return out


def prefix_row(
    prefix: str,
    row: Optional[Dict[str, str]],
    fieldnames: Sequence[str],
) -> Dict[str, str]:
    if row is None:
        return {f"{prefix}{k}": "" for k in fieldnames}

    return {
        f"{prefix}{k}": norm_str(row.get(k))
        for k in fieldnames
    }


# ---------------------------------------------------------------------------
# Join
# ---------------------------------------------------------------------------

def join_rows(
    gt_rows: Sequence[Dict[str, str]],
    pred_rows: Sequence[Dict[str, str]],
    gt_fields: Sequence[str],
    pred_fields: Sequence[str],
) -> Tuple[List[Dict[str, str]], List[Dict[str, str]], Dict[str, object]]:
    gt_index, _, _ = index_unique(gt_rows, "GT")
    pred_index, _, _ = index_unique(pred_rows, "prediction")

    gt_uids = set(gt_index.keys())
    pred_uids = set(pred_index.keys())

    joined_uids = sorted(gt_uids & pred_uids)
    missing_pred_uids = sorted(gt_uids - pred_uids)
    unknown_pred_uids = sorted(pred_uids - gt_uids)

    joined_rows: List[Dict[str, str]] = []

    type_mismatch_uids: List[str] = []
    role_mismatch_uids: List[str] = []
    invalid_prediction_uids: List[str] = []
    uid_component_mismatch_uids: List[str] = []

    status_counts: Counter = Counter()

    # Keep GT instances as the primary table.
    for uid in sorted(gt_uids):
        gt = gt_index[uid]
        pred = pred_index.get(uid)

        dataset = norm_str(gt.get("dataset"))
        sample_id = norm_str(gt.get("sample_id"))
        instance_id = norm_str(gt.get("instance_id"))

        if pred is None:
            join_status = "MISSING_PREDICTION"
            type_match = ""
            role_match = ""
            prediction_adapter_valid = "False"
        else:
            gt_type = norm_str(gt.get("feature_type")).lower()
            pred_type = norm_str(pred.get("feature_type")).lower()

            gt_role = norm_str(gt.get("boolean_role"))
            pred_role = norm_str(pred.get("boolean_role"))

            type_match = str(gt_type == pred_type)
            role_match = str(gt_role == pred_role)

            adapter_valid_raw = norm_bool(pred.get("adapter_valid"))
            prediction_adapter_valid = adapter_valid_raw

            if gt_type != pred_type:
                type_mismatch_uids.append(uid)

            if gt_role and pred_role and gt_role != pred_role:
                role_mismatch_uids.append(uid)

            if adapter_valid_raw != "True":
                invalid_prediction_uids.append(uid)

            pred_dataset = norm_str(pred.get("dataset"))
            pred_sample_id = norm_str(pred.get("sample_id"))
            pred_instance_id = norm_str(pred.get("instance_id"))
            pred_uid = row_uid(pred)

            if not validate_uid_components(
                pred_uid,
                pred_dataset,
                pred_sample_id,
                pred_instance_id,
            ):
                uid_component_mismatch_uids.append(uid)

            if gt_type != pred_type:
                join_status = "JOINED_TYPE_MISMATCH"
            elif adapter_valid_raw != "True":
                join_status = "JOINED_PREDICTION_INVALID"
            else:
                join_status = "JOINED"

        status_counts[join_status] += 1

        base = {
            "protocol": PROTOCOL,
            "instance_uid": uid,
            "dataset": dataset,
            "sample_id": sample_id,
            "instance_id": instance_id,
            "join_status": join_status,
            "prediction_present": str(pred is not None),
            "prediction_adapter_valid": prediction_adapter_valid,
            "feature_type_match": type_match,
            "boolean_role_match": role_match,
        }

        base.update(prefix_row("gt_", gt, gt_fields))
        base.update(prefix_row("pred_", pred, pred_fields))

        joined_rows.append(base)

    unknown_rows = [
        pred_index[uid]
        for uid in unknown_pred_uids
    ]

    summary: Dict[str, object] = {
        "protocol": PROTOCOL,
        "gt_instance_count": len(gt_uids),
        "prediction_record_count": len(pred_uids),
        "joined_count": len(joined_uids),
        "missing_prediction_count": len(missing_pred_uids),
        "unknown_prediction_count": len(unknown_pred_uids),
        "type_mismatch_count": len(type_mismatch_uids),
        "role_mismatch_count": len(role_mismatch_uids),
        "invalid_prediction_count": len(invalid_prediction_uids),
        "uid_component_mismatch_count": len(uid_component_mismatch_uids),
        "join_status_counts": dict(status_counts),
        "missing_prediction_uids": missing_pred_uids,
        "unknown_prediction_uids": unknown_pred_uids,
        "type_mismatch_uids": sorted(type_mismatch_uids),
        "role_mismatch_uids": sorted(role_mismatch_uids),
        "invalid_prediction_uids": sorted(invalid_prediction_uids),
        "uid_component_mismatch_uids": sorted(uid_component_mismatch_uids),
    }

    return joined_rows, unknown_rows, summary


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

def write_csv(
    path: Path,
    rows: Sequence[Dict[str, str]],
    fieldnames: Sequence[str],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open(
        "w",
        encoding="utf-8-sig",
        newline="",
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=list(fieldnames),
            extrasaction="ignore",
        )
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def write_json_ascii(path: Path, payload: Dict[str, object]) -> None:
    """
    Write ASCII-compatible JSON for cross-platform parsing.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(
        payload,
        ensure_ascii=True,
        indent=2,
        allow_nan=False,
    )
    path.write_text(text, encoding="ascii")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "RbRM Evaluation V2 Phase 2-B: "
            "direct GT/prediction identity join"
        )
    )

    parser.add_argument(
        "--gt-manifest",
        required=True,
        help="GT manifest v1.4 CSV",
    )

    parser.add_argument(
        "--prediction-records",
        required=True,
        help="prediction_adapters.py output CSV",
    )

    parser.add_argument(
        "--dataset",
        default=None,
        choices=["CADParser", "DeepCAD"],
        help=(
            "Optional dataset filter. "
            "Recommended for smoke tests."
        ),
    )

    parser.add_argument(
        "--sample-id",
        default=None,
        help=(
            "Optional sample filter. "
            "Recommended for one-sample smoke tests."
        ),
    )

    parser.add_argument(
        "--out-dir",
        required=True,
    )

    parser.add_argument(
        "--fail-on-unknown-prediction",
        action="store_true",
        help=(
            "Return non-zero if prediction contains an instance_uid "
            "not present in the selected GT manifest."
        ),
    )

    parser.add_argument(
        "--fail-on-type-mismatch",
        action="store_true",
        help=(
            "Return non-zero if joined GT/prediction feature types differ."
        ),
    )

    args = parser.parse_args()

    gt_path = Path(args.gt_manifest)
    pred_path = Path(args.prediction_records)
    out_dir = Path(args.out_dir)

    gt_fields, gt_all_rows = read_csv_rows(gt_path)
    validate_gt_manifest_schema(gt_all_rows)
    pred_fields, pred_all_rows = read_csv_rows(pred_path)

    gt_rows = filter_gt_rows(
        gt_all_rows,
        dataset=args.dataset,
        sample_id=args.sample_id,
    )

    pred_rows = filter_pred_rows(
        pred_all_rows,
        dataset=args.dataset,
        sample_id=args.sample_id,
    )

    if not gt_rows:
        raise RuntimeError(
            "No GT rows remain after dataset/sample filtering."
        )

    if not pred_rows:
        # 注意：全空 prediction 理论上可作为 FR=0 的合法输入，
        # 但在 adapter smoke test 阶段通常意味着路径/过滤错误。
        # 这里不直接抛异常，仍允许生成 MISSING_PREDICTION rows。
        print(
            "WARNING: no prediction rows remain after filtering; "
            "all GT rows will be marked MISSING_PREDICTION."
        )

    joined_rows, unknown_rows, summary = join_rows(
        gt_rows=gt_rows,
        pred_rows=pred_rows,
        gt_fields=gt_fields,
        pred_fields=pred_fields,
    )

    # Fixed join fields followed by prefixed GT and prediction fields.
    base_fields = [
        "protocol",
        "instance_uid",
        "dataset",
        "sample_id",
        "instance_id",
        "join_status",
        "prediction_present",
        "prediction_adapter_valid",
        "feature_type_match",
        "boolean_role_match",
    ]

    joined_fields = (
        base_fields
        + [f"gt_{x}" for x in gt_fields]
        + [f"pred_{x}" for x in pred_fields]
    )

    out_dir.mkdir(parents=True, exist_ok=True)

    join_csv = out_dir / "identity_join.csv"
    report_json = out_dir / "identity_join_report.json"
    unknown_csv = out_dir / "unknown_prediction_records.csv"

    write_csv(
        join_csv,
        joined_rows,
        joined_fields,
    )

    if unknown_rows:
        write_csv(
            unknown_csv,
            unknown_rows,
            pred_fields,
        )
    else:
        # 仍然写一个只有表头的文件，便于 production audit。
        write_csv(
            unknown_csv,
            [],
            pred_fields,
        )

    report_payload: Dict[str, object] = {
        "protocol": PROTOCOL,
        "gt_manifest": str(gt_path.resolve()),
        "prediction_records": str(pred_path.resolve()),
        "dataset_filter": args.dataset,
        "sample_id_filter": args.sample_id,
        "summary": summary,
    }

    write_json_ascii(
        report_json,
        report_payload,
    )

    print()
    print("=" * 78)
    print("RbRM DIRECT IDENTITY JOIN")
    print("=" * 78)
    print(f"dataset filter        : {args.dataset}")
    print(f"sample filter         : {args.sample_id}")
    print(f"GT instances          : {summary['gt_instance_count']}")
    print(f"prediction records    : {summary['prediction_record_count']}")
    print(f"joined                : {summary['joined_count']}")
    print(f"missing prediction    : {summary['missing_prediction_count']}")
    print(f"unknown prediction    : {summary['unknown_prediction_count']}")
    print(f"type mismatch         : {summary['type_mismatch_count']}")
    print(f"role mismatch         : {summary['role_mismatch_count']}")
    print(f"invalid prediction    : {summary['invalid_prediction_count']}")
    print(f"UID component mismatch: {summary['uid_component_mismatch_count']}")
    print(f"join status           : {summary['join_status_counts']}")
    print("=" * 78)

    if summary["missing_prediction_uids"]:
        print("\nMISSING PREDICTIONS:")
        for uid in summary["missing_prediction_uids"]:
            print("  -", uid)

    if summary["unknown_prediction_uids"]:
        print("\nUNKNOWN PREDICTIONS:")
        for uid in summary["unknown_prediction_uids"]:
            print("  -", uid)

    if summary["type_mismatch_uids"]:
        print("\nTYPE MISMATCH:")
        for uid in summary["type_mismatch_uids"]:
            print("  -", uid)

    if summary["role_mismatch_uids"]:
        print("\nROLE MISMATCH:")
        for uid in summary["role_mismatch_uids"]:
            print("  -", uid)

    if summary["invalid_prediction_uids"]:
        print("\nINVALID PREDICTION RECORDS:")
        for uid in summary["invalid_prediction_uids"]:
            print("  -", uid)

    print(f"\nJOIN CSV : {join_csv}")
    print(f"REPORT   : {report_json}")
    print(f"UNKNOWN  : {unknown_csv}")

    exit_code = 0

    # Missing predictions are retained for downstream FR computation.
    if (
        args.fail_on_unknown_prediction
        and int(summary["unknown_prediction_count"]) > 0
    ):
        exit_code = 2

    if (
        args.fail_on_type_mismatch
        and int(summary["type_mismatch_count"]) > 0
    ):
        exit_code = max(exit_code, 3)

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
