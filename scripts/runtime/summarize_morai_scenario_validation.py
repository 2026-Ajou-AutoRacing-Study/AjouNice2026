#!/usr/bin/env python3
"""Create one spreadsheet-ready report from MORAI validation sessions."""

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List


RUNTIME_DIR = Path(__file__).resolve().parent
if str(RUNTIME_DIR) not in sys.path:
    sys.path.insert(0, str(RUNTIME_DIR))

from morai_scenario_validation import (  # noqa: E402
    RESULT_COLUMNS,
    atomic_csv,
    atomic_json,
    result_table_row,
)


def load_json(path: Path) -> Dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("cannot read {}: {}".format(path, error)) from error
    if not isinstance(value, dict):
        raise RuntimeError("summary is not a JSON object: {}".format(path))
    return value


def session_summaries(input_root: Path) -> Iterable[Dict[str, Any]]:
    for summary_path in sorted(input_root.glob("*/summary.json")):
        aggregate = load_json(summary_path)
        session_id = str(aggregate.get("session_id") or summary_path.parent.name)
        results = aggregate.get("results")
        if not isinstance(results, list):
            continue
        for result in results:
            if not isinstance(result, dict):
                continue
            item = dict(result)
            item.setdefault("session_id", session_id)
            item.setdefault("summary_file", str(summary_path))
            yield item


def build_aggregate(results: List[Dict[str, Any]]) -> Dict[str, Any]:
    return {
        "scenarios": len(results),
        "passed": sum(1 for item in results if item.get("passed")),
        "failed": sum(1 for item in results if not item.get("passed")),
        "collisions": sum(1 for item in results if item.get("collision_detected")),
        "errors": sum(1 for item in results if item.get("error")),
        "dynamic_reaction_failures": sum(
            1
            for item in results
            if (item.get("dynamic_reaction") or {}).get("required")
            and not (item.get("dynamic_reaction") or {}).get("passed")
        ),
        "sessions": sorted({str(item.get("session_id", "")) for item in results}),
    }


def markdown_report(aggregate: Dict[str, Any], rows: List[Dict[str, Any]]) -> str:
    failed_rows = [row for row in rows if str(row["passed"]).lower() != "true"]
    lines = [
        "# MORAI Scenario Validation Results",
        "",
        "| Metric | Count |",
        "| --- | ---: |",
    ]
    for key in (
        "scenarios",
        "passed",
        "failed",
        "collisions",
        "errors",
        "dynamic_reaction_failures",
    ):
        lines.append("| {} | {} |".format(key.replace("_", " "), aggregate[key]))
    lines.extend(
        [
            "",
            "- Sessions: {}".format(", ".join(aggregate["sessions"]) or "none"),
            "- Full case table: `results.csv`",
        ]
    )
    if failed_rows:
        lines.extend(
            [
                "",
                "## Failed cases",
                "",
                "| Scenario | End reason | Collision | Dynamic reaction | Error |",
                "| --- | --- | --- | --- | --- |",
            ]
        )
        for row in failed_rows:
            lines.append(
                "| {scenario} | {end_reason} | {collision_detected} | {dynamic_failure_reasons} | {error} |".format(
                    **{
                        key: str(row.get(key, "")).replace("|", "\\|")
                        for key in (
                            "scenario",
                            "end_reason",
                            "collision_detected",
                            "dynamic_failure_reasons",
                            "error",
                        )
                    }
                )
            )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Combine MORAI validation session summaries into CSV/JSON/Markdown."
    )
    parser.add_argument("--input-root", type=Path, required=True)
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="defaults to --input-root",
    )
    args = parser.parse_args()

    input_root = args.input_root.resolve()
    if not input_root.is_dir():
        parser.error("--input-root does not exist: {}".format(input_root))
    output_dir = (args.output_dir or input_root).resolve()
    results = list(session_summaries(input_root))
    if not results:
        parser.error("no session summary.json with results found under {}".format(input_root))

    results.sort(key=lambda item: (str(item.get("session_id", "")), str(item.get("scenario", ""))))
    rows = [result_table_row(item) for item in results]
    aggregate = build_aggregate(results)
    output_dir.mkdir(parents=True, exist_ok=True)
    atomic_csv(output_dir / "results.csv", rows, RESULT_COLUMNS)
    atomic_json(output_dir / "summary.json", aggregate)
    (output_dir / "summary.md").write_text(
        markdown_report(aggregate, rows), encoding="utf-8"
    )
    print("Results CSV: {}".format(output_dir / "results.csv"))
    print("Results summary: {}".format(output_dir / "summary.md"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
