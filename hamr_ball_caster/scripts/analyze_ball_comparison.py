#!/usr/bin/env python3
"""Analyze completed ball/swivel matrix pairs once, or watch for new captures.

Default: finite one-shot batch. --watch optionally polls every 45 seconds of
wall time; it does not advance or launch Gazebo. Failed/partial
capture artifacts are retained. Run only one analysis writer per suite.
"""
import argparse
import csv
import hashlib
import io
import json
from pathlib import Path
import time
import traceback

from analyze_ball_stress import analyze_run
from compare_caster_runs import compare_runs, comparison_row


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_atomic(path, value):
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(value)
    temporary.replace(path)


def signature(case, make_plots):
    ball, swivel = Path(case["ball_directory"]), Path(case["swivel_directory"])
    scripts = Path(__file__).resolve().parent
    return {"ball_trajectory": digest(ball/"trajectory.json"),
            "ball_robot": digest(ball/"robot.urdf"),
            "swivel_trajectory": digest(swivel/"trajectory.json"),
            "ball_analyzer": digest(scripts/"analyze_ball_stress.py"),
            "pair_analyzer": digest(scripts/"compare_caster_runs.py"),
            "case": hashlib.sha256(json.dumps(case, sort_keys=True).encode()).hexdigest(),
            "make_plots": make_plots}


def analyze_suite(suite_dir, refresh=False, make_plots=True):
    """One finite pass; returns completion/error information without simulation."""
    suite = Path(suite_dir).expanduser().resolve()
    matrix = json.loads((suite/"comparison_matrix.json").read_text())
    rows, cases = [], []
    for case in matrix:
        name = case["name"]
        ball = Path(case["ball_directory"])
        output = suite/"pairs"/name
        status = {"name": name, "state": "pending"}
        cases.append(status)
        if not (ball/"run_summary.json").exists():
            continue
        try:
            missing = [str(path) for path in (ball/"trajectory.json", ball/"robot.urdf", Path(case["swivel_directory"])/"trajectory.json") if not path.exists()]
            if missing:
                raise ValueError("Completed capture is missing analysis input(s): "+", ".join(missing))
            current = signature(case, make_plots)
            cache_path = ball/"comparison_analysis_cache.json"
            cached = json.loads(cache_path.read_text()) if cache_path.exists() else None
            artifacts = [ball/"ball_stress.json", ball/"ball_stress.csv", output/"comparison.json"]
            if make_plots:
                artifacts += [ball/"ball_stress.png", output/"comparison.png"]
            if not refresh and cached == current and all(path.exists() for path in artifacts):
                report = json.loads((output/"comparison.json").read_text())
                status["recomputed"] = False
            else:
                analyze_run(ball, make_plot=make_plots)
                report = compare_runs(case["swivel_directory"], ball, output,
                    primary_region=case["tracking_comparison_region"], context=case, make_plot=make_plots)
                write_atomic(cache_path, json.dumps(current, indent=2)+"\n")
                status["recomputed"] = True
            rows.append({"name": name, "category": case["category"], **comparison_row(report)})
            status.update({"state": "analyzed", "run_status": report["run_status"]})
        except Exception as error:
            ball.mkdir(parents=True, exist_ok=True)
            status.update({"state": "analysis_error", "type": type(error).__name__, "message": str(error)})
            write_atomic(ball/"analysis_error.json", json.dumps({**status, "traceback": traceback.format_exc()}, indent=2)+"\n")
    write_atomic(suite/"paired_index.json", json.dumps(rows, indent=2, allow_nan=False)+"\n")
    stream = io.StringIO(newline="")
    if rows:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    write_atomic(suite/"paired_index.csv", stream.getvalue())
    result = {"expected_count": len(matrix), "analyzed_count": len(rows),
              "pending_count": sum(case["state"] == "pending" for case in cases),
              "analysis_error_count": sum(case["state"] == "analysis_error" for case in cases),
              "complete": len(rows) == len(matrix), "cases": cases,
              "note": "Completed/failed trajectory outcomes remain in each comparison. An analysis error is distinct from a physical trajectory failure. No capture artifacts are removed or modified."}
    write_atomic(suite/"analysis_status.json", json.dumps(result, indent=2, allow_nan=False)+"\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite_directory", type=Path)
    parser.add_argument("--watch", nargs="?", const=45., type=float, metavar="SECONDS",
                        help="Poll for captures; default interval 45s. Without this flag, run one finite pass.")
    parser.add_argument("--refresh", action="store_true", help="Recompute existing analyses on the first pass")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    if args.watch is not None and (not 1. <= args.watch <= 60.):
        parser.error("--watch interval must be between 1 and 60 seconds")
    refresh = args.refresh
    while True:
        result = analyze_suite(args.suite_directory, refresh, not args.no_plots)
        print(json.dumps({key: result[key] for key in ("expected_count", "analyzed_count", "pending_count", "analysis_error_count", "complete")}), flush=True)
        refresh = False
        if args.watch is None or result["complete"] or result["pending_count"] == 0:
            raise SystemExit(1 if result["analysis_error_count"] else 0)
        time.sleep(args.watch)


if __name__ == "__main__":
    main()
