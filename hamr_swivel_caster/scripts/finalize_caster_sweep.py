#!/usr/bin/env python3
"""Analyze completed sweep runs, label overviews, verify MP4s and build a gallery."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from analyze_caster_stress import analyze_run
from build_stress_report import build_report


def verify_videos(run):
    output = {}
    for stem in ("overview", "overview_annotated", "caster_closeup"):
        path = run / (stem + ".mp4")
        if not path.is_file():
            continue
        decoded = subprocess.run(["ffmpeg", "-v", "error", "-threads", "2", "-i", str(path),
                                  "-f", "null", "-"], capture_output=True, text=True)
        if decoded.returncode or decoded.stderr.strip():
            raise RuntimeError(f"Video decoding failed: {path}: {decoded.stderr}")
        metadata = json.loads(subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=width,height,codec_name,r_frame_rate,nb_frames:format=duration",
            "-of", "json", str(path)], text=True))
        stream = metadata["streams"][0]
        if stream["r_frame_rate"] != "25/1" or (stream["width"], stream["height"]) != (1280, 720):
            raise RuntimeError(f"Unexpected recording format: {path}")
        output[stem] = {"full_decode_passed": True, **metadata}
    (run / "video_validation.json").write_text(json.dumps(output, indent=2) + "\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", type=Path)
    parser.add_argument("--select", help="Comma-separated run names; default all completed recordings")
    parser.add_argument("--skip-existing", action="store_true", help="Retain already analyzed/validated artifacts")
    parser.add_argument("--refresh-analysis", action="store_true", help="Recompute analysis while retaining existing labeled videos/validation")
    args = parser.parse_args()
    selected = set(args.select.split(",")) if args.select else None
    for report in sorted((args.suite / "record").glob("*/run_summary.json")):
        run = report.parent
        if selected is not None and run.name not in selected:
            continue
        if args.refresh_analysis or not args.skip_existing or not (run / "caster_stress.json").exists():
            analyze_run(run)
        if (run / "overview.mp4").is_file() and not (run / "overview_annotated.mp4").exists():
            subprocess.run([sys.executable, str(Path(__file__).with_name("annotate_overview.py")), str(run)], check=True)
        if not args.skip_existing or not (run / "video_validation.json").exists():
            verify_videos(run)
        print(f"Reviewed: {run.name}", flush=True)
    build_report(args.suite.resolve())
    return 0


if __name__ == "__main__":
    sys.exit(main())
