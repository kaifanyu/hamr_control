#!/usr/bin/env python3
"""Add measured chassis-heading labels to a completed overview recording."""
import argparse
import json
import math
from pathlib import Path
import subprocess


def ass_time(seconds):
    centiseconds = max(0, round(seconds*100))
    minutes, remainder = divmod(centiseconds, 6000)
    hours, minutes = divmod(minutes, 60)
    return f"{hours}:{minutes:02}:{remainder//100:02}.{remainder%100:02}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    run = args.run_dir.expanduser().resolve()
    trajectory = json.loads((run / "trajectory.json").read_text())
    summary = json.loads((run / "run_summary.json").read_text())
    metadata = json.loads((run / "overview_metadata.json").read_text())
    source = run / "overview.mp4"
    output = run / "overview_annotated.mp4"
    if output.exists():
        parser.error(f"Refusing to overwrite {output}")
    start = metadata["first_sim_time_s"]
    duration = metadata["last_sim_time_s"]-start
    yaw = summary.get("initial_chassis_yaw_deg", 0.)
    title = f"Initial chassis heading: {yaw:g} deg | World waypoints fixed"
    header = """[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 720
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Default,DejaVu Sans,23,&H00FFFFFF,&H00FFFFFF,&H80000000,&H80000000,0,0,0,0,100,100,0,0,3,6,0,7,22,22,20,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    events = []
    samples = trajectory["samples"]
    first = max(0., samples[0]["odometry_stamp_s"]-start)
    if first > 0:
        events.append(f"Dialogue: 0,0:00:00.00,{ass_time(first)},Default,,0,0,0,,{title}\\NWaiting for measured motion")
    for index, sample in enumerate(samples):
        begin = max(0., sample["odometry_stamp_s"]-start)
        end = (samples[index+1]["odometry_stamp_s"]-start if index+1 < len(samples) else duration)
        end = min(end, duration)
        if end <= begin:
            continue
        angle = round(math.degrees(sample["base_rpy_rad"][2]), 1) % 360
        x, y = sample["position_m"][:2]
        phase = sample["phase"].replace("_", " ")
        label = (f"{title}\\NMeasured chassis yaw: {angle:.1f} deg"
                 f" | x: {x:.2f} m, y: {y:.2f} m"
                 f"\\NPath time: {sample['trajectory_time_s']:.1f} s | {phase}")
        events.append(f"Dialogue: 0,{ass_time(begin)},{ass_time(end)},Default,,0,0,0,,{label}")
    (run / "overview_heading.ass").write_text(header+"\n".join(events)+"\n")
    # Run in the result directory so arbitrary user paths never enter a filter.
    command = ["ffmpeg", "-v", "error", "-nostdin", "-n", "-i", source.name,
               "-vf", "subtitles=overview_heading.ass", "-c:v", "libx264",
               "-crf", "18", "-preset", "fast", "-threads", "2", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", "-an", output.name]
    subprocess.run(command, cwd=run, check=True)
    print(output)


if __name__ == "__main__":
    main()
