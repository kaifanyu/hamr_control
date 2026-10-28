"""Offline trajectory plots and schematic ordinary-caster image/video replay."""

import argparse
import json
import math
from pathlib import Path
import shutil

from .analysis import write_analysis
from .history import add_history_arguments, history_from_args
from .replay import frame_times, replay_frame, validate_replay


def draw_frame(figure, history, stamp, allow_uncalibrated=False):
    """Draw measured orientation and relative wheel phase; no robot motion is inferred."""
    from matplotlib.patches import Circle, Polygon
    figure.clear()
    pose = replay_frame(history, stamp, allow_uncalibrated)
    axes = figure.subplots(1, len(history.wheels), squeeze=False)[0]
    channels = {(item["wheel_id"], item["channel"]): item for item in pose["channels"]}
    for wheel, axis in zip(history.wheels, axes):
        swivel, rolling = channels[(wheel, "swivel")], channels[(wheel, "wheel")]
        axis.set_aspect("equal")
        axis.set_xlim(-1.7, 1.7)
        axis.set_ylim(-1.7, 1.9)
        axis.axis("off")
        axis.set_title(wheel.replace("_", " "))
        axis.add_patch(Circle((0, 0), .07, color="#384b5e"))
        if swivel["valid"]:
            angle = swivel["angle_rad"]
            direction = (math.cos(angle), math.sin(angle))
            normal = (-direction[1], direction[0])
            center = (-.55 * direction[0], -.55 * direction[1])
            axis.plot([0, center[0]], [0, center[1]], color="#8996a2", linewidth=10)
            corners = [(center[0] + x * direction[0] + y * normal[0],
                        center[1] + x * direction[1] + y * normal[1])
                       for x, y in ((-.4, -.16), (.4, -.16), (.4, .16), (-.4, .16))]
            axis.add_patch(Polygon(corners, color="#253543"))
            axis.arrow(0, 0, .7 * direction[0], .7 * direction[1], width=.025, color="#d3821e")
            axis.text(0, 1.45, f"Swivel {math.degrees(math.remainder(angle, math.tau)):.1f}°", ha="center")
        else:
            axis.add_patch(Circle((0, 0), .8, color="#e2e5e8", fill=False, linestyle="--"))
            axis.text(0, 1.45, "Swivel unavailable", ha="center", color="#a63124")
        # Separate side-view phase dial avoids fabricating rolling distance,
        # body trajectory, slip, wheel radius, or absolute marked-wheel phase.
        phase_center = (0., -1.03)
        axis.add_patch(Circle(phase_center, .25, fill=False, color="#677789"))
        if rolling["valid"]:
            phase = math.remainder(rolling["angle_rad"], math.tau)
            axis.plot([0, .22 * math.sin(phase)], [-1.03, -1.03 + .22 * math.cos(phase)], color="#2578ad", linewidth=3)
            axis.text(0, -1.48, f"Relative wheel {rolling['angle_rad'] / math.tau:.2f} turns", ha="center")
        else:
            axis.text(0, -1.48, "Wheel unavailable", ha="center", color="#a63124")
        axis.text(0, 1.12, f"Swivel: {swivel['state']}\nWheel: {rolling['state']}", ha="center", fontsize=8)
    override = any(item.get("uncalibrated_override") for item in pose["channels"])
    figure.suptitle(f"Ordinary caster history · {(stamp-history.start_ns)/1e9:.3f} s\n" +
                    ("UNCALIBRATED ILLUSTRATIVE PREVIEW · " if override else "") +
                    ("Estimated shared acquisition clock" if history.time_basis == "estimated_acquisition_epoch" else "Single device clock"))
    figure.text(.5, .015, "Schematic geometry. Wheel phase is relative. Invalid observations are not filled in.",
                ha="center", fontsize=9)
    figure.tight_layout(rect=(0, .04, 1, .9))
    return pose


def render_history(history, output, video=False, fps=25.0, allow_uncalibrated=False):
    validate_replay(history, allow_uncalibrated)
    times = list(frame_times(history, fps))
    if video and shutil.which("ffmpeg") is None:
        raise ValueError("MP4 rendering requires ffmpeg on PATH")
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    from matplotlib.animation import FFMpegWriter
    output = Path(output)
    report = write_analysis(history, output)
    figure = plt.figure(figsize=(6 * len(history.wheels), 6))
    # A short run can fit between video ticks. Select a still from actual
    # acquisition events so useful measured poses are not hidden by frame rate.
    candidates = sorted({stamp for stream in history.streams for stamp in stream.times})
    preview_stamp = next((stamp for stamp in candidates if history.pose_at(stamp, allow_uncalibrated)["complete_pose_available"]), times[len(times)//2])
    preview = draw_frame(figure, history, preview_stamp, allow_uncalibrated)
    figure.savefig(output / "preview.png", dpi=150)
    if video:
        writer = FFMpegWriter(fps=fps, codec="libx264", extra_args=["-pix_fmt", "yuv420p"])
        with writer.saving(figure, str(output / "ordinary_caster.mp4"), dpi=100):
            for stamp in times:
                draw_frame(figure, history, stamp, allow_uncalibrated)
                writer.grab_frame()
    plt.close(figure)
    provenance = {"source": str(history.source), "run_id": history.run_id,
                  "time_basis": history.time_basis, "accuracy_validated": False,
                  "allow_uncalibrated": allow_uncalibrated, "geometry": "illustrative schematic",
                  "video": video, "fps": fps, "frames": len(times), "preview": preview,
                  "warnings": report["warnings"]}
    (output / "render.json").write_text(json.dumps(provenance, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return provenance


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_history_arguments(parser)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--allow-uncalibrated", action="store_true")
    parser.add_argument("--video", action="store_true", help="Also write MP4; requires ffmpeg")
    parser.add_argument("--fps", type=float, default=25.0)
    args = parser.parse_args(argv)
    try:
        history = history_from_args(args, require_clock=True)
        output = args.output or history.source.parent / "ordinary_render"
        render_history(history, output, args.video, args.fps, args.allow_uncalibrated)
    except (ValueError, OSError, ImportError) as error:
        parser.error(str(error))
    print(f"Wrote ordinary caster replay artifacts to {output}")
    return 0
