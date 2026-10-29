"""Six measured encoder traces on the paired caster replay's common clock.

Plot the source encoder readings, not the zero/sign-adjusted CAD joint poses.
The caller supplies the clock-estimation provenance used to build the events.
"""
from bisect import bisect_right
import csv
import math
from pathlib import Path
import textwrap

import numpy as np

from caster_encoder_plots import BACKGROUND, COLORS, collect_readings, curve_points


WHEELS = ("front_left", "front_right")
ROLES = ("H1", "H2", "CENTER")
LABELS = {"front_left": "FRONT LEFT", "front_right": "FRONT RIGHT"}


class PairEncoderPlots:
    """Cache all six full-sample plots and overlay a single replay-time cursor."""

    def __init__(self, events_by_wheel, start_ns, end_ns, width,
                 recording_name, time_basis, stale_timeout_s=1.):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        from PIL import Image, ImageFont

        if set(events_by_wheel) != set(WHEELS):
            raise ValueError("Paired plots require front_left and front_right events")
        if end_ns <= start_ns:
            raise ValueError("Paired plots require a nonempty common time interval")
        self.start = start_ns
        self.duration = (end_ns - start_ns) / 1e9
        self.scale = width / 1920
        self.recording_name = recording_name
        self.time_basis = time_basis
        self.series, self.times = {}, {}
        self.metadata = {
            "quantity": "Source encoder angle in radians: H1/H2 multi-turn; CENTER firmware calibrated wrapped alpha.",
            "time_basis": time_basis,
            "cad_mapping_applied": False,
            "processing": "All accepted samples, without smoothing or decimation; curves break at sequence gaps, reboots, stale gaps and CENTER wraps.",
            "start_ns": start_ns, "end_ns": end_ns, "wheels": {},
        }
        for wheel in WHEELS:
            series, rejected = collect_readings(
                events_by_wheel[wheel], start_ns, stale_timeout_s, wheel_id=wheel)
            self.series[wheel] = series
            self.times[wheel] = {role: [r["bag_time_ns"] for r in rows]
                                 for role, rows in series.items()}
            self.metadata["wheels"][wheel] = {
                "invalid_envelopes": rejected,
                "devices": {role: sorted({r["device_id"] for r in rows})
                            for role, rows in series.items()},
                "encoders": {role: {
                    "samples": len(rows),
                    "segments": rows[-1]["segment"] + 1 if rows else 0,
                    "min_rad": min((r["angle_rad"] for r in rows), default=None),
                    "max_rad": max((r["angle_rad"] for r in rows), default=None),
                } for role, rows in series.items()},
            }

        self.font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 16)
        figure = Figure(figsize=(9.60, 8.28), dpi=100, facecolor=BACKGROUND)
        canvas = FigureCanvasAgg(figure)
        figure.text(.045, .966, "MEASURED ENCODER ANGLES", size=16,
                    weight="bold", color="#21334a")
        figure.text(.045, .933, "Common replay time  |  source readings before CAD offsets", size=11,
                    color="#687b90")
        self.axes = {}
        for wheel, left in zip(WHEELS, (.085, .575)):
            figure.text(left, .885, LABELS[wheel], size=14,
                        weight="bold", color="#21334a")
            for role, bottom in zip(ROLES, (.659, .397, .135)):
                ax = figure.add_axes((left, bottom, .385, .163))
                self.style_axes(ax, wheel, role, compact=True)
                self.axes[wheel, role] = ax
                ax.set_title(role, loc="left", color=COLORS[role],
                             fontsize=12, fontweight="bold", pad=12)
                if role != "CENTER":
                    ax.tick_params(axis="x", labelbottom=False)
                else:
                    ax.set_xlabel("Elapsed recording time (s)", fontsize=11, labelpad=8)
        figure.text(.085, .053, "H1 / H2: multi-turn  |  CENTER: wrapped 0–2π  |  gaps and wraps break curves",
                    size=10.5, color="#687b90")
        figure.text(.085, .024, "Timing uses acquisition-clock estimates; see replay provenance for uncertainty.",
                    size=9.5, color="#687b90")
        canvas.draw()
        self.panel = Image.fromarray(np.asarray(canvas.buffer_rgba())[:, :, :3].copy())

    def style_axes(self, ax, wheel, role, compact=False):
        from matplotlib.ticker import MaxNLocator

        ax.set_facecolor(BACKGROUND)
        rows = self.series[wheel][role]
        ax.plot(*curve_points(rows), color=COLORS[role], linewidth=1.25,
                marker=".", markersize=1.2, rasterized=True)
        ax.set_xlim(0, self.duration)
        ax.set_ylabel("rad" if compact else "Encoder angle (rad)", fontsize=10 if compact else 11)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=5, min_n_ticks=3))
        if role == "CENTER":
            ax.set_ylim(-.18, math.tau + .18)
            ax.set_yticks([0, math.pi, math.tau], ["0", "π", "2π"])
        else:
            ax.margins(y=.12)
            ax.yaxis.set_major_locator(MaxNLocator(nbins=3 if compact else 5, min_n_ticks=3))
            ax.ticklabel_format(axis="y", style="plain", useOffset=False)
        ax.tick_params(labelsize=10, colors="#43566d")
        ax.grid(color="#d6dfe8", linewidth=.7)
        for name, spine in ax.spines.items():
            spine.set_visible(name in ("left", "bottom"))
            spine.set_color("#bac6d2")
        if not rows:
            ax.text(.5, .5, "No valid readings", transform=ax.transAxes,
                    ha="center", color="#687b90")

    def current(self, wheel, role, stamp_ns):
        """Use only measurements available at this integer replay timestamp."""
        index = bisect_right(self.times[wheel][role], stamp_ns) - 1
        return self.series[wheel][role][index] if index >= 0 else None

    def draw(self, image, frames_by_wheel):
        from PIL import Image, ImageDraw

        stamps = {frames_by_wheel[wheel]["stamp_ns"] for wheel in WHEELS}
        if len(stamps) != 1:
            raise ValueError("Both plot columns must use the same replay timestamp")
        stamp = stamps.pop()
        elapsed = (stamp - self.start) / 1e9
        panel = self.panel.copy()
        draw = ImageDraw.Draw(panel)
        for wheel in WHEELS:
            states = {node["role"]: node["display_state"]
                      for node in frames_by_wheel[wheel]["status"]["nodes"]}
            for role in ROLES:
                ax = self.axes[wheel, role]
                _, bottom, right, top = ax.bbox.extents
                y0, y1 = panel.height - top, panel.height - bottom
                cursor_x = ax.transData.transform((min(self.duration, max(0, elapsed)), 0))[0]
                draw.line((cursor_x, y0, cursor_x, y1), fill="#21334a", width=2)
                row = self.current(wheel, role, stamp)
                value = "waiting" if row is None else f"{row['angle_rad']:+.3f} rad"
                if row is not None:
                    if states.get(role) != "fresh":
                        value += " held"
                    dot_y = panel.height - ax.transData.transform((elapsed, row["angle_rad"]))[1]
                    if y0 <= dot_y <= y1:
                        draw.ellipse((cursor_x-4, dot_y-4, cursor_x+4, dot_y+4),
                                     fill=COLORS[role], outline="white", width=1)
                draw.text((right, y0-31), value, font=self.font, fill="#21334a", anchor="rt")
        size = (round(960 * self.scale), round(828 * self.scale))
        if panel.size != size:
            panel = panel.resize(size, Image.Resampling.LANCZOS)
        image.paste(panel, (round(960 * self.scale), round(126 * self.scale)))
        return image

    def export(self, base_path):
        """Export every valid sample plus a two-column full-recording figure."""
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure

        base_path = Path(base_path)
        csv_path = base_path.with_suffix(".encoders.csv")
        rows = sorted((row for wheel in WHEELS for series in self.series[wheel].values()
                       for row in series), key=lambda row: row["bag_time_ns"])
        with csv_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=[
                "replay_time_ns", "time_s", "wheel_id", "role", "encoder_angle_rad",
                "seq", "boot_id", "device_id", "segment"])
            writer.writeheader()
            for row in rows:
                exported = dict(row)
                exported["replay_time_ns"] = exported.pop("bag_time_ns")
                exported["encoder_angle_rad"] = exported.pop("angle_rad")
                writer.writerow(exported)
        figure = Figure(figsize=(16, 10), dpi=150, facecolor=BACKGROUND)
        FigureCanvasAgg(figure)
        axes = figure.subplots(3, 2, sharex=True)
        figure.subplots_adjust(left=.065, right=.975, bottom=.20, top=.82,
                               hspace=.42, wspace=.17)
        figure.text(.065, .958, "HAMR · front caster encoder angles", size=22,
                    weight="bold", color="#21334a")
        figure.text(.065, .921, self.recording_name + f"  |  recorded span {self.duration:.3f} s",
                    size=12, color="#687b90")
        for col, wheel in enumerate(WHEELS):
            figure.text(.065 if col == 0 else .555, .872, LABELS[wheel],
                        size=15, weight="bold", color="#21334a")
            for row, role in enumerate(ROLES):
                ax = axes[row, col]
                self.style_axes(ax, wheel, role)
                ax.set_title(role, loc="left", color=COLORS[role], weight="bold", fontsize=12)
            axes[-1, col].set_xlabel("Elapsed recording time (s)", fontsize=12)
        figure.text(.065, .126,
                    "Source readings before CAD sign/zero mapping · H1/H2 multi-turn · CENTER wrapped 0–2π · all accepted samples",
                    size=11, color="#687b90")
        figure.text(.065, .063, textwrap.fill(str(self.time_basis), width=155),
                    size=10, color="#687b90", va="center")
        paths = [base_path.with_suffix(".encoders.png"), base_path.with_suffix(".encoders.pdf")]
        for path in paths:
            figure.savefig(path, dpi=150, facecolor=BACKGROUND)
        self.metadata["files"] = [str(path) for path in [*paths, csv_path]]
        return self.metadata["files"]
