"""A measured trajectory inset for a stationary-chassis caster replay."""
from bisect import bisect_right
import csv
import math
from pathlib import Path
import textwrap

import numpy as np

from caster_encoder_plots import BACKGROUND


class PairTrajectoryPlot:
    """Show only recorded positions available on the shared encoder timeline.

    Coordinates retain the trajectory source's recorded frame; they are not
    integrated from caster angles. Empty input intentionally draws empty axes.
    """

    def __init__(self, rows, start_ns, end_ns, width, metadata=None):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        from PIL import Image, ImageFont

        if end_ns <= start_ns:
            raise ValueError("Trajectory inset requires a nonempty replay interval")
        self.start, self.end = start_ns, end_ns
        self.scale = width / 1920
        self.rows = []
        for row in rows:
            stamp = row["stamp_ns"]
            if isinstance(stamp, bool) or not isinstance(stamp, int):
                raise ValueError("Trajectory timestamps must be integer nanoseconds")
            if not all(math.isfinite(float(row[key])) for key in ("x", "y")):
                raise ValueError("Trajectory x/y positions must be finite")
            if start_ns <= stamp <= end_ns:
                self.rows.append(dict(row))
        self.rows.sort(key=lambda row: row["stamp_ns"])
        self.times = [row["stamp_ns"] for row in self.rows]
        self.metadata = dict(metadata or {})
        self.metadata.update({
            "available": bool(self.rows), "samples": len(self.rows),
            "coordinates": "Recorded source-frame x/y in metres; no caster integration or chassis translation.",
            "display": "Path grows only through the common replay timestamp; gaps over one second break the line.",
        })
        if not self.rows and not self.metadata.get("reason"):
            self.metadata["reason"] = "No position messages overlap this encoder recording."

        font_root = Path("/usr/share/fonts/truetype/dejavu")
        self.font = ImageFont.truetype(str(font_root / "DejaVuSans.ttf"), 11)
        self.value_font = ImageFont.truetype(str(font_root / "DejaVuSansMono.ttf"), 12)
        figure = Figure(figsize=(3.30, 3.70), dpi=100, facecolor=BACKGROUND)
        canvas = FigureCanvasAgg(figure)
        figure.text(.06, .95, "ROBOT TRAJECTORY", size=12,
                    weight="bold", color="#21334a")
        source = self.metadata.get("topic") or self.metadata.get("source") or "Vicon / odometry"
        figure.text(.06, .899, textwrap.shorten(str(source), width=43, placeholder="…"),
                    size=8.5, color="#687b90")
        self.axes = figure.add_axes((.19, .22, .75, .60))
        ax = self.axes
        ax.set_facecolor(BACKGROUND)
        ax.set_xlabel("x (m)", fontsize=9, labelpad=4)
        ax.set_ylabel("y (m)", fontsize=9, labelpad=4)
        ax.tick_params(labelsize=8, colors="#43566d")
        ax.ticklabel_format(axis="both", style="plain", useOffset=False)
        ax.grid(color="#d6dfe8", linewidth=.7)
        for name, spine in ax.spines.items():
            spine.set_visible(name in ("left", "bottom"))
            spine.set_color("#bac6d2")
        if self.rows:
            positions = np.array([[row["x"], row["y"]] for row in self.rows], dtype=float)
            low, high = positions.min(axis=0), positions.max(axis=0)
            center = (low + high) / 2
            half_span = max(float(np.max(high-low)) * .60, .15)
            ax.set_xlim(center[0]-half_span, center[0]+half_span)
            ax.set_ylim(center[1]-half_span, center[1]+half_span)
        else:
            positions = np.empty((0, 2))
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.text(.5, .55, "No matching\nVicon / odometry", transform=ax.transAxes,
                    ha="center", va="center", color="#687b90", fontsize=10)
        ax.set_aspect("equal", adjustable="box")
        from matplotlib.ticker import MaxNLocator
        ax.xaxis.set_major_locator(MaxNLocator(nbins=3))
        ax.yaxis.set_major_locator(MaxNLocator(nbins=3))
        if not self.rows:
            reason = textwrap.shorten(str(self.metadata["reason"]), width=95, placeholder="…")
            figure.text(.06, .063, textwrap.fill(reason, width=47),
                        size=8, color="#687b90", va="center")
        canvas.draw()
        self.panel = Image.fromarray(np.asarray(canvas.buffer_rgba())[:, :, :3].copy())
        self.points = ax.transData.transform(positions)
        if len(self.points):
            self.points[:, 1] = self.panel.height - self.points[:, 1]
        self.segments = [0]
        for index in range(1, len(self.rows)):
            previous, row = self.rows[index-1], self.rows[index]
            if row["stamp_ns"]-previous["stamp_ns"] > 1_000_000_000 or row.get("segment") != previous.get("segment"):
                self.segments.append(index)

    def current(self, stamp_ns):
        """Return the last recorded position, never a future or invented pose."""
        index = bisect_right(self.times, stamp_ns) - 1
        return self.rows[index] if index >= 0 else None

    def draw(self, image, stamp_ns):
        from PIL import Image, ImageDraw

        panel = self.panel.copy()
        draw = ImageDraw.Draw(panel)
        count = bisect_right(self.times, stamp_ns)
        if count:
            for start, stop in zip(self.segments, self.segments[1:] + [count]):
                if start >= count:
                    break
                visible = self.points[start:min(stop, count)]
                if len(visible) > 1:
                    draw.line([tuple(point) for point in visible], fill="#2879d8", width=3)
            x0, y0 = self.points[0]
            draw.ellipse((x0-3, y0-3, x0+3, y0+3), fill="white", outline="#2879d8", width=2)
            x, y = self.points[count-1]
            row = self.rows[count-1]
            held = stamp_ns-row["stamp_ns"] > 1_000_000_000
            color = "#d36b1f" if held else "#209d94"
            draw.ellipse((x-5, y-5, x+5, y+5), fill=color, outline="white", width=1)
            draw.text((20, 325), f"x {row['x']:+.3f}  y {row['y']:+.3f} m",
                      font=self.value_font, fill="#21334a")
            draw.text((20, 345), "Last position held" if held else "Recorded path · chassis held fixed",
                      font=self.font, fill="#687b90")
        elif self.rows:
            draw.text((20, 330), "Waiting for first recorded position",
                      font=self.font, fill="#687b90")
        size = (round(330*self.scale), round(370*self.scale))
        if panel.size != size:
            panel = panel.resize(size, Image.Resampling.LANCZOS)
        image.paste(panel, (round(620*self.scale), round(210*self.scale)))
        return image

    def export(self, base_path):
        """Save the complete measured path, or the same explicit empty panel."""
        from PIL import Image

        base_path = Path(base_path)
        csv_path = base_path.with_suffix(".trajectory.csv")
        fields = ["stamp_ns", "x", "y"] + sorted(
            {key for row in self.rows for key in row} - {"stamp_ns", "x", "y"})
        with csv_path.open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        # Draw at the same reference resolution regardless of video width.
        saved_scale = self.scale
        self.scale = 1
        try:
            canvas = Image.new("RGB", (960, 610), BACKGROUND)
            self.draw(canvas, self.end)
        finally:
            self.scale = saved_scale
        png_path = base_path.with_suffix(".trajectory.png")
        canvas.crop((620, 210, 950, 580)).save(png_path)
        self.metadata["files"] = [str(png_path), str(csv_path)]
        return self.metadata["files"]
