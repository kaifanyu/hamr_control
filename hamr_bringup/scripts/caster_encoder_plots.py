"""Full-sample encoder angle plots synchronized to the recorded bag clock."""
from bisect import bisect_right
import csv
import math

import numpy as np

from caster_encoder_protocol import ROLES
from caster_replay_model import ReplayModel
from caster_encoder_selection import WheelIdentityError


COLORS = {'H1': '#2879d8', 'H2': '#d36b1f', 'CENTER': '#209d94'}
BACKGROUND = '#f6f8fb'


def collect_readings(events, start_ns, stale_timeout_s=1., wheel_id=None):
    """Retain all accepted samples, including multiple samples in one packet.

    X is recorded packet time, not unsynchronized device acquisition time.
    Sequences, boots, validity and duplicates follow the CAD replay adapter.
    No zero shift, CAD sign mapping, smoothing or decimation is applied.
    """
    model = ReplayModel(stale_timeout_s=stale_timeout_s, wheel_id=wheel_id)
    series = {role: [] for role in ROLES.values()}
    for stamp, envelope in events:
        try:
            readings = model.receive(envelope, stamp)
        except WheelIdentityError:
            raise
        except ValueError:
            continue
        for reading in readings:
            role, node = reading['role'], reading['node_id']
            rows = series[role]
            row = {'bag_time_ns': stamp, 'time_s': (stamp-start_ns)/1e9,
                   'role': role, 'angle_rad': reading['position'],
                   'seq': reading['seq'], 'boot_id': model.values[node]['boot_id'],
                   'wheel_id': reading['wheel_id'], 'device_id': reading['device_id'],
                   'segment': 0}
            if rows:
                last = rows[-1]
                discontinuity = (
                    row['boot_id'] != last['boot_id'] or
                    row['seq'] != last['seq'] + 1 or
                    stamp-last['bag_time_ns'] > stale_timeout_s*1e9 or
                    (role == 'CENTER' and abs(row['angle_rad']-last['angle_rad']) > math.pi)
                )
                row['segment'] = last['segment'] + int(discontinuity)
            rows.append(row)
    return series, model.invalid_envelopes


def curve_points(rows):
    """Separate discontinuities with NaNs; retain points on both sides."""
    x, y, segment = [], [], None
    for row in rows:
        if segment is not None and segment != row['segment']:
            x.append(float('nan')); y.append(float('nan'))
        x.append(row['time_s']); y.append(row['angle_rad'])
        segment = row['segment']
    return x, y


class EncoderPlots:
    """Draw the full recording once; add a cursor and last valid value per frame."""
    def __init__(self, events, start_ns, end_ns, bag_name, width, stale_timeout_s=1., wheel_id=None):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        from PIL import Image, ImageFont

        self.series, rejected = collect_readings(events, start_ns, stale_timeout_s, wheel_id)
        self.times = {role: [row['bag_time_ns'] for row in rows]
                      for role, rows in self.series.items()}
        self.start, self.duration = start_ns, (end_ns-start_ns)/1e9
        self.bag_name = bag_name
        self.scale = width / 1600
        self.font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 17)
        self.metadata = {
            'wheel_id': wheel_id or next((row['wheel_id'] for rows in self.series.values() for row in rows), None),
            'devices': {role: sorted({row['device_id'] for row in rows}) for role, rows in self.series.items()},
            'quantity': 'Encoder angle in radians; H1/H2 multi-turn, CENTER calibrated wrapped alpha.',
            'time_basis': 'Seconds since bag start, using packet bag timestamps (host receipt; device clocks unsynchronized).',
            'processing': 'All accepted samples, no smoothing or decimation. Break curves at sequence gaps, reboots, stale gaps, and CENTER wraps.',
            'cad_mapping_applied': False,
            'invalid_envelopes': rejected,
            'encoders': {role: {'samples': len(rows),
                                'segments': rows[-1]['segment']+1 if rows else 0,
                                'min_rad': min((row['angle_rad'] for row in rows), default=None),
                                'max_rad': max((row['angle_rad'] for row in rows), default=None)}
                         for role, rows in self.series.items()},
        }
        figure = Figure(figsize=(6.94, 6.48), dpi=100, facecolor=BACKGROUND)
        canvas = FigureCanvasAgg(figure)
        figure.text(.045, .965, 'ENCODER READINGS', size=14, weight='bold', color='#21334a')
        figure.text(.045, .928, 'Full recording  |  cursor = replay time', size=10.5, color='#687b90')
        self.axes = {}
        for role, bottom in zip(ROLES.values(), (.68, .405, .13)):
            ax = figure.add_axes((.14, bottom, .81, .18))
            self.style_axes(ax, role, compact=True)
            self.axes[role] = ax
            ax.set_title(role, loc='left', color=COLORS[role], fontsize=12, fontweight='bold', pad=8)
            if role != 'CENTER':
                ax.tick_params(axis='x', labelbottom=False)
            else:
                ax.set_xlabel('Elapsed bag time (s)', fontsize=11, labelpad=8)
        figure.text(.14, .018, 'H1 / H2: multi-turn   |   CENTER: wrapped 0–2π', size=9.5, color='#687b90')
        canvas.draw()
        self.panel = Image.fromarray(np.asarray(canvas.buffer_rgba())[:, :, :3].copy())

    def style_axes(self, ax, role, compact=False):
        from matplotlib.ticker import MaxNLocator
        ax.set_facecolor(BACKGROUND)
        ax.plot(*curve_points(self.series[role]), color=COLORS[role], linewidth=1.4 if compact else 1.2,
                marker='.', markersize=1.4, rasterized=True)
        ax.set_xlim(0, self.duration)
        ax.set_ylabel('rad' if compact else 'Angle (rad)', fontsize=11)
        ax.xaxis.set_major_locator(MaxNLocator(nbins=6, min_n_ticks=4))
        if role == 'CENTER':
            ax.set_ylim(-.18, 2*math.pi+.18)
            ax.set_yticks([0, math.pi, 2*math.pi], ['0', 'π', '2π'])
        else:
            ax.margins(y=.12)
            ax.yaxis.set_major_locator(MaxNLocator(nbins=3 if compact else 5, min_n_ticks=3))
            ax.ticklabel_format(axis='y', style='plain', useOffset=False)
        ax.tick_params(labelsize=10.5, colors='#43566d')
        ax.grid(color='#d6dfe8', linewidth=.7)
        for name, spine in ax.spines.items():
            spine.set_visible(name in ('left', 'bottom'))
            spine.set_color('#bac6d2')
        if not self.series[role]:
            ax.text(.5, .5, 'No valid readings', transform=ax.transAxes, ha='center', color='#687b90')

    def current(self, role, stamp_ns):
        index = bisect_right(self.times[role], stamp_ns)-1
        return self.series[role][index] if index >= 0 else None

    def draw(self, image, frame):
        from PIL import Image, ImageDraw
        panel = self.panel.copy()
        draw = ImageDraw.Draw(panel)
        elapsed = (frame['stamp_ns']-self.start)/1e9
        states = {node['role']: node['display_state'] for node in frame['status']['nodes']}
        for role, ax in self.axes.items():
            left, bottom, right, top = ax.bbox.extents
            y0, y1 = panel.height-top, panel.height-bottom
            cursor_x = ax.transData.transform((min(self.duration, max(0, elapsed)), 0))[0]
            draw.line((cursor_x, y0, cursor_x, y1), fill='#21334a', width=2)
            row = self.current(role, frame['stamp_ns'])
            value = 'waiting' if row is None else f"{row['angle_rad']:+.3f} rad"
            if row is not None:
                if states[role] != 'fresh':
                    value += ' (held)'
                dot_y = panel.height-ax.transData.transform((elapsed, row['angle_rad']))[1]
                if y0 <= dot_y <= y1:
                    draw.ellipse((cursor_x-4, dot_y-4, cursor_x+4, dot_y+4),
                                 fill=COLORS[role], outline='white', width=1)
            draw.text((right, y0-28), value, font=self.font, fill='#21334a', anchor='rt')
        size = (round(694*self.scale), round(648*self.scale))
        if panel.size != size:
            panel = panel.resize(size, Image.Resampling.LANCZOS)
        image.paste(panel, (round(906*self.scale), round(126*self.scale)))
        return image

    def export(self, video_path):
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        csv_path = video_path.with_suffix('.encoders.csv')
        with csv_path.open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=['bag_time_ns', 'time_s', 'role', 'angle_rad', 'seq', 'boot_id', 'wheel_id', 'device_id', 'segment'])
            writer.writeheader()
            writer.writerows(sorted((row for rows in self.series.values() for row in rows), key=lambda row: row['bag_time_ns']))
        figure = Figure(figsize=(12, 8), dpi=150, facecolor=BACKGROUND)
        FigureCanvasAgg(figure)
        axes = figure.subplots(3, 1, sharex=True)
        figure.subplots_adjust(left=.09, right=.97, bottom=.12, top=.85, hspace=.36)
        figure.text(.09, .95, 'Ball caster encoder angles', size=20, weight='bold', color='#21334a')
        figure.text(.09, .905, self.bag_name+'  |  Recorded span '+f'{self.duration:.3f} s', size=12, color='#687b90')
        for ax, role in zip(axes, ROLES.values()):
            self.style_axes(ax, role)
            ax.set_title(role, loc='left', color=COLORS[role], weight='bold', fontsize=12)
        axes[-1].set_xlabel('Elapsed bag time (s)', fontsize=12)
        figure.text(.09, .028, 'All valid readings · host receipt timing · H1/H2 multi-turn · CENTER wraps and missing samples break curves',
                    size=10, color='#687b90')
        paths = [video_path.with_suffix('.encoders.png'), video_path.with_suffix('.encoders.pdf')]
        for path in paths:
            figure.savefig(path, dpi=150, facecolor=BACKGROUND)
        self.metadata['files'] = [str(path) for path in [*paths, csv_path]]
