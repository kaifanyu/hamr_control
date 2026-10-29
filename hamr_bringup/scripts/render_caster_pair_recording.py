#!/usr/bin/env python3
"""Replay six recorded encoders on the HAMR chassis using one common clock."""
from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import shutil
import subprocess
import time
import xml.etree.ElementTree as ET

from caster_encoder_protocol import WHEELS
from caster_replay_model import DEFAULT_MAPPING, ReplayModel
from render_caster_encoder_video import (
    digest, frame_times, ground_start_mapping, package_share, sample_frames,
)


def paired_mappings(events, args):
    """Keep each wheel's physical reference and configurable signs independent."""
    import yaml
    selection = getattr(args, 'center_reference', 'auto')
    path = None
    defaults = {}
    if selection != 'none':
        path = (package_share('hamr_bringup') / 'config/caster_encoder_replay_pair.yaml'
                if selection == 'auto' else Path(selection).expanduser().resolve())
        defaults = yaml.safe_load(path.read_text())
        if not isinstance(defaults, dict) or set(defaults) != set(WHEELS):
            raise ValueError('Paired reference YAML must have front_left and front_right mappings')
    mappings, provenance = {}, {}
    for wheel in WHEELS:
        mapping = dict(defaults.get(wheel, {}))
        config = getattr(args, wheel.removeprefix('front_') + '_mapping_config', None)
        config = config or args.mapping_config
        stale = 1.
        if config:
            params = yaml.safe_load(Path(config).read_text())['replay_caster_encoder_joints']['ros__parameters']
            # An explicitly supplied mapping defines the complete mapping for this
            # wheel, including whether its absolute reference is enabled.
            mapping = {key: params[key] for key in DEFAULT_MAPPING if key in params}
            stale = params.get('stale_timeout_s', 1.)
        if not math.isfinite(stale) or stale <= 0:
            raise ValueError('stale_timeout_s must be finite and positive')
        model = ReplayModel(mapping, stale, wheel)
        info = {'reference_config': str(config or path) if config or path else None,
                'stale_timeout_s': stale, 'mode': 'recorded', 'assumed': False}
        if model.mapping['center_reference_enabled']:
            info['mode'] = 'absolute-center-reference'
            info['note'] = ('Saved wheel/device-specific raw CENTER reference; '
                            'H1/H2 shell phases and PPR remain unverified.')
        elif args.start_pose == 'ground-contact':
            robot = ET.parse(package_share('hamr_ball_caster') / 'urdf/ball_caster.urdf').getroot()
            mapping, ground_info = ground_start_mapping(events[wheel], robot, mapping, stale, wheel)
            info.update(ground_info)
        mappings[wheel] = mapping
        provenance[wheel] = info
    return mappings, provenance


def paired_frames(events, times, mappings, references):
    """Both models sample the same clock; no per-wheel restart or frame offset."""
    frames = {wheel: sample_frames(events[wheel], times, mappings[wheel],
                                  references[wheel]['stale_timeout_s'], wheel) for wheel in WHEELS}
    return [{wheel: frames[wheel][i] for wheel in WHEELS} for i in range(len(times))]


class PairOverlay:
    def __init__(self, width, recording_name, start, end, rate, recovered, trajectory, references, time_basis):
        from PIL import ImageFont
        self.scale = width / 1920
        self.name, self.start, self.duration, self.rate = recording_name, start, (end-start)/1e9, rate
        self.recovered = recovered
        self.time_basis = time_basis
        self.trajectory, self.references = trajectory, references
        root = Path('/usr/share/fonts/truetype/dejavu')
        self.fonts = {name: ImageFont.truetype(str(root / file), max(8, round(size*self.scale)))
                      for name, file, size in [('title', 'DejaVuSans-Bold.ttf', 34),
                                               ('label', 'DejaVuSans-Bold.ttf', 22),
                                               ('text', 'DejaVuSans.ttf', 21),
                                               ('small', 'DejaVuSans.ttf', 18)]}

    def draw(self, image, frames):
        from PIL import ImageDraw
        draw, s = ImageDraw.Draw(image), self.scale
        def text(x, y, value, font='text', fill='#21334a'):
            draw.text((x*s, y*s), value, font=self.fonts[font], fill=fill)
        draw.rectangle((0, 0, image.width, 126*s), fill='#12263b')
        draw.rectangle((0, 954*s, image.width, image.height), fill='#12263b')
        elapsed = (frames['front_left']['stamp_ns']-self.start)/1e9
        text(32, 22, 'HAMR | synchronized front caster replay', 'title', '#f7fbff')
        text(34, 76, self.name, 'text', '#c5d4e2')
        text(1490, 28, f'{elapsed:05.2f} / {self.duration:.2f} s   |   {self.rate:g}x', 'label', '#90e2cc')
        text(28, 140, 'HAMR SIMULATION MODEL', 'label')
        text(28, 170, 'Fixed body · Simulation Studio CAD · native caster size', 'small', '#687b90')
        text(28, 622, 'FRONT LEFT', 'label')
        text(500, 622, 'FRONT RIGHT', 'label')
        for wheel, x in [('front_left', 28), ('front_right', 500)]:
            states = frames[wheel]['status']['nodes']
            issue = [node['role'] + ': ' + node['display_state'].replace('_', ' ')
                     for node in states if node['display_state'] != 'fresh']
            text(x, 923, ', '.join(issue) if issue else 'H1 / H2 / CENTER: measured pose', 'small', '#687b90')
        text(32, 971, 'H1 = blue    H2 = orange    CENTER = carrier orientation    White dots = shell rotation markers',
             'text', '#f7fbff')
        timing = ('Approximate clock recovery enabled' if self.recovered else
                  'Bag receipt timing' if 'receipt time; batched' in self.time_basis else
                  'Estimated encoder acquisition timing')
        trajectory_note = ('trajectory: ' + str(self.trajectory['topic']) if self.trajectory.get('available')
                           else 'no matching robot trajectory')
        text(32, 1007, timing + ' · synchronization accuracy unvalidated · ' + trajectory_note, 'small', '#c5d4e2')
        reference_note = ('Saved CENTER references' if all(r['mode'] == 'absolute-center-reference' for r in self.references.values())
                          else 'CENTER pose: ' + ' / '.join(self.references[w]['mode'] for w in WHEELS))
        text(32, 1037, reference_note + '; shell phases/PPR unverified. Unmeasured joints held at nominal pose.',
             'small', '#c5d4e2')
        return image


def render_pair(args, source, output, recording=None, trajectory_data=None):
    from caster_pair_recording import load_pair_recording, write_pair_replay_bag
    from caster_pair_scene import Scene
    from caster_pair_plots import PairEncoderPlots
    from caster_pair_trajectory import load_pair_trajectory
    from caster_pair_trajectory_plot import PairTrajectoryPlot

    print(f'Reading both casters: {source}', flush=True)
    events, provenance = (recording if recording is not None else
                          load_pair_recording(source, recover_timing=args.recover_timing))
    start, end = provenance['start_ns'], provenance['end_ns']
    duration = (end-start)/1e9
    if args.preview_time is not None and args.preview_time > duration:
        raise ValueError(f'--preview-time must be within 0..{duration:.3f} s')
    mappings, references = paired_mappings(events, args)
    positions, trajectory_info = (trajectory_data if trajectory_data is not None else
                                  load_pair_trajectory(source, start, end, args.trajectory_bag, args.trajectory_topic))
    print('Trajectory: ' + (f'{trajectory_info["topic"]}, {len(positions)} positions' if positions
                            else trajectory_info['reason']), flush=True)
    times = ([start + round(args.preview_time*1e9)] if args.preview_time is not None
             else frame_times(start, end, args.fps, args.rate))
    # Preview at an early boundary may have no complete pose yet. Sample the
    # entire timeline first so unknown poses are visible rather than fabricated.
    if args.preview_time is not None:
        sampled = paired_frames(events, sorted(set([*times, end])), mappings, references)
        frames = sampled[:1]
    else:
        frames = paired_frames(events, times, mappings, references)
    print(f'{duration:.3f} s, {len(frames)} frames; samples: {provenance["sample_counts"]}', flush=True)
    output.mkdir(parents=True, exist_ok=True)
    stem = f'{source.name}_both_{args.rate:g}x' + ('' if args.no_plots else '_plots')
    video = output / (stem + '.mp4')
    replay_bag = None
    if recording is None:
        replay_bag = output / 'replay_bag' / source.name
        replay_bag.parent.mkdir(parents=True, exist_ok=True)
        write_pair_replay_bag(replay_bag, events, provenance)
    plots = (None if args.no_plots else PairEncoderPlots(
        events, start, end, args.width, source.name, provenance['time_basis']))
    trajectory = PairTrajectoryPlot(positions, start, end, args.width, trajectory_info)
    scene = Scene(args.width, args.width*9//16, args.full_detail)
    overlay = PairOverlay(args.width, source.name, start, end, args.rate,
                          bool(provenance.get('recovered_clock_mappings')), trajectory_info, references,
                          provenance['time_basis'])
    def draw(frame):
        image = overlay.draw(scene.render(frame), frame)
        image = image if plots is None else plots.draw(image, frame)
        return trajectory.draw(image, frame['front_left']['stamp_ns'])
    summary = {
        'mode': 'measured_motion_kinematic_replay', 'source_directory': str(source),
        'replay_bag': str(replay_bag) if replay_bag else None, 'wheel_ids': list(WHEELS), 'rate': args.rate,
        'fps': args.fps, 'frames': len(frames), 'width': args.width, 'height': args.width*9//16,
        'start_ns': start, 'end_ns': end, 'recording_duration_s': duration,
        'local_recording': provenance, 'model': scene.provenance,
        'mapping': {wheel: frames[-1][wheel]['status']['mapping'] for wheel in WHEELS},
        'references': references, 'robot_body_animated': False,
        'trajectory': trajectory.metadata,
        'timing': provenance['time_basis'] + ' Hold last valid sample, without interpolation.',
        'renderer_sha256': digest(__file__),
    }
    wall = time.monotonic()
    try:
        if args.preview_time is not None:
            preview = output / (stem + '_preview.png')
            draw(frames[0]).save(preview)
            summary.update(preview=str(preview), preview_stamp_ns=times[0])
            report = preview.with_suffix('.json')
            print(preview, flush=True)
        else:
            ffmpeg = shutil.which('ffmpeg')
            if not ffmpeg:
                raise RuntimeError('ffmpeg is required')
            temporary = video.with_suffix('.partial.mp4')
            command = [ffmpeg, '-nostdin', '-hide_banner', '-loglevel', 'error', '-n',
                       '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size',
                       f'{args.width}x{args.width*9//16}', '-framerate', str(args.fps),
                       '-i', 'pipe:0', '-an', '-c:v', 'libx264', '-preset', 'fast',
                       '-crf', '18', '-threads', '4', '-pix_fmt', 'yuv420p',
                       '-movflags', '+faststart', str(temporary)]
            with video.with_suffix('.encoding.log').open('w') as log:
                encoder = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
                try:
                    with video.with_suffix('.frames.csv').open('w', newline='') as stream:
                        writer = csv.writer(stream)
                        writer.writerow(['frame', 'video_time_s', 'replay_time_ns', 'elapsed_s', 'wheel_id',
                                         'h1_model_rad', 'h2_model_rad', 'center_model_rad',
                                         'h1_state', 'h2_state', 'center_state'])
                        for i, frame in enumerate(frames):
                            image = draw(frame)
                            encoder.stdin.write(image.tobytes())
                            for wheel in WHEELS:
                                stamp, nodes = frame[wheel]['stamp_ns'], frame[wheel]['status']['nodes']
                                writer.writerow([i, i/args.fps, stamp, (stamp-start)/1e9, wheel]
                                                + [n['model_position_rad'] for n in nodes]
                                                + [n['display_state'] for n in nodes])
                            if i in (min(len(frames)-1, args.fps), len(frames)//2, len(frames)-1):
                                image.save(output / f'{stem}_frame_{i:05d}.png')
                            if i % (args.fps*5) == 0:
                                print(f'{i}/{len(frames)} frames; wall {time.monotonic()-wall:.1f}s', flush=True)
                    encoder.stdin.close()
                    if encoder.wait(timeout=60):
                        raise RuntimeError('ffmpeg failed; inspect the encoding log')
                finally:
                    if encoder.poll() is None:
                        encoder.terminate()
                        encoder.wait(timeout=15)
            temporary.replace(video)
            summary.update(video=str(video), video_sha256=digest(video),
                           video_duration_s=len(frames)/args.fps,
                           duration_padding_s=len(frames)/args.fps-duration/args.rate)
            report = video.with_suffix('.json')
            print(video, flush=True)
        if plots:
            plots.export(video)
            summary['encoder_plots'] = plots.metadata
        trajectory.export(video)
        summary['wall_render_s'] = time.monotonic()-wall
        report.write_text(json.dumps(summary, indent=2) + '\n')
        return summary
    finally:
        scene.window.Finalize()
