#!/usr/bin/env python3
"""Read the original MCAP and create reproducible plots; never modify source data.

Run from a sourced ROS environment:
  python3 analyze_combined_caster_bag.py /path/to/encoder_run --output-dir /path/to/report
Requires ROS rosbag2 MCAP reader, NumPy, SciPy, matplotlib, PyYAML.
"""
from pathlib import Path
from collections import defaultdict
from datetime import datetime
from zoneinfo import ZoneInfo
import argparse
import csv
import hashlib
import json
import shlex
import struct
import warnings

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from scipy.spatial.transform import Rotation, Slerp
from scipy.signal import savgol_filter

# Set only after explicit CLI input validation. Importing numerical helpers never
# reads a machine-specific recording or writes into the source tree.
OUT = ROOT = BAG = META = T0 = DURATION = None
BAG_FILES = []
POSES = ['/HAMR_base/pose', '/Left_Caster/pose', '/Right_Caster/pose', '/HAMR_turret/pose']
NAMES = ['Base', 'Left fork', 'Right fork', 'Turret']
COLORS = ['#2563a6', '#d48616', '#c53d65', '#60826c']
ENC = [f'/caster_recording/{wheel}/{role}/joint_states'
       for wheel in ('front_left', 'front_right') for role in ('H1', 'H2', 'CENTER')]
STATUS_FIELDS = ['left_target_rpm', 'left_measured_rpm', 'left_ff_pwm', 'left_pid_pwm',
                 'left_output_pwm', 'left_error_rpm', 'left_integral_rpm_s', 'left_saturated',
                 'right_target_rpm', 'right_measured_rpm', 'right_ff_pwm', 'right_pid_pwm',
                 'right_output_pwm', 'right_error_rpm', 'right_integral_rpm_s', 'right_saturated', 'source']


def configure(source, output_dir, sidecar_dir=None):
    """Resolve a recording/bag without substituting files from another run."""
    global OUT, ROOT, BAG, META, T0, DURATION, BAG_FILES
    source = Path(source).expanduser().resolve()
    if not source.exists():
        raise ValueError(f'Input does not exist: {source}')
    recording = source.is_dir() and (source / 'bags/combined/metadata.yaml').is_file()
    bag_dir = source / 'bags/combined' if recording else (source.parent if source.is_file() else source)
    metadata_path = bag_dir / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError(f'Missing bag metadata: {metadata_path}')
    metadata = yaml.safe_load(metadata_path.read_text())['rosbag2_bagfile_information']
    names = metadata.get('relative_file_paths', [])
    files = [(bag_dir / name).resolve() for name in names]
    if not files or any(not p.is_file() or p.suffix not in ('.mcap', '.db3') or bag_dir not in p.parents for p in files):
        raise ValueError('Bag metadata must identify complete local MCAP/SQLite storage files')
    if source.is_file() and (len(files) != 1 or source != files[0]):
        raise ValueError('For a split bag, pass its directory so every metadata-counted shard is analyzed')
    if sidecar_dir:
        root = Path(sidecar_dir).expanduser().resolve()
    elif recording:
        root = source
    elif bag_dir.name == 'combined' and bag_dir.parent.name == 'bags':
        root = bag_dir.parent.parent
    else:
        raise ValueError('Use --sidecar-dir for the recording folder containing clock-map.json and export-summary.json')
    for filename in ('clock-map.json', 'export-summary.json'):
        if not (root / filename).is_file():
            raise ValueError(f'Missing recording sidecar: {root / filename}')
    output = Path(output_dir).expanduser().resolve()
    if output in (root, bag_dir, source):
        raise ValueError('Output must be a separate report directory')
    if output.exists() and (not output.is_dir() or any(output.iterdir())):
        raise ValueError(f'Output directory is not empty: {output}')
    t0 = metadata['starting_time']['nanoseconds_since_epoch']
    duration_ns = metadata['duration']['nanoseconds']
    if type(t0) is not int or type(duration_ns) is not int or duration_ns <= 0:
        raise ValueError('Bag metadata requires integer epoch start and positive duration')
    OUT, ROOT = output, root
    BAG, BAG_FILES, META = files[0] if len(files) == 1 else bag_dir, files, metadata
    T0, DURATION = t0, duration_ns / 1e9


def digest(path):
    checksum = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            checksum.update(block)
    return checksum.hexdigest()


def quaternion_steps_deg(quaternions):
    """Shortest physical 3D steps; q and -q are the same rotation."""
    q = np.asarray(quaternions, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4 or not np.all(np.isfinite(q)):
        raise ValueError('Quaternions must be a finite N-by-4 array')
    norms = np.linalg.norm(q, axis=1)
    if np.any(norms < 1e-8):
        raise ValueError('Quaternion orientation is undefined at zero norm')
    q = q / norms[:, None]
    return np.degrees(2 * np.arccos(np.clip(np.abs(np.sum(q[1:] * q[:-1], axis=1)), 0, 1)))


def common_encoder_window(arrays):
    missing = [topic for topic in ENC if topic not in arrays or len(arrays[topic]) < 2]
    if missing:
        raise ValueError('Need all six encoder streams with at least two samples: ' + ', '.join(missing))
    lo = max(arrays[k][0, 0] for k in ENC)
    hi = min(arrays[k][-1, 0] for k in ENC)
    if not lo < hi:
        raise ValueError('The six encoders have no common recording interval')
    return lo, hi

def htime(msg):
    return (msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec - T0) / 1e9

def quantiles(a):
    a = np.asarray(a)
    a = a[np.isfinite(a)]
    if not len(a):
        return {}
    return dict(zip(['min', 'median', 'p95', 'p99', 'max'], map(float, np.percentile(a, [0, 50, 95, 99, 100]))))

def intervals(t, mask):
    """Contiguous sampled intervals, with no claim beyond sampled endpoints."""
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [[float(t[v[0]]), float(t[v[-1]])] for v in runs]

def write_csv(path, names, rows):
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(names)
        w.writerows(rows)

def read():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    readers, types = [], {}
    for path in BAG_FILES:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap' if path.suffix == '.mcap' else 'sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for topic in reader.get_all_topics_and_types():
            if topic.name in types and types[topic.name] != topic.type:
                raise ValueError(f'Topic type changes across shards: {topic.name}')
            types[topic.name] = topic.type
        readers.append(reader)
    def messages():
        for reader in readers:
            while reader.has_next():
                yield reader.read_next()
    times, rows, frames, decode_errors = defaultdict(list), defaultdict(list), defaultdict(set), []
    classes = {}
    wanted = set(POSES + ENC + ['/HAMR_base/odom', '/HAMR_turret/odom', '/Left_Caster/odom',
                               '/Right_Caster/odom', '/local_HAMR/odom', '/wheel_odom',
                               '/waypoints_path', '/reference_trajectory', '/wheel_control/status',
                               '/left_wheel/cmd_vel', '/right_wheel/cmd_vel', '/turret/cmd_vel'])
    count = 0
    for topic, data, ns in messages():
        t = (ns - T0) / 1e9
        times[topic].append(t)
        count += 1
        if topic not in wanted:
            continue
        try:
            if topic == '/reference_trajectory':
                # Exact repo ReferenceTraj.msg: ten float64 fields, no header.
                # CDR encapsulation is four bytes; fields aligned relative to payload.
                if len(data) != 84 or data[:2] not in (b'\x00\x01', b'\x00\x00'):
                    raise ValueError('Unexpected ReferenceTraj CDR schema')
                values = struct.unpack(('<' if data[1] == 1 else '>') + '10d', data[4:])
                rows[topic].append([t, *values])
                continue
            if topic not in classes:
                classes[topic] = get_message(types[topic])
            cls = classes[topic]
            m = deserialize_message(data, cls)
            if types[topic] in ('geometry_msgs/msg/PoseStamped', 'nav_msgs/msg/Odometry'):
                p = m.pose.pose if types[topic] == 'nav_msgs/msg/Odometry' else m.pose
                frames[topic].add(m.header.frame_id)
                rows[topic].append([t, htime(m), p.position.x, p.position.y, p.position.z,
                                    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
            elif topic in ENC:
                if len(m.position) != 1:
                    raise ValueError('Expected a single encoder position')
                rows[topic].append([t, htime(m), m.position[0], m.velocity[0] if m.velocity else np.nan])
            elif topic == '/wheel_control/status':
                if len(m.data) != 17 or not m.layout.dim or m.layout.dim[0].label != 'wheel_control_status_v1':
                    raise ValueError('Unexpected wheel status schema')
                rows[topic].append([t, *m.data])
            elif topic == '/waypoints_path':
                frames[topic].add(m.header.frame_id)
                if topic not in rows:
                    rows[topic] = [[p.pose.position.x, p.pose.position.y] for p in m.poses]
            else:
                rows[topic].append([t, m.data])
        except Exception as e:
            decode_errors.append({'topic': topic, 'time_s': t, 'error': str(e)})
    arrays = {k: np.asarray(v, dtype=float) for k, v in rows.items()}
    counts_expected = {x['topic_metadata']['name']: x['message_count'] for x in META['topics_with_message_count']}
    mismatches = {k: [n, len(times[k])] for k, n in counts_expected.items() if len(times[k]) != n}
    if decode_errors or mismatches or count != META['message_count']:
        raise RuntimeError(json.dumps({'errors': decode_errors[:20], 'count_mismatches': mismatches, 'total': count}))
    topic_rows = []
    for topic in sorted(types):
        a = np.asarray(times[topic])
        dt = np.diff(a)
        topic_rows.append(dict(topic=topic, type=types[topic], count=len(a),
                               start_s=float(a[0]) if len(a) else None, end_s=float(a[-1]) if len(a) else None,
                               median_interval_ms=float(np.median(dt) * 1e3) if len(dt) else None,
                               max_interval_ms=float(np.max(dt) * 1e3) if len(dt) else None,
                               nonpositive_intervals=int(np.sum(dt <= 0))))
    return arrays, topic_rows, {k: sorted(v) for k, v in frames.items()}

def title(fig, text, subtitle):
    fig.suptitle(text, x=.065, y=.978, ha='left', fontsize=19, fontweight='bold', color='#16324c')
    fig.text(.065, .94, subtitle, ha='left', fontsize=10, color='#526479')
    fig.text(.065, .018, f'HAMR | {ROOT.name} | Time = seconds from combined-bag start; encoder time is estimated.',
             fontsize=8, color='#526479')

def decorate(ax, ylabel=None, xlabel=True):
    ax.grid(alpha=.2)
    ax.spines[['top', 'right']].set_visible(False)
    if ylabel:
        ax.set_ylabel(ylabel)
    if xlabel:
        ax.set_xlabel('Bag elapsed time (s)')

def save(fig, name, pdf):
    fig.savefig(OUT / (name + '.png'), dpi=170, facecolor='white')
    pdf.savefig(fig, facecolor='white')
    plt.close(fig)

def analyze():
    print('Reading original MCAP...', flush=True)
    arrays, topic_rows, frames = read()
    lo, hi = common_encoder_window(arrays)
    required = POSES + ['/wheel_control/status', '/reference_trajectory', '/waypoints_path']
    missing = [topic for topic in required if topic not in arrays or not len(arrays[topic])]
    if missing:
        raise ValueError('Seven-page report requires these missing topics: ' + ', '.join(missing))
    for topic in POSES:
        if len(frames.get(topic, [])) != 1 or not frames[topic][0]:
            raise ValueError(f'Pose topic must use one nonempty coordinate frame: {topic}')
        quaternion_steps_deg(arrays[topic][:, 5:9])
    if len({frames[topic][0] for topic in POSES}) != 1:
        raise ValueError('Base and fork pose topics must share one recorded coordinate frame')
    OUT.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(OUT / 'extracted_signals.npz', **arrays)
    write_csv(OUT / 'topics.csv', list(topic_rows[0]), [list(r.values()) for r in topic_rows])
    overlap = lambda a: (a[:, 0] >= lo) & (a[:, 0] <= hi)
    source_hashes = {str(path): digest(path) for path in BAG_FILES}
    report = dict(source=str(BAG), sha256=source_hashes.get(str(BAG)), source_sha256=source_hashes,
                  start_epoch_ns=T0, analysis_script_sha256=digest(__file__),
                  start_local=datetime.fromtimestamp(T0 / 1e9, ZoneInfo('America/New_York')).isoformat(),
                  duration_s=DURATION, total_messages=META['message_count'], metadata_counts_verified=True,
                  common_encoder_window_s=[float(lo), float(hi)], common_encoder_duration_s=float(hi-lo),
                  frames=frames, topics=topic_rows, poses={}, encoders={}, drive={}, notes=[
                      'All time-series axes use recorded bag time; encoder bag time was estimated from individual acquisition clocks.',
                      'Raw Vicon and encoder angles are not corrected, fitted, or transformed into CAD coordinates.',
                      'CENTER unwrapping uses nearest increments; inspect continuity, sample gaps and largest steps before interpreting net turns. No phase calibration applied.',
                      'Vicon quaternion jump metrics use absolute quaternion dot products and cannot be caused by q/-q representation flips.',
                      'Base path/speed summary uses 50 Hz resampling and a 0.42 s Savitzky-Golay window; it is a descriptive smoothed estimate.',
                      'Reference X/Y is plotted in its recorded numerical coordinates; no new frame alignment or calibration is inferred.',
                      'No independent ball-orientation prediction or encoder-accuracy claim is made.'])
    export = json.loads((ROOT / 'export-summary.json').read_text())
    clock = json.loads((ROOT / 'clock-map.json').read_text())
    report['encoder_export_quality'] = export
    report['clock_mapping'] = clock
    report['sidecar_sha256'] = {filename: digest(ROOT / filename) for filename in ('export-summary.json', 'clock-map.json')}
    events = []
    base = arrays['/HAMR_base/pose']
    order = np.argsort(base[:, 1])
    base_unique = base[order][np.r_[True, np.diff(base[order, 1]) > 0]]
    base_slerp = Slerp(base_unique[:, 1], Rotation.from_quat(base_unique[:, 5:9]))
    relatives, eulers = {}, {}
    for topic in POSES:
        a = arrays[topic]
        q = a[:, 5:9]
        norms = np.linalg.norm(q, axis=1)
        if not np.all(np.isfinite(a)) or np.any(norms < 1e-8):
            raise ValueError(f'Invalid pose data in {topic}')
        q = q / norms[:, None]
        rot = Rotation.from_quat(q)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            eulers[topic] = rot.as_euler('xyz', degrees=True)
        step = quaternion_steps_deg(q)
        pos_step = np.linalg.norm(np.diff(a[:, 2:5], axis=0), axis=1)
        m = overlap(a)
        md = m[1:] & m[:-1]
        report['poses'][topic] = dict(count=len(a), xyz_range_m=np.ptp(a[m, 2:5], axis=0).tolist(),
            overlap_z_m=quantiles(a[m, 4]), quaternion_norm=quantiles(norms),
            overlap_rotation_step_deg=quantiles(step[md]), steps_above_10deg=int(np.sum(step[md] > 10)),
            overlap_position_step_mm=quantiles(1000*pos_step[md]),
            overlap_header_interval_ms=quantiles(1000*np.diff(a[:, 1])[md]),
            overlap_bag_minus_header_ms=quantiles(1000*(a[m, 0]-a[m, 1])))
        for idx in np.flatnonzero(step > 10):
            events.append([topic, a[idx+1, 0], a[idx+1, 0]-lo, step[idx], pos_step[idx]*1000,
                           (a[idx+1, 1]-a[idx, 1])*1000])
        mask = m & (a[:, 1] >= base_unique[0, 1]) & (a[:, 1] <= base_unique[-1, 1])
        rel = base_slerp(a[mask, 1]).inv() * rot[mask]
        initial = a[mask, 0] < lo+2
        r0 = rel[initial].mean()
        err = np.degrees((r0.inv() * rel).magnitude())
        relatives[topic] = np.column_stack((a[mask, 0], err))
        report['poses'][topic]['relative_to_base_rotation_change_deg'] = quantiles(err)
        report['poses'][topic]['relative_to_base_above_30deg_samples'] = int(np.sum(err > 30))
        write_csv(OUT / (topic.strip('/').replace('/', '_') + '.csv'),
                  ['bag_time_s', 'header_time_s', 'x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw'], a)
    write_csv(OUT / 'vicon_orientation_jumps.csv', ['topic', 'bag_time_s', 'encoder_overlap_elapsed_s',
        'rotation_step_deg', 'position_step_mm', 'header_interval_ms'], events)
    enc_processed = {}
    enc_csv = []
    for topic in ENC:
        a = arrays[topic]
        angle = np.unwrap(a[:, 2]) if '/CENTER/' in topic else a[:, 2]
        delta = angle-angle[0]
        enc_processed[topic] = delta
        report['encoders'][topic] = dict(count=len(a), start_s=float(a[0, 0]), end_s=float(a[-1, 0]),
             net_change_rad=float(delta[-1]), net_turns=float(delta[-1]/(2*np.pi)),
             excursion_rad=float(np.ptp(angle)), total_abs_angle_change_rad=float(np.sum(np.abs(np.diff(angle)))),
             largest_unwrapped_step_rad=float(np.max(np.abs(np.diff(angle)))),
             peak_abs_reported_velocity_rad_s=float(np.nanmax(np.abs(a[:, 3]))),
             median_interval_ms=float(np.median(np.diff(a[:, 0]))*1000),
             max_interval_ms=float(np.max(np.diff(a[:, 0]))*1000))
        for row, d in zip(a, delta):
            enc_csv.append([topic, row[0], row[2], d, row[3]])
    write_csv(OUT / 'encoder_signals.csv', ['topic', 'bag_time_s', 'recorded_angle_rad',
              'unwrapped_change_from_first_sample_rad', 'reported_velocity_rad_s'], enc_csv)
    # Smooth base translation only to describe motion, not to modify the raw evidence.
    grid = np.arange(base_unique[0, 1], base_unique[-1, 1], .02)
    xyz = np.column_stack([np.interp(grid, base_unique[:, 1], base_unique[:, k]) for k in (2, 3, 4)])
    smooth = savgol_filter(xyz, 21, 3, axis=0)
    speed = np.linalg.norm(np.gradient(smooth[:, :2], .02, axis=0), axis=1)
    grid_bag = np.interp(grid, base_unique[:, 1], base_unique[:, 0])
    gm = (grid_bag >= lo) & (grid_bag <= hi)
    moving = (speed > .03) & gm
    report['base_motion'] = dict(overlap_smoothed_path_length_m=float(np.sum(np.linalg.norm(np.diff(smooth[gm, :2], axis=0), axis=1))),
        overlap_speed_m_s=quantiles(speed[gm]), moving_intervals_above_003_m_s=intervals(grid_bag, moving),
        overlap_start_xy=base[overlap(base)][0, 2:4].tolist(), overlap_end_xy=base[overlap(base)][-1, 2:4].tolist(),
        overlap_net_displacement_m=float(np.linalg.norm(base[overlap(base)][-1, 2:4]-base[overlap(base)][0, 2:4])))
    status = arrays['/wheel_control/status']
    sm = overlap(status)
    report['drive']['source_codes'] = sorted(set(status[:, -1].astype(int).tolist()))
    for wheel, offset in [('left', 1), ('right', 9)]:
        target, measured = status[:, offset], status[:, offset+1]
        active = sm & (np.abs(target) > .5)
        err = measured[active]-target[active]
        report['drive'][wheel] = dict(target_rpm=quantiles(target[sm]), measured_rpm=quantiles(measured[sm]),
            active_rpm_rmse=float(np.sqrt(np.mean(err**2))),
            active_median_measured_minus_target_rpm=float(np.median(err)),
            overlap_saturation_samples=int(np.sum(status[sm, offset+7] > .5)),
            active_saturation_sample_fraction=float(np.mean(status[active, offset+7] > .5)))
    ref = arrays['/reference_trajectory']
    report['reference'] = dict(first_s=float(ref[0, 0]), last_s=float(ref[-1, 0]),
        max_gap_s=float(np.max(np.diff(ref[:, 0]))), waypoints=arrays['/waypoints_path'].tolist())
    missing_samples = sum(node.get('missing_samples', 0) for node in export['nodes'])
    invalid_samples = sum(node['sample_count'] - node['flag_counts'].get('sample_valid', 0) for node in export['nodes'])
    hz = [1000 / report['encoders'][topic]['median_interval_ms'] for topic in ENC]
    uncertainty_ms = [entry['uncertainty_ns'] / 1e6 for entry in clock['mappings'] if 'uncertainty_ns' in entry]
    (OUT / 'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    notes = [
        'COMBINED MCAP RECORDING ANALYSIS',
        f'Source: {BAG}',
        f'SHA256: {report["sha256"]}',
        f'Recorded start: {report["start_local"]}',
        f'Duration: {DURATION:.3f} s; {META["message_count"]:,} messages. Every topic count matches bag metadata.',
        '',
        'This report retains the recorded six-channel encoder history and robot/fork pose observations.',
        f'Fork orientation steps above 10 degrees during overlap: left {report["poses"]["/Left_Caster/pose"]["steps_above_10deg"]}; right {report["poses"]["/Right_Caster/pose"]["steps_above_10deg"]}.',
        '',
        f'All six encoder streams overlap from {lo:.3f} to {hi:.3f} s after bag start ({hi-lo:.3f} s).',
        f'The smoothed base trajectory travels approximately {report["base_motion"]["overlap_smoothed_path_length_m"]:.2f} m.',
        'Inspect the measured path and recorded waypoint plots to assess route shape.',
        'The reference route and raw Vicon paths are plotted separately; frame calibration is not assumed.',
        f'Base peak smoothed planar speed is {np.max(speed[gm]):.3f} m/s.',
        f'Its start-to-finish displacement during encoder coverage is {report["base_motion"]["overlap_net_displacement_m"]:.3f} m.',
        'Motion above 0.03 m/s occurs in these smoothed sampled intervals (bag seconds):',
        str(report['base_motion']['moving_intervals_above_003_m_s']),
        '',
        'ENCODERS',
        f'Median sample rates span {min(hz):.2f}–{max(hz):.2f} Hz; local export reports {missing_samples} missing sequence samples and {invalid_samples} samples without sample_valid.',
        'Inspect encoder_export_quality in summary.json for per-board PPR, phase-reference and clock-sync flags. Continuity does not prove physical angle calibration.',
        'H1/H2 shell rotation and CENTER carrier rotation are different quantities; their patterns alone cannot identify contact state, slip, or accuracy.',
        'Net angle changes over each full encoder stream (raw sign convention):',
    ]
    for topic in ENC:
        s = report['encoders'][topic]
        notes.append(f'  {topic}: {s["net_change_rad"]:+.3f} rad ({s["net_turns"]:+.4f} turns)')
    notes += ['', 'VICON',
        'Quaternion comparisons use the shortest 3D rotation angle, with q/-q equivalence handled explicitly.',
        'Large steps therefore are not merely quaternion sign flips or Euler wrapping.']
    for topic in POSES[:3]:
        s = report['poses'][topic]
        notes.append(f'  {topic}: {s["steps_above_10deg"]} adjacent steps >10 deg; max {s["overlap_rotation_step_deg"]["max"]:.3f} deg; max 3D position step {s["overlap_position_step_mm"]["max"]:.2f} mm.')
    notes += [
        'Large fork orientation steps that persist relative to the base can indicate tracking/model ambiguity; no cause is inferred automatically.',
        'Quaternion sign equivalence is removed; no rigid-body flip correction or marker-frame calibration is applied.',
        'Position steps include real travel and variable sample intervals; compare their scale with the orientation jump table.',
        'Reject or resolve fork orientation faults before differentiating poses or applying a marker-to-ball-center lever arm.',
        '', 'DRIVE RESPONSE',
    ]
    for wheel in ('left','right'):
        s=report['drive'][wheel]
        notes.append(f'  {wheel}: active target-vs-measured RMS error {s["active_rpm_rmse"]:.2f} RPM; saturation flag on {100*s["active_saturation_sample_fraction"]:.1f}% of active samples.')
    notes += [
        'Active means |target| >0.5 RPM inside common encoder coverage; transient errors are included.',
        'Use the reported target/measured errors to assess drive response; later caster prediction should use measured motion with quality checks.',
        '', 'TIMING',
        'Vicon header interval and bag-minus-header quantiles are retained per pose topic in summary.json.',
        ('Encoder mapping uncertainty fields span ' + f'{min(uncertainty_ms):.2f}–{max(uncertainty_ms):.2f} ms; these are estimates, not validated bounds.'
         if uncertainty_ms else 'No encoder mapping uncertainty fields are available.'),
        'Bag-minus-header differences are not end-to-end Vicon capture latency.',
        'No fitted lag, timing shift, pose correction, or encoder-to-CAD mapping has been applied.',
        '', 'METHODS AND OUTPUTS',
        *report['notes'],
        'combined_bag_analysis.pdf contains seven plot pages; corresponding PNG files allow individual viewing.',
        'summary.json contains numerical results and quality metadata; CSV files contain signals, topic inventory and jump timestamps.',
        'extracted_signals.npz stores selected decoded numeric channels; unplotted messages remain in the original MCAP.',
        'Original recording files and repository source were not changed.',
        'Reproduce:',
        '  # Source the installed ROS environment first.',
        '  ' + shlex.join(['python3', str(Path(__file__).resolve()), str(BAG), '--sidecar-dir', str(ROOT), '--output-dir', str(OUT.with_name(OUT.name + '_reproduced'))]),
    ]
    (OUT / 'analysis_notes.txt').write_text('\n'.join(notes)+'\n')
    print('Decoded and checked all topic counts. Rendering plots...', flush=True)

    plt.rcParams.update({'font.size': 10, 'axes.titlesize': 12, 'axes.labelsize': 10,
                         'legend.fontsize': 9, 'lines.linewidth': 1.1, 'pdf.fonttype': 42})
    with PdfPages(OUT / 'combined_bag_analysis.pdf') as pdf:
        fig = plt.figure(figsize=(14, 10))
        gs = fig.add_gridspec(3, 2, left=.075, right=.96, bottom=.09, top=.89, hspace=.44, wspace=.30)
        ax = fig.add_subplot(gs[:2, 0])
        for topic, name, color in zip(POSES[:3], NAMES[:3], COLORS[:3]):
            a = arrays[topic]; a = a[overlap(a)]
            ax.plot(a[:, 2], a[:, 3], color=color, label=name+' marker origin', lw=1)
        a = base[overlap(base)]
        ax.scatter(*a[0, 2:4], marker='o', s=65, c='green', zorder=5, label='Base start')
        ax.scatter(*a[-1, 2:4], marker='X', s=65, c='black', zorder=5, label='Base finish')
        ax.set(xlabel='Vicon X (m)', ylabel='Vicon Y (m)', title='Measured paths during encoder overlap')
        ax.axis('equal'); ax.grid(alpha=.2); ax.legend(loc='lower left', fontsize=8)
        ax = fig.add_subplot(gs[0, 1])
        ax.plot(ref[:, 1], ref[:, 2], color='#79828e', label='Recorded reference')
        wp = arrays['/waypoints_path']; ax.plot(wp[:, 0], wp[:, 1], 'o--', color='#26384e', ms=4)
        waypoint_labels = defaultdict(list)
        for i, (x, y) in enumerate(wp):
            waypoint_labels[x,y].append(str(i))
        for (x,y), labels_at_point in waypoint_labels.items():
            ax.annotate(', '.join(labels_at_point), (x,y), xytext=(5,4), textcoords='offset points')
        ax.set(xlabel='Reference X (m)', ylabel='Reference Y (m)', title='Recorded waypoint route (separate frame)')
        ax.axis('equal'); ax.grid(alpha=.2)
        ax = fig.add_subplot(gs[1, 1])
        names = POSES[:3] + ENC
        for i, topic in enumerate(names):
            a = arrays[topic]
            ax.plot([a[0, 0], a[-1, 0]], [i, i], lw=5, color=COLORS[min(i, 2)] if i<3 else '#529777')
        labels = ['Base Vicon', 'Left fork Vicon', 'Right fork Vicon'] + ['FL '+r for r in ('H1','H2','CENTER')] + ['FR '+r for r in ('H1','H2','CENTER')]
        ax.set_yticks(range(len(labels)), labels); ax.invert_yaxis(); ax.set_xlim(0,DURATION)
        ax.axvline(lo,color='gray',ls=':'); ax.axvline(hi,color='gray',ls=':')
        ax.set_title(f'Data coverage | shared encoders {lo:.2f}–{hi:.2f} s')
        decorate(ax)
        ax = fig.add_subplot(gs[2, :])
        ax.plot(grid_bag, speed, color=COLORS[0], label='Base speed: 0.42 s smoothing')
        ax.axvspan(lo,hi,color='#74aa84',alpha=.12,label='All six encoder streams present')
        ax.set_xlim(0,DURATION); ax.set_ylim(bottom=0); decorate(ax,'Planar speed (m/s)'); ax.legend(loc='upper right')
        title(fig,'Combined recording overview', f'{DURATION:.2f} s total • {META["message_count"]:,} messages • {hi-lo:.2f} s common caster recording • original data retained')
        save(fig,'01_overview',pdf)

        fig, axes = plt.subplots(3,2,figsize=(14,10),sharex=True)
        fig.subplots_adjust(left=.085,right=.96,bottom=.09,top=.86,hspace=.34,wspace=.24)
        for col,wheel in enumerate(('front_left','front_right')):
            for row,role in enumerate(('H1','H2','CENTER')):
                topic=f'/caster_recording/{wheel}/{role}/joint_states'; a=arrays[topic]; ax=axes[row,col]
                ax.plot(a[:,0],enc_processed[topic],color=COLORS[col+1])
                s=report['encoders'][topic]
                ax.set_title(f'{"Front left" if col==0 else "Front right"} · {role} | net {s["net_change_rad"]:+.2f} rad')
                ax.set_xlim(lo,hi); decorate(ax,'Angle change (rad)',row==2)
        title(fig,'Six encoder angle histories','First sample subtracted; CENTER unwrapped. Independent y scales; no CAD signs, phase offsets, or smoothing applied.')
        save(fig,'02_encoder_angles',pdf)

        fig,axes=plt.subplots(3,2,figsize=(14,10),sharex=True)
        fig.subplots_adjust(left=.085,right=.96,bottom=.09,top=.86,hspace=.34,wspace=.24)
        for col,wheel in enumerate(('front_left','front_right')):
            for row,role in enumerate(('H1','H2','CENTER')):
                topic=f'/caster_recording/{wheel}/{role}/joint_states';a=arrays[topic];ax=axes[row,col]
                ax.plot(a[:,0],a[:,3],color=COLORS[col+1],lw=.65)
                ax.set_title(f'{"Front left" if col==0 else "Front right"} · {role}'); ax.set_xlim(lo,hi)
                decorate(ax,'Reported rate (rad/s)',row==2)
        title(fig,'Encoder angular velocities','Recorded JointState velocity fields; no differentiation or smoothing added. Missing initial rates remain missing.')
        save(fig,'03_encoder_rates',pdf)

        fig,axes=plt.subplots(3,2,figsize=(14,10),sharex=True)
        fig.subplots_adjust(left=.085,right=.96,bottom=.09,top=.86,hspace=.32,wspace=.25)
        for topic,name,color in zip(POSES[:3],NAMES[:3],COLORS[:3]):
            a=arrays[topic]; m=overlap(a)
            for row in range(3):
                axes[row,0].plot(a[m,0],a[m,2+row],color=color,label=name,lw=.8)
                # Break the plotted line at Euler wrapping; raw samples still shown.
                y=eulers[topic][m,row].copy(); y[np.r_[False,np.abs(np.diff(y))>180]]=np.nan
                axes[row,1].plot(a[m,0],y,color=color,label=name,lw=.8)
        for row in range(3):
            axes[row,0].set_title(['Recorded X','Recorded Y','Recorded Z'][row])
            axes[row,1].set_title(['Roll','Pitch','Yaw'][row]+' (raw marker frame)')
            decorate(axes[row,0],'m',row==2);decorate(axes[row,1],'degrees',row==2)
            axes[row,0].set_xlim(lo,hi);axes[row,1].set_xlim(lo,hi)
        axes[0,0].legend();axes[0,1].legend()
        title(fig,'Raw Vicon position and orientation','Marker origins and axes differ between rigid bodies. Euler wraps are display conventions; 3D jump checks are on the next page.')
        save(fig,'04_vicon_pose',pdf)

        fig,axes=plt.subplots(3,1,figsize=(14,10),sharex=True)
        fig.subplots_adjust(left=.09,right=.96,bottom=.09,top=.86,hspace=.38)
        for topic,name,color in zip(POSES[:3],NAMES[:3],COLORS[:3]):
            a=arrays[topic];m=overlap(a);a=a[m];q=a[:,5:9];q=q/np.linalg.norm(q,axis=1)[:,None]
            step=quaternion_steps_deg(q)
            axes[0].plot(a[1:,0],step,color=color,label=f'{name}: {report["poses"][topic]["steps_above_10deg"]} steps >10°',lw=.8)
            rel=relatives[topic];axes[1].plot(rel[:,0],rel[:,1],color=color,label=name,lw=.9)
            axes[2].plot(a[1:,0],1000*np.linalg.norm(np.diff(a[:,2:5],axis=0),axis=1),color=color,label=name,lw=.7)
        axes[0].axhline(10,color='gray',ls='--',lw=.8)
        axes[0].set_title('True rotation between adjacent poses: |quaternion dot product| removes sign ambiguity')
        axes[1].set_title('Fork orientation relative to base: deviation from its first 2 s mean relative orientation')
        axes[2].set_title('Adjacent 3D marker-position steps (includes genuine motion)')
        for ax,lab in zip(axes,['Rotation step (°)','Relative rotation change (°)','Position step (mm)']):
            decorate(ax,lab);ax.set_xlim(lo,hi);ax.set_ylim(bottom=0);ax.legend(loc='upper right')
        title(fig,'Vicon tracking discontinuities','Raw data, with no automatic correction. Large relative rotations of fixed forks suggest tracking/model ambiguity; cause is unconfirmed.')
        save(fig,'05_vicon_quality',pdf)

        fig,axes=plt.subplots(3,1,figsize=(14,10),sharex=True)
        fig.subplots_adjust(left=.09,right=.96,bottom=.09,top=.86,hspace=.38)
        for wheel,offset,color,ax in zip(('left','right'),(1,9),COLORS[1:3],axes[:2]):
            ax.plot(status[:,0],status[:,offset+1],color=color,alpha=.65,lw=.8,label='Measured')
            ax.plot(status[:,0],status[:,offset],color='#1c334f',ls='--',lw=1,label='Target')
            ax.set_title(f'{wheel.title()} drive wheel (not passive caster)'); decorate(ax,'RPM');ax.legend()
        for wheel,offset,color in zip(('left','right'),(1,9),COLORS[1:3]):
            axes[2].plot(status[:,0],status[:,offset+4],color=color,label=wheel.title())
        decorate(axes[2],'Reported PWM');axes[2].set_title('Drive controller outputs');axes[2].legend()
        for ax in axes:ax.set_xlim(lo,hi)
        title(fig,'Drive commands and measured response','wheel_control_status_v1 fields, decoded using the repository schema. No extra timing shifts or filters applied.')
        save(fig,'06_drive_response',pdf)

        fig,axes=plt.subplots(3,1,figsize=(14,10))
        fig.subplots_adjust(left=.09,right=.96,bottom=.09,top=.86,hspace=.43)
        for topic,name,color in zip(POSES[:3],NAMES[:3],COLORS[:3]):
            a=arrays[topic];a=a[overlap(a)]
            axes[0].plot(a[1:,0],1000*np.diff(a[:,1]),color=color,label=name,lw=.7)
            axes[1].plot(a[:,0],1000*(a[:,0]-a[:,1]),color=color,label=name,lw=.7)
        for ax in axes[:2]:ax.set_xlim(lo,hi);ax.legend(loc='upper right')
        axes[0].set_title('Vicon header intervals');decorate(axes[0],'Interval (ms)')
        axes[1].set_title('Bag timestamp minus Vicon header (not end-to-end sensor latency)');decorate(axes[1],'Difference (ms)')
        labels=[];unc=[]
        for item in clock['mappings']:
            labels.append(('FL' if item['wheel_id']=='front_left' else 'FR')+' '+{1:'H1',2:'H2',3:'CENTER'}[item['node_id']])
            unc.append(item['uncertainty_ns']/1e6)
        axes[2].bar(labels,unc,color=['#d48616']*3+['#c53d65']*3)
        axes[2].set_title('Saved encoder clock-mapping uncertainty fields — estimates, not validated error bounds')
        decorate(axes[2],'Reported uncertainty (ms)',False)
        title(fig,'Timing and synchronization','High sample rates do not establish cross-sensor synchronization. The original estimated encoder clock mapping is preserved.')
        save(fig,'07_timing',pdf)
    print(json.dumps({k:report[k] for k in ['common_encoder_window_s','base_motion','encoders','drive']},indent=2),flush=True)
    print(f'Artifacts saved to {OUT}',flush=True)

def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='Recording folder, combined bag directory, or single MCAP/SQLite file')
    parser.add_argument('--output-dir', type=Path, required=True, help='New or empty report directory')
    parser.add_argument('--sidecar-dir', type=Path, help='Recording folder with clock-map.json and export-summary.json when not inferred from source')
    args = parser.parse_args(argv)
    try:
        configure(args.source, args.output_dir, args.sidecar_dir)
        analyze()
    except (ValueError, OSError, KeyError, RuntimeError, ImportError) as error:
        parser.exit(2, f'analyze_combined_caster_bag: {error}\n')


if __name__=='__main__':
    main()
