"""Schematic flat-ground replay of recorded paths and two orientation histories.

Both panes use identical recorded Vicon XY paths. The left shows initial-aligned
encoder orientations; the right shows saved independent kinematic predictions.
No dynamics, trajectory prediction, fitting, or encoder feedback occurs here.
"""
from pathlib import Path
from _paths import DATA_ROOT, OUTPUT_ROOT, reconstruction_source, prepared_inputs, relocated
import argparse
import hashlib
import json
import math
import subprocess
import numpy as np
from scipy.spatial.transform import Rotation
from PIL import Image, ImageDraw, ImageFont

ROOT = DATA_ROOT
INPUT = reconstruction_source() / 'histories'
OUT = OUTPUT_ROOT / 'caster_trajectory_replay'
W, H, FPS = 1600, 900, 15
SELECTED = 'encoder_20261030T005046Z_7a2266b7'
WHEELS = ('front_left', 'front_right')
BG = '#f3f6f8'
INK = '#213a4d'
MUTED = '#60788a'
ORANGE = '#ca7638'
BLUE = '#257bb0'
PATH_COLORS = {'front_left': '#238b85', 'front_right': '#8970ad', 'base': '#8c9aa7'}
RADIUS = .1
from matplotlib import get_data_path
FONTS = Path(get_data_path())/'fonts/ttf'
SIZES = (12, 13, 14, 15, 16, 18, 20, 22, 24, 28, 32)
FONT = {s: ImageFont.truetype(str(FONTS / 'DejaVuSans.ttf'), s) for s in SIZES}
BOLD = {s: ImageFont.truetype(str(FONTS / 'DejaVuSans-Bold.ttf'), s) for s in SIZES}
VIEW = np.array([.5, -.85, .9]); VIEW /= np.linalg.norm(VIEW)
RIGHT = np.cross([0., 0., 1.], VIEW); RIGHT /= np.linalg.norm(RIGHT)
UP = np.cross(VIEW, RIGHT)
CAM = np.column_stack((RIGHT, -UP, VIEW))
PANE_X = (22, 818)
PANE_WIDTH = 760
WORLD_BOX = (18, 138, 742, 488)
MODE_NAMES = {0: 'Gap sector', 1: 'H1 shell sector', 2: 'H2 shell sector', 3: 'Polar sector'}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def text(draw, xy, message, size=16, fill=INK, bold=False):
    draw.text(xy, message, font=(BOLD if bold else FONT)[size], fill=fill)


def line(draw, xy, fill, width=1):
    if len(xy) >= 2:
        draw.line([tuple(point) for point in xy], fill=fill, width=width)


def sector(delta):
    u = abs(math.cos(float(delta)))
    if u < .1:
        return 0
    if u > math.sqrt(1 - .25**2):
        return 3
    return 1 if math.cos(float(delta)) < 0 else 2


def meshes():
    """Axisymmetric 100 mm shell faces with 20 mm gap and 50 mm polar opening."""
    result = []
    phi = np.linspace(0, 2*np.pi, 25)
    for side in (1, -1):
        lat = side * np.arcsin(np.linspace(.1, math.sqrt(1-.25**2), 8))
        faces, norms, colors = [], [], []
        base = np.array([221., 141., 69.]) if side == 1 else np.array([65., 144., 192.])
        for i in range(len(lat)-1):
            for j in range(len(phi)-1):
                vertices = []
                for la, ph in ((lat[i], phi[j]), (lat[i], phi[j+1]),
                               (lat[i+1], phi[j+1]), (lat[i+1], phi[j])):
                    vertices.append([math.cos(la)*math.cos(ph), math.cos(la)*math.sin(ph), math.sin(la)])
                vertices = np.array(vertices)
                normal = vertices.mean(axis=0); normal /= np.linalg.norm(normal)
                # A bright meridian and dark sectors make axial phase visible.
                color = np.array([241., 245., 247.]) if j == 0 else base * (.62 if j % 6 == 3 else 1.)
                faces.append(vertices); norms.append(normal); colors.append(color)
        result.append((np.array(faces), np.array(norms), np.array(colors)))
    return result


MESHES = meshes()
PHI = np.linspace(0, 2*np.pi, 73)
CIRCLE = np.column_stack((np.cos(PHI), np.sin(PHI), np.zeros(len(PHI))))


def sphere(draw, center_screen, radius_px, q1, q2, psi, label=None):
    """Render independently rotating shell halves in a common +H1-axis frame."""
    center = np.asarray(center_screen)
    # Axle behind the shells, fixed in the fork's horizontal direction.
    axle = np.array([[-math.sin(psi), math.cos(psi), 0.], [math.sin(psi), -math.cos(psi), 0.]]) * 1.17
    line(draw, (axle @ CAM)[:, :2]*radius_px + center, '#778894', max(1, round(radius_px/28)))
    polygons = []
    light = np.array([-.35, -.5, .8]); light /= np.linalg.norm(light)
    for q, (faces, normals, colors) in zip((q1, q2), MESHES):
        world = faces @ q.T
        normals_world = normals @ q.T
        projected = world @ CAM
        visible = normals_world @ VIEW > -.015
        lightness = .61 + .39*np.maximum(0., normals_world @ light)
        shades = np.clip(colors * lightness[:, None], 0, 255).astype(np.uint8)
        for index in np.flatnonzero(visible):
            points = projected[index, :, :2]*radius_px + center
            polygons.append((float(projected[index, :, 2].mean()), points,
                             tuple(map(int, shades[index]))))
    for _, points, color in sorted(polygons, key=lambda item: item[0]):
        draw.polygon([tuple(point) for point in points], fill=color)
    # Circular shell edges remain visible at the equator gap and polar opening.
    for sign, q in ((1, q1), (-1, q2)):
        for z in (sign*.1, sign*math.sqrt(1-.25**2)):
            rr = math.sqrt(1-z*z)
            ring = np.column_stack((rr*np.cos(PHI), rr*np.sin(PHI), np.full(len(PHI), z))) @ q.T
            xy = (ring @ CAM)[:, :2]*radius_px + center
            # Only front-facing arc, so rims do not show through the shell.
            visible = ring @ VIEW > 0
            for a, b in zip(range(len(ring)-1), range(1, len(ring))):
                if visible[a] and visible[b]:
                    line(draw, xy[[a, b]], '#4b6171', max(1, round(radius_px/80)))
    # Positive split axis marker and ideal sphere-bottom point (not measured contact).
    pole = q1[:, 2] @ CAM
    p = pole[:2] * radius_px * 1.04 + center
    size = max(2, round(radius_px/24))
    draw.ellipse((p[0]-size, p[1]-size, p[0]+size, p[1]+size), fill='#534633')
    bottom = np.array([0., 0., -1.]) @ CAM
    p = bottom[:2]*radius_px + center
    draw.ellipse((p[0]-size, p[1]-size, p[0]+size, p[1]+size), fill='#b7424e')
    if label:
        text(draw, (center[0]-radius_px, center[1]+radius_px+7), label, 13, MUTED)


def load_case(case):
    segments = []
    for path in sorted(INPUT.glob(case+'__*.npz')):
        with np.load(path) as source:
            a = {key: source[key].copy() for key in source.files}
        for wheel in WHEELS:
            for hemi in ('h1', 'h2'):
                for role in ('observed', 'pred'):
                    key = wheel + '__' + hemi + '_quaternion_' + role + '_xyzw'
                    a[wheel+'__'+hemi+'_'+role+'_matrix'] = Rotation.from_quat(a[key]).as_matrix()
        segments.append({'path': path, 'a': a, 't': a['case_elapsed_s']})
    segments.sort(key=lambda s: float(s['t'][0]))
    pose_path = ROOT/'work/calibration_audit'/(case+'.npz')
    with np.load(pose_path) as source:
        base = source['_HAMR_base_pose'].copy()
    for s in segments:
        s['a']['base_xy'] = np.column_stack([np.interp(s['a']['time_s'], base[:, 0], base[:, j]) for j in (1, 2)])
    return segments, pose_path


def scene_geometry(segments):
    points = np.vstack([s['a'][wheel+'__p_xy'] for s in segments for wheel in WHEELS] +
                       [s['a']['base_xy'] for s in segments])
    origin = segments[0]['a']['base_xy'][0]
    local = points-origin
    lo, hi = local.min(axis=0)-.35, local.max(axis=0)+.35
    corner_xy = np.array([[lo[0], lo[1]], [lo[0], hi[1]], [hi[0], lo[1]], [hi[0], hi[1]]])
    vertices = np.vstack((np.column_stack((corner_xy, np.zeros(4))), np.column_stack((corner_xy, np.full(4, .35)))))
    screen = vertices @ CAM
    plo, phi = screen[:, :2].min(axis=0), screen[:, :2].max(axis=0)
    x0, y0, x1, y1 = WORLD_BOX
    scale = min((x1-x0)/(phi[0]-plo[0]), (y1-y0)/(phi[1]-plo[1]))
    offset = np.array([(x0+x1)/2, (y0+y1)/2]) - (plo+phi)/2*scale
    return {'origin': origin, 'lo': lo, 'hi': hi, 'scale': scale, 'offset': offset}


def world_project(points, geometry, pane):
    p = np.asarray(points, dtype=float).copy()
    p[:, :2] -= geometry['origin']
    xy = (p @ CAM)[:, :2]*geometry['scale']+geometry['offset']
    xy[:, 0] += PANE_X[pane]
    return xy


def path_screen(path, z, geometry, pane):
    return world_project(np.column_stack((path, np.full(len(path), z))), geometry, pane)


def background(case, segments, geometry):
    image = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(image)
    stamp = case.split('_')[1].split('T')[-1].rstrip('Z')
    text(d, (24, 14), 'Flat-ground caster trajectory replay', 28, bold=True)
    text(d, (24, 51), 'Both panes follow the SAME recorded Vicon paths; only caster rotation differs.', 18, MUTED)
    text(d, (1135, 18), f'{stamp[:2]}:{stamp[2:4]}:{stamp[4:]} UTC  |  {case.split("_")[2]}', 18, bold=True)
    for pane, title, col in ((0, 'RECORDED ORIENTATION', ORANGE), (1, 'KINEMATIC PREDICTION', BLUE)):
        x = PANE_X[pane]
        d.rounded_rectangle((x-4, 91, x+PANE_WIDTH+4, 821), radius=10, fill='white', outline='#d5dfe6', width=1)
        d.rectangle((x+13, 106, x+19, 128), fill=col)
        text(d, (x+30, 104), title, 20, bold=True)
        text(d, (x+438, 109), 'Schematic flat-ground scene', 14, MUTED)
        lo, hi = geometry['lo'], geometry['hi']
        max_span = float(max(hi-lo))
        step = .25 if max_span < 4 else .5 if max_span < 8 else 1.
        for axis in (0, 1):
            for value in np.arange(math.floor(lo[axis]/step)*step, hi[axis]+step, step):
                if not lo[axis] <= value <= hi[axis]:
                    continue
                points = np.array([[lo[0], lo[1], 0.], [hi[0], hi[1], 0.]])
                points[:, axis] = value
                points[:, :2] += geometry['origin']
                line(d, world_project(points, geometry, pane), '#e5ebef', 1)
        for s in segments:
            for wheel in WHEELS:
                path = s['a'][wheel+'__p_xy']; indices = np.unique(np.linspace(0, len(path)-1, min(600, len(path))).astype(int))
                line(d, path_screen(path[indices], 0., geometry, pane), '#d5dfe5', 1)
        # An explicit local world-axis reference with scale.
        anchor = np.array([lo[0]+.12, lo[1]+.12])+geometry['origin']
        axes = np.array([[*anchor, 0.], [anchor[0]+step, anchor[1], 0.], [anchor[0], anchor[1]+step, 0.]])
        pp = world_project(axes, geometry, pane)
        line(d, pp[[0, 1]], '#9e5b52', 2); line(d, pp[[0, 2]], '#5b8e69', 2)
        text(d, tuple(pp[1]+[4, -8]), 'X', 12, '#9e5b52'); text(d, tuple(pp[2]+[4, -8]), 'Y', 12, '#5b8e69')
        text(d, (x+18, 490), f'Grid {step:g} m  ·  radius 0.10 m at scene scale', 13, MUTED)
        d.line((x+15, 520, x+PANE_WIDTH-15, 520), fill='#dbe4e9', width=1)
        for column, wheel in enumerate(WHEELS):
            cx = x+190+column*380
            text(d, (cx-160, 538), wheel.replace('_', ' ').title()+' · close-up', 18, bold=True)
            # Fixed local horizontal floor patch under enlarged spheres.
            floor = np.column_stack((CIRCLE[:, :2]*1.25, np.full(len(CIRCLE), -1.)))
            patch = (floor @ CAM)[:, :2]*77+np.array([cx, 660])
            d.polygon([tuple(point) for point in patch], fill='#eef2f4')
    text(d, (27, 834), 'H1 orange  ·  H2 blue  ·  white stripe = material phase  ·  red dot = ideal sphere-bottom point', 16)
    text(d, (27, 859), 'Gap 20 mm; polar opening 50 mm. Predicted shell spins held off their nominal shell sector; no roller/rim dynamics.', 14, MUTED)
    return image


def state_at(segments, time):
    for index, s in enumerate(segments):
        if s['t'][0]-1e-8 <= time <= s['t'][-1]+1e-8:
            i = min(int(np.searchsorted(s['t'], time)), len(s['t'])-1)
            return index, s, i
    return None


def render(case, segments, geometry, base, time, duration):
    image = base.copy(); d = ImageDraw.Draw(image)
    text(d, (1300, 51), f'{time:05.2f} / {duration:05.2f} s', 18, bold=True)
    state = state_at(segments, time)
    for pane in (0, 1):
        px = PANE_X[pane]
        for s in segments:
            j = int(np.searchsorted(s['t'], time, side='right'))
            if j < 2:
                continue
            for wheel in WHEELS:
                path = s['a'][wheel+'__p_xy'][:j]
                indices = np.unique(np.linspace(0, len(path)-1, min(600, len(path))).astype(int))
                line(d, path_screen(path[indices], .002, geometry, pane), PATH_COLORS[wheel], 2)
        if state is None:
            d.rounded_rectangle((px+90, 260, px+670, 370), radius=10, fill='#f5edf0', outline='#d6b7c3')
            text(d, (px+125, 281), 'Excluded data gap', 24, '#8b4762', True)
            text(d, (px+125, 321), 'No state interpolation across this interval.', 16, MUTED)
            continue
        segment_index, s, i = state
        a = s['a']; psi = float(a['front_left__psi'][i]); role = 'observed' if pane == 0 else 'pred'
        base_xy = a['base_xy'][i]
        top = np.array([base_xy[0], base_xy[1], .26])
        # Recorded base marker as a chassis-anchor proxy; no body mesh implied.
        support = np.array([top, [*a['front_left__p_xy'][i], .19], [*a['front_right__p_xy'][i], .19], top])
        line(d, world_project(support, geometry, pane), '#738999', 2)
        arrow = np.array([top, top+np.array([math.cos(psi)*.22, math.sin(psi)*.22, 0.])])
        arrow_xy = world_project(arrow, geometry, pane)
        line(d, arrow_xy, '#536a7c', 3)
        p = arrow_xy[0]; d.ellipse((p[0]-4, p[1]-4, p[0]+4, p[1]+4), fill='#536a7c')
        # Draw far caster first for deterministic painter ordering.
        order = sorted(WHEELS, key=lambda w: float(np.array([*a[w+'__p_xy'][i], .1]) @ VIEW))
        for wheel in order:
            k = wheel+'__'; position = np.array([*a[k+'p_xy'][i], .1])
            center = world_project(np.atleast_2d(position), geometry, pane)[0]
            q1, q2 = a[k+'h1_'+role+'_matrix'][i], a[k+'h2_'+role+'_matrix'][i]
            shadow = np.column_stack((CIRCLE[:, :2]*RADIUS, np.zeros(len(CIRCLE))))+np.array([*position[:2], 0.])
            d.polygon([tuple(point) for point in world_project(shadow, geometry, pane)], fill='#cfd9df')
            sphere(d, center, RADIUS*geometry['scale'], q1, q2, psi)
            label = 'FL' if wheel == 'front_left' else 'FR'
            text(d, (center[0]+RADIUS*geometry['scale']+3, center[1]-8), label, 12, PATH_COLORS[wheel], True)
        for column, wheel in enumerate(WHEELS):
            k = wheel+'__'; cx = px+190+column*380
            q1, q2 = a[k+'h1_'+role+'_matrix'][i], a[k+'h2_'+role+'_matrix'][i]
            sphere(d, (cx, 660), 77, q1, q2, psi)
            changes = a[k+('observed_change_rad' if pane == 0 else 'primary_change_rad')][i]/(2*np.pi)
            delta = a[k+('observed_delta_rad' if pane == 0 else 'delta_pred_rad')][i]
            mode = sector(delta)
            text(d, (cx-160, 754), MODE_NAMES[mode]+' (nominal)', 14, MUTED)
            text(d, (cx-160, 780), f'C {changes[0]:+6.2f}   H1 {changes[1]:+6.2f}   H2 {changes[2]:+6.2f} turns', 14)
        if segment_index > 0 and time-s['t'][0] < 2.:
            d.rounded_rectangle((px+100, 450, px+660, 482), radius=6, fill='#fff0d9')
            text(d, (px+115, 456), f'Segment {segment_index+1}: model reinitialized after gap', 15, '#8b5d24', True)
    d.rectangle((24, 887, W-24, 892), fill='#d9e3e9')
    d.rectangle((24, 887, 24+(W-48)*time/max(duration, 1e-9), 892), fill=BLUE)
    return image


def run_case(case, poster_only=False):
    segments, pose_path = load_case(case)
    if not segments:
        raise ValueError('No reconstructed input: '+case)
    duration = float(segments[-1]['t'][-1])
    geometry = scene_geometry(segments)
    base = background(case, segments, geometry)
    poster_time = min(18., duration*.45)
    if state_at(segments, poster_time) is None:
        poster_time = float(segments[0]['t'][len(segments[0]['t'])//2])
    poster = OUT/'posters'/(case+'.png')
    video = OUT/'videos'/(case+'.mp4')
    render(case, segments, geometry, base, poster_time, duration).save(poster)
    print('POSTER '+str(poster), flush=True)
    frames = int(math.ceil(duration*FPS))+1
    record = {'case': case, 'video': 'videos/'+video.name, 'poster': 'posters/'+poster.name,
              'source_duration_s': duration, 'duration': frames/FPS, 'fps': FPS, 'width': W, 'height': H,
              'frames': frames, 'segments': len(segments),
              'gaps': [{'start_s': float(a['t'][-1]), 'end_s': float(b['t'][0])} for a,b in zip(segments[:-1],segments[1:])],
              'source_history_sha256': {s['path'].name: sha(s['path']) for s in segments},
              'base_pose_source_sha256': sha(pose_path), 'scene_origin_world_xy_m': geometry['origin'].tolist(),
              'scene_scale_pixels_per_m': geometry['scale'],
              'path_definition': 'Identical recorded Vicon caster paths and base marker path in both panes; no independently predicted robot trajectory.',
              'flat_ground': 'Caster centers at z=0.1m; base marker proxy at z=0.26m. Rendering is schematic.',
              'orientation_definition': 'Recorded initial-aligned encoder rotation versus primary held-axial-phase prediction. H1/H2 material longitudes start at arbitrary common zero.',
              'contact_definition': 'Red point is ideal sphere bottom, not actual measured contact; shell/gap/polar badges are nominal geometry from each pane state.'}
    if poster_only:
        return record
    command = ['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-vcodec','rawvideo',
               '-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-','-an','-c:v','libx264',
               '-preset','veryfast','-crf','21','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(video)]
    proc = subprocess.Popen(command, stdin=subprocess.PIPE)
    try:
        for frame in range(frames):
            time = min(frame/FPS, duration)
            proc.stdin.write(render(case, segments, geometry, base, time, duration).tobytes())
            if frame % 150 == 0:
                print(f'{case}: {frame}/{frames} frames', flush=True)
    finally:
        proc.stdin.close()
    if proc.wait():
        raise RuntimeError('ffmpeg encoding failed: '+case)
    record['video_bytes'] = video.stat().st_size
    (OUT/'videos'/(case+'.json')).write_text(json.dumps(record, indent=2))
    print('VIDEO '+str(video), flush=True)
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', default=SELECTED)
    parser.add_argument('--all', action='store_true')
    parser.add_argument('--poster-only', action='store_true')
    parser.add_argument('--skip-existing', action='store_true')
    args = parser.parse_args()
    for name in ('videos', 'posters'):
        (OUT/name).mkdir(parents=True, exist_ok=True)
    cases = sorted({path.stem.split('__')[0] for path in INPUT.glob('*.npz')}) if args.all else [args.case]
    if SELECTED in cases:
        cases.remove(SELECTED); cases.insert(0, SELECTED)
    records = []
    for case in cases:
        previous = OUT/'videos'/(case+'.json')
        if args.skip_existing and previous.exists() and (OUT/'videos'/(case+'.mp4')).exists():
            records.append(json.loads(previous.read_text()))
            continue
        records.append(run_case(case, args.poster_only))
        (OUT/'videos_manifest.json').write_text(json.dumps({'videos': records, 'renderer_sha256': sha(__file__)}, indent=2))


if __name__ == '__main__':
    main()
