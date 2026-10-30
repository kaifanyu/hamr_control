"""Offline ideal CENTER and separate hemisphere reconstructions.

Primary: smooth-sphere CENTER; axial shell rolling in predicted material sectors;
hold each other axial joint. Sensitivity: constant last active joint-rate coasting.
These are declared kinematic closures, not solved roller/rim contact dynamics.
Only each valid segment's initial encoder state initializes the prediction.
"""
from pathlib import Path
from _paths import DATA_ROOT, OUTPUT_ROOT, reconstruction_source, prepared_inputs, relocated
import argparse
import hashlib
import json
import math
import shutil
import numpy as np
from scipy.spatial.transform import Rotation

R = .100
GAP_HALF = .010
OPENING_RADIUS = .025
U_GAP = GAP_HALF / R
U_POLE = math.sqrt(1 - (OPENING_RADIUS / R)**2)
WHEELS = ('front_left', 'front_right')
CHANNELS = ('center', 'h1', 'h2')
# All sector boundaries in a 2*pi interval, including both signs of sin(delta).
ACOS_BOUNDARIES = np.arccos([-U_POLE, -U_GAP, U_GAP, U_POLE])
BOUNDARIES = np.sort(np.r_[ACOS_BOUNDARIES, 2*np.pi-ACOS_BOUNDARIES])

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def sector(delta):
    u = math.cos(delta)
    if abs(u) < U_GAP:
        return 0  # nominal equatorial gap
    if abs(u) > U_POLE:
        return 3  # nominal polar opening
    return 1 if u < 0 else 2

def split_fractions(a, b):
    if abs(b-a) < 1e-14:
        return np.array([0., 1.])
    lo, hi = sorted((a, b))
    edges = []
    for turn in range(math.floor(lo/(2*np.pi))-1, math.floor(hi/(2*np.pi))+2):
        for edge in BOUNDARIES + 2*np.pi*turn:
            f = (edge-a)/(b-a)
            if 1e-12 < f < 1-1e-12:
                edges.append(float(f))
    return np.array([0., *sorted(set(edges)), 1.])

def csc_integral(a, b, dt):
    """Integral csc(delta(t)) dt on one eligible, linear-delta subinterval."""
    if abs(b-a) < 1e-6:
        # Second-order midpoint is stable when subtracting logs would cancel.
        return dt / math.sin((a+b)/2)
    def primitive(x):
        return math.log(abs(math.tan(x/2)))
    return dt * (primitive(b)-primitive(a))/(b-a)

def simulate(time_s, xy, psi, delta0):
    t = np.asarray(time_s, dtype=float)
    p = np.asarray(xy, dtype=float)
    yaw = np.unwrap(np.asarray(psi, dtype=float))
    if len(t) < 2 or np.any(np.diff(t) <= 0):
        raise ValueError('Need strictly increasing input time')
    dt = np.diff(t)
    dp = np.diff(p, axis=0)
    ym = (yaw[:-1]+yaw[1:])/2
    dx = dp[:, 0]*np.cos(ym) + dp[:, 1]*np.sin(ym)
    dy = -dp[:, 0]*np.sin(ym) + dp[:, 1]*np.cos(ym)
    vx, vy = dx/dt, dy/dt
    delta = delta0 - np.r_[0., np.cumsum(dx)]/R
    n = len(t)
    beta = np.zeros((n, 2))
    coast = np.zeros((n, 2))
    last_rates = np.zeros(2)
    mode_fraction = np.zeros((n-1, 4))
    mode_samples = np.array([sector(d) for d in delta], dtype=np.int8)
    nonshell_lateral_distance = 0.
    max_active_rate = np.zeros(2)
    for i in range(n-1):
        beta[i+1] = beta[i]
        coast[i+1] = coast[i]
        a, b = delta[i], delta[i+1]
        fractions = split_fractions(a, b)
        for f0, f1 in zip(fractions[:-1], fractions[1:]):
            da, db = a+(b-a)*f0, a+(b-a)*f1
            subdt = dt[i]*(f1-f0)
            mode = sector((da+db)/2)
            mode_fraction[i, mode] += f1-f0
            active = mode-1 if mode in (1, 2) else None
            if active is not None:
                sign = 1. if active == 0 else -1.
                increment = sign*vy[i]/R*csc_integral(da, db, subdt)
                beta[i+1, active] += increment
                coast[i+1, active] += increment
                last_rates[active] = sign*vy[i]/(R*math.sin(db))
                max_active_rate[active] = max(max_active_rate[active], abs(last_rates[active]),
                                            abs(sign*vy[i]/(R*math.sin(da))))
            else:
                nonshell_lateral_distance += abs(vy[i])*subdt
            for j in (0, 1):
                if j != active:
                    coast[i+1, j] += last_rates[j]*subdt
    primary = np.column_stack((delta-delta0, beta))
    alternative = np.column_stack((delta-delta0, coast))
    return dict(delta_pred_rad=delta, primary_change_rad=primary,
                coast_change_rad=alternative, mode=mode_samples,
                mode_fraction=mode_fraction, forward_velocity_mid_m_s=vx,
                lateral_velocity_mid_m_s=vy,
                primary_rate_mid_rad_s=np.diff(primary, axis=0)/dt[:, None],
                coast_rate_mid_rad_s=np.diff(alternative, axis=0)/dt[:, None],
                max_active_rate_rad_s=max_active_rate,
                nonshell_virtual_lateral_distance_m=nonshell_lateral_distance)

def metric(x, mask=None):
    x = np.asarray(x, dtype=float)
    if mask is not None:
        x = x[np.asarray(mask, dtype=bool)]
    x = x[np.isfinite(x)]
    if not len(x):
        return {'n':0, 'rmse':None, 'mean':None, 'max_abs':None, 'final':None}
    return dict(n=int(len(x)), rmse=float(np.sqrt(np.mean(x*x))),
                mean=float(np.mean(x)), max_abs=float(np.max(abs(x))), final=float(x[-1]))

def orientations(psi, delta, changes):
    frame = Rotation.from_euler('z', psi)*Rotation.from_euler('y', -delta)
    return [(frame*Rotation.from_euler('z', changes[:, 1])).as_quat(),
            (frame*Rotation.from_euler('z', -changes[:, 2])).as_quat()]

def checks():
    t = np.linspace(0, 1, 101)
    zero = np.zeros_like(t)
    p = np.column_stack((.2*t, zero))
    forward = simulate(t, p, zero, .6)
    assert abs(forward['primary_change_rad'][-1, 0]+2) < 1e-12
    assert np.max(abs(forward['primary_change_rad'][:, 1:])) < 1e-12
    for d, active, sign in [(np.pi/4, 2, -1), (3*np.pi/4, 1, 1), (5*np.pi/4, 1, 1)]:
        lateral = simulate(t, p[:, ::-1], zero, d)
        expected = sign*.2/(R*np.sin(d))
        assert abs(lateral['primary_change_rad'][-1, active]-expected) < 1e-11
        assert np.max(abs(lateral['primary_change_rad'][:, 3-active])) < 1e-12
    yawonly = simulate(t, np.zeros((len(t), 2)), t*np.pi, .6)
    assert np.max(abs(yawonly['primary_change_rad'])) == 0
    # One step and many steps must agree through gap and polar boundaries.
    end = np.array([.9, .15])
    coarse = simulate(np.array([0., 1.]), np.array([[0., 0.], end]), np.zeros(2), .6)
    fine = simulate(t, t[:, None]*end, zero, .6)
    error = float(np.max(abs(coarse['primary_change_rad'][-1]-fine['primary_change_rad'][-1])))
    assert error < 1e-10
    reverse_t = np.linspace(0, 2, 201)
    reverse_path = np.vstack((t[:, None]*end, (t[:, None]*end)[-2::-1]))
    retrace = simulate(reverse_t, reverse_path, np.zeros(201), .6)
    assert np.max(abs(retrace['primary_change_rad'][-1])) < 1e-10
    pole = simulate(t, p[:, ::-1], zero, 0.)
    assert np.max(abs(pole['primary_change_rad'])) == 0
    assert abs(pole['nonshell_virtual_lateral_distance_m']-.2) < 1e-12
    return {'straight_CENTER_0p2m_at_0p1m_radius':True, 'H1_H2_signs_lateral':True,
            'stationary_center_yaw_only':True, 'exact_sector_splitting_error_rad':error,
            'primary_reverse_path_returns_initial':True, 'pole_hold_explicit_residual':True}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    root = DATA_ROOT
    parser.add_argument('--input-dir', type=Path, default=prepared_inputs())
    parser.add_argument('--output-dir', type=Path, default=OUTPUT_ROOT/'all_caster_reconstruction')
    parser.add_argument('--case', help='Exact recording folder name; omitted means all.')
    args = parser.parse_args()
    out = args.output_dir
    (out/'histories').mkdir(parents=True, exist_ok=True)
    (out/'source').mkdir(exist_ok=True)
    manifest_path = args.input_dir.parent/'input_manifest.json'
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    if args.case and manifest:
        manifest['cases']=[r for r in manifest.get('cases',[]) if r['run']==args.case]
        manifest['failures']=[r for r in manifest.get('failures',[]) if r.get('run')==args.case]
    if args.case:
        existing={p.stem.split('__segment')[0] for p in (out/'histories').glob('*.npz')}
        if existing - {args.case}:
            raise ValueError('Output contains other cases; choose a fresh --output-dir for a selected-case run.')
    reports = []
    for path in sorted(args.input_dir.glob('*.npz')):
        if args.case and path.stem.split('__segment')[0] != args.case:continue
        src = np.load(path)
        t = src['time_s']
        arrays = {k:src[k].copy() for k in src.files if k in ('time_s','case_elapsed_s','elapsed_s','segment_elapsed_s')}
        # Input filenames use exact run name followed by __segmentNNN.
        case = path.stem.split('__segment')[0]
        item = dict(case=case, segment=path.stem, input_npz=str(path), input_sha256=sha(path),
                    start_s=float(t[0]), end_s=float(t[-1]), duration_s=float(t[-1]-t[0]),
                    history_file='histories/'+path.name, wheels={})
        for wheel in WHEELS:
            pre = wheel+'__'
            observed_delta = src[pre+'delta_enc_rad']
            obs = np.column_stack((observed_delta-observed_delta[0],
                                   src[pre+'beta_h1_change_rad'], src[pre+'beta_h2_change_rad']))
            result = simulate(t, src[pre+'p_xy'], src[pre+'psi'], observed_delta[0])
            primary, coast = result['primary_change_rad'], result['coast_change_rad']
            error, coast_error = primary-obs, coast-obs
            rate_obs = np.diff(obs, axis=0)/np.diff(t)[:, None]
            qpred = orientations(src[pre+'psi'], result['delta_pred_rad'], primary)
            qcoast = orientations(src[pre+'psi'], result['delta_pred_rad'], coast)
            qobs = orientations(src[pre+'psi'], observed_delta, obs)
            pose_errors, coast_pose_errors = [], []
            for q, qc, qo in zip(qpred, qcoast, qobs):
                pose_errors.append(np.degrees((Rotation.from_quat(q)*Rotation.from_quat(qo).inv()).magnitude()))
                coast_pose_errors.append(np.degrees((Rotation.from_quat(qc)*Rotation.from_quat(qo).inv()).magnitude()))
            rate_error = result['primary_rate_mid_rad_s']-rate_obs
            channel_stats = {}
            for j, channel in enumerate(CHANNELS):
                active = np.ones(len(t)-1, bool) if j == 0 else result['mode_fraction'][:, j] > .999999
                channel_stats[channel] = dict(
                    unwrapped_error_deg=metric(np.degrees(error[:, j])),
                    wrapped_error_deg=metric(np.degrees(np.arctan2(np.sin(error[:, j]),np.cos(error[:, j])))),
                    coasting_error_deg=metric(np.degrees(coast_error[:, j])),
                    predicted_total_turns=float(primary[-1,j]/(2*np.pi)),
                    recorded_total_turns=float(obs[-1,j]/(2*np.pi)),
                    coasting_total_turns=float(coast[-1,j]/(2*np.pi)),
                    rate_error_deg_s=metric(np.degrees(rate_error[:, j])),
                    predicted_active_interval_rate_error_deg_s=metric(np.degrees(rate_error[:, j]), active),
                    orientation_error_deg=metric(pose_errors[j-1]) if j else None)
            dt = np.diff(t)
            time_by_mode = (result['mode_fraction']*dt[:,None]).sum(axis=0)
            item['wheels'][wheel] = dict(channels=channel_stats,
                predicted_sector_duration_s=dict(zip(('gap','h1','h2','polar'), map(float,time_by_mode))),
                nonshell_virtual_lateral_distance_m=float(result['nonshell_virtual_lateral_distance_m']),
                max_active_spin_rate_rad_s=result['max_active_rate_rad_s'].tolist())
            values = dict(result)
            values.update(observed_change_rad=obs, primary_error_rad=error, coast_error_rad=coast_error,
                          observed_rate_mid_rad_s=rate_obs, observed_delta_rad=observed_delta,
                          psi=src[pre+'psi'], p_xy=src[pre+'p_xy'],
                          h1_quaternion_pred_xyzw=qpred[0], h2_quaternion_pred_xyzw=qpred[1],
                          h1_quaternion_observed_xyzw=qobs[0], h2_quaternion_observed_xyzw=qobs[1],
                          h1_quaternion_coast_xyzw=qcoast[0], h2_quaternion_coast_xyzw=qcoast[1],
                          hemisphere_orientation_error_deg=np.column_stack(pose_errors),
                          hemisphere_coast_orientation_error_deg=np.column_stack(coast_pose_errors))
            arrays.update({pre+k:v for k,v in values.items()})
        np.savez_compressed(out/'histories'/path.name, **arrays)
        reports.append(item)
        print(json.dumps({'segment':path.stem,'duration_s':item['duration_s']}), flush=True)
    if not reports:raise ValueError('No prepared input matched the requested case(s).')
    summary = dict(model='Geometry-gated ideal shell rolling, axial spin held outside active shell sector',
        radius_m=R, equatorial_gap_m=2*GAP_HALF, polar_opening_diameter_m=2*OPENING_RADIUS,
        sector_codes={'0':'nominal equatorial gap','1':'nominal H1 shell','2':'nominal H2 shell','3':'nominal polar opening'},
        formula={'center':'delta_dot=-v_x_fork/R','H1_active':'beta1_dot=v_y_fork/(R*sin(delta))',
                 'H2_active':'beta2_dot=-v_y_fork/(R*sin(delta))','inactive_or_nonshell':'beta_dot=0 primary; last active joint rate retained in coasting sensitivity'},
        assumptions=[
            'CENTER uses ideal 100 mm sphere throughout; physical rim/roller support height is not solved.',
            'Only predicted CENTER chooses nominal material sectors; observed future encoder values never feed prediction.',
            'H1/H2 phases are changes from their segment initial reading; uncalibrated absolute longitude is not needed for this relative comparison.',
            'Holding an axial spin angle does not freeze full hemisphere orientation: it still follows CENTER and base yaw.',
            'The coasting variant holds last active axial joint rate and starts inactive rates at zero; it is an assumption sensitivity, not a confidence interval or full free-body dynamics.',
            'Gap and polar sectors are geometric ideal-bottom classifications, not measured support transitions or a solved roller model.',
            'Non-shell virtual lateral distance integrates abs(v_y) during non-shell sectors; it is not physical measured slip.',
            'Every valid segment is initialized independently; no state is carried across excluded data gaps.',
            'Input positions, fork-frame/yaw convention, encoder sign/scale and time mapping retain prior assumptions; no parameter fitting.',
            'Quaternion comparisons are reconstructed from the same encoder conventions and yaw, not independent shell Vicon ground truth.'
        ], analytical_checks=checks(), source_sha256=sha(__file__), input_manifest=manifest,
        segments=reports, case_count=len(set(x['case'] for x in reports)))
    (out/'results.json').write_text(json.dumps(summary,indent=2))
    source_destination = out/'source/reconstruct_all.py'
    if Path(__file__).resolve() != source_destination.resolve():
        shutil.copyfile(__file__, source_destination)
    if manifest_path.exists():
        if manifest_path.resolve() != (out/'input_manifest.json').resolve():
            shutil.copyfile(manifest_path, out/'input_manifest.json')
    print(f'Done: {len(reports)} valid segments, {summary["case_count"]} cases',flush=True)

if __name__ == '__main__':
    main()
