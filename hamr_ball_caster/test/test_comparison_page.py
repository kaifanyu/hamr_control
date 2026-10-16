"""Check report semantics: incomplete data and failed checks must remain visible."""
import importlib.util
import json
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('comparison_page', Path(__file__).parents[1]/'scripts/build_caster_comparison.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def pair(swivel=.01, ball=.01, swivel_peak=.02, ball_peak=.02, passed=True):
    return {'primary_region': 'motion', 'run_status': {
        m: {'completed': True, 'trajectory_passed': passed} for m in ('swivel', 'ball')},
        'regions': {'motion': {m: {'tracking_error_m': {'rms': rms, 'max_abs': peak}}
            for m, rms, peak in [('swivel', swivel, swivel_peak), ('ball', ball, ball_peak)]}}}


def test_labels_do_not_treat_failure_or_missing_samples_as_benefit():
    assert module.tracking_label(None) == 'Pending'
    assert module.tracking_label(pair(passed=False)) == 'Inspect run checks'
    assert module.tracking_label(pair(ball=None)) == 'Insufficient samples'
    assert module.tracking_label(pair(.01, .009)) == 'Similar RMS tracking'
    assert module.tracking_label(pair(.01, .005)) == 'Ball lower tracking error'
    assert module.tracking_label(pair(.01, .005, .02, .04)) == 'Mixed tracking result'
    assert module.tracking_label(pair(.005, .01)) == 'Swivel lower tracking error'


def test_incomplete_report_retains_case_and_escapes_embedded_data(tmp_path):
    ordinary, ball = tmp_path/'ordinary', tmp_path/'ball'
    ordinary.mkdir()
    ball.mkdir()
    (ordinary/'run_summary.json').write_text('{"passed": true}')
    case = {'name': 'straight_y', 'category': 'trajectory', 'speed_m_s': .25,
            'initial_yaw_deg': 90, 'tracking_comparison_region': 'motion',
            'swivel_directory': str(ordinary), 'ball_directory': str(ball),
            'ball_role': 'nominal', 'points_m': [[0, 0], [0, 1]],
            'comparison_note': '</script><script>unexpected()</script>', 'physical_note': 'estimated'}
    (tmp_path/'comparison_matrix.json').write_text(json.dumps([case]))
    result = module.build_report(tmp_path)
    assert result['recorded_ball_cases'] == 0 and result['analyzed_pairs'] == 0
    assert result['cases'][0]['label'] == 'Pending'
    page = (tmp_path/'index.html').read_text()
    assert case['comparison_note'] not in page
    embedded = page.split('<script id="data" type="application/json">')[1].split('</script>')[0]
    assert json.loads(embedded)['cases'][0]['comparison_note'] == case['comparison_note']
    assert 'Ball capture and paired analysis pending.' in (tmp_path/'PER_TRAJECTORY.md').read_text()
    assert 'Pending' in (tmp_path/'comparison_summary.csv').read_text()


def test_quantities_are_readable():
    assert module.prose('At0.40m/s, a45mm trail') == 'At 0.40 m/s, a 45 mm trail'
