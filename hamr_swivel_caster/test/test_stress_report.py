"""Reports preserve evidence and distinguish missing runs from failed runs."""
import json
from pathlib import Path
import re
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from build_stress_report import build_report, collect_suite, thin_samples


def sample(t, phase='motion'):
    return {'elapsed_s': t, 'position_m': [0., t*.1, 0.],
            'reference_m': [0., t*.1], 'base_rpy_rad': [0., 0., 1.57], 'phase': phase}


def write_run(suite, name, passed=True):
    run = suite/'record'/name
    run.mkdir(parents=True)
    trajectory = {'samples': [sample(0, 'startup_heading_recovery'), sample(1), sample(2)],
                  'completed': True, 'summary': {'passed': passed, 'motion_rmse_m': .012,
                  'motion_max_error_m': .025, 'final_position_error_m': .002,
                  'failures': [] if passed else ['Example recorded acceptance failure']},
                  'configuration': {'points_m': [[0., 0.], [0., .2]]}}
    (run/'trajectory.json').write_text(json.dumps(trajectory))
    (run/'run_summary.json').write_text(json.dumps({'passed': passed}))
    (run/'overview_annotated.mp4').write_bytes(b'placeholder')
    (run/'caster_closeup.mp4').write_bytes(b'placeholder')
    return run


def test_matrix_categories_missing_runs_and_partial_writes_remain_distinct(tmp_path):
    (tmp_path/'experiment_matrix.json').write_text(json.dumps([
        {'name': 'path_a', 'category': 'trajectory', 'points_m': [[0, 0], [0, .2]]},
        {'name': 'heading_180', 'category': 'initial_heading', 'points_m': [[0, 0], [0, .2]]},
        {'name': 'friction', 'category': 'sensitivity'},
        {'name': 'not_started', 'category': 'trajectory'},
    ]))
    write_run(tmp_path, 'path_a')
    write_run(tmp_path, 'heading_180', passed=False)
    partial = tmp_path/'record/friction'; partial.mkdir()
    (partial/'trajectory.json').write_text('{partially written')
    result = collect_suite(tmp_path)
    assert result['path_shapes_with_trajectory'] == 1
    assert result['missing_cases'] == ['not_started']
    assert result['counts_by_category']['trajectory']['planned'] == 2
    assert result['counts_by_category']['initial_heading']['with_trajectory'] == 1
    assert [c['status'] for c in result['cases']] == ['passed', 'failed', 'partial']
    assert result['cases'][1]['failures'] == ['Example recorded acceptance failure']
    assert result['cases'][2]['failures'] == []
    assert result['cases'][2]['warnings']


def test_report_artifacts_and_video_links_are_portable(tmp_path):
    name = 'case with space'
    run = write_run(tmp_path, name)
    (tmp_path/'experiment_matrix.json').write_text(json.dumps([
        {'name': name, 'category': 'trajectory', 'points_m': [[0, 0], [0, .2]]}
    ]))
    before = {p.name: p.read_bytes() for p in run.iterdir()}
    findings = tmp_path/'FINDINGS.md'
    findings.write_text('Existing scientific conclusions must remain intact.\n')
    report = build_report(tmp_path)
    files = report['cases'][0]['files']
    assert files['overview_annotated.mp4'] == 'record/case%20with%20space/overview_annotated.mp4'
    assert not any(str(tmp_path) in url for url in files.values())
    assert before == {p.name: p.read_bytes() for p in run.iterdir()}
    assert findings.read_text() == 'Existing scientific conclusions must remain intact.\n'
    assert report['suite_files'] == {'FINDINGS.md': 'FINDINGS.md'}
    for name in ('index.html', 'suite_summary.json', 'suite_summary.csv',
                 'suite_comparison.png', 'trajectory_montage.png'):
        assert (tmp_path/name).stat().st_size > 0
    html = (tmp_path/'index.html').read_text()
    embedded = re.search(r'<script id="report-data" type="application/json">(.*?)</script>', html, re.S).group(1)
    assert len(json.loads(embedded)['cases'][0]['samples']) == 3
    assert 'src="http' not in html and '<script src=' not in html
    assert 'id="suiteEvidence"' in html


def test_suite_conclusions_and_comparisons_are_linked_only_when_present(tmp_path):
    assert collect_suite(tmp_path)['suite_files'] == {}
    names = ('FINDINGS.md', 'PHYSICS_ASSUMPTIONS.md',
             'startup_270_comparison.mp4', 'horizontal_shuttle_comparison.mp4',
             'tiny_square_slow_comparison.mp4')
    for name in names:
        (tmp_path/name).write_bytes(b'existing artifact')
    result = collect_suite(tmp_path)
    assert result['suite_files'] == {name: name for name in names}
    assert all(not Path(link).is_absolute() for link in result['suite_files'].values())


def test_downsample_preserves_short_startup_phase_boundaries():
    samples = [sample(i, 'startup_hold' if i < 100 else 'motion') for i in range(1000)]
    samples[3]['phase'] = 'startup_caster_lead_in'
    rows = thin_samples(samples, maximum=10)
    assert rows[0][0] == 0 and rows[-1][0] == 999
    times = {row[0] for row in rows}
    assert {2, 3, 4, 99, 100} <= times
    assert any(row[-1] == 'startup_caster_lead_in' for row in rows)
