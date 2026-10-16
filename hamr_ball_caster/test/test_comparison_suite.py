import importlib.util
import json
from pathlib import Path

import pytest
import yaml

spec = importlib.util.spec_from_file_location("comparison_suite", Path(__file__).parents[1]/"scripts/run_ball_comparison.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_comparison_preserves_requests_and_labels_non_equivalent_cases(tmp_path):
    source, output = tmp_path/'swivel', tmp_path/'comparison'
    source.mkdir()
    cases = []
    for name, assisted, overrides in [('path', False, []), ('heading', True, []), ('drag', False, ['swivel_damping=0.15'])]:
        run = source/'record'/name
        run.mkdir(parents=True)
        config = {'name': name, 'points_m': [[0,0],[0,2]], 'speed_m_s': .25,
                  'passive_joint_names': ['left_caster_swivel_joint'],
                  'max_wheel_rate_rad_s': 6., 'fixed_turret_yaw_rad': 1.5708}
        if assisted: config['startup_alignment'] = {'enabled': True, 'caster_tolerance_rad': .052}
        (run/'configuration.yaml').write_text(yaml.safe_dump(config))
        (run/'run_summary.json').write_text('{"passed": true}')
        cases.append({'name': name, 'category': 'initial_heading' if assisted else 'trajectory',
                      'initial_yaw_deg': 270 if assisted else 90, 'speed_m_s': .25,
                      'points_m': config['points_m'], 'route_length_m': 2., 'xacro_args': overrides})
    (source/'experiment_matrix.json').write_text(json.dumps(cases))
    result = module.prepare(source, output)
    assert len(result) == 3
    for case in result:
        config = yaml.safe_load(Path(case['ball_configuration']).read_text())
        assert config['points_m'] == [[0,0],[0,2]]
        assert config['speed_m_s'] == .25 and config['max_wheel_rate_rad_s'] == 6.
        assert config['passive_joint_names'] == module.BALL_JOINTS
        assert case['ball_xacro_overrides'] == []
    assert result[1]['tracking_comparison_region'] == 'later_legs'
    assert yaml.safe_load(Path(result[1]['ball_configuration']).read_text())['startup_alignment']['caster_model'] == 'ball'
    assert 'repeated baseline' in result[2]['ball_role']
    assert module.prepare(source, output) == result
    destination = Path(result[0]['ball_configuration'])
    destination.write_text('changed: true')
    with pytest.raises(ValueError, match='Refusing'):
        module.prepare(source, output)
