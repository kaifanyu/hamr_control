from pathlib import Path
import json
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_rocky_return import series,leg_metrics,contact_window,distance_to_path


@pytest.mark.parametrize('yaw,expected',[(0.,100.),(np.pi,0.)])
def test_retracing_does_not_necessarily_mean_chassis_backwards(yaw,expected):
    samples=[{'simulation_time_s':t,'trajectory_time_s':t,'position_m':[-.25*t,0,0],
              'reference_m':[-.25*t,0],'base_rpy_rad':[0,0,yaw],
              'reference_velocity_m_s':[-.25,0],'phase':'motion','route_leg':'return'}
             for t in np.linspace(0,1,21)]
    result=leg_metrics(series(samples),'return')
    assert result['xy_rms_mm']==0
    assert result['body_backward_percent']==pytest.approx(expected)
    assert result['observed_motion_s']==pytest.approx(1.)


def test_stationary_dwell_is_excluded_and_long_data_gaps_are_not_weighted():
    samples=[{'simulation_time_s':t,'trajectory_time_s':t,'position_m':[-.25*t,0,0],
              'reference_m':[-.25*t,0],'base_rpy_rad':[0,0,0],
              'reference_velocity_m_s':[-.25,0],'phase':'motion','route_leg':'return'}
             for t in [0,.1,.2,.8,.9,1.]]
    result=leg_metrics(series(samples),'return')
    assert result['observed_motion_s']==pytest.approx(.4)
    assert result['body_backward_percent']==pytest.approx(100.)


def test_contact_window_uses_source_clock_and_counts_each_link_once_per_snapshot(tmp_path):
    contact={'collision1':'terrain::surface::collision','collision2':'compa::left_caster_wheel_link::collision',
             'normals':[[1.,0.,0.]],'depths_m':[.001]}
    (tmp_path/'contact_samples.json').write_text(json.dumps({'motion_start_s':3.,'samples':[
        {'simulation_time_s':7.,'contacts':[contact]},
        {'simulation_time_s':8.,'contacts':[contact,contact]},
        {'simulation_time_s':8.02,'contacts':[]}]}))
    result=contact_window(tmp_path,5.,.1)
    assert result['snapshots']==2
    assert result['first_trajectory_s']==pytest.approx(5.)
    assert result['maximum_source_gap_s']==pytest.approx(.02)
    assert result['snapshots_with_link_contact']=={'left_caster_wheel_link':1}
    assert result['snapshots_with_assembly_contact']=={'left_caster_assembly':1}
    assert result['snapshots_with_reported_abs_normal_z_below_half']=={'left_caster_wheel_link':1}
    assert result['maximum_reported_depth_mm']==pytest.approx(1.)


def test_path_distance_ignores_schedule_but_respects_segment_endpoints():
    path=[[0.,0.],[1.,0.],[1.,0.],[0.,0.]]
    result=distance_to_path([[.2,0.],[.5,.3],[-1.,1.]],path)
    assert result==pytest.approx([0.,.3,np.sqrt(2.)])


def test_per_leg_timed_error_can_be_large_while_remaining_on_geometric_path():
    samples=[{'simulation_time_s':t,'trajectory_time_s':t,'position_m':[.1*t,0,0],
              'reference_m':[.1*t+.5,0],'base_rpy_rad':[0,0,0],
              'reference_velocity_m_s':[.1,0],'phase':'motion','route_leg':'outbound'}
             for t in np.linspace(0,1,21)]
    result=leg_metrics(series(samples,[[0.,0.],[1.,0.]]),'outbound')
    assert result['xy_rms_mm']==pytest.approx(500.)
    assert result['nearest_path_rms_mm']==pytest.approx(0.)
