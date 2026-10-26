"""Check video timing and measured CAD poses without rendering or hardware."""
import math
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))
from render_caster_encoder_video import frame_times, link_transforms, sample_frames, ground_start_mapping
from caster_encoder_plots import collect_readings, curve_points, EncoderPlots
from test_caster_encoder_replay import envelope, SECOND_BOOT


@pytest.mark.parametrize('swap', [False, True])
@pytest.mark.parametrize('sign', [-1, 1])
def test_ground_start_puts_h1_forward_and_preserves_signed_recorded_changes(swap, sign):
    from caster_replay_model import DEFAULT_MAPPING
    robot = ET.parse(SCRIPTS.parents[1] / 'hamr_ball_caster/urdf/ball_caster.urdf').getroot()
    mapping = dict(DEFAULT_MAPPING, h1_sign=sign, h2_sign=sign, center_sign=sign,
                   h1_zero_rad=77., center_zero_rad=-33., mapping_verified=True)
    if swap:
        mapping['h1_joint'], mapping['h2_joint'] = mapping['h2_joint'], mapping['h1_joint']
    events = [(100, envelope(1, angle=12.3)), (101, envelope(2, angle=-9.4)), (102, envelope(3, angle=6.2)),
              (200, envelope(1, seq=1, angle=13.)), (200, envelope(2, seq=1, angle=-10.)),
              (200, envelope(3, seq=1, angle=.01))]
    original = copy.deepcopy(events)
    effective, reference = ground_start_mapping(events, robot, mapping)
    assert events == original
    assert reference['reference_stamp_ns'] == 102
    assert reference['reference_encoder_rad'] == {'h1': 12.3, 'h2': -9.4, 'center': 6.2}
    assert reference['assumed'] is True and effective['mapping_verified'] is False
    frames = sample_frames(events, [101, 102, 200], effective)
    assert not frames[0]['status']['complete_pose_available']
    initial = frames[1]['joints']
    assert initial[mapping['h1_joint']] == initial[mapping['h2_joint']] == 0.
    transforms = link_transforms(robot, initial)
    for role, desired in (('h1', [1, 0, 0]), ('h2', [-1, 0, 0])):
        joint = robot.find(f"./joint[@name='{mapping[role + '_joint']}']")
        axis = transforms[joint.find('child').get('link')][:3, :3] @ np.fromstring(joint.find('axis').get('xyz'), sep=' ')
        np.testing.assert_allclose(axis, desired, atol=1e-12)
    moved = frames[2]['joints']
    assert moved[mapping['h1_joint']] == pytest.approx(sign * .7)
    assert moved[mapping['h2_joint']] == pytest.approx(sign * -.6)
    assert moved['caster_carrier_joint'] - initial['caster_carrier_joint'] == pytest.approx(sign * (.01 - 6.2))


def test_ground_reference_waits_for_valid_fresh_pose_and_does_not_hide_reboots():
    robot = ET.parse(SCRIPTS.parents[1] / 'hamr_ball_caster/urdf/ball_caster.urdf').getroot()
    events = [(100, envelope(1, angle=12.3)), (101, envelope(2, angle=-9.4)),
              (102, envelope(3, angle=1., valid=False)), (103, envelope(3, seq=1, angle=2.))]
    with pytest.raises(ValueError, match='complete fresh'):
        ground_start_mapping(events[:3], robot)
    effective, reference = ground_start_mapping(events, robot)
    assert reference['reference_stamp_ns'] == 103
    rebooted = events + [(200, envelope(1, angle=99., boot=SECOND_BOOT))]
    frame = sample_frames(rebooted, [200], effective)[0]
    assert frame['status']['nodes'][0]['boot_changes'] == 1
    assert frame['joints'][effective['h1_joint']] == pytest.approx(99. - 12.3)
    # No fresh common reference if the H1/H2 samples have already gone stale.
    stale_events = events[:2] + [(2_000_000_000, envelope(3, angle=2.))]
    with pytest.raises(ValueError, match='complete fresh'):
        ground_start_mapping(stale_events, robot)


@pytest.mark.parametrize('rate', [.5, 1., 2.])
def test_video_clock_preserves_rate_and_integer_epoch_precision(rate):
    start = 1792030863809132538
    span = 30_789_866_815
    stamps = frame_times(start, start+span, 30, rate)
    assert stamps[0] == start
    assert stamps[30] - start == int(rate * 1_000_000_000)
    assert stamps[-1] < start+span
    assert 0 <= len(stamps)/30 - span/1e9/rate < 1/30


@pytest.mark.parametrize('fps,rate', [(0,1), (121,1), (30,0), (30,-1), (30,float('nan'))])
def test_invalid_output_clock_is_rejected(fps,rate):
    with pytest.raises(ValueError):
        frame_times(0,10_000_000_000,fps,rate)


def test_frame_uses_only_past_valid_packets_and_marks_missing_stale_data():
    start=1_000_000_000
    events=[(start+10_000_000,envelope(node,angle=node*.1)) for node in (1,2,3)]
    events += [(start+110_000_000,envelope(1,seq=1,angle=2.)),
               (start+150_000_000,envelope(1,seq=2,angle=99,valid=False))]
    frames=sample_frames(events,[start,start+100_000_000,start+120_000_000,start+200_000_000,start+2_000_000_000])
    assert not frames[0]['joints']
    assert frames[1]['joints']['caster_positive_hemisphere_joint']==.1
    assert frames[2]['joints']['caster_positive_hemisphere_joint']==2.
    assert frames[3]['joints']['caster_positive_hemisphere_joint']==2.
    assert frames[3]['status']['nodes'][0]['display_state']=='invalid_held'
    assert frames[4]['status']['nodes'][0]['display_state']=='stale_held'
    assert frames[2]['joints']['caster_positive_polar_roller_joint']==0.


def test_missing_encoder_does_not_produce_a_fabricated_video():
    with pytest.raises(ValueError,match='No complete'):
        sample_frames([(100,envelope(1))],[100,200])


def test_plots_keep_batched_samples_and_share_integer_bag_time_with_video():
    start = 1792030863809132538
    packet = envelope(1, seq=0, angle=-10.)
    packet['packet']['samples'].append(envelope(1, seq=1, angle=-9.)['packet']['samples'][0])
    series, rejected = collect_readings([(start+123_456_789, packet)], start)
    rows = series['H1']
    assert rejected == 0
    assert [row['angle_rad'] for row in rows] == [-10., -9.]
    assert [row['time_s'] for row in rows] == [.123456789, .123456789]
    assert [row['segment'] for row in rows] == [0, 0]
    # At a shared timestamp the replay and chart use the last batched reading;
    # a future packet is never used early, including at large epoch values.
    plots = EncoderPlots.__new__(EncoderPlots)
    plots.series, plots.times = series, {'H1': [row['bag_time_ns'] for row in rows]}
    assert plots.current('H1', start+123_456_788) is None
    assert plots.current('H1', start+123_456_789)['angle_rad'] == -9.


def test_plot_curves_break_at_invalid_sequences_reboots_and_stale_gaps():
    events = [(0, envelope(1, seq=0, angle=-2.)),
              (10, envelope(1, seq=1, valid=False)),
              (20, envelope(1, seq=2, angle=-1.)),
              (30, envelope(1, seq=2, angle=99.)),  # duplicate
              (40, envelope(1, seq=1, angle=99.)),  # reordered
              (50, envelope(1, seq=0, angle=1., boot=SECOND_BOOT)),
              (60, envelope(1, seq=3, angle=99.)),  # superseded boot
              (2_000_000_000, envelope(1, seq=1, angle=2., boot=SECOND_BOOT)),
              (2_000_000_001, 'malformed')]
    series, rejected = collect_readings(events, 0)
    rows = series['H1']
    assert rejected == 1
    assert [row['angle_rad'] for row in rows] == [-2., -1., 1., 2.]
    assert [row['segment'] for row in rows] == [0, 1, 2, 3]


def test_center_wrap_breaks_curve_without_unwrapping_or_discarding_endpoints():
    events = [(0, envelope(3, angle=6.2)), (10, envelope(3, seq=1, angle=.01)),
              (20, envelope(3, seq=2, angle=.02))]
    series, _ = collect_readings(events, 0)
    x, y = curve_points(series['CENTER'])
    assert y[0] == 6.2 and math.isnan(y[1]) and y[2:] == [.01, .02]
    assert x[0] == 0 and math.isnan(x[1]) and x[2:] == [1e-8, 2e-8]


def test_transform_composes_parent_mount_joint_axis_and_child_offset():
    robot=ET.fromstring('''<robot name="test"><link name="root"/><link name="a"/><link name="b"/>
      <joint name="hinge" type="continuous"><parent link="root"/><child link="a"/>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/></joint>
      <joint name="fixed" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0 1 0"/></joint></robot>''')
    result=link_transforms(robot,{'hinge':math.pi/2})
    np.testing.assert_allclose(result['b'][:3,3],[1,2,4],atol=1e-14)
    np.testing.assert_allclose(result['a'][:3,:3] @ [1,0,0],[0,1,0],atol=1e-14)


def test_video_and_plot_refuse_cross_wheel_packets():
    from test_caster_encoder_replay import wheel_envelope
    events = [(100, wheel_envelope(node)) for node in (1, 2, 3)]
    events.append((101, wheel_envelope(wheel="front_right")))
    with pytest.raises(ValueError, match="Expected wheel front_left"):
        sample_frames(events, [100, 101], wheel_id="front_left")
    with pytest.raises(ValueError, match="Expected wheel front_left"):
        collect_readings(events, 100)


def test_v2_plots_preserve_identity_in_full_sample_rows():
    from test_caster_encoder_replay import wheel_envelope
    series, rejected = collect_readings([(100, wheel_envelope())], 100, wheel_id="front_left")
    assert rejected == 0
    assert series["H1"][0]["wheel_id"] == "front_left"
    assert series["H1"][0]["device_id"] == "aa:bb:cc:dd:ee:01"


def test_v2_plot_csv_export_preserves_physical_identity(tmp_path):
    import csv
    from test_caster_encoder_replay import wheel_envelope
    events = [(100, wheel_envelope(node)) for node in (1, 2, 3)]
    plots = EncoderPlots(events, 100, 200, "front_left", 1600, wheel_id="front_left")
    plots.export(tmp_path / "front_left.mp4")
    with (tmp_path / "front_left.encoders.csv").open() as handle:
        rows = list(csv.DictReader(handle))
    assert {row["wheel_id"] for row in rows} == {"front_left"}
    assert {row["device_id"] for row in rows} == {f"aa:bb:cc:dd:ee:{n:02x}" for n in (1, 2, 3)}
    assert plots.metadata["wheel_id"] == "front_left"
    assert plots.metadata["devices"]["H1"] == ["aa:bb:cc:dd:ee:01"]


def test_video_bag_scan_checks_every_identity_and_retains_malformed_records(monkeypatch):
    import json
    from types import SimpleNamespace
    from render_caster_encoder_video import read_bag
    from test_caster_encoder_replay import wheel_envelope
    topic = "/caster_encoder/front_left/packets"
    messages = [(topic, "malformed", 100),
                (topic, json.dumps(wheel_envelope()), 200)]
    class Reader:
        def open(self, *args): pass
        def get_all_topics_and_types(self):
            return [SimpleNamespace(name=topic, type="std_msgs/msg/String")]
        def has_next(self): return bool(messages)
        def read_next(self): return messages.pop(0)
    monkeypatch.setitem(sys.modules, "rosbag2_py", SimpleNamespace(
        SequentialReader=Reader, StorageOptions=lambda **kw: kw,
        ConverterOptions=lambda *args: args))
    monkeypatch.setitem(sys.modules, "rclpy.serialization", SimpleNamespace(
        deserialize_message=lambda value, _: SimpleNamespace(data=value)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=object))
    events, start, end = read_bag("unused", wheel_id="front_left")
    assert len(events) == 2 and (start, end) == (100, 200)
    messages.append((topic, json.dumps(wheel_envelope(wheel="front_right")), 300))
    with pytest.raises(ValueError, match="Expected wheel front_left"):
        read_bag("unused", wheel_id="front_left")
