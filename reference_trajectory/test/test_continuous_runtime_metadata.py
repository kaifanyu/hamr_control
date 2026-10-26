"""Keep long-route startup work outside the armed reference watchdog."""

import copy
import json
from types import SimpleNamespace

import pytest

from reference_trajectory.continuous_execution import build_plan, validate_config
from reference_trajectory import continuous_waypoint_node as adapter


@pytest.fixture
def short_plan():
    cfg = validate_config({'points_m_flat': [0.0, 0.0, 0.0, 0.10]})
    return cfg, build_plan(cfg)


def test_live_document_keeps_provenance_without_mutating_dense_plan(short_plan):
    cfg, plan = short_plan
    original = copy.deepcopy(plan.metadata())
    cached_samples = plan.metadata()['samples']

    live = adapter.runtime_document(cfg, plan)

    assert 'samples' not in live['continuous_plan']
    export = live['continuous_plan']['sample_export']
    assert export['omitted_from_live_metadata'] is True
    assert export['sample_count'] == len(cached_samples)
    assert '--plan-only' in export['full_export_command']
    assert live['continuous_plan']['certification'] == original['certification']
    assert live['config'] == cfg
    assert len(live['planner_sha256']) == 64
    assert plan.metadata() == original
    assert plan.metadata()['samples'] is cached_samples
    assert adapter.plan_document(cfg, plan)['continuous_plan'] == original


def test_live_serialization_does_not_grow_with_dense_sample_count(short_plan):
    cfg, plan = short_plan
    metadata = dict(plan.metadata())
    metadata['samples'] = metadata['samples'] * 1000
    long_plan = SimpleNamespace(metadata=lambda: metadata)

    short_json = json.dumps(adapter.runtime_document(cfg, plan))
    long_json = json.dumps(adapter.runtime_document(cfg, long_plan))

    # Only the digit count of the reported number of omitted samples changes.
    assert abs(len(long_json) - len(short_json)) < 10
    assert 'samples' in metadata


def test_plan_only_still_exports_dense_samples(short_plan, monkeypatch, tmp_path):
    cfg, plan = short_plan
    monkeypatch.setattr(adapter, 'load_config', lambda _: cfg)
    monkeypatch.setattr(adapter, 'build_plan', lambda _: plan)
    destination = tmp_path / 'complete_plan.json'

    adapter.main(['--config', 'profile.yaml', '--plan-only', str(destination)])

    document = json.loads(destination.read_text())
    assert document['continuous_plan']['samples'] == plan.metadata()['samples']
    assert 'sample_export' not in document['continuous_plan']


def test_preview_reuses_poses_and_replaces_translation_and_header(short_plan):
    pytest.importorskip('geometry_msgs.msg')
    time_type = pytest.importorskip('builtin_interfaces.msg').Time
    _, plan = short_plan
    path, offsets = adapter.build_preview_path(plan)
    original_offsets = tuple(offsets)
    pose_ids = tuple(map(id, path.poses))

    adapter.place_preview_path(
        path, offsets, (3.0, -2.0), 'first_world', time_type(sec=10)
    )
    adapter.place_preview_path(
        path, offsets, (-4.0, 5.0), 'second_world', time_type(sec=20)
    )

    assert offsets == original_offsets
    assert tuple(map(id, path.poses)) == pose_ids
    assert path.header.frame_id == 'second_world'
    assert path.header.stamp.sec == 20
    assert offsets[-1] == tuple(plan.metadata()['samples'][-1]['position_m'])
    for pose, (x, y) in zip(path.poses, offsets):
        assert pose.pose.position.x == pytest.approx(x - 4.0)
        assert pose.pose.position.y == pytest.approx(y + 5.0)
        assert pose.header is path.header
        assert pose.header.frame_id == 'second_world'
        assert pose.header.stamp.sec == 20


def test_explicit_document_builder_remains_supported(short_plan, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    pytest.importorskip('hamr_interfaces.msg')
    cfg, plan = short_plan
    called = []

    def custom_builder(actual_cfg, actual_plan):
        called.append((actual_cfg, actual_plan))
        return dict(adapter.plan_document(actual_cfg, actual_plan), custom=True)

    # The node is never spun or armed; isolate discovery from live hardware.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init(domain_id=187)
    node = None
    try:
        node = adapter.create_node(
            config_defaults=cfg,
            plan_builder=lambda _: plan,
            document_builder=custom_builder,
        )
        assert called == [(cfg, plan)]
        assert node.document['custom'] is True
        assert 'samples' in node.document['continuous_plan']
        assert not node.execution.active
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
