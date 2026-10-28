"""Four physical ordinary boards: independent identity, clocks and channels."""
from contextlib import ExitStack
import copy
import csv
import json
from pathlib import Path
import sys
import zlib

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'scripts')]
from ordinary_caster import protocol
from ordinary_caster.live import SplitLiveState
from ordinary_caster.history import load_history
from ordinary_caster.analysis import write_analysis
import caster_recording_format as binary
import caster_recording_client as client
import test_caster_recording_client as ball_tests
from test_ordinary_history import make_board, write_run

FIXTURES = ROOT / 'test/fixtures/ordinary_caster'


def stem(wheel, node):
    return wheel + ('_swivel' if node == 5 else '_wheel')


def fixture_packets(wheel, node):
    return [json.loads(line) for line in (FIXTURES / (stem(wheel, node) + '.jsonl')).read_text().splitlines()]


def board(wheel='front_left', node=5, shift_us=0, mapped=True, boot='0123456789abcdef'):
    metadata, packet, mapping = make_board(wheel, boot=boot, mapped=mapped)
    active = protocol.channels(node)[0]
    inactive = 'wheel' if active == 'swivel' else 'swivel'
    mac = fixture_packets(wheel, node)[0]['device_id']
    for value in (metadata, packet):
        value.update(v=4, node_id=node, device_id=mac)
    metadata['role'] = protocol.ROLES[node]
    del metadata[inactive]
    for sample in packet['samples']:
        if active == 'wheel':
            sample['t_us'] = sample['wheel']['t_us']
        sample['t_us'] += shift_us
        if active == 'wheel':
            sample['wheel']['t_us'] = sample['t_us']
        del sample[inactive]
    if mapping:
        mapping.update(node_id=node, device_id=mac, offset_ns=mapping['offset_ns'] - shift_us * 1000)
    return metadata, packet, mapping


@pytest.mark.parametrize('wheel', protocol.WHEELS)
@pytest.mark.parametrize('node', (5, 6))
def test_actual_cpp_wire_and_binary_agree_for_each_board(wheel, node):
    packets = fixture_packets(wheel, node)
    for packet in packets:
        assert protocol.decode_packet(json.dumps(packet).encode()) == packet
        assert set(packet if packet['type'] == 'metadata' else packet['samples'][0]) & {'wheel', 'swivel'} == set(protocol.channels(node))
    path = FIXTURES / (stem(wheel, node) + '.bin')
    verified = binary.verify_log(path)
    decoded = list(binary.iter_samples(path))
    assert decoded == [packet['samples'][0] for packet in packets[:-1]]
    assert verified['sample_count'] == 3 and verified['node_id'] == node
    assert set(verified['flag_counts']) == set(protocol.channels(node))
    rows = [row for sample in decoded for row in protocol.channel_rows(packets[0], sample)]
    assert len(rows) == 3
    if node == 6:
        assert rows[0]['raw_count'] == str(-(1 << 63))


@pytest.mark.parametrize('node', (5, 6))
def test_split_shape_and_version_are_not_ambiguous(node):
    packet = fixture_packets('front_left', node)[0]
    bad = copy.deepcopy(packet)
    bad['v'] = 3
    with pytest.raises(ValueError, match='version'):
        protocol.decode_packet(json.dumps(bad).encode())
    bad = copy.deepcopy(packet)
    inactive = 'wheel' if node == 5 else 'swivel'
    bad['samples'][0][inactive] = {}
    with pytest.raises(ValueError, match='inactive'):
        protocol.decode_packet(json.dumps(bad).encode())


def test_four_clocks_map_channels_without_cross_board_sequence_or_time_assumptions(tmp_path):
    boards = [board(wheel, node, shift_us=index * 1000000)
              for index, (wheel, node) in enumerate((('front_left', 5), ('front_left', 6), ('front_right', 5), ('front_right', 6)))]
    history = load_history(write_run(tmp_path / 'run', *boards), require_clock=True)
    assert len(history.streams) == 4
    assert len({stream.identity for stream in history.streams}) == 4
    common = 1_700_000_000_000_000_000 + 1_010_000 * 1000
    pose = history.pose_at(common)
    assert pose['complete_pose_available'] and len(pose['joint_names']) == 4
    left, right = history.streams[0], history.streams[2]
    assert left.observations[2].t_us != right.observations[2].t_us
    assert left.observations[2].time_ns == right.observations[2].time_ns
    report = write_analysis(history, tmp_path / 'analysis', plots=False)
    assert len(report['streams']) == 4
    assert {stream['node_id'] for stream in report['streams']} == {5, 6}
    rows = list(csv.DictReader((tmp_path / 'analysis/history.csv').open()))
    assert len(rows) == 20 and len({row['device_id'] for row in rows}) == 4
    assert {row['node_id'] for row in rows} == {'5', '6'}


def test_even_same_caster_sensors_require_clock_maps(tmp_path):
    path = write_run(tmp_path / 'run', board(node=5, mapped=False), board(node=6, shift_us=1000000, mapped=False))
    with pytest.raises(ValueError, match='Cannot align'):
        load_history(path, require_clock=True)
    with pytest.raises(ValueError, match='one sensor board'):
        load_history(path, wheel='front_left', device_local=True)
    history = load_history(path, wheel='front_left', role='SWIVEL', device_local=True)
    pose = history.pose_at(history.start_ns)
    assert pose['joint_names'] == ['front_left_caster_swivel_joint']
    assert not pose['complete_pose_available']
    assert any(item['state'] == 'sensor_history_missing' for item in pose['channels'])


def test_identity_guard_rejects_reused_mac_and_combined_overlap():
    guard = protocol.IdentityGuard()
    left = fixture_packets('front_left', 5)[0]
    right = fixture_packets('front_left', 6)[0]
    guard.accept(left)
    guard.accept(right)
    with pytest.raises(ValueError, match='assignment conflict'):
        guard.accept(dict(right, device_id=left['device_id']))
    combined = make_board()[1]
    with pytest.raises(ValueError, match='overlapping'):
        guard.accept(combined)


def receive(state, packet, now=1000000000):
    return state.receive(json.dumps(packet).encode(), ('127.0.0.1', 7000), 1700000000000000000, now, 1700000000000000000)


def test_live_independent_boots_failure_staleness_and_metadata():
    state = SplitLiveState('front_left')
    swivel = board(node=5)
    wheel = board(node=6, shift_us=1000000)
    for metadata, packet, _ in (swivel, wheel):
        receive(state, metadata)
        assert len(receive(state, packet)[1]) == 5
    report = state.diagnostics_snapshot(1000000001)
    assert set(report['boards']) == {'5', '6'}
    assert set(report['channels']) == {'swivel', 'wheel'}
    assert not any(ch['stale'] for ch in report['channels'].values())
    new = board(node=6, boot='fedcba9876543210')
    receive(state, new[0], now=2000000000)
    receive(state, new[1], now=2000000000)
    # A wheel-only reboot must neither clear nor reset the swivel's history.
    assert state.boards[5].boot_id == swivel[0]['boot_id']
    assert receive(state, wheel[1], now=2500000000)[1] == []
    report = state.diagnostics_snapshot(2500000000)
    assert report['channels']['swivel']['stale']
    assert not report['channels']['wheel']['stale']
    assert report['boards']['5']['metadata']['swivel']['resolution_bits'] == 12


class Recorder(ball_tests.Recorder):
    def __init__(self, wheel, node):
        super().__init__(node, wheel)
        self.device.update({key: fixture_packets(wheel, node)[0][key]
                            for key in ('caster_type', 'device_id', 'boot_id')}, record_version=2, record_bytes=116)

    def status(self):
        result = super().status()
        result.update(start_us=1000000 if self.run else None, stop_us=1010100 if self.state == 'stopped' else None)
        return result


def make_log(device, run='trial'):
    path = FIXTURES / (stem(device['wheel_id'], device['node_id']) + '.bin')
    with path.open('rb') as stream:
        metadata = binary.read_header(stream)
    metadata['run_id'] = run
    metadata['packet'].update(device_id=device['device_id'], boot_id=device['boot_id'])
    encoded = json.dumps(metadata, separators=(',', ':')).encode()
    header = binary.HEADER.pack(b'CASTLOG2', 2, 116, len(encoded), zlib.crc32(encoded)) + encoded
    header += b'\0' * (4096 - len(header))
    records = path.read_bytes()[4096:]
    return header + records, zlib.crc32(records)


def test_four_recorder_real_http_start_stop_download_recovery(tmp_path, monkeypatch):
    monkeypatch.setattr(ball_tests, 'make_log', make_log)
    with ExitStack() as stack:
        recorders = [stack.enter_context(Recorder(wheel, node)) for wheel in client.WHEELS for node in (5, 6)]
        devices = client.validate_inventory([r.device for r in recorders], caster_type='ordinary')
        assert len(devices) == 4
        session = client.Session.create(tmp_path, 'trial', devices, caster_type='ordinary', sync_samples=1)
        session.start()
        recorders[0].fail_downloads = 1
        recorders[3].fail_ack = True
        with pytest.raises(client.RecordingError, match='incomplete'):
            session.stop()
        assert all(item.get('verified') for item in session.manifest['devices'])
        recorders[3].fail_ack = False
        restored = client.Session.load(tmp_path, 'trial', caster_type='ordinary', sync_samples=1)
        restored.fetch()
        assert restored.manifest['state'] == 'complete'
        assert restored.manifest['ordinary_layout'] == 'split'
        assert len({item['log_path'] for item in restored.manifest['devices']}) == 4
        assert all(item['acknowledged'] for item in restored.manifest['devices'])
        rows = list(csv.DictReader((restored.directory / 'samples.csv').open()))
        assert len(rows) == 12 and len({row['device_id'] for row in rows}) == 4
        history = load_history(restored.directory)
        assert len(history.streams) == 4


def test_default_start_requires_four_boards_before_any_commands(tmp_path, monkeypatch):
    devices = [Recorder(wheel, node).device | {'host': '127.0.0.1', 'http_port': 8080}
               for wheel in client.WHEELS for node in (5, 6)]
    with pytest.raises(ValueError, match='missing recorders.*front_right/WHEEL'):
        client.validate_inventory(devices[:-1], caster_type='ordinary')
    monkeypatch.setattr(client, 'discover', lambda *a: devices[:-1])
    monkeypatch.setattr(client, 'load_config', lambda *a: {})
    monkeypatch.setattr(client.Session, 'start', lambda *a: pytest.fail('incomplete inventory must not start'))
    assert client.main(['--root', str(tmp_path), '--caster-type', 'ordinary', 'start', 'bad']) == 2
    assert not (tmp_path / 'encoder_bad').exists()
    assert len(client.validate_inventory(devices, ('front_left',), caster_type='ordinary')) == 2
    combined = make_board()[1] | {'host': '127.0.0.1', 'http_port': 8080}
    with pytest.raises(ValueError, match='layout'):
        client.validate_inventory([combined], ('front_left',), caster_type='ordinary')
