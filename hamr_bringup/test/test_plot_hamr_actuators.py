"""Offline contract tests for the generic HAMR actuator plotter."""

from importlib.util import module_from_spec
from importlib.util import spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


SCRIPT = (
    Path(__file__).resolve().parents[2]
    / 'rosbags'
    / 'plot_hamr_actuators.py'
)
SPEC = spec_from_file_location('plot_hamr_actuators', SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def status_message(
    *,
    data=None,
    label='wheel_control_status_v1',
    size=17,
    stride=17,
    data_offset=0,
    dimension_count=1,
):
    """Build one Float64MultiArray-shaped status message."""
    dimensions = [
        SimpleNamespace(label=label, size=size, stride=stride)
        for _index in range(dimension_count)
    ]
    values = [0.0] * 17 if data is None else list(data)
    if len(values) == 17 and data is None:
        values[16] = 1.0
    return SimpleNamespace(
        layout=SimpleNamespace(
            dim=dimensions,
            data_offset=data_offset,
        ),
        data=values,
    )


def test_exact_wheel_status_v1_layout_and_source_decode():
    """Accept only the documented 17-field firmware status schema."""
    rows = [
        (1, status_message()),
        (2, status_message(data=[0.0] * 16 + [7.0])),
    ]

    valid, reason = MODULE.wheel_status_v1_layout(rows[0][1])
    status, sources, source_reason = MODULE.wheel_status_v1_array(rows)

    assert valid
    assert reason == ''
    assert status.shape == (2, 17)
    assert sources is not None
    assert sources.tolist() == [1, 7]
    assert source_reason == ''


@pytest.mark.parametrize(
    ('message', 'reason_fragment'),
    (
        (
            status_message(dimension_count=0),
            'exactly one dimension',
        ),
        (
            status_message(dimension_count=2),
            'exactly one dimension',
        ),
        (
            status_message(label='legacy_status'),
            'not wheel_control_status_v1',
        ),
        (
            status_message(size=16),
            'size=17, stride=17, data_offset=0',
        ),
        (
            status_message(stride=16),
            'size=17, stride=17, data_offset=0',
        ),
        (
            status_message(data_offset=1),
            'size=17, stride=17, data_offset=0',
        ),
        (
            status_message(data=[0.0] * 16),
            'data length is not 17',
        ),
    ),
)
def test_wheel_status_layout_mismatch_fails_before_field_labels(
    message, reason_fragment
):
    """Never assign target/measured/saturation labels to unknown layouts."""
    valid, reason = MODULE.wheel_status_v1_layout(message)

    assert not valid
    assert reason_fragment in reason
    with pytest.raises(RuntimeError, match=reason_fragment):
        MODULE.wheel_status_v1_array([(1, message)])


@pytest.mark.parametrize('source', [np.nan, 1.5, -1.0, 8.0])
def test_wheel_status_source_must_be_finite_integer_enum(source):
    """Keep actuator fields but mark an invalid source enum unavailable."""
    values = [0.0] * 17
    values[16] = source

    status, sources, reason = MODULE.wheel_status_v1_array(
        [(1, status_message(data=values))]
    )

    assert status.shape == (1, 17)
    assert sources is None
    assert 'finite integer codes in 0..7' in reason


def test_wheel_status_actuator_fields_must_be_finite():
    values = [0.0] * 17
    values[3] = np.inf
    values[16] = 1.0

    with pytest.raises(RuntimeError, match='fields 0..15 must be finite'):
        MODULE.wheel_status_v1_array(
            [(1, status_message(data=values))]
        )


def test_forward_reverse_cap_denominator_includes_planned_zero_dwell():
    """Match the analyzer's outbound-through-reverse active definition."""
    reference_t = np.arange(0.0, 1.001, 0.02)
    reference_v = np.zeros((reference_t.size, 2))
    reference_v[5:20, 1] = 0.15
    reference_v[30:45, 1] = -0.15
    left = np.zeros(reference_t.size)
    right = np.zeros(reference_t.size)
    left[5] = 3.0

    exposure = MODULE.command_cap_exposure(
        reference_t,
        left,
        reference_t,
        right,
        reference_t,
        reference_v,
        wheel_cap=3.0,
        profile='forward_reverse',
    )

    assert exposure['available']
    assert exposure['include_internal_dwell']
    assert exposure['sample_count'] == 40
    assert exposure['cap_count'] == 1
    assert exposure['cap_fraction'] == pytest.approx(1.0 / 40.0)


def test_one_phase_cap_denominator_uses_only_active_reference_samples():
    reference_t = np.arange(0.0, 0.401, 0.02)
    reference_v = np.zeros((reference_t.size, 2))
    reference_v[5:15, 0] = 0.15
    wheel = np.zeros(reference_t.size)

    exposure = MODULE.command_cap_exposure(
        reference_t,
        wheel,
        reference_t,
        wheel,
        reference_t,
        reference_v,
        wheel_cap=3.0,
    )

    assert exposure['available']
    assert not exposure['include_internal_dwell']
    assert exposure['cap_count'] == 0
    assert exposure['sample_count'] == 10
    assert exposure['cap_fraction'] == 0.0


def test_sparse_command_support_marks_cap_exposure_unavailable():
    """Never label sparse command traces as zero-percent cap exposure."""
    reference_t = np.arange(0.0, 1.001, 0.02)
    reference_v = np.column_stack(
        (np.full(reference_t.size, 0.2), np.zeros(reference_t.size))
    )
    keep = ~((reference_t >= 0.40) & (reference_t <= 0.70))
    left_t = reference_t[keep]
    left = np.zeros(left_t.size)
    right = np.zeros(reference_t.size)

    exposure = MODULE.command_cap_exposure(
        left_t,
        left,
        reference_t,
        right,
        reference_t,
        reference_v,
        wheel_cap=3.0,
    )
    label = MODULE.cap_exposure_text(exposure, 3.0)

    assert not exposure['available']
    assert exposure['cap_count'] is None
    assert exposure['cap_fraction'] is None
    assert 'left wheel command active_phase_1: coverage' in exposure['reason']
    assert 'effective topic support gap' in exposure['reason']
    assert 'exposure unavailable' in label
    assert '0.0%' not in label


def test_exact_50ms_match_and_100ms_gap_boundaries_are_valid():
    """Use the same inclusive support thresholds as the analyzer."""
    reference_t = np.arange(0.0, 0.201, 0.05)
    reference_v = np.column_stack(
        (np.full(reference_t.size, 0.2), np.zeros(reference_t.size))
    )
    observed_t = np.asarray([0.0, 0.1, 0.2])

    support = MODULE.actuator_time_support(
        reference_t,
        reference_v,
        observed_t,
        include_internal_dwell=False,
        stream_label='wheel status',
    )

    assert support['valid']
    window = support['windows'][0]
    assert window['coverage_fraction'] == 1.0
    assert window['max_reference_match_dt_s'] == pytest.approx(0.05)
    assert window['max_effective_topic_gap_s'] == pytest.approx(0.1)


def test_forward_reverse_dwell_dropouts_invalidate_command_and_status_exposure():
    """The planned midpoint hold is required support for reversal exposure."""
    reference_t = np.arange(0.0, 1.001, 0.02)
    reference_v = np.zeros((reference_t.size, 2))
    reference_v[5:20, 1] = 0.15
    reference_v[30:45, 1] = -0.15
    keep = np.ones(reference_t.size, dtype=bool)
    keep[20:30] = False
    observed_t = reference_t[keep]
    command = np.zeros(observed_t.size)
    status = np.zeros((observed_t.size, 17))
    status[:, 16] = 1.0

    cap = MODULE.command_cap_exposure(
        observed_t,
        command,
        observed_t,
        command,
        reference_t,
        reference_v,
        wheel_cap=3.0,
        profile='forward_reverse',
    )
    firmware = MODULE.wheel_status_exposure(
        observed_t,
        status,
        reference_t,
        reference_v,
        profile='forward_reverse',
    )

    assert not cap['available']
    assert cap['cap_fraction'] is None
    assert 'effective topic support gap' in cap['reason']
    assert not firmware['available']
    assert firmware['saturation_fraction'] is None
    assert 'effective topic support gap' in firmware['reason']


def test_nonreversal_disjoint_phases_do_not_require_dwell_support():
    """Assess ordinary disjoint motion phases without bridging their hold."""
    reference_t = np.arange(0.0, 2.001, 0.02)
    reference_v = np.zeros((reference_t.size, 2))
    reference_v[5:20, 0] = 0.2
    reference_v[60:75, 0] = 0.2
    active = np.linalg.norm(reference_v, axis=1) > 0.02
    command_t = reference_t[active]
    command = np.zeros(command_t.size)

    exposure = MODULE.command_cap_exposure(
        command_t,
        command,
        command_t,
        command,
        reference_t,
        reference_v,
        wheel_cap=3.0,
        profile='straight_forward',
    )

    assert exposure['available']
    assert exposure['sample_count'] == 30
    assert len(exposure['left_support']['phase_support']) == 2


def test_sparse_status_support_suppresses_all_status_exposure_summaries():
    """Sparse status traces remain diagnostic but cannot claim zero exposure."""
    reference_t = np.arange(0.0, 1.001, 0.02)
    reference_v = np.column_stack(
        (np.full(reference_t.size, 0.2), np.zeros(reference_t.size))
    )
    keep = ~((reference_t >= 0.40) & (reference_t <= 0.70))
    status_t = reference_t[keep]
    status = np.zeros((status_t.size, 17))
    status[:, 16] = 1.0

    exposure = MODULE.wheel_status_exposure(
        status_t,
        status,
        reference_t,
        reference_v,
    )
    label = MODULE.saturation_exposure_text(exposure)

    assert not exposure['available']
    assert exposure['saturation_count'] is None
    assert exposure['saturation_fraction'] is None
    assert exposure['source_values'] is None
    assert 'wheel status active_phase_1: coverage' in exposure['reason']
    assert 'exposure unavailable' in label
    assert '0.0%' not in label


def test_supported_status_can_report_zero_exposure_on_reference_clock():
    reference_t = np.arange(0.0, 1.001, 0.02)
    reference_v = np.column_stack(
        (np.full(reference_t.size, 0.2), np.zeros(reference_t.size))
    )
    status = np.zeros((reference_t.size, 17))
    status[:, 16] = 1.0

    exposure = MODULE.wheel_status_exposure(
        reference_t,
        status,
        reference_t,
        reference_v,
    )
    label = MODULE.saturation_exposure_text(exposure)

    assert exposure['available']
    assert exposure['saturation_count'] == 0
    assert exposure['saturation_fraction'] == 0.0
    assert exposure['source_values'].tolist() == [1] * reference_t.size
    assert '0.0%' in label


def test_status_source_validation_uses_supported_active_samples_only():
    reference_t = np.arange(0.0, 1.001, 0.02)
    reference_v = np.zeros((reference_t.size, 2))
    reference_v[5:45, 0] = 0.2
    status = np.zeros((reference_t.size, 17))
    status[:, 16] = 1.0
    status[0, 16] = 7.4

    exposure = MODULE.wheel_status_exposure(
        reference_t,
        status,
        reference_t,
        reference_v,
    )

    assert exposure['available']
    assert exposure['source_reason'] == ''
    assert exposure['source_values'].tolist() == [1] * 40


def test_sparse_actuator_traces_still_render_with_unavailable_labels(
    tmp_path, monkeypatch
):
    """Keep diagnostic figures while withholding unsupported percentages."""
    reference_t = np.arange(0.0, 1.001, 0.02)
    keep = ~((reference_t >= 0.40) & (reference_t <= 0.70))

    def rows(timestamps, message):
        return [
            (int(round(timestamp * 1e9)), message(index))
            for index, timestamp in enumerate(timestamps)
        ]

    status_values = [0.0] * 17
    status_values[16] = 1.0
    data = {
        'reference': rows(
            reference_t,
            lambda _index: SimpleNamespace(x_dot=0.2, y_dot=0.0),
        ),
        'left_cmd': rows(
            reference_t[keep],
            lambda _index: SimpleNamespace(data=0.0),
        ),
        'right_cmd': rows(
            reference_t[keep],
            lambda _index: SimpleNamespace(data=0.0),
        ),
        'turret_cmd': rows(
            reference_t,
            lambda _index: SimpleNamespace(data=0.0),
        ),
        'gains': rows(
            reference_t,
            lambda _index: SimpleNamespace(d_x=0.0, d_y=0.0),
        ),
        'status': rows(
            reference_t[keep],
            lambda _index: status_message(data=status_values),
        ),
        'left_enc': rows(
            reference_t,
            lambda index: SimpleNamespace(data=float(index)),
        ),
        'right_enc': rows(
            reference_t,
            lambda index: SimpleNamespace(data=float(index)),
        ),
        'turret_enc': rows(
            reference_t,
            lambda _index: SimpleNamespace(data=0.0),
        ),
    }
    monkeypatch.setattr(MODULE, 'load_bag', lambda _bag: data)

    overview, stop_zoom, summary = MODULE.plot(
        tmp_path / 'sparse_bag',
        tmp_path / 'actuator',
        wheel_cap=3.0,
    )

    assert overview.is_file()
    assert stop_zoom.is_file()
    assert summary.count('exposure unavailable') >= 2
    assert '0.0%' not in summary


def test_command_pair_alignment_matches_analyzer_sampling_policy():
    left_t = np.asarray([1.0, 2.0, 3.0])
    left_v = np.asarray([1.0, 2.0, 3.0])
    right_t = np.asarray([1.01, 2.01, 3.03])
    right_v = np.asarray([-1.0, -2.0, -3.0])

    timestamps, pairs = MODULE.align_commands(
        left_t, left_v, right_t, right_v
    )

    assert timestamps.tolist() == [1.0, 2.0]
    assert pairs[:, 0].tolist() == [1.0, 2.0]
    assert pairs[:, 1].tolist() == pytest.approx([-1.0, -1.99])
