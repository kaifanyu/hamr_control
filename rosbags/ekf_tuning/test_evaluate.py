"""Evaluation rejects truncated outputs and only removes a common initial pose."""
import numpy as np
import pytest

from evaluate import Recording, relative_pose, interp_pose


def test_rigid_initial_transform_does_not_change_relative_path():
    t = np.arange(0., 3., .1)
    pose = np.column_stack([t, .2 * t**2, .1 * t])
    a = .7
    rotation = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
    transformed = pose.copy()
    transformed[:, :2] = pose[:, :2] @ rotation.T + [4., -2.]
    transformed[:, 2] += a
    np.testing.assert_allclose(relative_pose(pose), relative_pose(transformed), atol=1e-14)


def test_yaw_interpolation_crosses_wrap_continuously():
    out = interp_pose([.5], np.array([[0., 0., 0., 3.1], [1., 0., 0., -3.1]]))
    assert out[0, 2] == pytest.approx(np.pi)


@pytest.mark.parametrize('trajectory', [
    [[.2, 0., 0., 0.], [1., 1., 0., 0.]],
    [[0., 0., 0., 0.], [.8, 1., 0., 0.]],
    [[0., np.nan, 0., 0.], [1., 1., 0., 0.]],
])
def test_score_rejects_missing_coverage_or_nonfinite_output(trajectory):
    recording = Recording.__new__(Recording)
    recording.times = np.array([0., .5, 1.])
    with pytest.raises(ValueError):
        recording.score(np.array(trajectory))
