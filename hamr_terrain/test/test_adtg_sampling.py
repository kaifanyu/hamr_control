"""Validation guards can be checked without installing ONNX in the ROS env."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

MODULE = Path(__file__).resolve().parents[1]/"scripts/sample_adtg.py"
spec = importlib.util.spec_from_file_location("sample_adtg", MODULE)
sampling = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sampling)


@pytest.mark.parametrize("difficulties,shape,seed", [
    ([0], [321, 241], 7), ([101], [321, 241], 7),
    ([5, 5], [321, 241], 7), ([5], [1, 241], 7), ([5], [321, 241], -1),
])
def test_invalid_requests_rejected_before_generation(difficulties, shape, seed):
    with pytest.raises(ValueError):
        sampling.validate_request(difficulties, shape, seed)


def test_existing_artifacts_are_never_replaced(tmp_path):
    prior = tmp_path/"manifest.json"
    prior.write_text('{"complete":true}')
    with pytest.raises(ValueError, match="Refusing to overwrite"):
        sampling.require_empty_output(tmp_path)
    assert prior.read_text() == '{"complete":true}'


def test_statistics_keep_raw_meters_and_detect_nonfinite_output():
    values = np.arange(12, dtype=np.float32).reshape(3, 4)
    copy = values.copy()
    result = sampling.terrain_statistics(values, [3, 4])
    assert result["height_range_m"] == 11
    assert result["height_mean_m"] == 5.5
    assert np.array_equal(values, copy)
    values[0, 0] = np.nan
    with pytest.raises(ValueError, match="nonfinite"):
        sampling.terrain_statistics(values, [3, 4])


def test_wrong_package_version_is_rejected_before_source_access(tmp_path):
    with pytest.raises(ValueError, match="Expected adaptive-terrain-diffusion"):
        sampling.inspect_provenance(tmp_path/"missing", tmp_path/"missing", "0.0.0")
