"""Load an editable offline replay profile without importing ROS packages."""

from copy import deepcopy
import math
from pathlib import Path

import numpy as np
import yaml


COVARIANCE_SIZES = {
    "wheel_pose": 6,
    "wheel_twist": 6,
    "imu_orientation": 3,
    "imu_angular_velocity": 3,
    "imu_linear_acceleration": 3,
}
PROFILE_KEYS = {"base_ekf_config", "ekf", "covariances"}
STATE_NAMES = (
    "x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
    "vroll", "vpitch", "vyaw", "ax", "ay", "az",
)


def _read_mapping(path):
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8-sig"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f"Cannot read YAML {path}: {exc}") from exc
    if not isinstance(document, dict):
        raise ValueError(f"{path} must contain a YAML mapping")
    return document


def _matrix(name, rows, size):
    """Return a validated covariance as a ROS row-major float array."""
    if not isinstance(rows, list) or len(rows) != size or any(
        not isinstance(row, list) or len(row) != size for row in rows
    ):
        raise ValueError(f"covariances.{name} must have {size} rows of {size} numbers")
    values = [value for row in rows for value in row]
    if any(
        type(value) not in (int, float) or not math.isfinite(value)
        for value in values
    ):
        raise ValueError(f"covariances.{name} entries must be finite numbers, not booleans")
    matrix = np.asarray(rows, dtype=float)
    if np.any(np.diag(matrix) <= 0):
        raise ValueError(f"covariances.{name} diagonal variances must be positive")
    scale = float(np.max(np.abs(matrix)))
    tolerance = np.finfo(float).eps * size * scale * 10
    if not np.allclose(matrix, matrix.T, rtol=0, atol=tolerance):
        raise ValueError(f"covariances.{name} must be symmetric (matching cross terms)")
    if float(np.linalg.eigvalsh(matrix).min()) < -tolerance:
        raise ValueError(f"covariances.{name} must be positive semidefinite; reduce cross terms")
    return matrix.ravel().tolist()


def _selection(parameters, key, unavailable):
    selected = parameters.get(key)
    if not isinstance(selected, list) or len(selected) != 15 or any(
        type(value) is not bool for value in selected
    ):
        raise ValueError(f"ekf.{key} must contain exactly 15 true/false values")
    invalid = [STATE_NAMES[index] for index in unavailable if selected[index]]
    if invalid:
        raise ValueError(f"ekf.{key} selects unavailable measurements: {', '.join(invalid)}")


def load_profile(path):
    """Return (covariance_overrides, merged_ekf_document).

    The profile's base_ekf_config is relative to the profile file. Matrix values
    are flattened to float lists; null/omitted matrices preserve bag values.
    The returned EKF document uses the same wildcard ROS parameter layout as
    the base config, with profile overrides and simulation time enabled.
    No files or caller-owned data are modified.
    """
    profile_path = Path(path).expanduser().resolve()
    profile = _read_mapping(profile_path)
    unknown = set(profile) - PROFILE_KEYS
    if unknown:
        raise ValueError(f"Unknown profile keys: {', '.join(sorted(map(str, unknown)))}")
    base_name = profile.get("base_ekf_config")
    if not isinstance(base_name, str) or not base_name.strip():
        raise ValueError("base_ekf_config must name a base EKF YAML file")
    base_path = Path(base_name).expanduser()
    if not base_path.is_absolute():
        base_path = profile_path.parent / base_path
    base_document = _read_mapping(base_path.resolve())
    wildcard = base_document.get("/**")
    if not isinstance(wildcard, dict) or not isinstance(wildcard.get("ros__parameters"), dict):
        raise ValueError("Base EKF YAML must contain '/**: ros__parameters:'")
    parameters = deepcopy(wildcard["ros__parameters"])
    ekf_overrides = profile.get("ekf", {})
    if not isinstance(ekf_overrides, dict):
        raise ValueError("ekf must be a mapping of ROS parameter overrides")
    parameters.update(deepcopy(ekf_overrides))
    parameters["use_sim_time"] = True
    for sensor, topic in (("odom0", "/wheel_odom"), ("imu0", "/imu/data")):
        if parameters.get(sensor) != topic:
            raise ValueError(f"ekf.{sensor} must be {topic} for the offline launch remappings")
    _selection(parameters, "odom0_config", (12, 13, 14))
    _selection(parameters, "imu0_config", (0, 1, 2, 6, 7, 8))

    covariances = profile.get("covariances", {})
    if not isinstance(covariances, dict):
        raise ValueError("covariances must be a mapping of matrices (or null per matrix)")
    unknown = set(covariances) - set(COVARIANCE_SIZES)
    if unknown:
        raise ValueError(f"Unknown covariance keys: {', '.join(sorted(map(str, unknown)))}")
    overrides = {
        name: _matrix(name, rows, COVARIANCE_SIZES[name])
        for name, rows in covariances.items() if rows is not None
    }
    return overrides, {"/**": {"ros__parameters": parameters}}
