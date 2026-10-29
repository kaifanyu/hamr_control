"""Every robot recorder must retain the case-sensitive fork Vicon subjects."""
from pathlib import Path
import re

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("script", ["hamr_bringup/scripts/record_hamr_test_bag",
                                    "hamr_bringup/scripts/record_hamr_vicon_bag",
                                    "compa_slam/scripts/record_compa_slam_bag"])
def test_fork_subjects_are_recorded_with_base_and_turret(script):
    source = (ROOT / script).read_text()
    pattern = re.search(r"^TOPIC_REGEX='([^']+)'", source, re.MULTILINE).group(1)
    for subject in ("Left_Caster", "Right_Caster", "HAMR_base", "HAMR_turret"):
        for topic in ("odom", "pose", "individual_marker_array"):
            assert re.search(pattern, f"/{subject}/{topic}")
    assert not re.search(pattern, "/left_caster/pose")
    assert not re.search(pattern, "/Right_Caster/pose_extra")


@pytest.mark.parametrize("config", ["hamr_bringup/config/record_qos.yaml",
                                    "compa_slam/config/record_slam_qos.yaml"])
def test_fork_pose_delivery_accepts_best_effort_publishers(config):
    qos = yaml.safe_load((ROOT / config).read_text())
    for subject in ("Left_Caster", "Right_Caster"):
        for topic in ("odom", "pose"):
            assert qos[f"/{subject}/{topic}"] == dict(reliability="best_effort",
                                                     durability="volatile",
                                                     history="keep_last", depth=1)
