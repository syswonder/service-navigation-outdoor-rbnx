import json
from pathlib import Path

import pytest

from outdoor_nav.configuration import validate
from outdoor_nav.ground_reference import load_ground_reference

ROOT = Path(__file__).resolve().parents[1]


def test_file_is_mandatory_even_for_lidar_only(tmp_path):
    with pytest.raises(ValueError, match="ground_height_file is required"):
        load_ground_reference({}, tmp_path)
    with pytest.raises(ValueError, match="Cannot read"):
        load_ground_reference({"ground_height_file":"missing.json"}, tmp_path)


@pytest.mark.parametrize("value", [None, True, False, 0, .2, "-0.25", float("nan"), float("inf")])
def test_no_fallback_for_invalid_machine_value(tmp_path, value):
    path=tmp_path/"ground.local.json"
    path.write_text(json.dumps({"ground_z_m":value}))
    with pytest.raises(ValueError, match="finite negative"):
        load_ground_reference({"ground_height_file":path.name},tmp_path)


def test_single_value_drives_both_consumers(tmp_path):
    path=tmp_path/"ground.local.json"
    path.write_text(json.dumps({"ground_z_m":-.25,"source":"synthetic unit test"}))
    cfg={"params_file":str(ROOT/"config/nav2_params.example.yml"),"ground_height_file":path.name}
    runtime=validate(cfg,base=tmp_path)
    assert runtime["terrain"]["ground_z_m"] == -.25
    assert runtime["policy"]["camera_expected_ground_z_m"] == -.25
    assert runtime["ground_height_file"] == str(path)
    assert "ground_z_m" not in cfg.get("terrain",{})
    assert "camera_expected_ground_z_m" not in cfg.get("policy",{})


def test_no_inline_duplicate_ground_value(tmp_path):
    path=tmp_path/"ground.local.json";path.write_text('{"ground_z_m":-0.25}')
    for section,key in (("terrain","ground_z_m"),("policy","camera_expected_ground_z_m")):
        with pytest.raises(ValueError,match="Do not set"):
            load_ground_reference({"ground_height_file":path.name,section:{key:-.25}},tmp_path)


def test_changed_file_is_used_on_next_validation(tmp_path):
    path=tmp_path/"ground.local.json";path.write_text('{"ground_z_m":-0.25}')
    cfg={"params_file":str(ROOT/"config/nav2_params.example.yml"),"ground_height_file":path.name}
    assert validate(cfg,base=tmp_path)["terrain"]["ground_z_m"] == -.25
    path.write_text('{"ground_z_m":-0.33}')
    assert validate(cfg,base=tmp_path)["policy"]["camera_expected_ground_z_m"] == -.33
