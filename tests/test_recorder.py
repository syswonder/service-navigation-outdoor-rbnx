import json
import math
import os
import queue

import pytest

from outdoor_nav.recorder import Recorder, plain, validate


def test_validate_defaults_and_rejects():
    config = validate(None)
    assert config["enabled"] and config["state_hz"] == 10.0
    assert config["dir"] == "records" and config["imu_topic"].startswith("/")
    for bad in (
        {"dir": "/abs"},
        {"dir": "../out"},
        {"dir": ""},
        {"state_hz": 0},
        {"state_hz": 21},
        {"scan_hz": 0.1},
        {"flush_s": 9},
        {"enabled": "yes"},
        {"imu_enabled": 1},
        {"max_run_mb": 0},
        {"max_run_mb": 100, "max_total_mb": 10},
        {"imu_topic": "wit"},
        {"unknown": 1},
    ):
        with pytest.raises(ValueError):
            validate(bad)


def test_plain_removes_non_finite():
    assert plain({"a": math.nan, "b": [math.inf, 1.0], "c": {"d": -math.inf}}) == {
        "a": None,
        "b": [None, 1.0],
        "c": {"d": None},
    }


def test_records_only_while_active(tmp_path):
    recorder = Recorder({"flush_s": 0.2}, tmp_path)
    assert recorder.state(v=1.0) is False
    assert recorder.start({"task_id": "a" * 32}) is not None
    recorder.state(v=0.1, w=0.2)
    recorder.scan([1.0, None])
    recorder.imu([{"a": [0.0, 0.0, 9.8]}])
    recorder.event("blockers", reasons=["scan_stale"])
    directory = recorder.stop("succeeded", None)
    assert directory is not None and directory.is_dir()
    row = json.loads((directory / "state.jsonl").read_text().splitlines()[0])
    assert row["v"] == 0.1 and row["w"] == 0.2
    assert row["t"] > 0 and row["mono"] > 0
    result = json.loads((directory / "result.json").read_text())
    assert result["outcome"] == "succeeded" and result["dropped"] == 0
    meta = json.loads((directory / "meta.json").read_text())
    assert meta["task_id"] == "a" * 32
    for name in ("meta.json", "result.json", "state.jsonl", "imu.jsonl",
                 "scan.jsonl", "events.jsonl"):
        assert (directory / name).is_file()
    assert recorder.active is False
    assert recorder.state(v=2.0) is False


def test_disabled_recorder_is_inert(tmp_path):
    recorder = Recorder({"enabled": False}, tmp_path)
    assert recorder.start({"task_id": "b"}) is False
    assert recorder.state(v=1.0) is False
    assert recorder.stop("failed", "never started") is None
    assert not (tmp_path / "records").exists()


def test_full_queue_drops_rows_without_blocking(tmp_path):
    recorder = Recorder({}, tmp_path)
    recorder.directory = tmp_path
    recorder.queue = queue.Queue(maxsize=1)
    recorder.queue.put_nowait(("state", {}))
    assert recorder.state(v=1.0) is False
    assert recorder.dropped == 1
    assert recorder.event("blockers") is False


def test_prune_removes_oldest_runs(tmp_path):
    root = tmp_path / "records"
    root.mkdir()
    names = ("20260101_000000_aaaa", "20260102_000000_bbbb", "20260103_000000_cccc")
    for index, name in enumerate(names):
        run = root / name
        run.mkdir()
        (run / "state.jsonl").write_text("x" * 2048)
        os.utime(run, (index, index))
    recorder = Recorder({"max_run_mb": 0.001, "max_total_mb": 0.001}, tmp_path)
    assert recorder.prune() == 2
    assert [p.name for p in root.iterdir()] == ["20260103_000000_cccc"]


def test_restart_finishes_the_previous_run(tmp_path):
    recorder = Recorder({"flush_s": 0.2}, tmp_path)
    first = recorder.start({"task_id": "1" * 32})
    assert first is not None
    second = recorder.start({"task_id": "2" * 32})
    assert second is not None and second != first
    assert json.loads((first / "result.json").read_text())["outcome"] == "interrupted"
    recorder.stop("succeeded", None)
