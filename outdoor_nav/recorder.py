"""Execution-scoped sensor recording; no ROS imports so it stays testable.

Callbacks hand over plain values only; encoding, buffering and disk I/O happen
on a dedicated thread so the control loop never waits for the filesystem.
"""

import json
import math
import queue
import re
import shutil
import threading
import time
from pathlib import Path

DEFAULTS = {
    "enabled": True,
    "dir": "records",
    "state_hz": 10.0,
    "scan_hz": 2.0,
    "imu_enabled": True,
    "imu_topic": "/wit_10axis/data",
    "max_run_mb": 64.0,
    "max_total_mb": 2048.0,
    "flush_s": 1.0,
}
STREAMS = ("state", "imu", "scan", "events")
STOP = object()
_TOPIC = re.compile(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*")


def validate(value):
    if value is None:
        value = {}
    if not isinstance(value, dict):
        raise ValueError("record must be an object")
    unknown = set(value) - set(DEFAULTS)
    if unknown:
        raise ValueError("Unknown record keys: " + str(sorted(unknown)))
    config = {**DEFAULTS, **value}
    for key in ("enabled", "imu_enabled"):
        if type(config[key]) is not bool:
            raise ValueError(key + " must be boolean")
    for key, low, high in (
        ("state_hz", 1.0, 20.0),
        ("scan_hz", 0.5, 10.0),
        ("flush_s", 0.2, 5.0),
    ):
        number = config[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)):
            raise ValueError(key + " must be a number")
        if not math.isfinite(number) or not low <= number <= high:
            raise ValueError(key + " must be between %s and %s" % (low, high))
    for key in ("max_run_mb", "max_total_mb"):
        number = config[key]
        if isinstance(number, bool) or not isinstance(number, (int, float)):
            raise ValueError(key + " must be a number")
        if not math.isfinite(number) or not 0 < number <= 1024 * 1024:
            raise ValueError(key + " must be a positive size in MiB")
    if config["max_run_mb"] > config["max_total_mb"]:
        raise ValueError("max_run_mb must not exceed max_total_mb")
    name = config["dir"]
    if not isinstance(name, str) or not name:
        raise ValueError("record.dir must be a non-empty string")
    path = Path(name)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError("record.dir must stay inside run_dir")
    if not isinstance(config["imu_topic"], str) or not _TOPIC.fullmatch(
        config["imu_topic"]
    ):
        raise ValueError("record.imu_topic must be an absolute ROS topic")
    return config


def plain(value):
    """Replace non-finite floats with null so json.dumps never emits NaN."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def _size(path):
    try:
        return sum(f.stat().st_size for f in path.rglob("*") if f.is_file())
    except OSError:
        return 0


class Recorder:
    """One directory per task, one JSONL file per stream, one writer thread."""

    def __init__(self, config, run_dir):
        self.config = validate(config)
        self.root = Path(run_dir) / self.config["dir"]
        self.queue = queue.Queue(maxsize=4096)
        self.files = {}
        self.directory = None
        self.thread = None
        self.dropped = 0
        self.written = 0
        self.failed = None
        self.capped = False
        self.last_flush = 0.0
        self.lock = threading.Lock()

    @property
    def enabled(self):
        return bool(self.config["enabled"])

    @property
    def active(self):
        return self.directory is not None and self.failed is None

    def start(self, meta):
        if not self.enabled:
            return False
        self.stop("interrupted", "Recording restarted before finishing")
        with self.lock:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            task = str(meta.get("task_id") or "unknown")[:8]
            directory = self.root / (stamp + "_" + task)
            suffix = 1
            while directory.exists():
                directory = self.root / (stamp + "_" + task + "_" + str(suffix))
                suffix += 1
            directory.mkdir(parents=True, exist_ok=True)
            self._write_json(
                directory / "meta.json",
                {
                    **plain(meta),
                    "started_at_unix_s": time.time(),
                    "started_at_mono_s": time.monotonic(),
                    "config": {k: v for k, v in self.config.items() if k != "dir"},
                },
            )
            self.files = {
                name: (directory / (name + ".jsonl")).open("a", encoding="utf-8")
                for name in STREAMS
            }
            self.directory = directory
            self.dropped = self.written = 0
            self.failed = None
            self.capped = False
            self.last_flush = time.monotonic()
            self.thread = threading.Thread(
                target=self._run, name="outdoor-recorder", daemon=True
            )
        self.thread.start()
        return directory

    def state(self, **fields):
        return self._offer("state", fields)

    def scan(self, ranges):
        return self._offer("scan", {"ranges": ranges})

    def imu(self, rows):
        return self._offer("imu", {"samples": rows})

    def event(self, kind, **fields):
        return self._offer("events", {"event": kind, **fields})

    def stop(self, outcome, reason):
        directory, thread = self.directory, self.thread
        if directory is None:
            return None
        self.event(
            "stopped",
            outcome=outcome,
            reason=reason,
            dropped=self.dropped,
            bytes=self.written,
            capped=self.capped,
        )
        try:
            self.queue.put(STOP, timeout=2.0)
        except queue.Full:
            pass
        if thread is not None:
            thread.join(timeout=10.0)
        with self.lock:
            self._close()
            self._write_json(
                directory / "result.json",
                plain(
                    {
                        "outcome": outcome,
                        "reason": reason,
                        "finished_at_unix_s": time.time(),
                        "duration_s": time.time() - self._started(directory),
                        "bytes": self.written,
                        "dropped": self.dropped,
                        "capped": self.capped,
                        "failed": self.failed,
                    }
                ),
            )
            self.directory, self.thread, self.failed = None, None, None
        self.prune()
        return directory

    def prune(self):
        if not self.root.is_dir():
            return 0
        limit = self.config["max_total_mb"] * 1024 * 1024
        entries = sorted(
            (p for p in self.root.iterdir() if p.is_dir()),
            key=lambda p: p.stat().st_mtime,
        )
        total = sum(_size(p) for p in entries)
        removed = 0
        while total > limit and len(entries) > 1:
            oldest = entries.pop(0)
            total -= _size(oldest)
            shutil.rmtree(oldest, ignore_errors=True)
            removed += 1
        return removed

    def _started(self, directory):
        try:
            return json.loads((directory / "meta.json").read_text()).get(
                "started_at_unix_s", time.time()
            )
        except (OSError, ValueError):
            return time.time()

    def _write_json(self, path, value):
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(value, allow_nan=False, indent=2))
        temporary.replace(path)

    def _offer(self, name, fields):
        if not self.active:
            return False
        record = plain({**fields, "t": time.time(), "mono": time.monotonic()})
        try:
            self.queue.put_nowait((name, record))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        try:
            while True:
                try:
                    item = self.queue.get(timeout=0.1)
                except queue.Empty:
                    if time.monotonic() - self.last_flush >= self.config["flush_s"]:
                        self._flush()
                    continue
                if item is STOP:
                    return
                name, record = item
                if self.capped and name in ("imu", "scan"):
                    continue
                handle = self.files.get(name)
                if handle is None:
                    continue
                line = json.dumps(record, allow_nan=False, separators=(",", ":"))
                handle.write(line + "\n")
                self.written += len(line) + 1
                if (
                    not self.capped
                    and self.written > self.config["max_run_mb"] * 1024 * 1024
                ):
                    self.capped = True
                    handle.write(
                        json.dumps(
                            {
                                "event": "capped",
                                "limit_mb": self.config["max_run_mb"],
                                "t": time.time(),
                            }
                        )
                        + "\n"
                    )
                if time.monotonic() - self.last_flush >= self.config["flush_s"]:
                    self._flush()
        except OSError as error:
            self.failed = str(error)
        finally:
            self._flush()

    def _flush(self):
        for handle in self.files.values():
            try:
                handle.flush()
            except (OSError, ValueError):
                pass
        self.last_flush = time.monotonic()

    def _close(self):
        for handle in self.files.values():
            try:
                handle.close()
            except (OSError, ValueError):
                pass
        self.files = {}
