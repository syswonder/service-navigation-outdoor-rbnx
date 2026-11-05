"""Thread-safe task ownership independent of HTTP and ROS action callbacks."""

from copy import deepcopy
from dataclasses import dataclass, field
import threading
import uuid
import json
from pathlib import Path

TERMINAL = frozenset({"succeeded", "failed", "canceled"})
STATES = (
    frozenset({"accepted", "planning", "navigating", "replanning", "canceling"})
    | TERMINAL
)


@dataclass
class Task:
    task_id: str
    goal: dict
    blocked_road_ids: list
    schema_version: int = 1
    frame_id: str = "map"
    scene_version: str | None = None
    route_version: str | None = None
    completed_sim_s: float | None = None
    completion_pose: dict | None = None
    state: str = "accepted"
    path_revision: int = 0
    execution_path: list = field(default_factory=list)
    reference_path: list = field(default_factory=list)
    distance_remaining_m: float | None = None
    reason: str | None = None
    cancel_requested: bool = False
    localization_grade: str | None = None
    arrival_tolerance_m: float | None = None
    road_offroad_cost: int | None = None
    map_id: str | None = None
    map_version: str | None = None
    origin_wgs84: dict | None = None
    intervals: list = field(default_factory=list)
    gait_status: dict | None = None


class TaskStore:
    def __init__(self, scene_version=None, route_version=None, journal=None):
        self.scene_version, self.route_version = scene_version, route_version
        self.lock = threading.RLock()
        self.tasks = {}
        self.current_id = None
        self.journal = Path(journal) if journal else None
        if self.journal and self.journal.exists():
            saved = json.loads(self.journal.read_text())
            self.tasks = {t["task_id"]: Task(**t) for t in saved["tasks"]}
            self.current_id = saved["current_id"]
            for task in self.tasks.values():
                if task.state not in TERMINAL:
                    task.state = "failed"
                    task.reason = "Service restarted; motion was not resumed"
            self._persist()

    def _persist(self):
        if self.journal:
            self.journal.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.journal.with_suffix(".tmp")
            temporary.write_text(
                json.dumps(
                    {
                        "current_id": self.current_id,
                        "tasks": [vars(t) for t in self.tasks.values()],
                    }
                )
            )
            temporary.replace(self.journal)

    def submit(self, goal, blocked=(), task_id=None):
        with self.lock:
            task = Task(
                task_id or uuid.uuid4().hex,
                deepcopy(goal),
                list(blocked),
                scene_version=self.scene_version,
                route_version=self.route_version,
            )
            if self.current_id:
                old = self.tasks[self.current_id]
                if old.state not in TERMINAL:
                    old.cancel_requested = True
                    old.state = "canceling"
            self.tasks[task.task_id] = task
            self.current_id = task.task_id
            # Keep active/recent history bounded, never evict an in-flight task.
            for key in list(self.tasks):
                if len(self.tasks) <= 200:
                    break
                if key != self.current_id and self.tasks[key].state in TERMINAL:
                    del self.tasks[key]
            self._persist()
            return self.snapshot(task.task_id)

    def snapshot(self, task_id=None, *, include_paths=True):
        with self.lock:
            key = task_id or self.current_id
            if key is None:
                return None
            if key not in self.tasks:
                raise KeyError("Task does not exist")
            return deepcopy(
                {
                    k: v
                    for k, v in vars(self.tasks[key]).items()
                    if include_paths or k not in ("execution_path", "reference_path")
                }
            )

    def update(self, task_id, **values):
        with self.lock:
            if task_id not in self.tasks:
                return False
            task = self.tasks[task_id]
            if task.state in TERMINAL:
                return False
            if task.cancel_requested and values.get("state") in {
                "accepted",
                "planning",
                "navigating",
                "replanning",
            }:
                return False
            if "state" in values and values["state"] not in STATES:
                raise ValueError("Unknown task state")
            for key, value in values.items():
                setattr(task, key, deepcopy(value))
            if "state" in values:
                self._persist()
            return True

    def cancel(self, task_id):
        with self.lock:
            task = self.tasks[task_id]
            if task.state not in TERMINAL:
                task.cancel_requested = True
                task.state = "canceling"
            self._persist()
            return self.snapshot(task_id)


@dataclass
class ProgressBudget:
    """Bound consecutive planning failures and controller failures without progress."""

    planner_failures: int = 0
    controller_failures: int = 0
    progress_point: tuple | None = None
    last_progress_s: float = 0.0
    planner_patience_s: float = 0.0
    controller_patience_s: float = 20.0
    planner_failed_since: float | None = None

    def planned(self, valid, stamp=None):
        self.planner_failures = 0 if valid else self.planner_failures + 1
        if valid:
            self.planner_failed_since = None
            return True
        if self.planner_patience_s > 0 and stamp is not None:
            if self.planner_failed_since is None:
                self.planner_failed_since = stamp
            return stamp - self.planner_failed_since < self.planner_patience_s
        return self.planner_failures < 3

    def controller_ended(self, stamp):
        self.controller_failures += 1
        # A moving obstacle can block a controller briefly. Match the existing
        # 20-second progress allowance before declaring repeated retries stalled.
        return (
            self.controller_failures < 3
            or stamp - self.last_progress_s < self.controller_patience_s
        )

    def moved(self, x, y, stamp):
        import math

        point = (x, y)
        if self.progress_point is None or math.dist(self.progress_point, point) >= 0.5:
            self.progress_point = point
            self.controller_failures = 0
            self.last_progress_s = stamp
