import asyncio
from types import SimpleNamespace as S

import numpy as np
import pytest

pytest.importorskip("rclpy")
from outdoor_nav.descent_execution import RangerDescent
from outdoor_nav.core.ground_support import CELL, SIZE, SupportMask
from outdoor_nav.ground_support_node import SupportFeed
from outdoor_nav.terrain_execution import rotation_clear


def test_known_hazard_starts_a_fresh_bounded_survey(monkeypatch):
    now = [1.]
    node = S(guard_at=1., guard={"blocked_reasons": ["ground_hazard"]},
             config={"command_timeout_s": .5})
    ranger = S(node=node, feed=S(fresh=lambda: True), takeover_wait=None, last_handoff=None)
    monkeypatch.setattr("outdoor_nav.descent_execution.time.monotonic", lambda: now[0])
    assert not RangerDescent.should_take_over(ranger, "task")
    now[0] += .36
    assert RangerDescent.should_take_over(ranger, "task")
    assert not RangerDescent.should_take_over(ranger, "new task")
    node.guard["blocked_reasons"] = ["ground_unobserved"]
    assert not RangerDescent.should_take_over(ranger, "new task")
    node.guard["blocked_reasons"] = ["ranger_crest"]
    now[0] += 1.
    assert not RangerDescent.should_take_over(ranger, "new task")
    node.guard_at = now[0]
    node.guard = {"blocked_reasons": [], "ranger_terrain_ahead": "ground_hazard"}
    assert not RangerDescent.should_take_over(ranger, "new task")
    now[0] += .36
    assert RangerDescent.should_take_over(ranger, "new task")


def test_landing_settles_early_warning_but_keeps_real_guard_stops(monkeypatch):
    now = [1.]
    node = S(guard_at=1., guard={"ranger_terrain_ahead": "ground_hazard"},
             config={"command_timeout_s": .5})
    ranger = S(node=node, feed=S(fresh=lambda: True), takeover_wait=None, last_handoff=("task", 0.))
    monkeypatch.setattr("outdoor_nav.descent_execution.time.monotonic", lambda: now[0])
    assert not RangerDescent.should_take_over(ranger, "task")
    assert ranger.takeover_wait is None
    node.guard["blocked_reasons"] = ["ground_hazard"]
    assert not RangerDescent.should_take_over(ranger, "task")
    now[0] += .36
    assert RangerDescent.should_take_over(ranger, "task")
    node.guard = {"ranger_terrain_ahead": "ground_hazard"}
    now[0] = node.guard_at = 3.1
    ranger.takeover_wait = None
    assert not RangerDescent.should_take_over(ranger, "task")
    now[0] += .36
    assert RangerDescent.should_take_over(ranger, "task")


def test_landing_requires_flat_support_under_and_ahead_of_the_vehicle():
    start = -(SIZE//2 + .5)*CELL
    sample = SupportMask(np.ones((SIZE, SIZE), dtype=bool), (start, start), CELL, 1., .6)
    ranger = S(node=S(config={"footprint_radius_m": .6}))
    cols = start + (np.arange(SIZE) + .5)*CELL
    terrain = -.16*np.clip(cols, -2., 0.) - .32
    heights = np.broadcast_to(terrain, sample.safe.shape)
    assert not RangerDescent.landing_ready(ranger, sample, heights, (.18, 0.), -.32, 0., 0.)
    assert RangerDescent.landing_ready(ranger, sample, heights, (.9, 0.), -.32, 0., 0.)
    assert not RangerDescent.landing_ready(ranger, sample, heights, (.9, 0.), 0., 0., 0.)


def test_ranger_rotation_uses_live_costmap_and_actual_footprint(monkeypatch):
    monkeypatch.setattr("outdoor_nav.terrain_execution.time.monotonic", lambda: 1.)
    grid = S(header=S(frame_id="odom"),
             info=S(origin=S(position=S(x=-1., y=-1.)), resolution=.1, width=20, height=20),
             data=[0] * 400)
    pose = S(transform=S(translation=S(x=0., y=0.)))
    node = S(local_map=grid, stats={"local_map": 1.},
             config={"sensor_timeout_s": 1., "footprint_radius_m": .6},
             tf=S(lookup_transform=lambda *args: pose))
    assert rotation_clear(node)
    grid.data[10*20 + 15] = -1
    assert not rotation_clear(node)
    ground = S(ground_at=lambda xy: True)
    assert rotation_clear(node, ground=ground)
    grid.data[10*20 + 15] = 100
    assert not rotation_clear(node, ground=ground)


def test_handoff_uses_the_ordinary_ground_feed():
    sample = S(stamp=2.)
    normal = S(corridor=lambda xy, path: True, level=lambda xy, heights: -.32)

    async def fresh_survey(*args, **kwargs):
        return sample, object(), (0., 0.), -.32

    ranger = S(
        node=S(config={"ranger_depth_timeout_s": .1}),
        wait_support=fresh_survey,
        landing_ready=lambda *args: True,
        normal_feed=S(fresh=lambda: normal, elevation=lambda ground: object()),
    )
    assert asyncio.run(RangerDescent.verify_landing(ranger, "task", 1., 0., 0.))
    observed = [normal]
    ranger.normal_feed.fresh = lambda: observed[-1]
    normal.corridor = lambda xy, path: False

    async def expire(delay):
        await asyncio.sleep(.11)

    ranger.node.pause = expire
    assert not asyncio.run(RangerDescent.verify_landing(ranger, "task", 1., 0., 0.))


def test_survey_and_ordinary_ground_subscribe_to_distinct_streams():
    topics = []
    node = S(create_subscription=lambda kind, topic, callback, qos: topics.append(topic))
    SupportFeed(node, {}, None, with_heights=True)
    ordinary = set(topics)
    topics.clear()
    SupportFeed(node, {}, None, with_heights=True, survey=True)
    assert len(ordinary) == len(topics) == 2
    assert not ordinary.intersection(topics)
