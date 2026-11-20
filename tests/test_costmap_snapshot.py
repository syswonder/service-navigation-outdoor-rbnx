"""Exercise the actual snapshot method on a host without ROS installed."""
import ast
import math
import threading
from pathlib import Path
from types import SimpleNamespace as S
import pytest


@pytest.mark.parametrize('stale',[False,True])
def test_local_path_uses_its_frame_and_expired_transforms_hide_overlays(stale):
    source=Path(__file__).parents[1]/'outdoor_nav/executor.py'
    cls=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='Executor')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='costmap_snapshot')
    scope={'__name__':'outdoor_nav.executor','__package__':'outdoor_nav',
           'rclpy':S(time=S(Time=lambda:None)), 'time':S(monotonic=lambda:10.),
           'math':math,'yaw':lambda q:math.pi/2}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
    header=S(frame_id='odom',stamp=S(sec=10,nanosec=0))
    grid=S(header=header,info=S(width=2,height=2,resolution=1.,origin=S(
        position=S(x=0.,y=0.),orientation=S(x=0.,y=0.,z=0.,w=1.))),data=[0,100,-1,99])
    def lookup(target,source,*_):
        return S(header=S(stamp=S(sec=6 if stale else 10,nanosec=0)),
                 transform=S(translation=S(x=2.,y=3.),rotation=S()))
    node=S(state_lock=threading.RLock(),local_map=grid,global_map=grid,
           stats={'local_map':10.},base_frame='base_link',tf=S(lookup_transform=lookup),
           get_clock=lambda:S(now=lambda:S(nanoseconds=10_000_000_000)),
           tasks=S(snapshot=lambda:{'execution_path':[{'x':1.,'y':0.},{'x':2.,'y':0.}]}),costmap_view_cache={})
    value=scope['costmap_snapshot'](node,'local')
    assert value['frame_id']=='odom' and not value['stale']
    if stale:
        assert value['robot'] is None and not value['path']
        assert set(value['overlay_warnings'])=={'robot_transform_unavailable','path_transform_unavailable'}
    else:
        assert value['path'][0]=={'x':2.,'y':4.}
        assert value['robot']['x']==2. and not value['overlay_warnings']


def test_cloud_view_parse_and_voxel_downsample():
    import numpy as np
    import sys
    from pathlib import Path as _P
    from types import SimpleNamespace as S
    sys.path.insert(0, str(_P(__file__).parents[1]))
    from outdoor_nav.core.cloud_view import parse_xyz, voxel_downsample

    count = 400
    points = np.random.default_rng(0).uniform(-5, 5, (count, 3)).astype(np.float32)
    raw = np.zeros((count, 12), dtype=np.uint8)
    for i, name in enumerate(("x", "y", "z")):
        raw[:, i * 4:(i + 1) * 4] = points[:, i].copy().view(np.uint8).reshape(-1, 4)
    msg = S(fields=[S(name="x", offset=0, datatype=7),
                    S(name="y", offset=4, datatype=7),
                    S(name="z", offset=8, datatype=7)],
            is_bigendian=False, width=count, height=1, point_step=12,
            data=raw.tobytes(), header=S(frame_id="base_link", stamp=S(sec=1, nanosec=0)))
    parsed = parse_xyz(msg)
    assert parsed.shape == (count, 3) and np.allclose(parsed, points, atol=1e-6)
    down = voxel_downsample(parsed, 0.06, 200)
    assert 0 < len(down) <= 200
    # Voxel cells are unique at the downsample size.
    keys = np.floor(down / 0.06).astype(np.int64)
    assert len(np.unique(keys, axis=0)) == len(down)
    # A malformed cloud is rejected, not guessed at.
    for bad in (S(fields=[S(name="x", offset=0, datatype=7)], is_bigendian=False,
                  width=1, height=1, point_step=4, data=b"\0" * 4,
                  header=S(frame_id="base_link", stamp=S(sec=1, nanosec=0))),
                S(fields=[S(name="x", offset=0, datatype=7), S(name="y", offset=4, datatype=7),
                          S(name="z", offset=8, datatype=7)], is_bigendian=True,
                  width=1, height=1, point_step=12, data=b"\0" * 12,
                  header=S(frame_id="base_link", stamp=S(sec=1, nanosec=0)))):
        try:
            parse_xyz(bad)
            assert False, "expected ValueError"
        except ValueError:
            pass


def test_cloud_snapshot_serves_fresh_cache_only():
    source = Path(__file__).parents[1] / 'outdoor_nav/runtime.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'OutdoorExecutor')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'cloud_snapshot')
    scope = {'__name__': 'outdoor_nav.runtime', '__package__': 'outdoor_nav',
             'time': S(monotonic=lambda: 10.), 'math': math,
             'threading': threading}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    payload = {'ok': True, 'count': 3, 'points_b64': 'AAAA'}
    node = S(state_lock=threading.RLock(), cloud_preview=payload, cloud_preview_at=9.)
    assert scope['cloud_snapshot'](node) == payload
    stale = S(state_lock=threading.RLock(), cloud_preview=payload, cloud_preview_at=4.)
    try:
        scope['cloud_snapshot'](stale)
        assert False, "expected 503 for a stale cloud"
    except Exception as error:
        assert 'stale' in str(error)
    empty = S(state_lock=threading.RLock(), cloud_preview=None, cloud_preview_at=-math.inf)
    try:
        scope['cloud_snapshot'](empty)
        assert False, "expected 503 before the first cloud"
    except Exception as error:
        assert 'Waiting' in str(error)
