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
