"""Recovery budget tests execute the production coroutine without ROS imports."""
import ast
import asyncio
from pathlib import Path
from types import SimpleNamespace as S
import pytest


def result_method(clock):
    source=Path(__file__).parents[1]/'outdoor_nav/executor.py'
    cls=next(n for n in ast.parse(source.read_text()).body
             if isinstance(n,ast.ClassDef) and n.name=='Executor')
    method=next(n for n in cls.body if isinstance(n,ast.AsyncFunctionDef) and n.name=='result')
    scope={'time':S(monotonic=lambda:clock[0])}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),scope)
    return scope['result']


@pytest.mark.parametrize('finished_during_recovery',[False,True])
def test_action_recovery_wait_does_not_consume_request_budget(finished_during_recovery):
    clock=[0.]; done=[False]; ready=[False]
    async def recover(task):
        clock[0]+=14.  # Below the separate recovery budget, above action's 10s.
        ready[0]=True
        done[0]=finished_during_recovery
    async def pause():
        clock[0]+=.1
        done[0]=True
    node=S(interrupted=lambda task:False,status=lambda include:{'ready':ready[0]},
           wait_for_ready_pose=recover,pause=pause)
    future=S(done=lambda:done[0],result=lambda:'accepted')
    assert asyncio.run(result_method(clock)(node,future,'task'))=='accepted'


def test_action_still_times_out_when_ready_but_no_response():
    clock=[0.]
    async def pause():clock[0]+=1.
    node=S(interrupted=lambda task:False,status=lambda include:{'ready':True},pause=pause)
    future=S(done=lambda:False)
    with pytest.raises(RuntimeError,match='timed out'):
        asyncio.run(result_method(clock)(node,future,'task',timeout=2.))
