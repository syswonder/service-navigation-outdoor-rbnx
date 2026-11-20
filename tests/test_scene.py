"""Scene consumes current map routes without any legacy bundle or planner."""
import copy
import pytest
from outdoor_nav.scene import ApiError, SceneNavigation

MAP={'map_id':'beijing-field','version':'a'*16,'origin_wgs84':{'latitude':39.9042,'longitude':116.4074}}


class Execution:
    def __init__(self):
        self.value={'ready':True,'localization_grade':'fixed','not_ready_reasons':[],
            'pose':{'x':0.,'y':0.,'age_s':0.01},'coordinate_reference':{
                **MAP,'frame_id':'map','axes':'east_north','reference_point':'base_link',
                'map_offset_xy_m':[0.,0.],'antenna_xy_m':[0.,0.]}}
        self.received=[]
        self.lose_reply=False

    def status(self,run_id=''):return copy.deepcopy(self.value)

    def follow_reference(self,payload):
        self.received.append(copy.deepcopy(payload))
        if self.lose_reply:
            self.lose_reply=False
            raise TimeoutError('reply lost after acceptance')
        return payload['request_id']

    def cancel(self,run_id):return {'task_id':run_id,'state':'canceling'}


@pytest.fixture
def nav(tmp_path):
    value=SceneNavigation(run_dir=tmp_path,execution=Execution(),execution_enabled=True)
    value.bind_map(MAP)
    return value


def route():
    return {**MAP,'frame_id':'map','reference_path':[{'x':0.,'y':0.},{'x':20.,'y':0.}],
        'goal':{'x':20.,'y':0.},'intervals':[{'start_m':0.,'end_m':20.,'gait':'stairs','gait_code':0x1003}]}


@pytest.mark.parametrize('change',[{'map_id':'old-map'},{'version':'c'*16},
    {'origin_wgs84':{'latitude':30.,'longitude':120.}},{'map_offset_xy_m':[1.,0.]},
    {'antenna_xy_m':[1.,0.]},{'reference_point':'other'}])
def test_live_coordinate_mismatch_refuses_execution(nav,change):
    nav.execution.value['coordinate_reference'].update(change)
    assert not nav.status()['ready']
    with pytest.raises(ApiError):nav.submit_reference(route(),'c'*32,{})
    assert not nav.execution.received


def test_lost_reply_retries_original_request_after_restart(nav):
    nav.execution.lose_reply=True
    with pytest.raises(TimeoutError):nav.submit_reference(route(),'d'*32,{'goal':20})
    sent=nav.execution.received[-1]
    nav.execution.value['ready']=False
    restarted=SceneNavigation(run_dir=nav.run_dir,execution=nav.execution,execution_enabled=True)
    restarted.bind_map(MAP)
    restarted.submit_reference(route(),'d'*32,{'goal':20})
    assert nav.execution.received[-1]==sent
    with pytest.raises(ApiError):restarted.submit_reference(route(),'d'*32,{'goal':21})


def test_disabled_profile_and_active_navigation_block_execution_or_map_change(nav):
    nav.execution_enabled=False
    assert not nav.status()['ready']
    with pytest.raises(ApiError):nav.submit_reference(route(),'e'*32,{})
    nav.execution_enabled=True
    nav.execution.value['task']={'state':'navigating'}
    with pytest.raises(ValueError,match='stopped'):nav.bind_map(MAP)


def test_retry_storage_bound_keeps_existing_ids(nav, monkeypatch):
    from outdoor_nav import scene
    monkeypatch.setattr(scene,'MAX_REQUEST_RECORDS',1)
    nav.submit_reference(route(),'f'*32,{})
    nav.submit_reference(route(),'f'*32,{})
    with pytest.raises(ApiError,match='storage is full'):
        nav.submit_reference(route(),'a'*32,{})
    assert len(list(nav.run_dir.glob('*.json')))==1


def test_costmap_proxy_works_in_check_mode_and_preserves_layer_failures():
    from types import SimpleNamespace
    from outdoor_nav.http import serve
    calls=[]
    def costmap(layer, standard=False):
        calls.append((layer,standard))
        if layer=='local':raise ApiError(503,'Waiting for local costmap')
        return {'frame_id':'map','image':'actual-grid','stale':True,'age_s':4.}
    server=serve(SimpleNamespace(costmap_snapshot=costmap),port=0)
    try:
        reader=SceneNavigation(execution_enabled=False,navigation_http_url=f'http://127.0.0.1:{server.server_port}')
        assert reader.costmap('global')['image']=='actual-grid'
        with pytest.raises(ApiError,match='Waiting for local'):reader.costmap('local')
        with pytest.raises(ApiError) as error:reader.costmap('../status')
        assert error.value.status==404
        assert calls==[('global',False),('local',False)]
    finally:
        server.shutdown();server.server_close()
