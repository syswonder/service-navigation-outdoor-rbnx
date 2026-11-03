"""Reuses the original browser scenarios with the service as sole executor.

Docker-only; external baseline supplies virtual RTK/WIT and the business UI.
No baseline navigation executor or watchdog is launched.
"""

import os
from pathlib import Path
import sys

if not Path("/.dockerenv").exists():
    raise SystemExit("Docker only")
BASE = Path(os.environ.get("OUTDOORNAV_BASELINE", "/baseline"))
SERVICE = Path(__file__).resolve().parents[1]
MANUAL = "--manual" in sys.argv
if MANUAL:
    import signal

    def startup_stop(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, startup_stop)
SDK_PYTHON = os.environ.get(
    "OUTDOOR_SERVICE_PYTHON", "/tmp/service-sdk-venv/bin/python"
)
text = (BASE / "tests/isolated_task_flow.py").read_text()
text = text.replace(
    "ROOT=Path(__file__).resolve().parents[1]", f"ROOT=Path({str(BASE)!r})"
)
text = text.replace(
    "OUTPUT=ROOT/'artifacts/validation'/('isolated-task-flow-'+time.strftime('%Y%m%d'))",
    f"OUTPUT=Path({str(SERVICE)!r})/'rbnx-build/validation'/('isolated-task-flow-'+time.strftime('%Y%m%d'))",
)
a = text.index("launch=f'''")
b = text.index("(RUN/'isolated.launch.py').write_text(launch)", a)
replacement = """from outdoor_nav.configuration import DEFAULTS as POLICY_DEFAULTS
runtime = {'params_file':str(Path('/service/config/nav2_params.example.yml')),
    'policy':{key:config.get(key,value) for key,value in POLICY_DEFAULTS.items()},
    'global_grid':{'origin':bundle['grid']['origin'], 'width':bundle['grid']['width']*bundle['grid']['resolution'],
                   'height':bundle['grid']['height']*bundle['grid']['resolution'], 'resolution':bundle['grid']['resolution']},
    'topic_remap':{'cloud':config['cloud_topic'],'scan':'/outdoor/scan','odom':'/outdoor/odom',
                  'localization':'/outdoor/localization','chassis':'/outdoor/chassis_status',
                  'tf':'/outdoor/tf','tf_static':'/tf_static'},
    'velocity_output_topic':COMMAND,'run_dir':str(RUN/'service'),'http_port':19975}
(RUN/'service').mkdir()
(RUN/'runtime.json').write_text(json.dumps(runtime,indent=2))
launch=f\'''from launch import LaunchDescription
from launch.actions import GroupAction, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
import sys

def generate_launch_description():
    return LaunchDescription([
      GroupAction(actions=[SetRemap(src='/tf',dst='/outdoor/tf'),
        Node(package='outdoornav_ros',executable='lynx_localizer',parameters=[{{
            'bundle':'{ROOT}/scenes/north_campus/generated/bundle.json',
            'robot_config':'{RUN}/synthetic-only.json','use_sim_time':False,'diagnostic':False}}],output='screen')]),
      IncludeLaunchDescription(PythonLaunchDescriptionSource('/service/outdoor_nav/service.launch.py'),
            launch_arguments={{'runtime':'{RUN}/runtime.json'}}.items()),
      ExecuteProcess(cmd=['{SDK_PYTHON}','/service/tests/local_rpc.py','--runtime','{RUN}/runtime.json'],output='screen'),
      ExecuteProcess(cmd=['{SDK_PYTHON}','/service/tests/business_facade.py',
        '--bundle','{ROOT}/scenes/north_campus/generated/bundle.json',
        '--config','{RUN}/synthetic-only.json','--places','{ROOT}/scenes/north_campus/places.json',
        '--web','{ROOT}/web/dist'],output='screen')])
\'''
"""
text = text[:a] + replacement + text[b:]
# Obstacles must fall inside the current projected height band (+0.2 m mount).
text = text.replace("),.5) for i in range(720)", "),0.) for i in range(720)")
# Ensure cleanup catches nested launch, facade and Nav2 subprocesses on all failures.
text = text.replace(
    "env={**os.environ,'OUTDOORNAV_WEB_ROOT'",
    "env={**os.environ,'OUTDOOR_SERVICE_INSTANCE':str(RUN/'service'),'OUTDOORNAV_WEB_ROOT'",
)
if MANUAL:
    begin = text.index("    browser_log=(RUN/'browser.log').open('w')")
    end = text.index("finally:\n    # Request cancellation", begin)
    manual = """    manual_stop=threading.Event()
    def request_stop(*_): manual_stop.set()
    signal.signal(signal.SIGINT,request_stop)
    signal.signal(signal.SIGTERM,request_stop)
    print('MANUAL_WEB_READY http://127.0.0.1:19865/ '+str(RUN),flush=True)
    while not manual_stop.wait(.2):
        if backend.poll() is not None:raise RuntimeError('Manual test stack exited')
        if virtual.isolation_failure:raise RuntimeError(virtual.isolation_failure)
    report['manual_web']=True
    report['passed']=virtual.isolation_failure is None
    report['events']=virtual.events
    report['final_scope']=virtual.snapshot()
    report['final_status']=http('/api/status')
"""
    text = text[:begin] + manual + text[end:]
    (SERVICE / "rbnx-build/manual.pid").write_text(str(os.getpid()))

sys.argv = [str(BASE / "tests/isolated_task_flow.py"), "--rtk-imu", "--field-policy"]
exec(
    compile(text, str(BASE / "tests/isolated_task_flow.py"), "exec"),
    {
        "__name__": "__main__",
        "__file__": str(BASE / "tests/isolated_task_flow.py"),
        "SDK_PYTHON": SDK_PYTHON,
    },
)
