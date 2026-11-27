# 执行期数据记录

导航执行时把定位、IMU、控制指令、底盘反馈和 2D 扫描写进 JSONL，测试结束后取回来复盘。

## 什么时候记录

点击执行者、任务进入规划时开始，任务成功、失败或取消时停止。规划前取消不会产生目录，服务空闲时不写盘。

## 存在哪

```
<run_dir>/records/<YYYYmmdd_HHMMSS>_<任务ID前8位>/
```

山猫上 `run_dir` 是 `/home/nvidia/outdoorNav-workspace/robot-deep_robotics-lynx_s10/outdoor/run/navigation`，所以记录在：

```
/home/nvidia/outdoorNav-workspace/robot-deep_robotics-lynx_s10/outdoor/run/navigation/records/
```

`run/` 不在同步范围内，部署覆盖不会动到记录。

## 每个文件是什么

| 文件 | 内容 | 频率 |
|---|---|---|
| `meta.json` | 任务 ID、目标、地图、参考路线长度、开始时间、生效配置 | 一次 |
| `state.jsonl` | 定位、里程计、三段指令（想发的/安全层改过的/发给底盘的）、底盘实际速度、步态、停车原因 | 10 Hz |
| `imu.jsonl` | 加速度、角速度、姿态（含航向） | IMU 原始频率（默认 100 Hz） |
| `scan.jsonl` | 2D 扫描，单位米，保留两位小数 | 2 Hz |
| `events.jsonl` | 停车原因变化、步态切换、丢行统计 | 事件 |
| `result.json` | 结束状态和原因；**这个文件缺失说明服务异常退出** | 一次 |

每行都带 `t`（Unix 秒）和 `mono`（单调时间）。复盘对齐用 `mono`，它不受系统对时影响。

`state.jsonl` 示例：

```json
{"t":1764138601.53,"mono":1723.42,"kind":"state","task":"navigating",
 "pose":{"x":12.3,"y":-4.5,"yaw":1.2},"odom":{"x":12.1,"y":-4.4,"yaw":1.19},"heading":1.19,
 "cmd":{"v":0.35,"w":0.05},"safe":{"v":0.35,"w":0.05},"out":{"v":0.30,"w":0.05},
 "actual":{"v":0.29,"w":0.04},"chassis":{"gait":12290},"blockers":[]}
```

`pose` 是定位输出，`odom` 是里程计，`heading` 是融合后的航向（弧度）；三者对比能区分抖动来自定位、航向还是底盘。

## 不记录什么

原始 3D 点云和深度图不记录（每次任务 15 分钟分别是几十 GB 和十几 GB）。需要时另外开一个 `ros2 bag` 进程，和导航进程隔离，不要加进这个功能。

## 配置

导航服务配置里的 `record` 段，默认值：

```yaml
record:
  enabled: true
  dir: records           # 相对 run_dir
  state_hz: 10
  scan_hz: 2
  imu_enabled: true
  imu_topic: /wit_10axis/data
  max_run_mb: 64         # 单次任务上限，超了先停扫描和 IMU
  max_total_mb: 2048     # 目录总量上限，超了删最老的
  flush_s: 1
```

15 分钟一趟大约 20 MB。写盘在独立线程里做，不 fsync、不阻塞控制回调。

## 取回和复盘

```bash
scp -r "nvidia@10.29.1.4:/home/nvidia/outdoorNav-workspace/robot-deep_robotics-lynx_s10/outdoor/run/navigation/records/2026*" ./records/
python3 scripts/summarize_record.py records/<目录> --csv run.csv
```

脚本输出时长、轨迹长度、定位跳变次数、最大/平均速度、航向抖动、指令跟踪偏差、停车原因排行；`--csv` 导出 10 Hz 对齐表，可以直接拖进 Excel 画图。
