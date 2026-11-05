# Outdoor navigation service for Robonix

把 outdoorNav 的 **Nav2 执行与避障链**封装为 Robonix service，支持外部提供的整条参考路线，不重新实现道路业务规划。

```text
业务层（网页、地点、道路权重） ── follow_reference ──► 此 service
外部点云、定位/速度、TF ──► 投影 → 动态代价图 → NavFn → DWB → 平滑 → 安全守卫
                                                               │
                                          指定速度输出 → 外部底盘桥
```

不自带地图场景、网页、RTK/IMU/雷达驱动，不使用山猫 `/ODOM`，不自动站起或切使用模式。明确标注的路线区段可以请求切换导航步态，由外部原语校验并转发。
默认速度输出是 **`/robonix/nomotion/outdoor_cmd_vel`**，初始未激活且 `motion_enabled=false`。
本轮完成隔离验证；真实上下楼梯和现场到点停车仍待验收。

## 仓库结构

- `package_manifest.yaml`、`package_manifest.jetson-native.yaml`：标准 service 包入口。
- `capabilities/`：包内参考路线、状态、取消、停车和代价图契约；Driver 显式声明命名空间生命周期契约，兼容参考服务使用的 SDK，不再并列声明第二种 Driver。
- `config.spec`、`config/`：部署配置、原 DWB/NavFn 参数基线与部署条目示例。
- `outdoor_nav/atlas_bridge.py`：Atlas 依赖发现，Driver 生命周期，gRPC/MCP 能力。
- `outdoor_nav/runtime.py`、`executor.py`：唯一执行 owner、滚动窗口、停车确认和状态。
- `outdoor_nav/guard.py`：独立速度/扫描/定位看门狗，保留原链路的失效归零。
- `outdoor_nav/service.launch.py`：仅启动投影、执行器、守卫及 Nav2；不启动外部传感器与定位。
- `tests/`：冻结算法对照、参数/请求/生命周期、Scene/gRPC 接入和隔离 Nav2 楼梯测试。
- `docs/upstream-snapshot.json`：被提取源码的 SHA-256；`tests/upstream_executor.py.txt` 是只用于对比的冻结参照。

参考服务结构：[syswonder/service-navigation-rbnx](https://github.com/syswonder/service-navigation-rbnx)。不直接复制其室内静态地图依赖、控制器插件、停止脚本或传感器默认参数。

## 构建与 Robonix 部署

运行目标：Ubuntu 22.04 / ROS 2 Humble，原生 Jetson 或本地 x86 ROS 环境。
需要 Robonix CLI/API（含 codegen，代码生成器须支持包内 capabilities）、Nav2 planner/controller/velocity smoother/lifecycle manager、pointcloud_to_laserscan、NumPy、PyYAML、Pillow。

```bash
source /opt/ros/humble/setup.bash
bash scripts/build.sh
```

启动解释器需安装 `robonix-api` 与本包 `[robonix]` 可选依赖，固定 `mcp>=1.27,<2.0.dev0` 和 `fastmcp>=3,<4`，避免 SDK 的 v1 FastMCP 接口被 mcp 2.x 替换；可用 `OUTDOOR_SERVICE_PYTHON` 指定 ROS-compatible Python/venv。

构建只生成 gRPC/MCP bindings，**不生成 ROS IDL overlay**。将 `config/deployment.example.yaml` 条目复制到机器人部署仓库；将 `config/nav2_params.example.yml` 复制为部署侧 `config/outdoor_nav2.yaml` 并检查部署参数，再通过该部署 manifest 的 `rbnx build` / `rbnx boot` 启动。不要直接使用占位 provider 名称当作现成部署。

`start.sh` 只起 bridge；Driver INIT 才绑定依赖并起执行进程。外部反馈尚未就绪时不接受运动任务，activate 不能绕过健康门控。deactivate 与 stop 等待新鲜的速度反馈确认停车；服务停机只回收本实例进程组，不按通用 Nav2 进程名清理。

## 配置与外部依赖

`params_file` 相对部署 manifest 所在目录解析。`policy` 保留原现场执行策略，确认项默认关闭，不能为让启动通过而随意设为 true。

- `provider_ids.cloud`：提供 `robonix/primitive/lidar/lidar3d` 的前雷达。
- `provider_ids.localization`：独立定位 service，同一实例提供 `robonix/service/localization/odom` 和 `.../status`；不将定位数据冒充底盘原语。`provider_ids.odom` 仍可单独指定 odom 角色。
- `topic_remap`：显式话题绑定覆盖 Atlas；显式值覆盖 Atlas；未显式绑定的定位状态与 Odometry 来自所选 localization service。
- `tf` / `tf_static`：已有 `map→odom→base_link` 动态树和传感器静态外参，服务不另造一套。
- `global_grid`：`origin: [x,y]`、米制 `width/height`、`resolution`；只决定传感器地图范围，不是静态通行先验。
- `velocity_output_topic`：默认为非运动 sink；接真实底盘属于后续部署步骤。

### 定位就绪 JSON

当前兼容原 `/outdoor/localization`（ROS `std_msgs/String`）消息结构：

```json
{"ready": true, "grade": "fixed", "ownership_conflict": false,
 "extrinsics": {"ready": true}, "heading": {"ready": true}}
```

消息必须持续新鲜，不能仅靠一帧 `ready=true`。完整定位数据可额外带 `heading` 细节、`feedback`。
`require_chassis_ready=true` 时还要求外部 chassis JSON 有新鲜的 `ready=true`。里程反馈和 TF、雷达也分别检查新鲜度。

## 能力调用

`robonix/service/navigation_outdoor/follow_reference` 的 `request_json`：

```json
{
  "frame_id": "map",
  "reference_path": [{"x":0,"y":0},{"x":0,"y":10},{"x":10,"y":10}],
  "goal": {"x":10,"y":10},
  "request_id": "0123456789abcdef0123456789abcdef",
  "arrival_tolerance_m": 3.0,
  "localization_grade": "fixed"
}
```

只接受有限坐标、长度不超过 4096 的参考折线和与末点完全相同的目标。不能将参考路线写进 Nav2 障碍图。重试复用相同请求 ID；不同内容复用 ID 拒绝。新任务取消旧任务并等待释放/停车，不能争抢速度出口。

- `status(run_id)`：空 ID 查服务与当前执行；非空 ID 查对应执行。`succeeded` 才是确认到达并停车。
- `cancel(run_id)`：接受取消不代表已停稳，需要继续查终态。
- `stop()`：停用并等待确认停车，恢复需要 Driver activate。
- `costmap`：ROS2 全局传感器 OccupancyGrid，transient_local QoS；局部地图和执行路径也由执行节点发布。

### 已标注路线的步态切换

可选字段 `map_id`、`version`、`origin_wgs84` 必须一起提供；`intervals` 只能描述同一条 `reference_path` 的累计二维路程，按本次行进方向排序，不能复用另一条规划线的下标。每段至少有 `start_m/end_m/gait/gait_code`，支持 flat、stairs、unknown，实际反馈原码与标签必须一致。原始手动码 0x1001/0x1003 分别映射为导航目标 0x3002/0x3003。

执行端校验路线原点与实时定位状态一致，当前只支持零 map 平移、零天线杆臂的迁移配置。地图内容版本随任务绑定并参与重复请求校验；Scene 仍须对照建图服务核实版本，不能把调用方自报的哈希当成源端验证。

启用配置 `gait_switching_enabled: true` 后，输出从所选 `provider_ids.chassis` 的 `robonix/primitive/chassis/gait_in` 发现；也可显式设置 `gait_output_topic`。默认关闭，默认输出为非运动 sink。底盘原语还必须显式启用步态出口；否则带区段的新任务拒绝。

楼梯区段前 1 米请求楼梯步态，离开后 1 米请求平地步态；邻近的扩展区间重叠时合并，避免在短缝隙反复切换。沿原参考折线有序跟踪进度，不用全局最近道路猜路段；反向行走必须使用已按方向重排的区段。

切换时先阻断速度输出，等待新鲜实测线速度、角速度低于停车阈值并持续 `stop_hold_s`，再每秒请求一次步态。只有新鲜 `/MOTION_INFO` 确认目标码后才继续；默认 `policy.gait_switch_timeout_s=10` 秒失败退出并停车。Scene 显示等待原因和任务失败原因。未知步态区段拒绝执行。

所有楼梯区段及上述缓冲使用 `StairsFollowPath` 沿参考线跟踪，暂停局部碰撞检查及 NavFn 绕行；平地切回原 DWB 和避障。需要安装 `nav2_regulated_pure_pursuit_controller`。保留急停、底盘状态、传感器新鲜度、定位断流及停车确认。默认到点容差仍为 3 米。

只请求 gait，不调用 posture、站起或使用模式切换。外部姿态指令改变步态时会停车重新确认。1 米缓冲不保证覆盖定位误差；本轮未验证真实上下楼梯。隔离真实 Nav2 测试入口：`tests/stairs_smoke.py`（仅域 188，虚拟传感器及命令出口）。

## Scene 接入

Scene 使用本包的 `outdoor_nav.scene.SceneNavigation`，只接收当前建图服务的参考路线，不读取旧 bundle、地点文件或旧网页。部署由机器人仓库的 `outdoor/` 提供，地图与原始记录始终由建图服务保存。

导航请求记录最多保留 1024 个，每个最多 2 MiB；达到上限时拒绝新请求并提示归档，不静默删除幂等重试记录。旧网页地图/规划/障碍注入入口已从导航进程移除。

## 测试

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=. python3 -m pytest -q
```

独立回归不需要原工程或校园资源。`tests/test_scene.py` 与 `test_scene_wire.py` 验证新地图身份、步态区段、幂等重试和真实 gRPC；`tests/stairs_smoke.py` 在隔离域 188 使用真实 Nav2 和虚拟传感器验证空地图启动、步态确认、楼梯避障暂停/恢复及超时停车。历史算法冻结快照仅用于回归来源对照，不在运行路径中。
