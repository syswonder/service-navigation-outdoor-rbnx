# Outdoor navigation service for Robonix

把 outdoorNav 的 **Nav2 执行与避障链**封装为 Robonix service，支持外部提供的整条参考路线，不重新实现道路业务规划。

```text
业务层（网页、地点、道路权重） ── follow_reference ──► 此 service
外部点云、定位/速度、TF ──► 投影 → 动态代价图 → NavFn → DWB → 平滑 → 安全守卫
                                                               │
                                          指定速度输出 → 外部底盘桥
```

不自带地图场景、网页、RTK/IMU/雷达驱动，不使用山猫 `/ODOM`，不调用站起/步态接口。
默认速度输出是 **`/robonix/nomotion/outdoor_cmd_vel`**，初始未激活且 `motion_enabled=false`。
本轮仅本地隔离验证，不代表已在机器人上部署或验收。

## 仓库结构

- `package_manifest.yaml`、`package_manifest.jetson-native.yaml`：标准 service 包入口。
- `capabilities/`：包内参考路线、状态、取消、停车和代价图契约；Driver 显式声明命名空间生命周期契约，兼容参考服务使用的 SDK，不再并列声明第二种 Driver。
- `config.spec`、`config/`：部署配置、原 DWB/NavFn 参数基线与部署条目示例。
- `outdoor_nav/atlas_bridge.py`：Atlas 依赖发现，Driver 生命周期，gRPC/MCP 能力。
- `outdoor_nav/runtime.py`、`executor.py`：唯一执行 owner、滚动窗口、停车确认和状态。
- `outdoor_nav/guard.py`：独立速度/扫描/定位看门狗，保留原链路的失效归零。
- `outdoor_nav/service.launch.py`：仅启动投影、执行器、守卫及 Nav2；不启动外部传感器与定位。
- `tests/`：原算法 AST 对比、参数/请求/生命周期测试、接原网页场景的隔离测试。
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
- `provider_ids.odom`：提供 Odometry 的定位 provider；可为 RTK 多普勒+WIT 的输出，不等于物理轮/足里程计。
- `topic_remap`：显式话题绑定覆盖 Atlas；`localization` 必须显式给出。
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

## 手动网页联调

本机已有联调容器时，运行 `bash scripts/demo_web.sh` 并保持终端开启，再打开 **http://127.0.0.1:19865/**。网页来自原仓，业务网关把完整参考路线经 gRPC 交给本 service；运动仅发生在虚拟底盘。停止用 `bash scripts/demo_web.sh stop`。启动条件、数据流和请求格式见 [网页联调](docs/WEB_TESTING.md)。

## 测试

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=. python3 -m pytest -q
```

隔离全流程需要原 outdoorNav **作为外部业务/虚拟传感器测试夹具**，不是服务运行依赖。测试容器挂载本仓 `/service`、原仓 `/baseline`，安装其 Node/Playwright 和本仓 ROS 依赖后：

```bash
# 仅在 Docker 内；启动的是虚拟底盘，命令被限制到 /isolated_task_flow/cmd_vel。
PYTHONPATH=/service:/baseline:/baseline/ros_ws/src/outdoornav_ros \
  /baseline/.venv/bin/python /service/tests/run_isolated_flow.py
```

测试脚本复用原网页任务用例，但原 `lynx_navigation` 和原 watchdog **不启动**：网页业务层生成参考路线，新 service 执行。RTK/WIT 数据与底盘为虚拟输入。扫描点高度落在当前配置高度带内，以真实触发闭合障碍圈。

## 保持与不包含的能力

保留：约 6 m 有序窗口、短段 NavFn 规划、DWB 连续跟随、中间点不停稳、任务到达/停车分离、取消/替换、阻塞等待与恢复、航向/定位/扫描/指令断流归零、速度限幅、禁止倒车/横移、输入无效/输出冲突拒绝。

不包含：道路权重与滑条、地图底图、业务地点/网页、原始定位解算、底盘桥、后雷达、坑洞/台阶感知、额外 Collision Monitor 安全区。前雷达近距过滤仍是兼容配置，不宣称已完成自车/腿部几何分割。

验证范围、原功能对应表与未验项目见 [VALIDATION.md](docs/VALIDATION.md)。
