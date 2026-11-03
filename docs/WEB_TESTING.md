# 网页如何接新的 service

## 当前本机怎么试

已准备的本机测试容器是 `outdoor-service-test`：本仓挂到 `/service`，原 outdoorNav 仓挂到 `/baseline`。这里只运行虚拟底盘、合成 RTK/WIT 和真实 Nav2；不会连接山猫或发布真实 `/cmd_vel`、`/NAV_CMD`。

在 **WSL** 中：

```bash
cd /mnt/c/Users/xjy/projects/service-navigation-outdoor-rbnx
bash scripts/demo_web.sh
```

这是前台常驻命令，保持此终端开启。出现 `MANUAL_WEB_READY` 后在浏览器打开：

**http://127.0.0.1:19865/**

- 网页文件直接来自原仓的 `web/dist`，由联调业务网关提供；不需要再启动一套 Vite。
- 进入 A→B 页面，A 是当前虚拟车位置；选一个距车十米左右的终点，点“规划 A→B”，再点“执行导航”。
- 可以改变路外代价、重规划、取消或替换目标；车位姿与执行路径会更新。
- 当前任务到达容差为 3 m，因此只点离车很近的目标可能直接判断到达。
- 查看实时代价地图也通过同一端口转发。
- `8877` 旧纯规划预览服务已关闭；当前用 `19865` 执行虚拟导航。
- 重复运行启动命令会拒绝第二次启动，不会关闭现有实例。

停止可在上述终端按 Ctrl+C，或另开 WSL 终端：

```bash
cd /mnt/c/Users/xjy/projects/service-navigation-outdoor-rbnx
bash scripts/demo_web.sh stop
```

脚本会验证 PID 身份，只关闭这个联调实例，并等待退出；不按通用 ROS 进程名清理。

本入口依赖**当前已准备的本地测试容器和 SDK/浏览器依赖**，不是任意机器 clone 后即可直接运行的生产启动脚本。完整 Robonix 部署仍按 README 的 manifest/build/Driver 流程，不使用 tests/ 下的联调入口。

## 终点和中间点怎么过去

```text
原仓网页：点击 B，POST /api/plans
     ↓
业务网关：调用原 OutdoorService + 道路连续代价规划
     ↓ 返回 reference_path，网页显示路线和关键中间点
点击执行：POST /api/tasks
     ↓
业务网关：生成/确认参考折线，保留精确终点与任务 ID
     ↓ ExecutionClient.follow_reference（真实 gRPC）
robonix/service/navigation_outdoor/follow_reference
     ↓ service 校验并保存完整 reference_path + goal
service 内部 NavFn/DWB：有序约 6 m 窗口，实时避障、连续跟随
     ↓ /isolated_task_flow/cmd_vel
虚拟底盘更新位姿 → 状态返回网页
```

**不是只把终点交给 Nav2 自由选路，也不是把页面上几个编号逐个当成停车目标。** 传的是后端完整参考折线 `reference_path`；页面编号是它的关键点展示。service 用折线约束行进顺序，同时在每个短窗口里按传感器代价图重新规划避障，最终目标才确认到达停车。

核心请求格式（米制 map 坐标）：

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

网页不直接生成速度，也不经 LLM 逐帧控制。执行、查询和取消走 gRPC；本地演示的激活控制与只读代价图走 localhost worker 管理接口。

## 代码与可核对记录

- `outdoor_nav/client.py`：外部业务进程调用 gRPC 的客户端。
- `tests/business_facade.py`：原网页/API/道路规划到执行 service 的联调业务网关，不含避障控制算法。
- `tests/local_rpc.py`：本地演示使用的同一组 provider gRPC handlers；仅省去 Atlas 自动发现，不代替执行器。生产改由 `atlas_bridge.py` 和 Driver/Atlas 管理。
- `tests/run_isolated_flow.py --manual`：保持虚拟传感器、原定位、service、gRPC 端点和业务网关存活，不自动跑完网页测试退出。
- 本地 gRPC 显式端点是 `127.0.0.1:29975`；正式部署应由 Atlas 发现，不把这个测试端口写成生产固定值。
- 每轮运行目录的 `route-handoff.jsonl` 会记录 `transport=grpc`、请求的终点/完整参考折线和 service 接受后的折线，可对比是否逐点保持。

已通过浏览器手动链路验证：网页选点、规划、执行，gRPC 下发后虚拟车产生实际位移；随后取消并停车。对比记录中请求折线与 service 接受折线完全相同。
