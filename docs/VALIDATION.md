# 隔离验证与原功能对应

日期：2026-11-03。没有连接山猫，没有真实运动，没有部署到 NX。

## 已验证

1. **Robonix 包验证**：真实 CLI `rbnx validate` 通过，识别本包 6 项 capability。
2. **官方代码生成**：编译官方 `robonix-codegen` / `rbnx` 后，真实 `rbnx codegen --mcp` 通过，验证 68 个生成 Python 模块；未生成 ROS2 IDL overlay。SDK 公共树有 3 个无关 IDL 缺依赖提示，本包契约全部成功生成。
3. **26 项测试通过**：使用真实 Robonix API、官方生成的 protobuf/MCP types、Humble 消息；含真实 localhost gRPC Driver INIT/ACTIVATE/DEACTIVATE/SHUTDOWN、配置错误、依赖缺失 Deferred、参考执行/状态/取消调用与 MCP typed handler。wire 测试中的 worker 是受控测试替身，不将它当作 Nav2 实测。
4. **新 service 的独立真实 worker 测试通过**：`tests/manager_smoke.py` 不导入原仓库，虚拟点云、外部 Odometry/TF/就绪状态驱动真实 Nav2 与新 manager。INIT 后未激活；激活后完成路线并确认停车；第二任务停用后取消与停车；shutdown 回收拥有的进程组。见 `validation/manager-smoke.json`。
5. **原网页任务场景通过**：原业务层、原 RTK/WIT 虚拟输入、原网页测试继续使用；只把导航执行换成本 service。原 `lynx_navigation` / 原 watchdog 没有启动。完成路外目标、真实障碍圈阻塞等待与恢复、到达停车、取消、替换、航向断流停车和拒绝新请求、恢复后新任务、应用保留。最终虚拟速度为零。见 `validation/service-flow.json`。

第 4、5 项分别验证“新包独立可运行”和“接原业务链不丢功能”，不是仅靠 import 或 mock 宣称一致。

## 原功能对应表

| 原对应部分 | 新实现 | 保持方式 |
| --- | --- | --- |
| RouteGuidance 有序约 6 m 窗口 | core/guidance.py | 直接提取；不跳到 U 型路线后半段 |
| 短窗口 NavFn + 连续 FollowPath | executor.py | 主要方法与冻结原源码做 AST 对比 |
| 规划/控制失败预算、临时阻塞等待 | core/tasks.py / executor.py | 原逻辑提取，原网页阻塞恢复用例通过 |
| 原最终 watchdog | guard.py | tick/scan/receive/geo/check_owners 等做 AST 一致检查，重放断流/NaN/限速/禁止倒车用例 |
| 3 m 任务到达与控制器 0.5 m 上限分离 | runtime.py.prepare_route | 保留任务级容差，不让每个窗口提前“到达” |
| 取消/替换/最终停车确认 | executor.py / requests.py | 旧控制器释放与新任务所有权串行，场景用例验证 |
| 扫描 +inf 是空场、不等于断流 | core/scan_health.py | 直接提取并单测 |
| 只读动态代价图 | core/costmap_view.py / costmap capability | 未加入道路或建筑静态先验 |

明确的边界变化只有：原执行器内部生成道路参考路线的接缝改为**读取外部已提交的参考折线**，并在首次执行快照中保留该折线。道路权重、地点与 UI 不搬进 service。源码冻结参照与 SHA-256 记录在本仓，运行不依赖 sibling outdoorNav。

## 尚未声称完成

- 没有在 NX/Jetson 实机验证，也没有执行底盘真实输出。
- 没有把整套生产 `rbnx boot`（Atlas、真实 provider、底盘桥）宣称已跑通；已分别验证 CLI manifest/codegen、SDK Driver/RPC/MCP、实际 worker 和原业务闭环。
- 本地脚本 `build.sh` 各构件已经验证；当前测试容器为 Ubuntu 22.04、临时编译 CLI 的宿主为较新 glibc，完整 CLI 在容器内不可直接执行。代码生成通过宿主 CLI + 容器 Python 完成；部署须使用目标系统兼容的 Robonix CLI。
- 不宣称两轮运动轨迹逐点或耗时完全相同；一致指对应执行/安全语义与场景结果，不是控制器浮点计算的逐拍一致。

## 本地测试条件

ROS 2 Humble x86，Docker 本地 DDS（domain 94 或 95，localhost-only），命令仅到虚拟话题；NavFn、DWB、速度平滑和激光投影为真实节点。Python SDK 环境固定 `mcp<2.0.dev0`、`fastmcp<4`。

## 手动网页入口补充

已补保持运行的 `scripts/demo_web.sh` 与 gRPC ExecutionClient。浏览器实测选点、规划、gRPC 下发、虚拟车位移和取消停车，参考折线逐点一致；记录见 `validation/web-handoff.json`。已验证停止后重启、重复启动拒绝且不影响现有实例。前述整套网页任务场景记录来自首次业务适配层；本次 gRPC 接入后单独验证了手动链路，没有把两者混说成“完整自动场景均经 gRPC 重跑”。
