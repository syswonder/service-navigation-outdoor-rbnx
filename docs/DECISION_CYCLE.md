# 周期式方向引导（显式启用）

这条执行链保留测量路线的顺序和拓扑，但不要求车轮沿录制折线行驶。默认关闭，旧执行方式保持不变。

```yaml
terrain:
  enabled: true
decision_cycle:
  enabled: true
  period_s: 2.0
  hint_spacing_m: 6.0
  corner_error_m: 0.3
  arrival_side_m: 3.0
  lookahead_m: 3.0
  grid_resolution_m: 0.1
  grid_half_width_m: 8.0
  sample_limit: 512
```

`period_s: 2.0` 是初始配置值，不是实机通过的周期保证。部署前必须结合本机诊断及全部启用的模型测量长尾耗时。保持已有运动许可、标定和底盘确认参数；开启周期不会绕过这些条件。

## 点池与队列

mapping 保留原测量点，并声明 `reference_role: ordered_hint_pool`、`tracking_required: false`。navigation 接收最多 65536 个点，用 RDP 的最大折线偏差 `corner_error_m` 提取显著拐角。沿各段按与上个保留点的直线距离至少 `hint_spacing_m` 选取原采样点。拐角及最终点必须保留，允许它们小于间隔。

`corner_error_m` 是拐角识别容差，不是地理路线约束。过小会保留 GNSS 噪声形成的假拐角；过大会忽略小转弯。路线自交时仍保持原顺序，不按离车最近的点跳转。

中间点进入车体坐标下 `|x| <= arrival_side_m/2` 且 `|y| <= arrival_side_m/2` 时切换下一点，每周期至多切换一次。默认就是车体朝向对齐的 3×3 米方框。最终点仍使用已有 arrival_tolerance_m 距离容差，并检查当前定位、终点朝向要求和停车确认；3×3 米方框只放宽中间提示点。

局部规划器只接收朝当前提示点方向、距离不超过 `lookahead_m` 的局部目标，使用传感器代价图自由绕行，不输入原录制折线作为跟踪路径。局部目标附近没有可通行路径时停止等待，不能为了朝向提示点忽略障碍。

## 坐标与时间

localization 的状态中必须包含 `navigation_sample`：schema_version=1、stamp_s、sample_id、epoch、reference_point=base_link、local_frame=odom、global_frame=map，以及同一时刻的 map_pose/odom_pose（x、y、yaw）。重复发布同一个样本不会重复计权；定位重置/缺口恢复改变 epoch 后，需重新发起导航任务。

对于地图提示点 p，每份配对定位计算：

`p_odom = T_odom_base × inverse(T_map_base) × p_map`

在决策边界使用此前一个周期内不同采样时刻的 p_odom 算术均值。本周期的新样本进入下一次计算。雷达和视觉融合图始终位于 odom；GNSS 修正只改变提示点的位置，不直接平移局部障碍。这个方法平滑目标方向，不会消除 GNSS 系统偏差，也不是对漂移的定位校正算法。

## 周期顺序与过期

1. 截止边界前一个周期的定位样本，平均提示点并更新队列。
2. 推进一次地形状态机。原高频状态机计时器在该模式下不再推进它。
3. 等待状态机更新后新获取的已处理雷达/视觉观测，避免新状态使用旧状态过滤的障碍结果。
4. 在 odom 生成单份融合图，等待 global/local costmap 均确认该快照。
5. 请求一次局部路径，交给对应控制器。对齐楼梯或等待步态确认时遵循状态机输出，不发普通前进路径。

只有启用的传感器参与等待。融合使用最大占据代价，未知不清除障碍、视觉可通行不覆盖雷达障碍。两个 Nav2 costmap 均改为 odom 滚动窗口，仅加载周期快照和膨胀层；额外自定义层会被拒绝，避免未经冻结的数据混入。

传感器时效为一个 `period_s`，且控制有效期不超过本周期边界和最老输入的过期时刻。计算超时或数据缺失停止输出并撤销已接收的控制路径；下一边界重试，漏掉的周期不追赶执行。连续失败沿用 `policy.planner_patience_s`；反复重新规划却没有 odom 位移进展沿用 `policy.controller_patience_s`。楼梯对齐和步态等待不计入平移进展时限。底层控制器、速度平滑与停车看门狗仍按原频率运行，不降到决策周期频率。

当前实现每个周期重新计算期间会暂停运动输出。因此它是保守的“计算后运动”模式，并非无缝连续运动；不能仅根据平均计算时间推算行驶速度。若需连续运动，必须另行设计双缓冲与更短的紧急障碍停车链。

## 诊断与测量

状态中的 `decision_cycle` 报告周期编号、提示点、样本数、有效性和失败原因。时间单位均为 ms：controller_release_ms、state_and_hint_ms、fsm_ms、sensor_wait_ms、fusion_ms、costmap_wait_ms、fusion_and_costmap_ms、planning_ms、controller_accept_ms，以及总 compute_ms。

模型推理、原始点云投影和定位估计器在异步生产端运行；sensor_wait_ms 只代表等待新处理结果，不等于这些模块的独立计算时间。必须分别测量，不能将合成 scan/grid 测试称作实传感器全链路通过。测试脚本和设备测量报告保存在被忽略的 local_only/，不纳入 Git。
