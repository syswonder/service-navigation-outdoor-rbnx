# 地形状态机与传感器优先级

本实现已合并 navigation main `fe738b6`。保留当前分支的独立雷达/视觉 costmap、原始 RGB-D 几何链路、可替换分割模型及现有 Robonix Driver/任务 API。没有合并主线的整套机器人参数。

## 开关与兼容

`terrain.enabled` 默认 false，旧部署继续执行已有路线/步态区间逻辑。开启后，实时状态机监督地形动作；启用攀爬能力时才拥有地形步态请求。仅平地监督时共用平地导航步态准备与确认；旧地图步态区间只保留为记录信息，不再触发切换，其格式仍校验，硬件编号不再限定为山猫编号。`terrain.slope_filter_enabled` 独立控制线性高度阈值，默认 false；`slope_angle_deg` 默认 15°。这两个开关须由部署显式启用。

`policy.lidar_enabled`、`camera_enabled`、`camera_geometry_enabled`、`semantic_segmentation.enabled` 继续独立控制各路输入。`policy.camera_max_range_m=4.0` 对视觉几何、语义投影、楼梯识别同时生效；按 base_link 前方的水平距离计算，后方和更远点不参与。模型仍对完整 RGB 图像推理，空间裁剪在深度反投影之后完成。

Ranger 轮式设备必须保持 `can_climb_stairs: false`。开关不会赋予硬件实际能力，也不会修改 `motion_enabled`、标定确认、底盘反馈或命令输出话题。

## 四状态与动作阶段

运动状态集合为 `stop / flat / stairs_up / stairs_down`，其中 `stairs_down` 仅预留，没有进入转移。上下坡均处于 flat。

`enable` 在现有任务链路中解释为：服务已激活、`policy.motion_enabled=true`、存在未取消的执行任务，且该任务完成初始停车确认和配置准备。停止、取消、任务终止都会撤销 enable，不会创建第二个底盘速度发布者。

令俯仰角为 θ，楼梯观测为 S，硬件上楼梯能力为 C。水平判定为 |θ|≤`pitch_flat_deg`，明显倾斜判定为 |θ|≥`pitch_hold_deg`，两阈值间保持既有状态。

| 条件 | 结果 |
|---|---|
| enable=0 | stop，输出零速度 |
| enable 从关闭进入执行 | flat，继续检查输入与真实平地步态反馈 |
| flat，前方楼梯，C=false | stop，原因 `stairs_unsupported` |
| flat，水平，稳定楼梯，正向抬升和整片点云朝向有效，C=true | stairs_up |
| 楼梯方向/朝向无法确认 | 在最近可见楼梯点距离大于 `stair_enter_range_m` 时可平地逼近；否则停车等待，不切步态 |
| stairs_up，明显倾斜 | 保持 stairs_up |
| 水平且连续确认无楼梯 | 恢复平地步态，反馈确认后 flat |
| 缺失/过期 IMU | 暂停输出 |
| climb 中缺失语义、IMU 仍倾斜 | 保持楼梯阶段；只有新鲜楼梯步态反馈匹配才允许继续 |
| 语义缺失、IMU 水平 | 只提供有限观测宽限；缺数据不等同于“确认无楼梯” |
| 对正/步态确认超时 | 锁定 stop；关闭导航后才能重新开始 |

stairs_up 内的动作阶段为：

1. `align`：停车，确认平地步态；利用整片楼梯点云的三维 PCA 斜面法向求上升方向，再调整朝向。无需看到最底部台阶。默认角速度上限 0.15 rad/s。转动前保留新分支的融合 costmap 无障碍检查；本次山猫楼梯实验将半径缩至 0.25 m，尚未实测对应的机身旋转包络，不能作为安全间距保证。
2. `switch_gait`：对正后再次确认停车，请求配置的上楼梯步态；真实反馈匹配之前保持零速度。
3. `climb`：使用部署的 `StairsFollowPath`，最高 0.15 m/s。该控制器必须是 RPP 且 `use_collision_detection: true`。原任务控制器切换/释放逻辑仍生效。
4. `restore_flat`：到达水平且楼梯消失后停车，恢复平地步态并确认反馈。

这沿用了 main 中“平地步态转向、楼梯步态攀爬”的硬件约束。楼梯中途不自动切平地步态原地旋转，尚未实现的下楼梯也不会由负俯仰猜测触发。

## 感知与融合

模型仍遵守通用 `predict(rgb) -> (labels, confidence)` 接口。楼梯类别通过 `terrain.stairs_classes` 引用 `semantic_segmentation.classes` 的**类别名称**，没有固定类别编号、模型名称或权重路径。

模型内部先按置信度阈值将低分像素标为未知，不再单独发布置信度图。楼梯节点只配对类别图、注册深度及两路内参，在 base_link 前向 ROI 中反投影。整片楼梯点云做三维 PCA，以最小主分量作为斜面法向，再求水平上升方向；不截取最近台阶，也不假设最长主轴总是楼梯宽度。点数、宽度、抬升或斜面退化检查不通过时不生成有效朝向。目标方向与语义三维点保存在 odom 坐标系。

当前现场配对容差为 0.30 s，TF 取最近可用值；彩色图保留镜头畸变，只有注册深度要求零畸变。这些是待校验的现场改动，不等价于采样同刻、像素严格对齐的标定结果。明日先在不输出运动的条件下采集 RGB-D/TF，验证投影与朝向。

默认优先级采用有序仲裁，大值优先：

| 数据源 | flat | stairs_up |
|---|---:|---:|
| 雷达障碍 `lidar_obstacles` | 100 | 60 |
| 雷达语义 `lidar_semantics` | 50 | 90 |
| 视觉障碍 `visual_obstacles` | 90 | 50 |
| 视觉语义 `visual_semantics` | 40 | 100 |

flat 中障碍按原有最大代价融合，语义提供偏好，不能清除物理障碍。climb 中，只有与最新楼梯语义三维采样点距离小于 `semantic_match_radius_m`、且语义排名高于对应障碍源的点，才会作为可攀爬表面处理。相同排名保留障碍。不清空整张 costmap，不关闭整个雷达层，不把楼梯包围框中的人或其他物体一并清掉。视觉跌落点仍立即保留。每路先独立处理，最后 costmap 仍用 maximum 合并。

雷达语义保留为 TODO，lidar_semantics_enabled 必须为 false；误开启会在初始化时报错，不会启动后等待。当前不创建雷达语义订阅、不参与仲裁、不影响其余三路。消息约定仅供未来实现参考。已启用的视觉楼梯语义若过期，对准仍暂停；已进入 climb 且 IMU 倾斜时可保持爬升。主动关闭该来源则不等待它。

## 共享线性高度阈值

先用采样时刻 TF 将点转换到 base_link，令 d=√(x²+y²)，h=z−`ground_z_m`：

`h_min(d) = height_margin_m + d × tan(slope_angle_deg)`。

正障碍保留区间是 `h_min(d) < h ≤ h_min(d)+max_obstacle_height_m`。上边界随下边界一起抬升，避免远处下阈值超过固定上阈值后整片点云被删除。开启后，该范围取代这一侧原来的固定投影高度带；关闭后仍使用原有 `scan_projection` 参数。

`ground_z_m` 是地面在 base_link 中的 z，不是雷达安装高度。相机几何侧使用同一公式，低于阈值的已观测正向表面进入可通行覆盖；已检测到的负障碍不经过这项豁免。相机地面拟合/RTAB-Map 允许的坡度随配置同步；语义地面投影也扩展到该坡度范围。

这是用户指定的距离高度启发式，**不是可通行性证明**：例如 4 m 处默认最小高度约为 1.17 m，较矮的真实障碍也可能落在阈值下。需要用实地低矮障碍与坡道数据标定范围及角度；不能仅凭这个过滤器宣称适用于无人值守运行。

## 新增参数

以下字段除特别说明外均在 `terrain` 下。完整字段类型同时声明在 `config.spec`。

| 参数 | 默认 | 含义 |
|---|---|---|
| `enabled` | false | 启用实时地形状态机 |
| `slope_filter_enabled` | false | 共用线性高度阈值总开关 |
| `lidar_slope_enabled` / `camera_slope_enabled` | true / true | 各路几何过滤开关，受总开关约束 |
| `slope_angle_deg` | 15 | 线性阈值角度，必须小于 45° |
| `ground_z_m` | 无默认值 | 从必填的本机 `ground_height_file` 派生，不在清单中直接填写 |
| `height_margin_m` | 0.10 | 近距离高度余量 |
| `max_obstacle_height_m` | 2.5 | 线性下阈值以上保留高度带宽 |
| `can_climb_stairs` | false | 本体上楼梯能力声明 |
| `stairs_classes` | [] | 模型类别表中的楼梯类别名称 |
| `lidar_semantics_enabled` | false | 是否接入外部雷达地形语义 |
| `pitch_flat_deg` / `pitch_hold_deg` | 4 / 8 | 水平与保持状态阈值 |
| `pitch_sign` / `pitch_offset_deg` | 1 / 0 | 安装 TF 变换之后的俯仰符号及零位补偿 |
| `confirm_s` / `clear_s` | 0.5 / 0.8 | 楼梯确认、无楼梯确认的连续观测时间 |
| `observation_timeout_s` | 1.0 | IMU、语义和状态消息的最大采样年龄 |
| `roi_half_width_m` | 0.8 | 检测前方楼梯走廊的半宽，不是整个避障视场宽度 |
| `min_stair_points` | 20 | 整片楼梯点云的最低点数 |
| `min_stair_rise_m` | 0.08 | 楼梯点高度的 75 分位数相对地面至少抬升量 |
| `min_stair_width_m` | 0.35 | 楼梯点云在上升方向横向的最小跨度 |
| `stair_enter_range_m` | 1.2 | 最近可见楼梯点进入此距离后，平地逼近停止，需先对准并确认楼梯步态 |
| `alignment_tolerance_deg` | 6 | 目标朝向允许误差与确认期朝向稳定阈值 |
| `alignment_speed_rps` / `alignment_kp` | 0.15 / 0.8 | 对正角速度上限及比例系数 |
| `alignment_timeout_s` | 20 | 对正及平地步态准备最长时间 |
| `stairs_speed_mps` | 0.15 | 上楼梯速度上限，不能高于部署速度限制 |
| `semantic_match_radius_m` | 0.12 | 同一楼梯表面的三维关联半径 |
| `flat_gait_code` / `upstairs_gait_code` | null / null | 真实 UInt32 命令及反馈码，必须不同，不内置山猫编号 |
| `priorities.flat` / `priorities.stairs_up` | 见上表 | 每种状态下四个来源的有序排名 |
| `policy.camera_max_range_m` | 4 | 所有视觉空间处理的前向水平距离上限 |

沿用 `policy.gait_switch_timeout_s`、`stop_hold_s`、`stopped_linear_mps`、`stopped_angular_rps`、`sensor_timeout_s`、`command_timeout_s` 及原有视觉稳定过滤参数。

## ROS 2 接口与 Robonix

没有新增 Atlas 导出能力。状态放入原有 navigation status 的 `terrain_status` 和任务详情，同时发布 `topic_remap.terrain_state`（默认 `/outdoor/terrain_state`，`std_msgs/msg/String` JSON）。仍经现有 Robonix Driver 启动，命令经过原 command watchdog。

输入 `topic_remap.imu` 默认 `/wit_10axis/data`（`sensor_msgs/msg/Imu`），必须包含有效姿态及到 base_link 的 TF。可通过已有 `robonix/primitive/imu/imu` 能力发现并配置 `provider_ids.imu`。按 IMU 安装旋转转换姿态后再求本体 pitch。

`topic_remap.visual_terrain` 默认 `/outdoor/vision/terrain`，由本包 RGB-D 观测节点产生。`topic_remap.lidar_terrain` 默认 `/outdoor/lidar/terrain`，留给外部雷达语义实现。二者类型均为 `std_msgs/msg/String`，JSON 为：

```json
{"schema_version":1,"stamp":1234.5,"frame_id":"odom","valid":true,
 "stairs":true,"direction":"up","heading_yaw":0.0,"front_m":1.0,"points":[[2.0,0.1,0.2]]}
```

`stamp` 必须是原始采样的 ROS 时间（秒），不能用重发时间刷新。`heading_yaw` 是 odom 中朝向楼梯的法向角；无法确认时为 null。`direction` 只接受 `up` 或 `unknown`。`front_m` 是最近可见楼梯语义点在 base_link 中的水平距离，不是脚到第一级台阶的真实间距。`points` 是实际楼梯三维点，最多 4096 个。`stairs=false` 只可用于可靠观测到没有楼梯，不能用来代替掉帧、遮挡或无有效深度。来源身份由订阅话题决定。

底盘反馈仍使用 `topic_remap.chassis` 的 JSON：`ready`、`gait_output_enabled`、`gait`、`motion_feedback_age_s`。新状态机不依赖山猫的 state=17/usage_mode=1；底盘适配器负责将自己的真实反馈转换为此约定。`gait` 必须与配置的命令码处于同一编号域，不能用“已发送”伪装“已切换”。

`topic_remap.filtered_cloud` 默认 `/outdoor/cloud_for_scan` 是过滤后供原 pointcloud_to_laserscan 使用的内部点云。所有上述话题可重映射。

## 验证范围

单元验证覆盖状态转移、缺数据、反馈超时、对正顺序、坡度边界、距离裁剪和三维优先级关联；ROS 隔离验证见本地 `local_only/`。测试脚本与报告均不上传。2026-11-24 山猫实测走到过 stairs_up/climb 状态，但未完成可靠的识别式上楼验收；状态切换不是实机成功攀爬的证明。实际识别召回率、朝向稳定性和可通行性仍需验证。新增主轴估计已通过合成点云单元测试，尚未实机复测。


## 四路可选性

| 来源 | 有效开关 | 关闭后的行为 |
|---|---|---|
| 雷达障碍 | policy.lidar_enabled | 无雷达投影、雷达健康要求或雷达 costmap 层 |
| 视觉障碍 | policy.camera_enabled 且 policy.camera_geometry_enabled | 无几何检测和 RTAB-Map，不等待几何证书 |
| 视觉语义 | policy.camera_enabled 且 semantic_segmentation.enabled | 无模型、语义投影和楼梯观测，不要求权重或语义数据 |
| 雷达语义 | TODO，terrain.lidar_semantics_enabled=false | 无订阅、无健康依赖、无仲裁席位 |

优先级允许只填写需要覆盖的状态/来源，其余使用默认值。保留禁用来源的排名不会启用它，运行时 terrain_status.priorities 只展示启用来源。
至少开启一种实际感知路径。camera_enabled=true 时几何和语义不能同时关闭。
全部语义关闭时可执行普通平地导航，但没有自动楼梯识别能力；声明可上楼梯却未配置有效楼梯语义会被拒绝。

## 本机地面配置

必须通过顶层 `ground_height_file` 指定 Git 忽略的本机 JSON 文件。文件中 `ground_z_m` 为有限负数，表示地面在 base_link 下的高度；没有有效值就拒绝初始化，不回退到示例值。`terrain.ground_z_m` 和 `policy.camera_expected_ground_z_m` 在读取后由同一值派生，不允许分别在清单中填写。

复制 `config/ground.local.example.json` 为本机 `config/ground.local.json`，填入该设备的测量值。模板 null 不是可用值。`run/runtime.json` 中会记录实际采用的数值，但运行目录也不纳入 Git。
