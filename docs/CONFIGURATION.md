# 参数配置约定

参考 [service-navigation-rbnx](https://github.com/syswonder/service-navigation-rbnx)
的部署参数规范（核对版本 `0fbf1c78ecb38c194b87671009a1d4819aee2b2a`）。
室外服务继续使用自己的参考路线契约和原视觉链路。

## 配置放在哪里

| 位置 | 内容 |
| --- | --- |
| 部署 `config/outdoor_nav2.yaml` | 完整 planner/controller/smoother、两个 costmap、足迹、速度/加速度、容差、膨胀层、各传感器层参数 |
| 部署 manifest 的 `provider_ids` / `topic_remap` | Atlas 实例选择、显式 ROS 话题绑定、TF 话题 |
| 部署 manifest 的 `scan_projection` | 雷达点云转扫描的高度、距离和 TF 容差 |
| 部署 manifest 的 `policy` | 传感器开关、原相机地面标定、稳定性过滤、观测超时、参考路线验收及运动确认项 |
| 部署 manifest 的 `semantic_segmentation` | 可选语义开关、适配器、必需权重、设备及模型选项 |
| 服务 `config.spec` | 可接受字段、单位、默认值、分组和迁移说明；与运行时校验对应 |

服务示例 YAML 是可复制的完整结构；机器人实际调参应在部署副本中完成。
不在服务启动代码中重建 costmap、补入控制器或覆盖 YAML 中的调参。
`params_file` 和权重相对活动 manifest 所在目录解析，使用 `RBNX_INVOCATION_CWD`；
权重路径可以是文件或目录，具体格式由适配器验证。可选 SegFormer 适配器要求含权重、`config.json` 和 `preprocessor_config.json` 的本地目录；配置示例及接口约定见 README。
`run_dir` 显式为相对路径时也遵循这一规则。

## 话题绑定和生效参数

YAML 支持以下占位符，绑定到 `topic_remap` 显式值或 Atlas 解析结果：

| 占位符 | 角色 |
| --- | --- |
| `__ROBONIX_ODOM_TOPIC__` | `odom`，独立定位服务的 Odometry |
| `__ROBONIX_SCAN_TOPIC__` | `scan`，雷达独立投影结果 |
| `__ROBONIX_SCAN_CLOUD_TOPIC__` | `cloud`，原始雷达 PointCloud2 |
| `__ROBONIX_CAMERA_COSTMAP_TOPIC__` | `camera_costmap`，原相机检测后经过稳定性过滤的本层代价图 |

未知或未解析的 `__ROBONIX_*` 占位符会拒绝初始化。
点云输入、定位输入优先使用 `provider_ids`；显式 `topic_remap` 对相应角色优先。
原相机 RGB、配准深度和 CameraInfo 使用显式话题。
不引入参考服务的静态地图、AMCL 或通用深度点云检测器。

校验后的完整配置保存在运行目录的 `runtime.json.nav2_params`；
启动时写出同目录 `effective_nav2.yaml`，Nav2 实际读取该文件。
调参可对照源 YAML 与此文件；正常新配置只做话题绑定和禁用传感器层裁剪。

## 三种传感器组合

`policy.lidar_enabled` / `policy.camera_enabled` 选择雷达、相机或两者。
完整 YAML 预先声明 `lidar_layer`、`camera_layer`、`inflation_layer`。
两个 costmap 中，关闭的传感器层从 `plugins` 列表裁剪；其配置块保留但不加载。
开启的传感器必须有启用的对应层，否则初始化失败。

雷达层读取其独立扫描，`combination_method=1` 按最大值合并。
相机层必须使用 `outdoor_camera_layer::CameraLayer`，读取过滤后的 `camera_costmap`；
不以相机清除雷达障碍。最后由 Nav2 融合两层并执行膨胀。
`camera_layer.max_age_s` 在 YAML 中配置；`policy.sensor_timeout_s` 是执行与命令守卫的
反馈时效限制，两者各自生效。

相机沿用原 RGB-D 检查、RTAB-Map、正负障碍检测和 可替换模型辅助语义。
新增稳定性过滤仍在原检测结果之后；本次参数调整不改检测算法。

雷达为 3D 输入时使用 `scan_projection`；`enabled` 未指定则跟随雷达开关。
部署目录的 `scripts/render.py` 会同步设置该值。它只影响雷达投影，不影响相机。

## 速度和到点条件

线速度上限取 YAML 中 `FollowPath.max_vel_x`、`max_speed_xy` 与
`velocity_smoother.max_velocity[0]` 的较严格值；角速度同时考虑 DWB 和平滑器正负方向限制。
这些值供最终守卫限幅，服务不反向改写 Nav2 参数。
保留现有室外执行链最大 0.4 m/s、0.4 rad/s 的校验边界，以及禁止倒车/横移的守卫行为。

`policy.command_timeout_s` 管最终命令守卫；YAML 的 `velocity_timeout` 管速度平滑器。
`policy.stopped_linear_mps` 管实测停车确认；DWB 的 `trans_stopped_velocity` 在 YAML 中独立配置。
`fixed_arrival_m` / `float_arrival_m` 管 GNSS 参考路线验收，不替换 Nav2 局部目标容差。
本次没有增加动态调速能力或新能力契约。

当前执行器使用 `map` / `odom` / `base_link`、`GridBased` 和 DWB `FollowPath`。
此版本会拒绝不支持的 frame 或 FollowPath 插件组合，避免静默按另一组参数运行。
启用步态切换时，部署 YAML 还需声明 `StairsFollowPath`；服务不自动插入。

## 旧参数迁移

以下显式旧值暂时兼容，校验时发出 `FutureWarning`；新部署不要继续使用。

| 旧配置 | 新位置 |
| --- | --- |
| `global_grid` | YAML `global_costmap` 的尺寸、原点、分辨率、rolling_window |
| `policy.max_speed_mps` / `max_angular_speed_rps` | YAML DWB 和 velocity_smoother 的速度限制 |
| `policy.footprint_radius_m` | 两个 costmap 的 `robot_radius` 或实测 `footprint` |
| `policy.planner_tolerance_m` | YAML `planner_server.GridBased.tolerance` |
| `policy.transform_tolerance_s` | YAML 对应 TF 参数，以及 `scan_projection.transform_tolerance_s` |
| `policy.inflation_padding_m` / `inflation_cost_scaling` | YAML 各 costmap 的膨胀半径和代价衰减 |
| `policy.local_track_unknown` | YAML `local_costmap.track_unknown_space` |
| `policy.obstacle_layer_enabled` | YAML 各传感器层 enabled；所选传感器层必须启用 |
| `policy.cloud_min_height_m` / `cloud_max_height_m` | `scan_projection.min_height_m` / `max_height_m` |
| `policy.scan_range_min_m` | `scan_projection.range_min_m` |

旧覆盖只在用户显式提供字段时执行，默认值不再覆盖 YAML。
雷达投影新旧字段同时出现会拒绝初始化，要求移除旧字段。
只包含 planner/controller/smoother 的历史不完整 YAML 需要补入 costmap 和控制器声明；
服务会指出缺少的段，完整示例位于 `config/nav2_params.example.yml`。

配置中的未知顶层字段、未知角色、错误类型、非有限数值、无效投影边界、未知 topic token、
缺少的必要 Nav2 段均在启动子进程前拒绝。运动默认保持关闭。
