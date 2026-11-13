# Outdoor navigation service for Robonix

此视觉版本位于 `feat/vision-multisensor-navigation` 分支，包含原 RGB-D / RTAB-Map / 可选语义分割 链路及检测后稳定性过滤。现有使用者可继续使用 `main`；采用视觉版本时，在部署 manifest 中显式选择此分支，并按下文安装视觉依赖、配置完整 Nav2 参数。

## 部署参数约定

完整 Nav2 参数由机器人部署的 `params_file` 拥有，服务只绑定话题占位符并裁剪关闭的传感器层，
不再用启动代码覆盖速度、足迹、膨胀、地图尺寸或插入控制器。
雷达投影放在 `scan_projection`，原视觉检测/稳定过滤及执行确认项保留在 `policy`。
字段、单位和旧配置迁移见 [配置说明](docs/CONFIGURATION.md) 与 `config.spec`。
实际生效配置写入运行目录的 `effective_nav2.yaml`。

## 原视觉链路与检测后过滤

相机链路沿用 xiangruicheng 当前 geometry_assisted/rtabmap 实现：
配准 RGB/深度/CameraInfo 检查 -> 原地面参考估计 -> RTAB-Map 法向量/聚类 ->
原离地 0.10m 正障碍及负障碍复核 -> visual_obstacles + observed_ground + depth_status。
可替换语义模型 -> 完整类别图和置信度；按配置生成道路掩码 -> 原语义投影，作为辅助偏好。
原检测源码按文件保留在 vendor/，没有使用先前的固定高度裁切替代检测器。

新增过滤只位于原检测输出之后。先按同一采集时刻配对障碍、地面和有效状态，
再在连续 odom 中按位置关联、累计命中/持续时间筛选稳定正障碍；
已测得的负障碍直接保留。重复帧不累计，失效证书立即撤销相机就绪并清空本层图。
地面覆盖继续使用原 observed_ground_region，四个角都需接近实际地面点；
不以激光式射线或未知深度制造地面自由空间。

语义分割仍只在几何已知自由区域叠加非道路软代价 40，不覆盖物理障碍或未知。
相机本层使用原 build_costmap，再由随包编译的 CameraLayer 与雷达独立层取最大值合并；
未知相机单元不修改雷达，旧相机标记在本帧/超时后退休。最后由 Nav2 统一执行规划控制。

`policy.lidar_enabled` / `camera_enabled` 选择雷达、相机或两者。相机几何检测与可选语义分割独立，默认 `semantic_segmentation.enabled: false`，无需语义模型权重或 PyTorch。
启用语义分割必须先下载对应模型权重，再设置 `semantic_segmentation.weights`；仓库不附带权重，不自动下载。缺失权重在初始化时报错并提示本文。相对路径以部署 manifest 所在目录为基准。

```yaml
policy:
  lidar_enabled: true
  camera_enabled: true
  camera_frame_confirmed: false # 必须实测确认配准和外参
semantic_segmentation:
  enabled: false
  backend: "" # 安装适配器后填写 your_package.model:create_backend
  weights: "" # 权重文件或目录；格式由适配器检查
  device: auto
  confidence_threshold: 0.7
  classes: {} # 模型原始类别编号 -> 类别名称
  road_classes: [] # classes 中参与道路软代价的类别名称
  options: {} # 原样传给适配器的模型专用配置
```

### 可替换模型接口与权重

当前默认模型、权重和类别表暂时留空，不附带或自动下载任何模型。
开启时缺少适配器、权重路径或类别表会报错，并提示本文；关闭时不导入模型依赖。
模型的来源、权重下载地址和许可由所选适配器提供，不沿用旧模型的许可结论。

通用接口位于 `outdoor_nav/segmentation.py`。`backend` 只接受 `模块:工厂函数`，
模块需安装到导航 worker 使用的 Python 环境。工厂签名为
`create_backend(*, weights, device, options)`，返回具有 `predict(rgb)` 方法的对象。
`weights` 支持文件或目录；通用层只检查路径存在且为文件/目录，文件格式、目录完整性和模型兼容性由适配器检查。

- 输入：原尺寸 `H×W×3`、`uint8`、RGB 图像；核心不固定输入尺寸或预处理方法。
- 返回 `(labels, confidence)`：均为原尺寸 `H×W`，适配器负责恢复缩放、裁剪后的像素对应关系。
- `labels`：整数类别编号 `0..65534`，`65535` 保留为未知。编号与含义由 `classes` 明确配置；未声明或低置信度像素变为未知。
- `confidence`：`float32/float64`，有限概率 `[0,1]`；非法输出、过期帧和像素尺寸不一致均拒绝。
- 例如某模型输出 `{7: road, 12: slope, 23: stairs}`，可以配置 `road_classes: [road]`。编号仅为接口示例，不是任何默认模型的类别表。
- 更换模型只需更换适配器、权重、类别表及 `options`；不修改几何检测、时间稳定过滤或 costmap 合并代码。

以下均为包内部 ROS 2 话题，可通过 `topic_remap` 配置，不新增对外 Robonix 能力约定：

| 参数 | 默认话题 | 消息 |
| --- | --- | --- |
| `camera_labels` | `/outdoor/vision/semantic_labels` | `sensor_msgs/Image`, `mono16`，保留完整类别 |
| `camera_confidence` | `/outdoor/vision/semantic_confidence` | `sensor_msgs/Image`, `32FC1` |
| `camera_road_mask` | `/outdoor/vision/road_mask` | `sensor_msgs/Image`, `mono8`：0 非道路、1 道路、255 未知 |
| `camera_segmentation_status` | `/semantic/segmentation_status` | `std_msgs/String` JSON，包含类别表、未知编号、源时间戳、状态 |
| `camera_overlay` | `/semantic/debug_overlay` | `sensor_msgs/Image`, `rgb8` |

三张输出图像保留原 RGB 的时间戳和坐标系。斜坡、楼梯等类别不会被压成道路后丢失，
但当前完整类别图只提供识别结果；本轮未实现基于三维坡度、阶高和底盘能力的可通行判定。
`road_classes` 仅影响几何已知地面的软代价，不能清除相机或雷达障碍，也不触发步态切换。
推理故障或过期结果按原有语义时效退出，几何链路继续工作；关闭语义时不启动推理和投影。

旧 PIDNet 专用配置和内置模型加载分支已移除，历史源码与验证材料归档到忽略的 `local_only/legacy_pidnet/`。

### 可选 SegFormer 适配器

设置 `backend: outdoor_nav.segformer_backend:create_backend`，并在导航 worker 的 Python 环境中安装本包的 `[segformer]` 可选依赖（例如 `python -m pip install '.[segformer]'`）。Jetson 应安装适配其 JetPack 的 PyTorch，再安装匹配的 Transformers；不启用此适配器时无需安装这些依赖。

此适配器要求 `weights` 为本地 Hugging Face `save_pretrained` 目录，包含模型权重（如 `model.safetensors`）、`config.json` 和 `preprocessor_config.json`。模型与图像处理器均须保存；仅有单独 `.pth` 文件不能直接使用此适配器。加载使用 `local_files_only=True`，缺失文件会报错，不在线补下载。

图像缩放、重采样、归一化等设置读取 `preprocessor_config.json`，不得用固定 ImageNet 参数替代训练时配置。当前适配器要求预处理保留完整视野，拒绝 crop/pad 配置；推理 logits 恢复原图尺寸后计算 softmax，返回原始类别编号和对应最大概率。`do_reduce_labels` 用于训练标签处理，不对预测类别再次减一。`options` 当前须为 `{}`。

根据所选权重的实际类别表填写 `classes` 和 `road_classes`，不要复制其他权重的编号。适配器不做二值化、不提供 `last_stairs`，所有类别走同一个输出接口。置信度是 softmax 分数，不等同于已校准的识别准确率。

官方模型示例：[NVIDIA SegFormer B0 Cityscapes 权重与配置](https://huggingface.co/nvidia/segformer-b0-finetuned-cityscapes-1024-1024/tree/main)。该示例不保证识别楼梯或坡道，也不是本包默认模型；自定义地形模型仍需提供对应权重和类别表。使用前核对所选模型卡和权重许可；导航代码的许可不替代模型许可。预处理接口见 [Transformers SegFormer 文档](https://huggingface.co/docs/transformers/model_doc/segformer)。

需安装原几何链路依赖 RTAB-Map、SciPy、Shapely 2、OpenCV、message_filters；模型运行依赖由适配器决定。
build.sh 会编译相机适配插件，start.sh 会加载包内 overlay。
相机原检测输出固定使用原 live-domain guard 所要求的 outdoor observation 端点。
关闭的传感器不参与依赖发现和健康门控；定位包不增加视觉里程计。

稳定性仍是可配置的时间/位置判据，不能保证识别所有运动物体；
短暂停止、慢速运动、簇合并或遮挡可能误判。现场过滤阈值须验收。
旧“高度裁切 + 二维扫描”方案和相应测试已撤下，不能再把其验证结果当作本链路验收。

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
- `docs/CONFIGURATION.md`、`docs/upstream-snapshot.json`：配置说明与源码来源记录。
- `local_only/`：本地测试、验证报告、备份与机器专用配置，由 `.gitignore` 排除，不随仓库分发。

参考服务结构：[syswonder/service-navigation-rbnx](https://github.com/syswonder/service-navigation-rbnx)。不直接复制其室内静态地图依赖、控制器插件、停止脚本或传感器默认参数。

## 构建与 Robonix 部署

运行目标：Ubuntu 22.04 / ROS 2 Humble，原生 Jetson 或本地 x86 ROS 环境。
需要 Robonix CLI/API（含 codegen，代码生成器须支持包内 capabilities）、Nav2 planner/controller/velocity smoother/lifecycle manager、pointcloud_to_laserscan、NumPy、PyYAML、Pillow。

```bash
source /opt/ros/humble/setup.bash
bash scripts/build.sh
```

启动解释器需安装 `robonix-api` 与本包 `[robonix]` 可选依赖，固定 `mcp>=1.27,<2.0.dev0` 和 `fastmcp>=3,<4`，避免 SDK 的 v1 FastMCP 接口被 mcp 2.x 替换；可用 `OUTDOOR_SERVICE_PYTHON` 指定 ROS-compatible Python/venv。

构建生成 gRPC/MCP bindings 和相机 costmap 插件 overlay，**不生成 Robonix ROS IDL overlay**。将 `config/deployment.example.yaml` 条目复制到机器人部署仓库；将完整的 `config/nav2_params.example.yml` 复制为部署侧 `config/outdoor_nav2.yaml` 并检查部署参数，再通过该部署 manifest 的 `rbnx build` / `rbnx boot` 启动。不要直接使用占位 provider 名称当作现成部署。

`start.sh` 只起 bridge；Driver INIT 才绑定依赖并起执行进程。外部反馈尚未就绪时不接受运动任务，activate 不能绕过健康门控。deactivate 与 stop 等待新鲜的速度反馈确认停车；服务停机只回收本实例进程组，不按通用 Nav2 进程名清理。

## 配置与外部依赖

`params_file` 相对部署 manifest 所在目录解析。`policy` 保留原现场执行策略，确认项默认关闭，不能为让启动通过而随意设为 true。

- `provider_ids.camera`：按相机包声明发现 RGB、深度、彩色 CameraInfo；配准深度 CameraInfo 仍通过 `topic_remap.camera_depth_info` 显式配置。关闭相机不发现这些输入。
- `provider_ids.cloud`：提供 `robonix/primitive/lidar/lidar3d` 的前雷达。
- `provider_ids.localization`：独立定位 service，同一实例提供 `robonix/service/localization/odom` 和 `.../status`；不将定位数据冒充底盘原语。`provider_ids.odom` 仍可单独指定 odom 角色。
- `topic_remap`：显式话题绑定覆盖 Atlas；显式值覆盖 Atlas；未显式绑定的定位状态与 Odometry 来自所选 localization service。
- `tf` / `tf_static`：已有 `map→odom→base_link` 动态树和传感器静态外参，服务不另造一套。
- `scan_projection`：雷达投影高度、距离和 TF 容差；相机使用原检测链路。
- 地图尺寸、足迹、分辨率及各层参数直接写在部署 Nav2 YAML 中；旧 `global_grid` 只做有警告的兼容迁移。
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

只请求 gait，不调用 posture、站起或使用模式切换。外部姿态指令改变步态时会停车重新确认。1 米缓冲不保证覆盖定位误差；本轮未验证真实上下楼梯。本地保留的隔离测试见 `local_only/README.md`。

## Scene 接入

Scene 使用本包的 `outdoor_nav.scene.SceneNavigation`，只接收当前建图服务的参考路线，不读取旧 bundle、地点文件或旧网页。部署由机器人仓库的 `outdoor/` 提供，地图与原始记录始终由建图服务保存。

导航请求记录最多保留 1024 个，每个最多 2 MiB；达到上限时拒绝新请求并提示归档，不静默删除幂等重试记录。旧网页地图/规划/障碍注入入口已从导航进程移除。

## 本地验证材料

测试脚本和验证报告集中保存在 `local_only/`，运行方法见该目录的 `README.md`。该目录不提交 Git，新克隆仓库不包含这些材料。部署用配置示例保留在 `config/`；机器专用配置可放在 `local_only/config/`，并在部署中显式引用对应路径。
