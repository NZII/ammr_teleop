# AMMR XR 遥操作迁移与重构执行方案

> 历史归档：本文记录 2026-08-31 的设计与执行方案。当前操作命令和目录结构请见
> [工程结构与部署](../../guides/工程结构与部署.md)。

日期：2026-08-31  
目标目录：`/home/ai/project/yj`  
建议新项目名：`ammr_teleop`

## 1. 目标

本次迁移不是简单更换目录，而是将当前分散在以下位置的代码整理成一套可维护、可部署的AMMR遥操作工程：

- `xr_teleoperate`中的XR输入、仿真遥操作、真机遥操作、录制、回放和控制中心；
- `/home/ai/project/jr/ammr_robot`中的Isaac Sim后端、IK和仿真资产；
- `/home/ai/project/jr/agilex_ws/src`中的真机ROS 2接口、机械臂、底盘、升降柱和相机源码。

最终需要同时支持：

1. 开发机上的Isaac Sim遥操作与录制；
2. 服务器上的真机XR、IK和控制中心；
3. AGX上的机械臂、底盘、升降柱、相机和本地数据录制；
4. 不部署Isaac Sim时，服务器和AGX仍能独立运行真机链路；
5. 仿真和真机共享相同的XR映射、IK、滤波、轨迹约束、14维数据契约和控制配置。

## 2. 核心设计结论

### 2.1 统一使用 `--sim` 选择仿真

启动器通过 `--sim` 选择仿真；不带 `--sim` 时默认为真机，也可显式写
`--real`。真机运动仍必须另外提供硬件输出授权：

```bash
./scripts/ammr.sh teleop --sim
./scripts/ammr.sh teleop --real
```

`--sim` 和 `--real` 互斥；仅凭 `--real` 不会开始运动：

```bash
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm right_arm lift chassis
```

### 2.2 Shell只负责环境和进程编排

统一Shell入口不承载映射、控制和录制业务逻辑。它只负责：

- 读取配置；
- 检查ROS、Python和Isaac环境；
- 根据`--sim/--real`启动对应Python入口；
- 在仿真模式下分别使用Isaac环境和XR环境启动两个进程；
- 通过`trap`完成有序退出；
- 将剩余参数原样转发给Python CLI。

仿真与XR当前使用不同Python环境，因此不应强行合并到一个Python进程。

### 2.3 真机部署和仿真部署分层

采用一个代码仓库、三个部署角色：

| 部署角色 | 内容 | 不包含 |
|---|---|---|
| 开发/仿真机 | 公共控制、XR、Isaac后端、完整仿真资产、仿真录制与回放 | AGX运行时进程 |
| 真机服务器 | 公共控制、XR、IK、真机ROS适配、控制中心、真机回放、最小运动学模型 | Isaac Sim、超市场景、相机驱动 |
| AGX | ROS驱动、三相机、升降柱、底盘、机械臂、本地录制器、远程进程管理器 | Isaac Sim、XR前端、IK求解器、完整仿真资产 |

部署时通过清单选择文件，不维护三份互相漂移的源代码。

## 3. 升降柱接口处理

### 3.1 必须复制的数据类型

必须将以下ROS 2接口包复制到新项目：

```text
ros2_ws/src/lifting_msg_pkg/
├── CMakeLists.txt
├── package.xml
├── msg/LiftMotorMsg.msg
├── srv/LiftInterfaceSrv.srv
└── srv/LiftMotorSrv.srv
```

当前XR真机节点实际使用：

```text
lifting_msg_pkg/srv/LiftMotorSrv
```

接口定义为：

```text
int64 val
int64 mode
---
int64 state
```

只复制`.srv`文件不够。ROS 2必须通过`rosidl`生成Python/C类型支持后，`rclpy`才能创建`/LiftingMotorService`客户端。

### 3.2 编译范围

- 服务器：必须编译`lifting_msg_pkg`，供真机XR节点导入服务类型；
- AGX：部署新工作区时也编译同一份`lifting_msg_pkg`，供升降柱服务节点使用；
- 仿真纯模式：不需要加载该overlay；
- 不需要为了接口包编译整个AgileX工作区。

不要复制旧的`build/`、`install/`和`log/`作为新工程的一部分。它们包含旧路径、旧Python解释器和CMake缓存。

建议提供统一构建脚本：

```bash
./scripts/build_ros_interfaces.sh
```

脚本内部执行：

```bash
source /opt/ros/humble/setup.bash
cd "${PROJECT_ROOT}/ros2_ws"

colcon build \
  --packages-select lifting_msg_pkg \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
```

编译后验证：

```bash
source ros2_ws/install/setup.bash
/usr/bin/python3 -c 'from lifting_msg_pkg.srv import LiftMotorSrv; print(LiftMotorSrv)'
```

### 3.3 升降柱驱动部署

`lifting_ctrl_ros2_pkg`是硬件驱动，只部署在AGX；服务器仅需要`lifting_msg_pkg`。两端必须来自同一份接口源码，避免服务类型不一致。

## 4. 建议目录结构

```text
/home/ai/project/yj/ammr_teleop/
├── pyproject.toml
├── README.md
├── configs/
│   ├── common.yaml
│   ├── sim.yaml
│   ├── real_server.yaml
│   ├── real_agx.yaml
│   └── hardware.yaml
├── scripts/
│   ├── ammr.sh
│   ├── build_ros_interfaces.sh
│   ├── deploy_server.sh
│   ├── deploy_agx.sh
│   └── verify_install.sh
├── src/ammr_teleop/
│   ├── common/
│   │   ├── contract.py
│   │   ├── kinematics.py
│   │   ├── pose_mapper.py
│   │   ├── trajectory.py
│   │   ├── profiles.py
│   │   └── event_logging.py
│   ├── xr/
│   │   └── frontend.py
│   ├── sim/
│   │   ├── teleop.py
│   │   ├── recorder.py
│   │   ├── replay.py
│   │   └── isaac_backend/
│   └── real/
│       ├── teleop.py
│       ├── hardware_interface.py
│       ├── recorder.py
│       ├── replay.py
│       ├── control_center.py
│       └── process_manager.sh
├── ros2_ws/src/
│   ├── lifting_msg_pkg/
│   └── lifting_ctrl_ros2_pkg/
├── hardware_ws/src/
│   ├── agx_arm_ros2/
│   ├── agx_bringup/
│   └── orbbec_ros2/
├── models/
│   ├── kinematics/
│   └── descriptions/
├── simulator/
│   ├── scripts/
│   └── assets/
├── third_party/
│   └── televuer/
├── deploy/
│   ├── server.manifest
│   ├── agx.manifest
│   └── sim.manifest
├── tests/
└── docs/
```

目录命名原则：

- 仓库名使用`ammr_teleop`，明确项目用途；
- 原`ammr_robot`不再作为独立模糊目录，拆成`simulator/`、`models/`和公共运动学代码；
- Python包统一为`ammr_teleop`；
- `ammr`继续作为机器人型号和ROS话题前缀，不必修改机器人名称；
- 第三方TeleVuer单独放在`third_party/`并记录当前版本或提交号。

## 5. 公共层与后端边界

### 5.1 公共层

以下代码必须由仿真和真机共同使用，禁止复制两份：

- 左右手柄及手部追踪输入；
- 左右腕姿映射；
- 绝对/增量映射；
- IK与正则化；
- One Euro滤波；
- 速度、加速度和jerk约束；
- 双臂14维数据契约；
- 控制profile；
- 日志事件格式。

### 5.2 仿真后端

仿真后端只负责：

- `/ammr/joint_command`和`/ammr/joint_states`；
- Isaac articulation读写；
- 物理步、渲染步和三路相机；
- 共享内存录制；
- 仿真场景和USD资产加载。

### 5.3 真机后端

真机后端只负责：

- 左右Piper ROS 2控制和反馈；
- 夹爪映射；
- `/cmd_vel`；
- `/LiftingMotorService`；
- AGX Recorder服务；
- 真机安全门和保持接口。

公共控制循环不直接知道Isaac或AGX进程的实现细节。

## 6. 统一启动器设计

推荐入口：

```bash
./scripts/ammr.sh <command> [--sim|--real] [options]
```

第一版支持：

```bash
# 遥操作
./scripts/ammr.sh teleop --sim
./scripts/ammr.sh teleop --real --enable-hardware-output \
  --hardware-components left_arm right_arm lift chassis

# 仿真 Recorder 随独立 Isaac 环境静态启动
../ammr_sim/scripts/ammr_sim.sh simulator

# 真机 Recorder（在 AGX 本机运行）
./scripts/ammr.sh recorder --domain-id 101 --task "pick fruit"

# 回放
./scripts/ammr.sh replay --sim --data-path episode.parquet
./scripts/ammr.sh replay --real --data-path episode.parquet --validate-only

# 控制中心
./scripts/ammr.sh control-center
```

仿真遥操作需要两个子进程：

```text
ammr.sh --sim
├── Isaac生产端（unitree_sim_env）
└── XR控制端（tv/teleop环境）
```

两个进程使用同一个`ROS_DOMAIN_ID`，启动器负责等待Isaac状态话题就绪；任一进程异常退出时，启动器应停止另一进程并给出日志路径。

真机模式不默认远程启动AGX硬件。第一阶段仍由控制中心管理AGX组件；后续可增加：

```bash
--manage-agx
```

用于显式请求远程启动和清理。

## 7. 配置与路径重构

### 7.1 消除硬编码路径

以下路径必须从源码中移除：

- `/home/ai/project/jr/ammr_robot`
- `/home/ai/project/zw/unitree_pro/unitree_avp/xr_teleoperate`
- `/home/ai/project/jr/agilex_ws/...`
- `/home/lin/yanjun/xr_teleoperate`
- `/home/lin/agilex_ws/install`
- `/home/lin/midea_humanoid_robot/install`

路径解析优先级统一为：

1. CLI参数；
2. 环境变量；
3. YAML配置；
4. 项目相对默认路径。

例如：

```text
AMMR_PROJECT_ROOT
AMMR_ASSET_ROOT
AMMR_DATA_ROOT
AMMR_LOG_ROOT
AMMR_ROS_WS
AMMR_AGX_ROOT
AMMR_TELEOP_PYTHON
AMMR_ISAAC_PYTHON
```

Python内使用`pathlib.Path`或`importlib.resources`解析项目资源，不使用用户名相关绝对路径。

### 7.2 资产配置

`configs/sim.yaml`中明确配置：

```yaml
assets:
  robot_usd: simulator/assets/robot/c2436/robot_v2_chassis.usd
  scene_usd: simulator/assets/scenes/Market001/Market001.usd
  kinematics_urdf: models/kinematics/c2436/robot_v2_chassis.urdf
```

启动时检查文件存在、可读并输出最终解析路径。大型场景和视频数据不进入普通Git，可采用：

- 本地外部目录加manifest和SHA256；或
- Git LFS；或
- 对象存储下载脚本。

第一阶段建议复制现有资产并记录校验和，等功能等价后再决定是否转Git LFS。

### 7.3 最小运动学模型

当前服务器IK间接依赖完整`ammr_robot`和大量可视化mesh。重构时应将IK改为读取最小运动学URDF，仅保留：

- 关节层级；
- joint axis；
- origin；
- joint limit；
- 左右末端frame。

服务器真机部署不应携带超市场景、USD和数百MB网格。完整模型只属于仿真部署。

## 8. 数据、日志和密钥

- 数据集默认放到仓库外，例如`/data/ammr/datasets`；
- 可用`AMMR_DATA_ROOT`覆盖；
- 日志放到`AMMR_LOG_ROOT`或用户cache目录；
- 不复制现有`teleop/utils/data`到代码仓库；
- 不在仓库中保存SSH和sudo明文密码；部署阶段改用SSH key或环境注入；
- 相机序列号、CAN口和AGX IP写入`configs/hardware.yaml`，不散落在Shell代码中。

建议配置示例：

```yaml
ros:
  domain_id: 101

agx:
  host: 192.168.3.99
  user: lin

server:
  host: 192.168.3.25

arms:
  left_can: can4
  right_can: can3

cameras:
  left_serial: CC1N16200TW
  right_serial: CC1WC5200R4
  head_serial: "130222250419"
```

## 9. 分阶段执行计划

### 阶段0：冻结与清单

1. 停止仿真、XR、录制器和回放进程；
2. 记录源文件清单、大小、时间戳和SHA256；
3. 保存当前启动命令和已验证profile；
4. 不删除或修改原目录。

验收：可以根据manifest确认迁移前后文件一致。

### 阶段1：复制当前可运行快照

复制：

- AMMR仿真/真机代码；
- TeleVuer；
- EpisodeWriter相关工具；
- AMMR测试和文档；
- `ammr_robot`源码、脚本和必需资产；
- AgileX、升降柱、底盘和Orbbec源码。

排除：

- `.git`、`__pycache__`和`.pyc`；
- 数据集和日志；
- ROS `build/install/log`；
- `Market001.zip`；
- 当前未使用的Unitree G1、OmniPicker和pan-tilt代码。

验收：文件数量、大小和SHA256与清单一致。

### 阶段2：原路径基线验证

先不改控制行为，只通过环境变量让复制后的代码使用新位置：

```bash
export AMMR_SIM_ROOT=/home/ai/project/yj/ammr_sim
export XR_TELEOP_ROOT=/home/ai/project/yj/ammr_teleop
```

依次验证：

1. 单元测试；
2. 仿真无相机启动；
3. 仿真三相机录制；
4. 真机DRY-RUN；
5. 真机单臂；
6. 真机双臂；
7. AGX录制；
8. 真机校验与回放。

验收：复制版行为与当前版本一致。

### 阶段3：配置和路径解耦

1. 引入`configs/`；
2. 替换所有`/home/ai`、`/home/lin`和`/media/hdda`硬编码；
3. 将数据、日志和资产路径独立；
4. 加入启动前路径与依赖检查；
5. 每完成一组修改运行测试和DRY-RUN。

验收：`rg '/home/ai|/home/lin|/media/hdda'`不再命中运行时代码。

### 阶段4：Python包重构

1. 建立`ammr_teleop.common`；
2. 合并仿真/真机重复profile和参数；
3. 建立`sim`与`real`后端边界；
4. 抽离最小运动学模型；
5. 保持ROS话题和14维数据格式不变。

验收：既有episode校验结果一致，仿真/真机IK输出在允许误差内一致。

### 阶段5：统一启动器

1. 新增`ammr.sh`；
2. 实现互斥的`--sim/--real`；
3. 仿真双环境进程编排；
4. 真机硬件显式使能；
5. 统一日志目录和退出清理；
6. 保留旧启动脚本作为一个版本的兼容包装器。

验收：旧命令与新命令分别执行同一测试，话题、频率和控制输出一致。

### 阶段6：部署拆分

生成三份manifest：

- `sim.manifest`
- `server.manifest`
- `agx.manifest`

部署脚本仅复制manifest列出的文件。服务器安装包不带Isaac资产，AGX安装包不带XR/IK，仿真安装包不带AGX驱动运行时。

验收：在不挂载原`jr`和`zw`目录的情况下，各角色独立启动。

### 阶段7：清理旧兼容层

只有完成全部验收后才执行：

- 删除旧入口别名；
- 删除过渡环境变量；
- 删除重复模型和旧路径兼容；
- 将旧项目改为只读归档。

## 10. 验收矩阵

| 测试 | 仿真 | 真机服务器 | AGX |
|---|---:|---:|---:|
| Python单元测试 | 是 | 是 | 录制相关 |
| XR连接与按键 | 是 | 是 | 否 |
| 双臂IK | 是 | 是 | 否 |
| 单臂分阶段输出 | 不适用 | 是 | 驱动开启 |
| 底盘/升降柱 | 是 | 是 | 驱动开启 |
| 三相机录制 | 是 | 状态控制 | 是 |
| 14维数据校验 | 是 | 是 | 是 |
| 原速回放 | 是 | 控制 | 执行 |
| 异常退出清理 | 是 | 是 | 是 |

## 11. 迁移期间禁止事项

- 不在复制和重命名同时修改控制算法；
- 不同时改变ROS话题、数据格式和目录结构；
- 不默认进入真机输出；
- 不复制旧ROS构建产物作为正式部署；
- 不删除原目录；
- 不在未完成DRY-RUN时启动双臂真机；
- 不将3GB场景和采集数据直接提交到普通Git。

## 12. 第一轮需要确认的决策

1. 新仓库名称是否确定为`ammr_teleop`；
2. `Market001`是否复制到新目录，还是暂时作为外部资产挂载；
3. Orbbec源码是否纳入主仓库，还是仅记录版本和下载地址；
4. 是否保留TeleImager作为参考模块；
5. 统一启动器是否采用“必须显式`--sim`或`--real`”；
6. 数据集根目录最终使用项目内路径还是独立`/data/ammr`路径；
7. 第一阶段是否继续沿用AGX现有安装，待服务器重构稳定后再迁移AGX。

## 13. 推荐执行顺序

推荐先完成：

```text
复制清单 → 文件复制 → lifting_msg_pkg构建 → 新路径基线运行
→ 路径配置化 → 公共/后端重构 → 统一启动器
→ 服务器部署清单 → AGX部署清单 → 删除兼容层
```

每一步都保持可回退到当前`jr`和`zw`目录，不进行一次性大改。
