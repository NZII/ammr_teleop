# AMMR XR 新路径基线验证（2026-08-31）

> 历史归档：测试数量和环境状态是 2026-08-31 的基线快照。当前验证方式见
> [路径配置与验证](../../reference/路径配置与验证.md)。

## 自动验证结果

| 项目 | 结果 |
|---|---|
| 迁移清单 | 1742项逐文件大小和SHA256一致 |
| AMMR单元测试 | 84 passed |
| `lifting_msg_pkg`构建 | 成功，使用系统`/usr/bin/python3` |
| `LiftMotorSrv`导入 | 成功 |
| `ros2 interface show` | `int64 val`、`int64 mode`、返回`int64 state` |
| 仿真XR入口参数解析 | 成功，`--help`共116行 |
| 真机XR入口参数解析 | 成功，`--help`共188行 |
| 新目录Isaac Python包导入 | 成功 |
| C2436场景、USD和URDF | 存在 |
| 双臂IK模型初始化 | 成功，`nq=12`，左右末端frame为105/106 |

单元测试使用`unitree_sim_env`执行。发现一项非阻断环境警告：当前NumPy为1.26.3，
已安装SciPy声明需要NumPy至少1.26.4。后续制作环境锁文件时应统一版本，本轮不直接修改已验证环境。

## 基线中发现并修复的问题

第一次IK初始化失败，因为C2436 URDF通过`package://`引用以下三个描述包：

- `ranger_air_description`；
- `realsense2_description`；
- `agx_arm_description`。

已把三个包补入`simulator/assets/robot/Agilex_model/`，重新生成迁移清单并完成逐文件校验。

## 新路径环境

```bash
cd /home/ai/project/yj/ammr_teleop

export ROS_DOMAIN_ID=101
export XR_TELEOP_ROOT=/home/ai/project/yj/ammr_teleop

source /opt/ros/humble/setup.zsh
source /home/ai/project/yj/ammr_teleop/ros2_ws/install/setup.zsh
```

## 仿真人工验证

终端一：

```bash
cd /home/ai/project/yj/ammr_sim
export ROS_DOMAIN_ID=0
./scripts/ammr_sim.sh simulator
```

终端二：

```bash
cd /home/ai/project/yj/ammr_teleop
export ROS_DOMAIN_ID=0

./scripts/ammr.sh teleop --sim --record \
  --profile piper_absolute_smooth_v1 \
  --input-mode controller \
  --display-mode pass-through
```

## 真机无输出验证

以下命令不带`--enable-hardware-output`，只验证ROS反馈、XR连接和IK，不发布真机命令：

```bash
cd /home/ai/project/yj/ammr_teleop
export ROS_DOMAIN_ID=101
source /opt/ros/humble/setup.bash
source /home/ai/project/yj/ammr_teleop/ros2_ws/install/setup.bash

./scripts/ammr.sh teleop --real \
  --profile piper_absolute_smooth_v1
```

## 真机输出验证边界

完成无输出验证后，再沿用原先已经验收的单臂、双臂测试顺序。硬件输出授权仍必须显式填写，
不会被profile或新路径环境自动打开：

```bash
./scripts/ammr.sh teleop --real \
  --profile piper_absolute_smooth_v1 \
  --enable-hardware-output \
  --hardware-components left_arm
```

阶段3路径重构应在上述仿真和真机基线通过后开始，避免把迁移问题与结构改造问题混在一起。
