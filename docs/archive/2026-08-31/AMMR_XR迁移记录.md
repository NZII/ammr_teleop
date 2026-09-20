# AMMR XR 迁移记录（2026-08-31）

> 历史归档：本文用于追溯首次迁移，不作为当前部署说明。当前文档入口见
> [AMMR Teleop 文档](../../README.md)。

## 迁移结果

- 新项目目录：`/home/ai/project/yj/ammr_teleop`
- 源清单：迁移完成后已随临时快照清理；以下摘要和SHA256保留用于追溯
- 清单SHA256：`8f601677cc07578f3de981b8cb82882aaf8a1e93e9e4c173f4516aadfea6fc5a`
- 清单数据项：1742
- 清单文件总字节数：4096921562
- 目标目录磁盘占用：约3.9 GiB
- 校验结果：1742项逐文件大小和SHA256全部匹配

## 当前目录映射

| 来源 | 新目录 | 说明 |
|---|---|---|
| `xr_teleoperate/teleop/ammr` | `src/ammr_teleop/common`、`src/ammr_teleop/sim` | 仿真公共契约、IK映射、录制与回放 |
| `xr_teleoperate/teleop/ammr_real` | `src/ammr_teleop/real` | 真机遥操作、录制、回放和控制中心 |
| `xr_teleoperate/teleop/televuer` | `third_party/televuer` | XR前端依赖 |
| `xr_teleoperate/teleop/teleimager` | `src/ammr_teleop/real/agx_image_relay.py` | 当前真机图像转发实现 |
| `ammr_robot/src` | `simulator/src` | Isaac Sim Python后端 |
| `ammr_robot/scripts` | `simulator/scripts` | Isaac启动和录制入口 |
| `ammr_robot/assets/.../c2436` | `simulator/assets/robot/.../c2436` | 当前C2436仿真模型 |
| `ranger_air_description`、`realsense2_description`、`agx_arm_description` | `simulator/assets/robot/Agilex_model/` | C2436 URDF的`package://`模型依赖，Pinocchio初始化必需 |
| `ammr_robot/assets/scene/Market001` | `simulator/assets/scene/Market001` | 当前超市场景 |
| `lifting_msg_pkg` | `ros2_ws/src/lifting_msg_pkg` | 升降柱ROS 2接口 |
| `lifting_ctrl_ros2_pkg` | `ros2_ws/src/lifting_ctrl_ros2_pkg` | AGX升降柱驱动 |
| `agx_arm_ros-ros2` | `hardware_ws/src/agx_arm_ros2` | Piper/AGX机械臂ROS 2源码 |
| `ranger_ros2-air_delta` | `hardware_ws/src/ranger_ros2_air_delta` | 底盘ROS 2源码 |
| `OrbbecSDK_ROS2` | `hardware_ws/src/orbbec_ros2` | 双腕Orbbec相机ROS 2源码 |
| `robot_des/c2436` | `models/descriptions/c2436` | 真机模型与映射核对资料 |

## 已排除内容

- 原仓库`.git`目录；
- `__pycache__`和`.pyc`；
- `teleop/utils/data`历史数据集；
- ROS 2的`build/install/log`；
- `ammr_robot`历史日志；
- 重复压缩包`Market001.zip`；
- Unitree G1、OmniPicker、pan-tilt等当前AMMR链路未使用代码。

## 当前状态与边界

本次仅完成原样快照，没有修改控制算法、ROS话题、14维数据契约、profile参数或硬件安全逻辑。原目录未删除、未修改，仍作为回退版本。

复制后已实际初始化`DualArmKinematics`，确认12关节Pinocchio/CasADi模型可以从新目录完整加载。

现有启动脚本中的绝对路径尚未解耦。在阶段2基线验证时至少设置：

```bash
export XR_TELEOP_ROOT=/home/ai/project/yj/ammr_teleop
export AMMR_SIM_ROOT=/home/ai/project/yj/ammr_sim
```

阶段3再统一迁移至YAML配置和项目相对路径。

## 当前校验命令

```bash
cd /home/ai/project/yj/ammr_teleop
./scripts/ammr.sh check --real
./scripts/ammr.sh check --sim
```

注：原逐文件迁移清单只用于首次复制验收，重构稳定后已删除。当前使用两个工程的
路径检查和自动化测试验证运行结构。
