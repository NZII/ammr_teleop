# AMMR Teleop

AMMR 的统一 XR 遥操作工程：连接独立运行的 Isaac Sim，或连接真实机器人；同时
提供录制、回放、控制中心和数据集检查入口。

中文说明见 [`README_zh-CN.md`](README_zh-CN.md)，完整文档见
[`docs/README.md`](docs/README.md)。

## 环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- 仿真模式需要同级目录中的 `ammr_sim`
- 真机模式需要已经构建的 `agilex_ws` 驱动环境

安装 Python 依赖：

```bash
python -m pip install -e .
python -m pip install -r requirements.txt
```

如需使用 XR HTTPS 服务，在用户配置目录准备证书：

```bash
mkdir -p ~/.config/xr_teleoperate
openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
  -keyout ~/.config/xr_teleoperate/key.pem \
  -out ~/.config/xr_teleoperate/cert.pem \
  -subj "/CN=localhost"
```

## 快速开始

```bash
# 仅检查并显示实际启动命令
./scripts/ammr.sh teleop --sim --plan \
  --profile piper_absolute_smooth_v1

# 仿真遥操作；先启动 ../ammr_sim
./scripts/ammr.sh teleop --sim --record \
  --profile piper_absolute_smooth_v1

# 真机遥操作；硬件输出必须显式授权
./scripts/ammr.sh teleop --record \
  --profile piper_absolute_smooth_v1 \
  --enable-hardware-output \
  --hardware-components left_arm right_arm lift chassis
```

## 常用入口

```bash
./scripts/ammr.sh control-center
./scripts/ammr.sh inspect-dataset --dataset-path /path/to/dataset
./scripts/ammr.sh replay --sim --dataset-path /path/to/dataset
```

路径默认值位于 `configs/paths.yaml`；环境变量优先。仓库内 ROS 2 工作空间只保留
`lifting_msg_pkg` 源码，机械臂、相机、底盘和升降柱驱动由 `agilex_ws` 提供。

## 目录

```text
configs/      运行配置
docs/         架构与操作文档
models/       最小运动学模型
native/       相机原生组件
ros2_ws/src/  ROS 2 接口源码
scripts/      统一命令入口
src/          Python 核心代码
tests/        单元测试
third_party/  TeleVuer 依赖
```
