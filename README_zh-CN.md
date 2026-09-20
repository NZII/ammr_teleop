# AMMR 统一 XR 遥操作

完整文档索引见 [`docs/README.md`](docs/README.md)。

本工程统一执行仿真和真机 XR 遥操作：

- `--sim`：连接已单独启动的 `../ammr_sim`；
- 不带 `--sim`：默认连接真机（也可显式写 `--real`）；
- `--record`：在仿真和真机模式下统一授权左 X/键盘 `s` 开始或停止 episode；
- 不带 `--record`：左 X 只提示 `REC_DISABLED`，不会产生数据。

```bash
# 仿真：先在 ammr_sim 单独启动 Isaac
./scripts/ammr.sh teleop --sim --record \
  --profile piper_absolute_smooth_v1

# 真机
./scripts/ammr.sh teleop --record \
  --profile piper_absolute_smooth_v1 \
  --enable-hardware-output \
  --hardware-components left_arm right_arm lift chassis
```

真实部署 manifest 会排除 `src/ammr_teleop/sim`、仿真 Shell 入口、Isaac
环境和仿真资产，因此 AGX/真机服务器不会导入仿真模块。

`configs/paths.yaml` 的本机相对路径全部以项目根目录为基准；AGX 相对路径在
远端按用户 `$HOME` 解析。仓库内 ROS 2 工作空间仅保留 `lifting_msg_pkg`，
机械臂、相机、底盘和升降柱驱动均由部署环境提供。
