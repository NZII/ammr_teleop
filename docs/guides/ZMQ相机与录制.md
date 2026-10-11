# AMMR ZMQ 相机与录制

## 默认端口

真机和仿真使用不同端口，因此可以同时运行：

| 相机 | 真机 AGX | 仿真服务器 |
|---|---:|---:|
| head | 55555 | 56555 |
| left_wrist | 55556 | 56556 |
| right_wrist | 55557 | 56557 |

每个消息是两段 ZMQ multipart：JSON metadata 和 JPEG。metadata 包含协议版本、相机名、帧序号、设备时间戳、采集单调时钟、尺寸；仿真还会让三路消息共享同一个 `bundle_sequence`。

## 真机相机

AGX 使用厂商 SDK 直接采集 RGB，不再启动 ROS 相机节点：

- head：RealSense D455，SN `130222250419`。
- left_wrist：Dabai DC1，SN `CC1N16200TW`，USB `1-4.1.1.2.4`。
- right_wrist：Dabai DC1，SN `CC1WC5200R4`，USB `1-4.1.3.4`。
- 三路均为 640x480@30 Hz。

首次部署或修改 C++ 发布器后，在 AGX 编译：

```bash
cd ~/yanjun/ammr_teleop
./scripts/build_camera_zmq_agx.sh
```

控制中心的 Head/Left/Right Camera 按钮现在分别启动三个 SDK ZMQ 发布器。AGX recorder 通过 `127.0.0.1` 订阅，服务器 XR 通过 `192.168.3.99` 订阅；远程订阅只保留最新帧，不会反压本地录制。

也可以在 AGX 上直接启动。必须从实际工程目录执行；管理脚本现在会自动识别
自身所在工程，因此无需再额外设置 `AMMR_AGX_ROOT`：

```bash
cd ~/yanjun/ammr_teleop
bash src/ammr_teleop/real/agx_process_manager.sh start head_camera
bash src/ammr_teleop/real/agx_process_manager.sh start left_camera
bash src/ammr_teleop/real/agx_process_manager.sh start right_camera
bash src/ammr_teleop/real/agx_process_manager.sh status
```

对应日志位于 `~/.cache/ammr_xr_ui/logs/head_camera.log`、
`left_camera.log`、`right_camera.log`。这里启动的是厂商 SDK→ZMQ 发布器，
不要再同时启动旧的 ROS2 相机节点占用同一设备。

`configs/paths.yaml` 中的 AGX 路径保持为相对 AGX `$HOME` 的路径：

```json
"agx_root": "yanjun/ammr_teleop",
"agx_data_dir": "yanjun/ammr_teleop/datasets/ammr_real_xr"
```

## 服务器显示

真机 XR 默认显示三路：

```bash
./scripts/ammr.sh teleop --real --image-stream --image-transport zmq
```

如果网络或解码负载需要降级为仅头部：

```bash
./scripts/ammr.sh teleop --real \
  --no-image-stream --head-image-stream --image-transport zmq
```

完全关闭图像显示：

```bash
./scripts/ammr.sh teleop --real --no-image-stream --no-head-image-stream
```

## 录制同步规则

机械臂 state/action 仍使用 ROS2，图像使用 ZMQ。Recorder 按配置频率采样，每次分别取各路缓存的最新值，不再按共同目标时间匹配或拒绝数据。某路暂时没有新消息时继续使用其最后值，并保存每路年龄、相机 sequence 和 `reused` 诊断；LeRobot 时间轴仍按 `frame_index / fps` 生成。

真机三台独立物理相机没有硬件触发，因此是时间戳软件对齐。仿真三路相机来自同一个渲染帧，并使用公共 `bundle_sequence`，仍为强同步。仿真 recorder 继续直接读取完整共享内存帧包，ZMQ 用于显示和外部消费者。

录制完成后可在服务器统一检查整个数据集：

```bash
cd /home/ai/project/yj/ammr_teleop
./scripts/ammr.sh inspect-dataset \
  --dataset-path /home/ai/project/zw/openpi/dataset/ammr_pick_place
```

Dashboard 的 `Actual capture Hz` 优先来自 `meta/episodes.jsonl` 的质量指标；旧数据
其次使用 `ammr.sync.target_timestamp`。新数据还展示各路源数据年龄与相机帧复用率。
完整用法见[数据集可视化与批量校验](数据集可视化与批量校验.md)。

## 单臂遥操作与双臂录制

单臂采集时仍要让左右臂驱动和输出都在线，只用 enable 参数选择跟随 XR 的
一侧。下例仅操控左臂，右臂通过同一套速度/加速度/jerk 限制平滑回到六轴和
夹爪零位，此后继续发布右臂 action；Recorder 因此仍保存完整同步的 14 维
state/action：

```bash
./scripts/ammr.sh teleop --real --record \
  --profile piper_absolute_smooth_v1 \
  --enable-hardware-output \
  --hardware-components left_arm right_arm \
  --left-enable --no-right-enable
```

仅操控右臂使用 `--no-left-enable --right-enable`；双臂操控是默认值，也可显式
写成 `--left-enable --right-enable`。不要通过删掉某个
`--hardware-components` 来选择单臂，否则该侧不会获得持续 action，Recorder
也会因缺少完整双臂 action/feedback 而拒绝样本。
