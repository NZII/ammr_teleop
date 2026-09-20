# AMMR XR 真机遥操作阶段任务记录

> 历史归档：本文保留当时的任务拆分，其中多项已经完成。当前真机操作说明见
> [真机 XR 遥操作](../../guides/真机XR遥操作.md)。

本文记录之前完整方案中的“第九部分：部署顺序”及当时实现状态。

  ## 第一阶段：Recorder 独立化

  目标是先把数据保存从服务器遥操作进程中移出去。

  步骤：

  1. 在 AGX 新增独立 Recorder 节点。
  2. 保留三路 RealSense ROS 图像。
  3. Recorder 本地订阅：
      - 左右臂 feedback
      - 左右臂 action
      - 头部相机
      - 左右腕相机

  4. 使用标准 ROS 2 Service 启停录制。
  5. Left X 从服务器异步调用 AGX Recorder。
  6. 数据保存到 AGX 本地磁盘。
  7. 录制与保存不阻塞服务器 100 Hz 控制。
  8. 录制期间锁定底盘和升降柱。
  9. Left Y、暂停、急停、反馈超时和退出时自动保存。

  当前状态：基本完成。

  另外后来补充完成了：

  - Recorder action/feedback 使用 BEST_EFFORT 旁路订阅。
  - MP4 Writer 独立进程化。
  - 头部预览默认关闭。
  - 服务器不再订阅腕部 raw 图像。

  ## 第二阶段：可选压缩图像回传

  目标是默认不占用服务器与 AGX 之间的图像带宽，需要时再实时查看三路画面。

  当前实现：

  1. AGX 相机和 Recorder 仍然只使用本地 raw ROS 图像。
  2. 控制中心新增 `Compressed Image Stream` 按钮。
  3. 按钮启动 AGX 独立 `agx_image_relay.py`，每路只保留一个待编码帧，
     编码为 JPEG 后发布 `.../image_raw/compressed`。
  4. 服务器 XR 以 `sensor_msgs/CompressedImage` 接收三路图像；头部画面继续
     用于 XR 预览（`immersive/ego` 模式），左右腕图像保留在服务器缓存中供
     后续界面或视觉模块使用。
  5. 关闭按钮只停止 relay，保留相机、机械臂和 Recorder 运行，控制链路不重启。
  6. 默认 JPEG 质量为 80、每路上限 30 Hz；relay 使用有界最新帧队列，避免
     编码积压拖慢控制。

  本阶段不引入 ZMQ、Image Server 或 teleimager 后端；录制链路仍使用 AGX 本地
  raw 图像，压缩回传只是独立旁路。

  ## 第三阶段：时间同步和数据质量

  这是接下来最重要的数据质量阶段。

  原计划步骤：

  1. 为每一种数据建立短时间环形缓冲：
      - 左臂 feedback
      - 右臂 feedback
      - 左臂 action
      - 右臂 action
      - 三路 RGB

  2. 以30 Hz产生目标采样时刻。
  3. 三路相机选择距离目标时刻最近的帧。
  4. action选择目标时刻之前最近的一条，避免使用“未来指令”。
  5. feedback选择最接近目标时刻的反馈。
  6. 设置时间门限：
      - 相机最大年龄
      - 三相机最大偏差
      - state最大年龄
      - action最大年龄

  7. 任意一路不满足要求时丢弃整组数据。
  8. 保存诊断时间：
      - 三相机源时间戳
      - state/action接收时间
      - camera skew
      - state/action age

  9. 统计：
      - 实际采样率
      - 重复帧
      - 过期帧
      - Producer drops
      - Writer drops

  10. 超过质量阈值时将 episode 标记为无效。

  当前状态：时间环形缓冲与目标时刻匹配已经实现。

  - 三路相机选择距离目标时刻最近的帧。
  - feedback选择距离目标时刻最近的反馈。
  - action只选择目标时刻之前最近的命令，禁止未来指令。
  - 支持state/action/camera年龄和camera skew门限。
  - Parquet保存逐帧`ammr.sync.*`诊断量。
  - `episodes_stats.jsonl`保存诊断量汇总。
  - 状态话题提供实际帧率、target skips、重复、拒绝和队列统计。

  基于统计阈值自动将episode标记为无效：已完成，待AGX真机验收。

  ## 第四阶段：编码与存储优化

  这部分只在性能测试表明有必要时执行。

  原计划步骤：

  1. Writer 从控制进程中分离。
  2. 使用有界完整帧队列。
  3. 队列满时丢弃整组三路图像，禁止部分帧。
  4. 使用预分配缓冲区。
  5. 必要时改成共享内存环形缓冲，避免进程间复制三路 raw 图像。
  6. 必要时使用 Jetson 硬件 H.264 编码。
  7. Parquet/metadata 批量写入。
  8. 使用 .in_progress 临时目录。
  9. 验证三个视频帧数和 Parquet 行数一致。
  10. 验证成功后原子移动到正式目录。
  11. 异常退出时保留可恢复的临时 episode。
  12. 保存结束后异步同步到服务器。

  当前状态：

  - 独立 Writer 进程：完成
  - 有界队列：完成
  - 整组丢帧：完成
  - 共享内存：未完成
  - Jetson H.264：未完成
  - 原子保存：已完成，待AGX真机验收
  - 自动数据校验：已完成，待AGX真机验收
  - AGX到服务器同步：未完成

  按照目前进度，接下来最合理的顺序是：

  1. 复测当前 Recorder 帧率和机械臂流畅度。
  2. 真机验收时间缓冲与同步匹配。
  3. 真机验收episode质量判定和原子保存。
  4. 实现AGX到服务器异步同步（AGX：192.168.3.99，服务器：192.168.3.25）。
  5. 图传功能暂缓；只有性能仍不足时，再做共享内存和Jetson硬件编码。

## 第五阶段：真机数据回放

当前状态：基础安全回放已经实现。

- `src/ammr_teleop/real/replay_real.py`严格读取并校验14维Piper episode。
- `scripts/ammr.sh replay --real`支持服务器或AGX系统Python启动。
- 默认DRY-RUN；硬件输出必须显式指定双臂组件。
- 硬件模式等待Space确认，并从当前反馈平滑进入首帧。
- 100 Hz插值并应用速度、加速度和jerk限制。
- Space暂停，`q`/Ctrl+C/反馈超时执行双臂保持。
- 不控制底盘、升降柱和相机。

真机分阶段验收已完成：validate-only、DRY-RUN、低速短片段、完整episode。
默认 DRY-RUN，必须显式添加 --enable-hardware-output。
  2. 校验 ammr_real_joint14_v1、14维顺序和关节范围。
  3. 等待左右臂与夹爪完整反馈。
  4. 启用 /left/control_enable 和 /right/control_enable。
  5. 从当前真机反馈平滑移动到 episode 第一帧，禁止直接跳变。
  6. 以100 Hz插值跟踪录制的 action。
  7. 使用与遥操作一致的速度、加速度、jerk限制。
  8. 支持 --speed、起止帧、Space暂停和q退出。
  9. 反馈超时、Ctrl+C或异常时执行双臂保持。
  10. 只回放双臂和夹爪，不控制底盘与升降柱。
  11. 支持 --validate-only，先检查数据但不发布。
  12. 真机输出前打印第一帧与当前反馈的最大关节差。

## 第六阶段：真机recorder环境部署过程
source /opt/ros/humble/setup.zsh

python3 -m pip install --user pyarrow

python3 -c "
import cv2
import numpy
import pandas
import pyarrow
import rclpy
import sensor_msgs.msg
import std_msgs.msg
import std_srvs.srv
print('AGX recorder system Python OK')
"

## 第七阶段：真机启动控制中心

当前状态：已实现服务器端浏览器控制中心和 AGX SSH 进程管理器，入口为
`scripts/ammr.sh control-center`。启动后访问 `http://127.0.0.1:8787`；网页控制中心
使用 Python 标准库 HTTP 服务，不依赖 Tk/X11 或额外 Web 框架。

```bash
./scripts/ammr.sh control-center
# browser: http://127.0.0.1:8787
```

- AGX 总开关负责清理遗留 ROS 进程、初始化并检查 `can2/can3/can4`。
- 左右臂、左右腕相机、头部相机、三路压缩图像回传、Recorder、底盘和升降柱均有独立启停按钮，控制中心界面统一使用英文显示。
- 服务器进程区提供独立的数据录制接口：启动 AGX Recorder 后，可调用
  `/ammr/recording/set_enabled` 开始/停止 episode，并在关闭总开关或退出前先请求保存。
- AGX 组件使用独立进程组、PID 文件和组件日志，停止时清理 launch 父子进程。
- 服务器 XR 遥操作有独立启停按钮。
- 默认 AGX 地址为 `lin@192.168.3.99`，服务器地址为 `192.168.3.25`。
- 当前部署默认使用 AGX 密码 `1212` 完成 SSH 和 sudo 认证；也可通过启动终端的
  `AMMR_AGX_PASSWORD` 环境变量覆盖，密码不会写入控制中心日志。首次开启图像回传时会自动把
  独立 relay 脚本同步到 AGX 的 `~/.cache/ammr_xr_ui/`。

待现场验收：AGX 重启后通过控制中心启动 CAN、双臂、三路相机、压缩图像回传和
Recorder，确认每个按钮的启动/停止、数据录制接口的 episode 开始/保存、重复启动
清理、三路 `.../compressed` 话题约 30 Hz，以及总开关关闭前先保存活动 episode。
