# AMMR XR 真机遥操作

> 新工程统一入口（2026-09-05）：从项目根目录运行
> `./scripts/ammr.sh teleop --real --profile piper_absolute_smooth_v1`。
> 硬件输出仍必须显式添加`--enable-hardware-output`和`--hardware-components`。
> 所有命令均使用当前统一入口；服务器/AGX 部署见
> [工程结构与部署](工程结构与部署.md)。

本入口只连接已经运行的真机 ROS 2 驱动，不主动编译或修改驱动工作空间。
仿真模式由 Isaac 提供同名的左右臂、底盘、升降柱和安全接口，因此两种模式
使用同一套遥操作发布实现；公开控制面不再使用仿真专属 21 维 Topic。

## 安全边界

- 默认是只读 `DRY-RUN`，不会发布机械臂、夹爪、底盘或升降柱命令。
- 真机输出必须同时提供 `--enable-hardware-output` 和明确的
  `--hardware-components`。
- 可选组件为 `left_arm`、`right_arm`、`chassis`、`lift`，用于逐项测试。
- 启用前必须收到左右臂和双夹爪完整反馈。
- `incremental` 映射在 Right B 恢复控制时以当前反馈和当前 XR 腕姿重新标定；`absolute` 映射始终使用固定 XR
  中性姿态及 AMMR 双臂零位参考，不会因 Right B 改变参考。
- Left Y（控制器内部字段为 `left_ctrl_bButton`，键盘备用键 `y`）令已启用的机械臂平滑回到
  六轴零位；完成后控制自动暂停。回零会先停止正在进行的数据录制。
- 暂停、双摇杆急停、（启用时的）反馈超时和退出都会关闭双臂控制门、保持双臂、
  发布底盘零速度并停止升降柱（仅作用于已启用的组件）。
- 当前远程 DDS 部署默认关闭反馈超时自动暂停；如需重新启用，显式传入
  `--feedback-timeout`。启用后，`--state-timeout`（默认 `0.25 s`）才会生效。
- 录制期间强制屏蔽底盘和升降柱运动。
- 三路 ZMQ 图像默认开启；每个订阅线程只保留最新帧，服务器显示变慢或断网时
  不会反压 AGX 本地采集和录制。可用 `--no-image-stream --head-image-stream`
  降级为仅头部显示。
- 默认将 TeleVuer 左右姿态直接映射到同名机械臂，且不会交换扳机和按键。
  如果诊断确认头显的姿态流左右标签反了，使用 `--swap-controller-poses`；
  不要仅依据站在机器人前后的位置猜测左右关系。

## ROS 2 接口

| 功能 | 接口 |
|---|---|
| 左臂命令/反馈 | `/left/control/joint_states`、`/left/feedback/joint_states` |
| 右臂命令/反馈 | `/right/control/joint_states`、`/right/feedback/joint_states` |
| 双臂控制门 | `/left/control_enable`、`/right/control_enable` |
| 双臂保持 | `/left/emergency_stop`、`/right/emergency_stop` |
| 底盘 | `/cmd_vel` |
| 升降柱 | `/LiftingMotorService`，速度模式 `-4`、停止模式 `-2` |
| 真机三路图像 | RealSense/Orbbec SDK 直接发布 ZMQ：head `55555`、left `55556`、right `55557` |
| 服务器图像显示 | 默认 `--image-stream --image-transport zmq` 接收三路；`pass-through` 只接收，`immersive/ego` 显示头部画面 |
| AGX本地三路录制 | Recorder 连接 `tcp://127.0.0.1:55555/55556/55557`，不经过服务器 |
| AGX录制控制/状态 | `/ammr/recording/set_enabled`、`/ammr/recording/status` |

服务器地址或端口变化时，可使用 `--head-zmq-endpoint`、
`--left-wrist-zmq-endpoint` 和 `--right-wrist-zmq-endpoint` 覆盖。

## 运行前提

先在终端中加载真机当前使用的 ROS 2 环境和自定义消息环境，再进入本项目。
启动脚本不会 source 或构建 `agilex_ws`。

建议真机使用独立 domain：

```bash
export ROS_DOMAIN_ID=101
```

## 一键启动控制中心

服务器端提供英文浏览器控制中心，将 AGX 的 CAN、双臂、三个 SDK ZMQ 相机进程、
Recorder、底盘和升降柱分别做成启停按钮，并提供服务器 XR 遥操作按钮。AGX 侧所有
进程由 SSH 管理器启动为独立进程组；再次启动某组件前会清理同名旧进程，停止时会同时清理
ROS launch 父子进程和受管 PID。AGX 总开关启动时会先清理遗留组件、初始化并核对
`can2/can3/can4`，只有 CAN 就绪后才开放各组件按钮；关闭总开关会停止所有 AGX 组件并尝试关闭
CAN 接口。

服务器已配置 SSH key 时直接运行：

```bash
cd /home/ai/project/yj/ammr_teleop
./scripts/ammr.sh control-center
```

启动后在浏览器打开 `http://127.0.0.1:8787`。网页使用系统标准 HTTP 服务和浏览器字体，
不依赖 Tk/X11，因此不会再受到终端或桌面中文字体缺失、缩放和像素化问题影响。可通过
`--host`、`--port` 修改监听地址和端口；例如需要从同一可信局域网的另一台机器访问时：

```bash
./scripts/ammr.sh control-center --host 0.0.0.0 --port 8787
```

此时在服务器本机访问 `http://127.0.0.1:8787`，从同一局域网的其他电脑访问
`http://192.168.3.25:8787`。`0.0.0.0` 模式没有登录认证，只应在可信网络使用。

旧的 `src/ammr_teleop/real/control_center.py` 仅作为桌面兼容实现保留；推荐入口脚本已切换到
`control_center_web.py`，不会再创建 Tk/X11 窗口。

控制中心默认保持 `pass-through`，三路图像会回到服务器但不覆盖头显透视画面。
若需要在 XR 中显示头部相机，可这样启动控制中心：

```bash
./scripts/ammr.sh control-center --display-mode immersive
```

默认连接：

```text
AGX：lin@192.168.3.99
服务器：192.168.3.25（本机）
ROS_DOMAIN_ID：101
AGX 工程配置：yanjun/ammr_teleop（相对 AGX 用户 $HOME）
```

使用顺序：

1. 点击 `AGX Master: Start`，等待状态变为 `Ready`。
2. 启动左右腕相机和头部相机；三个按钮分别启动一个 SDK ZMQ 发布器。数据采集前
   再启动 `AGX Recorder`。
3. 需要底盘或升降时，分别点击对应按钮；不需要时保持关闭。
4. 在 `XR teleoperation` 卡片勾选 `Left enable`、`Right enable`：默认两项都选；
   只选 Left 为左单臂，只选 Right 为右单臂。然后点击
   `Start Server XR Teleoperation`。启动后选项会锁定，必须先停止 Server XR 才能
   切换。无论选哪一侧，AGX 左右臂组件都应保持运行，以便未操控侧回零并保留完整
   双臂录制。服务器默认连接 AGX 的 `55555/55556/55557`，相机可以在 XR 运行期间
   分别启停，订阅端会自动等待恢复。
5. 关闭时先停止服务器 XR，再停止 AGX Recorder/相机/机械臂，最后关闭 AGX Master。

### 控制中心数据录制接口

AGX 组件区的 `AGX Recorder` 按钮只负责启动/停止录制进程；服务器进程区的
`Data Recording` 才负责当前 episode 的开始与保存：

1. 在 AGX 组件区点击 `Start AGX Recorder`，等待状态变为 `Running`。
2. 在 `Data Recording` 点击 `Start Episode`。按钮会通过 SSH 在 AGX 上调用
   `/ammr/recording/set_enabled`，服务成功后显示 `Recording episode_xxxxxx`。
3. 再次点击 `Stop & Save Episode`。Recorder 完成同步、质量检查和 Parquet 写入后，
   控制中心日志会显示服务响应及保存路径。
4. XR 运行期间仍可使用 Left X/键盘 `s` 切换录制；控制中心按钮与该快捷键不要
   同时连续点击。关闭 `AGX Master`、点击 `Stop All AGX Components` 或退出控制中心时，
   管理器会先请求停止并保存活动 episode，再清理 Recorder 进程。

录制接口要求 AGX 总开关和 AGX Recorder 均处于运行状态；录制期间底盘和升降柱
仍由服务器 XR 节点锁定。若服务返回失败，查看 AGX 的
`~/.cache/ammr_xr_ui/logs/recorder.log` 以及 `/ammr/recording/status`。

控制中心日志会显示 SSH 返回信息；每个 AGX 组件的详细日志保存在 AGX：

```text
~/.cache/ammr_xr_ui/logs/
```

相机日志分别为 `head_camera.log`、`left_camera.log` 和 `right_camera.log`。
控制中心本身只负责开关和状态，不在网页中解码图像；服务器 XR 的独立 ZMQ
线程完成解码，慢消费者只丢显示帧，不会阻塞 AGX publisher。

当前部署已将 AGX 登录和 sudo 密码设为 `1212`，点击 `AGX Master: Start` 时会自动完成
SSH 和 sudo 认证。密码不会写入控制中心日志。若后续修改密码，可在启动网页控制中心的同一个
终端用环境变量覆盖：

```bash
export AMMR_AGX_PASSWORD='新的AGX密码'
./scripts/ammr.sh control-center
```

密码认证由控制中心的临时 PTY 自动应答，不需要安装 `sshpass`。长期使用仍建议配置 SSH key：
`ssh-copy-id lin@192.168.3.99`；使用完成后可执行 `unset AMMR_AGX_PASSWORD`。
如果是首次连接该 AGX，先手动执行一次 `ssh lin@192.168.3.99` 并确认 host key；控制中心不会
自动接受未知主机密钥。

相机发布器首次部署或源码修改后需要在 AGX 编译一次：

```bash
cd ~/yanjun/ammr_teleop
./scripts/build_camera_zmq_agx.sh
```

发布器使用系统中的 librealsense、OrbbecSDK、OpenCV 和 libzmq；控制中心之后
直接启动已安装到 `install/camera_zmq/bin/` 的可执行文件。

## 第一阶段：只读验证

```bash
./scripts/ammr.sh teleop --real \
  --display-mode pass-through
```

预期日志包含：

```text
Real hardware output mode: DRY-RUN
Waiting for complete left/right arm and gripper feedback...
Ready: Right B/[r] enable-pause ...
```

此模式可以验证反馈、XR 标定、IK、相机和录制，但不会发出运动命令。

反馈超时保护当前默认关闭，因此不会因为服务器短暂收不到某一侧的反馈而自动暂停。
仍保留启动阶段的完整反馈等待、双摇杆急停、机械臂限位和 IK 连续失败保护。
如需进行安全保护 A/B 测试，可加入：

```bash
--feedback-timeout --state-timeout 0.50
```

## 第二阶段：单臂低速测试

正式单臂采集与早期“只启动一侧驱动”的硬件排查不同。采集时左右臂驱动必须
都启动，`--hardware-components left_arm right_arm` 必须保留；使用独立的
`--left-enable/--right-enable` 选择跟随 XR 的一侧。这样未操控侧会平滑回零，
但左右两侧 state/action 仍持续发布，Recorder 保存的仍是完整 14 维数据。

左臂：

```bash
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm right_arm \
  --left-enable --no-right-enable \
  --arm-velocity 0.20 \
  --operator-arm-length 0.65 \
  --robot-teleop-reach 0.13 \
  --wrist-rotation-gain 0.35
```

该命令只让左臂跟随左手柄。右臂仍等待真实反馈、打开控制门，并通过相同的
平滑控制链路回到六轴和夹爪零位；之后仍以 100 Hz 发布零位 action。建议先按
Right B 使能并等待右臂到达零位，再按 Left X 开始录制。

如需确认头显的姿态标签，先不启用硬件输出运行：

```bash
./scripts/ammr.sh teleop --real \
  --diagnose-controller-poses
```

一次只移动左手柄或右手柄。日志中 `TeleVuer left/right Δ` 最大的一侧就是
当前物理手柄写入的姿态流。物理左手柄若只让 `right Δ` 增大，才需要在正式
命令中加入 `--swap-controller-poses`。

右臂测试使用 `--no-left-enable --right-enable`，不要删除任一 arm hardware
component。确认两侧单独安全后再运行双臂默认模式：

```bash
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm right_arm \
  --arm-velocity 0.20 \
  --operator-arm-length 0.65 \
  --robot-teleop-reach 0.13
```

双臂命令始终包含 `joint1`～`joint6` 和 `gripper`，不会发送部分关节。

### 绝对腕部映射与 Left Y 回零

绝对映射需在双臂增量模式已安全验收后使用：它不以启用时的手柄位置为零点，
而是将 XR 固定中性姿态映射到 AMMR 双臂六轴零位。

```bash
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm right_arm \
  --wrist-mapping absolute \
  --arm-velocity 0.20 \
  --operator-arm-length 0.65 \
  --robot-teleop-reach 0.13 \
  --wrist-rotation-gain 0.35
```

启用后按 Left Y 可让当前已启用臂在 `--arm-reset-duration`（默认 2.5 秒）内
平滑回到六轴零位。回零期间忽略腕部 IK，仍保留夹爪扳机；到位后自动暂停，
需按 Right B 才能恢复控制。单臂阶段只回该单臂。

### 抖动诊断（不改变控制行为）

使用 `--jitter-log` 可记录同一控制周期内的原始 XR 腕姿、滤波前/后的任务空间
目标、IK 解、已发布命令和真机反馈。该选项不改变滤波、IK 或硬件输出参数。

```zsh
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm \
  --arm-velocity 0.20 \
  --operator-arm-length 0.65 \
  --robot-teleop-reach 0.13 \
  --jitter-log /tmp/ammr_left_jitter.csv
```

按 Right B 启用后，依次采集：手柄静止 10 秒、缓慢平移 10 秒、缓慢转腕 10 秒，
再按 Right B 暂停。CSV 每列以 `xr_`、`target_raw_`、`target_filtered_`、`ik_q_`、
`command_q_`、`feedback_q_` 区分链路位置，并包含 `ik_solve_ms`。将该 CSV 提供给
分析即可确定抖动来自输入、映射/滤波、IK 或真机跟踪。

### 多频率控制架构

真机控制链路默认使用分频调度：

| 环节 | 参数 | 默认频率 |
|---|---|---:|
| XR目标采样、任务空间滤波和IK | `--xr-frequency` | 50 Hz |
| jerk轨迹推进和ROS关节命令发布 | `--command-frequency` | 100 Hz |
| 头部画面渲染 | `--display-frequency` | 30 Hz |
| 三相机和关节数据录制 | `--recording-frequency` | 30 Hz |

`--frequency` 保留为 `--xr-frequency` 的兼容别名。IK只在新一轮XR目标到来时
求解；两个IK目标之间由100 Hz速度/加速度/jerk受限轨迹连续推进，不会对同一个
XR帧重复求解。显示和录制也不会再把机械臂发布频率限制在30 Hz。

当前CasADi IK的常见求解时间约为2 ms，因此50 Hz足够。这里没有照搬Neuracore
的250 Hz Pink差分IK：两种求解器的输出语义不同。若100 Hz周期偶尔被极端IK
耗时占用，再将IK移入独立工作线程；在实测表明需要之前不引入共享状态竞态。

#### 推荐配置档：piper_absolute_smooth_v1

生效时间：`2026-08-15 10:54:35 CST（Asia/Shanghai）`

当前左臂最优参数已经固化为版本化配置档
`src/ammr_teleop/common/profiles/piper_absolute_smooth_v1.json`。配置档当前使用直接平移
增益 `1.10` 和旋转增益 `0.55`，不再用人体臂长参数兼任灵敏度旋钮。
仿真XR入口也直接加载这一配置档，因此映射、滤波、IK和轨迹参数只有一份参数源；
真机硬件授权参数仍只存在于真机启动命令中。

日常左臂启动命令缩短为：

```zsh
cd /home/ai/project/zw/unitree_pro/unitree_avp/xr_teleoperate

./scripts/ammr.sh teleop --real \
  --profile piper_absolute_smooth_v1 \
  --enable-hardware-output \
  --hardware-components left_arm \
  --jitter-log /tmp/ammr_left_profile_v1.csv
```

配置档固定以下已验收参数：绝对腕部映射、XR/IK 50 Hz、命令100 Hz、显示与录制
30 Hz、平移增益1.10、旋转增益0.55、最大腕部位移0.50 m、IK正则化、One Euro
滤波、机械臂速度/加速度/jerk以及夹爪轨迹限制。

命令行显式参数总是覆盖配置档。日常建议只调以下四项：

```zsh
--translation-scale 1.10
--wrist-rotation-gain 0.55
--max-wrist-translation 0.50
--arm-velocity 0.50
```

例如精细操作可运行：

```zsh
./scripts/ammr.sh teleop --real \
  --profile piper_absolute_smooth_v1 \
  --enable-hardware-output \
  --hardware-components left_arm \
  --translation-scale 1.00 \
  --wrist-rotation-gain 0.45 \
  --max-wrist-translation 0.40 \
  --arm-velocity 0.40 \
  --jitter-log /tmp/ammr_left_precision.csv


./scripts/ammr.sh teleop --real \
    --profile piper_absolute_smooth_v1 \                                              
    --enable-hardware-output \
    --hardware-components left_arm right_arm \
    --task "XR双臂遥操作数据采集"

```

`--enable-hardware-output` 和 `--hardware-components` 故意不写入配置档，必须由
操作者每次显式授权。底层参数仍可从命令行覆盖，用于受控A/B测试。

启用 `--jitter-log PATH` 后会同时生成：

```text
PATH
PATH.meta.json
```

元数据JSON记录配置档名称/版本、启动时间、最终生效的全部参数、实际平移增益、
完整命令行和Git版本/工作区状态。分析CSV时应始终同时保留对应的元数据文件。

#### 真机测试基线：绝对映射、多频率控制

记录时间：`2026-08-08 20:40:48 CST（Asia/Shanghai）`

测试目标：左臂绝对位置映射，XR/IK 50 Hz、轨迹与ROS命令100 Hz，验证Piper
连续运动、前伸范围和命令跟踪延迟。测试日志固定保存为
`/tmp/ammr_left_multirate_50_100.csv`。

```zsh
cd /home/ai/project/zw/unitree_pro/unitree_avp/xr_teleoperate

./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm \
  --wrist-mapping absolute \
  --xr-frequency 50 \
  --command-frequency 100 \
  --display-frequency 30 \
  --recording-frequency 30 \
  --operator-arm-length 0.50 \
  --robot-teleop-reach 0.65 \
  --wrist-rotation-gain 0.60 \
  --max-wrist-translation 0.50 \
  --ik-motion-regularization 0.40 \
  --ik-posture-regularization 0.01 \
  --pose-filter one_euro \
  --one-euro-min-cutoff 0.8 \
  --one-euro-beta 5.0 \
  --one-euro-d-cutoff 0.9 \
  --arm-velocity 0.50 \
  --arm-acceleration 1.60 \
  --arm-jerk 5.0 \
  --gripper-velocity 0.10 \
  --gripper-acceleration 0.40 \
  --gripper-jerk 2.0 \
  --jitter-log /tmp/ammr_left_multirate_50_100.csv
```

IK 默认以最新真机反馈作为 warm-start；`--ik-feedback-seed-max-error`（默认
`0.12 rad`）限制每个关节相对上一命令允许采用的最大反馈修正量，以隔离陈旧或
突变的反馈。诊断 CSV 的 `ik_seed_q_` 列可验证实际使用的 seed。

### 人体臂长、平移增益与旋转增益

`--operator-arm-length` 是操作者从肩关节中心到手柄中心的实测距离，默认
`0.65 m`；`--robot-teleop-reach` 是操作者移动一个臂长时，希望机器人腕部移动的
距离，默认 `0.13 m`。实际平移增益为：

```text
translation gain = robot-teleop-reach / operator-arm-length
                 = 0.13 / 0.65 = 0.20
```

因此手柄平移 `10 cm`，机器人任务空间目标平移 `2 cm`。人体臂长越长，在相同
`robot-teleop-reach` 下增益越低。`--translation-scale` 是旧的直接增益参数；一旦
传入就覆盖上述臂长标定，例如 `--translation-scale 0.25` 表示手柄移动 `10 cm`，
机器人目标移动 `2.5 cm`。正常标定测试不要同时传它。

`--wrist-rotation-gain`（兼容旧名 `--rotation-scale`）只控制旋转，默认 `0.35`。
例如手腕相对旋转 `30°`，机器人腕部目标旋转约 `10.5°`。它与长度无关，也不会
改变平移灵敏度。

IK 对全部 12 个双臂关节进行正则化，不局限于第 3、4 关节：

- `--ik-motion-regularization 0.20`：惩罚本次解偏离反馈 warm-start，抑制相邻解跳变。
- `--ik-posture-regularization 0.01`：轻微偏好中性关节姿态，减少冗余解漂移。

两项可分别设为 `0` 做 A/B 对照。它们作用于优化目标，不是四帧均值滤波，不会
重新引入已移除的四帧延迟。

为与现有固定 α 滤波作 A/B 比较，可显式启用 1€ 自适应滤波：

```zsh
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm \
  --arm-velocity 0.20 \
  --operator-arm-length 0.65 \
  --robot-teleop-reach 0.13 \
  --wrist-rotation-gain 0.35 \
  --pose-filter one_euro \
  --one-euro-min-cutoff 0.8 \
  --one-euro-beta 5.0 \
  --one-euro-d-cutoff 0.9 \
  --jitter-log /tmp/ammr_left_one_euro.csv
```

真机当前默认使用 `one_euro`；如需回退对照，传入 `--pose-filter ema`。两种模式
互斥，不会叠加。

真机和仿真XR链路均固定旁路 AMMR IK 的 `[0.4, 0.3, 0.2, 0.1]`
四帧关节解滤波。真机命令行不再提供
`--ik-solution-filter/--no-ik-solution-filter`，启动时无需再传该参数。真机使用基于实际
`dt` 的速度和加速度限制平滑命令：

```zsh
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm \
  --arm-velocity 0.20 \
  --operator-arm-length 0.65 \
  --robot-teleop-reach 0.13 \
  --wrist-rotation-gain 0.35 \
  --ik-motion-regularization 0.20 \
  --ik-posture-regularization 0.01 \
  --pose-filter one_euro \
  --arm-acceleration 0.80 \
  --arm-jerk 4.0 \
  --jitter-log /tmp/ammr_left_jerk_limit.csv
```

`--arm-velocity` 限制最大速度，`--arm-acceleration` 限制速度变化率；夹爪对应
`--gripper-velocity`、`--gripper-acceleration` 和 `--gripper-jerk`。
`--arm-jerk` 默认为 `4.0 rad/s³`，用于限制加速度的翻转速度。诊断 CSV 的
`command_velocity_q_*` 和 `command_acceleration_q_*` 分别保存限幅后的
速度和加速度。反馈 warm-start 始终保留。
当IK目标突然反向时，命令会先按加速度上限减速、过零后再反向，
不会直接吸附到新目标。只有在同时满足速度和加速度约束时才精确落到目标。

## 第三阶段：底盘测试

```bash
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components chassis
```

映射为左摇杆前后/横移、右摇杆 X 左右转。默认限制为 `0.20 m/s`、
`0.15 m/s` 和 `0.30 rad/s`。此阶段不需要启动任一机械臂节点，也不会等待
机械臂反馈；进入 XR 后按 Right B 启用/暂停底盘输出。

## 第四阶段：升降柱方向验证

### 首次准备：在 zsh 中构建消息接口

`lifting_msg_pkg` 是 ROS 2 接口包，不能只将其源码目录加入 `PYTHONPATH`。它必须
由 ROS 2 的 `rosidl` 生成 Python 类型；只需构建这个包，不需要编译整个
`agilex_ws`。

Conda 的 Python / EmPy 4 会与 ROS Humble 的消息生成器冲突。因此在新的 zsh
终端中先退出 Conda，再强制 CMake 的两项 Python 变量使用系统 Python：

```zsh
conda deactivate
source /opt/ros/humble/setup.zsh

cd /home/ai/project/jr/agilex_ws/src/lifting_ws_ros2-master

colcon build \
  --packages-select lifting_msg_pkg \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args \
    -DPython3_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3
```

构建前 `which python3` 应输出 `/usr/bin/python3`。构建成功后验证生成的服务类型：

```zsh
source install/setup.zsh
/usr/bin/python3 -c 'from lifting_msg_pkg.srv import LiftMotorSrv; print(LiftMotorSrv)'
```

### 每次启动升降柱 XR 前加载 overlay

XR 启动脚本不会自动 source 真机工作空间；每个新的 zsh 终端都先执行：

```zsh
source /opt/ros/humble/setup.zsh
source /home/ai/project/jr/agilex_ws/src/lifting_ws_ros2-master/install/setup.zsh
```

确认真机服务实际使用同一服务类型：

```zsh
ros2 service type /LiftingMotorService
```

预期为 `lifting_msg_pkg/srv/LiftMotorSrv`。确认后启动：

```zsh
cd /home/ai/project/zw/unitree_pro/unitree_avp/xr_teleoperate

./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components lift \
  --lift-speed 20
```

右摇杆 Y 控制升降。第一次测试使用低速，并现场确认正负方向、上下限位及
实际电机 ID。回中时发送 `mode=-2` 停止。此阶段同样不需要机械臂节点；进入
XR 后按 Right B 启用/暂停升降输出。

## 完整遥操作

所有组件分别验收后：

```bash
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm right_arm chassis lift
```

按键：Right B/`r` 启用或暂停，Left X/`s` 开始或停止录制，Left Y/`y` 回零，
双摇杆按下紧急保持，Right A/`q` 安全退出。

## 数据录制

数据由 AGX 上的独立 Recorder 进程保存。三个厂商 SDK publisher 同时服务本机
Recorder 和服务器 XR：Recorder 走 loopback，服务器显示走局域网。ZMQ PUB 使用
小型发送队列且不等待订阅端，因此服务器网络或显示负载不会阻塞本地录制。

可在服务器用公共客户端检查三路回传；现场实测三路总带宽约 29.4 Mbps，单路
接收接近 30 Hz：

```bash
PYTHONPATH=src ~/miniconda3/envs/tv/bin/python -c \
  'from ammr_teleop.common.camera_zmq import CameraSubscriberSet; print("ZMQ client ready")'
```

手动启动服务器 XR 并接收三路 ZMQ 图像时：

```bash
./scripts/ammr.sh teleop --real \
  --enable-hardware-output \
  --hardware-components left_arm right_arm \
  --image-stream \
  --image-transport zmq
```

如果网络或服务器解码资源受限，可只接收头部：

```bash
./scripts/ammr.sh teleop --real \
  --no-image-stream --head-image-stream --image-transport zmq
```

完全关闭显示使用 `--no-image-stream --no-head-image-stream`。这些选项只影响
服务器显示，不影响 AGX Recorder。先在 AGX 启动 Recorder：

```bash
export ROS_DOMAIN_ID=101

./scripts/ammr.sh recorder --domain-id 101 \
  --recording-frequency 30 \
  --sync-delay 0.04 \
  --history-duration 1.0 \
  --camera-max-age 0.20 \
  --camera-max-skew 0.08 \
  --state-max-age 0.25 \
  --action-max-age 0.25 \
  --quality-min-frames 30 \
  --quality-min-fps 20 \
  --quality-max-rejected-ratio 0.30 \
  --quality-max-duplicate-ratio 0.30 \
  --quality-max-target-skip-ratio 0.05 \
  --quality-max-producer-drops 0 \
  --writer-queue-size 16 \
  --data-dir ~/yanjun/ammr_teleop/datasets/ammr_real_xr \
  --task "XR双臂遥操作数据采集"
```

Left X/`s`通过标准`std_srvs/srv/SetBool`异步调用
`/ammr/recording/set_enabled`。开始、停止和AGX保存期间不会阻塞服务器100 Hz控制
循环；从开始请求到保存完成期间，底盘和升降柱保持锁定。Left Y回零、控制暂停、
反馈超时和退出会先请求AGX停止并保存当前episode。

### 运行时事件日志

真机XR终端使用与Unitree一致的Rich日志布局，记录
`SESSION_START`、`CONFIG`、`ROS_READY`、`IK_READY`、`XR_CONNECTED`、
`CONTROL_ENABLE/CONTROL_PAUSE`、`REC_START/REC_SAVED`、`ARM_RESET_*`、
`FEEDBACK_TIMEOUT`、`EMERGENCY_HOLD`和`SHUTDOWN`。每条事件包含本地时间、
等级、运行模式、事件编码及源文件行号。

日志同时写入控制台和滚动文件，默认目录为：

```text
/tmp/ammr_real_xr_ros2_logs/ammr_real_YYYYMMDD_HHMMSS_pPID.log
```

可使用`AMMR_EVENT_LOG_DIR=/path/to/logs`替换默认目录。当前只记录状态切换和异常，
不打印逐帧关节、IK或相机数据，避免影响真机控制周期。

数据默认保存在AGX Recorder启动命令的`--data-dir`：

```text
~/yanjun/ammr_teleop/datasets/ammr_real_xr/
```

采用 `ammr_real_joint14_v1` 契约：左臂 6、左夹爪 1、右臂 6、右夹爪 1。
每帧保存实际反馈 `observation.state`、实际已发布目标 `action`，以及头部、
左腕和右腕三路 RGB。Recorder 使用1秒短时环形缓冲，以固定30 Hz产生目标时刻：
三路相机和feedback选择最近样本，action只选择目标时刻之前最近的命令，避免把
未来指令与当前观测配对。`--sync-delay 0.04`允许匹配目标时刻前后到达的相机帧，
只增加录制链路延迟，不影响遥操作控制。任意一路完全缺失或超过
`--camera-max-age`才拒绝该目标。默认不因三相机 skew 超限而丢弃；如确需严格模式，
显式使用 `--strict-camera-skew`。

相机在相邻 30 Hz 目标点没有新帧时会复用最近有效帧，不再丢弃整条 state/action。
每路 `sequence` 和 `reused` 都写入诊断列，因此既保持动作时间轴连续，又能在训练前
评估每台相机的独立有效帧率。

Parquet逐帧保存`ammr.sync.*`诊断列，包括目标时间、匹配延迟、三相机源时间戳、
各路相对目标时刻的偏差、action age和camera skew；`episodes_stats.jsonl`保存这些
诊断量的count/min/max/mean/std汇总。Recorder状态还提供实际录制帧率、目标时刻
跳过数、拒绝数、重复数、producer drops和writer队列深度。

停止录制时会根据命令行中的`--quality-*`阈值自动判定episode质量。质量不合格的
episode仍然保存，但在`episodes.jsonl`、`episodes_stats.jsonl`和Recorder状态中
记录`quality.valid=false`及具体原因，不应进入训练数据集。

保存过程先写入：

```text
~/yanjun/ammr_teleop/datasets/ammr_real_xr/.in_progress/episode_XXXXXX/
```

Writer关闭三路视频后，会检查Parquet行数及三路MP4帧数完全一致。全部通过才将
文件移动到正式`data/chunk-000`和`videos/chunk-000`目录，并原子更新metadata。
验证或写盘失败时保留`.in_progress`目录，不覆盖已有episode，方便后续恢复和排查。

开始录制时若检查失败，AGX日志会逐路输出最近帧年龄；从未收到的相机显示
`age=missing`，超过阈值的相机可直接从毫秒数识别，例如：

```text
Recording rejected:
  head age=35ms
  left_wrist age=620ms
  right_wrist age=42ms
```

LeRobot 输出包括：

```text
data/chunk-000/episode_XXXXXX.parquet
videos/chunk-000/observation.images.head/*.mp4
videos/chunk-000/observation.images.left_wrist/*.mp4
videos/chunk-000/observation.images.right_wrist/*.mp4
meta/info.json
```

## 数据集检查与可视化

录制完成后，优先使用数据集级检查入口，而不是逐条执行回放器的
`--validate-only`。在保存或已同步数据集的服务器上运行：

```bash
cd /home/ai/project/yj/ammr_teleop
./scripts/ammr.sh inspect-dataset \
  --dataset-path /home/ai/project/zw/openpi/dataset/ammr_pick_place
```

默认打开一个复用所有 episode 的 Dashboard，不会给每条数据生成单独图片。
服务器本机访问 `http://127.0.0.1:8050`。需要从同一可信局域网访问时：

```bash
./scripts/ammr.sh inspect-dataset \
  --dataset-path /home/ai/project/zw/openpi/dataset/ammr_pick_place \
  --host 0.0.0.0 \
  --port 8050
```

只做批量契约校验、不启动网页：

```bash
./scripts/ammr.sh inspect-dataset \
  --dataset-path /home/ai/project/zw/openpi/dataset/ammr_pick_place \
  --validate-all \
  --no-web
```

`--dataset-path` 应优先指向包含 `data/`、`videos/`、`meta/` 的数据集根目录。
Dashboard 同时显示所有 episode 的 Contract/Quality、Actual capture Hz、拒绝率、
重复率、动作跳变热力图，以及所选 episode 的 14 路 state/action 和误差曲线。
详细说明见[数据集可视化与批量校验](数据集可视化与批量校验.md)。

## 真机数据回放

真机回放只支持`ammr_real_joint14_v1`的LeRobot parquet，并固定控制双臂6+6和
双夹爪1+1；不回放底盘、升降柱或相机。回放前必须退出XR遥操作，禁止两个进程
同时向`/left/control/joint_states`和`/right/control/joint_states`发布命令。

下面的 `replay --validate-only` 是“准备回放某一个 episode”时的第二层安全检查，
不能替代上面的整个数据集检查。它不初始化真机控制。对于严格符合契约的新数据，不需要
`--clamp-limits`：

```bash
./scripts/ammr.sh replay --real \
  --data-path ~/yanjun/ammr_teleop/datasets/ammr_real_xr/data/chunk-000/episode_000000.parquet \
  --validate-only
```

如果是旧录制且出现夹爪宽度略微超过 `0.10 m`，必须显式启用兼容限幅。限幅只
作用于内存中的回放轨迹，不会改写原始 parquet；校验输出会报告被限幅的帧数：

```bash
./scripts/ammr.sh replay --real \
  --data-path ~/yanjun/ammr_teleop/datasets/ammr_real_xr/data/chunk-000/episode_000002.parquet \
  --validate-only \
  --clamp-limits
```

第二步执行DRY-RUN。它会读取真机反馈、计算完整轨迹和时序，但不发布控制命令：

```bash
export ROS_DOMAIN_ID=101

./scripts/ammr.sh replay --real \
  --data-path ~/yanjun/ammr_teleop/datasets/ammr_real_xr/data/chunk-000/episode_000002.parquet \
  --clamp-limits \
  --speed 0.5 \
  --start-index 0 \
  --end-index 100
```

首次真机测试使用低速、短片段：

```bash
./scripts/ammr.sh replay --real \
  --data-path ~/yanjun/ammr_teleop/datasets/ammr_real_xr/data/chunk-000/episode_000002.parquet \
  --enable-hardware-output \
  --hardware-components left_arm right_arm \
  --clamp-limits \
  --speed 0.5 \
  --start-index 0 \
  --end-index 100 \
  --arm-velocity 0.20 \
  --arm-acceleration 0.80 \
  --arm-jerk 4.0
```

常规完整回放直接使用采集 profile 的默认限制。`--speed 1.0`、`100 Hz` 命令频率、
机械臂与夹爪的速度/加速度/jerk 均已有默认值，不需要在命令中重复填写：

```bash
export ROS_DOMAIN_ID=101

./scripts/ammr.sh replay --real \
  --data-path ~/yanjun/ammr_teleop/datasets/ammr_real_xr/data/chunk-000/episode_000002.parquet \
  --enable-hardware-output \
  --hardware-components left_arm right_arm
```

`--clamp-limits`仅建议用于已经检查过的历史数据；新录制数据若越界，优先查明
夹爪标定或控制链路原因，不应长期依赖限幅。硬件回放前确认XR遥操作进程已退出，
并确保左右臂反馈和控制服务均已启动。

硬件模式收到完整反馈和全部安全服务后不会立即运动；终端打印首帧与当前反馈的
最大差值，然后等待Space确认。确认后先以100 Hz、速度/加速度/jerk限制平滑进入
首帧，再按episode时间戳插值回放。Space暂停/继续，`q`或Ctrl+C停止并触发双臂
保持。`--end-index`为不包含的结束下标。
