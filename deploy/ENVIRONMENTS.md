# 运行环境

本轮复用已验证的Python环境，不跨机器复制Conda目录或ROS build/install产物。

- 服务器：ROS2 Humble Python3.10；`AMMR_TELEOP_PYTHON`指向现有tv解释器。
  需要numpy、scipy、pinocchio（含CasADi绑定）、casadi、opencv、sshkeyboard、TeleVuer相关依赖。
  控制中心用系统Python，ROS消息来自Humble和lifting_msg_pkg overlay。
- AGX：系统Python3.10与ROS2 Humble；需要numpy、opencv、pandas、pyarrow；
  `AMMR_RECORDER_PYTHON`/`AMMR_REPLAY_PYTHON`可显式指定解释器。没有Conda时直接使用系统Python。
  驱动沿用已验证的机械臂、相机、底盘和升降柱工作空间。
- 仿真：Isaac Python与XR Python分开。`AMMR_ISAAC_PYTHON`指向现有unitree_sim_env；
  图像、录制依赖参照同级`../ammr_sim/requirements_teleop_record.txt`。

第一次部署到新架构设备时必须按该设备安装Pinocchio/ROS/相机SDK，不能用服务器二进制包替代AGX ARM64包。
`pyproject.toml`定义项目命名和Python包发现；硬件/Isaac环境不通过一次pip安装混装。
