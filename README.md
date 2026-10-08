# so101-px4-ros

SO101 + PX4 + Gazebo Classic 的 ROS1 仿真配套源码。
当前验证环境：Ubuntu 20.04 / ROS Noetic / Gazebo 11；不使用 Ubuntu22。

## 内容与依赖

- `so101_gazebo`：关节位置控制、状态发布及命令失联保持策略。
- `so101_gazebo_bridge`：可选的六关节 ROS 数组命令桥接，**不与 PX4 命令源同时启动**。
- `super_px4_bridge`：点云/里程计适配、SUPER 指令转发、起飞及终点悬停管理。
  飞行配置保存在 `config/px4_sim.yaml`，launch 不依赖 SUPER 中的本地未提交配置。
- `tests/flight/run_empty_arm_hover.py`：隔离的空载折叠臂飞行、目标切换、断流验证。

SUPER 不复制进本仓库，版本固定在 `dependencies.repos`。
对应仿真固件/模型为 [PX4-RoboMaster](https://github.com/piaochenghzang/PX4-RoboMaster)：
`bc1fdaaad46ec3578b6ae6fe4a92078550f7dfb1`，Gazebo 子模块
`045351aa5a6db73aca23346bf6aa51819c50aae1`。
此 ROS 仓库本身不包含 PX4 二进制、飞机/机械臂网格、构建目录或实验日志。
SUPER 原有本地 CMake 修改仅开启辅助调参程序，不是本次飞行运行依赖。

## 在新工作区构建

现有工作区已包含这些包时，不要重复克隆同名包；当前 `/home/pcz/super_ws`
仍使用原包目录，新仓库 `/home/pcz/so101-px4-ros` 是已测试源码的发布副本，
没有移动原目录或改动原 SO101 Git 索引。

在一个没有同名包的新 catkin 工作区中，可将本仓库克隆到 `src/so101-px4-ros`，
SUPER 作为 `src/SUPER` 的独立依赖：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/super_ws/src
git clone git@github.com:piaochenghzang/so101-px4-ros.git ~/super_ws/src/so101-px4-ros
git clone https://github.com/hku-mars/SUPER.git ~/super_ws/src/SUPER
git -C ~/super_ws/src/SUPER checkout 2ad3419c127a617c6d7df6925e81a14175a9c096
cd ~/super_ws
catkin_make -j4
source devel/setup.bash
```

需要已安装 ROS/MAVROS、Gazebo 开发库及 SUPER 原项目依赖。
先按上述 PX4 仓库准备 SITL 和模型；本仓库不执行系统安装或修改全局配置。

## 本次飞行接管修复

1. 保留请求三维高度（`click_height=-10`），启用目标朝向约束。
   ROG-Map 仍可能把目标移到体素中心；**请求目标不等于解析终点**。
2. 起飞后跟踪接管采用 1 s 平滑混合；轨迹断流时必须确认最后有效命令
   已到达已提交多项式轨迹的结束时间、终端速度足够小且终点偏移有界。
3. 正常结束进入 `SETTLE_HOVER`，保持实际终点，经过连续过渡和实测到位/静止驻留
   后进入 `FINAL_HOVER`。不直接跳回原始请求坐标。
4. 发布 `/planning/resolved_goal`（`PoseStamped`，world，latched）告知接受的终点。
   后续任务须关联目标时间戳，不能把上一目标的 latched 消息当作新目标成功。
   该消息姿态是指令朝向；当前到位判据是位置/平移速度，**不代表实测偏航精确到位**。
5. 最终 MAVROS 偏航指令统一限制为 0.35 rad/s、内部限速器加速度 0.50 rad/s²；
   跨 ±π 走最短角差，仿真暂停后不一次性追赶。加速度/yaw-rate 前馈仍被屏蔽。
6. 不符合正常终止条件时进入 `FAILSAFE_HOVER`，在当前实测位置悬停；该状态
   不自动继续缓存任务，不把断流当作到点成功。失效、过期和旧目标指令不会续命。

关键参数见 `launch/super_navigation.launch`：`max_goal_projection=0.35 m`、
`terminal_capture_radius=0.50 m`、`terminal_speed_tolerance=0.05 m/s`，
稳定判据 `goal_tolerance=0.20 m`、`velocity_tolerance=0.25 m/s`。
这些是当前粗导航阈值，不是抓取末端厘米级精度阈值。
本轮没有调 PID、质量/惯性、推力模型或碰撞参数。

## 回归与飞行验证

```bash
cmake --build ~/super_ws/build --target test_super_command_transition so101_command_watchdog_test -- -j4
cd ~/super_ws/build
ctest -R 'super_command_transition|so101_command_watchdog' --output-on-failure
```

先关闭已有仿真；测试拒绝已有 ROS/PX4/Gazebo 会话，只清理自己启动的进程。
默认输出目录为 `$PX4_DIR/build/empty_arm_hover/<唯一案例名>`，保存遥测、逐层指令、
状态事件、配置快照、ULog 和 JSON 结果。
源码位置由 ROS 包查找决定，工作区可通过 `PX4_DIR` 与 `ROS_WS` 指定。

```bash
source /opt/ros/noetic/setup.bash
source ~/super_ws/devel/setup.bash
export PX4_DIR=~/PX4-RoboMaster
export ROS_WS=~/super_ws
python3 ~/so101-px4-ros/tests/flight/run_empty_arm_hover.py review_sequence_01 \
  --super-goal 0.3 0 1.8 --next-super-goal -0.3 0 1.2 \
  --goal-body-frame --goal-yaw-offset 0.8
```

上例测试文件路径适用于本次发布副本；克隆到 catkin `src` 后改为仓库实际路径。
`--super-goal` 的 XY 是相对当前位置偏移，Z 是 MAVROS 本地绝对高度；
`--goal-body-frame` 仅将偏移按发送时偏航旋转。第一次终点悬停 10 s 后发送第二目标，
最终悬停 20 s。故障测试可改为单目标加 `--drop-super-after 0.1`，
该选项主动关闭本轮桥接节点，只适合隔离测试。

2026-09-30：两个 C++ 回归测试通过；正式配置不同高度及连续目标复测通过，
正常终点接管的 XYZ 指令跳变为 0，偏航指令速度不超过约 0.35 rad/s。
带完整轨迹结束校验的连续目标最终悬停，后 15 s 平均/最大高度误差
约 1.35/2.57 cm（相对解析终点）。断流测试进入故障悬停而非正常到点，
5 s 后实测速度约 0.058 m/s，未出现 NaN。
此前失稳的 0.7 m ENU 横向目标复测通过：后 15 s 平均/最大高度误差
约 1.08/2.12 cm，正常接管 XYZ 跳变为 0。

## 使用边界

该阶段验证为空载折叠臂、无障碍短程飞行。没有验收携物飞行、避障空间中混合轨迹的
碰撞安全性、长时间统计重复性或飞行抓取。地图分辨率仍为 0.3 m，最终抓取必须另做
精确接近/视觉或几何对齐；不能直接把解析体素中心当作抓取位。
SUPER 的全局偏航轨迹仍可能异常，管理器限制的是实际发往飞控的指令，
不是宣布规划器优化问题全部修复。
带偏航目标的升高阶段仍出现短时单电机达到 1.0；对应一个跟踪窗口内约
91% 的推力请求能完全满足。**推力余量问题没有因此消失**，携物前需单独验证动力裕度。
现有 `start_super_sim.sh` 会按进程名清理 Gazebo，适用于用户独占会话；
隔离实验使用本仓库测试脚本，不调用该全局清理入口。

## 只读坐标观察（2026-10-01）

`super_px4_bridge` 新增同一物理步采样的诊断 WorldPlugin、坐标观察器和被动验证入口。
仅发布 `/grasp_audit/truth` 诊断；观察器只有订阅和 JSONL 输出，所有记录的
`close_permitted=false`。不发布飞行/关节/闭爪命令，不改 PID、生产 SDF 或 arm_control。

```bash
source /opt/ros/noetic/setup.bash
source /home/pcz/super_ws/devel/setup.bash
cd /home/pcz/super_ws
catkin_make -j4 so101_coordinate_truth
cd build
ctest --output-on-failure -R 'grasp_frame|super_command_transition|so101_command_watchdog'
rosrun super_px4_bridge run_readonly_frame_audit.py --name frame_check_01 --case tilted --duration 4
```

请替换为实际 catkin 工作区；`PX4_DIR` / `ROS_WS` 可覆盖依赖路径。
测试发现现有仿真则拒绝启动；结果写入 PX4 `build/frame_audit/<name>`，名称不能重复。
生成的副本移除控制插件、传感器和碰撞，车辆 dynamic、世界零重力，方块 static；
不启动 PX4/MAVROS，不调用模型 set/spawn/reset，也不改原文件。
不使用整机 static 的 Joint::Position 缓存作为编码器，采样器明确拒绝这类角度。

零偏航、偏航、小倾角的被动链核对通过，实际 FK 位置差约 4.21 μm；
物体缺失和错安装配置负例正确拒绝。数学/回调测试覆盖非零原点、错 frame、错时刻、
旧目标和重置。该被动场景不验证重力、接触、飞行稳定或抓取；真实 MAVROS 配准尚未验收。

正常仿真使用时，须在 world 内显式加载 `libso101_coordinate_truth.so`
（plugin name=coordinate_truth，vehicleName=yhang550，targetName=test_cube），
并有 gazebo_ros API、use_sim_time=true、devel/lib 插件路径；本轮未自动修改生产启动流程。
随后可只运行：

```bash
rosrun super_px4_bridge observe_grasp_frames.py --output /tmp/grasp_frames_01.jsonl --duration 20
```

无 MAVROS 时 local_valid=false；配准冻结后继续记录残差，不逐帧重拟合掩盖误差。
`local_valid` 仅表示转换可用，毫米精度另由 `local_precision_consistent` 表示。
旧 joint_states 对照标为近似 ROS clock；主 FK 用物理步实测角。
参考点是当前 20 mm 方块/碰撞代理的模板，不是通用 TCP；几何对齐不等于准许闭爪。

2026-10-07 时间配对修正：保持原 20 ms 双侧支持、100 ms 真值新鲜度和
2 mm / 0.001 rad 精度门限不变；仅在等待未来 odom 端点时有界等待至 80 ms。
单次缺样/时差超限只拒绝该样本，不清除被冻结的原点；重复 odom 时间戳拒绝两条，
若污染校准支持则锁定失效，不能自动重拟合来消掉误差。时钟/实体变化、FCU 断联
或 frame 错配清除配准。没有 EKF reset-counter 输入，输出对此明确标为 unavailable；
因此配准只是诊断变换，不是可直接用于导航或闭爪的 TF/许可。

正常场景可加 `--require-local`：缺少当前世代至少十个有效局部配对时返回非零。
它不等于精度通过；JSON summary 分开给出 observation_complete、
local_registration_current、local_precision_verdict，且始终 grasp_tested=false。
有无方块都核对机体/TCP；有方块时另核对物体相对位置，不制造缺失的目标。
正常未解锁场景连续 40 s 复测得到 1941 条真值、467 条有效局部配对，
原点仅冻结一次；位置残差最大约 0.330 m，精度明确 FAIL，不能用于抓取对齐。
该结果只证明时间配对/漂移可观测性，不证明飞行、物体对齐或抓取已验收。

## 已知目标飞行接管（阶段 2，2026-10-08）

飞行管理器新增 known_goal_mode，默认 false。只有显式开启且 use_sim_time=true 时，
才绕过雷达等待并接收 /grasp_demo/known_setpoint（KnownFlightSetpoint）。
正常 SUPER 导航默认路径不改；专用模式独占导航输入，旧包/换任务不能续期或接管，
输入断流进入故障悬停且不自动恢复，模式/解锁丢失后不自动夺回或重新解锁。
WorldPlugin 新增同物理步机体线/角速度真值，仅为诊断/仿真输入，不发控制。

PX4 仓库的 src/modules/arm_control/tests/hover/ 包含已知目标配置、纯坐标预览、
短程飞行测试及隔离接口负例。飞行必须显式传 --execute-flight。
首版 world 轨迹用 Gazebo 机体真值与 MAVROS 局部估计进行实时控制坐标转换，
并以速度前馈补偿估计速度偏差；不向 EKF 注入真值，但也不是已经验证真实定位。
两次无雷达/无锚点、折叠臂到位测试均保持 10 s：世界位置平均误差约 2.2–2.3 cm，
最大约 3.5 cm，仅通过本阶段 5 cm 粗到位包络。没有机械臂运动或闭爪请求；
更严格的悬停抓取许可、悬停版 profile、抓取和携物仍未验收。
完整修改说明集中保存在 PX4 的 SO101_REVIEW.md 阶段 21–22，不再分散新增说明文件。
