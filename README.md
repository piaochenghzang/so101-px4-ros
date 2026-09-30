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
