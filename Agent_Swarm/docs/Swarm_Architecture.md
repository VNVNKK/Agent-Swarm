Swarm 架构（基于 Leader 的编队策略系统）
====================================

范围
----
本文描述基于"Leader 引导 + 编队策略（ring/line/column/v_shape/v/wedge）"的 Swarm 架构。
目标是：仅给 leader 下达动作/目标，其它无人机自动保持编队移动，ORCA 负责避障。

核心原则
--------
- 仅指定一个 Leader（默认 agent_id=0）作为编队参考中心。
- 编队通过“二维偏移量表”定义，OffsetBasedPolicy 将偏移转换为目标点。
- ORCA 只负责避障与速度输出，编队逻辑与避障解耦。
- 阵型变换/移动到位后自动进入 HOVER。
- Leader 本身也参与 ORCA 避障计算，其目标点来自 `/sunray/leader_goal` 或 leader 当前位姿。
- 支持 UAV（无人机）和 UGV（无人车）两种 agent 类型，通过 `agent_type` 参数区分。

模块协作总览（图）
------------------

```
┌─────────────────────────────────────────────────────────────────────┐
│                          外部输入（操控）                            │
│  formation_switch -> /sunray/formation_cmd  (ring/line/column/v/...) │
│                     /sunray/leader_goal     (leader 目标点)         │
└─────────────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Agent_Swarm 控制层                            │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              AgentSwarmNode (节点入口与调度)                    │   │
│  │  - 定时器驱动 (control_timer_ @ control_rate Hz)              │   │
│  │  - 订阅: formation_cmd, leader_goal, uav_state, orca_cmd     │   │
│  │  - 发布: orca/agent_state, orca/setup, uav_control_cmd       │   │
│  └──────────────────────────────────────────────────────────────┘   │
│        │            │             │             │                    │
│        ▼            ▼             ▼             ▼                    │
│ ┌────────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────────────┐  │
│ │ Leader     │ │Formation │ │Formation │ │ AgentStateCache      │  │
│ │ Tracker    │ │StateMach.│ │ Policy   │ │ (状态缓存+Odom发布)   │  │
│ └────────────┘ └──────────┘ │ Context  │ └──────────────────────┘  │
│        │            │       └──────────┘          │                 │
│        │            │            │                 ▼                 │
│        │            │            ▼        /uavX/orca/agent_state    │
│        │            │  ┌──────────────┐                             │
│        │            │  │FormationPoli-│                             │
│        │            │  │cyFactory     │                             │
│        │            │  └──────┬───────┘                             │
│        │            │         ▼                                     │
│        │            │  ┌──────────────┐                             │
│        │            │  │FormationPolicy│                            │
│        │            │  │(Ring/Line/   │                             │
│        │            │  │ Column/V/W) │                             │
│        │            │  └──────┬───────┘                             │
│        │            │         ▼                                     │
│        │            │  ┌──────────────┐   ┌──────────────────────┐  │
│        │            │  │GoalDispatcher│──▶│ /uavX/orca/setup     │  │
│        │            │  └──────────────┘   └──────────────────────┘  │
│        │            │                              │                │
│        │            │                              ▼                │
│        │            │                     ┌──────────────────────┐  │
│        │            │                     │ OrcaClient           │  │
│        │            │                     │ (订阅 /uavX/orca_cmd)│  │
│        │            │                     └──────────┬───────────┘  │
│        │            │                                │              │
│        │            │                                ▼              │
│        │            │                     ┌──────────────────────┐  │
│        │            │                     │ ControlCommandMapper │  │
│        │            │                     │ /uavX/uav_control_cmd│  │
│        │            │                     └──────────────────────┘  │
│        │            │                                               │
└────────┼────────────┼───────────────────────────────────────────────┘
         │            │
         ▼            ▼
┌─────────────────────────────────────────────────────────────────────┐
│                           ORCA 避障层                               │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                OrcaNode (节点入口)                             │   │
│  │  - 定时器驱动 (timer_ @ rate Hz)                              │   │
│  └──────────────────────────────────────────────────────────────┘   │
│        │                                                            │
│        ▼                                                            │
│  ┌──────────┐    ┌──────────────┐    ┌───────────────────────────┐  │
│  │ OrcaIO   │───▶│ OrcaEngine   │───▶│ RVOSimulator             │  │
│  │(ROS适配) │    │(避障计算核心) │    │ (Agent/KdTree/Obstacle)  │  │
│  └──────────┘    └──────────────┘    └───────────────────────────┘  │
│        │                                        ▲                   │
│        │              ┌─────────────────────────┘                   │
│        │              │                                             │
│        │       ┌──────────────┐                                     │
│        │       │ObstacleBuilder│                                    │
│        │       │(围栏/障碍物)  │                                    │
│        │       └──────────────┘                                     │
│        │                                                            │
│        ▼                                                            │
│  /uavX/orca_cmd (OrcaCmd: 速度+状态)                               │
│  /uavX/orca/viz_* (可视化 markers)                                  │
└─────────────────────────────────────────────────────────────────────┘
```

参数配置
--------

### Agent_Swarm 参数 (agent_swarm.yaml)

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `agent_id` | int | 0 | 本机 ID，0 为 Leader |
| `agent_num` | int | 3 | 集群总数 |
| `agent_type` | string | "uav" | 类型: "uav" 或 "ugv" |
| `leader_id` | int | 0 | Leader 的 ID |
| `uav_prefix` | string | "/uav" | UAV 话题前缀 |
| `spacing` | double | 3.0 | 编队相邻间距(m) |
| `spacing_step` | double | 0.5 | expand/contract 每次步进(m) |
| `spacing_min` | double | 1.5 | 最小间距(m) |
| `spacing_max` | double | 10.0 | 最大间距(m) |
| `control_rate` | double | 20.0 | 控制循环频率(Hz) |
| `takeoff_altitude` | double | 2.0 | 起飞高度(m) |
| `formation_altitude` | double | 2.0 | 编队飞行高度(m) |
| `arrived_threshold` | double | 0.3 | 到位判定阈值(m) |
| `goal_resend_interval` | double | 2.0 | 目标重发间隔(s) |

### ORCA 参数 (orca.yaml)

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `agent_id` | int | 0 | 本机 Agent ID |
| `agent_num` | int | 3 | 集群总数 |
| `uav_prefix` | string | "/uav" | 话题前缀 |
| `rate` | double | 20.0 | 主循环频率(Hz) |
| `neighbor_dist` | double | 5.0 | 邻居搜索距离 |
| `max_neighbors` | int | 10 | 最大邻居数 |
| `time_horizon` | double | 5.0 | ORCA 时间窗 |
| `time_horizon_obst` | double | 5.0 | 障碍物时间窗 |
| `agent_radius` | double | 0.5 | Agent 碰撞半径 |
| `max_speed` | double | 1.0 | ORCA 最大速度(m/s) |
| `goal_tolerance` | double | 0.3 | 目标到达容差 |
| `fence_enabled` | bool | false | 是否启用围栏 |
| `fence_x_min/max` | double | ±20.0 | 围栏 X 范围 |
| `fence_y_min/max` | double | ±20.0 | 围栏 Y 范围 |
| `fence_margin` | double | 1.0 | 围栏外边距 |

数据流（逐步详解）
-----------------

### 1 动作/目标输入

外部通过 `formation_switch` 工具或其他节点发送：
- `/sunray/formation_cmd` (`sunray_msgs::Formation`)：
  - `cmd` 字段: `FORMATION`(=1), `HOVER`(=2), `TAKEOFF`(=5), `LAND`(=6), `RETURN_HOME`(=7), `SET_HOME`(=8)
  - `name` 字段: `"ring"`, `"line"`, `"column"`, `"v_shape"`, `"wedge"`, `"expand"`, `"contract"`, `"custom"`
- `/sunray/formation_offsets` (`sunray_msgs::FormationOffsets`)：
  - `offsets` 数组：自定义阵型偏移量（leader 体坐标系，spacing=1 时的归一化偏移）
- `/sunray/leader_goal` (`geometry_msgs::PoseStamped`)：Leader 目标点

`formation_switch` 是一个交互式命令行工具，从标准输入读取按键：
- `1`=ring, `4`=line, `5`=column, `6`=v_shape, `7`=wedge
- `2`=expand, `3`=contract
- `101`=takeoff, `102`=land, `103`=hover, `104`=set_home, `105`=return
- `201`=输入 leader 目标点 (x y z yaw)

`formation_tui` 是基于 ncurses 的 TUI 小工具，提供“影院选座”式自定义阵型编辑：
- 20x20 网格，中心为 Leader（不可选），其余格子代表 follower 偏移
- 方向定义：上方为 +x（前方），左侧为 +y（左方），与编队偏移坐标一致
- `Space` 选中/取消座位，`c` 发布自定义阵型（`name="custom"` + `/sunray/formation_offsets`）
- `S` 保存、`L` 加载自定义阵型（文本文件）
- `m` 输入 leader 目标点 (x y z yaw)
- `1/4/5/6/7` 切阵型，`2/3` 扩散/聚拢，`t/g/h/o/b` 起飞/降落/悬停/设定home/返航

补充说明（formation_tui 细节）：
- `agent_num` 读取顺序：私有参数 `~agent_num` → 全局参数 `agent_num` → `/agent_num`，默认 1
- follower 数量 = `agent_num - 1`，自定义阵型座位数必须与 follower 数量一致，否则不允许发送
- 保存/加载文件路径：
  - 按 `S`/`L` 后输入的路径会原样使用
  - 直接回车时默认文件名为 `custom_formation.txt`
  - 默认文件保存到 **formation_tui 进程的当前工作目录**
    - 如果是 `roslaunch` 启动且未设置 `cwd`，通常为 `ROS_HOME`（常见是 `~/.ros`）
  - 需要固定位置时，请在提示中输入**绝对路径**（例如 `/home/xx/formation/custom_formation.txt`）

### 2 AgentSwarmNode 主循环

[`AgentSwarmNode`](Agent_Swarm/include/agent_swarm_node.h) 是核心调度节点，在 [`onControlTimer`](Agent_Swarm/src/agent_swarm_node.cpp) 中按 `control_rate` Hz 执行：

```
onControlTimer():
  1. state_cache_.publishAgentState()    // 发布自身 Odom 给 ORCA
  2. leader_tracker_.update()            // 更新 Leader 位姿
  3. state_machine_.update()             // 状态机驱动
  4. 根据状态机输出执行:
     - FORMATION: 计算编队目标 → GoalDispatcher 下发
     - HOVER: GoalDispatcher 下发悬停目标
     - TAKEOFF/LAND/RETURN_HOME: ControlMapper 直接发送指令
  5. mapper_.publishIfNeeded()           // ORCA → 控制指令映射
```

### 3 Leader 位姿获取

[`LeaderTracker`](Agent_Swarm/include/leader_tracker.h) 订阅 Leader 对应的 `/{prefix}{leader_id}/sunray/uav_state` 话题：
- 缓存 Leader 的当前位姿 (`leader_pose_`)
- 当收到 `/sunray/leader_goal` 时，缓存目标点 (`leader_goal_`)
- 提供 `getLeaderPose()` 和 `hasLeaderGoal()` / `consumeLeaderGoal()` 接口
- 如果本机就是 Leader（`agent_id == leader_id`），则从自身状态获取

### 4 编队状态机

[`FormationStateMachine`](Agent_Swarm/include/formation_state_machine.h) 管理编队状态转换：

```
状态定义:
  IDLE        = 0   // 空闲
  TAKEOFF     = 1   // 起飞中
  FORMATION   = 2   // 编队飞行
  HOVER       = 3   // 悬停
  LAND        = 4   // 降落
  RETURN_HOME = 5   // 返航

状态转换触发:
  - onFormationCmd(action):
      IDLE + TAKEOFF → TAKEOFF
      任意 + FORMATION → FORMATION
      任意 + HOVER → HOVER
      任意 + LAND → LAND
      任意 + RETURN_HOME → RETURN_HOME
      FORMATION + EXPAND → FORMATION (调整 spacing)
      FORMATION + CONTRACT → FORMATION (调整 spacing)

  - update() 中的自动转换:
      TAKEOFF → 当到达起飞高度后 → FORMATION
      FORMATION → 当所有 follower 到位 → HOVER
```

`onFormationCmd` 在收到 `EXPAND`/`CONTRACT` 时会调用 [`FormationPolicyContext`](Agent_Swarm/include/formation_policy.h) 的 `adjustSpacing()` 方法，步进为 `spacing_step`，并 clamp 在 `[spacing_min, spacing_max]` 范围内。

### 5 编队策略计算（OffsetBasedPolicy）

[`OffsetBasedPolicy`](Agent_Swarm/include/formation_policy.h) 提供通用 `computeTarget()`：
- 将 `generateOffsets()` 生成的二维偏移表按 `spacing` 缩放
- 根据 Leader 航向旋转偏移
- 叠加 Leader 位置得到目标点
  
`Offset2D` 是偏移向量结构（单位化或按队形尺度定义），由各阵型生成。

每种编队仅需实现 `generateOffsets(int follower_count)`：
- Ring/Line/Column/V/V-Shape/Wedge 均在 [`formation_policies.h/cpp`](Agent_Swarm/src/formation_policies.cpp) 中实现

**通用计算流程**：

```cpp
computeTarget(leader_pose, ctx, agent_id, agent_num) → 目标 PoseStamped

算法（摘要）:
  follower_count = agent_num - 1
  若 agent_id == leader_id: 目标 = leader_pose

  否则:
    offsets = generateOffsets(follower_count)
    follower_index = (agent_id > leader_id) ? agent_id - 1 : agent_id
    offset = offsets[follower_index] * spacing
    offset = rotate(offset, leader_yaw)
    target = leader_pose + offset
    target.z = formation_altitude
```

[`FormationPolicyContext`](Agent_Swarm/include/formation_policy.h) 持有当前策略指针和 `spacing` 值，[`FormationPolicyFactory`](Agent_Swarm/include/formation_policy_factory.h) 根据 `name` 创建策略实例（支持 `ring/line/column/v_shape/v/wedge`）。

### 6 目标点下发

[`GoalDispatcher`](Agent_Swarm/include/goal_dispatcher.h) 负责将编队计算出的目标点发送给 ORCA 层：

- 发布话题: `/{prefix}{id}/orca/setup` (`sunray_msgs::OrcaSetup`)
- 消息字段:
  - `goal`: 目标位置 (geometry_msgs::Point)
  - `cmd`: 运行模式，`GOAL_RUN`(=1) 表示向目标移动, `GOAL_STOP`(=0) 表示停止
- **防抖机制**: 通过 `goal_resend_interval` 控制重发频率，仅当目标点变化超过阈值或距上次发送超时时才重新发布
- **到位检测**: 比较当前位置与目标距离，小于 `arrived_threshold` 时认为到位

### 7 ORCA 避障

ORCA 层作为独立节点运行，每个 agent 一个实例：

[`OrcaNode`](ORCA/include/orca_node.h) → [`OrcaIO`](ORCA/include/orca_io.h) → [`OrcaEngine`](ORCA/include/orca_engine.h) → [`RVOSimulator`](ORCA/include/RVOSimulator.h)

**OrcaIO** 负责 ROS 通信适配：
- 订阅所有 agent 的 `/{prefix}{i}/orca/agent_state` (nav_msgs::Odometry)
- 订阅本机的 `/{prefix}{id}/orca/setup` (sunray_msgs::OrcaSetup)
- 发布 `/{prefix}{id}/orca_cmd` (sunray_msgs::OrcaCmd)
- 发布可视化 markers: `/{prefix}{id}/orca/viz_goal`, `viz_vel`, `viz_neighbors`

**OrcaEngine** 封装 RVO 计算：
- `init()`: 创建 RVOSimulator，设置参数 (neighbor_dist, max_neighbors, time_horizon, agent_radius, max_speed)
- `addAgent(id)`: 为每个 agent 创建 RVO Agent
- `updateAgentState(id, pos, vel)`: 更新 agent 当前位置和速度
- `setGoal(id, goal)`: 设置 agent 目标点
- `step(dt)`: 执行一步 ORCA 计算
  1. 为每个 agent 设置 preferred velocity（朝目标方向，大小为 max_speed）
  2. 若距目标小于 goal_tolerance，preferred velocity 设为 0
  3. `simulator->doStep()` 执行 ORCA 避障
  4. 返回每个 agent 的新速度

**OrcaCmd 消息** 包含：
- `velocity`: 避障后的速度向量 (geometry_msgs::Vector3)
- `state`: 状态码 —— `RUNNING`(=1) 正在移动, `ARRIVED`(=2) 已到达目标, `STOPPED`(=0) 停止

**ObstacleBuilder** 构建虚拟围栏：
- 当 `fence_enabled=true` 时，在 `[fence_x_min-margin, fence_x_max+margin] × [fence_y_min-margin, fence_y_max+margin]` 范围构建矩形障碍物边界
- 障碍物以逆时针顶点序列添加到 RVOSimulator

### 8 OrcaClient 接收避障输出

[`OrcaClient`](Agent_Swarm/include/orca_client.h) 订阅 `/{prefix}{id}/orca_cmd`：
- 缓存最新的 OrcaCmd
- 提供 `latestCmd()` / `hasNew()` 接口供 AgentSwarmNode 主循环查询
- 检测 `state == ARRIVED` 触发到位回调

### 9 控制指令映射

[`ControlCommandMapper`](Agent_Swarm/include/control_command_mapper.h) 将 ORCA 输出转为最终控制指令：
- 发布话题: `/{prefix}{id}/sunray/uav_control_cmd` (`sunray_msgs::UAVControlCMD`)
- **速度模式**: 将 ORCA 的 2D 速度映射为 UAVControlCMD 的速度字段
- **高度控制**: Z 轴使用 `formation_altitude` 维持定高
- **特殊指令**:
  - `CMD_TAKEOFF`: 起飞到 `takeoff_altitude`
  - `CMD_LAND`: 降落
  - `CMD_RETURN_HOME`: 返航
  - `CMD_HOVER`: 悬停（速度置零）
- `publishIfNeeded()`: 由主循环调用，根据当前状态决定是否发布指令

### 10 AgentStateCache

[`AgentStateCache`](Agent_Swarm/include/agent_state_cache.h) 维护自身状态并向 ORCA 报告：
- 订阅 `/{prefix}{id}/sunray/uav_state` 缓存自身的位姿和速度
- 定时发布 `/{prefix}{id}/orca/agent_state` (nav_msgs::Odometry)
- 提供 `getPose()` / `getVelocity()` 接口

关键话题与作用（表）
-------------------

| 话题 | 方向 | 消息类型 | 发布者 | 订阅者 | 用途 |
|---|---|---|---|---|---|
| `/sunray/formation_cmd` | ext→Agent | `sunray_msgs/Formation` | formation_switch | AgentSwarmNode | 编队指令(RING/LINE/COLUMN/V_SHAPE/V/WEDGE/EXPAND/CONTRACT/TAKEOFF/LAND/RETURN) |
| `/sunray/leader_goal` | ext→Agent | `geometry_msgs/PoseStamped` | formation_switch | LeaderTracker | Leader 目标点 |
| `/{prefix}{id}/sunray/uav_state` | ext→Agent | `sunray_msgs/UAVState` | 飞控/仿真 | AgentStateCache, LeaderTracker | 无人机状态 |
| `/{prefix}{id}/orca/agent_state` | Agent→ORCA | `nav_msgs/Odometry` | AgentStateCache | OrcaIO | 本机状态→ORCA |
| `/{prefix}{id}/orca/setup` | Agent→ORCA | `sunray_msgs/OrcaSetup` | GoalDispatcher | OrcaIO | 目标点与运行模式 |
| `/{prefix}{id}/orca_cmd` | ORCA→Agent | `sunray_msgs/OrcaCmd` | OrcaIO | OrcaClient | 避障后速度与状态 |
| `/{prefix}{id}/sunray/uav_control_cmd` | Agent→飞控 | `sunray_msgs/UAVControlCMD` | ControlCommandMapper | 飞控/仿真 | 最终控制指令 |
| `/{prefix}{id}/orca/viz_goal` | ORCA→RViz | `visualization_msgs/Marker` | OrcaIO | RViz | 目标点可视化 |
| `/{prefix}{id}/orca/viz_vel` | ORCA→RViz | `visualization_msgs/Marker` | OrcaIO | RViz | 速度向量可视化 |
| `/{prefix}{id}/orca/viz_neighbors` | ORCA→RViz | `visualization_msgs/MarkerArray` | OrcaIO | RViz | 邻居可视化 |

动作指令（形成编队的方式）
------------------------
- `cmd=FORMATION`：进入编队模式，`name` 指定编队或动作
  - `name=ring/line/column/v_shape/wedge`：切换阵型
  - `name=expand/contract`：调整 `spacing`（步进 `spacing_step`，限幅 `[spacing_min, spacing_max]`）
- `cmd=HOVER`：悬停
- `cmd=TAKEOFF`：全体起飞
- `cmd=LAND`：全体降落
- `cmd=RETURN_HOME`：全体返航

状态机图示
------------------

```
                    ┌──────────┐
                    │   IDLE   │
                    └────┬─────┘
                         │ TAKEOFF cmd
                         ▼
                    ┌──────────┐
              ┌─────│ TAKEOFF  │
              │     └────┬─────┘
              │          │ 到达起飞高度
              │          ▼
              │     ┌──────────┐ ◄──── FORMATION cmd (从任意状态)
              │  ┌──│FORMATION │──┐
              │  │  └────┬─────┘  │
              │  │       │        │ EXPAND/CONTRACT
              │  │       │        │ (调整 spacing, 保持 FORMATION)
              │  │       │ ARRIVED│
              │  │       ▼        │
              │  │  ┌──────────┐  │
              │  └──│  HOVER   │──┘
              │     └────┬─────┘
              │          │
              ▼          ▼
         ┌──────────┐  ┌──────────────┐
         │   LAND   │  │ RETURN_HOME  │
         └──────────┘  └──────────────┘

  注: LAND 和 RETURN_HOME 可从任意状态进入
```

RVO/ORCA 内核说明
-----------------

ORCA 层使用经典的 RVO2 (Reciprocal Velocity Obstacles) 算法实现多 agent 避障：

### 核心类

- [`RVOSimulator`](ORCA/include/RVOSimulator.h)：仿真器主类，管理所有 Agent、Obstacle 和 KdTree
  - `addAgent(pos, ...)`: 添加 agent 及其参数
  - `addObstacle(vertices)`: 添加多边形障碍物
  - `processObstacles()`: 构建障碍物 KdTree
  - `setAgentPrefVelocity(id, vel)`: 设置期望速度
  - `doStep()`: 执行一步避障计算
  - `getAgentVelocity(id)`: 获取避障后速度

- [`Agent`](ORCA/include/Agent.h)：单个 agent 的 ORCA 计算
  - `computeNeighbors()`: 通过 KdTree 搜索邻居
  - `computeNewVelocity()`: 计算 ORCA 约束平面，求解线性规划得到最优速度

- [`KdTree`](ORCA/include/KdTree.h)：空间索引，加速邻居搜索
  - `buildAgentTree()`: 构建 agent KdTree
  - `buildObstacleTree()`: 构建障碍物 KdTree
  - `computeAgentNeighbors()`: 查询 agent 邻居
  - `computeObstacleNeighbors()`: 查询障碍物邻居

- [`Obstacle`](ORCA/include/Obstacle.h)：障碍物线段
  - 以链表形式存储多边形边

- [`Vector2`](ORCA/include/Vector2.h)：2D 向量运算工具类

### ORCA 算法流程

```
每个时间步 (OrcaEngine::step):
  1. 更新所有 agent 位置和速度 → RVOSimulator
  2. 设置每个 agent 的 preferred velocity (朝目标方向)
  3. simulator->doStep():
     a. 每个 Agent::computeNeighbors()  // KdTree 搜索
     b. 每个 Agent::computeNewVelocity() // ORCA 线性规划
     c. 更新 agent 位置/速度
  4. 读取新速度 → 发布 orca_cmd
```

Launch 配置说明
--------------

### swarm_sim.launch

集群仿真总 launch，为每个 agent 启动 agent_swarm 和 orca 节点：

```xml
<launch>
  <arg name="agent_num" default="3"/>

  <!-- 为每个 agent 启动 -->
  <include file="agent_swarm.launch">
    <arg name="agent_id" value="0"/>  <!-- Leader -->
    ...
  </include>
  <include file="orca.launch">
    <arg name="agent_id" value="0"/>
    ...
  </include>

  <!-- agent 1, 2, ... -->
  ...
</launch>
```

### agent_swarm.launch

单个 agent 的控制节点：
- 加载 `agent_swarm.yaml` 参数
- 启动 `agent_swarm_node`

### orca.launch

单个 agent 的 ORCA 节点：
- 加载 `orca.yaml` 参数
- 启动 `orca_node`

### formation_switch.launch

启动 `formation_switch` 交互控制工具。

编译说明
--------

基于 catkin/CMake 构建（ROS 1）：

```bash
cd ~/catkin_ws
catkin_make --pkg swarm
# 或
catkin build swarm
```

依赖：
- `roscpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`
- `sunray_msgs`（自定义消息包：Formation, OrcaSetup, OrcaCmd, UAVState, UAVControlCMD）

代码文件职责（与文件头中文说明同步）
-------------------------------

### Agent_Swarm 层

| 文件 | 职责 |
|---|---|
| [`agent_swarm_node.h/cpp`](Agent_Swarm/src/agent_swarm_node.cpp) | 节点入口与调度，订阅/发布、定时器与状态机驱动 |
| [`formation_state_machine.h/cpp`](Agent_Swarm/src/formation_state_machine.cpp) | 编队状态机（状态转换、指令分发、安全兜底） |
| [`leader_tracker.h/cpp`](Agent_Swarm/src/leader_tracker.cpp) | Leader 位姿获取与缓存，leader_goal 管理 |
| [`formation_policy.h`](Agent_Swarm/include/formation_policy.h) | 编队策略接口与通用偏移策略基类（`Offset2D`, `OffsetBasedPolicy`, `FormationPolicyContext`） |
| [`formation_policies.h/cpp`](Agent_Swarm/src/formation_policies.cpp) | Ring/Line/Column/V/V-Shape/Wedge 的偏移表生成与通用目标计算 |
| [`formation_policy_factory.h/cpp`](Agent_Swarm/src/formation_policy_factory.cpp) | 策略工厂，注册 `ring/line/column/v_shape/v/wedge` |
| [`agent_state_cache.h/cpp`](Agent_Swarm/src/agent_state_cache.cpp) | 自身状态缓存与 `/orca/agent_state` Odometry 发布 |
| [`goal_dispatcher.h/cpp`](Agent_Swarm/src/goal_dispatcher.cpp) | 编队目标 → OrcaSetup 封装与发布，含防抖和到位检测 |
| [`orca_client.h/cpp`](Agent_Swarm/src/orca_client.cpp) | 订阅 `/orca_cmd`，缓存 ORCA 避障输出 |
| [`control_command_mapper.h/cpp`](Agent_Swarm/src/control_command_mapper.cpp) | ORCA 速度输出 → UAVControlCMD 映射（含特殊指令） |
| [`formation_switch.cpp`](Agent_Swarm/utils/formation_switch.cpp) | 键盘交互控制工具（ring/line/column/v_shape/wedge/expand/contract/起降/leader 目标点） |

### ORCA 层

| 文件 | 职责 |
|---|---|
| [`orca_node.h/cpp`](ORCA/src/orca_node.cpp) | ORCA 节点入口，定时器驱动 |
| [`orca_engine.h/cpp`](ORCA/src/orca_engine.cpp) | ORCA/RVO 避障计算核心封装 |
| [`orca_io.h/cpp`](ORCA/src/orca_io.cpp) | ROS 话题适配（订阅状态/目标，发布速度/可视化） |
| [`obstacle_builder.h/cpp`](ORCA/src/obstacle_builder.cpp) | 围栏/矩形障碍物构建 |
| [`RVOSimulator.h/cpp`](ORCA/src/RVOSimulator.cpp) | RVO2 仿真器主类 |
| [`Agent.h/cpp`](ORCA/src/Agent.cpp) | RVO2 单 Agent ORCA 计算（邻居搜索+线性规划） |
| [`KdTree.h/cpp`](ORCA/src/KdTree.cpp) | KdTree 空间索引 |
| [`Obstacle.h/cpp`](ORCA/src/Obstacle.cpp) | 障碍物线段数据结构 |
| [`Vector2.h`](ORCA/include/Vector2.h) | 2D 向量工具 |
| [`Definitions.h`](ORCA/include/Definitions.h) | 公共类型定义与常量 |
| [`RVO.h`](ORCA/include/RVO.h) | RVO 库统一头文件 |

扩展指南
--------

### 添加新编队阵型

1. 在 [`formation_policies.h/cpp`](Agent_Swarm/src/formation_policies.cpp) 中新增策略类，继承 `OffsetBasedPolicy`
2. 实现 `generateOffsets(follower_count)`，返回二维偏移表
3. 在 [`FormationPolicyFactory`](Agent_Swarm/src/formation_policy_factory.cpp) 中注册新类型
4. 在 `sunray_msgs/Formation` 消息中新增 `name` 约定（必要时扩展常量）
5. 在 `formation_switch` 中添加对应的按键绑定

### 添加新的外部控制指令

1. 在 [`FormationStateMachine`](Agent_Swarm/src/formation_state_machine.cpp) 中添加新状态/转换
2. 在 [`AgentSwarmNode::onControlTimer`](Agent_Swarm/src/agent_swarm_node.cpp) 中处理新状态
3. 在 [`ControlCommandMapper`](Agent_Swarm/src/control_command_mapper.cpp) 中添加新的指令映射
