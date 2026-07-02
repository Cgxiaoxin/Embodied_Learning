# 机械臂 SDK 与 VLA 项目 — 面试梳理

> 面向面试的四块核心经历：**7DOF 机械臂 SDK（运动学/动力学）**、**MoveIt 运动规划**、**Pinocchio/KDL 逆解框架**、**π0 折纸 VLA 微调**。每节含「项目背景 → 实现思路 → 面试话术 → 可能的追问」。

---

## 目录

1. [7DOF 蓝思机械臂 SDK](#一7dof-蓝思机械臂-sdk)
2. [MoveIt 运动规划（轨迹 + 碰撞）](#二moveit-运动规划轨迹--碰撞)
3. [Pinocchio / KDL 逆解规划框架](#三pinocchio--kdl-逆解规划框架)
4. [π0 折纸 VLA 微调项目](#四π0-折纸-vla-微调项目)
5. [四块内容的串联话术](#五四块内容的串联话术)

---

## 一、7DOF 蓝思机械臂 SDK

### 1.1 项目背景（30 秒版）

> 我负责从零搭建一套 **7 自由度机械臂的控制 SDK**。硬件侧厂商（蓝思）提供的是**关节模组电机驱动 + PID 参数文档**，通信走 **UDP**。我的工作是：先通过 UDP **扫描/发现各关节模组 ID**，建立通信映射；再在其上封装**控制层 SDK**（状态读取、位置/速度/力矩指令、安全限位）；并实现**正逆运动学**和基础**动力学接口**（重力补偿前馈），供上层轨迹规划和应用调用。

### 1.2 整体架构

```
┌─────────────────────────────────────────────────────────┐
│  应用层：轨迹跟踪、MoveIt / Pinocchio IK、任务逻辑        │
└──────────────────────────┬──────────────────────────────┘
                           │ 关节角目标 q_d / 力矩 τ_ff
┌──────────────────────────▼──────────────────────────────┐
│  运动学层：FK / IK（臂角法）/ Jacobian                   │
│  动力学层：M(q), G(q) → 重力补偿前馈                      │
└──────────────────────────┬──────────────────────────────┘
                           │ 各关节位置指令 + 前馈力矩
┌──────────────────────────▼──────────────────────────────┐
│  控制层 SDK：插值、限位、状态机、异常处理                   │
│  底层 PID 参数按厂商文档配置                              │
└──────────────────────────┬──────────────────────────────┘
                           │ UDP 报文（读写寄存器/状态）
┌──────────────────────────▼──────────────────────────────┐
│  关节模组 1 … 7（各带独立 ID）                            │
└─────────────────────────────────────────────────────────┘
```

**面试强调的分工：**

| 层级 | 我做了什么 | 厂商文档提供了什么 |
|------|-----------|-------------------|
| 通信 | UDP 广播/扫描 ID、报文编解码、超时重传 | 寄存器地址、通信协议 |
| 控制 | 封装 `setJointPosition` / `getJointState`、轨迹插值 | 各轴 PID 增益建议值 |
| 运动学 | DH 建模、FK、臂角法 IK | — |
| 动力学 | 基于 URDF/Pinocchio 算 \(G(q)\) 做重力补偿 | — |

---

### 1.3 通信层：UDP 扫描关节模组 ID

**实现思路：**

1. **广播发现**：向局域网广播 UDP 查询包（含协议头 + 「scan」指令），各关节模组按文档规定回复自己的 **Module ID / 轴号**。
2. **建立映射表**：`map<ID, joint_index>`，例如 ID=0x01 → joint1，保证后续读写不会搞错轴序。
3. **周期性心跳**：读各轴 `position / velocity / current / fault_code`，断连则标记 offline。
4. **指令下发**：位置模式写目标角 + 使能位；底层模组内部闭环 PID（参数按文档填）。

**面试话术：**

> 底层是分布式关节模组，每轴有独立 ID。上电后先做 UDP 扫描，把「物理 ID → 逻辑 joint1~7」映射建好，后面 SDK 对外只暴露统一的关节接口，上层不用关心具体 ID。

---

### 1.4 运动学：正解 + 臂角法 IK

#### 1.4.1 为什么 7DOF 需要「臂角法」

- 末端 6D 位姿任务只有 **6 个约束**，7 个关节 → **1 维冗余**。
- 同样末端位姿，肘部可以「抬高」或「压低」，对应无穷多组解。
- **臂角（arm angle / ψ）**：用冗余自由度的一个参数描述「肘部绕肩-腕轴线转了多少」，固定它后 IK 从欠定变成**可解析或低维数值**求解。

#### 1.4.2 我的 IK 思路：固定第 4 轴

> **核心做法：把第 4 关节角 \(q_4\) 当作冗余参数，由上层给定或沿用上一时刻值，然后对其余 6 个关节做解析/半解析求解。**

**具体流程：**

```
输入：目标末端位姿 T_target（4×4 或 pos+quat）
      冗余参数：q4 = q4_ref（上一解 / 用户指定 / 避障偏好）

Step 1  正解验证当前 FK 链，确认 DH 参数与 URDF 一致

Step 2  固定 q4 后，把 7DOF 问题降维：
        - 利用几何关系，先求 shoulder / wrist 中心位置
        - 在已知 q4 的平面内，用余弦定理解 q2, q3（类似 6DOF 臂的 elbow 三角）

Step 3  腕部三轴（q5, q6, q7）求旋转分解：
        R_0^7 = R_target
        R_3^7 = (R_0^3(q1,q2,q3,q4))^{-1} · R_0^7
        → 对腕部 ZYZ / 欧拉角解析求 q5, q6, q7

Step 4  校验：关节限位、解的连续性（与上一帧差最小）

Step 5  失败时：微调 q4（±Δ）重试，或 fallback 到数值 IK（Jacobian 迭代）
```

**为什么选「固定 4 轴」：**

- 对该 7DOF 构型，第 4 轴通常是 **肘部/大臂-小臂** 转角，对臂形（肘上/肘下）影响最大。
- 固定 \(q_4\) 后，肩-腕几何约束可拆成「位置 3 轴 + 姿态 3 轴」，和经典 6DOF 分解类似，**计算快、可复现**，适合实时控制。
- 冗余自由度不浪费：上层可按任务改 \(q_4\)——避障时抬高肘部、紧凑时压低。

**面试话术（1 分钟）：**

> 7 轴臂对 6D 位姿是冗余的，我用臂角法处理：把第 4 关节当作冗余参数，给定 \(q_4\) 后问题变成确定的 6 轴几何求解。先解肩-肘-腕的位置三角形得到 \(q_1,q_2,q_3\)，再用腕部旋转分解求 \(q_5,q_6,q_7\)。最后做限位检查和与上一帧的最近解选择。若不可达就小步调整 \(q_4\) 或退到 Jacobian 数值迭代。

#### 1.4.3 正运动学（FK）

- 建立 **Modified DH** 参数表（与 URDF 一致）。
- 逐级连乘：\(T_0^i = \prod_{j=1}^{i} T_j(q_j)\)。
- 同时算 **几何 Jacobian** \(J(q)\)，供笛卡尔速度映射 \(\dot{x} = J\dot{q}\) 和数值 IK 备用。

---

### 1.5 动力学：我做了什么 & 怎么诚实回答

**标准动力学方程：**

\[
\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) + F(\dot{q})
\]

| 项 | 含义 | SDK 中的实际用途 |
|----|------|-----------------|
| \(G(q)\) | 重力项 | **重力补偿前馈**（最常用、收益最大） |
| \(M, C\) | 惯性/科氏 | 高速轨迹时可做
| \(F\) | 摩擦 | 低速段补偿，需辨识 |

**我当时的实现边界（建议如实说）：**

> 控制主环还是**模组自带的位置 PID**（参数按厂商文档）。我在 SDK 里用 URDF + Pinocchio（或递归牛顿-欧拉）算了 **\(G(q)\)**，在位置指令上叠加 **重力补偿力矩前馈** \(\tau_{ff} = G(q)\)，减轻竖直方向漂移和稳态误差。完整的 \(M,C\) 辨识和计算力矩控制（CTC）没有全做完，但我理解其公式和辨识流程，知道怎么从 regressor 最小二乘标定连杆参数。

**若被追问「动力学辨识做过吗」：**

> 没有完整做全参数辨识。了解标准流程：设计激励轨迹 → 采集 \(q,\dot{q},\ddot{q},\tau\) → 堆叠 \( \tau = Y(q,\dot{q},\ddot{q}) P \) → 最小二乘求 \(P\)。真机难点在电流估力矩误差和摩擦非线性，需要滤波和正则化。

**CTC 公式（展示理论深度）：**

\[
\tau = M(q)\ddot{q}_d + C(q,\dot{q})\dot{q}_d + G(q) + K_p e + K_d \dot{e}
\]

动力学做前馈，PID 做反馈；参数不准时靠反馈兜底。

---

### 1.6 本节可能的追问

| 追问 | 回答要点 |
|------|----------|
| 7DOF IK 多解怎么选？ | 限位内、与上一帧差最小、避障时调整 \(q_4\) |
| 奇异点怎么处理？ | 监测 \(det(JJ^T)\)，降速或改冗余角绕开 |
| 半闭环 vs 全闭环？ | 模组多为电机端编码器（半闭环），齿隙在减速器内不可测 |
| SDK 和控制器的边界？ | SDK 给关节角目标；PID 在驱动器内环 |

---

## 二、MoveIt 运动规划（轨迹 + 碰撞）

### 2.1 项目背景（30 秒版）

> 上层任务需要**从当前构型到目标构型的无碰撞轨迹**。我基于 **MoveIt2** 开源框架集成，不是从零写 RRT。MoveIt 负责 **运动学 + 采样规划 + 碰撞检测 + 轨迹时间参数化 + 执行接口**，我主要做 URDF/SRDF 配置、Planning Scene 障碍物管理、以及 Python/C++ 调用和与自研 SDK 的执行对接。

### 2.2 MoveIt2 是什么（纠正「只是接口」的印象）

```
你的代码 (Python/C++)
    ↓
MoveIt2 框架
    ├── 运动学      KDL / Trac-IK / 自定义 IK 插件
    ├── 路径规划    OMPL (RRTConnect/RRT*) / CHOMP / STOMP / Pilz
    ├── 碰撞检测    FCL / Bullet  ← Planning Scene
    └── 轨迹执行    FollowJointTrajectory Action → 你的 SDK / ros2_control
```

**一句话：** MoveIt2 = 运动学 + 路径规划 + 碰撞检测 + 执行，是完整规划框架。

---

### 2.3 完整流程（从配置到动起来）

#### 第一层：URDF / SRDF（前置，做错后面全错）

| 文件 | 内容 |
|------|------|
| `robot.urdf` | 连杆、关节、限位、**碰撞几何**（mesh/box/cylinder）、惯性 |
| `robot.srdf` | Planning Group（如 `arm` = joint1~7）、**ACM 自碰撞忽略对**、命名姿态 `home` |

用 **MoveIt Setup Assistant** 生成 SRDF：定义哪些连杆对永远不用测碰撞（相邻连杆），大幅加速碰撞检测。

#### 第二层：运行时节点

```
move_group          ← 中枢：收规划请求、调规划器、维护 Planning Scene
robot_state_publisher ← 发 TF 树
/joint_states       ← 当前关节角（必须订阅）
```

#### 第三层：代码调用

```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="my_planner")
arm = moveit.get_planning_component("arm")

# 笛卡尔目标：末端位姿
arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="end_effector")

# 或关节空间目标
arm.set_goal_state(configuration_name="home")

plan_result = arm.plan()
if plan_result:
    moveit.execute(plan_result.trajectory, controllers=[])
```

#### 第四层：规划器内部（OMPL + FCL）

```
arm.plan()
    ↓
MoveIt 构造 MotionPlanRequest
    - start_state ← 当前 /joint_states
    - goal_state  ← 目标位姿经 IK 转成关节目标，或直接关节角
    ↓
OMPL 采样规划（默认 RRTConnect：双向 RRT，速度快）
    - 在关节空间随机采样 q_rand
    - 扩展树：q_near → q_new
    - 每个 q_new：调用 FCL 做碰撞检测
    ↓
找到无碰撞路径 → shortcut 平滑 → 时间参数化
    ↓
输出 JointTrajectory（带 time_from_start 的 q 序列）
    ↓
FollowJointTrajectory Action → 底层 controller / 自研 SDK
```

#### 第五层：笛卡尔直线（抓取接近段）

```cpp
// 末端沿直线插补，保证垂直插入
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(start_pose);
waypoints.push_back(end_pose);

moveit_msgs::RobotTrajectory trajectory;
double fraction = move_group.computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
// fraction < 1.0 表示因奇异/限位未走完，需加中间路点或改步长
```

---

### 2.4 碰撞检测：具体怎么做

**Planning Scene 维护两类信息：**

1. **机器人自身**：各 link 的 collision body（来自 URDF），做**自碰撞检测**（ACM 跳过相邻 link）。
2. **环境障碍物**：`CollisionObject`（box/mesh）— 地面、货架、工装，可动态增删。

**FCL 检测流程：**

```
对每个候选关节配置 q：
    FK 算出各 link 的 collision body 位姿
    ↓
    自碰撞：非 ACM 的 link 对做 broad phase + narrow phase
    环境碰撞：每个 link vs 每个 obstacle
    ↓
    任一碰撞 → 该采样点无效
```

**码垛 / 工业场景实践：**

- 障碍物位置相对固定 → 直接在 Planning Scene 里**手摆 CollisionObject**，不必上实时 SLAM。
- 动态障碍（人、移动箱）→ 感知（点云/nvblox）更新 Planning Scene。

---

### 2.5 规划器选型（面试常问）

| 规划器 | 类型 | 特点 | 适用 |
|--------|------|------|------|
| **RRTConnect** | 采样 | MoveIt 默认，双向搜索，快 | 通用 demo、首次跑通 |
| **RRT\*** | 采样 | 渐近最优，路径更短 | 路径质量要求高 |
| **CHOMP** | 优化 | 梯度优化，轨迹平滑 | 重复轨迹、码垛 |
| **Pilz** | 工业 | 严格速度/加速度约束 | 产线节拍 |

**我的项目说法：**

> Demo 阶段用 **MoveIt2 + OMPL RRTConnect** 跑通；取货/放货段用 **computeCartesianPath** 做笛卡尔直线接近；回原点用**关节空间规划**（更快、可预测）。若后期要毫秒级重规划，可考虑 **cuRobo / isaac_ros_cumotion**（GPU），但那是优化项不是初版必需。

---

### 2.6 与自研 SDK 的衔接

```
MoveIt2 输出 JointTrajectory (q[], t[])
    ↓
轨迹插值节点（100Hz~1kHz）
    ↓
SDK.setJointPosition(q_d) + 可选 G(q) 前馈
    ↓
UDP → 关节模组 PID
```

MoveIt 给的是**稀疏路径点**；执行层要做**时间插值**（样条/线性），保证连续速度。

---

### 2.7 本节可能的追问

| 追问 | 回答要点 |
|------|----------|
| 规划失败怎么办？ | 换 IK seed、放宽目标姿态、关节空间绕路、缩小障碍物 |
| 关节空间 vs 笛卡尔？ | 大范围移动用关节空间；接近抓取用笛卡尔直线 |
| OMPL 和 CHOMP 区别？ | 采样 vs 优化；CHOMP 要好的初始猜测，轨迹更平滑 |
| 看过哪些源码？ | `move_group` 入口 → `ompl_interface` → `planning_scene` 碰撞 |

---

## 三、Pinocchio / KDL 逆解规划框架

### 3.1 框架对比（先建立地图）

| 框架 | 特点 | 典型用途 |
|------|------|----------|
| **KDL** | ROS 生态老牌，轻量，Tree/Chain | MoveIt 默认 FK/IK 插件 |
| **Pinocchio** | 高效 C++，解析 URDF，支持 \(M,C,G\)、自动微分 | 动力学、优化 IK、MPC |
| **Trac-IK** | KDL + NLopt，兼顾速度和解质量 | MoveIt IK 插件 |
| **CasADi + Pinocchio** | 符号优化 IK | 约束多、冗余臂 |

**我的项目说法：**

> 运动学 SDK 里有一套自研臂角法 IK；在需要**和 URDF 严格一致的动力学**、或 MoveIt 里做**数值 IK / 碰撞约束优化**时，用 **Pinocchio** 加载同一份 URDF，保证参数一致。MoveIt 侧默认 **KDL** 做 FK 和简单 IK seed。

---

### 3.2 Pinocchio 从 URDF 得到什么

**加载流程：**

```python
import pinocchio as pin

# 1. 解析 URDF → 模型
model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

# 2. 可选：加载 collision 模型（hpp-fcl）
geom_model = pin.buildGeomFromUrdf(model, "robot.urdf", pin.GeometryType.COLLISION)
```

**从 URDF 自动提取的信息：**

| 信息 | 用途 |
|------|------|
| 关节树拓扑、父子关系 | FK 链、Jacobian 结构 |
| `nq`, `nv`（位形/速度维度） | 状态向量大小 |
| 关节限位 `lower/upper` | IK 约束 |
| 各 link **质量、质心、惯量张量** | \(M(q), G(q)\) |
| 关节轴向、原点 | DH 验证、FK |
| Collision geometry | 自碰撞 / 与 Pinocchio 可视化 |

**不直接给、需自己处理的：**

- 摩擦参数（URDF 里 `<dynamics>` 常不准）
- 减速比、电机侧 vs 关节侧（需在 SDK 层换算）

---

### 3.3 Pinocchio 常用计算流程

#### 正运动学

```python
q = pin.neutral(model)  # 或当前关节角
pin.forwardKinematics(model, data, q)
pin.updateFramePlacements(model, data)

# 末端位姿
T_ee = data.oMf[ee_frame_id]  # SE3
```

#### Jacobian

```python
pin.computeJointJacobians(model, data, q)
J = pin.getFrameJacobian(model, data, ee_frame_id, pin.LOCAL_WORLD_ALIGNED)
```

#### 动力学（重力补偿）

```python
pin.computeGeneralizedGravity(model, data, q)
G = data.g  # τ_gravity

# 或完整 RNEA
tau = pin.rnea(model, data, q, v, a)  # M(q)a + C(q,v)v + G(q)
```

---

### 3.4 用 Pinocchio 做 IK 的详细流程

**两种路线（我项目里可能组合使用）：**

#### 路线 A：解析 IK（SDK 臂角法）+ Pinocchio 验证

```
Pinocchio FK( q_ik )  vs  T_target  →  验证位置/姿态误差 < ε
```

臂角法求出 \(q\) 后，用 Pinocchio FK 做交叉验证，确保 URDF 与 DH 一致。

#### 路线 B：数值优化 IK（Pinocchio + CasADi / scipy）

适合约束多、臂角法失败时的 fallback：

```
决策变量：q ∈ R^7

目标函数：
    min  w_p || p_ee(q) - p* ||²  +  w_R || log(R_ee(q)^T R*) ||²  +  w_reg || q - q_ref ||²

约束：
    q_min ≤ q ≤ q_max
    可选：碰撞 constraint（Pinocchio + hpp-fcl distance > d_min）

求解：SQP / IPOPT / 阻尼最小二乘 Jacobian 迭代

迭代一步（Jacobian 伪逆法）：
    Δq = J^† Δx  +  (I - J^† J) · Δq_null   ← 零空间调节冗余，如固定臂角趋势
```

**Jacobian 迭代伪代码：**

```python
q = q_init
for i in range(max_iter):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    err = log6(T_ee.inverse() * T_target)  # SE3 误差
    if norm(err) < tol: break
    J = pin.computeFrameJacobian(...)
    dq = J.T @ solve(J @ J.T + damp*I, err)
    q = integrate(model, q, dq * dt)
    q = clip(q, q_min, q_max)
```

---

### 3.5 KDL 在 MoveIt 里的角色

```
MoveIt 收到笛卡尔目标 pose
    ↓
KDL ChainIkSolverPos_LMA / Trac-IK
    ↓
输出一组 joint seed → 作为 OMPL 的 goal_state
    ↓
若 IK 失败 → 规划直接失败（error code: NO_IK_SOLUTION）
```

**Pinocchio vs KDL 我怎么说：**

> **KDL** 嵌在 MoveIt 规划管线里，负责「目标位姿 → 关节目标」；**Pinocchio** 我在 SDK 和动力学前馈里用，同一份 URDF，算 \(G(q)\) 和做 IK 验证/优化。两者服务不同层，不冲突。

---

### 3.6 本节可能的追问

| 追问 | 回答要点 |
|------|----------|
| URDF 惯量和真机差多少？ | CAD 估重常偏差 10–20%；重力补偿仍有效，精细 CTC 要辨识 |
| 冗余臂零空间？ | \( \Delta q_{null} = (I - J^+J) z \)，可优化臂角、避障 |
| Pinocchio 为什么快？ | sparse 结构、C++、缓存友好；比纯 Python 循环快一个数量级 |

---

## 四、π0 折纸 VLA 微调项目

### 4.1 项目背景（30 秒版）

> 任务是让机械臂**折纸**。采用 **π0（Physical Intelligence）** 作为 VLA 基座：PaliGemma 视觉-语言骨干 + **Flow Matching 连续动作头**，输出高频关节动作。数据通过**遥操作**采集约 **2–3 小时**成功 demo，在预训练 π0 上做**微调（LoRA / 全量 action head）**，再在仿真和真机上做闭环推理验证。

---

### 4.2 π0 架构（面试必须讲清）

```
┌─────────────────────────────────────────────────────────┐
│  输入（每控制步 / 每 chunk）                              │
│  ├── 多相机 RGB 图像（如 1–3 路）                         │
│  ├── 语言指令（可选，如「折这一步」）                      │
│  └── 本体感知 proprio（关节角 q、夹爪状态）               │
└──────────────────────────┬──────────────────────────────┘
                           ↓
┌──────────────────────────▼──────────────────────────────┐
│  PaliGemma VLM 骨干                                      │
│  图像 token + 文本 token → 多模态特征                      │
└──────────────────────────┬──────────────────────────────┘
                           ↓
┌──────────────────────────▼──────────────────────────────┐
│  Flow Matching 动作专家（Action Expert）                   │
│  从噪声 → 连续动作向量（非离散 token）                      │
│  输出：未来 H 步 action chunk [a_t, ..., a_{t+H-1}]       │
└──────────────────────────┬──────────────────────────────┘
                           ↓
                    关节 Δq 或绝对 q → 底层 SDK / 控制器
```

**与 RT-2 / OpenVLA 的区别（一句话）：**

> RT-2 / OpenVLA 把动作**离散化成 token**；π0 用 **Flow Matching 直接回归连续动作**，频率更高（~50Hz）、精度更好，适合精细操作如折纸。

---

### 4.3 数据采集：遥操作 2–3 小时

**采集设备：**

- 主相机：固定俯视 / 腕部相机看纸面
- 遥操作：同构主从臂 / 空间鼠标 / VR 手柄 → 记录 master 关节角作为 action
- 同步：`header.stamp` 对齐图像、关节状态、动作

**采集原则：**

| 要点 | 说明 |
|------|------|
| 只录成功轨迹 | IL 阶段失败 demo 会拉低策略 |
| 覆盖变化 | 纸张初始位置小范围随机、光照变化 |
| 阶段清晰 | 折纸分步（对折、压平、翻边），每步可打 sub-task 标签 |
| 2–3 小时量级 | 约几百条 trajectory，对微调够用但偏少，靠预训练泛化 |

**数据量估算：**

```
2–3 h ≈ 7200–10800 s
若 10 Hz 录 ≈ 7–10 万 frame
若按 episode 30–60 s ≈ 120–360 条 trajectory
```

---

### 4.4 数据格式

**典型 LeRobot / OpenPI 兼容结构（按 episode 存）：**

```
dataset/
├── meta/info.json          # fps, feature schema, robot type
├── data/chunk-000/
│   └── episode_000.parquet
└── videos/chunk-000/
    └── observation.images.top/episode_000.mp4

每条 frame 字段示例：
{
  "timestamp": float,
  "observation.images.top": uint8[H,W,3],
  "observation.state": float32[7],      # 关节角
  "action": float32[7],                 # 目标 Δq 或 q_{t+1}
  "task": "fold the paper in half"       # 语言（可选）
}
```

**动作表示（关键细节）：**

| 选项 | 说明 |
|------|------|
| **Delta action** | \( a_t = q_{t+1} - q_t \)，泛化更好，常用 |
| **Absolute** | 直接预测 \( q_{t+1} \) |
| **归一化** | 用训练集 mean/std 或 min/max 归一化到 [-1,1] |
| **Action chunk** | 一次预测 H=16~50 步，执行前 k 步再 replan，减抖动 |

**图像预处理：**

- Resize（如 224×224）、ImageNet normalize 或模型指定 normalize
- 与预训练 π0 的输入分布对齐（**微调时不要轻易改**）

---

### 4.5 训练细节

#### 4.5.1 微调策略

| 策略 | 说明 |
|------|------|
| **冻结 VLM，只训 action head** | 数据少时首选，防过拟合 |
| **LoRA on VLM 顶层** | 2–3h 数据可小 rank（8–16），学 task-specific 视觉 |
| **学习率** | action head ~1e-4；LoRA ~1e-5 ~ 5e-5 |
| **Batch** | 受 GPU 显存限，常用 gradient accumulation |
| **Loss** | Flow Matching 的 velocity matching loss（非简单 MSE on action） |

#### 4.5.2 Flow Matching 训练直觉

```
训练时：
    取真实 action x_1，采样噪声 x_0 ~ N(0,I)
    插值 x_t = t·x_1 + (1-t)·x_0
    网络预测 velocity v_θ(x_t, t | obs)
    Loss = || v_θ - (x_1 - x_0) ||²

推理时：
    从噪声出发，多步 ODE 积分（如 5–10 步）→ 得到 action chunk
```

#### 4.5.3 超参参考（2–3h 小数据）

```
epochs: 50–200（看 val loss，易过拟合）
chunk size H: 16–32
执行 horizon k: 8–16（receding horizon）
augmentation: 颜色 jitter、小幅 crop（轻量，折纸对纹理敏感）
```

---

### 4.6 效果与精度（如何诚实 + 专业地描述）

**建议表述框架（按你实际情况微调数字）：**

| 维度 | 仿真 | 真机 |
|------|------|------|
| **单步动作 MSE** | 低（分布接近） | 高于 sim |
| **任务成功率** | 较高（60–80%+，若 sim 域随机化好） | 低于 sim（30–60% 区间常见，小数据微调） |
| **失败模式** | 偶发偏移 | 纸张滑动、光照变、累积误差 |

**面试话术：**

> 2–3 小时数据属于**小样本微调**，主要依赖 π0 预训练的视觉和动作先验。仿真里闭环成功率还可以，真机主要掉点在**纸张形变、接触滑动、相机标定误差**导致的 domain gap。精度上单步 joint 误差在几度量级，但折纸是**长程接触任务**，关键看**整段 episode 成功率**而不是单帧 MSE。

---

### 4.7 仿真 vs 真机推理流程

```
┌─────────────── 推理循环（50Hz 或按 chunk）───────────────┐
│  1. 读相机 + joint_states                                │
│  2. 预处理 → 与训练一致                                   │
│  3. π0 推理 → action chunk [a_t, ..., a_{t+H-1}]        │
│  4. 执行前 k 步 → SDK / ros2_control                     │
│  5. 重复（receding horizon）                             │
└──────────────────────────────────────────────────────────┘

仿真：Isaac Sim / MuJoCo，物理参数可随机化
真机：同一推理 node，底层换成 UDP SDK + 安全限位/急停
```

**Sim-to-Real 差距来源：**

- 纸的柔性未在仿真里精确建模
- 相机内外参、延迟
- 关节 backlash / 跟踪误差

---

### 4.8 微调改进思路（面试加分项）

按**投入产出**排序：

| 优先级 | 方法 | 说明 |
|--------|------|------|
| 1 | **加数据** | 再采 5–10h，覆盖失败边界情况 |
| 2 | **DAgger / 干预回放** | 自主跑时人介入修正，把修正轨迹加入训练集 |
| 3 | **Domain randomization** | 仿真里随机纸色、摩擦、初始位姿，预训练后再 sim2real |
| 4 | **Action chunk + 低频 replan** | 减累积误差；或缩短 k 提高反应 |
| 5 | **LoRA rank / 只训 action head** | 数据少时防过拟合；数据多了再解冻 VLM |
| 6 | **RECAP / RL 后训练** | π0.6 思路：自采轨迹 + 批评模型 + 优势加权微调 |
| 7 | **力/触觉** | 折纸压平阶段加力控或触觉传感器 |

**一句话总结改进路线：**

> 短期：**补数据 + DAgger**；中期：**仿真域随机 + 更准的 contact 建模**；长期：**RL 后训练**突破模仿上限。

---

### 4.9 本节可能的追问

| 追问 | 回答要点 |
|------|----------|
| 2–3 小时够吗？ | 微调够用，靠预训练；要鲁棒必须加量 |
| 为什么选 π0 不选 OpenVLA？ | 连续 Flow Matching，50Hz，适合精细接触 |
| 延迟怎么控？ | chunk 一次算多步；VLM 可降分辨率；TensorRT |
| 怎么评估？ | episode 成功率、关键步骤到达率、人工打分 |

---

## 五、四块内容的串联话术

**若面试官说「介绍一下你机械臂相关的经历」—— 2 分钟版：**

> 我做过一条完整链路：底层是蓝思 **7DOF 模组**，UDP 扫描 ID 后封装 **SDK**，实现了 **DH 正解**和**臂角法 IK**（固定第 4 轴解冗余），并用 Pinocchio 从 URDF 算重力补偿前馈。上层运动规划用 **MoveIt2**，OMPL 做关节空间采样规划，FCL 做碰撞检测，笛卡尔接近用 `computeCartesianPath`，轨迹通过 FollowJointTrajectory 接到 SDK。IK 方面 SDK 用解析臂角法，MoveIt 里 **KDL** 做 goal 转换，动力学和优化 IK 用 **Pinocchio** 保证和 URDF 一致。后面做折纸任务时，用 **π0** 预训练模型，**遥操作采了 2–3 小时**数据微调，Flow Matching 输出连续关节 action chunk，在仿真和真机做了闭环验证，真机主要挑战是接触和 domain gap，改进方向是加数据和 DAgger。

---

## 附录：快速对照表

| 话题 | 关键词 | 我用的工具/方法 |
|------|--------|----------------|
| 通信 | UDP、模组 ID 扫描 | 自研 SDK |
| IK | 7DOF 冗余、臂角法、固定 q4 | 解析几何 + Jacobian fallback |
| 动力学 | G(q) 重力补偿、CTC 理论 | Pinocchio RNEA |
| 规划 | RRTConnect、Planning Scene | MoveIt2 + OMPL + FCL |
| 碰撞 | ACM、CollisionObject | URDF + MoveIt |
| URDF 解析 | nq, 惯量, 限位 | Pinocchio / KDL |
| VLA | π0、Flow Matching、chunk | LoRA 微调 |
| 数据 | 遥操作、parquet、delta action | LeRobot 格式 |

---

*祝面试顺利。被追问细节时，**诚实边界 + 展示相邻知识**（如动力学辨识流程、OMPL 源码路径）比硬撑更有说服力。*
