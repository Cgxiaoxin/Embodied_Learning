# 机械臂运控 + VLA/RL 面试前 QA

> **类型**：面试 · **层级**：L3 · **状态**：稳定 · **更新**：2026-07-08  
> **用途**：围绕 4 个高频面试题，按 **Q/A** 形式给出可口述、可展开、可落地的回答。  
> **关联**：[MoveIt速查](./MoveIt速查.md) · [机械臂运控算法详解](./机械臂运控算法详解.md) · [运控知识点快速自查表](./运控双方向深度学习路线.md) · [π0.5 双臂L6项目实践](../../docs/projects/π0.5双臂L6折纸VLA与SAC微调项目实践.md) · [VLA算法入门](../../docs/theory/01-VLA算法入门.md) · [VLA与RL进阶](../../docs/theory/02-VLA与RL进阶.md)

---

## 一、MoveIt2 接口库如何做机械臂轨迹规划

### Q1：如果面试官问“你用 MoveIt2 怎么做机械臂轨迹规划”，我应该按什么顺序讲？

**A：建议按“建模 → 场景 → 目标 → 规划 → 时间参数化 → 执行 → 监控重规划”七步讲。**

#### 1. 前置建模

先把机器人描述完整，否则后面所有规划都不可靠：

- `URDF`：连杆、关节、限位、碰撞体、惯量
- `SRDF`：Planning Group、末端执行器、命名姿态、ACM 自碰撞忽略对
- 控制器接口：`FollowJointTrajectory` 或自研 SDK 桥接

**面试话术：** MoveIt2 本质不是一个“单接口库”，而是“机器人模型 + Planning Scene + OMPL 规划器 + 碰撞检测 + 时间参数化 + 执行器”的完整规划框架。

#### 2. 初始化 Planning Scene

把“当前机器人状态”和“环境障碍物”同步进去：

- 当前关节角来自 `/joint_states`
- 环境障碍来自桌面、工件、夹具、相机支架
- 机器人自碰撞规则由 SRDF 里的 ACM 决定

如果环境变了，比如视觉检测到新障碍物，需要更新 `Planning Scene`，否则轨迹规划出来也不安全。

> **补充说明见 [Q1.5](#q15planning-scene-如何建模场景信息怎么读进去和-mujoco-一样吗)**

#### 3. 定义目标

目标一般有两种：

- **关节空间目标**：例如回 `home`
- **笛卡尔目标**：例如末端移动到抓取位姿

如果是笛卡尔位姿目标，MoveIt2 先要经过 IK，把目标位姿转成关节目标或约束集合。

#### 4. 调规划器生成“几何路径”

典型是 `OMPL + RRTConnect`：

1. 从当前状态出发采样关节空间点
2. 每扩展一步都做状态有效性检查
3. 状态有效性检查内部会做：
   - FK 计算各连杆位姿
   - FCL/Bullet 碰撞检测
   - 关节限位检查
4. 找到一条从起点到终点的无碰撞路径

这里得到的还是**无时间信息的几何路径**，还不能直接下发给机械臂。

#### 5. 路径后处理和时间参数化

几何路径出来后通常要做：

- shortcut / simplify：去掉多余拐点
- 时间参数化：给每个路径点分配 `time_from_start`
- 满足速度/加速度上限

常见方法：

- IPTP
- TOTG

这一步之后才得到真正的 `JointTrajectory`。

#### 6. 执行轨迹

MoveIt2 通过 `trajectory_execution_manager` 下发：

```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="planner")
arm = moveit.get_planning_component("arm")
arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="tool0")
plan_result = arm.plan()
if plan_result:
    moveit.execute(plan_result.trajectory, controllers=[])
```

底层可以接：

- `ros2_control`
- 厂商控制器
- 自研 SDK

#### 7. 监控与重规划

执行时要监控：

- 是否偏离轨迹
- 是否超时
- 障碍物是否变化

如果场景动态变化，经典 MoveIt2 是 **plan-then-execute**，通常要重新规划；如果追求更强实时性，要引入 `cuMotion`、Servo 或更快的反应式层。

---

### Q1.5：Planning Scene 如何建模？场景信息怎么读进去？和 MuJoCo 一样吗？

**A：不一样。Planning Scene 是 MoveIt2 内部的「碰撞世界快照」，不是物理仿真器。**

#### 它是什么

Planning Scene 本质是一个**数据结构**，存两类东西：

```text
Planning Scene
├── RobotState          # 当前各关节角 → FK 算出各连杆位姿
├── Collision Objects   # 环境中的障碍物（几何体 + 位姿）
├── Attached Objects    # 抓在手上的物体（随末端一起动）
└── ACM                 # 允许忽略的自碰撞对（来自 SRDF）
```

MoveIt2 规划时只问一个问题：**这个关节构型会不会和障碍物/自身碰撞？** 它不算力、不算接触、不算物体会不会倒。

#### 和 MuJoCo / Isaac Sim 的区别

| 维度 | MoveIt2 Planning Scene | MuJoCo / Isaac Sim |
|------|------------------------|---------------------|
| 本质 | 碰撞检测用的几何快照 | 完整物理仿真引擎 |
| 物理 | ❌ 无重力、无摩擦、无接触力 | ✅ 有动力学、接触、柔性体 |
| 用途 | 离线/在线**运动规划** | **仿真验证、RL 训练、Sim-to-Real** |
| 更新频率 | 规划前或场景变化时 | 每仿真步（如 1ms） |
| 几何表示 | 简单 primitive（box/cylinder/mesh） | 完整 mesh + 物理属性 |

**面试一句话：** Planning Scene 像「3D 碰撞地图」，MuJoCo 像「物理世界」——MoveIt 用它做避障规划，不做物理仿真。

#### 场景信息怎么读进去

常见 4 种来源：

**① 静态配置（启动时加载）**

```python
# 从参数文件或 launch 里加载固定障碍
planning_scene = moveit.get_planning_scene()
box = CollisionObject()
box.id = "table"
box.primitives = [SolidPrimitive(type=BOX, dimensions=[1.0, 0.8, 0.05])]
box.pose = Pose(position=Point(x=0.5, y=0, z=-0.025))
planning_scene.apply_collision_object(box)
```

**② `/joint_states` 实时更新机器人状态**

```text
/joint_states → PlanningSceneMonitor → 更新 RobotState → FK 刷新各连杆位姿
```

**③ 感知模块动态添加障碍**

```text
深度相机 / 激光
    ↓
Octomap / nvblox / 点云聚类
    ↓
转成 CollisionObject（box 或 mesh）
    ↓
planning_scene_monitor 写入 Planning Scene
```

**④ 抓取后 attach 物体**

```python
# 物体被抓取后，从 world 移除，挂到末端 link 上
planning_scene.apply_attached_collision_object(attached_object)
# 这样规划时物体随手动，不会和自己碰撞
```

#### 内部建模流程

```text
URDF（机器人连杆 mesh/碰撞体）
    +
SRDF（Planning Group + ACM 自碰撞忽略对）
    +
CollisionObject（环境障碍）
    ↓
Planning Scene 维护一份「当前世界状态」
    ↓
OMPL 每采样一个关节构型 q
    ↓
FK(q) → 各连杆位姿
    ↓
FCL/Bullet 做碰撞检测（连杆 vs 障碍、连杆 vs 连杆）
    ↓
返回 valid / invalid
```

#### 面试 30 秒版

> Planning Scene 不是仿真器，是 MoveIt2 维护的碰撞世界快照：机器人当前状态 + 环境障碍物 + 手上物体。场景信息来自静态配置、joint_states 实时更新、或深度相机/Octomap 动态写入。它和 MuJoCo 的区别是：MoveIt 只做几何碰撞检测来规划无碰撞路径，不算物理力；MuJoCo 才是完整动力学仿真。

---

### Q2：MoveIt2 里“路径”和“轨迹”有什么区别？

**A：路径只管几何形状，轨迹还带时间。**

- **路径（path）**：从 A 到 B 经过哪些构型点
- **轨迹（trajectory）**：这些点分别在什么时刻到达，速度和加速度怎么分配

**一句话答法：** OMPL 先解无碰撞路径，时间参数化再把路径变成可执行轨迹。

---

### Q3：抓取任务里，什么时候用关节规划，什么时候用笛卡尔规划？

**A：大范围移动用关节空间，接近/插入/压合用笛卡尔直线。**

- **关节空间规划**：回零、绕障、长距离移动，快且容易找到解
- **笛卡尔规划**：抓取接近段、插孔、压装、沿法向压下，更容易控制末端直线运动

工程上常见组合是：

```text
预抓取位姿：MoveIt 关节空间规划
接近工件：computeCartesianPath
接触后：切到阻抗/力控
```

#### 笛卡尔路径代码示例

```python
# C++ MoveGroupInterface 伪代码
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(current_pose);
waypoints.push_back(pre_grasp_pose);   # 沿法向下降 10cm

moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group.computeCartesianPath(
    waypoints,
    0.01,    # eef_step：每步 1cm
    0.0,     # jump_threshold：禁止关节跳变
    trajectory,
    true     # avoid_collisions
);
# fraction < 1.0 说明路径未完全到达，需排查 IK/奇异/碰撞
move_group.execute(trajectory);
```

**面试要点：** `fraction` 是笛卡尔规划最常考的指标——1.0 表示全程可达，<0.9 通常需要换目标姿态或改用关节空间绕路。

---

### Q4：如果面试官追问“你实际做 MoveIt2 时最容易踩什么坑”，怎么答？

**A：重点答 5 个坑。**

1. **URDF/SRDF 不一致**：FK、碰撞、IK 全会偏
2. **碰撞体过细**：规划速度会明显下降
3. **目标姿态过严**：IK 经常失败
4. **笛卡尔路径 fraction 低**：通常是 IK 失败、奇异、碰撞或步长过大
5. **执行层和规划层没对齐**：MoveIt 规划的是一套关节顺序，SDK 执行是另一套映射

---

### Q5：这一题面试时最简洁的 30 秒版本怎么说？

**A：**

> 我用 MoveIt2 做轨迹规划时，会先准备 URDF/SRDF 和 Planning Scene，把当前关节状态和环境障碍同步进去；然后给 MoveIt 一个关节目标或末端位姿目标，内部通过 IK、OMPL 采样和 FCL 碰撞检测先求无碰撞路径，再做 shortcut 和时间参数化生成 `JointTrajectory`，最后通过 `FollowJointTrajectory` 或自研 SDK 执行。大范围位移我用关节空间规划，抓取接近和插入段用笛卡尔路径，接触后再切阻抗或力控。

---

## 二、动力学参数辨识，以及如何开发动力学力控功能

### Q1：机械臂动力学参数辨识，标准流程应该怎么讲？

**A：按“建模 → 激励 → 采集 → 回归 → 求参 → 验证”六步讲。**

#### 1. 建动力学方程

标准刚体动力学模型：

```text
τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q) + F(q̇)
```

**逐项解释：**

| 符号 | 名称 | 物理含义 | 直观理解 |
|------|------|----------|----------|
| `τ` | 关节力矩向量 | 各关节驱动器需要输出的力矩 | 「电机要出多大力」 |
| `q` | 关节角度 | 当前各关节位置 | 机器人「弯成什么姿势」 |
| `q̇` (qd) | 关节角速度 | 各关节转多快 | 运动有多快 |
| `q̈` (qdd) | 关节角加速度 | 各关节加速多快 | 运动变化有多急 |
| `M(q)` | 惯性矩阵 | 把加速度映射成所需力矩 | 姿势不同，同样加速度需要的力不同（大臂伸展时惯量大） |
| `C(q,q̇)·q̇` | 科氏力/离心力项 | 关节间耦合 + 旋转产生的惯性力 | 快速挥臂时 felt 到的「甩力」 |
| `G(q)` | 重力项 | 对抗重力所需的关节力矩 | 竖直抬起大臂 vs 水平伸出，所需力矩差很多 |
| `F(q̇)` | 摩擦力项 | 与速度相关的摩擦阻力 | 低速爬行、换向时的「粘滞感」 |

**一句话：** 这个方程回答的是——在当前姿势 `q`、速度 `q̇`、加速度 `q̈` 下，电机要输出多少力矩 `τ` 才能产生期望运动。

进一步写成线性参数化形式：

```text
τ = Y(q, q̇, q̈) · P
```

**逐项解释：**

| 符号 | 名称 | 含义 |
|------|------|------|
| `Y(q,q̇,q̈)` | Regressor 回归矩阵 | 把当前运动状态映射成「参数组合系数」的矩阵；只依赖可测量的 q/q̇/q̈，不依赖未知参数 |
| `P` | 待辨识参数向量 | 质量、质心、惯量、摩擦等物理量的**组合参数**（base parameters），不是单独一个质量值 |

**为什么要写成这个形式？**

原始方程里 `M(q)`、`C(q,q̇)`、`G(q)` 都含未知参数（各连杆质量、质心等），是非线性的。但把它们展开后，可以证明：

```text
τ = [已知函数(q,q̇,q̈)] × [未知参数组合 P]
```

右边对 `P` 是**线性的**，所以可以用最小二乘直接解：

```text
已知：每个时刻的 q, q̇, q̈, τ（实测）
算出：每个时刻的 Y(q, q̇, q̈)
求解：P = argmin ||τ - Y·P||²
```

**面试一句话：** `τ = YP` 是把非线性动力学「对参数线性化」，让辨识变成解线性方程组；`Y` 由运动状态算，`P` 是要估的物理参数组合。

#### 2. 设计激励轨迹

辨识的关键不是“跑一条普通轨迹”，而是让参数充分被激发出来。常见方法：

- 多频正弦
- 五次多项式拼接
- Fourier 级数激励

设计原则：

- 覆盖关节工作空间
- 同时激发速度和加速度项
- 避免长期停在单一姿态

**这一步的作用是什么？（核心理解）**

辨识本质是解方程 `τ = Y·P`。如果机器人只在一个固定姿势附近小幅度晃，会出现：

```text
问题：Y 矩阵各行几乎一样 → 方程「欠激励」→ P 解不唯一或噪声放大
```

类比：要估一个人的身高和体重，只测「同一个人站着不动」永远估不准；必须让他**做不同动作**，让不同参数「显形」。

| 激励目的 | 对应激发的参数 |
|----------|----------------|
| 大范围运动、多姿态 | 质量、质心、惯量（M, G 项） |
| 正反转、变速 | 科氏/离心项（C 项） |
| 低速正反扫 | 摩擦系数（F 项） |
| 竖直/水平姿态变化 | 重力项 G(q) |

**Persistent Excitation（持续激励）**：轨迹必须足够「丰富」，使 `Y` 矩阵满秩，参数才可辨识。

**面试一句话：** 激励轨迹不是随便跑一圈，而是专门设计的「体检动作」，让质量、惯量、重力、摩擦各项在不同姿态和速度下都有贡献，否则最小二乘解出来的参数不可信。

#### 3. 采集数据

至少采：

```text
q    — 关节角度（rad 或 deg），来自编码器
q̇   — 关节角速度（rad/s），编码器差分或驱动器直接给
q̈   — 关节角加速度（rad/s²），对 q̇ 数值微分（需滤波，噪声最大）
τ    — 关节力矩（N·m），来自关节电流 × 力矩常数，或驱动器力矩反馈
```

**为什么 q̈ 最难采？** 速度差分一次 → 加速度差分两次，噪声被放大；通常对 q̇ 低通滤波后再微分，或直接用 regressor 形式避免显式求 q̈。

同时要做：

- 时间同步
- 低通滤波
- 数值微分抗噪

因为 `q̈` 最容易被噪声放大。

#### 4. 堆叠回归方程

把所有采样点堆起来：

```text
[τ1]   [Y1]
[τ2] = [Y2] P
[...]  [...]
```

形成超定方程组后，用：

- 最小二乘 `LS`
- 加权最小二乘 `WLS`
- 递推最小二乘 `RLS`

去解 `P`。

#### 5. 得到参数后做物理筛选

不能只看数学误差，还要看物理合理性：

- 质量是否为正
- 惯量矩阵是否正定
- 摩擦参数是否符合速度方向

否则可能出现“拟合上看着对，实际不可用”的假解。

#### 6. 独立轨迹验证

用一条**没参与辨识**的验证轨迹，检查：

- 力矩预测误差
- 重力项误差
- 轨迹跟踪误差是否改善

**这一步的作用是什么？**

辨识是在「训练轨迹」上拟合出参数 `P`，但可能**过拟合**——在训练轨迹上误差小，换一条轨迹就崩。独立验证轨迹的作用是：

```text
用新轨迹采集 q, q̇, q̈, τ_meas
    ↓
用辨识出的 P 预测：τ_pred = Y(q,q̇,q̈) · P
    ↓
比较 τ_pred vs τ_meas
    ↓
误差小 → 参数泛化好，可以上线
误差大 → 参数不可信，需重新激励或加正则
```

| 验证指标 | 含义 | 合格参考 |
|----------|------|----------|
| 力矩预测 RMSE | 模型能否预测真实力矩 | < 10–15% 额定力矩 |
| 重力项误差 | G(q) 是否准（竖直姿态最明显） | 静态持臂不下滑 |
| 轨迹跟踪误差 | 前馈补偿后跟踪是否改善 | 比无补偿时明显减小 |

**为什么不能只用训练轨迹验证？** 和机器学习一样——训练集 loss 低不代表泛化好。独立轨迹是「测试集」。

**面试话术：** 真正工程里最有价值的通常不是一步到位把全部 `M/C/G/F` 做满，而是先把 `G(q)` 和摩擦项辨识准，马上就能提升低速稳态和竖直方向控制效果。

---

### Q1.5：重力补偿前馈和摩擦补偿分别是什么？怎么做？

**A：两者都是「前馈补偿」——在 PID 反馈之前，先根据模型算出一部分已知力矩，减轻反馈环负担。**

#### 重力补偿前馈 G(q)

**问题：** 机械臂在不同姿势下，重力对关节力矩影响不同。竖直抬起大臂需要很大力矩，水平伸出又不同。如果只靠 PID 位置环，PID 要一直「扛着」重力，积分项会累积，换姿势时容易超调或稳态误差。

**做法：** 用动力学模型算出当前姿势下的重力项，直接加到力矩指令里：

```text
τ_cmd = τ_PID + G(q)
         ↑反馈    ↑前馈（重力补偿）
```

**G(q) 从哪来？**

```text
URDF（各连杆质量、质心）
    ↓
Pinocchio / KDL / RNEA 算法
    ↓
输入当前 q → 输出 G(q) 向量
```

**效果：** 竖直方向持臂时，电机几乎不用额外出力，PID 只负责跟踪小偏差。

#### 摩擦补偿 F(q̇)

**问题：** 关节有静摩擦、库仑摩擦、黏性摩擦。低速时摩擦力和运动方向相反，导致：

- 换向时有「死区」——指令变了但关节不动
- 低速跟踪出现爬行、锯齿波
- 力控时力读数被摩擦污染

**常见摩擦模型：**

```text
F(q̇) = Fc · sgn(q̇) + Fv · q̇
       ↑库仑摩擦      ↑黏性摩擦
       （与方向有关）  （与速度成正比）
```

**做法：** 估计或辨识出 Fc、Fv，在控制指令里反向补偿：

```text
τ_cmd = τ_PID + G(q) + F(q̇)
                        ↑摩擦补偿
```

**辨识方法：** 单关节低速正反匀速转动，测 τ vs q̇，拟合 Fc 和 Fv。

#### 两者对比

| | 重力补偿 G(q) | 摩擦补偿 F(q̇) |
|--|---------------|----------------|
| 依赖 | 当前姿势 q | 当前速度 q̇ |
| 主要改善 | 竖直持臂、换姿稳态 | 低速跟踪、换向响应 |
| 参数来源 | URDF 质量/质心（或辨识） | 低速扫频辨识 |
| 工程优先级 | **更高**（效果立竿见影） | 次之（低速/力控才明显） |

#### 代码示意（Pinocchio）

```python
import pinocchio as pin

model = pin.buildModelFromUrdf("robot.urdf")
data = model.createData()

q = get_joint_positions()   # 当前关节角
qd = get_joint_velocities()

# 重力补偿：RNEA 算法，加速度=0, 速度=0 → 只剩 G(q)
G = pin.rnea(model, data, q, np.zeros(n), np.zeros(n))

# 摩擦补偿（简化模型）
Fc, Fv = friction_params[joint_id]
F = Fc * np.sign(qd) + Fv * qd

# 前馈叠加
tau_ff = G + F
tau_cmd = tau_pid + tau_ff
```

**面试 30 秒版：**

> 重力补偿是用 URDF + RNEA 算出当前姿势的重力力矩 G(q)，前馈给控制器，让 PID 不用一直「扛着」自重。摩擦补偿是用 Fc·sgn(q̇)+Fv·q̇ 模型估计摩擦，反向补偿，改善低速爬行和换向死区。工程上优先做 G(q)，因为效果最明显；摩擦补偿在力控和精细低速任务里更重要。

---

### Q2：如果我现在只有位置环，怎么诚实回答“你做过动力学吗”？

**A：可以这样答，既真实又不显得浅。**

> 控制主环如果还是驱动器内部位置 PID，那我能落地的第一步通常是重力补偿前馈和摩擦补偿，而不是完整 CTC。实现上我会用 URDF + Pinocchio/KDL 建模，先算 `G(q)` 做前馈补偿；如果后续能拿到比较可信的电流/力矩反馈，再做 regressor 最小二乘辨识，逐步把摩擦项和惯性项补起来。

这类回答的重点是：**你知道完整方法，也知道当前硬件边界。**

---

### Q3：阻抗控制、导纳控制、力位混合控制分别是什么？面试怎么区分？

**A：按“谁更像位置环机器人，谁更像力矩环机器人”来区分。**

#### 1. 阻抗控制

核心思想：**给机器人定义一个期望机械阻抗。**

典型形式：

```text
M_d (xdd - xdd_d) + D_d (xd - xd_d) + K_d (x - x_d) = F_ext
```

含义是：

- 没受外力时，机器人跟踪期望位姿
- 受到外力时，允许像“弹簧-阻尼-质量”那样偏移

**适合：**

- 协作机器人
- 柔顺装配
- 打磨、接触、插孔

#### 2. 导纳控制

导纳和阻抗是“反过来”的：

- 输入：外力
- 输出：位移/速度修正

更适合：

- 底层只有位置控制接口
- 末端有六维力传感器

工程上很多工业臂因为只有位置接口，所以更常落地的是**导纳控制**。

#### 3. 力位混合控制

本质是按任务空间方向拆分：

- 某些方向控位置
- 某些方向控力

例如平面打磨：

- 平面法向：控接触力
- 平面切向：控运动轨迹

可以用选择矩阵 `S` 来表达。

---

### Q4：如果要“给某个方向施加力”，系统应该怎么做？

**A：本质是做任务空间的法向力控制，而不是直接拍脑袋给某个关节加力矩。**

#### 方案 1：机器人有力矩接口

可以直接做笛卡尔空间力控制：

```text
F_cmd = [0, 0, Fz, 0, 0, 0]
τ_cmd = J(q)^T F_cmd
```

再叠加：

- 重力补偿
- 摩擦补偿
- 关节限幅

#### 方案 2：只有位置接口

用导纳/阻抗外环：

1. 六维力传感器读到 `F_ext`
2. 只取目标方向误差，比如 `e_f = Fz_ref - Fz_meas`
3. 通过导纳方程生成一个位移修正 `Δz`
4. 把 `z_d = z_nominal + Δz` 发给位置控制器

#### 工程实现注意点

- 力传感器要先零漂标定
- 法向方向要和工具坐标系对齐
- 进入接触前需要限速
- 超过阈值必须触发保护退让

#### 导纳控制伪代码（只有位置接口时）

```python
# 1D 法向导纳：期望力 Fz_ref，实测 Fz_meas
e_f = Fz_ref - Fz_meas
# 导纳方程：M_d·zdd + D_d·zd + K_d·z = e_f
z_ddot = (e_f - D_d * z_dot - K_d * z) / M_d
z_dot += z_ddot * dt
z += z_dot * dt
# 把修正量叠加到名义轨迹
pose_cmd.position.z = z_nominal + z
sdk.setCartesianPose(pose_cmd)
```

**参数初值参考：** `K_d=500 N/m, D_d=50 N·s/m, M_d=1 kg`（需按实际系统整定）。

---

### Q5：如果要从 0 开发一个“阻抗/某方向施力”的功能，最实用的开发顺序是什么？

**A：建议按下面顺序落地。**

1. **先打通传感链路**：关节状态、TCP 位姿、六维力传感器
2. **先做重力补偿**：不然力控会被自重误差污染
3. **先做一维法向导纳**：最容易验证
4. **再扩到 3D 位置 + 1D 力控**
5. **最后再上完整 6D 阻抗**

因为真正最难的不是公式，而是：

- 坐标系没对齐
- 力传感器噪声大
- 接触切换抖动
- 执行器带宽不够

---

### Q6：这一题面试时最简洁的 30 秒版本怎么说？

**A：**

> 动力学辨识我会先把模型写成 `τ = YP`，再设计激励轨迹采集 `q/qd/qdd/τ`，通过最小二乘或加权最小二乘识别 base parameters，并用独立轨迹验证。工程上优先做的是重力补偿和摩擦补偿。力控开发上，如果底层有力矩接口，我会做 `τ = J^T F` 的任务空间力控或阻抗；如果只有位置接口，更实用的是导纳控制，把期望力误差转成位移修正。某方向施力本质是对该任务空间方向做力闭环，而不是直接给某个关节硬加力矩。

---

## 三、VLA + RL：数据预处理、SFT/LoRA、微调、测试、仿真、Sim-to-Real、SAC

### Q1：我已经用遥操作采了“机械臂 + L6 灵巧手”数据，第一步怎么做数据预处理？

**A：建议按“同步、清洗、重采样、动作定义、归一化、切片、标注”七步做。**

#### 1. 时间同步

把多路数据对齐：

- 顶视相机
- 腕部相机
- 关节状态
- 手指状态
- 力/触觉（如果有）
- 指令文本

先解决“同一时刻看到的图像和动作是不是一个世界状态”的问题，否则后面训练全是脏数据。

#### 2. 清洗无效段

去掉：

- 起录前静止段
- 收尾停顿段
- 遥操作误触
- 相机丢帧
- 标定明显漂移的 episode

如果模型是 BC/SFT 阶段，通常优先保留**成功轨迹**；失败恢复可以留到 RL 阶段。

#### 3. 重采样

这是最容易被忽略的一步。

- **OpenVLA 路线**：更适合 `5-10 Hz`
- **π0 / π0.5 / Octo / Diffusion 路线**：更适合 action chunk，可保留 `20-50 Hz`

如果原始控制 100Hz，但模型推理只有 5Hz，就必须：

- 降采样
- 或改成 action chunk

#### 4. 定义动作空间

典型有三种：

1. **关节增量 `Δq`**
2. **绝对关节角 `q_target`**
3. **末端增量 `Δx + gripper`**

对于双臂 + 灵巧手，最常见是：

```text
action = [左臂7维, 左手6维, 右臂7维, 右手6维]
```

更推荐用**增量动作**，因为：

- 学习稳定
- 对标定小误差更鲁棒
- 与 chunk 执行更匹配

#### 5. 归一化

常见两种：

- `mean/std`
- `quantile(q01/q99)`

对灵巧手数据，常常更推荐 **quantile**，因为指尖动作和接触动作容易长尾。

#### 6. 切成训练样本

如果是 chunk 模型：

```text
输入：当前观测 o_t
标签：未来 H 步动作 [a_t, a_t+1, ..., a_t+H-1]
```

如果是单步模型：

```text
输入：o_t
标签：a_t
```

#### 7. 打辅助标签

能加就加：

- 子任务标签：`reach / grasp / align / fold / crease`
- 成功标记
- 接触阶段标记
- 重置点

这些标签后面会帮助：

- 采样均衡
- 困难阶段加权
- SAC 奖励设计

---

### Q1.5：双臂 + L6 灵巧手遥操作数据，工程上具体长什么样？

**A：推荐统一成 LeRobot v2.1 格式，26 维增量动作 + 3 路相机 + 子任务标签。**

#### 典型数据规格（参考 π0.5 折纸项目）

| 项 | 数值 |
|----|------|
| 总 DOF | 26（左臂 7 + 左手 L6 6 + 右臂 7 + 右手 L6 6） |
| 控制/记录频率 | 50 Hz |
| 相机 | top + left_wrist + right_wrist，640×480 |
| 有效 episode | ~186 条成功轨迹（约 3h） |
| 子任务标签 | `align / pinch / fold / crease` |

#### 目录结构

```text
dataset/origami_dual_l6/
├── meta/
│   ├── info.json          # fps、action_dim=26
│   ├── stats.json         # quantile 归一化统计
│   └── tasks.jsonl        # 子任务指令
├── data/chunk-000/
│   └── episode_*.parquet  # 关节、动作、时间戳
└── videos/chunk-000/
    ├── observation.images.top/
    ├── observation.images.left_wrist/
    └── observation.images.right_wrist/
```

#### 动作向量 layout

```python
action = [
    dq_l_arm[0:7],   # 左臂增量
    dq_l_hand[0:6],  # L6 左手（5 指 + 1 整体开合/协同维）
    dq_r_arm[0:7],   # 右臂增量
    dq_r_hand[0:6],  # L6 右手
]
# chunk 标签：shape [H, 26]，H=16/32/50 视基座而定
```

#### 同步要点

- 多路相机 + `/joint_states` 用 `message_filters` 近似时间同步（slop≈20ms）
- 遥操作记录的是 **master 指令** 作为 action，slave 状态作为 observation
- 每 30–45 min 重新做 hand-eye 标定，防止腕部相机外参漂移污染数据

**面试话术：** 双臂 + L6 的数据预处理核心不是“多录几条”，而是把 26 维动作定义、3 路相机同步、子任务标签和归一化统计一次性对齐到训练框架的 schema，否则后面改模型比改数据还痛苦。

---

### Q2：SFT 和 LoRA 的区别是什么？

**A：SFT 是训练范式，LoRA 是参数更新方式，它们不是同一层概念。**

#### SFT 是什么

SFT = **Supervised Fine-Tuning**，在机器人里通常就是：

- 输入：图像 + 指令 + 本体状态
- 监督目标：动作标签
- 本质：行为克隆 `BC`

所以在 VLA 里，很多时候你可以直接说：

> 机器人里的 SFT，本质上就是“拿遥操作演示做监督微调”。

#### LoRA 是什么

LoRA = **Low-Rank Adaptation**，是参数高效微调方法：

```text
W' = W + BA
```

训练时：

- 冻结原始大权重 `W`
- 只更新低秩增量 `A/B`

#### 二者关系

- **SFT** 决定“怎么训练”
- **LoRA** 决定“训练哪些参数”

所以完整说法应该是：

> 我用 SFT 这个训练范式，采用 LoRA 这种参数高效方式去做微调。

---

### Q3：为什么很多 VLA 微调优先用 LoRA，而不是一上来全参微调？

**A：因为机器人任务通常是“小数据 + 大模型 + 强部署约束”，LoRA 更符合工程现实。**

主要原因有 4 个：

1. **显存和算力省**
   - 7B 级 VLA 全参微调成本太高
2. **不容易灾难性遗忘**
   - 保留原有语言和视觉泛化能力
3. **迁移更方便**
   - 换机器人、换末端、换任务时只切 adapter
4. **数据量通常不够支撑全参**
   - 100-500 条演示时，全参更容易过拟合

**面试上要补一句：**

> 不是所有情况都只能用 LoRA。如果机器人本体、任务分布、传感输入和预训练差得非常远，而且数据量足够，后面也可能需要部分解冻甚至全参微调。

---

### Q4：为什么这里会提到“LoRA 去微调 action chunk”？这个说法准确吗？

**A：更准确的表达是：用 LoRA 去适配“生成 action chunk 的相关层”，不是 LoRA 和 chunk 本身是同一个概念。**

要把这件事拆成两层：

#### 第一层：为什么要 action chunk

因为 VLA 推理慢，而机器人控制频率高。

例如：

- 模型推理：`5 Hz`
- 机械臂控制：`50 Hz`

那就不能每 20ms 跑一次大模型。解决方法是一次预测未来 `H` 步动作：

```text
一次前向 → 输出未来 10/16/32/50 步动作
控制器逐步执行
中途按 receding horizon 重新规划
```

这样做的收益：

- 解决推理延迟
- 轨迹更平滑
- 对接触任务更稳定

#### 第二层：为什么要用 LoRA 去适配它

因为本体变化最明显的地方，往往就在：

- 动作维度
- 动作分布
- 时间相关性
- 末端执行器控制习惯

也就是说，预训练模型“看懂任务”的能力可以保留，但“怎么把理解转成你这台机器人未来 H 步动作”的部分需要重点适配。

所以面试时可以这么说：

> 用 LoRA 微调 action chunk，本质是在尽量不破坏 VLM 通用语义能力的前提下，重点去适配动作生成相关层，让模型学会我这台双臂 + 灵巧手在未来一小段时域里的动作分布。

---

### Q5：如果让我讲完整的 VLA 微调步骤，应该怎么讲？

**A：建议按“选基座 → 整数据 → 改 action/head → 跑 SFT → 离线验证 → 仿真验证 → 真机验证”七步讲。**

#### 1. 选基座模型

按任务选：

- **OpenVLA**：开源完整，适合研究和快速起步
- **π0 / π0.5**：更适合连续高频精细操作
- **Octo**：轻量、快速适配

如果任务是双臂 + L6 灵巧手折纸、插接、压合，通常更偏向 **π0.5 / Diffusion / Flow Matching** 路线，而不是纯离散 token 路线。

#### 2. 对齐输入输出

要明确：

- 几路相机
- 文本指令格式
- proprio 维度
- action 维度
- action horizon `H`

例如：

```text
obs = [top_cam, left_wrist_cam, right_wrist_cam, q, hand_state, instruction]
action = [左臂7, 左手6, 右臂7, 右手6]
chunk_horizon = 16 或 32 或 50
```

#### 3. 改数据集适配器

常见要改：

- 自定义数据读取脚本
- 字段名映射
- 相机路径映射
- 指令模板
- 归一化统计文件

#### 4. 改策略头

至少要确认：

- `action_dim`
- `chunk_horizon`
- 是否输出 `Δq` 还是 `Δx`
- 是否增加 gripper/hand 特殊维度

#### 5. 先跑 SFT/BC

损失函数取决于动作头：

- OpenVLA 离散 token：`CrossEntropy`
- Diffusion/Flow：`MSE / diffusion loss / flow matching loss`

训练策略通常是：

- 冻结 vision encoder
- 冻结大部分 language backbone
- 对上层或动作相关层加 LoRA
- 动作头全训或重点训练

#### 典型配置示例（openpi + π0.5）

```yaml
model:
  type: pi05
  pretrained_path: lerobot/pi05_base
  action_dim: 26
  action_horizon: 50          # 一次预测 50 步，1s 前瞻 @ 50Hz
  num_inference_steps: 10     # Flow Matching ODE 去噪步数

lora:
  enabled: true
  rank: 16
  alpha: 32
  target_modules: [q_proj, v_proj]   # 仅 PaliGemma 顶层 4 层
  freeze_vlm_below_layer: 14

optimizer:
  lr_action_expert: 1.0e-4    # Action Expert 全量训
  lr_lora: 3.0e-5             # VLM LoRA 小学习率
  warmup_steps: 2000

training:
  batch_size: 8               # per GPU，2×4090 DDP
  gradient_accumulation: 4    # 有效 batch=64
  max_steps: 80000
```

**冻结策略记忆口诀：** ViT 全冻 → VLM 底层冻 + 顶层 LoRA → Action Expert 全训。

#### 6. 离线验证

离线不能只看训练 loss，还要看：

- 验证集动作误差
- chunk 逐步 rollout 是否发散
- 不同子任务成功率
- OOD 场景是否明显崩

#### 7. 仿真和真机验证

顺序建议：

```text
离线回放 → 仿真闭环 → 真机低速安全模式 → 真机全流程
```

---

### Q6：如果问“怎么改代码”，面试里应该怎么回答才像做过？

**A：不要泛泛说“改训练脚本”，要按模块说。**

#### 典型改动位置

```text
project/
├── configs/
│   ├── robot.yaml           # 相机、本体维度、action_dim、chunk_horizon
│   └── train.yaml           # lr、batch、LoRA rank、冻结策略
├── datasets/
│   └── my_robot_dataset.py  # 把遥操作数据转成模型要的 sample
├── models/
│   ├── policy.py            # 动作头输出维度、chunk 长度
│   └── adapters.py          # LoRA 注入层
├── train/
│   └── finetune.py          # SFT 主训练循环
├── deploy/
│   └── receding_horizon.py  # chunk 缓存与执行
└── rl/
    └── sac_residual.py      # SAC 残差策略或后训练
```

#### 你真正要改的 6 件事

1. **数据 schema**
   - 把你自己的多相机、双臂、灵巧手状态喂进模型
2. **动作维度**
   - 从单臂 7 维改成双臂 + 手的 26 维或更多
3. **chunk 长度**
   - 例如从 `H=8` 改到 `H=16/32/50`
4. **归一化统计**
   - 重新统计 `dataset_stats.json`
5. **LoRA 注入位置**
   - `q_proj/k_proj/v_proj/o_proj`，或只注入高层 block
6. **部署执行器**
   - 把模型输出转成 SDK/ROS2 能执行的命令

#### openpi 典型改动点（以 π0.5 为例）

```text
openpi/
├── config/
│   └── origami_pi05_il.yaml     # action_dim=26, horizon=50, LoRA rank
├── datasets/
│   └── lerobot_origami.py       # 读 LeRobot parquet + 3 路视频
├── models/
│   └── pi05_policy.py           # Action Expert 输出维度
└── deploy/
    └── receding_horizon.py      # chunk 缓存 + 每 8 步 replan
```

```python
# deploy/receding_horizon.py 核心逻辑
class RecedingHorizonExecutor:
    def __init__(self, replan_every=8):
        self.buffer = None
        self.step = 0
        self.replan_every = replan_every

    def get_action(self, obs, policy):
        if self.buffer is None or self.step >= self.replan_every:
            chunk = policy.infer(obs)          # shape [50, 26]
            self.buffer = chunk
            self.step = 0
        action = self.buffer[self.step]
        self.step += 1
        return action  # Δq → SDK setJointPosition + L6 CAN
```

#### 面试上最好加一句

> 真正最容易出问题的不是模型 forward，而是数据字段、action 维度、归一化和部署执行器这四件事对不齐。

---

### Q7：测试、仿真环境验证、真机 Sim-to-Real 应该怎么做？

**A：建议按“单元测试 → 离线回放 → 仿真 → 真机灰度 → 线上迭代”五层做。**

#### 1. 单元测试

测最基础的：

- 数据读取是否对齐
- 图像/状态/action 维度是否正确
- 归一化反归一化是否一致
- chunk 展开后是否越界

#### 2. 离线回放测试

给模型喂验证集观测，看预测动作与 GT 的差异：

- `MSE / MAE`
- chunk rollout drift
- 子任务分段误差

#### 3. 仿真闭环测试

仿真主要验证 4 件事：

1. 模型输出是否稳定
2. 动作是否满足关节限位
3. 接触前后的策略是否切换合理
4. 能否在分布内场景复现基本任务

仿真里要尽量加入：

- 相机噪声
- 光照变化
- 摩擦变化
- 物体初始位姿扰动

#### 4. 真机灰度验证

不要直接全速上真机，建议：

- 先限速
- 先限位
- 先加安全工作空间
- 先只测 reach / grasp 子任务
- 逐步放开到完整任务

#### 5. Sim-to-Real 优化主线

真正常见的问题：

1. **视觉外参漂移**
2. **关节零位偏差**
3. **摩擦/接触模型不一致**
4. **时延比仿真大**
5. **灵巧手动作分布和仿真手不一致**

最有效的优化手段通常是：

- 在线重标定或频繁重标定
- 动作 clamp
- 域随机化
- chunk 重规划更频繁
- 加残差 RL
- 接触阶段切阻抗/导纳

---

### Q8：如果要讲“如何用 SAC 去做 RL”，应该怎么讲？

**A：最好的讲法不是“从零训练一个 SAC”，而是“基于已训练好的 VLA 做残差或后训练”。**

#### 为什么这里选 SAC

因为你的场景是：

- 连续动作
- 维度高
- 演示数据少
- 接触精细阶段很重要

SAC 的优势：

- off-policy，样本效率高
- 可以复用 demo + rollout
- 最大熵机制更利于探索接触策略

#### 推荐做法：Residual SAC

不要直接让 SAC 从零学整条策略，而是：

```text
a_final = a_vla + λ * Δa_sac
```

含义：

- `a_vla`：大模型给出的基础动作
- `Δa_sac`：SAC 学的残差修正

这样好处是：

- 保留高层语义和大体动作结构
- 把 RL 重点用在接触、纠偏、恢复

#### SAC 流程

1. **先有 SFT/VLA 基础策略**
2. **在仿真中 rollout**
3. **构造 replay buffer**
   - 成功 demo
   - 失败轨迹
   - 在线探索数据
4. **设计 reward**
   - 到位奖励
   - 接触稳定奖励
   - 力过大惩罚
   - 碰撞惩罚
   - 超时惩罚
5. **训练 twin Q + actor + temperature**
6. **先仿真收敛，再小步上真机**

#### SAC 最关键的不是公式，而是 reward 和 state

对接触任务，建议把 state 里显式加上：

- 力/扭矩
- 接触标记
- 上一时刻动作
- VLA latent 或倒数第二层特征

#### 具体实现示例（Residual SAC，π0.5 折纸项目）

**状态定义（Critic 输入 ~256 维）：**

```python
s_t = concat([
    proprio_26,              # 当前关节角
    vla_latent_pooled,       # Action Expert 倒数第二层 mean pool → 128d
    phase_onehot,            # align/pinch/fold/crease 子任务阶段
    last_action_26,          # 上一步执行动作
])
```

**动作定义（残差叠加）：**

```text
a_exec = a_vla + α · Δa_sac
α: 0.3（前 20k step）→ 0.5（后期放开修正幅度）
```

**奖励 shaping（接触任务典型）：**

| 事件 | 奖励 |
|------|------|
| 完整任务成功 | +10.0 |
| 完成子步（折痕角度 < 5°） | +1.0 |
| 纸张滑出工作区 | -2.0 |
| 臂-臂碰撞 | -1.5 |
| 指节限位 | -0.3 |
| 每步时间惩罚 | -0.005 |
| crease 阶段法向力 ∈ [2N, 5N] | +0.1/step |

**训练流程：**

```text
IL checkpoint（真机 ~34% 成功率）
    ↓
真机自主 rollout → Replay Buffer（demo 权重 ×2）
    ↓
SAC Actor 用 IL 的 Action Expert 权重初始化（非从零）
Critic 随机初始化
    ↓
仿真预训练 → 真机小步 online fine-tune
    ↓
Knowledge Insulation：SAC 梯度只更新 Action Expert，VLM 冻结
```

**为什么 Residual 而不是从零 SAC：**

- 3h 演示数据不够支撑 26 维全策略探索
- VLA 已学会 coarse 轨迹（align/pinch/fold），RL 只需修正 crease 接触力
- 残差幅度 α 可控，真机安全性更高

#### 什么时候不该优先用 SAC

如果你连 BC/SFT 基础策略都没有，或者任务非常依赖语言长程推理，SAC 不一定是第一选择。此时应先把：

- 数据质量
- action 定义
- chunk 执行

这三件事打牢。

---

### Q9：这一大题面试时最简洁的 40 秒版本怎么说？

**A：**

> 我会先把遥操作数据做时间同步、清洗、重采样、动作定义和归一化，再按 action chunk 切成训练样本。机器人里的 SFT 本质是监督式行为克隆，LoRA只是参数高效微调方法，两者不是一个概念。我一般先基于 OpenVLA 或 π0.5 这类基座做 SFT，重点改数据适配器、动作维度、chunk 长度和 LoRA 注入位置，然后做离线回放、仿真闭环和真机灰度验证。接触类任务如果 IL 上限明显，我会在 VLA 基础上叠一个 Residual SAC，让 RL 只学接触和纠偏，而不是从零学整条大策略。

---

## 四、VLA 模型、RL 模型，以及当前学术/工业界前沿

### Q1：VLA 模型的共性架构是什么？

**A：可以统一总结成“三段式”。**

```text
视觉编码器 + 语言骨干 + 动作头
```

更细一点是：

```text
图像 / 多视角视频 / 文本 / 本体状态
        ↓
多模态 backbone（VLM / Transformer）
        ↓
动作解码器（离散 token / Diffusion / Flow Matching）
        ↓
单步动作或 action chunk
```

VLA 的核心，不是“把图像扔进一个控制器”这么简单，而是：

- 用大模型继承互联网语义知识
- 用机器人数据把语义 grounding 到动作
- 用 chunk/diffusion/flow 解决连续控制和推理延迟

---

### Q2：OpenVLA、π0、π0.5、π0.6、π0.7 的主线区别是什么？

**A：主线可以理解为：从“离散动作 + 可复现”，走向“连续动作 + 更强泛化 + 更强记忆/可控性 + 从经验学习”。**

#### 1. OpenVLA

- **定位**：当前最重要的开源研究基线之一
- **骨干**：Prismatic VLM，视觉常见为 DINOv2 + SigLIP，语言为 Llama 2 7B
- **动作形式**：离散 token
- **优点**：开源完整、易复现、LoRA 微调方便
- **短板**：精细连续控制上限不如 flow/diffusion 路线

#### 2. π0

- **定位**：Physical Intelligence 的连续动作 VLA 起点
- **骨干**：PaliGemma 类 VLM
- **动作形式**：Flow Matching 连续 action chunk
- **创新点**：
  - 从离散动作 token 转向连续 chunk
  - 更适合高频精细操作
  - 推理可配合 receding horizon 到 50Hz 级控制

#### 3. π0.5

- **定位**：在 π0 基础上增强开放世界泛化
- **核心变化**：
  - 更强 co-training / 数据规模
  - 更好的开放环境泛化
  - 更强调“看懂陌生家居场景后仍能动作泛化”
- **补充说明**：
  - 公开资料里，π0.5 这代常被一起讨论到 `FAST` 这类动作序列 token 化路线；面试时更稳妥的讲法是：**π0.5 代表的是从纯连续控制能力往“更强开放世界泛化 + 更强动作表示”演进的一代**，不要把它机械地简化成单一一种动作头。

#### 4. π0.6

- **定位**：从“模仿人类”走向“从自身经验继续学习”
- **核心变化**：
  - 更强 backbone（Gemma 3 4B 路线）
  - 引入 **Knowledge Insulation**
  - 配合 **RECAP** 做 RL 式后训练
- **创新点**：
  - 保护 VLM 语义能力，不让动作梯度破坏骨干知识
  - 允许从成功/失败 rollout 中继续学习

#### 5. π0.7

- **定位**：可控、更通用、更强调长时程上下文和可引导性
- **核心变化**：
  - 在 π0.6 基础上继续扩展
  - 引入历史观测编码与多模态上下文条件
  - 可加入子目标图像、episode metadata、策略偏好信息
- **创新点**：
  - 更强“steerability”
  - 更适合利用多源、甚至带噪和次优的数据
  - 更利于长时程和多阶段任务

---

### Q3：面试官如果问“这些模型到底差在哪”，最容易记住的对比法是什么？

**A：按 4 个维度记。**

| 维度 | OpenVLA | π0 | π0.5 | π0.6 | π0.7 |
|------|---------|----|------|------|------|
| 动作表示 | 离散 token | 连续 flow chunk | 连续 flow chunk | 连续 + 经验学习 | 连续 + 多模态可控上下文 |
| 优势 | 开源易复现 | 精细控制起点 | 开放世界泛化 | 可从 rollout 持续学习 | 长时程、更可引导 |
| 适合 | 研究、LoRA 微调 | 精细 manipulation | 家庭/开放环境 | RL 后训练 | 通用机器人基础模型 |
| 短板 | 高频控制一般 | 泛化不如后续版 | 仍偏闭源 | 工程门槛更高 | 最新、复现门槛最高 |

---

### Q4：RL 模型在具身智能里，面试主要要会讲哪几个？

**A：至少会讲 4 类。**

#### 1. SAC

- **类型**：off-policy
- **特点**：样本效率高，适合连续控制
- **典型用途**：灵巧手、接触、残差控制、力控微调

#### 2. PPO

- **类型**：on-policy
- **特点**：稳定、通用
- **典型用途**：人形步态、全身控制、仿真大规模训练

#### 3. GRPO / RECAP 类后训练

- **定位**：更适合 VLA 后训练
- **特点**：从整条轨迹质量做相对优势或 retrospective critique
- **典型用途**：让大模型从 rollout 成败继续变强

#### 4. World Model / Dreamer 路线

- **定位**：减少真实交互成本
- **特点**：先学环境模型，再在“想象空间”里训练策略
- **典型用途**：高样本成本场景，长期值得关注

---

### Q5：当前国外学术和工业界前沿，应该怎么扩展回答？

**A：建议分成 5 条主线。**

#### 1. Google DeepMind 路线

- **RT-1 / RT-2 / RT-X**
- 强项是：大规模语义泛化、跨机器人数据整合、语言 grounding
- 关键词：**VLM + robot data co-training**

#### 2. Stanford / Berkeley 开源路线

- **OpenVLA**
- **Octo**
- 强项是：开源、可复现、便于学术界微调和 benchmark
- 关键词：**Open X-Embodiment、PEFT、LoRA**

#### 3. Physical Intelligence 路线

- **π0 → π0.5 → π0.6 → π0.7**
- 强项是：连续动作 chunk、开放世界泛化、经验驱动后训练、可控性
- 关键词：**Flow Matching、Knowledge Insulation、RECAP、Steerability**

#### 4. NVIDIA 路线

- **GR00T N1 / N1.5 / 1.7**
- 强项是：双系统、与 Isaac Sim / Isaac Lab 深度结合、人形机器人生态
- 关键词：**慢思考 VLM + 快执行 DiT + Sim 数据工厂**

#### 5. Figure / 产业闭源路线

- **Helix**
- 以及与 humanoid 全身控制相关的闭源系统
- 强项是：完整软硬件闭环、真机规模化、长时任务集成

#### 6. 其他值得提及的国外路线

| 机构/公司 | 代表工作 | 特点 |
|-----------|----------|------|
| **Covariant** | RFM-1 | 工业仓储 VLA，强调跨 SKU 泛化 |
| **Skild AI** | Skild Brain | 通用机器人基础模型，跨本体 |
| **1X** | NEO + 内部 VLA | 人形 + 端到端，强调安全部署 |
| **Meta FAIR** | V-JEPA / 世界模型 | 自监督视频表征，为 RL/规划提供 latent |
| **Columbia / TRI** | Diffusion Policy | 连续 action chunk 的开源先驱 |
| **HuggingFace LeRobot** | 统一数据格式 + 训练框架 | 降低 VLA 微调门槛，π0/π0.5/Smolvla 均支持 |

**面试扩展话术：**

> 国外工业界和学术界正在收敛到「大 VLM 做语义理解 + 独立 Action Expert 做连续控制 + IL 预训练 + RL 后训练 + Sim 数据工厂」的分层架构。开源侧 OpenVLA/Octo/LeRobot 降低研究门槛，闭源侧 PI/NVIDIA/Figure 在真机规模和长时任务上领先。国内（智元、宇树、银河通用等）也在快速跟进，但数据规模和跨本体 co-training 仍是差距所在。

---

### Q6：如果面试官继续追问“未来通用机器人基础模型会往哪走”，怎么答更像有判断？

**A：给出下面 6 个判断最稳。**

1. **从单步动作转向 action chunk / long-horizon chunk**
2. **从离散 token 更多转向连续 diffusion / flow**
3. **从单次模仿学习转向 IL + RL 后训练**
4. **从单机器人数据转向跨本体统一表征**
5. **从静态 prompt 转向历史记忆、子目标图像、多模态条件控制**
6. **从“只会做”转向“可解释、可控、可部署”**

如果再加一句总结：

> 未来真正能落地的通用机器人大模型，不会只是一个大 VLM 接动作头，而是“语义大模型 + 动作基础模型 + 记忆/世界模型 + 安全控制层”的分层系统。

---

### Q7：这一题面试时最简洁的 40 秒版本怎么说？

**A：**

> VLA 的共同结构是视觉编码器、语言骨干和动作头。OpenVLA 是当前最重要的开源离散动作路线，适合研究和 LoRA 微调；π0 系列代表连续 action chunk 路线，更适合精细操作。π0.5 强调开放世界泛化，π0.6 通过 Knowledge Insulation 和 RECAP 开始从 rollout 经验继续学习，π0.7 进一步加入历史观测、多模态上下文和 steerability。RL 这边，接触和灵巧操作里我更看重 SAC，VLA 后训练更关注 RECAP/GRPO 这类方法。国外前沿主要是 Google 的 RT 系列、Stanford/Berkeley 的 OpenVLA/Octo、PI 的 π 系列、NVIDIA 的 GR00T，以及 Figure 这种闭源全栈路线。

---

## 五、面试建议：这 4 题怎么组织口述

### Q：如果面试时 4 题都问到，我应该怎么组织整体表达？

**A：建议统一成“传统运控层 + 学习策略层 + 落地验证层”三层。**

```text
第一层：MoveIt2 / 运动学 / 动力学 / 力控
第二层：VLA / SFT / LoRA / action chunk / SAC
第三层：测试 / 仿真 / Sim-to-Real / 真机闭环
```

这样你给面试官的感觉会是：

- 不是只会调模型
- 也不是只会传统控制
- 而是能把“规划、控制、学习、部署”串成一条完整链路

### Q：最后一句总括怎么说最有用？

**A：**

> 我的理解是，机械臂真正落地不是单点算法，而是“MoveIt 做无碰撞几何规划，动力学和力控解决接触执行，VLA 负责感知到动作的泛化，RL 负责在接触细节上超越纯示教，最后靠仿真和 Sim-to-Real 把整套系统收敛到真机上”。这条链路讲清楚，面试就会比较完整。

