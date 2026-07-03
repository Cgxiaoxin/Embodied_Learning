# MoveIt2 与 Isaac ROS 速查

> **类型**：面试 · **层级**：L3 · **状态**：稳定 · **更新**：2026-07-03  
> **关联**：[面试知识储备 §9](../面试知识储备.md#9-moveit2-轨迹规划与开源实现) · [机械臂运控 §8](./机械臂运控算法详解.md#八轨迹规划与路径规划) · [机械臂SDK面试梳理 §二](../../docs/projects/机械臂SDK与VLA项目面试梳理.md#二moveit-运动规划轨迹--碰撞)

MoveIt2 工程向速查笔记。完整面试答案见 [面试知识储备 §9](../面试知识储备.md#9-moveit2-轨迹规划与开源实现)。

---

## MoveIt2 定位

MoveIt2 不只是接口，是完整的运动规划框架：

```
你的代码 (Python/C++)
    ↓
MoveIt2
    ├── 机器人模型 (URDF + SRDF)
    ├── Planning Scene（世界状态）
    ├── 运动学 (KDL / Trac-IK)
    ├── 路径规划 (OMPL / CHOMP / STOMP / Pilz)
    ├── 碰撞检测 (FCL / Bullet)
    ├── 轨迹后处理（简化 + 时间参数化）
    └── 轨迹执行 (FollowJointTrajectory)
```

MoveIt2 = 模型 + 场景 + 规划 + 碰撞 + 后处理 + 执行。

---

## SRDF 生成（Setup Assistant）

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

| 步骤 | 产出 |
|------|------|
| Load URDF | — |
| Self-Collisions | ACM 自碰撞忽略对 |
| Planning Groups | `arm` = joint1~joint7 |
| Robot Poses | `home`、`ready` 命名姿态 |
| End Effectors | 末端执行器配置 |
| Generate | `robot.srdf` + launch/yaml |

**ACM 作用：** 相邻连杆碰撞体常重叠，ACM 跳过误报对，加速检测。

---

## 完整流程：从代码到机械臂动起来

### 运行时节点

| 节点 | 职责 |
|------|------|
| `move_group_node` | 接收规划请求，调度规划器，维护 Planning Scene |
| `robot_state_publisher` | 发布 TF 树 |
| `planning_scene_monitor` | 感知更新障碍、执行监控（可选） |

### Python 调用

```python
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="my_planner")
arm = moveit.get_planning_component("arm")
arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="end_effector")
plan_result = arm.plan()
if plan_result:
    moveit.execute(plan_result.trajectory, controllers=[])
```

### `plan()` 内部（OMPL 管线）

```
arm.plan()
    ↓
① MotionPlanRequest（起点/终点/规划组/约束）
    ↓
② OMPL 关节空间采样（RRTConnect 等）
    ↓
③ 每点/每段 → StateValidityChecker → FK + FCL 碰撞
    ↓
④ 几何路径（无时间）
    ↓
⑤ shortcut 路径简化
    ↓
⑥ IPTP/TOTG 时间参数化 → JointTrajectory
    ↓
⑦ FollowJointTrajectory → controller
```

| 规划器 | 特点 | 场景 |
|--------|------|------|
| RRTConnect | 默认，双向 RRT，快 | 通用 |
| RRT* | 渐近最优 | 路径质量 |
| CHOMP | 梯度优化平滑 | 码垛重复轨迹 |
| Pilz | 工业节拍 | 产线 |

---

## OMPL 碰撞检测（面试必背）

> **OMPL 不做碰撞**；回调 MoveIt Planning Scene + FCL。  
> **规划时几何检测**，非执行时力控。

```
采样 q → FK → FCL(AABB/BVH → GJK/EPA) → 自碰撞(减ACM) + 环境障碍
```

| 维度 | 说明 |
|------|------|
| 类型 | 几何穿透/距离 |
| 时机 | 规划阶段，每 state + 路径插值点 |
| 频率 | 一次 plan 可能数万次查询 |
| 实时避障 | 需 scene monitor 重规划 或 cuMotion |

**规划时碰撞 ≠ 执行时避障：** 经典 MoveIt 是 plan-then-execute；毫秒级反应式需 GPU 规划器。

---

## 笛卡尔路径

```cpp
double fraction = move_group.computeCartesianPath(
    waypoints, 0.01, 0.0, trajectory);
// fraction < 1.0 → IK失败/奇异/碰撞/步长过大
```

---

## 轨迹执行

```
MoveIt2 → trajectory_execution_manager
    ↓
FollowJointTrajectory action → ros2_control / Isaac Sim
```

---

## Isaac ROS 与 MoveIt2

Isaac ROS **不替代** MoveIt2，加速感知层：

```
相机/激光 → Isaac ROS → Planning Scene
              ├── nvblox (3D重建)
              └── isaac_ros_cumotion (GPU 5–20 ms)
```

| 组件 | 作用 |
|------|------|
| **nvblox** | 实时 3D 重建，动态障碍 |
| **isaac_ros_cumotion** | GPU 规划替代 OMPL（200–500 ms → 5–20 ms） |

---

## 工程选型

| 场景 | 建议 |
|------|------|
| Demo / 初版 | MoveIt2 + RRTConnect |
| 实时重规划 | cuMotion |
| 固定障碍（码垛） | 手摆 CollisionObject |
| 抓取接近段 | `computeCartesianPath` |
| 回原点 | 关节空间规划 |

---

## 面试三句话

1. **架构：** URDF/SRDF + Planning Scene + OMPL 采样 + FCL 碰撞 + 时间参数化 + FollowJointTrajectory。  
2. **SRDF：** Setup Assistant 生成 Planning Group、ACM、命名姿态。  
3. **碰撞：** OMPL 回调 MoveIt；规划时几何检测；执行信任轨迹，变化需 replan。
