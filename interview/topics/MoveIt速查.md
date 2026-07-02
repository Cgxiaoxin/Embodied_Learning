# MoveIt2 与 Isaac ROS 速查

> **类型**：面试 · **层级**：L3 · **状态**：稳定 · **更新**：2026-07-03  
> **关联**：[面试知识储备 §9](../面试知识储备.md#9-moveit2-轨迹规划与开源实现) · [机械臂SDK面试梳理 §二](../../docs/projects/机械臂SDK与VLA项目面试梳理.md#二moveit-运动规划轨迹--碰撞)

MoveIt2 工程向速查笔记。完整面试答案见 [面试知识储备 §9](../面试知识储备.md#9-moveit2-轨迹规划与开源实现)。

---

## MoveIt2 定位

MoveIt2 不只是接口，是完整的运动规划框架：

```
你的代码 (Python/C++)
    ↓ 调用
MoveIt2 框架
    ├── 运动学计算 (KDL / KDL_parser)       ← 正逆运动学
    ├── 路径规划器 (OMPL / CHOMP / STOMP)   ← 生成无碰撞轨迹
    ├── 碰撞检测 (FCL / Bullet)             ← Planning Scene
    └── 轨迹执行 (FollowJointTrajectory)    ← 下发给机器人
```

MoveIt2 = 运动学 + 路径规划 + 碰撞检测 + 执行接口。

---

## 完整流程：从代码到机械臂动起来

### 第一层：URDF / SRDF 配置（前置必做）

| 文件 | 作用 |
|------|------|
| `robot.urdf` | 机械臂物理结构（连杆、关节、碰撞体） |
| `robot.srdf` | 规划组、自碰撞禁止对、末端执行器（MoveIt Setup Assistant 生成） |

SRDF 需定义 Planning Group（如 `arm` = joint1~joint7）和 Allowed Collision Matrix。**这一步做错，后面全错。**

### 第二层：运行时节点

| 节点 | 职责 |
|------|------|
| `move_group_node` | 接收规划请求，调度规划器，维护 Planning Scene |
| `robot_state_publisher` | 发布 TF 树 |
| `rviz2`（可选） | 可视化 + 手动设目标 |

### 第三层：Python 调用示例

```python
from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped

moveit = MoveItPy(node_name="my_planner")
arm = moveit.get_planning_component("arm")

# 笛卡尔目标
target_pose = PoseStamped()
target_pose.header.frame_id = "base_link"
target_pose.pose.position.x = 0.5
target_pose.pose.position.z = 0.3
target_pose.pose.orientation.w = 1.0
arm.set_goal_state(pose_stamped_msg=target_pose, pose_link="end_effector")

# 或关节空间目标
arm.set_goal_state(configuration_name="home")

plan_result = arm.plan()
if plan_result:
    moveit.execute(plan_result.trajectory, controllers=[])
```

### 第四层：规划器内部（OMPL）

```
arm.plan()
    ↓
OMPL：采样 → FCL 碰撞检测 → 路径平滑
    ↓
JointTrajectory（带时间戳的关节角序列）
```

常用算法：RRTConnect（默认）、RRT*（渐近最优）、CHOMP（平滑优化）。

### 第五层：轨迹执行

```
MoveIt2 → trajectory_execution_manager
    ↓
FollowJointTrajectory action → 真实 controller / Isaac Sim ros2_control
```

---

## Isaac ROS 与 MoveIt2 的关系

Isaac ROS **不替代** MoveIt2，而是加速感知层，给 Planning Scene 提供更好的「眼睛」：

```
相机/激光雷达 → Isaac ROS 感知包 → MoveIt2 Planning Scene
                    │
            nvblox (3D重建)    isaac_ros_object_detection
                    ↓
          isaac_ros_cumotion (cuRobo GPU 规划，5–20 ms)
```

| 组件 | 作用 |
|------|------|
| **nvblox** | 实时 3D 重建，动态障碍物 |
| **isaac_ros_cumotion** | GPU 规划替代 OMPL（200–500 ms → 5–20 ms） |

---

## 工程选型建议

| 场景 | 建议 |
|------|------|
| Demo / 初版 | MoveIt2 + RRTConnect |
| 实时重规划 | 引入 cuMotion |
| 固定障碍物（码垛） | Planning Scene 手动加 CollisionObject，不必 nvblox |
| 抓取接近段 | 笛卡尔直线 `computeCartesianPath` |
| 回原点 | 关节空间规划（快且可预测） |
