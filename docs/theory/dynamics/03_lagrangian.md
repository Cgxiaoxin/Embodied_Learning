# 拉格朗日：能量法建模

> **类型**：理论 · **层级**：L1 · **状态**：草稿 · **更新**：2026-09-11  
> **关联**：[02 牛顿-欧拉](./02_newton_euler.md) · [动力学索引](./00_index.md) · [04 方程性质](./04_dynamics_equation_properties.md)

> 拉格朗日力学用标量能量 \(L=T-V\) 与广义坐标 \(q\)，经欧拉-拉格朗日方程得到与牛顿-欧拉等价的运动方程。其优势在于**结构清晰**：自然整理出 \(M(q)\ddot q+C(q,\dot q)\dot q+G(q)=\tau\)，便于控制器推导、参数辨识与性质分析（反对称性、被动性等）。

---

## 一、能量法直觉：\(L = T - V\)

对 \(n\) 自由度机械臂，选关节角 \(q\) 为广义坐标（与 [01 运动学回顾](./01_kinematics_review.md) 一致）。系统动能、势能分别为：

\[
T = \frac{1}{2}\sum_i \Big( m_i\, v_{c,i}^\top v_{c,i} + \omega_i^\top I_i \omega_i \Big),\qquad
V = \sum_i m_i\, g^\top r_{c,i} + V_{spring}(q)
\]

**拉格朗日函数**定义为动能减势能：

\[
L(q,\dot q) = T(q,\dot q) - V(q)
\]

直觉：\(T\) 刻画「动起来有多难」，\(V\) 刻画「重力/弹性把系统拉向哪里」。用 \(L\) 而非逐个连杆写 \(f,n\)，是为了在**统一标量框架**下自动消去约束力（理想关节不做功），并得到仅含 \(q,\dot q,\ddot q\) 的闭式方程。

对固定基座机械臂，\(T\) 是 \(\dot q\) 的二次型，\(V\) 只依赖 \(q\)；这直接预示了标准方程里会出现质量矩阵 \(M(q)\) 与重力项 \(G(q)\)。

---

## 二、欧拉-拉格朗日方程

对每一广义坐标 \(q_j\)，欧拉-拉格朗日方程为：

\[
\frac{d}{dt}\frac{\partial L}{\partial \dot q_j}
- \frac{\partial L}{\partial q_j} = \tau_j + \tau_{ext,j}
\]

其中 \(\tau_j\) 为关节驱动力矩，\(\tau_{ext,j}\) 为外力/接触力等效到关节的项（见 04 篇）。代入 \(L=T-V\) 并注意 \(T\) 不含 \(q_j\) 的显式偏导时 \(\partial T/\partial \dot q_j\) 给出广义动量：

\[
\frac{d}{dt}\frac{\partial T}{\partial \dot q_j}
- \frac{\partial T}{\partial q_j}
+ \frac{\partial V}{\partial q_j} = \tau_j + \tau_{ext,j}
\]

**推导骨架**（单自由度压缩版）：\(T=\frac{1}{2}m(q)\dot q^2\) 时 \(\partial T/\partial\dot q=m\dot q\)，对时间求导得 \(m\ddot q+\frac{\partial m}{\partial q}\frac{\dot q^2}{2}\)；\(-\partial T/\partial q\) 给出 \(\frac{\partial m}{\partial q}\frac{\dot q^2}{2}\) 的科氏/离心部分；\(\partial V/\partial q\) 为重力项。推广到 \(n\) 维即矩阵形式。

多连杆时，把各连杆 \(v_{c,i},\omega_i\) 用 \(q,\dot q\) 与雅可比表出，代入 \(T,V\) 再对 \(q_j\) 求导——符号工具（SymPy、Mathematica）或 Pinocchio 的符号接口常走此路径。

---

## 三、如何整理成 \(M(q)\ddot q + C(q,\dot q)\dot q + G(q) = \tau\)

由 \(T=\frac{1}{2}\dot q^\top M(q)\dot q\) 定义**质量矩阵** \(M(q)\)（对称正定，对树形臂满秩）。对欧拉-拉格朗日方程做标准整理，可得**标准操作空间形式**：

\[
M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q) = \tau + \tau_{ext}
\]

各项含义：

| 项 | 来源 | 工程含义 |
|----|------|----------|
| \(M(q)\ddot q\) | \(\frac{d}{dt}(\partial T/\partial\dot q)\) 中显含 \(\ddot q\) 的部分 | 惯性；任务空间 \(\Lambda=(JM^{-1}J^\top)^{-1}\) 的基础 |
| \(C(q,\dot q)\dot q\) | \(T\) 对 \(q\) 的偏导与 \(\dot q\) 耦合 | 科氏/离心；速度相关「假重力」 |
| \(G(q)\) | \(\partial V/\partial q\) | 重力（及弹性势）；可离线标定、前馈补偿 |

**\(C\) 的选取不唯一**，但常用 **Christoffel 符号**形式保证 \( \dot M - 2C \) 反对称，从而用于稳定性证明与能量整形。定义

\[
c_{jk\ell} = \frac{1}{2}\Big(
\frac{\partial m_{j\ell}}{\partial q_k}
+ \frac{\partial m_{k\ell}}{\partial q_j}
- \frac{\partial m_{jk}}{\partial q_\ell}
\Big),\qquad
\big(C(q,\dot q)\dot q\big)_j = \sum_{k,\ell} c_{jk\ell}\,\dot q_k \dot q_\ell
\]

则 \(C\) 满足 \(\dot q^\top C\dot q = 0\)（功率无贡献），与 04 篇反对称性讨论衔接。

**与 NE 的关系**：同一物理系统，NE 直接算 \(\tau\)；拉格朗日先识别 \(M,C,G\) 再代入 \(\ddot q\)。Pinocchio 的 `crba` 组装 \(M\)，`rnea` 等价于算右端已知 \(\ddot q\) 时的 \(\tau\)；两种路径数值应一致（差数值误差）。

---

## 四、与牛顿-欧拉对照表

| 维度 | 牛顿-欧拉（02 篇） | 拉格朗日（本篇） |
|------|-------------------|------------------|
| **建模入口** | 每连杆力/力矩平衡 | 标量 \(L=T-V\)，广义坐标 \(q\) |
| **计算效率** | 逆动力学 **\(O(n)\)**，适合每周期 RNEA | 显式 \(M\) 约 \(O(n^2)\)；全符号展开随 \(n\) 膨胀 |
| **解析性 / 结构** | 递推式紧凑，不直接暴露 \(M,C,G\) | 自然得到 \(M,C,G\) 及矩阵性质（对称、反对称） |
| **实现与库** | Pinocchio `rnea`/`aba` 内核 | `crba`+分解；符号推导、教学推导 |
| **适用场景** | 实时仿真、WBC 前馈、力矩查询 | 控制器设计、稳定性证明、**参数辨识**、任务空间方程推导 |
| **外力/接触** | 末端/连杆上施加 \(f,n\) 再反向递推 | \(\tau_{ext}\) 进方程右端；与 \(J^\top F\) 一致 |
| **扩展** | 浮基、闭环同样递推 | 约束可用拉格朗日乘子或降维 \(q\) |

**怎么选**：控制环里算「这一拍要多少 \(\tau\)」→ NE；要证「PD+重力补偿半全局稳定」或写 \(Y\theta\) 做辨识 → 拉格朗日。工程上二者常混用：离线/低频用拉格朗日结构，高频用 NE 算数。

---

## 五、适合参数辨识与控制器推导的原因

**参数辨识**：惯性参数（质量、质心、惯量）在 \(M,C,G\) 中**线性出现**时可合并为向量 \(\theta\)，满足

\[
Y(q,\dot q,\ddot q)\,\theta = \tau
\]

（回归矩阵 \(Y\) 与基参数定义见 05 篇）。拉格朗日推导便于系统化生成 \(Y\)，而 NE 递推式对参数是隐式耦合的。

**控制器推导**：

- **计算力矩**：\(\tau = M\ddot q_d + C\dot q + G + K_p e + K_d \dot e\)，需要 \(M,C,G\) 或等效 RNEA；
- **能量整形 / 被动性**：利用 \(\dot M - 2C\) 反对称证 \(\dot q^\top\tau\) 的耗散结构；
- **自适应控制**：以 \(Y\theta\) 做参数估计，\(\hat\theta\) 更新律与 \(C,G\) 结构绑定；
- **阻抗/操作空间**：\(\Lambda=(JM^{-1}J^\top)^{-1}\) 把关节空间 \(M\) 映到任务空间（06 篇）。

因此拉格朗日路径在「**要看见方程形状**」时更省力；NE 在「**要快**」时更省力。

---

## 小结

- 用 \(L=T-V\) 与欧拉-拉格朗日方程，得到 \(M(q)\ddot q+C(q,\dot q)\dot q+G(q)=\tau+\tau_{ext}\)。
- \(M\) 来自动能二次型，\(G\) 来自势能梯度，\(C\) 吸收速度耦合项；Christoffel 形式便于性质分析。
- 与 NE 物理等价：NE 擅 \(O(n)\) 实时算力矩；拉格朗日擅结构、辨识与控制器证明。
- 方程的反对称性、\(Y\theta\)、\(\tau_{ext}\) 处理见 [04 动力学方程性质](./04_dynamics_equation_properties.md)。

## 下一篇

→ [04 动力学方程性质：\(M,C,G\) 与 \(\tau_{ext}\)](./04_dynamics_equation_properties.md)
