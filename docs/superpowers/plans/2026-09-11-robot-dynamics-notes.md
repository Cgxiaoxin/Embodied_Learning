# Robot Dynamics Notes Series Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a textbook-depth L1 dynamics note series under `docs/theory/dynamics/` (`00`–`09`) and wire it into the root README.

**Architecture:** Modular Chinese markdown notes matching existing L1 style (front-matter meta, TOC, intuition → formulas → engineering meaning, end-of-doc summary + next link). Kinematics stays thin and delegates to `03`; force/impedance and contact connect forward to VTLA-relevant sections in `07`–`09`.

**Tech Stack:** Markdown + GitHub-flavored math (`\( \)` / `\[ \]`); no new SVG; relative links only.

**Spec:** `docs/superpowers/specs/2026-09-11-robot-dynamics-notes-design.md`

## Global Constraints

- Location: `docs/theory/dynamics/` only for note bodies
- Depth: textbook B, ~3–5k Chinese characters per article (`01` may be shorter)
- Language: Chinese; front matter pattern: `类型 · 层级 · 状态 · 更新 · 关联`
- Status on ship: `草稿` or `稳定` — use `草稿` for first pass, date `2026-09-11`
- Do not edit `docs/theory/03-机器人控制与坐标系基础.md` body
- Do not create interview Q&A or new SVG assets
- VTLA/视触觉: short bridging paragraphs only in `00`, `07`, `08`, `09`
- Commits: one commit per task below (user already OK with design commits; keep note commits focused)

### Shared front-matter template

Every dynamics note must start with:

```markdown
# <中文标题>

> **类型**：理论 · **层级**：L1 · **状态**：草稿 · **更新**：2026-09-11  
> **关联**：[00 索引](./00_index.md) · <prev> · <next> · <external if any>

> <1–3 sentence导读>

---

## 目录

1. [...](#...)
...
```

### Shared closing template

```markdown
---

## 小结

- <3–5 bullets>

## 下一篇

→ [<title>](./XX_....md)
```

### Verification helper (all tasks)

After writing a file, run from repo root:

```bash
test -f docs/theory/dynamics/<file>.md && \
  rg -n "类型|层级|小结|下一篇" docs/theory/dynamics/<file>.md && \
  wc -m docs/theory/dynamics/<file>.md
```

Expected: file exists; meta/小结/下一篇 present; `01` may be ~2–4k chars, others ~3–8k chars (markdown+math counting).

---

## File map

| File | Responsibility |
|------|----------------|
| `docs/theory/dynamics/00_index.md` | Series map, reading paths, cross-repo links |
| `docs/theory/dynamics/01_kinematics_review.md` | Thin kinematics bridge to `03` + dynamics interfaces |
| `docs/theory/dynamics/02_newton_euler.md` | Recursive Newton–Euler |
| `docs/theory/dynamics/03_lagrangian.md` | Lagrangian energy method |
| `docs/theory/dynamics/04_dynamics_equation_properties.md` | \(M,C,G\), skew-symmetry, \(Y\theta\), \(\tau_{ext}\) |
| `docs/theory/dynamics/05_parameter_identification.md` | Inertial params, excitation, base params, LS |
| `docs/theory/dynamics/06_task_space_dynamics.md` | Op-space dynamics / \(\Lambda(x)\) |
| `docs/theory/dynamics/07_dynamics_based_control.md` | CTC, hybrid, impedance/admittance, adaptive/robust |
| `docs/theory/dynamics/08_contact_dynamics.md` | Contact models, complementarity, force closure |
| `docs/theory/dynamics/09_sim_tools_and_learning.md` | Simulators + learning / WBC / tactile force |
| `README.md` | Index + reading path |

---

### Task 1: Scaffold directory + `00_index.md` (skeleton)

**Files:**
- Create: `docs/theory/dynamics/00_index.md`

**Interfaces:**
- Produces: stable filenames `01`–`09` listed in index (bodies may still be empty until later tasks fill them; index links must use final names)
- Consumes: spec reading paths (速览 / 力控线 / VTLA线)

- [ ] **Step 1: Create directory and write `00_index.md`**

Include sections:
1. 为什么要补动力学（运控→VLA/视触觉）
2. 模块地图（表格链接 `01`–`09`）
3. 三条阅读路径：速览（00→04→07→08）；力控线（01→06→07→08）；VTLA线（00→07→08→09）
4. 仓库交叉链接：`../03-机器人控制与坐标系基础.md`、`../01-VLA算法入门.md`、`../02-VLA与RL进阶.md`、`../../projects/π0.5双臂L6折纸VLA与SAC微调项目实践.md`、`../../projects/机械臂SDK与VLA项目面试梳理.md`
5. 与视触觉 / VTLA 的关系（短段）
6. 小结 + 下一篇 → `01_kinematics_review.md`

Note: If `01`–`09` are not written yet, still link them; subsequent tasks create targets.

- [ ] **Step 2: Verify**

```bash
mkdir -p docs/theory/dynamics
test -f docs/theory/dynamics/00_index.md
rg -n "速览|力控|VTLA|03-机器人控制" docs/theory/dynamics/00_index.md
```

Expected: matches found for all three paths and link to `03`.

- [ ] **Step 3: Commit**

```bash
git add docs/theory/dynamics/00_index.md
git commit -m "$(cat <<'EOF'
Add dynamics notes index skeleton.

EOF
)"
```

---

### Task 2: `01_kinematics_review.md`

**Files:**
- Create: `docs/theory/dynamics/01_kinematics_review.md`

**Interfaces:**
- Consumes: link target `../03-机器人控制与坐标系基础.md`
- Produces: statements of \(\dot x = J(q)\dot q\) and \(\tau = J^\top F\) for later notes

- [ ] **Step 1: Write article (keep shorter than others)**

Required sections:
1. 本篇定位（详文在 `03`，此处只 bridging）
2. 位姿：旋转矩阵 / 四元数 / 齐次变换（各 3–6 句 + 何时用）
3. 正运动学与 DH（标准 vs 改进一句话对比）
4. 逆运动学：解析 vs 数值
5. 微分运动学：几何雅可比 vs 解析雅可比；奇异位形直觉
6. 进入动力学的两个接口公式 + 工程含义（速度映射、静力映射）
7. 小结 + 下一篇 → `02_newton_euler.md`

Must contain explicit markdown link to `../03-机器人控制与坐标系基础.md` at least twice (开篇 + 小结).

- [ ] **Step 2: Verify**

```bash
rg -n "03-机器人控制|\\\\dot x|J\^\\\\top|小结" docs/theory/dynamics/01_kinematics_review.md
wc -m docs/theory/dynamics/01_kinematics_review.md
```

Expected: links and formulas present; not longer than ~6k chars preferred.

- [ ] **Step 3: Commit**

```bash
git add docs/theory/dynamics/01_kinematics_review.md
git commit -m "$(cat <<'EOF'
Add kinematics review bridge for dynamics series.

EOF
)"
```

---

### Task 3: `02_newton_euler.md` + `03_lagrangian.md`

**Files:**
- Create: `docs/theory/dynamics/02_newton_euler.md`
- Create: `docs/theory/dynamics/03_lagrangian.md`

**Interfaces:**
- Produces: contrast table content reused by both (NE vs Lagrangian)
- Consumes: joint coordinates \(q,\dot q,\ddot q\) from Task 2 narrative

- [ ] **Step 1: Write `02_newton_euler.md`**

Sections:
1. 方法直觉（力/力矩递推）
2. 前向递推：角速度、角加速度、线加速度
3. 反向递推：力、力矩到关节 \(\tau\)
4. 复杂度 \(O(n)\) 与实时控制意义
5. 工程：Pinocchio / 底层 WBC 里常见
6. 与拉格朗日对照预告
7. 小结 → `03_lagrangian.md`

- [ ] **Step 2: Write `03_lagrangian.md`**

Sections:
1. 能量法直觉：\(L=T-V\)
2. 欧拉-拉格朗日方程
3. 如何整理成 \(M(q)\ddot q+C(q,\dot q)\dot q+G(q)=\tau\)
4. 与牛顿-欧拉对照表（效率 / 解析性 / 适用场景）
5. 适合参数辨识与控制器推导的原因
6. 小结 → `04_dynamics_equation_properties.md`

- [ ] **Step 3: Verify both**

```bash
rg -n "前向|反向|O\(n\)" docs/theory/dynamics/02_newton_euler.md
rg -n "L = T - V|M\(q\)|对照" docs/theory/dynamics/03_lagrangian.md
wc -m docs/theory/dynamics/02_newton_euler.md docs/theory/dynamics/03_lagrangian.md
```

- [ ] **Step 4: Commit**

```bash
git add docs/theory/dynamics/02_newton_euler.md docs/theory/dynamics/03_lagrangian.md
git commit -m "$(cat <<'EOF'
Add Newton-Euler and Lagrangian dynamics notes.

EOF
)"
```

---

### Task 4: `04_dynamics_equation_properties.md`

**Files:**
- Create: `docs/theory/dynamics/04_dynamics_equation_properties.md`

**Interfaces:**
- Consumes: standard form from Task 3
- Produces: \(\dot M-2C\) skew-symmetry statement; \(Y(q,\dot q,\ddot q)\theta\); \(\tau_{ext}\) narrative for contact notes

- [ ] **Step 1: Write article**

Sections:
1. 标准形式回顾：\(M\ddot q+C\dot q+G=\tau+\tau_{ext}\)
2. \(M(q)\)：对称正定、物理含义
3. \(C(q,\dot q)\)：科氏/离心；\(\dot M-2C\) 反对称性与自适应控制稳定性用途
4. \(G(q)\)：重力补偿
5. 参数线性化：\(Y\theta\) 形式与可辨识性预告
6. \(\tau_{ext}\)：接触/外力映射，点到触觉与 VTLA 的重要性（短）
7. 小结 → `05_parameter_identification.md`

- [ ] **Step 2: Verify**

```bash
rg -n "反对称|Y\\\\(|\\\\tau_\{ext\}|对称正定" docs/theory/dynamics/04_dynamics_equation_properties.md
```

- [ ] **Step 3: Commit**

```bash
git add docs/theory/dynamics/04_dynamics_equation_properties.md
git commit -m "$(cat <<'EOF'
Add dynamics equation properties note.

EOF
)"
```

---

### Task 5: `05_parameter_identification.md` + `06_task_space_dynamics.md`

**Files:**
- Create: `docs/theory/dynamics/05_parameter_identification.md`
- Create: `docs/theory/dynamics/06_task_space_dynamics.md`

**Interfaces:**
- Consumes: \(Y\theta\) from Task 4; Jacobian interfaces from Task 2
- Produces: \(\Lambda(x)\) definition for Task 6→7 handoff

- [ ] **Step 1: Write `05_parameter_identification.md`**

Sections:
1. 每连杆 10 个惯性参数
2. 为什么需要激励轨迹
3. 最小二乘辨识流程
4. 基参数（不可观/耦合）
5. 工程注意：摩擦、关节柔性、传感器噪声
6. 小结 → `06_task_space_dynamics.md`

- [ ] **Step 2: Write `06_task_space_dynamics.md`**

Sections:
1. 为什么要任务空间动力学（力控/阻抗）
2. 关节空间 ↔ 任务空间映射（via \(J\)）
3. 操作空间惯量 \(\Lambda(x)=(J M^{-1} J^\top)^{-1}\)（写出形式与直觉）
4. 与末端力/加速度关系
5. 为第 07 章铺垫
6. 小结 → `07_dynamics_based_control.md`

- [ ] **Step 3: Verify**

```bash
rg -n "基参数|激励|10" docs/theory/dynamics/05_parameter_identification.md
rg -n "\\\\Lambda|操作空间|J" docs/theory/dynamics/06_task_space_dynamics.md
```

- [ ] **Step 4: Commit**

```bash
git add docs/theory/dynamics/05_parameter_identification.md docs/theory/dynamics/06_task_space_dynamics.md
git commit -m "$(cat <<'EOF'
Add parameter ID and task-space dynamics notes.

EOF
)"
```

---

### Task 6: `07_dynamics_based_control.md`

**Files:**
- Create: `docs/theory/dynamics/07_dynamics_based_control.md`

**Interfaces:**
- Consumes: \(M,C,G\), \(\Lambda\) from earlier tasks
- Produces: impedance/admittance framing consumed by Task 7 (contact)

- [ ] **Step 1: Write article**

Sections:
1. 基于模型控制总览
2. 计算力矩控制（反馈线性化 + PD）
3. 力/位混合控制（选择矩阵直觉）
4. 阻抗控制 vs 导纳控制（虚拟弹簧阻尼；与触觉/柔顺任务关系 — **VTLA bridge 段**）
5. 自适应控制（接 \(Y\theta\)）
6. 鲁棒 / 滑模（未建模动态）
7. 与 `03` 控制链路的衔接（相对链接）
8. 小结 → `08_contact_dynamics.md`

- [ ] **Step 2: Verify**

```bash
rg -n "阻抗|导纳|计算力矩|混合|视触觉|VTLA|03-机器人控制" docs/theory/dynamics/07_dynamics_based_control.md
```

- [ ] **Step 3: Commit**

```bash
git add docs/theory/dynamics/07_dynamics_based_control.md
git commit -m "$(cat <<'EOF'
Add dynamics-based control note (CTC/impedance).

EOF
)"
```

---

### Task 7: `08_contact_dynamics.md`

**Files:**
- Create: `docs/theory/dynamics/08_contact_dynamics.md`

**Interfaces:**
- Consumes: \(\tau_{ext}\), impedance narrative
- Produces: complementarity / friction vocabulary for Task 8 simulators

- [ ] **Step 1: Write article**

Sections:
1. 为什么接触动力学对 VTLA/视触觉关键（**bridge 段**）
2. 接触模型：点接触、软接触
3. 摩擦：库仑、LuGre（各讲清适用）
4. 互补性约束与仿真（MuJoCo/Pinocchio/Drake 背后直觉，细节留给 09）
5. 力闭合与抓取稳定性
6. 从接触力到关节 \(\tau_{ext}=J^\top F\)
7. 小结 → `09_sim_tools_and_learning.md`

- [ ] **Step 2: Verify**

```bash
rg -n "LuGre|互补|力闭合|视触觉|VTLA|\\\\tau_\{ext\}" docs/theory/dynamics/08_contact_dynamics.md
```

- [ ] **Step 3: Commit**

```bash
git add docs/theory/dynamics/08_contact_dynamics.md
git commit -m "$(cat <<'EOF'
Add contact dynamics note for VTLA context.

EOF
)"
```

---

### Task 8: `09_sim_tools_and_learning.md`

**Files:**
- Create: `docs/theory/dynamics/09_sim_tools_and_learning.md`

**Interfaces:**
- Consumes: contact/complementarity terms from Task 7
- Produces: series endpoint; links back to VLA docs and projects

- [ ] **Step 1: Write article**

Sections:
1. 工具定位表：MuJoCo / Pinocchio / Drake / RBDL
2. Sim2Real：动力学参数域随机化
3. 基于模型的 RL / 可微物理 / 结构先验（\(M,C,G\) 嵌入）
4. WBC：动力学约束进 QP
5. 触觉信号反推/辅助估计接触力（**VTLA bridge 段**）
6. 回到 VLA 学习路径：链 `../01`、`../02`、项目实践
7. 小结（系列收束；下一篇可指回 `00_index.md` 或 `../02-VLA与RL进阶.md`）

- [ ] **Step 2: Verify**

```bash
rg -n "MuJoCo|Pinocchio|Drake|域随机化|WBC|触觉|01-VLA|02-VLA" docs/theory/dynamics/09_sim_tools_and_learning.md
```

- [ ] **Step 3: Commit**

```bash
git add docs/theory/dynamics/09_sim_tools_and_learning.md
git commit -m "$(cat <<'EOF'
Add sim tools and learning cross-over note.

EOF
)"
```

---

### Task 9: Backfill `00_index.md` + update `README.md`

**Files:**
- Modify: `docs/theory/dynamics/00_index.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: all `01`–`09` now exist

- [ ] **Step 1: Polish `00_index.md`**

Ensure every module row links to a real file; add one-line status; fix any forward-reference wording that assumed unfinished pages.

- [ ] **Step 2: Update `README.md`**

1. In 推荐阅读路径, add a line such as:

```
03 控制与坐标系 → dynamics/00 动力学系列 → 01/02 VLA
```

2. In L1 理论 table:
   - Add row for `[03-机器人控制与坐标系基础](./docs/theory/03-机器人控制与坐标系基础.md)` if still missing (needed as「控制基础」入口)
   - Add row for `[动力学系列索引](./docs/theory/dynamics/00_index.md)` with short description

3. In 目录结构 tree, show `theory/dynamics/`

- [ ] **Step 3: Link sanity check**

```bash
# all series files exist
ls docs/theory/dynamics/*.md | wc -l
# expect 10

# README points to index
rg -n "dynamics/00_index" README.md

# relative targets from dynamics exist
test -f docs/theory/03-机器人控制与坐标系基础.md
test -f docs/theory/01-VLA算法入门.md
test -f docs/theory/02-VLA与RL进阶.md
```

Expected: 10 files; README match; external targets exist.

- [ ] **Step 4: Commit**

```bash
git add docs/theory/dynamics/00_index.md README.md
git commit -m "$(cat <<'EOF'
Wire dynamics series into README and finish index.

EOF
)"
```

---

### Task 10: Final acceptance pass

**Files:**
- Modify only if broken links / missing 小结 found

- [ ] **Step 1: Structural grep across series**

```bash
for f in docs/theory/dynamics/*.md; do
  echo "=== $f ==="
  rg -n "^\> \*\*类型\*\*|## 小结|## 下一篇" "$f" || echo "MISSING META/CLOSING"
done
```

Expected: every file has 类型 meta, 小结, 下一篇 (09 may point back to index or VLA).

- [ ] **Step 2: Spec coverage checklist**

Confirm against spec sections 3–5:
- [ ] All 10 filenames present
- [ ] Depth/style constraints respected
- [ ] VTLA bridges in 00/07/08/09
- [ ] README updated
- [ ] `03` body untouched: `git diff -- docs/theory/03-机器人控制与坐标系基础.md` empty

- [ ] **Step 3: Commit fixes only if needed**

```bash
git add docs/theory/dynamics/
git commit -m "$(cat <<'EOF'
Fix dynamics notes cross-links and closings.

EOF
)"
```

(Skip commit if working tree clean.)

---

## Spec coverage (self-review)

| Spec requirement | Task |
|------------------|------|
| `docs/theory/dynamics/` + `00`–`09` | Tasks 1–8 |
| Textbook depth B, Chinese L1 style | Global Constraints + each write step |
| `01` thin → `03` | Task 2 |
| Content boundaries table | Tasks 2–8 section lists |
| Cross-links internal/external | Tasks 1, 6–9 |
| VTLA bridges in 00/07/08/09 | Tasks 1, 6, 7, 8 |
| README path + L1 index | Task 9 |
| No SVG / no `03` body edit / no interview Q&A | Global Constraints + Task 10 |
| Writing order 00→…→README | Task order 1→9 |

**Placeholder scan:** none intentional.  
**Type/name consistency:** filenames locked to spec; standard equation notation \(M,C,G,\tau_{ext},\Lambda,Y\theta\) reused across tasks.
