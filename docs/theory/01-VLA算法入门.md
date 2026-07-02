# VLA 算法全解：从入门到上手

> **类型**：理论 · **层级**：L1 · **状态**：稳定 · **更新**：2026-07-03  
> **关联**：[02-VLA与RL进阶](./02-VLA与RL进阶.md) · [π0.5 项目实践](../projects/π0.5双臂L6折纸VLA与SAC微调项目实践.md)

> **Vision-Language-Action（VLA）** 是当前具身智能领域最热门的方向之一。本文从「是什么、为什么、怎么做」三个层次系统梳理 VLA 的核心概念、主流架构、代表模型与工程实践，适合有一定深度学习基础的读者。  
> **进阶阅读**：模型演进主线、四条技术路线、RL 后训练详见 [02-VLA与RL进阶](./02-VLA与RL进阶.md)。

---

## 目录

1. [VLA 是什么](#一vla-是什么)
2. [为什么需要 VLA](#二为什么需要-vla)
3. [典型架构](#三典型架构)
4. [动作表示：离散 vs 连续](#四动作表示离散-vs-连续)
5. [代表性模型](#五代表性模型)
6. [训练范式详解](#六训练范式详解)
7. [Action Head 工程设计](#七action-head-工程设计)
8. [推理延迟与实时控制](#八推理延迟与实时控制)
9. [数据集：Open X-Embodiment](#九数据集open-x-embodiment)
10. [国产 VLM 做 VLA 微调](#十国产-vlm-做-vla-微调)
11. [学习路径建议](#十一学习路径建议)
12. [训练框架与工具生态](#十二训练框架与工具生态)

---

## 一、VLA 是什么

VLA 模型把三类能力统一在一个系统里：

- **Vision（视觉）**：理解相机看到的场景（物体位置、状态、空间关系）
- **Language（语言）**：理解自然语言指令（如"把红色杯子放到桌上"）
- **Action（动作）**：输出机器人可执行的控制信号（关节角度、末端位姿、夹爪开合等）

**一句话概括**：让机器人"看懂场景 + 听懂指令 + 做出动作"。

```
VLM = Vision + Language          →  理解世界、回答问题
VLA = VLM + Action               →  理解世界 + 控制身体
```

---

## 二、为什么需要 VLA

传统机器人 pipeline 是分模块的：

```
感知模块 → 规划模块 → 控制模块
```

各模块独立训练，误差逐级累积，在新场景、新指令面前泛化能力极差。

VLA 的核心思路是借鉴大语言模型（LLM）的成功经验，用**端到端**或**统一表征**的方式，直接从「图像 + 文本」映射到「动作」，从而：

- 消除模块间的信息损失
- 借助大规模互联网预训练提升语义理解
- 具备对新物体、新指令更强的零样本泛化能力

---

## 三、典型架构

```
┌──────────────┐    ┌─────────────────┐
│  相机图像     │───▶│  Vision Encoder  │──┐
│  (RGB/深度)  │    │ SigLIP/DINOv2   │  │    ┌──────────────┐    ┌────────────────┐
└──────────────┘    └─────────────────┘  ├───▶│ LLM Backbone │───▶│  Action Head   │
                                         │    │ (Fusion/推理) │    │ MLP/Diffusion/ │
┌──────────────┐    ┌─────────────────┐  │    └──────────────┘    │  Flow Matching │
│  语言指令    │───▶│ Language Encoder │──┘                        └───────┬────────┘
│  自然语言    │    │ LLaMA/Gemma/Qwen│                                   │
└──────────────┘    └─────────────────┘                         ┌─────────▼──────────┐
                                                                 │  机器人动作         │
                                                                 │ [Δx,Δy,Δz,R,P,Y,g] │
                                                                 └────────────────────┘
```

**核心组件梳理与选型要点：**

1. **Vision Encoder（视觉编码器）**
  - **作用**：提取图像中的视觉特征，是感知输入的第一步。
  - **常用模型及特点**：
    - **SigLIP**（Google）：新一代多模态视觉表征，零样本泛化强，适合各类视觉任务。
    - **CLIP**（OpenAI）：图文对齐模型，通用性强、社区资源丰富，但对细节理解略弱于新方案。
    - **DINOv2**（Meta）：自监督视觉特征，迁移力强、下游微调表现好，细节表达突出。
    - **ViT**：Vision Transformer 结构，可以作为各种视觉方案的主干，算力/数据要求高但通用性好。
  - **选型建议**：SigLIP/CLIP 针对多模态任务，DINOv2 强于自监督/迁移，ViT 适合定制和基础架构搭建。
2. **Language Model（语言模型）**
  - **作用**：解析指令、推理，理解任务语义。
  - **常见选择**：LLaMA、Gemma、Qwen、InternVL 等大语言模型。
3. **Action Head（动作生成头）**
  - **作用**：将表征映射为具体机器人动作，是控制信号的输出端。
  - **主流结构及适用场景对比**：

    | 结构                   | 适用场景        | 主要特点                    |
    | -------------------- | ----------- | ----------------------- |
    | **MLP**              | 简单/单步动作     | 直接回归连续动作，快速简单，表达能力有限    |
    | **Diffusion Policy** | 精细/多步/复杂动作  | 扩散模型生成动作序列，可建模多峰分布，推理慢  |
    | **Flow Matching**    | 高频/长序列/实时任务 | 特征空间流采样，推理快，轨迹平滑，适合高频控制 |

  - **简要对比**：
    - MLP：结构简单，适合基础任务，但难以处理复杂分布和长序列。
    - Diffusion Policy：可生成多步动作，精度高但推理慢。
    - Flow Matching：兼顾精度与速度，适合对响应和轨迹平滑有要求的复杂机器人场景。
4. **训练数据**
  - **作用**：支撑模型泛化与能力迁移。
  - **来源**：人类遥操作数据、仿真轨迹、OXE 等各类机器人体验数据。

> ⚠️ 总结：选型需结合任务复杂度、算力条件及实时性要求。视觉/语言/动作各模块都有“轻量-通用-高精”三类方案可选，新手建议自上而下分别选用 SigLIP（或 DINOv2）、LLaMA、MLP 起步，高阶可逐步替换为 Diffusion/Flow Matching 等结构。

---

## 四、动作表示：离散 vs 连续

这是 VLA 里**最关键的设计选择**，直接影响精度、速度和工程复杂度。

### 4.1 离散化（Tokenization）

把连续动作离散成 token，和 LLM 词表一起训练。

```python
# 每一维动作 ∈ [-1, 1]，切成 256 份
bin_id = int((action_value + 1) / 2 * 255)
token  = f"<robot_action_{dim}_{bin_id}>"
# 7 维 → 7 个 token，自回归生成
```

**优点**：架构统一，可复用 LLM 预训练；天然支持多模态序列建模。  
**缺点**：量化误差（256 bin 对精细操作可能不够）；难表达多峰分布。  
**代表**：RT-2、OpenVLA。

### 4.2 连续输出（Diffusion / Flow Matching）

直接从 hidden states 回归或去噪，得到连续动作向量。

```python
# Flow Matching：条件生成连续动作
action = flow_matching.sample(condition=vlm_features)  # shape: [7]

# Diffusion Policy：可一次预测 action chunk
actions = diffusion_policy.sample(...)  # shape: [H×7]
```

**优点**：精度高、轨迹平滑；能建模多峰分布；action chunk 一次出多步，补偿推理延迟。  
**缺点**：训练和推理更复杂；与 LLM 训练范式不统一。  
**代表**：π0、Octo、Diffusion Policy。

> **补充：**
**Diffusion Policy 简介**：Diffusion Policy 是一种视觉-动作策略，基于扩散模型，将机器人动作序列的生成建模为从高斯噪声逐步去噪的过程。相比传统的行为克隆或直接回归，它能更好地捕捉多样性和复杂动作分布（比如多种抓取方式），输出更平滑、自然的动作轨迹，尤其实用于高精度、多步操作。诞生自哥伦比亚大学团队，代表论文和开源代码见：[https://diffusion-policy.cs.columbia.edu/](https://diffusion-policy.cs.columbia.edu/)

> **Octo 简介**：Octo 是由伯克利等机构提出的轻量级泛化机器人策略模型和训练框架，以 Diffusion Policy 为核心算法，架构参数量小（27M~93M），推理快、微调门槛低、适合快速适配新机器人或新任务。Octo 支持少量数据的迁移训练，适合资源有限的开发者开展原型实验。开源主页：[https://github.com/octo-models/octo](https://github.com/octo-models/octo)

> **π0 简介**：π0（pi-zero）是 Google DeepMind 提出的端到端 Vision-Language-Action 流模型系列的首个版本。它采用 PaliGemma 作为视觉语言骨干，加上专门的 Action Expert 层，通过 Flow Matching 连续采样输出多步动作 chunk。π0 拥有极强的细粒度操作与泛化能力，是工业级通用机器人的代表模型之一。相关论文："A Vision-Language-Action Flow Model for General Robot Control"（2024）。

### 4.3 如何选择？


| 因素              | 倾向离散 | 倾向连续                 |
| --------------- | ---- | -------------------- |
| 与 LLM 深度结合      | ✓    |                      |
| 精细操作（装配、穿线）     |      | ✓                    |
| 大规模 co-training | ✓    |                      |
| 工业精细部署          |      | Diffusion Policy 已验证 |


**2024–2025 趋势**：大 VLM 做语义理解 + 连续 head 做精细控制（π0、CogACT、GR00T N1 等），两者结合成为主流。

---

## 五、代表性模型

按**动作输出方式**，主流 VLA 可分为两大类（与第四节对应）：

```
离散动作模型：动作 → token → 自回归生成        连续动作模型：VLM 特征 → Action Head → 连续向量/chunk
代表：RT-1、RT-2、OpenVLA                      代表：π0 系列、Octo、GR00T N1、Helix
优点：与 LLM 训练统一、易 co-training            优点：精度高、轨迹平滑、适合精细操作
```


| 类别     | 动作表示                      | 代表模型             | 适合场景                      |
| ------ | ------------------------- | ---------------- | ------------------------- |
| **离散** | 256-bin token，自回归         | RT-2、OpenVLA     | 研究复现、语义泛化、大规模 co-training |
| **连续** | Flow / Diffusion，chunk 输出 | π0、Octo、GR00T N1 | 精细操作、工业部署、高频控制            |


---

### 5A 离散动作模型

#### 5A.1 RT-1（2022，Google）

**定位**：第一个大规模 Transformer 机器人策略，验证了 Transformer + 大规模 BC【Behavior Cloning, BC。 这里的大规模 BC 指的是用大量机器人演示（约13万条数据，用于监督学习）直接学习“观察（视觉+文本）→ 动作”映射——即通过端到端模仿人类遥操作，让模型直接拟合人类操作行yppo为，而不是先分模块提取特征、再做决策和控制。】 RT-1 的实验成功验证了：“用大模型 + 大数据做 BC”也可以实现机器人复杂任务的泛化和可迁移性。

```
输入：多帧 RGB 图像 + 自然语言任务描述
架构：EfficientNet-B3 → Token Learner → Transformer Encoder-Decoder → 离散动作分类（256 bin）
输出：7 维离散动作 [x, y, z, roll, pitch, yaw, gripper]
规模：~13 万条演示、700+ 任务
```

局限：纯机器人数据，无 web co-training，对新物体泛化有限。

---

#### 5A.2 RT-2（2023，Google DeepMind）

**定位**：把 VLM 和机器人控制统一为一个模型，核心创新是 **Web Co-training + 动作 Token 化**。

```
PaLI-X (55B) 或 PaLM-E (12B)  ← 预训练 VLM
          +
机器人演示数据 + Web 图文数据（VQA、caption 等）  ← Co-training
          ↓
动作也变成 token，和文本共用词表，自回归生成
```

经典案例：即使没专门训练过"把茄子放进黄色袋子"，模型靠 web 知识仍能理解"茄子"和"黄色袋子"，成功率明显高于 RT-1。

局限：模型极大（12B–55B），推理约 1–3 Hz；闭源，难复现。

---

#### 5A.3 OpenVLA（2024，Stanford 等，开源 ⭐）

**定位**：可复现、可微调的 RT-2 风格开源 VLA，研究首选。

```
架构：Prismatic-7B VLM
  ├── Vision: SigLIP + DINOv2 双塔融合
  ├── Language: Llama 2 7B
  └── 动作：256-bin 离散 token（类似 RT-2）
训练数据：Open X-Embodiment 子集（~97 万条轨迹）
HuggingFace 直接加载，支持 LoRA 微调到新机器人
```

适合原因：代码、权重、数据格式完整开源；社区活跃；改 backbone（如换 Qwen-VL）的参考实现多。

---

### 5B 连续动作模型

连续模型的共同模式：**VLM 负责"看懂+听懂"，Action Head 负责"精细出手"**。推理时通常一次输出 action chunk（未来 H 步），控制环按 50 Hz 逐步执行。

```
┌────────────┐   ┌────────────┐   ┌──────────────────┐
│ 相机图像    │──▶│ Vision Enc │──▶│                  │
│ (多视角)   │   └────────────┘   │  VLM Backbone    │
├────────────┤   ┌────────────┐   │  (语义理解)       │──▶ hidden states
│ 语言指令    │──▶│ Text Enc   │──▶│                  │         │
├────────────┤   └────────────┘   └──────────────────┘         │
│ 本体状态    │──▶ (关节角等)                                     ▼
└────────────┘                              ┌─────────────────────────┐
                                              │  Action Expert Head      │
                                              │  Flow / Diffusion        │
                                              └───────────┬─────────────┘
                                                          ▼
                                              连续动作 chunk [H × 7]
```

#### 5B.1 π0 / π0.5 / π0.6（2024–2025，Physical Intelligence）⭐

**定位**：当前 VLA 工业界最关注的连续动作路线，强调跨本体泛化与精细操作。

**π0 核心架构**（论文：*A Vision-Language-Action Flow Model for General Robot Control*）：

```
┌─────────────────────────────────────────────────────────────┐
│                    PaliGemma（预训练 VLM）                    │
│   SigLIP 视觉编码 + Gemma 语言模型，继承互联网语义知识          │
└──────────────────────────┬──────────────────────────────────┘
                           │ Cross-Attention
┌──────────────────────────▼──────────────────────────────────┐
│  机器人本体状态（关节角、夹爪等）→ 线性投影后注入 Transformer    │
└──────────────────────────┬──────────────────────────────────┘
                           │
┌──────────────────────────▼──────────────────────────────────┐
│  Action Expert（独立专家层，约与 backbone 同层数）               │
│  Flow Matching → 连续动作分布，非离散 token                     │
└──────────────────────────┬──────────────────────────────────┘
                           ▼
              Action Chunk [H 步，如 50 步] → 最高 50 Hz 控制
```

**三版演进与区别**：


|               | π0                     | π0.5                      | π0.6 / π0.6                     |
| ------------- | ---------------------- | ------------------------- | ------------------------------- |
| **发布**        | 2024.10                | 2025                      | 2025.11                         |
| **VLM 骨干**    | PaliGemma              | PaliGemma（增强 co-training） | Gemma 3 4B                      |
| **动作输出**      | Flow Matching 连续 chunk | 同上 + 可输出中间文本（子任务/推理）      | 连续 chunk + FAST 离散 token（KI 训练） |
| **核心能力**      | 跨本体 BC、精细操作（叠衣服等）      | **开放世界泛化**：未见过的家庭环境       | **从经验中学习**：RL 微调（Recap）         |
| **训练数据**      | 10k+ 小时多机器人遥操          | + 更多真实家庭场景                | + 在线 rollout + 优势信号             |
| **Attention** | 标准                     | 图像 token 双向；文本因果          | 同 π0.5，action expert 双向         |
| **开源**        | π0-base 部分开源           | 更闭源                       | 闭源（π0.6 为 RL 版）                 |


**π0 → π0.5 的关键变化**：不止输出动作，还能输出**文本中间表示**（如子任务分解、"先打开抽屉"），本体状态被**离散化成 text token** 输入，使模型在陌生环境中也能靠语义推理泛化。

**π0.5 → π0.6 的关键变化**：引入 **Knowledge Insulation（KI）** 训练——VLM 骨干同时学 FAST 离散动作 token 和 web co-training 数据；Action Expert 学连续动作，但**梯度 stop 不回传骨干**，防止破坏预训练知识。π0.6 在此基础上加入**优势指示器**（Advantage: positive/negative），用 RL（Recap）从自主经验中持续改进。

```python
# π0 推理流程（简化）
images, instruction, robot_state = get_obs()
vlm_feat = paligemma.encode(images, instruction)
vlm_feat = cross_attn(vlm_feat, project(robot_state))
action_chunk = flow_matching.sample(vlm_feat)   # [50, 7]，连续值
for a in action_chunk:
    robot.step(a)   # 50 Hz 执行，chunk 用完再推理下一批
```

与 OpenVLA 的核心差异：**不用离散 token 自回归**，而是用 Flow Matching 直接采样连续轨迹，精度更高，更适合穿线、折叠等精细任务。

> 这里的"token自回归"指的是将每个动作的连续值离散化为若干个 token（比如每一维动作离散为 256 个 bin），然后像语言模型一样逐步一个一个地预测下一个 token。模型每生成一个 token，都会根据之前生成的 token 作为输入，依次生成完整动作。这样方法的优点是和 LLM 结构兼容，但因为每一步都要预测 token，会导致推理延迟增加，而且有误差累积的问题。因此，Flow Matching 直接生成整个连续轨迹（chunk），避免了 token 自回归的局限。

---

#### 5B.2 Octo（2024，UC Berkeley 等，开源）

**定位**：轻量、通用、易微调的 generalist policy，适合快速适配新任务。

```
┌──────────┐     ┌─────────────┐     ┌──────────────────┐
│ 图像      │────▶│ 小型 ViT     │────▶│                  │
└──────────┘     └─────────────┘     │  Transformer      │
┌──────────┐     ┌─────────────┐     │  + Embodiment     │──▶ Diffusion Head
│ 语言指令  │────▶│ T5-Base     │────▶│    Token（区分     │         │
└──────────┘     └─────────────┘     │    不同机器人）    │         ▼
                                     └──────────────────┘    连续 action chunk
参数量：27M（Octo-S）~ 93M（Octo-B）
```

特点：推理快（10–30 Hz）；预训练在 OXE；新机器人只需少量数据 + block-wise 微调。相比 π0，体量小、上手快，但精细操作上限较低。

---

#### 5B.3 GR00T N1 / N1.5（2025，NVIDIA，开源）

**定位**：人形机器人通用基础模型，双系统架构，首个开源的 humanoid VLA 基础模型。

```
┌─────────────────────────────────────────────────────────┐
│  System 2（慢思考）  NVIDIA Eagle-2 VLM                  │
│  场景理解、语义推理、任务规划                              │
└────────────────────────┬────────────────────────────────┘
                         │ latent representation（联合训练）
┌────────────────────────▼────────────────────────────────┐
│  System 1（快行动）  Diffusion Transformer（DiT）          │
│  高频连续动作 chunk，L40 上 16 步仅需 63.9ms               │
└────────────────────────┬────────────────────────────────┘
                         ▼
              人形机器人全身控制信号
总参数 2.2B；训练数据：机器人轨迹 + EgoScale 2 万小时自视角视频 + Isaac Sim
```

意义：将"世界模型"（Cosmos WFM）引入 VLM 骨干，探索 VLA 与世界模型的融合。

---

#### 5B.4 Helix（2025，Figure AI）

同样采用双系统：S2（大 VLM，场景理解）→ S1（visuomotor policy，连续控制）。训练数据约 500 小时遥操演示 + 自动生成文本描述。特点：首个能高频控制人形机器人**全身上半身**（手臂、手、躯干、头、手指）的 VLA。

---

#### 5B.5 Gemini Robotics（2025，Google DeepMind）

**定位**：基于 Gemini 2.0 的闭源工业级 VLA，"Think Before Act"范式。

```
策略：单一大模型内部先生成自然语言推理，再输出动作（顺序而非并行）
数据：ALOHA 2 机器人 12 个月大规模遥操 + web 文档/图像/视频 co-training
支持：Motion Transfer，统一不同机器人平台的数据表示空间
```

与 GR00T N1 的对比：NVIDIA 将"慢推理 + 快控制"并行化（两个独立模块）；Google 则串行化（VLM 先思考，再出动作）。

---

### 5C 国内代表方向


| 模型/方向            | 机构       | 特点                                                  |
| ---------------- | -------- | --------------------------------------------------- |
| CogACT           | 清华等      | InternVL + Diffusion Action Transformer，解耦认知与控制     |
| RDT-1B           | 清华/RDT团队 | 扩散基础模型，双臂操作，ICLR 2025                               |
| TinyVLA          | 多机构      | 小参数量，强调端侧部署                                         |
| 基于 Qwen2-VL      | 多论文      | Qwen2-VL + LoRA + MLP/Diffusion head，2024–2025 主流路线 |
| AgiBot / 智元 / 宇树 | 工业界      | VLA + 人形/双臂，结合 OXE 和自建遥操数据                          |


---

### 5D 模型对比总结


| 模型               | 类别  | 动作表示                | 参数量     | 开源  | 推理速度                       | 适合场景      |
| ---------------- | --- | ------------------- | ------- | --- | -------------------------- | --------- |
| RT-1             | 离散  | 256-bin 分类          | ~35M    | 否   | -                          | 历史参考      |
| RT-2             | 离散  | 离散 token            | 12B–55B | 否   | -                          | 语义泛化      |
| OpenVLA          | 离散  | 离散 token            | 7B      | ✅   | ~5–10 Hz（常规）               | 研究/微调     |
| π0 / π0.5 / π0.6 | 连续  | Flow Matching chunk | ~3–4B   | 部分  | ~50 Hz（action chunk）       | 精细操作、开放世界 |
| Octo             | 连续  | Diffusion chunk     | 27–93M  | ✅   | >30 Hz（action chunk）       | 快速适配      |
| GR00T N1         | 连续  | DiT Flow chunk      | 2.2B    | ✅   | L40 上 16 步63.9ms（250 Hz 级） | 人形机器人     |
| Gemini Robotics  | 连续  | 连续                  | 未知      | 否   | -                          | 工业级泛化     |


---

## 六、训练范式详解

> 一句话：**先用演示数据模仿（BC）训出 base，再用自己机器人的数据微调适配。RL 是进阶选项，工业界主流仍是 BC。**

### 6.1 三种训练方式（由简到难）


| 范式              | 做什么              | 数据               | 何时用              |
| --------------- | ---------------- | ---------------- | ---------------- |
| **BC（行为克隆）**    | 监督学习：看演示，学动作     | 人类遥操轨迹           | 永远的第一步           |
| **Co-training** | web 图文 + 机器人数据混训 | VQA/caption + 遥操 | 需要语义泛化时（RT-2 路线） |
| **RL / RLHF**   | 环境中试错，用奖励优化      | rollout + 奖励/偏好  | 演示不够、需超越人类时      |


**BC 损失函数**（二选一，取决于动作表示）：

```python
# 离散 token（OpenVLA 路线）
loss = CrossEntropy(pred_action_tokens, gt_action_tokens)

# 连续动作（Octo / π0 路线）
loss = MSE(pred_action, gt_action)          # MLP head
loss = flow_matching_loss(pred, gt)         # Flow head
```


|      | Co-training            | 世界模型                 |
| ---- | ---------------------- | -------------------- |
| 训练什么 | VQA、caption + 机器人轨迹一起训 | 预测未来图像/状态            |
| 目的   | 把 web 语义迁移到机器人         | 做规划、仿真、数据增广          |
| 代表   | RT-2、OpenVLA           | GAIA-1、UniSim、V-JEPA |


### 6.2 典型训练流水线

```
① 选预训练 base（OpenVLA / Octo / 自训 VLM+Head）
        ↓
② 准备目标机器人遥操数据（RLDS 或 LeRobot 格式）
        ↓
③ LoRA 微调 LLM + 全量训 Action Head（离散或连续）
        ↓
④ 仿真/真机评测 → 不够再补数据或上 RL
```

### 6.3 完整微调示例：OpenVLA + LIBERO 仿真

下面是一个**从头到尾可跑通**的离散 VLA 微调案例，帮你把"训练范式"和"输入输出"串起来。

**场景**：在 LIBERO 仿真环境里，让机械臂完成"把杯子放到盘子上"类任务。

**① 硬件与软件**


| 项目  | 推荐配置                                        |
| --- | ------------------------------------------- |
| GPU | 单卡 A100 80GB（或 2× RTX 4090，减小 batch + 梯度累积） |
| 框架  | PyTorch + HuggingFace + PEFT（LoRA）          |
| 仿真  | LIBERO（MuJoCo）                              |
| 数据  | LIBERO-Spatial 子集（RLDS 格式）                  |
| 模型  | `openvla/openvla-7b`（预训练 base）              |


**② 数据长什么样（一条样本）**

```python
sample = {
    "image":        tensor[3, 224, 224],   # 第三人称相机 RGB
    "instruction":  "pick up the cup and place it on the plate",
    "action":       [0.02, -0.01, 0.05, 0.0, 0.0, 0.1, 1.0],  # 7维末端增量
    # 训练时 action 会被离散成 7 个 token，如 <act_32>, <act_128>, ...
}
```

**③ 训练命令（官方脚本）**

```bash
# clone: git clone https://github.com/openvla/openvla
torchrun --standalone --nnodes 1 --nproc-per-node 1 vla-scripts/finetune.py \
  --vla_path "openvla/openvla-7b" \
  --data_root_dir /path/to/rlds/datasets \
  --dataset_name libero_spatial_no_noops \
  --run_root_dir ./checkpoints \
  --lora_rank 32 \
  --batch_size 8 \
  --grad_accumulation_steps 2 \
  --learning_rate 5e-4 \
  --image_aug True
```

**④ 训练时内部发生了什么**

```
输入：image + tokenized instruction
  ↓
Vision Encoder（SigLIP + DINOv2）→ 视觉 token
  ↓
LLM（Llama 7B + LoRA）→ 自回归预测 7 个 action token
  ↓
损失：CrossEntropy(pred_tokens, gt_tokens)
  ↓
只更新 LoRA 权重 +（可选）projector；主干大部分冻结
```

**⑤ 推理部署（真机/仿真控制环）**

```python
# 伪代码：控制循环
while not done:
    image = camera.get_rgb()                          # 输入：当前帧
    instruction = "pick up the cup ..."               # 输入：任务指令
    action_tokens = model.generate(image, instruction) # 输出：7 个 token
    action = detokenize(action_tokens)                # 还原为连续 7 维
    robot.execute(action)                             # 发给机械臂
    # 控制频率 ~5-10 Hz；可配合 action chunk 提升到有效 50 Hz
```

**⑥ 关键超参参考（LIBERO 官方）**


| 参数        | 值                                    |
| --------- | ------------------------------------ |
| LoRA rank | 32                                   |
| 学习率       | 5e-4                                 |
| 训练步数      | 50k–80k（看 action token 准确率是否到 ~100%） |
| 精度        | bfloat16                             |
| 数据增强      | 随机裁剪 + 颜色抖动                          |


**⑦ 如果选连续模型（Octo 路线）差异在哪？**


|      | OpenVLA（离散）   | Octo（连续）                 |
| ---- | ------------- | ------------------------ |
| 动作输出 | 7 个 token，自回归 | 直接回归 `[H, 7]` chunk      |
| 损失   | CrossEntropy  | Diffusion denoising loss |
| 微调难度 | 官方脚本完善        | 更轻量，50–100 条轨迹即可适配新机器人   |
| 适合   | 研究复现、语义任务     | 快速原型、端侧部署                |


### 6.4 RL 何时才需要？

BC 的三大痛点：演示覆盖不全、人类操作次优、误差累积。RL 用于**超越演示上界**（如长 horizon 叠衣服）。工业操作类 VLA 目前仍以 **BC + 微调** 为主；π0.6 的 Recap 代表了"从自主经验中学习"的前沿方向，但工程门槛高。

```
BC 预训练 base → 环境中 rollout → 奖励/优势信号 → 策略微调
适用：演示难覆盖的长任务；安全对齐；π*0.6 类在线学习
```

---

## 七、Action Head 工程设计

### Step 1：定义动作空间

```python
# 末端执行器空间（最常见，7 维）
action = [Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]

# 关节空间（8 维，含夹爪）
action = [q1, q2, ..., q7, gripper]

# 推荐使用增量式（delta），更稳、更易泛化
```

### Step 2：选 Action Head 类型


| Head 类型            | 特点                      | 代表                    |
| ------------------ | ----------------------- | --------------------- |
| MLP 回归             | 简单、快，适合单步动作             | 早期工作、TinyVLA          |
| 离散 Token           | 与 LLM 统一，支持 co-training | RT-2、OpenVLA          |
| Diffusion Head     | 连续、多峰、chunk 输出          | Octo、Diffusion Policy |
| Flow Matching Head | 更快收敛、轨迹更直               | π0、CogACT、GR00T N1    |


### Step 3：单步 vs Action Chunk

```python
# 单步（简单，但推理频率要求高）
action_t = head(features)  # shape: [7]

# Chunk（主流，一次预测未来 H 步）
actions = head(features)   # shape: [H, 7]，H=10~50
```

### Step 4：训练目标

```python
# 离散 token
loss = CrossEntropy(pred_tokens, gt_tokens)

# 连续 MLP
loss = MSE(pred_action, gt_action)

# Diffusion / Flow Matching
loss = noise_prediction_loss  # 或 flow_matching_loss
```

### 完整示例：Qwen2-VL + Flow Matching VLA

```python
class QwenVLA(nn.Module):
    def __init__(self):
        self.vlm = Qwen2VLForConditionalGeneration.from_pretrained(...)
        self.vlm.requires_grad_(False)       # 冻结主干
        # 可选：向 VLM 注入 LoRA
        hidden_dim = 3584                    # Qwen2-VL-7B
        self.action_head = FlowMatchingHead(
            input_dim=hidden_dim,
            action_dim=7,
            chunk_size=16,
        )

    def forward(self, images, instruction, robot_state):
        # 1. VLM 编码图像 + 指令
        outputs = self.vlm(
            pixel_values=images,
            input_ids=tokenize(instruction),
            output_hidden_states=True,
        )
        feat = outputs.hidden_states[-1][:, -1, :]  # 取最后 token

        # 2. 可选：拼接机器人本体状态
        feat = torch.cat([feat, robot_state], dim=-1)

        # 3. Action Head 输出 chunk
        pred_actions = self.action_head(feat)   # [B, 16, 7]
        return pred_actions
```

---

## 八、推理延迟与实时控制

**简短结论**：7B 级 VLA 很难稳定 50 Hz；工程上常用 **5–15 Hz 推理 + Action Chunking**。


| 模型              | 推理速度                | 硬件       |
| --------------- | ------------------- | -------- |
| RT-2 (55B)      | ~1–3 Hz             | 云端 TPU   |
| OpenVLA (7B)    | ~3–10 Hz            | A100     |
| GR00T N1 (2.2B) | chunk 16步 ≈ 15 Hz   | L40      |
| Octo (93M)      | ~15–30 Hz           | RTX 4090 |
| π0              | ~10–20 Hz（配合 chunk） | A100     |


### 工程对策

**① Action Chunking（最常用）**

```python
# 一次预测未来 H=25~50 步，控制循环只执行缓存
actions = model.predict(image, text)  # shape: [50, 7]
for i in range(50):
    robot.execute(actions[i])  # 20ms 一次（50Hz）
# 快用完时，在另一线程异步计算下一 chunk
```

**② 异步推理**：控制线程 50 Hz 执行上一 chunk；推理线程 5–10 Hz 计算下一 chunk。

**③ 模型压缩**：INT8 量化、TensorRT 加速、蒸馏到小模型（Octo、TinyVLA 思路）。

**④ 分层控制**：

```
VLA 策略层（5 Hz）：目标位姿 / 子目标
传统控制器（500 Hz）：关节 PID / MPC 跟踪
```

> 核心逻辑：执行环 50 Hz + 策略环更低频，这是当前主流工业部署方案。

---

## 九、数据集：Open X-Embodiment

**OXE** 是 Google 牵头、21 家机构、22 种机器人形态、约 100 万+ 条轨迹的开源机器人数据集联盟。

```
主要子集（部分）：
  Bridge V2       ~60k 条   WidowX 双臂
  RT-1 Data      ~130k 条   Google 单臂
  TACO            ~38k 条   双臂操作
  Jaco Play       ~10k 条   Kinova 臂
  ...共 20+ 子集
```

- **格式**：RLDS（TensorFlow Record，OXE/OpenVLA 主流）；LeRobot 用 Parquet+MP4，两者可互转（见第十二节）
- **动作空间**：不统一，训练时通常归一化或用 embodiment-specific head
- **意义**：OpenVLA、Octo、RT-X 的预训练基础；跨机器人泛化研究标准数据源

**使用建议**：先从 **Bridge V2**（小、常用）+ **LIBERO**（仿真 benchmark）入手，再考虑 OXE 子集。

---

## 十、国产 VLM 做 VLA 微调

学术侧常见做法：**冻结或轻量微调预训练 VLM + 新增 Action Head + BC 监督学习**，和 OpenVLA/RT-2 思路一致，只是 backbone 换成国产 VLM。

### 三种接法

**方案 A：Action Head（学术首选）**

```
Qwen-VL / InternVL（backbone，可 LoRA 微调）
        ↓ 最后一层 hidden states
   Action Decoder（新增）
   ├── MLP：简单，适合单步动作
   ├── Transformer Decoder：适合 action chunk
   └── Diffusion / Flow Head：连续动作、多峰分布
        ↓
   连续动作向量 [Δx, Δy, Δz, Δroll, Δpitch, Δyaw, gripper]
```

代表工作：CogACT（InternVL + Diffusion Action Transformer）、TinyVLA、Qwen2-VL 系列论文。

**方案 B：动作 Token 化（RT-2 / OpenVLA 路线）**

```python
# 每维离散成 256 bin
action_tokens = ["<act_32>", "<act_128>", "<act_201>", ...]
# 训练：给定 image + text，预测 action_tokens
loss = CrossEntropy(predicted_tokens, ground_truth_tokens)
```

Qwen-VL 也能做，但需改 tokenizer 和扩词表，工程改动较大。

**方案 C：双阶段**

VLM 做高层语义（子任务分解、物体定位）+ 小策略网络输出低层动作，类似 π0.5 的"语义层 + 动作层"分工。

### 微调时各模块策略


| 模块             | 常见策略            | 原因      |
| -------------- | --------------- | ------- |
| Vision Encoder | 冻结或极小 LR        | 视觉表征已很强 |
| LLM 主体         | LoRA（rank 8–64） | 省显存，防遗忘 |
| Action Head    | 全量训练            | 从零学，必须训 |
| Projector（若有）  | 全量或较大 LR        | 连接视觉和语言 |


**典型超参**：1–5 条轨迹 × 50–500 epoch（数据少时），或 OXE 子集训数天。

---

## 十一、学习路径建议

### 阶段 1：概念与论文（约 1–2 周）


| 顺序  | 内容                | 重点                     |
| --- | ----------------- | ---------------------- |
| 1   | Diffusion Policy  | 连续动作、chunk             |
| 2   | RT-2 论文           | 动作 token 化、co-training |
| 3   | OpenVLA 论文 + 代码   | 开源复现主线                 |
| 4   | Octo 论文           | 轻量、快速微调                |
| 5   | π0 技术报告           | Flow Matching          |
| 6   | CogACT 或 GR00T N1 | 国产/开源最新进展              |


### 阶段 2：代码实践（约 2–4 周）

```bash
VLA/
├── papers/          # 论文笔记
├── openvla/         # git clone OpenVLA
├── lerobot/         # HuggingFace 机器人库
├── experiments/
│   ├── libero/      # 仿真 benchmark
│   └── finetune/    # 微调脚本
└── notes/
    └── action_repr.md
```

推荐动手顺序：

1. 装 LeRobot，跑通数据加载与 BC
2. 下载 OpenVLA，在 LIBERO 上评测
3. 读 action_head 实现，对照离散 token 逻辑
4. 试 Octo 微调（小、快）
5. 尝试把 backbone 换成 Qwen2-VL（改 projector + action head）

### 阶段 3：深入国产 VLM 路线

1. 读 Qwen2-VL 文档，弄清 hidden_states 怎么取
2. 参考 CogACT：InternVL + Diffusion head 实现
3. 用 LoRA 只训语言层 + 全量训 action head
4. 数据：LIBERO 仿真 或 Bridge V2 子集起步

---

## 十二、训练框架与工具生态

做 VLA 不只是训模型，还涉及**数据采集 → 格式转换 → 训练 → 仿真评测 → 真机部署**。下面按「全栈平台 / VLA 专用 / 数据与仿真」三类，梳理目前最常用的开源框架。

```
                    ┌─────────────────────────────────────┐
                    │  LeRobot（HF 全栈，PyTorch）          │
                    │  采集 · 数据集 · 训练 · 评测 · 部署    │
                    └──────────┬──────────────────────────┘
         ┌─────────────────────┼─────────────────────┐
         ▼                     ▼                     ▼
   OpenVLA / Octo        RLDS 数据格式          LIBERO / Isaac
   （VLA 模型仓库）        （OXE 标准）            （仿真 benchmark）
```

### 12.1 全栈平台：LeRobot ⭐

**[LeRobot](https://github.com/huggingface/lerobot)**（Hugging Face）是目前社区最活跃的**端到端机器人学习库**，基于 **PyTorch** 纯 Python 实现。


| 维度       | 说明                                                                                        |
| -------- | ----------------------------------------------------------------------------------------- |
| **定位**   | 从遥操采集到策略训练、仿真评测、真机部署的**一站式流水线**                                                           |
| **架构**   | 策略多为 **Transformer** 系（ACT、VQ-BeT）或 **Diffusion** 系；v0.4+ 已集成 π0、GR00T N1.5、SmolVLA 等 VLA |
| **数据格式** | **LeRobotDataset**（Parquet 存状态/动作 + MP4 存视频），与 HF Hub 直连，可和 RLDS 互转                       |
| **硬件**   | 硬件无关 API，支持 SO-100、LeKiwi、Unitree G1 等；插件式接入新机器人                                          |
| **适合谁**  | 入门实践、低成本真机、快速跑通 BC/VLA 全流程                                                                |


```bash
pip install lerobot
# 典型流程：record → train → eval
lerobot-record    # 遥操采集
lerobot-train     # 训练 ACT / Diffusion / SmolVLA 等
lerobot-eval      # LIBERO / Meta-World 评测
```

**和本文的关系**：第六节 OpenVLA 微调示例偏「模型侧」；若要从零搭流水线，LeRobot 是更省事的入口。

### 12.2 VLA 专用训练仓库


| 框架                                                | 技术栈            | 核心特点                                 | 适合场景                 |
| ------------------------------------------------- | -------------- | ------------------------------------ | -------------------- |
| **[OpenVLA](https://github.com/openvla/openvla)** | PyTorch + FSDP | 离散 token VLA；原生 **RLDS**；LoRA 微调脚本成熟 | 复现 RT-2 路线、LIBERO 刷榜 |
| **[Octo](https://github.com/octo-models/octo)**   | JAX / Flax     | 轻量 Diffusion 策略（27–93M）；OXE 预训练      | 小 GPU 快速适配新机器人       |
| **Prismatic**                                     | PyTorch        | OpenVLA 的 VLM 底座训练代码                 | 改 backbone、研究 VLM 结构 |
| **openpi**（Physical Intelligence）                 | PyTorch        | π0 系列官方/社区实现                         | 连续 Flow Matching VLA |


> 选型口诀：**要语言理解 + 开源生态完整 → OpenVLA；要轻量快速 → Octo；要全流程真机 → LeRobot；要 π0 路线 → openpi / LeRobot 内置策略。**

### 12.3 数据格式与仿真环境


| 工具                        | 类型           | 特点                                               |
| ------------------------- | ------------ | ------------------------------------------------ |
| **RLDS**                  | 数据规范         | Google 提出的机器人数据集标准（TFRecord）；OXE、OpenVLA 默认格式    |
| **LeRobotDataset**        | 数据格式         | Parquet + 视频，HF 生态友好；LeRobot 原生，可转 RLDS          |
| **LIBERO**                | 仿真 benchmark | 130+ 操作任务，VLA 评测事实标准；LeRobot / OpenVLA 均支持       |
| **robomimic**             | IL 工具包       | Berkeley 出品，经典 BC 算法集合（BC-RNN、Diffusion 等），偏仿真研究 |
| **Isaac Lab / Isaac Sim** | 仿真平台         | NVIDIA 生态，GR00T 等人形训练常用；物理仿真强、上手成本高              |


### 12.4 框架怎么选？（小结）


| 你的目标            | 推荐组合                                               |
| --------------- | -------------------------------------------------- |
| 零基础跑通「采集→训练→评测」 | **LeRobot** + LIBERO 仿真                            |
| 微调离散 VLA、发论文对标  | **OpenVLA** + RLDS 数据 + LIBERO                     |
| 24GB 显卡、快速适配新臂  | **Octo** 或 LeRobot 内置 Diffusion/ACT                |
| 工业人形、NVIDIA 栈   | **Isaac Lab** + GR00T N1（LeRobot 也支持）              |
| 已有 OXE 数据       | RLDS 直喂 OpenVLA/Octo；或转 LeRobotDataset 用 LeRobot 训 |


**关于 LeRobot 是不是 Transformer**：是的，其核心模仿学习策略（如 **ACT**、**VQ-BeT**）基于 Transformer 做 action chunking；Diffusion Policy 则用 U-Net/DiT 做去噪。整体工程栈是 **PyTorch**，与本文前面讲的 VLA 架构（VLM + Action Head）可以直接衔接——LeRobot 相当于把「数据、训练脚本、评测、硬件驱动」打包好了。

---

## 常见误区速查


| 误区                 | 正确理解                                      |
| ------------------ | ----------------------------------------- |
| BC = 遥操模仿学习        | ✅ 正确                                      |
| Co-training = 世界模型 | ❌ Co-training 是 web 图文 + 机器人联合训练，不是预测未来状态 |
| 微调 = 预训练 + 遥操数据    | ✅ 正确                                      |
| RL = 走路特技专用        | ⚠️ 更广：超越演示上界、长 horizon 任务、安全对齐            |
| VLA 必须达到 50 Hz 推理  | ❌ 大 VLA 难直接做到；靠 chunk + 异步 + 分层控制         |
| 双系统 = 两个独立模型       | ⚠️ GR00T N1 是联合训练的，S1/S2 共享梯度             |


---

## 延伸阅读

**经典论文**

- RT-1: Robotics Transformer（2022）
- RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control（2023）
- OpenVLA: An Open-Source Vision-Language-Action Model（2024）
- Diffusion Policy: Visuomotor Policy Learning via Action Diffusion（2023）
- π0: A Vision-Language-Action Flow Model for General Robot Control（2024）
- GR00T N1: An Open Foundation Model for Generalist Humanoid Robots（2025）

**工具与框架**（详见[第十二节](#十二训练框架与工具生态)）

- [LeRobot（HuggingFace）](https://github.com/huggingface/lerobot) — PyTorch 全栈：采集、训练、评测、部署
- [OpenVLA](https://github.com/openvla/openvla) — 离散 VLA 微调（RLDS + LoRA）
- [Octo](https://github.com/octo-models/octo) — 轻量 Diffusion 通用策略（JAX）
- [Open X-Embodiment](https://robotics-transformer-x.github.io/) — 大规模机器人数据集（RLDS）
- [robomimic](https://github.com/ARISE-Initiative/robomimic) — 经典模仿学习算法工具包

---

## 延伸阅读

| 文档 | 内容 |
|------|------|
| [02-VLA与RL进阶](./02-VLA与RL进阶.md) | RT-2 → π0.6 主线、四条路线、VLA+RL 后训练 |
| [π0.5 项目实践](../projects/π0.5双臂L6折纸VLA与SAC微调项目实践.md) | 折纸 VLA 微调 + SAC 真机落地 |
| [面试知识储备](../../interview/面试知识储备.md) | 工程向面试 Q&A |

---

> 作者：Alex | 具身智能算法工程师

