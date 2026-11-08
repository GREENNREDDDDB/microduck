# Microduck 摔倒保护项目 · 实施方案与开发流程

> 本文与 [microduck摔倒保护分析报告.md](./microduck摔倒保护分析报告.md) 分工：**分析报告回答“问题是什么、为什么要做”**（形态/质量分布/研究综述/算法层受身研究），**本文回答“怎么做、怎么开源”**。两份文档冲突时，本文以仓库源码为准。
>
> 目标读者：本项目作者本人。前提：**当前没有硬件实体，全部工作在仿真中完成**。交付形态：一个独立于 `pollen-robotics/microduck` 与 `pollen-robotics/microduck_rl`、可被社区“即插即用”的开源仓库。

---

## 0. 结论摘要

1. **你要做的项目，本质上是“再训练一个更抗摔的 gait 策略 + 一套损伤度量与场景夹具”**，形态与 `microduck_rl` 完全一致：它是训练侧仓库，通过 **ONNX 策略** 接入机器人，通过 **entry point** 接入训练框架。你应当照抄这套接缝，而不是改 `microduck` 主仓库。
2. **仿真是当前唯一可用的闭环**：`microduck_rl` 的 `duck-body`（MuJoCo 身体）+ 主仓库的 `robotd --sim` 让**真机代码控制仿真身体**，`scripts/duck-sim` 一条命令起全套。你在仿真里调的保护逻辑，就是机器人上跑的逻辑。
3. **“即插即用”有两条已存在的官方通道**（无需 daemon 发版）：
   - **训练接缝**：`[project.entry-points."mjlab.tasks"]` —— 你的仓库注册自己的任务，`uv run train Mjlab-YourTask-...` 即可用，和 `microduck_rl` 对 `mjlab` 做的事一样。
   - **部署接缝**：`robotctl policy add/load` + Hugging Face 策略集 —— 机器人一条命令装你的 ONNX，`robotd` 校验 `[1,61]→[1,14]` 后加载。
4. **一个硬边界必须先讲清**：Microduck **没有手臂**、`head_roll` 只有 **±25°**、舵机是 288:1 塑料齿轮的 XL330。所以“跑酷式滚翻受身”在 Microduck 上只能是**微型受身**——收颌、腿先接地、躯干/背先接触、把头壳留到最后。这一点决定了奖励设计与可达成指标（详见分析报告 §3.5）。
5. **不建议一开始改 `robotd`**。daemon 的摔倒反射链路（`FallPredictor → LimpFall`）已经存在且被 `busy()` 门控；把它改成“触发 ukemi 技能”会破坏即插即用。先用**替换 gait 槽位**的路线把保护能力做进策略里，等指标站得住，再单独提案 daemon 侧改动。

---

# 第一部分 · Microduck 工程详解（含 microduck_rl）

## 1.1 主仓库 `microduck` 是什么

一台约 **25 cm 高、800 g** 的开源双足机器人的**整机工程**：硬件、板载运行时、守护进程、客户端工具、安装升级、文档。技术栈是 **Rust**（守护进程 + 控制）+ **Rockchip RK3566** 主控 + **Dynamixel XL330** 舵机（14 个）。学习入口是 [CONTRIBUTING.md](./CONTRIBUTING.md) 与 [docs/README.md](./docs/README.md)。

工作区（cargo workspace）里与本项目相关的 crate：

| crate | 职责 | 对摔倒保护的意义 |
|---|---|---|
| `duck-ipc-proto` | 协议 + 观测/动作结构 + `JOINT_NAMES` | **契约层**：关节顺序、观测维度在这里定 |
| `duck-control` | 控制核心：`RobotIo`、`Policy`、`FallPredictor`、运动学 | `RobotIo` 之上就是真机代码；`--sim` 只替换它 |
| `robotd` | 守护进程：50 Hz 环 + 技能调度 + 安全 | **摔倒反射的所在地** |
| `robotd-params` | 参数模型（`PolicyParams`/`SkillDef`） | **技能即配置**，你加能力的入口 |
| `robotctl` | 命令行客户端 | `policy add/load`、`robot do` 从这里发 |
| `tof` / `mediad` / `btd` / `padd` / `configd` / `updaterd` | 传感器/媒体/蓝牙/手柄/身份/升级 | 保护项目一般不碰 |

## 1.2 关节契约（`duck-ipc-proto::JOINT_NAMES`）

对外 **15 个名字**、模型内 **14 个舵机**，差异是 `mouth`：

```
左腿5:  0 hip_yaw 1 hip_roll 2 hip_pitch 3 knee 4 ankle
头颈4:  5 neck_pitch 6 head_pitch 7 head_yaw 8 head_roll
嘴:     9 mouth              ← 协议层有，模型层无（sim 层插入/删除）
右腿5:  10 hip_yaw 11 hip_roll 12 hip_pitch 13 knee 14 ankle
```

**关键含义**：第 15 个关节是**嘴**，不是躯干 yaw —— **Microduck 没有手臂，也没有躯干 yaw**。所有依赖“用手撑地”“躯干扭转”的受身手段都不可用。详见分析报告 §1.1。头部行程（`robot_walk.xml`）：

| 关节 | 行程 | 受身用途 |
|---|---|---|
| `neck_pitch` | −90° ~ +60° | 收颌（把头顶放到地面）主力 |
| `head_pitch` | ±90° | 配合收颌/抬头 |
| `head_yaw` | ±170° | 转开喙与相机面 |
| `head_roll` | **仅 ±25°** | 侧向保护能力很弱 |

## 1.3 摔倒链路（逐层，改造落点）

```
IMU/状态 → FallPredictor(duck-control/src/fall.rs)  投影重力导数 ġz = −(ωx·gy − ωy·gx)
           阈: tilt_z=-0.90, lookahead=300ms, debounce=60ms, arm=200ms
         → robotd 的 LimpFall 状态机 (robotd/src/main.rs)  Idle → Limp → Posing
           gain_limp = 50（主动“放软”让机器人瘫倒）
         → 回正 → 交回站立策略
```

**一个必须记住的门**：任何 one-shot skill 运行时 `busy()` 为真，会**关闭摔倒反射**（`robotd/src/control.rs`）。这条规则同时意味着：**“自动触发的 ukemi 技能”需要改 daemon**，因为技能只能由显式请求触发。

## 1.4 安全与调度

- `robotd` 是**唯一**能命令电机的进程；`Safety` 独占唯一 `RobotIo` 写句柄，关节有夹紧、有意图 deadman（500 ms 无指令即停）。
- 技能调度优先级（`robotd/src/control.rs`）：`roulade > kick > ground pick > sit/rise > stand-by-magnitude > walk`。
- **技能是配置，不是代码**：`robotd-params::SkillDef` 让机器人“多一个技能 = 多一条配置”（源码注释明确举了社区策略 `polite-bow` 的例子）。字段：`name`、`path`（.onnx）、`duration`、`twist`、`unwind_s`、`chain` 等。

## 1.5 观察/动作契约（跨所有策略共享）

- **actor obs = 61 维**：48 本体感知 + 13D 命令块 `[twist(3), head_pose(4), body_pose(6)]`。不用某槽位的任务**零填充**而不是删列。
- **action = 14 维**（14 个舵机）。`publish` 在**上传前**校验图必须是 `[1,61] → [1,14]`，否则拒绝（历史 51 维策略会被明确拒绝）。
- 正是这个共享契约，让运行时可以**热插拔**不同策略（walk/recover/trick）。

## 1.6 `microduck_rl` 是什么

**训练侧仓库**，与主仓库解耦，基于 **mjlab（MuJoCo Warp）+ PPO（rsl_rl）**：

| 维度 | 事实 |
|---|---|
| 包名 / 入口 | `mjlab-microduck`；`[project.entry-points."mjlab.tasks"] mjlab_microduck = "mjlab_microduck.tasks"` |
| 控制频率 | 50 Hz |
| Python | `>=3.12,<3.13`（被 bam 钉死） |
| 关键依赖 | `mjlab==1.3.0`、`warp-lang`、`better-actuator-models`(BAM)、`onnxruntime`、`torch==2.9.1` |
| 控制台脚本 | `train`、`duck-body`（仿真身体服务器）、`publish`、`share-training` |
| 执行器模型 | **BAM M6 XL330**（电压控制律、反电动势、库仑/Stribeck/负载相关摩擦）+ DR |
| 仿真保真 | **backlash 变体**（每个舵机 +1° 齿轮间隙，编码器穿透 backlash 读数） |
| 任务数 | 14 个基础任务 + 各自 Backlash 变体 |

### 1.6.1 目录结构

```
src/mjlab_microduck/
├── robot/microduck/            # MJCF（robot_walk.xml / robot_groundcontact.xml / *_backlash.xml / scene_*.xml）
├── robot/microduck_constants.py# 机器人 cfg、HOME 帧、BAM 执行器 cfg
├── actuator/friction_dr_bam.py # BAM + 摩擦 DR + backlash 编码器反馈
├── tasks/
│   ├── __init__.py             # register_mjlab_task(...) 全部任务注册在这
│   ├── mdp.py                  # 奖励/事件/观测/自定义类（~7000+ 行，摔倒是它的主战场）
│   ├── backlash.py             # make_backlash_variant() 包装器
│   └── microduck_*_env_cfg.py  # 每个任务族一个 cfg 模块
├── sim/body_server.py          # `duck-body`：把 MuJoCo 身体通过 TCP 供给真 robotd
└── publish/                    # `publish`：导出+校验+上传 HF Hub
```

### 1.6.2 与本项目直接相关的既有资产

**（a）仿真闭环**：`sim/body_server.py` 的模块 docstring 说得很清楚——

```
uv run duck-body                 # MuJoCo 里的一只鸭，带窗口
robotd --sim 127.0.0.1:7801      # daemon 侧，用 RemoteIo 替换舵机总线
```
> "Everything above `duck_control::io::RobotIo` is the code that runs on a real robot."

协议：TCP NDJSON，`protocol` 在握手里校验，`TIMESTEP=0.005`。这一层负责 15↔14 关节的 `mouth` 插入/删除。

**（b）VelStand 策略**（`docs/velstand_policy.md`）：**已经**是“行走 + 保护性摔倒 + 起身”一体的策略，当前 v8 = wandb `0045i2si` @ `model_2749.pt`（tag v8，2026-10-12，真机验证）。**这是你的起点基线，不是你要从零做的事。**

**（c）Roulade 任务**（`tasks/microduck_roulade_env_cfg.py`）：前滚翻过头顶的**完整动态翻滚奖励范式**，且证明了收颌参数：

```python
TUCK_OVERRIDES = {..., 5: -1.0,   # neck_pitch 收颌
                       6:  1.0, ...}  # head_pitch 收颌
# 注释：neck_pitch −1 / head_pitch +1 把平头顶方正地放到地面
#       实测 axis_z −0.99，对比被动的脸着地 +0.6
```

**（d）损伤代理函数**（`tasks/mdp.py`）：已有现成的摔倒/舵机保护项，可直接复用：

| 函数 | 作用 | 关键阈值 |
|---|---|---|
| `fallen_state_penalty` | 倒地惩罚（门控 tilt>40°） | `gate_tilt_above_deg=40` |
| `recovery_success` | 成功起身奖励 | `up_tilt_deg=25`, `up_z=0.105` |
| `servo_stall_penalty` | 堵转计数惩罚 | `torque>0.4 N·m`, `vel>0.5 rad/s` |
| `servo_acc_spike_penalty` | **关节加速度尖峰**惩罚 | `acc>300 rad/s²` |

> `mdp.py` 注释原文点明损坏机理：**(1) 冲击反驱动 → 接触瞬间的关节加速度是损伤代理；(2) 堵转；(3) 外壳撞地**；并记录“CPU 基线推倒，各策略下舵机关节峰值 **450–550 rad/s²**”。

**（e）`AGENTS.md`（开发规则）**：新环境六步工作流、奖励设计规则（符号约定、无 jackpot、势函数整形、两类正则项）、“训练前先在 sim 验证物理假设”“5 迭代 64 envs 冒烟测试能抓 ~95% 配置错误”。**这是你写自己仓库 AGENTS.md 的模板。**

## 1.7 两个仓库的接缝（一张图）

```
┌─ 训练侧（可替换/可扩展）─────────────┐   ┌─ 部署侧（机器人/仿真真机代码）──────┐
│ microduck_rl  ──entry point──▶ mjlab │   │ microduck 主仓库                    │
│   └ duck-body ───────TCP NDJSON─────▶│──▶│   robotd --sim / 真机               │
│   └ ONNX 策略 ───HF 策略集──────────▶│──▶│   robotctl policy add/load          │
└──────────────────────────────────────┘   └─────────────────────────────────────┘
        ▲                                              ▲
        │ 你的项目从这里接：                             │ 你的项目从这里接：
        │  依赖 mjlab-microduck（拿模型/BAM/mdp 函数）   │  产出 ONNX → HF → robotctl
        └──────────────────────────────────────────────┘
```

## 1.8 对本项目最重要的 6 条事实

1. **无手臂、无躯干 yaw、`head_roll` ±25°** → 受身手段被强约束。
2. **头部链 ≈ 0.2401 kg / 总质量 ≈ 0.7444 kg ≈ 32.3%** → 真正的“头重脚轻”。
3. **`robotd` 已有摔倒反射**（`FallPredictor → LimpFall`, `gain_limp=50`），且**技能运行会关掉它**。
4. **`microduck_rl` 已有保护性摔倒基线**（VelStand v8）与**头顶着地的翻滚范例**（Roulade 收颌）。
5. **部署是即插即用的**：`robotctl policy add <名> <你的HF仓库>`。
6. **仿真是真机代码**：`duck-body` + `robotd --sim`，你在 sim 里调的就是机器人上的逻辑。

---

# 第二部分 · 具体可行的实现路径

## 2.0 总体思路

你的仓库 = **microduck_rl 的“兄弟仓库”**，做三件事：

1. **复用**：从 `mjlab-microduck` 拿机器人模型、BAM 执行器、`mdp.py` 的损伤代理函数、`duck-body`、`publish`。
2. **新增**：自己的任务（env cfg + 奖励）、自己的**损伤度量与摔倒场景夹具**、自己的 `ukemi`/保护性 gait 策略。
3. **产出**：一个 ONNX（烤入 obs 归一化），发布为自己的 HF 策略集，机器人 `robotctl policy load walk <你的仓库>` 即可用。

**两条部署路线（先做 A，B 留作提案）**：

| | 路线 A：替换 gait 槽位（推荐先做） | 路线 B：自动触发 ukemi 技能 |
|---|---|---|
| 做法 | 训一个含保护性落地的 `walk`/`stand` 策略，装进 `walk` 槽 | daemon 在 `FallPredictor` 命中时触发 `ukemi` 技能 |
| 是否改 daemon | **否** | **是**（需上游提案） |
| 即插即用 | 完全 | 需机器人更新 daemon 版本 |
| 风险 | 与 daemon 的 LimpFall 需协同（见 §2.3.4） | 破坏即插即用，且 `busy()` 会关掉摔倒反射 |

## 2.1 阶段 0 · 环境搭建（不训练，先跑通）

**前置**：Linux/macOS + `uv` + **CUDA GPU**（训练要）；纯跑仿真观看用 CPU 亦可。

```bash
# 1) 主仓库：编译守护进程 + 提供 duck-sim 编排脚本
cd <microduck>
cargo build                      # duck-sim 会自己重编，第一次先跑通

# 2) microduck_rl：建 venv（提供 duck-body / 场景 / libonnxruntime）
cd <microduck>/microduck_rl
UV_HTTP_TIMEOUT=600 uv sync      # ARM 机器首次约 2GB CUDA wheel，必须放宽超时

# 3) 一条命令起全套仿真（真 robotd + MuJoCo 身体）
cd <microduck>
DUCK_SIM_RL=<microduck>/microduck_rl scripts/duck-sim
scripts/duck-sim status          # 健康 + 是否站起
scripts/duck-sim realtime        # 必须 ≥ 1.0x 实时，否则策略站不住
scripts/duck-sim drive           # 走 8 秒（默认 0.15 m/s），会报实际位移
scripts/duck-sim down
```

**验收标准**：`status` 显示 standing（trunk ≈0.12 m）；`drive` 报位移达到 commanded 的 ≥25%。

> 关键点（来自 `docs/robot/simulation.md` + `scripts/duck-sim`）：
> - 默认场景是 **`backlash`**（不是 `scene.xml`）——VelStand 就是在带 backlash 的模型上训的，换模型它会“一动不动”。
> - `--sim` ≠ `--fake`：后者**没有物理**，只适合单测。
> - 实时比 <1.0 时策略**无法平衡**，先 `DUCK_SIM_VIEWER=0`，再减相机/鸭子。

**你仓库自己的环境**（后续阶段用）：

```bash
cd <microduck-ukemi>
uv sync                          # 见 §3.3 的依赖配置
uv run list-envs                 # 应能看到你自己的任务 id
```

## 2.2 阶段 1 · 基线与测量（“Measure before theorizing”）

**目标**：在仿真里量化 **当前**的保护表现，作为后续一切改动的对照。分析报告 §4 已给出方案，这里落到可执行：

**(a) 场景夹具**：写一组可复现的推倒/跌落场景（正面推、侧面推、后推、台阶边踩空、带自旋），固定随机种子。实现为一个 `events` 里对基座施加脉冲的 env，或用 `duck-body` 直接做 `read/write` 注入。

**(b) 度量（损伤代理）**：每步记录并落盘：

| 指标 | 物理量 | 来源 |
|---|---|---|
| 峰值关节加速度 | `ReLU(|q̈|)` 的 episode 最大值 | `mdp.py` `servo_acc_spike_penalty` 的同一信号 |
| 加速度超阈占比 | `(#|q̈|>300)/步数` | 同上 |
| 头壳/颌冲击 | 接触力峰值 × 时长（HIC 类比） | Roulade 的 `head_ground_contact` ContactSensor |
| 体轴冲击 | `trunk_vertical_accel_penalty` | `mdp.py` |
| 堵转计数 | 力矩>0.4 & 速度<0.5 的步数 | `servo_stall_penalty` |
| 自旋（头颈扭转） | `|ω_yaw|` 峰值 | vetbook 的 `spin` 字段语义 |

**(c) 时钟对齐**：仿真步与 daemon tick 用一次**可辨识的冲击事件**做锚点对齐（分析报告 §4(c)）。

**(d) 产出**：一张“基线 vs 目标”表 + 一个**回归夹具**（同样的种子、同样的场景，改动后能一键对比）。这个夹具是你仓库的**核心资产**，比策略本身更值得先做。

## 2.3 阶段 2 · 环境与奖励开发

遵循 `microduck_rl/AGENTS.md` 的新环境六步：

1. **选最近模板**：`microduck_velstand_env_cfg.py`（含保护性摔倒+起身）为最贴近的模板。
2. **先在 sim 验证物理假设**：目标姿态必须是**稳定平衡点**（hold ctrl 3 秒看 TILT）。
3. **配置约定**：`ENABLE_*` 开关、`make_..._env_cfg(play, rough)`、注册到你的 `tasks/__init__.py`。
4. **写 cfg 测试**（CPU、不训练）。
5. **冒烟测试**：`--env.scene.num-envs 64 --agent.max-iterations 5`。
6. **再训练**。

**你的任务命名**（示例）：

- `Mjlab-Ukemi-Rough-Backlash-MicroDuck` —— 保护性落地/受身主任务。
- `Mjlab-UkemiStand-Rough-Backlash-MicroDuck` —— 受身 + 起身（对标 VelStand）。

**奖励设计要点**（直接沿用 `mdp.py` 的既有项并新增受身项）：

| 项 | 来源 | 说明 |
|---|---|---|
| `servo_acc_spike_penalty` | 复用 | **核心**：接触瞬间关节加速度是损伤代理 |
| `servo_stall_penalty` | 复用 | 防堵转 |
| `head_ground_contact` 的温和落地项 | 复用 Roulade 思路 | 头壳接触要**软**、要**晚** |
| `roulade` 式“密进度 + 完成门控” | 复用范式 | 用势函数整形，避免 jackpot |
| `ukemi_roll_progress` | 新增 | 侧向跌倒时用 `head_roll` 满行程 + 同侧腿先接地 |
| `chin_tuck_reward` | 新增 | 奖励 `neck_pitch` 收、`head_pitch` 配合（Roulade 已验证） |
| `landing_order_reward` | 新增 | 奖励“脚/膝 → 髋/腹 → 头壳”的正确着地顺序 |

**禁止项**（`AGENTS.md` 明令）：符号约定别搞反、不要 jackpot、不要用**正奖励**去 gate 在坏状态上、别用权重比拼代替奖励质量比较。

### 2.3.4 路线 A 的协同问题（必须处理）

`robotd` 在 `FallPredictor` 命中时会 `LimpFall`（`gain_limp=50`）——**主动放软**。你的策略若想做“主动受身”，两者会打架。三种处理：

1. **让策略适应 limp**：把 daemon 的 limp 行为当作环境的一部分（它本来就是），训练策略在 `gain=50` 下完成受身。**最即插即用**。
2. **提高预测阈值/缩短 limp 窗口**：通过 `robotd.toml` 参数调整（若暴露）。
3. **提案 daemon 侧**：让 LimpFall 之后优先切到你的保护策略——**这条要单独走上游提案**，不放进“即插即用”承诺里。

## 2.4 阶段 3 · 训练 / 导出 / 仿真部署验证

```bash
cd <microduck-ukemi>

# 训练（本地 GPU）
uv run train Mjlab-Ukemi-Rough-Backlash-MicroDuck --env.scene.num-envs 4096

# 无 GPU：走 HF Jobs
uv run train Mjlab-Ukemi-Rough-Backlash-MicroDuck --env.scene.num-envs 4096 --hf-jobs

# 导出 ONNX（必须走 export，归一化才会烤进图里）
uv run scripts/export.py Mjlab-Ukemi-Rough-Backlash-MicroDuck \
    --wandb-run-path <entity/project/run_id>
```

**仿真部署验证（关键一步，证明“即插即用”）**：把导出的 ONNX 命名为 `velstand.onnx` 放进 `<microduck>/policies/`（`scripts/duck-sim` 的 `policies()` 会**优先**采用仓库内的 `policies/`），然后：

```bash
# 让真 robotd 加载你的策略，在 MuJoCo 上推倒它
cd <microduck> && scripts/duck-sim
scripts/duck-sim drive
# 用你自己的脚本施加推倒脉冲（走 body 的 TCP read/write），记录 §2.2 的指标
```

**验收**：同一场景下，你的策略在“峰值关节加速度 / 头壳冲击 / 堵转计数”三项上优于 VelStand 基线，且**仍能站、仍能走**。

## 2.5 阶段 4 · 发布为机器人可安装的策略集

```bash
# 发布（先 --dry-run 看要传什么）
uv run publish --onnx output.onnx --repo <user>/microduck-ukemi \
    --kind perpetual --slot walk \
    --description "Protective-landing gait: falls onto torso, not the head." --dry-run

uv run publish --onnx output.onnx --repo <user>/microduck-ukemi --kind perpetual --slot walk
```

`publish` 会：校验图 `[1,61]→[1,14]`、拒绝 NaN/常量输出、从 git+wandb 填 `training` 块、生成符合 schema 2 的 `manifest.json`。

**机器人与社区使用者侧**（无需 daemon 发版）：

```bash
sudo robotctl policy load walk <user>/microduck-ukemi   # 装进 walk 槽
# 或作为一次性技能（若你选择 episodic 形态）：
sudo robotctl policy add ukemi <user>/microduck-ukemi
robotctl robot do ukemi
```

> 注意：`PolicyParams::allow_community` **默认关闭**——机器人开箱只从 `pollen-robotics` 拉策略。社区使用者需先打开该开关或手动 `policy load`。这一点要在你的 README 里写清楚。

## 2.6 阶段 5（可选）· 真机

拿到硬件后：先 `robotctl policy load` 装上，再按 §2.2 的指标采集真机数据，与仿真对齐。分析报告 §5 的风险（无臂、`head_roll` ±25°、舵机力矩不足）在这一步会最真实。

## 2.7 参考依据对照表

| 本文论断 | 依据文件 |
|---|---|
| 15 关节契约含 `mouth`；无手臂 | [duck-ipc-proto/src/lib.rs](./duck-ipc-proto/src/lib.rs) `JOINT_NAMES` |
| 头部行程、头壳质量 | [kinematics/assets/alpha/robot_walk.xml](./kinematics/assets/alpha/robot_walk.xml) |
| 摔倒反射 + `busy()` 关闭反射 | [robotd/src/control.rs](./robotd/src/control.rs)、[robotd/src/main.rs](./robotd/src/main.rs) `LimpFall` |
| 技能即配置 `SkillDef` | [robotd-params/src/lib.rs](./robotd-params/src/lib.rs) `SkillDef`/`PolicyParams` |
| 策略槽位与 `Net::Skill` | [duck-control/src/policy.rs](./duck-control/src/policy.rs) |
| 轨迹预测阈值 | [duck-control/src/fall.rs](./duck-control/src/fall.rs) |
| 技能优先级 | [robotd/src/control.rs](./robotd/src/control.rs) |
| 仿真即真机代码 | [docs/robot/simulation.md](./docs/robot/simulation.md)、[docs/design/simulation.md](./docs/design/simulation.md) |
| 一键仿真编排 | [scripts/duck-sim](./scripts/duck-sim) |
| 策略集下载 | [scripts/seed-policies.sh](./scripts/seed-policies.sh) |
| 61D obs / 14 action / 任务注册 / 发布流程 | [microduck_rl/README.md](./microduck_rl/README.md)、[microduck_rl/pyproject.toml](./microduck_rl/pyproject.toml) |
| 新环境工作流与奖励规则 | [microduck_rl/AGENTS.md](./microduck_rl/AGENTS.md) |
| 仿真身体协议 | [microduck_rl/src/mjlab_microduck/sim/body_server.py](./microduck_rl/src/mjlab_microduck/sim/body_server.py) |
| 收颌参数与翻滚奖励范式 | [microduck_rl/src/mjlab_microduck/tasks/microduck_roulade_env_cfg.py](./microduck_rl/src/mjlab_microduck/tasks/microduck_roulade_env_cfg.py) |
| 损伤代理函数与阈值 | [microduck_rl/src/mjlab_microduck/tasks/mdp.py](./microduck_rl/src/mjlab_microduck/tasks/mdp.py) |
| 保护性摔倒基线 v8 | [microduck_rl/docs/velstand_policy.md](./microduck_rl/docs/velstand_policy.md) |

---

# 第三部分 · 独立开源项目开发流程

## 3.1 仓库定位与命名

**定位**：Microduck 的**摔倒保护（Ukemi）训练与策略仓库**，与 `microduck_rl` 平级、互相补充，**不 fork、不修改**任何上游仓库。

**命名建议**：`microduck-ukemi`（受身）。理由：语义准确、与 `microduck_rl` 并列、HF 仓库名可直接复用 `<user>/microduck-ukemi`。备选 `microduck-fall-guard`、`microduck-protect`。

## 3.2 目录结构（照抄 microduck_rl 的形状）

```
microduck-ukemi/
├── pyproject.toml              # 依赖 mjlab-microduck；注册 mjlab.tasks entry point
├── README.md                   # 定位 / 快速开始 / 如何在机器人上装
├── AGENTS.md                   # 开发规则（复用 microduck_rl 的写法）
├── LICENSE                     # Apache-2.0（与上游一致）
├── LICENSE-MODELS.md           # 若复用 3D 模型资源，需遵守 CC-BY-NC-SA-4.0
├── src/microduck_ukemi/
│   ├── tasks/
│   │   ├── __init__.py         # register_mjlab_task(...) —— 即插即用的接缝
│   │   ├── rewards.py          # 复用 mdp.py + 新增受身项
│   │   ├── ukemi_env_cfg.py
│   │   └── ukemi_stand_env_cfg.py
│   └── eval/
│       ├── scenarios.py        # 推倒/跌落场景矩阵（固定种子）
│       ├── metrics.py          # 损伤代理指标（§2.2）
│       └── regression.py       # 回归夹具：一键跑基线对比
├── policies/                   # 开发用 ONNX（被 duck-sim 优先采用）
├── docs/
│   ├── design.md               # 机制归属：本页唯一权威
│   ├── metrics.md              # 指标定义
│   └── deploy.md               # 机器人侧安装说明
├── tests/                      # CPU 单测：索引映射、奖励符号、NaN 守卫
└── .github/workflows/ci.yml
```

## 3.3 依赖与 entry point（**即插即用的关键**）

`pyproject.toml` 的核心片段：

```toml
[project]
name = "microduck-ukemi"
requires-python = ">=3.12, <3.13"
dependencies = ["mjlab-microduck", "mjlab==1.3.0", "torch==2.9.1"]

# 训练框架靠这个发现你的任务：`uv run train Mjlab-Ukemi-...` 才能工作
[project.entry-points."mjlab.tasks"]
microduck_ukemi = "microduck_ukemi.tasks"

[tool.uv.sources]
# 上游未上 PyPI，用 git 依赖拿模型 / BAM / mdp 函数 / duck-body / publish
mjlab-microduck = { git = "https://github.com/pollen-robotics/microduck_rl", branch = "develop" }
```

你的 `tasks/__init__.py`：

```python
from mjlab.tasks.registry import register_mjlab_task
from mjlab_microduck.tasks import MicroduckOnPolicyRunner   # 直接复用上游 runner
from .ukemi_env_cfg import make_ukemi_env_cfg, MicroduckUkemiRlCfg

register_mjlab_task(
    task_id="Mjlab-Ukemi-Flat-MicroDuck",
    env_cfg=make_ukemi_env_cfg(),
    play_env_cfg=make_ukemi_env_cfg(play=True),
    rl_cfg=MicroduckUkemiRlCfg,
    runner_cls=MicroduckOnPolicyRunner,
)
```

> **为什么这是“即插即用”**：`mjlab` 通过 entry point 加载任务包，因此你的仓库和 `microduck_rl` 可以**同时安装、互不干扰**；你的 ONNX 通过 HF + `robotctl` 进机器人，和上游策略走**同一条通道**。

## 3.4 开发工作流（沿用上游 AGENTS 规则）

强制流程，写进你的 `AGENTS.md`：

1. `main` 保持可运行；每个改动小步提交。
2. **改奖励/环境前先跑基线夹具**（§2.2），记录数字。
3. 新任务：模板 → 验证物理假设 → 配置 → cfg 测试 → **64 envs × 5 iters 冒烟** → 训练。
4. **报告 rollout 真实结果**，不写“it works!”。
5. 奖励改动必须说明**符号方向**与**为什么不 jackpot**。
6. 训练前必跑 CPU 单测（`uv run --with pytest pytest tests/`）。

## 3.5 测试与 CI

| 层 | 内容 | 何时跑 |
|---|---|---|
| 单元 | 关节索引映射、奖励符号、NaN 守卫（对齐上游 `tests/`） | 每次 push（CPU） |
| 配置 | 每个 env cfg 可实例化、`ENABLE_*` 组合不炸 | 每次 push（CPU） |
| 冒烟 | 64 envs × 5 iters | PR（可选，需 GPU runner） |
| 回归 | §2.2 场景夹具 vs 基线 | 发布前（需 GPU） |

CI 矩阵：Python 3.12、CPU 单测默认跑，GPU 任务用手动触发或 self-hosted runner（避免公共 CI 烧 GPU）。

## 3.6 发布流程

**版本语义**：
- 仓库版本 `vX.Y.Z`（git tag）。
- 策略集版本独立于代码：HF 上按 `--tag vN` 打标签（对齐上游 `pollen-robotics/microduck-policies` 的 `v5`/`v9` 习惯）。

**一次发布的步骤**：

1. `main` 绿 + 回归夹具通过。
2. `uv run scripts/export.py <任务> --wandb-run-path <...>` 导出 ONNX（**必须**走 export，归一化才会烤进图）。
3. `uv run publish --onnx ... --repo <user>/microduck-ukemi --kind perpetual --slot walk --dry-run` 自检。
4. 去掉 `--dry-run` 正式上传，`--tag vN`。
5. 在 README 更新“安装命令”与“与基线的指标对比”。
6. git tag `vX.Y.Z` + GitHub Release，附 rollout 视频（`render_policy.py` 产 `replay.mp4`）。

**许可证**：代码 **Apache-2.0**（与上游一致，便于被社区采纳）；若复用上游 `.stl` 模型资源，**必须**遵守 **CC-BY-NC-SA-4.0** 并保留 `LICENSE-MODELS.md`。建议在自己的 env 里**只引用上游模型、不复制**，从而只受 Apache-2.0 约束。

## 3.7 文档与社区

- `docs/design.md` **唯一权威**：一个机制只在一页写，其它页一句话 + 链接（沿用上游 `docs/README.md` 的纪律）。
- `README.md` 顶部放真机（或仿真）**摔倒保护对比视频**——这是“为什么要用”的第一屏。
- 提供**一键验证**：`uv run python -m microduck_ukemi.eval.regression` 输出基线 vs 你的策略的指标表。
- 面向社区的两条安装路径写清楚：
  1. **仿真试用**：起 `duck-sim`，把 `policies/velstand.onnx` 换成你的。
  2. **真机安装**：`robotctl policy load walk <user>/microduck-ukemi`，并提示 `allow_community` 默认关闭。
- 与上游互动：把“daemon 侧 ukemi 触发”（路线 B）作为 **issue/RFC** 提给 `pollen-robotics/microduck`，而不是自行 fork 修改。

---

## 附录 · 风险与边界（诚实版）

| 风险 | 说明 | 缓解 |
|---|---|---|
| 无手臂、`head_roll` ±25° | 受身手段被物理限制 | 目标定为“**微型受身**”：收颌 + 腿先接地 + 躯干/背先接触 |
| 舵机力矩不足 | 冲击 5 mm 刹停 ≈96 N，XL330 级 ≈20 N | 不靠“撑住”，靠“让开 + 拉长减速时间 + 分散” |
| daemon 的 LimpFall 与主动受身冲突 | 反射会放软 | 先做路线 A（让策略适应 limp），daemon 改动单独提案 |
| 仿真保真度 | 驱动层（Dynamixel 总线、ISP、NPU）不建模 | 仿真只验证控制/策略；驱动问题留待真机 |
| 实时比 <1.0 | 策略会站不住，指标失真 | `DUCK_SIM_VIEWER=0`、减鸭子/相机 |
| `allow_community` 默认关 | 社区使用者装不上你的策略 | README 显式说明开关位置与手动 `policy load` |
| 只发常量指令策略 | `publish` 只接受常量指令策略；相位驱动的（ground pick、sit↔stand）不行 | 你的 gait/ukemi 属常量指令类，可发布 |

**下一步建议**：先完成 §2.1 环境搭建与 §2.2 基线夹具——**在写任何奖励之前，先有一张能重复测量的对照表**。这与上游 `AGENTS.md` 的 “Measure before theorizing” 一致。
