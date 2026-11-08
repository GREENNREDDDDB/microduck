# Microduck 摔倒保护（重点：头/颈）分析报告

> 目标：为 Microduck 设计一套摔倒保护功能，核心诉求是解决“头重脚轻 → 摔倒时头部与颈部承受重创”的问题。
> 本文分三部分：① 拆解 Microduck 现有工程（只讲与保护相关的部分）；② 机器人领域摔倒保护研究综述；③ 把研究落到 Microduck 上的具体方案与实施路线。

---

## 0. 结论摘要（TL;DR）

1. **问题成立，且有定量依据。** 依据 MJCF 模型 `kinematics/assets/alpha/robot_walk.xml`，头部链质量约 **0.24 kg**，占整机模型质量（约 0.74 kg）的 **~32%**，且位于全机最高的位置；仅“头壳”一体（含相机）就有 **0.1726 kg**，比一整条腿（约 0.12 kg）还重。这是一台典型的“上重下轻”双足机器人，前倾跌倒时头部/喙最先着地。

2. **项目已经有一套“摔倒软着陆”机制，但它是“整机变软”，没有任何针对头/颈的保护。** 现有链路为：
   `FallPredictor`（摔倒预测，`duck-control/src/fall.rs`）→ `LimpFall` 状态机（`robotd/src/main.rs`）→ `gain_limp=50` 整体让步 → 落地后回正姿态 → 交回站立策略。它降低了整机冲击，但没有回答“用什么部位着地”“颈部舵机如何避免被打坏”这两个对头/颈最关键的问题。

3. **可利用但尚未使用的信号：** 头部自带 IMU（beta 板 LSM6DSV16X，`robotd/src/head_imu.rs`）与头部 ToF 深度（`tofd`）。**控制环当前不读深度**（`docs/design/architecture.md` 明确 “Nothing in the control loop reads depth”），头部 IMU 也只发布不消费。这两个信号恰好是头部保护最需要的（头部姿态/角速度 + 离地高度）。

4. **建议三层防护体系：**
   - **A 软件层（主力，可在现有代码上直接扩展）**：预测式“保护性落地”——按跌倒方向选择头部保护姿态（收颌/转向，让刚性最强的颅侧着地）+ 接触链分层顺从 + 头部 IMU 二次冲击看门狗。
   - **B 结构层（最可靠、成本最低、必须最先做）**：头部碰撞几何与缓冲（软质/非牛顿流体护具）、颈部机械限位与缓冲垫、颈部舵机扭矩限制（牺牲件优先于舵机损坏）。
   - **C 策略层（上限最高，长期）**：在 `microduck_rl`（MuJoCo+PPO）中训练“自我保护摔倒策略”，并作为独立 policy slot 通过现有的 `limp_fall` 交接机制挂载。

5. **优先级建议：结构被动保护 → 软件收头反射 → 接触感知二次冲击防护 → ToF 高度自适应 → 学习式保护策略。** 理由：软件会失效，结构不会；且颈部舵机（XL330 级）在 25 cm 跌落冲击下几乎必然被反驱动，靠“顶住”来保护是错误方向，靠“软 + 改变着地部位 + 结构耗能”才是正确方向。

---

## 1. Microduck 工程拆解（与摔倒保护相关的部分）

### 1.1 形态与硬件

来自 `README.md` 与官网技术参数：约 **25 cm 高、800 g 重**，主控 Rockchip RK3566，**15 个舵机**，**50 Hz 控制环**，相机 + LiDAR(ToF) + **双 IMU**，出厂 7 种训练动作（走/坐站/踢/抓取/轮滑/起身）。开源（Apache-2.0），策略在隔壁仓库 `microduck_rl` 用 MuJoCo + PPO 训练后导出 ONNX。

**关节拓扑**（`duck-ipc-proto/src/lib.rs` 的 `JOINT_NAMES`，共 15 个）：

| 部位 | 关节 |
|---|---|
| 左腿（5） | `left_hip_yaw` / `left_hip_roll` / `left_hip_pitch` / `left_knee` / `left_ankle` |
| 右腿（5） | `right_hip_yaw` / `right_hip_roll` / `right_hip_pitch` / `right_knee` / `right_ankle` |
| **头部链（4）** | `neck_pitch` / `head_pitch` / `head_yaw` / `head_roll` |
| **第 15 个** | **`mouth`（嘴）** |

> **关键约束：Microduck 没有手臂。** 第 15 个关节是 `mouth`，不是躯干 yaw——全机只有 **10 个腿关节 + 4 个头部关节 + 1 个嘴关节**。这一点对 §3.5 的算法分析是决定性的：人形机器人摔倒保护里最常用的“用手臂支撑/三角结构/两手一脚”等手段，对 Microduck **都不存在**，它只有腿、躯干、头壳这三个可用的接触/发力部位。

头部链是 **4 自由度串联链**：`trunk → neck(neck_pitch) → neck_pitch(head_pitch) → yaw_roll_motion(head_yaw) → bottom_head_shell(head_roll)`（见 `robot_walk.xml`）。注意其关节名称与 MJCF body 名称有一点反直觉：MJCF 中的 `neck` body 带 `neck_pitch` 关节，而名为 `neck_pitch` 的 body 带 `head_pitch` 关节。**颈部/头部的 4 个舵机是整机最薄弱、也最容易被摔倒冲击打坏的执行器。**

头部 4 个关节的**行程**（取自 `robot_walk.xml` 的 `range`，弧度→角度）与默认站立位姿（`duck-control/src/model.rs` 的 `DEFAULT_POSITION`）：

| 关节 | 行程 | 默认 | 对保护的意义 |
|---|---|---|---|
| `neck_pitch` | −90° ~ +60° | +20° | 收颌（向 −90°）行程充足 |
| `head_pitch` | −90° ~ +90° | +20° | 可再叠加收颌/仰头 |
| `head_yaw` | **±170°** | 0° | 可把喙/相机面转开，避开直面着地 |
| `head_roll` | **仅 ±25°** | 0° | “把强侧转过来着地”的能力**受限** |

（关节正负方向与“收/仰”的对应关系需在 MuJoCo 里实测确认，不要凭符号推断。）

### 1.2 质量分布：为什么这是一台“头重脚轻”的机器（定量）

直接从 `kinematics/assets/alpha/robot_walk.xml` 的 `<inertial mass=...>` 提取（单位 kg）：

| 部件 | 质量 | 说明 |
|---|---|---|
| `trunk_base`（躯干） | 0.2644 | 单件最重 |
| `bottom_head_shell`（头壳，含相机） | **0.1726** | **比一条腿还重，位于最高处** |
| `neck` | 0.0368 | 颈部结构 |
| `yaw_roll_motion` | 0.0249 | 头部偏航/滚转机构 |
| `neck_pitch`(body) | 0.0057 | 头部俯仰机构 |
| **头部链合计** | **≈ 0.2401** | |
| 左腿合计（yaw2roll+hip+upper_leg+leg+ankle） | 0.1199 | |
| 右腿合计 | 0.1199 | |
| **模型总质量** | **≈ 0.7444** | 不含电池等（800 g 为整机规格） |

**头部链占比 ≈ 32.3%**，与双腿各自（≈16.1%）相比，头部链相当于“两条腿加起来”的重量，并且悬在躯干上方的末端（躯干 z≈0.12 m，头部质心约在 0.17–0.20 m，头/喙顶端接近全机 25 cm）。这就是“头重脚轻”的物理根源。

**冲击力估算（用于论证“靠舵机顶住不可行”）**：
- 从站立高度跌落，头部着地速度约 `v = √(2gh) ≈ √(2×9.81×0.20) ≈ 2.0 m/s`。
- 头部链动能 `E ≈ ½mv² = ½×0.24×2.0² ≈ 0.48 J`。
- 若靠颈部舵机在 5 mm 内“刹停”，平均力 `F ≈ E/d = 0.48/0.005 ≈ 96 N`；即便给到 20 mm 缓冲，仍有 ≈24 N。
- 而 XL330 级舵机堵转扭矩约 0.4 N·m，在 2 cm 力臂处只能提供 ≈20 N。**结论：颈部舵机无法“顶住”落地冲击，只会被反驱动、掉齿或烧毁。** 因此保护策略必须是“让开 + 让结构耗能 + 改变着地部位”，而不是“加刚度顶住”。

### 1.3 软件架构与安全体系（保护相关的关键设计）

七个守护进程通过 Unix socket + JSON-RPC 通信，**`robotd` 是唯一能命令电机的进程**（`docs/design/architecture.md` §1.1）。安全层的核心设计（`duck-control/src/safety.rs`）：

- `Safety` **独占唯一的 `RobotIo` 写句柄**，用借用检查器保证“没有别的代码能绕过安全层命令电机”。
- 两条无条件规则：**非有限值（NaN/Inf）目标直接拒绝**（不钳位，避免机器人“猛地冲到一个限位”）；**超出舵机行程的目标钳位**并上报。
- **Deadman**：指令停发超过 `deadman_ms=500` → 速度清零（“停下而不是瘫软”，因为站立对双足才是安全态）。
- **关键设计原则：`fallen` 判定是一个“报告”，不是“门控”**。`Safety::fallen`（投影重力 z > `fall_gravity_z=-0.5` 持续 `fall_debounce=200ms`）只被记录和发布，**不抢占任何控制**——“摔倒该做什么”被明确划归到上层（`robotd`）的 `LimpFall`。

### 1.4 现有的摔倒处理链路（重点·逐层拆解）

这是本次改造的地基，需要精确理解。

#### （1）`FallPredictor` —— 摔倒“开始”的预测器（`duck-control/src/fall.rs`）

`Safety::fallen` 是“已经倒地”的判定，等到它 latch 时机器人已经在地上，对“落地前变软”没用。因此有一个独立的、**更早**的预测器。它的核心思想非常干净：

- 投影重力随躯干旋转，其导数有解析解：`ġ = −ω × g`。只关心 z 分量：
  `ġz = −(ωx·gy − ωy·gx)`
  ——**无需滤波、无需对四元数做差分**（那会引入滤波自身的延迟，而本预测器的全部价值就在于“早”），直接取同一 IMU 数据块里的陀螺仪。
- 把 `ġz` 线性外推 `lookahead=300ms`，问“300ms 后重力 z 到哪”。
- **三重必须同时满足的条件**：
  1. **已经倾斜**超过 `tilt_z=-0.90`（约 26°，正常行走达不到）——把落脚冲击、被人推、被人拿起都排除掉；
  2. **仍在倒**（`ġz > 0`，向下而不是从倾斜中恢复）；
  3. **外推值超过** `predicted_z=-0.5`（与 `fall_gravity_z` 同义，默认同一数值）。
- **去抖** `debounce=60ms`（50 Hz 下 3 个 tick，长于任何落脚冲击）。
- **只在“站立”时武装**：必须在 `tilt_z` 内持续 `arm=200ms` 才允许触发，触发后解除武装。这是为了修一个真实 bug（2026-10-27）：前跌后 `limp` 姿态把头留在约 58° 俯卧（未达 `fallen`），策略慢慢下坠时会被误判为“摔倒在途”，导致“limp→pose→limp”死循环。

#### （2）`LimpFall` 状态机（`robotd/src/main.rs`）

`enum LimpFall { Idle, Limp{since, landing}, Posing{from, since} }`，默认开启（`safety.limp_fall = true`）。

- **Idle**：仅当 `was_driving && !safety.fallen() && 无 shutdown_sit && 未断电 && 控制器不 busy 且不在坐姿` 时才喂给 `FallPredictor`；触发即进入 `Limp`。
- **Limp**：`gain` 降到 `gain_limp=50`，关节目标=“当前位置”（最软但不断电——断电会让头砸到地上并丢失后续回正的起点）。落地判据从**陀螺仪**读（不是定时）：角速度模长 < `limp_fall_still_rate=1.0 rad/s` 持续 `limp_fall_still_ms=200ms`，或硬超时 `limp_fall_max_ms=1500`（避免被人拿在手里时永远瘫软）。
- **Posing**：从实际落地位姿线性回正到 `DEFAULT_POSITION`，历时 `limp_fall_pose_ms=600`，增益 `limp_fall_pose_gain=160`（“软化后的站立增益”——关节要在地面上真正移动，不能太软）。
- 结束后交回站立策略。

参数默认值（`robotd-params/src/lib.rs` 的 `SafetyParams::default`，注释见 `deploy/robotd.toml` 第 262–327 行）：

| 参数 | 默认 | 含义 |
|---|---|---|
| `fall_gravity_z` | -0.5 | `fallen` 判定的重力 z 阈值 |
| `fall_debounce_ms` | 200 | `fallen` 去抖 |
| `gain_limp` | 50 | limp 时的增益（行走 200） |
| `limp_fall_tilt_z` | -0.90 | 预测器“已倾斜”门槛（约 26°） |
| `limp_fall_predict_z` | -0.5 | 外推落点门槛 |
| `limp_fall_lookahead_ms` | 300 | 外推时长 |
| `limp_fall_debounce_ms` | 60 | 预测去抖 |
| `limp_fall_arm_ms` | 200 | 站立武装时长 |
| `limp_fall_still_rate` | 1.0 rad/s | 落地静止判定 |
| `limp_fall_still_ms` | 200 | 静止持续 |
| `limp_fall_max_ms` | 1500 | limp 硬上限 |
| `limp_fall_pose_ms` | 600 | 回正时长 |
| `limp_fall_pose_gain` | 160 | 回正增益 |

#### （3）两条必须知道的“约束/陷阱”

- **任何 one-shot skill 运行期间，摔倒反射被关闭。** `LimpFall` 只在不 `busy()` 时被喂入；`docs/design/policy-channel-design.md` §10.2 明确指出这是“继承来的、应当重新决策的”行为：一个被配置成持续 10 秒的技能 = 10 秒没有摔倒反射。**新的头部保护反射不能走这条被 `busy()` 关掉的路。**
- **`fallen` 不门控任何东西**，所以 limp 的姿态回正可以在“躺在地上的机器人”上像普通 tick 一样移动关节。这是好的设计，改造时要保留这个性质（保护动作应作为普通 target/gain 通过 `Safety::apply` 下达，不新增绕过 `Safety` 的后门）。

### 1.5 现有机制对“头/颈”的不足（逐条）

1. **没有“着地部位”的选择。** 前倾跌倒时 `beak`（喙）与头壳正面必然先着地；`LimpFall` 只是让整机变软，不改变着地几何。而研究（§2.2）一致认为“控制着地部位”是降低损伤最有效的手段。
2. **头部 4 个关节在 limp 时只是“被动跟随”。** 没有主动把头收成保护姿态（收颌/转侧），因此头部质心的力矩臂没有缩短，颈部舵机承受的弯矩没有降低。
3. **触发偏晚、窗口极窄。** `tilt_z=-0.90`（约 26°）+ `lookahead=300ms` 是刻意偏保守的（文档承认“宁可晚一点”）。但从 26° 到落地往往只有几十到一百多毫秒，留给“主动调整头部姿态”的时间非常少——除非把“头部保护”前置为独立的、更早/更专用的通道。
4. **没有高度/跌落能量感知。** 从桌面（几十 cm）跌落与平地绊倒（几 cm）冲击能量差一个数量级，但参数完全相同。（现有 ToF 恰好在头部，却未被控制环使用。）
5. **没有“二次冲击”防护。** 落地后弹跳/翻滚的第二次接触没有专门处理；头部一旦磕碰，颈部舵机可能已被反驱动，后续动作缺乏“已被击打”的看门狗。
6. **头部 IMU 未被消费。** `robotd/src/head_imu.rs` 以 `head_imu.stream` 发布融合后的四元数/陀螺/加速度（60 Hz），但控制环里没有用它做任何事。
7. **ToF 深度未进控制环。** 架构文档明确写“Nothing in the control loop reads depth”，`tofd` 只是发布。一个天然的“离地高度/接近速度”传感器被闲置（对保护而言正当性很强）。
8. **sk ill 期间反射关闭**（见 1.4-3）。

---

## 2. 机器人领域摔倒保护研究综述

摔倒保护按“时间轴”可分成五个环节，下面每一类都给出代表工作与可迁移点。

### 2.1 摔倒检测与预测（何时介入）

- **基于 IMU 阈值 + 去抖**：Microduck 现有做法（`FallPredictor`）。优点是简单、无模型；缺点是需要人工调参、对“缓慢倾倒”不敏感。
- **基于动力学/动量的判据**：捕获点（Capture Point）、零力矩点（ZMP）、角动量/离心判据、Foot Rotation Indicator 等，用“是否还能恢复”的物理量预测不可恢复性，比单纯姿态阈值更早、更有物理意义。
- **学习式预测**：用 LSTM/Transformer 从 IMU 时序预测摔倒，可更早（但需要数据与推理开销）。
- **接触前测距预警**：用深度/ToF/视觉估计“离地高度 + 落地时间（TTC）”，在真正翻转前就知道“这一摔有多高”。**这是把 Microduck 头部 ToF 用起来的最直接理由。**

### 2.2 落地姿态规划 / Ukemi（“摔到哪、用什么着地”）

这是与“头/颈保护”最相关的一类。

- **UKEMI（柔道受身思想）**：Fujiwara 等提出控制摔倒过程中的姿态，使机器人**以预置的减震部位着地**，并证明可在仿真中命中指定减震部位（[ukemi.pdf](http://www.cs.cmu.edu/~cga/falling/ukemi.pdf)）。
- **Tripod Fall**（Yun & Goswami, ICRA 2014）：不追求“找个好看姿势摔”，而是**用一系列受控接触主动刹停**，阻止势能→动能的全量转换，从而显著降低最大冲击力（[Tripod Fall](http://www.ambarish.com/paper/Yun_Goswami_ICRA2014_Tripod_Humanoid_Fall.pdf)）。
- **全身轨迹优化最小化落地冲量**：把摔倒建成最优控制问题，目标是最小化与地面碰撞的冲量，受全身动力学与关节约束（[CMU ACC2012](http://www.cs.cmu.edu/~cga/ewhitman/webContent/ACC2012_2_paper.pdf)）。
- **能量整形 + 能量分布多边形（EDP）**：用能量整形把系统总能量最小化，再通过 EDP 把能量分散到多个接触点（[Survey on Control of Humanoid Fall Over](https://elib.dlr.de/195344/4/J17__Subburaman__2023__RAS.pdf)）。
- **综述**：同一篇 Survey 对摔倒检测、姿态规划、冲击缓解做了系统整理，适合作为进一步阅读入口。

**可迁移点**：Microduck 头部保护的本质就是一个“迷你 Ukemi”——目标不是“不摔”，而是“让头部**收拢后由躯干/背部先接地**，而不是喙/相机面直接着地”。（注意：`head_roll` 仅 ±25°，见 §1.1，因此“精确让强侧面着地”做不到，实际可行的是“收颌 + 转开喙 + 让躯干先接触”。完整算法见 **§3.5**。）

### 2.3 顺从控制 / 冲击缓冲（“怎么软”）

- **阻抗/导纳控制、reflex 增益调度**：接触建立时降低关节刚度、注入阻尼，把冲击峰值“摊平”到更长时间上。你现有的 `gain_limp` 是它的最简形式（全局开环降刚度）。
- **四足顺从着陆**：把“参考着陆轨迹尽量柔顺”建成 QP 在线求解，再用 MPC 跟踪关节力矩（[Online Multi-Phase Trajectory Generation for Compliant Landing](https://ieeexplore-qa.ieee.org/document/10406414/)）。
- **被动柔顺腿设计**：弹性/阻尼腿在空投着陆中把尖锐加速度峰值平滑成更长的缓坡，显著优于刚性腿（[DLuchten](https://i636f6f706572o656475z.oszar.com/sites/default/files/uploads/assets/engineering/files/2017/DLuchten_Paper.pdf)）。
- **软材料 / 非牛顿流体护具**：软—刚协同设计，平时柔顺、受冲击瞬间变硬耗能，42 kg 人形可重复跌落而不损坏（[Soft Responsive Materials Enhance Humanoid Safety](https://arxiv.org/html/2601.02857)）。

### 2.4 学习式自我保护策略

- **Discovering Self-Protective Falling Policy**：用大规模 RL + 课程学习，让人形自主发现保护姿势，发现**“三角形”结构**能显著降低刚性机身的摔伤（[arXiv 2512.01336](https://arxiv.org/pdf/2512.01336)）。
- **GYF: Guardians as You Fall**：四足主动模式切换框架，含“安全敏捷摔倒”的 transition controller 与快速恢复控制器，接触力/力矩显著小于基线（[arXiv 2310.04828](https://arxiv.org/pdf/2310.04828v1.pdf)）。
- **FIRM**：从少量人类演示中学“摔倒缓解 + 恢复”统一策略，按冲击方向选择姿态可测量地降低结构损伤（[arXiv 2511.07407](https://arxiv.org/pdf/2511.07407)）。
- **RL 落地冲击缓解（四足/月面）**：端到端 RL 控制器隐含区分“接触准备 / 能量耗散 / 稳定”三阶段（[scite 报告](https://scite.ai/reports/reinforcement-learning-based-landing-impact-mitigation-vJQlz1lJ)）。

**可迁移点**：Microduck 已有完整 RL 训练栈（`microduck_rl`）与 policy-channel 机制，训练一个“自我保护摔倒策略”在工程上是顺理成章的，且不侵入主控制环。

### 2.5 机械 / 被动保护

- **碰撞几何设计**：加圆角肩盖、髋部护板、头/腕部橡胶保险杠，把力分散到机身更强的部位（[smashingrobotics 综述](https://www.smashingrobotics.com/how-do-humanoid-robots-recover-from-a-fall/)）。
- **外部安全气囊**：Wandercraft 用 Simcenter Madymo 仿真论证**单气囊比多气囊更安全可靠**——多气囊依赖每个都在精确时刻触发，任一个失效等于没有保护（[Siemens 案例](https://resources.sw.siemens.com/en-US/case-study-wandercraft/)）。
- **可穿戴缓冲（外骨骼/护具）**：自适应顺应控制可在跌倒中实时调整关节刚度，峰值冲击较刚性驱动降低约 42%（[Adaptive Compliance Control](https://freederia.com/adaptive-compliance-control-in-wearable-exoskeletons-for-dynamic-fall-impact-mitigation/)）。

### 2.6 接触估计（用于“无传感器”感知冲击）

- **多动量观测器接触估计**：不依赖足端力传感器，用多组“不同接触假设”的动力学模型约束全身动力学来估计接触状态（[arXiv 2412.03462](https://arxiv.org/pdf/2412.03462)）。
- **足式触地检测综述**：外部传感器（力/触觉）与本体传感器（编码器+IMU 反演）两大类（[ Academax 综述](https://www.academax.com/ZDXBGXB/doi/10.3785/j.issn.1008-973X.2024.02.012)）。

**可迁移点**：Microduck 没有接触力传感器，但有 **双 IMU + 舵机电流/负载**（`Safety::measures_load`）。头部 IMU 的角速度/加速度突变可作为“头部撞击”的隐式力传感，用于二次冲击看门狗。

### 2.7 小结：对 Microduck 最重要的四条结论

1. **改变着地部位 > 单纯变软**（Ukemi / Tripod Fall）。
2. **主动顺从/耗能 > 加刚度顶住**（冲击力估算已证明舵机顶不住）。
3. **被动结构保护最可靠**（软件会失效、结构不会）。
4. **学习式保护策略是上限最高的方向**，且 Microduck 的训练栈天生支持。

把这四条**落成可实施算法**的，是 **§3.5（姿势变换式自我保护）**——它给出方向识别、着地选择表、姿态切换执行与接触能量分散四步，并诚实标注哪些研究因 Microduck 无手臂而不可直接迁移。

---

## 3. 方案设计：把研究落到 Microduck 上

### 3.0 保护对象与量化目标

保护对象：**头壳（含相机/ToF 光学面）+ 颈部 4 个舵机与结构 + 喙**。
量化目标（可测）：
- 降低**头部链峰值加速度/角加速度**（用头部 IMU 测量，可对比基线）；
- 降低**颈部/头部舵机的峰值扭矩与电流**（`vetbook` 已记录“peak current”与“falls, soft landings”）；
- 改变着地部位：让**头壳最后着地**（由躯干/背部先接触），并**转开喙/相机面**避免其直接着地（受 `head_roll` 仅 ±25° 限制，“精确让头侧着地”做不到，见 §1.1 与 §3.5.3）。

### 3.1 层 A —— 软件：预测式“保护性落地”（主力，可复用现有骨架）

**A1. 头部收拢反射（reflexive head tuck / 迷你 Ukemi）**
- 在 `FallPredictor`（`duck-control/src/fall.rs`）触发时，**额外输出“跌倒方向”**：由重力水平分量 `[gx, gy]` 判断前/后/左/右倾，由陀螺仪 `ω` 判断旋转方向。
- 依据方向选择**头部保护姿态**并在一两帧内下达到头部 4 关节：
  - **前倾跌**：`neck_pitch`/`head_pitch` 收到极限（收颌/抬头后仰），让**颅顶/颅侧**着地 → 保护喙与相机；
  - **后倾跌**：收颌压低头部质心，缩短头对颈的力矩臂（力矩 = 力 × 距离）；
  - **侧向跌**：`head_roll`/`head_yaw` 转到让头侧（强侧）着地。
- 关键：这一反射**不经过会主动驱动关节的站立策略**，而是直接作为 target 下达（保持“从 `Safety::apply` 走”的既有原则，不新增后门）。

**A2. 接触链分层顺从（layered compliance）**
- 把现在“全局 `gain=50`”升级为**分层增益调度**：
  - **接触链（颈部 4 个关节）**：低刚度 + 高阻尼（“让开”）；
  - **支撑链（腿部）**：保持相对刚性以“减速整机、吸收主要动能”。
- 这与研究里的“阻抗控制 + 能量在多接触点分散（EDP）”是同一思想的最小可行版本。

**A3. 头部 IMU 二次冲击看门狗**
- 消费 `head_imu.stream`（当前只发布未用）。当 `|ω|` 或 `|a|` 超过阈值（“头已经受击”），立即进入**颈部最大让步并锁定**，避免弹跳/翻滚造成的二次冲击把颈部舵机继续打坏。
- 这与“多动量观测器接触估计”同源：用本体 IMU 反演接触事件，不依赖力传感器。

**A4. 高度/落地时间自适应（用现成的头部 ToF）**
- 让 `robotd` 订阅 `tof.frame`（或 `tofd` 出一个轻量“地面距离/接近速度”特征），结合头部姿态外推估计**离地高度与落地时间**，据此决定保护力度：
  - 低摔（平地绊倒）：轻保护，避免“为了防摔反而绊倒自己”；
  - 高摔（从桌面/台阶）：强保护，主动收头 + 最大让步。
- 这是把现有闲置 ToF 信号正当引入控制环的最佳场景。

**A5. 与现有 `LimpFall` 的整合**
- 在 `LimpFall` 状态机中新增相位，例如 `Brace`（收拢头部保护姿态 + 分层顺从）→ `Limp`（现有落地）→ `Posing`（现有回正）。保持“只在从站立武装、触发后解除武装”的既有约束不变。

### 3.2 层 B —— 结构：碰撞几何与被动缓冲（最可靠，应先做）

1. **头部护具/护垫**：在颅顶与头侧加软质或**非牛顿流体**护具，扩大接触面积、降低峰值压强；给相机/ToF 光学面加凸起或护圈，使其不直接触地。
2. **颈部机械限位 + 缓冲垫**：在 `neck_pitch`/`head_pitch` 行程端点增加弹性/摩擦耗能结构，让“到极限时的能量”由结构吸收，而不是由舵机齿轮承担。
3. **舵机扭矩限制 / 牺牲件**：对颈部 4 个舵机设置 `goal current` / 扭矩上限，过载前先打滑；或设计**可更换的塑料牺牲件**，让其先于舵机损坏。
4. **降低整机重心（可选，治本）**：在脚/腿部配重，或对头壳做减重/把电池等重物下移——直接削弱“头重脚轻”。
5. **外部安全气囊（仅针对高台跌落）**：若目标场景包含从高桌跌落，可按 Wandercraft 的教训用**单气囊**方案，并先用 MuJoCo 仿真确定触发时机与包覆位置。

### 3.3 层 C —— 策略：学习式自我保护（长期，上限最高）

1. 在 `microduck_rl`（MuJoCo+PPO）中新增**保护性奖励**：惩罚头部接触力/头壳峰值加速度/颈部力矩，激励网络自发学到“蜷缩/三角”保护姿态（对应 §2.4）。
2. 产出的 ONNX 作为一个**独立 policy slot**，利用现有 policy-channel（`robotctl policy load`）安装，无需改动 daemon 发布。
3. **挂接方式**：`FallPredictor` 触发 → 切换到 protective-fall 策略 → 落地静定 → 交回 `stand`。这正好复用现有 `LimpFall` 的“接管—回正—交回”机制。
4. **必须解决的架构约束**：现有的 `busy()` 门控会关掉摔倒反射；保护策略本身是反射的执行者，**不能走这条会被关掉的路**，需要专门设计（例如让 `FallPredictor` 的接管独立于 `busy()`）。

### 3.4 与现有代码的耦合点（改造落点一览）

| 改造点 | 文件 | 说明 |
|---|---|---|
| 摔倒方向输出 | [fall.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-control/src/fall.rs) | `FallPredictor` 扩展输出前/后/左/右 |
| 保护状态机 | [main.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd/src/main.rs#L3219-L3338) | `LimpFall` 增加 `Brace` 相位 |
| 分层增益 | [safety.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-control/src/safety.rs) | 保留 `apply` 唯一写句柄原则，按关节分层传 gain |
| 头部 IMU 消费 | [head_imu.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd/src/head_imu.rs) | 从“只发布”变为“控制环消费” |
| ToF 进控制环 | [tof/src/lib.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/tof/src/lib.rs) | 控制环读 `tof.frame` 或轻量特征 |
| 新参数 | [registry.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd-params/src/registry.rs#L277-L357) | 新增头部保护参数并入 `SafetyParams` |
| 训练保护策略 | `microduck_rl`（外部仓库） | 新奖励 + 新 ONNX |

### 3.5 算法层深入：姿势变换式自我保护（Ukemi / 滚翻卸力的“机器人化”）

这一节是本次扩展的重点：把“跑酷/柔道运动员摔倒时用滚翻受身卸力”这件事，拆成可以在 Microduck 上实现的**算法**。§3.1 的 A1 是它的最小雏形，这里给出完整的物理原理、机器人学对应算法、可行性边界与可实施设计。

#### 3.5.1 先搞清楚：为什么“滚翻受身”真的有效（物理本质）

人体受身的有效性不来自“动作好看”，而来自三个可量化的力学机制：

| 机制 | 人体做法 | 物理收益 |
|---|---|---|
| **① 让开关键部位** | 收下巴、把后脑/头移出接触链 | 头部根本不出现在“最先着地”的部位里 |
| **② 拉长减速时间** | 用身体曲线滚动，接触连续而非瞬时 | 同样的动量，作用时间 t↑ ⇒ 平均力 F=Δp/t↓ |
| **③ 分散到多点** | 力沿肩→背→腰→臀依次传递 | 峰值压强与局部力矩↓，避开单点折断 |

**量级参考**：柔道受身的生物力学研究一致显示，受身把头部运动学量压到远低于损伤阈值——例如一项对比给出“有受身时头部旋转加速度约 679 rad/s²，无受身约 5081 rad/s²”，而脑震荡阈值约 4500 rad/s²（[Hyakusha 训练资料整理](https://www.hyakusha.com/en/training/breakfall-comparison)；学术侧见 [MDPI 关于 Ukemi 生物力学的系统综述](https://www.mdpi.com/1660-4601/19/7/4259)，该综述的核心结论是“Ukemi 产生的运动学量始终低于直接枕部着地的损伤阈值”）。**这组数字只用于说明“机制成立、量级可观”，具体数值以各自论文的实验条件为准，不要当成 Microduck 的预期收益。**

一句话：**受身 = 用“改变着地部位 + 连续多接触 + 关节屈曲减速”把单点瞬时冲击拆成多点持续耗散。**

#### 3.5.2 机器人学里对应哪些算法（按“控制量”分四族）

这是把上表落成算法的关键。摔倒保护研究基本可归入四族：

**族 1｜方向识别与落点选择（决定“摔到哪、用什么着地”）**
- **UKEMI**（Fujiwara 等，CMU）：**由机器人沿 roll/pitch 轴的角动量判定跌倒方向 D**，然后在倒地过程中主动控制姿态，使机器人**命中预先标定的“减震部位”**着地；仿真验证可命中指定部位（[ukemi.pdf](http://www.cs.cmu.edu/~cga/falling/ukemi.pdf)）。
- **可迁移**：Microduck 已有整套角动量/重力方向信号（`FallPredictor` 的 `ġz` + 重力水平分量 + 头部 IMU），做“方向识别 → 落点选择表”是直接可做的。

**族 2｜接触时序与能量管理（决定“能量怎么被吃掉”）** ← 与“卸力”最相关
- **Tripod Fall**（Yun & Goswami, ICRA 2014）：让机器人以**三次连续接触（双手 + 单脚）**的“三角架姿态”摔倒，**刻意只让一部分势能转成动能**，从而降低落地速度与损伤（[Tripod Fall](http://www.ambarish.com/paper/Yun_Goswami_ICRA2014_Tripod_Humanoid_Fall.pdf)）。
- **Online Rolling Motion Generation**（Humanoids 2018，能量控制思想）：定义**临界滚动参数（Critical Rolling Parameters, CRP）= 接触位置 + 攻击角（attack angle）**，并引入**能量注入速度（energy-injection velocity）**来保证“连续多接触”；用 **能量分布多面体（Energy Distribution Polyhedron, EDP）**把系统能量**沿多个接触点逐步分散**，从而减小冲击（[Humanoids 2018](http://crlab.cs.columbia.edu/humanoids_2018_proceedings/media/files/0152.pdf)）。该线工作在综述中被描述为“可产生类似 Ukemi 与 Parkour roll 的动态动作”（[Survey on Control of Humanoid Fall Over](https://elib.dlr.de/195344/4/J17__Subburaman__2023__RAS.pdf)）。
- **可迁移**：Microduck 无臂，但可以把“双手+单脚”换成 **“双脚/膝 + 躯干”的多次连续接触**；把“能量分散”理解为**故意不让腿一次伸直**、让接触按“脚→髋/躯干→头”的顺序发生。

**族 3｜最优控制（离线算出“最佳保护姿态序列”）**
- **全身轨迹优化最小化落地冲量**：把摔倒建成最优控制问题，目标是最小化与地面碰撞的**冲量**，约束为全身动力学 + 关节空间限制，用非线性规划离线求解（[CMU, ACC 2012](http://www.cs.cmu.edu/~cga/ewhitman/webContent/ACC2012_2_paper.pdf)）。
- **可迁移**：Microduck 有 MuJoCo 模型（`kinematics/assets/alpha/robot_walk.xml`）与 `microduck_rl` 训练栈——**可以离线为每个“跌倒方向 × 高度”算出一段保护姿态序列，在线只做查表/插值**（在线求解器在 50 Hz 下不现实）。

**族 4｜学习式（把“最优保护”交给 RL 自己发现）**
- **Self-Protective Falling Policy**：大规模 RL + 课程学习，让刚性机身的人形**自己发现保护姿势**，学到三类行为——**“三角”结构（用上臂把冲量分散到多个刚体）**、**双腿展开降低 CoM 并减少在地面上的平移**、**对头/躯干/骨盆等关键部件的保护**（[arXiv 2512.01336](https://arxiv.org/pdf/2512.01336)）。
- **FIRM**：从少量人类演示学“缓解 + 恢复”统一策略，奖励里显式**惩罚身体碰撞、动量突变（momentum change）与身体猛拽（body yank）**（[arXiv 2511.07407](https://arxiv.org/pdf/2511.07407)）。
- **SafeFall**：**与主控解耦**的保护控制框架——轻量 **GRU 摔倒预测器** + RL 保护控制器，奖励函数**感知结构脆弱性**，联合优化关键关节保护（[SafeFall 摘要页](https://www.newx.sg/paper/detail/319c5992-966f-11f1-b84c-00163e10baa7)）。**这一条与 Microduck 的架构（`FallPredictor` 预测 + 独立保护动作）契合度最高。**
- **可迁移**：`microduck_rl`（MuJoCo+PPO）+ policy-channel 天生支持，且可作为独立 policy slot，不侵入主控制环。

#### 3.5.3 关键约束：Microduck 的身体决定了它能做什么、不能做什么

把 §3.5.2 的算法对照 Microduck 的硬件，必须诚实承认几个**硬边界**：

1. **没有手臂。** 这直接废掉了三类最有效的手段：
   - Tripod Fall 的“双手 + 单脚三接触” → 不存在手；
   - Self-Protective Falling Policy 的“三角结构”（上臂） → 不存在上臂；
   - Ukemi 的“肩部滚翻” → 不存在肩/臂当滚轴。
2. **没有浮动基座控制、没有动量轮。** 机器人不能像大型人形那样在空中用反作用轮/全身摆动大范围改变角动量——**唯一能改变整机角动量与质心的执行器是腿（10 个关节）**，头的动作因为质量小（头链 0.24 kg / 整机 0.74 kg，见 §1.2）只能做“局部微调 + 让开”。
3. **接触面极其有限。** 可用接触部位只有：脚/小腿、躯干（腹/背/侧）、头壳、喙、嘴。
4. **头部关节行程不对称**（见 §1.1 表）：
   - `neck_pitch` + `head_pitch` 收颌行程充足 → **“把头收起来 / 把喙缩到躯干轮廓内”是可行的**；
   - `head_yaw` ±170° → **可以把喙/相机面转到侧面，避免正面着地**；
   - `head_roll` 仅 ±25° → **“把强侧（颅侧）精准朝下”的能力有限**，不要指望像人头那样侧脸着地。
5. **时间预算极短。** 从 `FallPredictor` 触发（约 26°）到落地通常只有几十到一百多毫秒，50 Hz 下等于 **3~8 个 tick**。这意味着：
   - **不要做长而平滑的轨迹**（几十 ms 的平滑插值根本跑不完）；
   - 只能做**“快、带阻尼、2~4 帧到位”的姿态切换**（这正好是 §3.1 A1 说的“作为 target 直接下达”的那种短脉冲反射）。
6. **尺度的两面性。** 25 cm / 800 g 意味着转动惯量小、舵机相对力矩大 —— **姿态切换能做得很快**（有利）；但也意味着**一旦开始翻滚，整机会迅速转过去**、且落地高度低、留给“连续接触”的时间和空间都很少（不利）。

> **结论（这条很重要）：Microduck 的“姿势变换”在物理上不可能做成跑酷式的完整滚翻。它能做的是它的“微型等价物”——用腿主动制造首次接触 + 快速收头 + 让躯干/背部先接地，把“单点瞬时砸头”变成“多点连续倒伏”。**

#### 3.5.4 落到 Microduck 的可实施算法（四步）

下面四步都可作为现有 `FallPredictor` → `LimpFall` 骨架的扩展，不新增绕过 `Safety` 的写路径（见 §3.4 与 §5）。

**第 ① 步：方向与能量估计（复用现有信号，不新增硬件）**
- **跌倒方向**：重力水平分量 `[gx, gy]` 判前/后/左/右；头部 IMU 角速度（`head_imu.stream`）判是否带**自旋**（yaw 方向翻滚）。
- **落地能量**：头部 ToF（`tof.frame`）给地面距离，结合姿态外推估**落地速度 v̂ 与剩余时间 TTC**，据此把保护力度**分档**（低摔轻保护、高摔强保护，避免“为防摔而绊倒自己”）。
- 输出：一个 `{方向, 自旋, v̂, TTC}` 的结构化描述，喂给第 ② 步。

**第 ② 步：着地目标选择表（把“受身”写成人能读懂的决策表）**

| 跌倒方向 | 头部动作（保护头部） | 腿部动作（制造首次接触 / 管理动量） | 目标着地顺序 |
|---|---|---|---|
| **前倾** | 收颌（`neck_pitch`/`head_pitch` 向极限）+ `head_yaw` 把喙转开 | 屈膝、腿前伸先接地，髋后坐 | 脚/膝 → 髋/腹 → 头壳（已收拢） |
| **后倾** | 收颌压低头部、缩短头对颈的力矩臂 | 腿后撤，让臀/背先接地 | 臀/背 → 躯干 → 头壳 |
| **左/右侧倾** | `head_roll` 用满 ±25°（能做多少做多少）+ `head_yaw` 配合 | 同侧腿快速伸出接地、对侧腿跟随 | 同侧脚 → 躯干侧 → 头壳 |
| **带自旋** | 沿用主方向策略 | 优先**消旋**：`hip_yaw`/`ankle` 反向发力抑制 yaw 旋转 | 先消旋，再按主方向 |

> 这张表就是 §3.1 A1 的具体化，也是阶段 5 里 RL 奖励塑形的**先验**（见第 ④ 步）。

**第 ③ 步：姿态变换的执行（在 3~8 个 tick 内完成）**
- **腿部优先**：腿是唯一能改变整机角动量/质心的执行器 → 先动腿（接地 + 减速），再动头（让开）。
- **头部用短脉冲到位**：不做插值，直接以“高增益 + 阻尼”在 2~4 帧内进入保护姿态（`head_roll` 受限，就依赖 `neck_pitch`/`head_yaw`）。
- **力矩上限**：给颈部 4 个舵机设 `goal current` 上限（见 §3.2 第 3 条）——保护动作本身不能把颈部舵机打坏（§1.2 已证明“顶住”不可行）。

**第 ④ 步：接触时序与能量分散（Tripod Fall / EDP 的微型版）**
- **顺序目标**：脚/膝（腿）→ 髋/躯干 → 头壳（最后，且已收拢）。**让“头最后着地”**，把腿/躯干当成它的“手臂减震器”。
- **能量整形**：故意**不让腿一次伸直**（保留部分势能），让一部分能量在“腿接触 → 屈曲 → 躯干接触”的过程中被逐步耗散——这正是 Tripod Fall 的“只让一部分势能转成动能”与 EDP 的“沿多接触点分散能量”在 Microduck 尺度上的等价物。
- **滚动（受限）**：如果头壳几何允许，侧倒时可尝试**部分滚动到背部**（相当于 EDP 的接触沿体表推进）；但不要设计成完整滚翻（第 3.5.3 条 6 已说明）。

#### 3.5.5 与学习式策略的分工（规则式 vs 学习式）

- **规则式（本节的 ①~④）**：确定、可解释、可在现有 50 Hz 骨架里实现；但在“几十毫秒”的窗口里，人工设计的姿态很难最优，且难覆盖所有方向 × 高度的组合。
- **学习式（阶段 5）**：在 `microduck_rl` 里用 MuJoCo + PPO 训练一个**保护性倒伏策略**，把本节第 ② 步的决策表**当作奖励塑形的先验**（类似 [SafeFall](https://www.newx.sg/paper/detail/319c5992-966f-11f1-b84c-00163e10baa7) 的“结构脆弱性感知奖励”），让网络自己发现“收头 + 腿先接地 + 头最后着地”的动作，而不是硬编码姿态。产出 ONNX 后经 policy-channel（`robotctl policy load`）作为**独立 slot** 安装，复用 `LimpFall` 的“接管—回正—交回”机制。

#### 3.5.6 可行性诚实评估（哪些研究能迁移、以什么形式）

| 研究/算法 | 它依赖什么 | Microduck 能否用 | 迁移形式 |
|---|---|---|---|
| Ukemi（方向识别 + 落点选择） | 角动量判定方向 + 姿态控制 | **可以** | 用 `FallPredictor` 的 `ġz` + 重力水平分量实现方向识别；落点选择见第 ② 步表 |
| Tripod Fall | 双手 + 单脚的三接触 | **否（无臂）** | 改造成“双脚/膝 + 躯干”的多次连续接触 |
| 全身轨迹优化最小化冲量 | 全身动力学模型 + 离线求解器 | **可以（离线）** | 用 MuJoCo 离线算保护姿态序列，在线查表 |
| Online Rolling / EDP | 多接触点 + 连续滚动 | **部分** | 只做“接触顺序 + 能量分档”，不做完整滚翻 |
| Self-Protective “三角”结构 | 上臂 | **否** | 用腿/躯干替代“三角”的支撑作用 |
| FIRM / SafeFall（RL 保护） | 预测 + 解耦控制 + 训练栈 | **可以** | 与 Microduck 架构契合度最高，走 `microduck_rl` + policy-channel |

---

## 4. 分阶段实施路线

> 原则（借自 `AGENTS.md` / 用户规则）：小步重构、每步可测、频繁提交；结构保护先行（软件会失效，结构不会）。

**阶段 0 — 基线与测量方案（必须先做 · 详细展开）**

> 目标：在改动任何代码/结构之前，得到**可复现、可对比**的基线。没有基线，任何“降低 X%”都是空话；有了基线，后面每个阶段才有客观验收标准。下面是这一阶段的可执行版本。

**(a) 测什么：物理量 → 信号源**

| 物理量 | 定义 | 信号源（仓库内） | 采样 |
|---|---|---|---|
| 头部链合成加速度 | `a_head(t)` | `head_imu.stream`（芯片内融合，[head_imu.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd/src/head_imu.rs)） | 60 Hz |
| 头部角速度/角加速度 | `ω_head`、`α_head` | 同上 | 60 Hz |
| 颈部/头部舵机电流、负载 | `I_neck` | 总线 `present_current`（与位置/速度同块读取） | 50 Hz |
| 关节跟踪误差 | `q_cmd − q_meas` | [robotctl monitor](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotctl)（`joints` / `targets`） | 50 Hz |
| 整机姿态时序 | `q_base(t)` | 躯干 IMU 四元数 | 50 Hz |
| 离地高度 | `h(t)` | 头部 ToF [tof.frame](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/tof/src/lib.rs) | 15 Hz |
| 事件计数 | falls / soft landings / peak current / spin | [vetbook.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd/src/vetbook.rs)（`life.record`） | 事件级 |
| 着地时序与部位 | 接触事件序列 | IMU/电流突变推断 + **外部高速摄像（≥240 fps）** | 相机帧率 |

**(b) 摔倒场景矩阵（每一格重复 ≥5 次）**

- **方向**：前倾 / 后倾 / 左倾 / 右倾 / 后仰 / 带自旋。
- **高度**：平地（行走中绊倒）/ 桌面约 40 cm / 台阶。
- **方式**：主动行走中绊倒 / 静止被推 / 从极限倾斜位松手。
- 每次记录**峰值的中位数与分位数**（不要只用均值——摔倒冲击的方差很大，均值会被单次极端值带偏）。

**(c) 记录与时钟对齐**

- 统一用 `robotctl monitor`（或直接订阅）落 **NDJSON**，带机器人单调时钟戳。
- 高速摄像记录帧时间码；用一次明确可辨识的**冲击事件**做**时间对齐锚点**。
- **复现性**：把触发脚本化（同一初始姿态、同一推法），否则每次摔的都不一样，基线不可比。

**(d) 指标定义（都能从上面数据算出来）**

- 头部峰值合成加速度 `a_max`；头部峰值角加速度 `α_max`。
- **HIC 类比（头部损伤准则）**：`HIC = (t2−t1)·[ (1/(t2−t1))·∫ a dt ]^2.5`，取使 HIC 最大的时间窗——把“一条加速度曲线”压成**一个损伤标量**，最适合做 A/B 对比。
- 颈部峰值扭矩 `τ_max` ≈ `I_neck·α_head`，或用 `I_neck × 扭矩常数` 估计（用于判断是否超过舵机极限，对照 §1.2）。
- 能量吸收 `E_abs` = 冲击前后整机动能差。
- **着地顺序**（尤其“头壳是否是第一个着地的刚性部位”）。
- **二次冲击次数**（落地后弹跳/翻滚的再次接触）。

**(e) 产出物**

1. 基线数据集（每个场景 × 每次，结构化 CSV/NDJSON）。
2. 一页“**基线卡片**”：每场景的 `a_max` / `α_max` / HIC / 着地顺序。
3. **回归夹具**：把基线做成**自动化对比**（在 [scripts/duck-sim](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/scripts/duck-sim) 里重放同一摔倒，断言新策略的 `a_max`/HIC 低于基线的 X%）——这样后面每个阶段都有客观验收，而不是“感觉好点了”。

**(f) 工具与注意事项**

- 现有工具：[scripts/duck-sim](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/scripts/duck-sim)、`robotctl monitor`、`vetbook`。
- 安全：防摔垫、限时、手边备断电开关。
- 已知局限：头部 IMU 是 60 Hz，可能**低估真实峰值**（带宽不足）——高摔场景必须结合高速摄像交叉验证，不要只信 IMU 的峰值。

**阶段 1 — 结构被动保护（风险最低、收益确定）**
- 头部护垫 + 相机/ToF 护圈；颈部机械限位与缓冲；颈部舵机扭矩限制/牺牲件。
- 验证：重复跌落，对比阶段 0 基线的头部峰值加速度与颈部舵机损伤。

**阶段 2 — 软件收头反射 + 分层顺从（复用现有骨架）**
- 扩展 `FallPredictor` 输出方向；`LimpFall` 增加 `Brace`；分层 gain。
- 验证：单元测试（沿用 `fall.rs` 现有的强测试风格）+ 台架/仿真（`scripts/duck-sim` 跑真实 daemon 对 MuJoCo 身体）。

**阶段 3 — 头部 IMU 二次冲击看门狗**
- 消费 `head_imu.stream`；落地后二次冲击立即锁定让步。
- 验证：仿真里制造“落地后弹跳/翻滚”场景。

**阶段 4 — ToF 高度自适应**
- `robotd` 订阅 `tof.frame`（或特征）；按离地高度分档保护力度。
- 验证：桌面跌落 vs 平地绊倒两档对比。

**阶段 5 — 学习式保护策略（长期）**
- `microduck_rl` 训练保护策略；通过 policy-channel 安装；解决 `busy()` 门控约束后挂接。
- 验证：与阶段 2/3 的规则式保护对比峰值冲击。

---

## 5. 关键风险与设计约束

1. **假阳性 = 机器人自己造成摔倒。** 现有文档反复强调：“每一个误报都是一次由机器人*造成的*摔倒，严格比摔得更硬更糟。”收头反射必须极其保守，并严格保留 `arm`/`reset` 语义。
2. **颈部舵机是瓶颈。** 保护动作**不能给颈部加过大扭矩**；“软、让开、结构耗能”是主手段，“猛动去接”是危险手段（量化见 §1.2）。
3. **保护动作会改变落地姿态**，可能影响后续 `get back up` 策略——现有交接假设“落在一个已知位姿上”，新增的收头相位要保证回正起点仍然已知。
4. **不能新增绕过 `Safety` 的写路径。** `Safety` 独占写句柄是重要的架构不变量；保护必须作为普通 target/gain 经 `apply` 下达。
5. **`busy()` 门控问题。** 现存“技能运行时关闭摔倒反射”的行为要显式决策，否则保护反射会被技能悄悄关掉。
6. **版本与交付约束。** 按 `AGENTS.md`：不要围绕版本差异做设计；涉及 API/参数变更应 bump `API_VERSION` 并说明安装后果；改动要真正随 release 到达机器人（`main` 修好 ≠ 机器人修好）。
7. **文档归属。** 按 `AGENTS.md`，一个机制只有一个文档拥有者；本方案的机制应新增/归入一个设计文档（`docs/design/` 下），其他页面只做一句话引用。

---

## 6. 参考文献

**Microduck 仓库内部**
- `README.md`、`docs/design/architecture.md` §1.1/§6、`docs/design/policy-channel-design.md` §10.2
- [fall.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-control/src/fall.rs)、[safety.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-control/src/safety.rs)
- [robotd/src/main.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd/src/main.rs)、[head_imu.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd/src/head_imu.rs)、[posture.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd/src/posture.rs)
- [tof/src/lib.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/tof/src/lib.rs)、[kinematics/assets/alpha/robot_walk.xml](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/kinematics/assets/alpha/robot_walk.xml)
- [deploy/robotd.toml](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/deploy/robotd.toml)（safety 段注释）、[robotd-params/src/lib.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/robotd-params/src/lib.rs)

**外部研究**
1. Fujiwara et al., *UKEMI: Falling Motion Control to Minimize Damage to Biped Humanoid Robot* — http://www.cs.cmu.edu/~cga/falling/ukemi.pdf
2. Yun & Goswami, *Tripod Fall: Concept and Experiments of a Novel Approach to Humanoid Robot Fall Damage Reduction* (ICRA 2014) — http://www.ambarish.com/paper/Yun_Goswami_ICRA2014_Tripod_Humanoid_Fall.pdf
3. *Whole-Body Trajectory Optimization for Humanoid Falling* (CMU, ACC 2012) — http://www.cs.cmu.edu/~cga/ewhitman/webContent/ACC2012_2_paper.pdf
4. Subburaman et al., *A Survey on Control of Humanoid Fall Over* (RAS 2023) — https://elib.dlr.de/195344/4/J17__Subburaman__2023__RAS.pdf
5. *Online Multi-Phase Trajectory Generation for Compliant Landing Control of Quadruped Robots* — https://ieeexplore-qa.ieee.org/document/10406414/
6. *Using Compliant Leg Design for Impact Attenuation of Airdrop Landings of Quadruped Robots* — https://i636f6f706572o656475z.oszar.com/.../DLuchten_Paper.pdf
7. *Soft Responsive Materials Enhance Humanoid Safety* — https://arxiv.org/html/2601.02857
8. *Discovering Self-Protective Falling Policy for Humanoid Robot via Deep RL* — https://arxiv.org/pdf/2512.01336
9. *Guardians as You Fall (GYF): Active Mode Transition for Safe Falling* — https://arxiv.org/pdf/2310.04828v1.pdf
10. *FIRM: Unified Humanoid Fall-Safety Policy from a Few Demonstrations* — https://arxiv.org/pdf/2511.07407
11. *Multi-Momentum Observer Contact Estimation for Bipedal Robots* — https://arxiv.org/pdf/2412.03462
12. Siemens/Wandercraft, 单气囊 vs 多气囊仿真论证 — https://resources.sw.siemens.com/en-US/case-study-wandercraft/
13. *How Do Humanoid Robots Recover From a Fall?*（碰撞几何设计） — https://www.smashingrobotics.com/how-do-humanoid-robots-recover-from-a-fall/
14. *Adaptive Compliance Control in Wearable Exoskeletons for Dynamic Fall Impact Mitigation* — https://freederia.com/adaptive-compliance-control-in-wearable-exoskeletons-for-dynamic-fall-impact-mitigation/
15. *Online Rolling Motion Generation for Humanoid Falls Based on Active Energy Control Concepts*（Humanoids 2018；接触位置 + 攻击角 = 临界滚动参数 CRP，能量分布多面体 EDP） — http://crlab.cs.columbia.edu/humanoids_2018_proceedings/media/files/0152.pdf
16. *A Systematic Review on the Biomechanics of Breakfall Technique (Ukemi) in Relation to Injury in Judo*（MDPI） — https://www.mdpi.com/1660-4601/19/7/4259
17. *SafeFall: Learning Protective Control for Humanoid Robots*（GRU 预测 + RL 保护控制，与主控解耦，结构脆弱性感知奖励） — https://www.newx.sg/paper/detail/319c5992-966f-11f1-b84c-00163e10baa7
18. *Breakfalls in Seven Systems*（Hyakusha 训练资料整理；受身 vs 不受身的头部运动学量对比，**仅作量级参考**） — https://www.hyakusha.com/en/training/breakfall-comparison
