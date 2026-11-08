# microduck-ukemi 实操流程

从零到「仿真训练 → 仿真闭环验证 → 实机部署」的完整操作序列。

配套阅读：

- `microduck摔倒保护项目开发方案.md` —— 实现路径、奖励设计、开源流程的依据
- `microduck工程架构与开发参考.md` —— 工程构成与开发逻辑

所有命令以 **WSL2 Ubuntu** 为前提。Windows 原生不可行（unix socket、`duck-sim`、`cargo test` 都需要 Linux）。

---

## 0. 全景：三个仓库，各自角色

| 仓库 | 在你的工作里扮演什么 | 必须？ |
|---|---|---|
| `microduck_rl`（pollen-robotics） | 训练框架（mjlab/MuJoCo Warp/PPO）+ `duck-body`（MuJoCo 身体服务器）+ BAM 执行器模型 + MJCF 模型 | ✅ 硬依赖 |
| `microduck`（你的 fork） | 真实运行时：`robotd --sim`、`scripts/duck-sim`、`robotctl` | ✅ 闭环验证 + 实机部署 |
| `microduck-ukemi`（你新建） | 你的环境配置 + 奖励 + 任务注册 | ✅ 你的产出 |

数据流：

```
microduck-ukemi (env + reward)  ─┐
                                 ├─→ uv run train ─→ checkpoint ─→ export.py ─→ ONNX
microduck_rl (mjlab/BAM/MJCF)  ──┘                                                 │
                                                                                   ↓
      实机：  robotctl policy add / robot do   ←──  HF Hub  ←──  uv run publish
      仿真：  duck-body ⇄ robotd --sim ⇄ robotctl        （与实机同一份 daemon 代码）
```

关键点：**实机和仿真用的是同一份 daemon 代码**。在仿真鸭子上跑通 `robotctl policy add` + `robot do`，就等于在真机上跑通了策略部署这条链。

---

## 1. 一次性环境搭建

### 1.1 依赖

```bash
# ① Rust 侧构建所需（CONTRIBUTING 列出的四个 C 库）
sudo apt update && sudo apt install -y \
  libudev-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev

# ② 容器形态仿真用（只有 duck-sim boot N 需要）
sudo apt install -y systemd-container mmdebstrap

# ③ Rust —— 仓库没有 rust-toolchain.toml，可用区间 1.89 – 1.94.1
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# ④ uv
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 1.2 克隆

```bash
mkdir -p ~/dev && cd ~/dev

git clone https://github.com/GREENNREDDDDB/microduck.git        # 你的 fork
git clone https://github.com/pollen-robotics/microduck_rl.git

cd microduck_rl && git checkout develop && cd ..                # 相机等能力在 develop
```

得到 `~/dev/microduck`、`~/dev/microduck_rl`，稍后再加上 `~/dev/microduck-ukemi`。

WSL 里 fresh clone（而不是拷 Windows 目录）是必要的：Windows 检出的符号链接、可执行位、行尾都是坏的。

### 1.3 验证

```bash
cd ~/dev/microduck && cargo test --workspace          # 期望全绿（约 942 个测试）
cd ~/dev/microduck_rl && uv sync && uv run list-envs   # 列出任务注册表
```

> ⚠️ **WSL2 内存**：4096 个环境的 PPO 很吃内存，WSL2 默认只给宿主的一半。在 `%UserProfile%\.wslconfig` 里加 `memory=32GB`（按机器调整），否则长训会中途 OOM。

---

## 2. 建立项目骨架

```bash
cd ~/dev && mkdir microduck-ukemi && cd microduck-ukemi
mkdir -p src/microduck_ukemi/tasks scripts tests
```

### 2.1 `pyproject.toml`

```toml
[project]
name = "microduck-ukemi"
version = "0.0.1"
requires-python = ">=3.12,<3.13"
dependencies = ["mjlab-microduck", "mjlab==1.3.0"]

[tool.uv.sources]
mjlab-microduck = { git = "https://github.com/pollen-robotics/microduck_rl", branch = "develop" }

[project.entry-points."mjlab.tasks"]
microduck_ukemi = "microduck_ukemi.tasks"
```

**两个必须知道的约束：**

1. **不要声明 `[project.scripts] train`**（也不要在自己仓库里放 `train` 脚本）。`train` 与 mjlab 的同名脚本是 last-writer-wins，`microduck_rl/pyproject.toml` 里有一段注释专门记录了这个坑：删掉那行会导致 venv 里**根本没有** `train`。
2. **但你仍然可以直接 `uv run train`**：`microduck_rl` 作为依赖被装进来后，它的 `train` / `publish` / `duck-body` 控制台脚本就进了你的 venv。这就是"即插即用"的落地方式——**你只提供任务，不提供工具**。

### 2.2 `src/microduck_ukemi/tasks/__init__.py`

```python
from mjlab.tasks.registry import register_mjlab_task

# 复用官方 runner：它处理了对称性 cfg 的序列化细节（见 microduck_rl 里的同名类）
from mjlab_microduck.tasks import MicroduckOnPolicyRunner

from .ukemi_env_cfg import make_ukemi_env_cfg, MicroduckUkemiRlCfg

register_mjlab_task(
    task_id="Mjlab-Ukemi-Flat-MicroDuck",
    env_cfg=make_ukemi_env_cfg(),
    play_env_cfg=make_ukemi_env_cfg(play=True),
    rl_cfg=MicroduckUkemiRlCfg,
    runner_cls=MicroduckOnPolicyRunner,
)
```

导入 `mjlab_microduck.tasks` 会顺带注册官方的那十几个任务，这是期望行为（同一个注册表里共存）。

### 2.3 `scripts/export.py`（6 行，照抄）

```python
"""导出 checkpoint → ONNX：包装 mjlab_microduck.export（normalizer 必须由它烘进图里）。"""
from mjlab_microduck.export import main

if __name__ == "__main__":
    main()
```

### 2.4 `scripts/infer_policy.py`

从 `microduck_rl/scripts/infer_policy.py` **拷贝**一份（Apache-2.0，保留来源说明即可），然后按 §4.5 加一个 `--ukemi` 槽位。

---

## 3. 在哪里设计策略

三处文件，按改动频率从低到高。

### 3.1 环境配置：`tasks/ukemi_env_cfg.py` —— 起点是 roulade，不是 velocity

模板选择（按贴近程度排序）：

| 模板 | 为什么 |
|---|---|
| `microduck_roulade_env_cfg.py` | **最贴近**：前滚翻过顶、落回脚上，是同一类"大角速度 + 大冲击"的动力学事件；文件头 docstring 记录了五轮 run 的完整教训（run 1 学成暴力甩尾 → support gate → reverse curriculum 用中段 spawn 修好后半程） |
| `microduck_standup_env_cfg.py` | "从仰卧起"的完整解，受身落地的后半程就是这个问题 |
| `microduck_velstand_env_cfg.py` | **行走 + 摔倒 + 恢复在同一策略里**，是"摔倒"这件事在官方仓库的现成先例 |

必须一并保留的骨架（照抄 roulade 段的 `ENABLE_*` 与范围）：BAM 执行器 + 摩擦 DR、COM 与头 COM DR、质量/惯量/armature DR、encoder bias DR、**IMU 安装错位 DR**。

### 3.2 奖励与事件：`tasks/ukemi_mdp.py`（你自己的文件）

可直接 `import` 复用的（都在 `mjlab_microduck.tasks.mdp`）：

| 函数 | 用途 |
|---|---|
| `_fallen_mask(z, tilt)` | 判定"已倒地"（trunk 高度低于阈值 **或** 倾角超阈值），用于 gate |
| `fallen_state_penalty` | 躺地惩罚，现成的措辞与阈值 |
| `_servo_joint_ids` / `_servo_joint_pos` / `_servo_joint_vel` | **必须用**：roller/backlash 模型下被动关节会插进下标，硬编码必错 |
| `randomize_delayed_actuator_gains` | 执行器延迟 DR |
| `reset_from_supine` / `reset_roulade_state` | 初始姿态与反向课程的 spawn |
| `projected_gravity_imu_misaligned` / `base_ang_vel_imu_misaligned` | 带 IMU 安装错位的 obs |
| `raw_accelerometer` | 含冲击的加速度（**未进 61 维 actor obs**，见 §3.3） |

奖励项、损伤代理、gate 的具体设计见 `microduck摔倒保护项目开发方案.md` §2.3，这里不重复。

### 3.3 观测：第一版**不要扩** 61 维

61 维是跨仓库契约，同时钉在三处：`docs/policy-manifest.md` 的 `obs_len`、`duck-ipc-proto`、`robotd-params`，且 `publish` 上传前会校验 `[1,61] → [1,14]`。扩维 = 改契约 = 要 bump `model_api` 并写明安装后果。

好消息：这 61 维里的 6 维（gyro + projected_gravity）**就是 IMU**；而 `imu_accel`、`subtreeangmom` 已在 MJCF 里建模、`mdp.py` 里可读，只是没进 actor obs——所以"加观测"是一条随时可走的升级路径，不是从零搭。

一个现成的旋钮：`infer_policy.py` 已有 `--raw-accelerometer`（"Use raw accelerometer instead of projected gravity"），说明这条替换路径在部署预演里是通的。

### 3.4 测试：`tests/test_ukemi_cfg.py`

照 `microduck_rl/tests/test_*_cfg.py` 写：关节下标在真实模型上能解析、奖励权重符号正确（**惩罚项在 wandb 里必须 ≤ 0**）、gate 在预期处开合。CPU-only：

```bash
uv run --with pytest pytest tests/
```

---

## 4. 训练与仿真验证：六道门，逐级过

### 4.1 门 1 — 注册检查

```bash
cd ~/dev/microduck-ukemi && uv sync
uv run list-envs | grep -i ukemi     # 必须看到 Mjlab-Ukemi-Flat-MicroDuck
```

### 4.2 门 2 — 冒烟（64 envs × 5 iters）

```bash
uv run train Mjlab-Ukemi-Flat-MicroDuck --env.scene.num-envs 64 --agent.max_iterations 5
```

**永远先跑这个。** 官方口径：5 iteration / 64 env 能抓住约 95% 的配置错误，成本几毛钱。它同时验证：obs 是 61 维、每个奖励项都算得出来、没有 NaN、ONNX 能导出。

### 4.3 门 3 — 长训（4096 envs）

```bash
uv run train Mjlab-Ukemi-Flat-MicroDuck --env.scene.num-envs 4096

# 断点续训
uv run train Mjlab-Ukemi-Flat-MicroDuck --env.scene.num-envs 4096 \
    --agent.run-name resume --agent.load-checkpoint model_29999.pt --agent.resume True
```

- 预算参考：简单 episodic 技巧 ≈ 1000 iters @4096 envs；步态与重课程化的恢复类要 4000–6000。**受身属于后者。**
- 没有 GPU：任意 train 命令加 `--hf-jobs`（走 Hugging Face Jobs）。
- wandb 项目 `mjlab_microduck`，日志在 `logs/<experiment_name>/`。

**读训练曲线的口径（官方列的判据）：**

- **主任务项**在涨，而不是总奖励。总奖励可以只靠正则项涨而技巧根本没发生。
- 每一项 `Episode_Reward/<penalty>` ≤ 0。**任何惩罚项读数是正的，就是权重符号反了，策略在"刷"这个违规**（mdp.py 里有两类惩罚风格：mjlab 原生 cost 返回 ≥0 配负权重；自否定型 `*_penalty` / `*_l1` 返回 ≤0 配**正**权重）。
- `Episode_Reward/<term>` 记录的是**加权后**的值——权重 0 的项永远读 0，要对着权重曲线解释。
- 某项恰好卡在课程阶段边界上突然下跌 → 节奏错了，把该阶段拉长或往后挪，**不要往前挪**。

### 4.4 门 4 — 看视频 + 导出

```bash
uv run play Mjlab-Ukemi-Flat-MicroDuck --wandb-run-path <entity/project/run_id>
uv run scripts/export.py Mjlab-Ukemi-Flat-MicroDuck --wandb-run-path <entity/project/run_id>
```

- obs 归一化是开着的，`play` 反正也会应用它，所以在仿真里看不出问题——**必须用 `scripts/export.py` 导出**，绝不手工转换 checkpoint，否则真机上策略看到的是未归一化的 obs。
- 官方警告："sim metrics can pass while the video fails the human eye"——**两样都要看**，并且要检查是哪个 geom、哪根轴在接触。

### 4.5 门 5 — CPU 部署预演（还不用起 daemon）

这一步排练"部署时到底发生什么"：同一个 61 维契约下的策略热插拔、命令槽写入、BAM M6 执行器（与训练同一套模型）。

`infer_policy.py` 里**没有 `--ukemi` 槽位**（现有：`--walking` / `--standing` / `--sitstand` / `--roulade` / `--ground-pick` / `--kick-left` / `--kick-right`）。两个办法：

- **零改动**：受身是 episodic，借 roulade 的槽位（`R` 键 + `--roulade-duration N`）：

```bash
uv run scripts/infer_policy.py --walking walk.onnx --roulade ukemi.onnx \
    --roulade-duration 2.5 --new-cmd-obs --debug
```

- **加一个槽位**（推荐做，因为实机上你就是这么用的）：在拷过来的 `infer_policy.py` 里照 `--roulade` 抄一份 `--ukemi`，绑一个键、一个 duration。

其它有用的开关：`--debug` 打印 obs/action、`--save-csv` 存对比数据、`--record` 存 pickle、`--no-bam` 退回 XML PD 执行器、`--vin` / `--vin-drop-gain` / `--kp-fw` 把训练 DR 范围钉到单值。

### 4.6 门 6 — 真闭环（daemon 在环，仍然不需要硬件）

这一步才验证**运行时**行为：安全层、跌倒判定、`limp_fall`、命令互斥、50 Hz 调度。这些不在"环境"里，而在"运行时"里，MuJoCo 那条环上没有。

```bash
export DUCK_SIM_RL=~/dev/microduck_rl     # 默认是 ~/Pollen/microduck_rl，必须改

cd ~/dev/microduck
scripts/duck-sim                 # 开窗、鸭子从 SIT 站起
scripts/duck-sim status          # 健康 + 是否站立
scripts/duck-sim realtime        # ★ 实时比，必须 ≥ 1.0
scripts/duck-sim ctl health
scripts/duck-sim monitor         # 关节 / IMU / ToF / 摇杆
scripts/duck-sim drive           # 走 8 秒（参数 vx vyaw，默认 0.15 0）
scripts/duck-sim down
```

**把你的策略装到仿真鸭子上**（`publish` 默认建**私有**仓库，所以这条路闭环、不留公开痕迹）：

```bash
# ① 让仿真鸭子登录你的 HF 账号（否则读不到私有仓库）
scripts/duck-sim ctl account login      # 打开 hf.co/oauth/device 输码

# ② 发布（先 --dry-run 看会上传什么，不碰 Hub）
cd ~/dev/microduck-ukemi
uv run publish --task Mjlab-Ukemi-Flat-MicroDuck \
    --wandb-run-path <entity/project/run_id> --checkpoint 5000 \
    --repo <user>/microduck-ukemi --kind episodic --duration-s 2.5 \
    --description "Tucks and lands on its back instead of its head." --dry-run

# ③ 装到仿真鸭子并触发
cd ~/dev/microduck
scripts/duck-sim ctl policy add ukemi <user>/microduck-ukemi
scripts/duck-sim ctl robot do ukemi
```

**两条实时性铁律：**

- daemons 的循环走**墙钟**。`duck-sim realtime` 低于 1.0× 时，策略在驱动一个"动得比它以为的慢"的机器人，**它平衡不了**——这不是策略的 bug。
- 鸭子多了或相机多了，结果不是"变慢"而是**在 45 Hz 门控上转 unhealthy**，容器形态下更新器还会开始回滚。依次减鸭子、减相机、`DUCK_SIM_VIEWER=0`。

需要验证更新器 / 健康门控 / 回滚 / 重启顺序时，才用容器形态：

```bash
sudo scripts/duck-sim boot 2          # 每个鸭子一个 systemd-nspawn 容器
scripts/duck-sim shell duck-a         # 进去手工敲 robotctl / journalctl
```

另外两条容易搞混的：`--sim` ≠ `--fake`（后者没有物理）；`up` 形态看不见的一类 bug 是"某个 daemon 的 `User=` 不存在，导致 unit 从未启动"。

---

## 5. 实机部署

### 5.1 策略级 —— 你要的"方便快捷"

```bash
cd ~/dev/microduck-ukemi
uv run publish --task Mjlab-Ukemi-Flat-MicroDuck \
    --wandb-run-path <entity/project/run_id> --checkpoint 5000 \
    --repo <user>/microduck-ukemi --kind episodic --duration-s 2.5 \
    --description "Tucks and lands on its back instead of its head."
```

可选：`--no-private` 公开、`--tag v1` 给 revision 打标、`--video media/ukemi.mp4` 上传 rollout 视频（Hub 的 replay widget 会播）、`--base-model <owner>/<repo>` 标注衍生关系、`--dry-run` 只看不上传。

`publish` 在上传前会：校验图是 `[1,61] → [1,14]`、用合理输入跑一遍并拒绝 NaN 或常量输出、从 git/wandb 填 `training` 溯源块、拒绝覆盖已存在的 `.onnx`（除非 `--force`）。仓库默认建为**私有**。

板上：

```bash
sudo robotctl policy add ukemi <user>/microduck-ukemi              # episodic：长度来自 manifest
sudo robotctl policy add ukemi <repo> --hold 5 --command 1,1,0     # 持姿型：你定长度与命令
robotctl robot do ukemi
robotctl policy list                                               # 确认已装
```

**先在仿真鸭子上跑通同一串命令再上真机**——daemon 侧是同一份代码，这就是 §4.6 的价值。

### 5.2 运行时级 —— 只有要改 Rust 时才用

```bash
cd ~/dev/microduck
scripts/dev-push.sh --dry-run           # 无硬件时：验证到 swap 之前
scripts/dev-push.sh --name duck-c51b    # 有板时：约一分钟上板，走真实签名/哈希/健康门控
```

前提：板是 dev 板、`~/.duck-keys/team.dev.key`、`cargo install cargo-zigbuild --locked` + `zig`（或每次加 `--docker`）。

### 5.3 真机专属确认项（仿真测不到）

- **IMU 冲击饱和**：仿真里冲击是真实物理量；真机上 SM 级跌倒可能把 IMU 打饱和（信号削顶）。必须单独确认。
- **`limp_fall` 协同**：`robotd` 预测约 0.3 s 后要摔时会提前软化关节。你的策略和它谁先动、会不会互相打架——只在 `robotd --sim` 或真机上能看出来。
- **舵机力矩/热/电流的真实值**：仿真的电流是 `abs(torque) × 100`，注释自己写明 "a fiction with a unit"。

---

## 6. 坑清单

| 坑 | 后果 | 为什么 |
|---|---|---|
| 手工转换 checkpoint 成 ONNX | 部署时策略看到未归一化的 obs | obs 归一化开着，`play` 会掩盖；只有 `export.py` 把 normalizer 烘进图 |
| 自否定型惩罚给了负权重 | 双重取负 → 变成"奖励违规"，策略开始刷（臀行、摔坐） | `Episode_Reward/<penalty>` 必须 ≤ 0 |
| "到达 X 就给分" | jackpot，买来任意暴力 | 用 potential-based：付 Δprogress，保持付 0 |
| 把正向奖励 gate 在"倒地"上 | 策略趴进最便宜的合格姿态刷分 | 用 `Δcos(tilt)` 一类 shaping |
| 动态任务上开大 smoothness/正则 | 探索期"什么都不做"赢，技巧永远学不出来 | 课程从 ~0 起，技巧发现后再引入 |
| 硬编码关节下标 | roller/backlash 模型下选错关节 | 用 `_servo_joint_ids` 等 helper |
| 独立 env cfg 漏了 `expand_bam_friction_fields` | BAM 下 `dof_frictionloss` 为 0，摩擦 DR 静默失效 | BAM 的摩擦由 actuator 自己算 |
| 直接改 `env.cfg` 上的 term | 静默 no-op | manager 在 init 时 deepcopy 了 cfg，要用 `env.event_manager.get_term_cfg(...)` |
| 命令槽永远给 0 | 该输入的权重永远死，后续课程也救不回来 | 每槽保留一个小非零采样范围 |
| 在自己的 `pyproject.toml` 声明 `train` 脚本 | 同名控制台脚本互相覆盖，venv 里可能没有 `train` | `microduck_rl/pyproject.toml` 的注释 |
| 冷启动 `uv sync` 之外手工装的包 | 依赖它的一切（含 HF Jobs）在远程会死 | 新 `uv sync` 才是 ground truth |
| 把仿真里测到的冲击当"真机也会这样" | 真机 IMU 饱和 / 舵机真实限流 | 仿真不建模 IMU 量程与舵机热 |

---

## 7. 最小可行路径（今天就能开始的顺序）

```bash
# 1) 环境（§1）：WSL2 + 依赖 + Rust + uv + 两个仓库 + cargo test 全绿
# 2) 骨架（§2）：microduck-ukemi + pyproject + tasks/__init__.py + scripts/export.py
# 3) 环境配置先直接复用 roulade，只改 episode 长度与 spawn —— 先让流水线转起来
# 4) 门 1→5：list-envs → 冒烟 → 短训 → play → export → infer_policy（借 --roulade 槽）
# 5) 流水线跑通后，再按 方案 §2.3 把奖励换成受身专用（这时才有基线可比）
# 6) 门 6：duck-sim + publish（私有）+ policy add / robot do
# 7) 有硬件后：同一串 policy 命令直接在真机上跑；要改 Rust 时用 dev-push.sh
```

**先让流水线转起来，再调奖励。** 这既是官方写给自己的第一条教训（"Measure before theorizing"），也是这个项目里已经写下的基线原则。
