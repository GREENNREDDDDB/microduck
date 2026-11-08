# Microduck 工程架构与开发参考

> 面向"要在这个仓库上做开发"的人。目标是回答三件事：**它由什么组成**、**开发者是怎么把它做出来的**、**怎么用和怎么测**。
> 依据全部来自仓库自身的权威文档与源码；每条关键结论都给了可点击的出处，便于你顺着读下去。

---

## 0. 怎么用这份文档

| 你的目的 | 读哪几节 |
|---|---|
| 先建立整体印象 | §1、§2.6（一张图） |
| 要改控制/策略/安全相关代码 | §2、§3.2、§5 |
| 要改更新器、配置、BLE、媒体 | §3.3、§3.4 |
| 要跑起来、要调试、要上板 | §4、§6.1 |
| 要基于本项目做独立项目（如摔倒保护） | §7、§8 |

**一条总的阅读原则**（来自仓库自身的规定）：每个机制只有一页文档"拥有"它，其它页只写一句并指过去；两页冲突时，不拥有该机制的那份是 bug；行为与设计文档冲突时，文档是 bug。见 [docs/README.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/README.md)。

---

## 1. 项目速览

**Microduck 是一台 25 cm 高、约 800 g 的开源双足机器人**，主控是 Rockchip RK3566（Radxa Zero 3W，另有 beta 板），15 个 Dynamixel XL330 舵机，50 Hz 控制环，带相机、8×8 ToF、双 IMU。许可证 Apache-2.0。

**这个仓库不是机器人本体，而是"鸭子的大脑"**——[README.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/README.md) 的第一句话就是这个意思。策略（走路、站立、捡拾、踢球、前滚翻）在另一个仓库 `microduck_rl` 里训练，训练好的 `.onnx` 通过 Hugging Face 发布，再由本仓库的运行时加载执行。

### 1.1 仓库的物理构成

```
microduck/
├── Cargo.toml            ← Rust workspace 总纲（成员、工具链上下限、四类 artifact pin）
├── CONTRIBUTING.md       ← 构建/测试/布局/约定/发布的唯一权威入口
├── AGENTS.md             ← 给 AI/新人的"最容易搞反的几件事"
├── src crates/           ← 26 个 crate（守护进程 / 库 / 工具，见 §3.1）
├── deploy/               ← 板上静态资产：robotd.toml 默认值、受信密钥、systemd override、音频 DTS
├── hooks/                ← postinstall / preinstall（更新时的钩子，见 §3.3）
├── scripts/              ← 30+ 个运维脚本（建板、交叉编译、推板、仿真、播种策略…）
├── docs/                 ← design/（机制所有权）· project/（路线与工程记录）· robot/（操作手册）· ideas/
├── .github/workflows/    ← 10 个 CI 工作流
└── microduck_rl/         ← 训练侧（MuJoCo + mjlab + PPO），作为"兄弟仓库"形式存在于此目录
```

### 1.2 技术栈与硬性约束

| 项 | 值 | 出处 |
|---|---|---|
| 语言 / 版本 | Rust，edition 2024，**下限 1.89**、上限 **1.94.1**（Yocto wrynose） | [Cargo.toml](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/Cargo.toml) |
| 当前版本 | `workspace.package.version = "0.18.9"` | 同上 |
| 目标平台 | `aarch64-unknown-linux-gnu`，glibc pin 到 **2.31** | [.cargo/config.toml](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/.cargo/config.toml) |
| 板级系统 | Armbian 26.2.x（Debian 13 Trixie 用户态），kernel 6.1.115 | [updater-design.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/updater-design.md) §5.6 |
| 协议版本 | `API_VERSION = 51` | [duck-ipc-proto/src/lib.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-ipc-proto/src/lib.rs#L525) |
| 关节表 | `JOINT_NAMES: [&str; 15]`（含 `mouth`；模型/策略层 14 个，`mouth` 被排除） | 同文件 L634 |
| obs / action | `[1,61] → [1,14]` | [robotd-design.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/robotd-design.md) §2.2 |

> **一个容易搞错的点**：协议层 15 个关节（`left_hip_yaw` … `mouth`），但**策略观测/动作都是 14 维**——`mouth` 全程被排除在 obs/action 之外，动作回写到 15 个电机槽位时第 9 位留零。见 [robotd-design.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/robotd-design.md) §2.2。另外 **Microduck 没有手臂，也没有躯干 yaw**。

---

## 2. 组成架构

### 2.1 七个守护进程 + 一个 CLI

这是全项目最重要的一张表。每个守护进程是**独立 crate + 独立 systemd unit + 同一个发布产物**。

| 守护进程 | 它是谁 | socket | 关键约束 |
|---|---|---|---|
| `robotd` | **唯一能命令电机者**：50 Hz 控制环 + 安全权威 + 大部分本地子系统 | `/run/robotd.sock` | 运行在 root（电机设备需要），持有总线排他锁 |
| `configd` | wifi 配网、身份/名称、PIN、重启、手柄配对 | `/run/configd.sock` | 必须在 `robotd` 死亡时仍可用 |
| `updaterd` | 更新引擎（拉取、验签、原子换、健康门控、回滚） | `/run/updaterd.sock` | 同上；**绝不命令电机**，只提问 |
| `btd` | BLE 传输层，**不持有状态**，只是中继 | `/run/btd.sock` | 同上 |
| `padd` | 手柄 → 意图，**无特权**（只有 `input`/`robot` 组） | `/run/padd/pad.sock` | 配对不归它，归 `configd` |
| `netpadd` | UDP 手柄（`[netpad] enabled` 时替代 `padd`） | — | 与 `padd` 由 `ExecCondition=` 二选一 |
| `mediad` | 相机/麦克风/WebRTC/远程网关/内嵌控制台 | `/run/mediad/media.sock` + TCP `:8080`/`:8443` | 独立进程，避免媒体崩溃拖死电机 |
| `tofd` | 头部 8×8 ToF | `/run/tofd/tof.sock` | 有无传感器都跑 |

`robotctl` 是**无状态 CLI**，随发布一起装到板上；`duckctl` 是**笔记本侧**工具，**永不进发布**。

### 2.2 四条不变量（架构层的地基）

来自 [architecture.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/architecture.md) §1.1：

1. **`btd` / `configd` / `updaterd` 必须在 `robotd` 死亡时存活**——它们是恢复路径。所以它们没有对 `robotd` 的 systemd 依赖、没有 ML 运行时、没有媒体栈。
2. **`robotd` 对安全有权威**——只有它持有总线写句柄。
3. **控制环从不阻塞等待其它服务**——发给控制环的一切都是 last-value-wins 的快照（原子读）。
4. **每份状态只有一个写者**——两个进程写同一个文件是设计错误。

### 2.3 IPC 契约：一条 socket、一个协议

- **传输**：Unix socket + JSON-RPC 2.0 + NDJSON（`tokio_util::codec::LinesCodec`），每服务一条 socket。
- **接口定义**：集中在 [`duck-ipc-proto`](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-ipc-proto/src/lib.rs)——**只依赖 serde/serde_json/semver**，不依赖 tokio/http/crypto。这是"契约层"，客户端可以只链接它。
- **两套词汇**：*意图进、状态出*。JSON-RPC 的两类消息正好对应：
  - **通知**（无 `id`、无回复、last-writer-wins）= 连续意图，如 `robot.move`、`robot.head`。50 Hz 下这意味着**零回复流量**。
  - **请求**（有 `id`、必须回复）= 离散动作，如 `robot.stop`、`robot.enable`、`robot.init`、`robot.do`。
- **控制面 vs 数据面**：控制面是几十字节、≤100 Hz 的 RPC；数据面（640×480 RGB@30fps ≈ 27 MB/s）**绝不过 socket**——"features, not frames"。控制环发布一个可订阅的 `robot.state` 流，**按订阅者抽帧**，慢客户端拿到的是空洞而不是背压。

**版本不匹配不拒绝**：一条 skew 被记录并被服务，只有真正缺失的路由或未知参数才会拒绝（`METHOD_NOT_FOUND` 指名方法）。

### 2.4 状态归属：谁的状态放在哪，更新和回滚中谁活下来

| 状态 | 位置 | 更新 | 回滚 |
|---|---|---|---|
| 二进制、默认配置、systemd unit | `/opt/robot/daemon/releases/<ver>/` + `current` 符号链接 | 整目录原子替换 | 符号链接切回 |
| 板级参数（镜像层） | `/etc/robot/robotd.toml` | 被替换 | 被替换 |
| 板级参数（用户层） | `/var/lib/robot/robotd.toml` | **逐键覆盖镜像层，永不被覆盖** | 同 |
| 名称 + PIN | `/var/lib/robot/config/config.json`（文件 + flock） | 存活 | 存活 |
| 策略集 | `/opt/robot/policies/releases/seed-<pin>/` + `current` | 由 postinstall 播种 | **不随 daemon 回滚**（见 §3.3） |
| 策略库（社区/本地） | `/var/lib/robot/policies/<org>/<name>/<rev>/` | 存活 | 存活 |
| vetbook（"鸭子的一生"） | `/var/lib/robot/vetbook.json` | 存活 | 存活 |
| HF 令牌 | `/var/lib/robot/hf-token` | 存活 | 存活 |
| 更新历史 | `/var/lib/robot/updater/runs/*.jsonl`（每条 fsync） | 存活 | 存活 |
| 运行中身份 | `/run/<service>/identity.json`（进程自发布，unit 停即被 systemd 删除） | 每次启动重写 | 同 |

两条规则值得背下来：

- **`releases/` 之外的一切在更新与回滚中都存活。** 所以"必须活着"的东西只能放 `/etc/robot/` 或 `/var/lib/robot/`。
- **用户层配置损坏 = `degraded`，绝不是 `unhealthy`。** 否则一个任何发布都修不了的文件会把所有发布回滚掉。

### 2.5 三条生命周期

**(a) 启动**：systemd 从 `multi-user.target` 起各守护进程，**彼此之间几乎没有顺序**（只有 `padd`/`netpadd` 在 `robotd` 之后、`btd` 在 `bluetooth` 之后，且都是"起不来就每 5 s 重试"的软顺序）。`robot-boot-check.timer` 在 1 秒后启动看护。

**(b) 更新**：整目录替换 + `current` 符号链接原子切换 + 健康门控 + 回滚。**这是一个必须先读的完整流程**，见 §3.3。

**(c) 仿真**：同样的二进制、同样的 unit、同样的 `robotctl`，只把 `RobotIo` 从"真实舵机总线"换成"TCP 上的 MuJoCo 身体"，见 §4.4。

### 2.6 一张图

```
                    ┌──────────────────────── 恢复路径（robotd 死了也得活）────────────────────────┐
                    │  configd（wifi/身份/PIN/reboot/配对手柄）   updaterd（更新引擎）   btd（BLE 中继） │
                    └───────────────────────────────┬──────────────────────────────────────────────┘
                                                    │ unix socket + JSON-RPC 2.0 + NDJSON
┌───────────────────────────────────────────────────▼──────────────────────────────────────────────┐
│                                    robotd —— 唯一能命令电机的进程                                  │
│                                                                                                   │
│   50 Hz 控制环（自己的 tokio runtime，MissedTickBehavior::Skip）                                    │
│     read()  ── 一次 fast sync_read：IMU 板 + 15 舵机（寄存器 124–136）                              │
│     decide  ── observation[61] → Policy::infer → action[14] → home + scale×action → 低通 → targets  │
│     write() ── sync_write 目标位置                                                                 │
│     publish ── 原子量总是；state 帧仅当有人订阅                                                      │
│     ↓ 每 1 s  slow_sensors()：电压 + 各关节温度（寄存器 144–146），另读 sysfs 的 SoC 温度与降频状态   │
│                                                                                                   │
│   ╔══════════════════════════════════════════════════════════════════════════════════════════╗    │
│   ║ safety.apply —— 持有唯一的 RobotIo 写句柄（借用检查器强制，不是约定）                        ║    │
│   ║   · 拒绝非有限值   · 夹紧到执行器行程   · deadman（意图停 → 速度归零）                       ║    │
│   ║   跌倒判定是"报告"，不是"规则"：它不影响谁被允许驾驶                                        ║    │
│   ╚══════════════════════════════════════════════════════════════════════════════════════════╝    │
│                                                                                                   │
│   同一条 tick 上还挂着（都不许阻塞它）：话音 / ToF theremin / chorale / 情绪 / pet-detect / vetbook   │
│   以及 kinematics（FK）、odometry（接触式里程计）、posture（看两眼决定怎么起身）                     │
└───────┬───────────────────────────────────────────────────────────────────────────────────────────┘
        │ /dev/ttyS2 · 1 Mbps · Dynamixel protocol v2（TIOCEXCL + 建议锁，一次只允许一个拥有者）
        ▼
 id 200: imu_to_dxl 板   ids 20–24: 左腿   ids 30–34: 颈/头/嘴   ids 10–14: 右腿
```

---

## 3. 详细构成（开发者视角）

### 3.1 workspace 里有 26 个 crate，分三类

来自 [Cargo.toml](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/Cargo.toml) 的 `members`。分类的判据是**它们是否会进发布产物、是否自己起进程**：

**① 守护进程（daemon）——每个一个 crate、一个 unit、随发布**

| crate | 说明 |
|---|---|
| `robotd` | 控制环进程本体。另含 `control.rs`（优先级链）、`posture.rs`、`sound.rs`、`theremin.rs`、`chorale.rs`、`emotion.rs`、`pickup.rs`、`soc.rs`、`vetbook.rs`、`leds.rs`、`intents.rs`、`params.rs`、`head_imu.rs`、`idle_head.rs` |
| `updater` | 更新引擎（lib）+ `updaterd`（bin）。含 `engine.rs`、`manifest.rs`、`preflight.rs`、`reconcile.rs`、`orphan.rs`、`verify.rs`、`journal.rs`、`store.rs`、`hooks.rs`、`os.rs`、`policy.rs`、`account.rs`、`source/{github,hf_hub,http,local}.rs` |
| `configd` | `net.rs`/`nm.rs`（NetworkManager 客户端，**Linux-only**）、`bluez.rs`、`pad.rs`、`identity.rs`、`store.rs`、`power.rs`、`indicator.rs`、`logs.rs`、`units.rs` |
| `btd` | `route.rs`（**每方法的路由允许表 = 安全边界**）、`gatt.rs` 类逻辑在 `duck-ble`、`session.rs`、`pairing.rs`、`link.rs`、`upstream.rs`、`chorale.rs`、`bluez.rs` |
| `padd` | `main.rs` + `tap.rs`，读 `gilrs` |
| `netpadd` | `receiver.rs`（UDP 收）、`robotd.rs`、`health.rs` |
| `mediad` | 最厚重的一个：`pipeline.rs`（GStreamer）、`camera.rs`、`exposure.rs`、`sensor.rs`、`producer.rs`、`stream.rs`、`relay.rs`、`session.rs`、`turn.rs`、`web.rs`、`snapshot.rs`、`detect.rs`、`vetbook.rs`、`config.rs`、`route.rs`、`frame.rs`、`indicator.rs`、`upstream.rs` |
| `tof` | `sensor.rs`、`link.rs`、`status.rs`、`imu.rs`、`config.rs` |
| `robotctl` | 板上 CLI，随发布 |

**② 库（library）——没有 socket、没有 tokio runtime、没有 systemd 启动它**

| crate | 说明 / 为什么单独成 crate |
|---|---|
| `duck-control` | **控制核心**：`io.rs`（`RobotIo` trait = 仿真接缝）、`bus.rs`（`DynamixelIo`）、`imu.rs`、`model.rs`（关节表/默认位姿/行程）、`obs.rs`、`policy.rs`（ONNX + 运行时探测）、`safety.rs`（**持有唯一写句柄**）、`fall.rs`（limpf-fall 预测器）、`pickup.rs`（被抱起分类器）、`sim.rs`（`RemoteIo`）。**编译器保证守护进程的关注点进不来**——这也是它能被整体搬进另一个仓库的原因 |
| `kinematics` | MJCF 模型 + 正运动学 + 头/手链。`mjcf.rs`/`math.rs`/`head.rs`/`hand.rs`/`tof.rs`。**有两个消费者**（`odometry` 与 `robotd` 的头 FK），所以必须共享一份模型 |
| `odometry` | 接触式里程计：`anchors.rs` + `lib.rs` |
| `robotd-params` | 参数 schema、默认值、校验、`edit.rs` 写回（带锁）、`registry.rs`、`board.rs`、`pad_axes.rs`、`serial.rs` |
| `pad-map` | 手柄映射的**唯一实现**，`padd`/`netpadd` 共用：`mapper.rs`、`bindings.rs`、`buttons.rs`、`continuous.rs`、`hold.rs`、`loss.rs`、`wire.rs` |
| `pad-imu` | 手柄 IMU → 头部控制 |
| `sounds` | 语音合成、每只鸭子的音色、chorale 总谱：`synth.rs`、`voices.rs`、`personality.rs`、`rng.rs`、`stream.rs`、`chorale/{beat,midi,text}.rs` |
| `pet-detect` | ~20 KB CNN，麦克风 40 段 log-mel 窗口，独立 worker |
| `duck-detect` | 鸭子检测（NPU/ONNX 两条后端：`rknn.rs`/`onnx.rs`）+ 跑分 bin |
| `duck-ble` | BLE 的 GATT/广播/分帧 |
| `uyvy` | YUV 帧格式转换 |
| `nfc` | NFC 读卡：`st25r100.rs`/`clrc663.rs`/`ndef.rs`/`pairing.rs`/`vtag.rs` |
| `duck-ipc-proto` | **线上契约**：serde-only。含 `led.rs`、`vetbook.rs`、`API_VERSION`、`JOINT_NAMES`、`ROBOT_MODEL_API` |
| `test-support` | 测试辅助 |

**③ 工具 / 测试替身——永不进发布**

| crate | 为什么永不发布 |
|---|---|
| `xtask` | 构建/打包/签名工具（`package`/`sign`/`promote`/`keygen`/`keycheck`） |
| `duckctl` | 笔记本侧客户端（BLE 访问机器人）。**被排除出 `default-members`**，因为它是本机客户端、绝不该被交叉编译 |
| `duck-ether` | 故意做得很差的假电台（丢包 + 延迟），因为"完美的电台会掩盖真实电台暴露的 bug" |

### 3.2 控制核心怎么构成的（读懂这一节就能读懂 robotd）

#### 3.2.1 总线层与 `RobotIo`——唯一的接缝

`duck-control/src/io.rs` 定义了六个方法，这是**仿真被允许存在的唯一地方**：

```rust
trait RobotIo {
    fn read(&mut self) -> Result<Sensors>;              // 关节 + IMU，一次事务
    fn write(&mut self, targets: &JointTargets) -> Result<()>;
    fn set_gain(&mut self, kp: u16) -> Result<()>;      // 每关节一次写，所以不是逐 tick 调用
    fn set_torque(&mut self, on: bool) -> Result<()>;   // 同上
    fn slow_sensors(&mut self) -> Result<SlowSensors>;  // 电压 + 各关节温度
    fn imu_stale(&self) -> ImuStale;                    // 有默认实现
    fn imu_ready(&self) -> bool;                        // 有默认实现
}
```

三个实现：`DynamixelIo`（真机）、`FakeIo`（脚本化样本，**这就是 `cargo test` 不需要硬件的原因**）、`RemoteIo`（TCP → MuJoCo）。

**几个非常具体、且都带注释说明"为什么"的工程细节**（值得学习，也直接决定你改代码时不能碰什么）：

- `return_delay_time=0` 是**承重的**：XL330 出厂值 250 意味着每设备 500 µs 转向开销，16 个设备 = 每 tick ~8 ms = 预算的 40%。启动时的寄存器检查会**断言并纠正**它。
- `shutdown = 52`（`0b110100`）清掉了输入电压位，而出厂 53 设置了它——正是这一位让这条总线能在 2S 电池（>6.6 V）上驱动额定 6.0 V 的舵机。
- **换上去的新舵机是被"收编"的，不是手工配的**：新 XL330 出厂是 ID 1 / 57 600 baud。`open_bus` 先 ping 那 15 个预期 ID，若**恰好一个**沉默，就去找 ID 1（先 1 Mbps，再重开 57 600），写 ID、写 baud、重开、跑寄存器检查、重启它。总线完整时这一步只花 15 次 ping。**两个缺失就不动**——无法判断新舵机替换的是哪一个。
- **两次总线事务 + 每秒第三次**：tick 读 124–136 连续块；电压/温度在 144–146，隔了 12 字节没人要的轨迹寄存器，所以单独每秒采样一次，而不是把 tick 的读宽到 22 字节/舵机。
- **电压取平均**（15 个舵机同一条电池），**温度不取平均**（按名字报告最热的那个——蹲姿的膝盖远高于嘴，均值会掩盖正在逼近过热关断的关节）。
- **每次 sync read 都是 fast sync read**（协议 2.0 指令 0x8A），一次广播回包代替 16 个状态包。代价是**XL330 固件必须 ≥ v46**；不满足时不报错，而是**每次读都超时**——所以它做成 `bus.fast_sync_read`（默认开），失败时整条环表现为 unhealthy 并自动回滚。
- **IMU 陈旧度被永久跟踪**：读成功但返回同一帧 = 给策略喂死掉的姿态，不做计数就完全不可见。

#### 3.2.2 唯一的观测构造器

所有 alpha 策略都是 `obs[1,61] → actions[1,14]`，所以**只有一个布局**：

```
[ gyro(3) | projected_gravity(3) | joint_pos(14) | joint_vel(14) | last_action(14) | command(13) ]
                                                                              command = vel(3) + head(4) + body(6)
```

命令块（曾是最可疑的部分，最终是从原型 `control_step` 里读出来的，不是猜的）：

```
48..51  vx, vy, vyaw
51..55  neck_pitch, head_pitch, head_yaw, head_roll
55..57  body x, y        —— 硬编码 0，训练中未绑定
57..60  body z, roll, pitch
60      body yaw         —— 硬编码 0，训练中未绑定
```

三个"单独看都合理、但都错"的点：全零 body 是**名义编码**而非占位；头部目标**在命令里，不能再叠加到策略输出上**（原型两种模式都做，会导致头被弯两次）；body 块顺序是 `z, roll, pitch` 而非 `z, pitch, roll`（换错会让"前倾"变成"侧倾"）。

#### 3.2.3 优先级链与"一次只做一个动作"

`robotd/src/control.rs` 持有优先级链和所有数值默认值：

```
skill 窗口 ← 推进/过期（roulade 窗口、kick 计时、ground-pick 相位、sit↔stand 起身）
command    ← 调用方平滑后的命令，按当前 skill 重新编码
net        ← roulade > kick > ground pick > sit/rise > stand（按 |twist| 或强制）> walk
action     ← ONNX
targets    ← home pose + action_scale × action
filters    ← 头与腿的一阶低通
```

两条"容易被误改"的原型行为被刻意保留：**kick 窗口用站立调参跑**、**sit↔stand 的"起身"也用站立增益**（因为它们的命令都是全零，站立网络就是在这个条件下触发的）。一处刻意的偏离：scale/gain 改为**每 tick 从活动状态重算**，而不是像原型那样保存/恢复 `action_scale`（后者会在 sit→stand 后留下陈旧值）。

**一次只做一个动作，且不能从坐姿起**：捡拾、skill、坐下、起身，任一在飞行中时其它都被拒绝（包括正在滑入座椅）；只有"站起来"能从坐姿被接受。唯一的例外是按钮**按住**导致的同一 skill 串联。

#### 3.2.4 安全层：结构化的不变量

`safety` 持有唯一的 `RobotIo` 写句柄，所以"上层不能动电机"是**借用检查器保证的，而不是靠记忆**。两条无条件规则：

- **拒绝非有限值**（`NaN` 不夹紧，直接拒绝）；
- **夹紧到执行器行程**（注意：这是*执行器*行程，不是解剖学限制——它抓 `NaN`、荒谬的 scale、垃圾张量，但不会阻止一个关节被驱到机械上不明智的位置）。

外加 command deadman：意图停止到达 → 速度归零。**停止 ≠ 松力**：失联时双足的安全状态是"站住别动"，而失衡是另一件事，这一层不回答它。

**跌倒判定是"报告"，不是"规则"**：每 tick 计算（躯干系投影重力，去抖 0.2 s），发布，但**不影响任何决策**——跌倒的机器人依然可以被 enable / init / 驾驶 / 接受 skill。这是刻意的：躺在地上时恰恰是这些调用最需要能用。早期版本这里曾有一个 `fall_limp` 门和一个 `fall_recover` 自动起身，都被删掉了，因为"一个恢复必须绕过的安全规则不是安全规则"。

除"跌倒"之外还有两个事件（这两个和你的摔倒保护项目直接相关）：

- **§2.4.1 `limp_fall`（落地前的软化，默认开）**：跌倒判定问的是"机器人是不是已经倒了"，而要软化落地时那个窗口已经关闭。所以另有一个基于**变化率**的检测器 `duck_control::fall`：投影重力随躯干旋转，`ġ = −ω × g` 是精确的，直接来自同一个 IMU 块里的陀螺；把它外推约 0.3 s 就知道重力要去哪。触发条件：已经倾斜（≈26°）、仍在倾倒而非恢复、预测越过跌倒阈值、去抖三 tick。**只在站立状态下布防**（在倾斜门内停留 200 ms），触发后解除直到再次站起。
  它买到的**不是落地本身，而是落地之后的起身**：直立策略面对一个静止、已知姿态的机器人能干净地起来，面对一个在地上挣扎的机器人要试好几次走路增益——电机负载正是从那里来的。序列：`gain_limp` 跟关节下垂 → 等陀螺安静 → 约 1 s 回到站立姿态 → 交还。
  **调参就是特性，且是不对称的**：假阳性是"机器人自己造成的跌倒"，比它想避免的僵直落地更糟，所以默认值刻意偏晚。
- **§2.4.2 被抱起来（`[pickup]`，默认开）**：一个小的分类器读控制环**已经在读**的最后一秒数据（陀螺、投影重力、关节位置/速度、上一 tick 的命令目标），打分"是否被手抱着"。>0.8 持续 100 ms → 暂停策略；<0.35 持续 80 ms 且暂停已满 300 ms → 交还。暂停姿态不是 home，而是走路策略自己的**平均站立姿态**（home 在走路增益下只有持续平衡它的策略才能保持）。模型完全在 `microduck_rl` 的仿真里训练（`pickup/`、`scripts/pickup_*.py`），特征布局/暂停姿态/迟滞时序是 `duck_control::pickup` 里的常量，**只有两个阈值是参数**——改其它部分而不重训是"静默地错"。

#### 3.2.5 机器人模型与"两个 load-bearing 表"

`duck_control/src/model.rs` 只有 alpha 一套：15 关节、Dynamixel ID `20–24 / 30–34 / 10–14`、名字、`DEFAULT_POSITION`、执行器行程。其中两张表**超出本 crate 也有约束力**：

- `JOINT_NAMES` **来自 `duck-ipc-proto`**——因为线上按位置索引 `joints`/`targets`，两个顺序决不允许漂移，一个 `const` 断言把"不允许"变成事实。
- `DEFAULT_POSITION` **必须与训练环境的 `HOME_FRAME` 一致**——策略观测的是相对 home 的关节位置，差异就是 14 个观测槽上的常量偏移。

#### 3.2.6 其余五个子系统（没有独立设计页，因为只有一个实现、一个消费者）

| 在进程里 | 是什么 | 推理放在哪 |
|---|---|---|
| `sound.rs` | 播放时的话音——一个 `aplay` 子进程，新声音杀掉旧的（codec 的 PCM 是独占的） | 模块头注释 |
| `theremin.rs` | `tofd` 的深度 15 Hz → 一个音符 + 张嘴 + 一行状态 | 模块头 |
| `chorale.rs` | 多鸭合唱：最低 id 指挥，指挥拥有座次，`btd` 只传信标不做思考 | 模块头 |
| `emotion.rs` | 情绪 = 关键帧 + wav，随发布；`robot.do <name>` 走的是客户端同样的头/姿态/嘴命令 | 模块头 |
| `pet-detect/` | 麦克风 40 段 log-mel → ~20 KB CNN，独立 worker | crate 头 |
| `vetbook.rs` | 鸭子的一生：tick 无锁累加，每秒 `try_lock` 交给共享账本，每分钟原子保存；每 10 s 读一次每个舵机的锁存告警 | 模块头 + `duck_ipc_proto::vetbook` |

> **"没有设计页"是规则在起作用，不是缺口**：一个服务只有在"第二个读者必须从代码反推契约"时才值得一页文档。这些各自只有一个实现、一个消费者，决策足够局部，放在模块头正合适。它们真正需要的是**可操作**，那是速查表的活。

### 3.3 更新引擎（`updater`）——"发布是被换掉的，不是被补丁的"

这是全项目最完整、也最值得当作范本的一段工程。核心是 [updater-design.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/updater-design.md) 与 [restart-order.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/restart-order.md)（后者是"什么在什么时候重启"的唯一权威）。

**发布产物**（一个 `.tar.zst`）：

```
bin/robotd  bin/mediad  bin/btd  bin/configd  bin/updaterd  bin/padd  bin/netpadd  bin/tofd  bin/robotctl
systemd/*.service  systemd/sysusers.d/*.conf
version.toml                # semver, min_hw_rev, schema_version
hooks/preinstall  hooks/postinstall   # 可选、可执行、随版本签名
```

**签名与信任**：minisign。CI 签名（产物 + manifest）。机器人带**一组**受信公钥（`/etc/robot/trusted_keys/`），任一验证通过即可——**从第一版就发布一组密钥**，因为丢了唯一一把就等于要手工重刷每台机器人。验证顺序：验 manifest 签名 → 下载 → 验 sha256 → 验产物签名。**任何未签名的字节都不会被执行的、也不会被解到活路径。**

**状态机（`updater/src/engine.rs`）**：

```
触发 → PREFLIGHT（单飞锁 · 时钟/NTP · 机器人已停 · 无 WebRTC 会话 · 磁盘空间）
     → 取 manifest → 验 manifest 签名 → 版本/pin/min_hw_rev/model_api 检查
     → 下载 → 验 sha256 → 验产物签名
     → 解到 releases/.staging-<ver>/root/ → 孤儿 unit 检查 → [preinstall]
     → rename 成 releases/<ver>/ → 布防启动计数器 → 切 current 符号链接
     → [postinstall] → on_apply（逐个 systemctl restart）→ updaterd --self-test
     → 健康门控（每 500 ms 轮询 robotd，最多 30 s）→ 确认启动计数器 → 剪枝
     → 释放锁 → 调度延迟重启 → 回复 → 5 s 后重启 updaterd / btd
```

**八个守护进程里，`updaterd` 和 `btd` 绝不在更新中重启**——一个是执行者本身，一个是传输通道；它们在**回复发出后约 5 秒**通过 `systemd-run --on-active=5s` 的**瞬态 unit** 重启（不能用子进程：子进程在 `updaterd` 的 cgroup 里，重启父进程时会把它一起杀掉）。其余六个（`configd, mediad, netpadd, padd, robotd, tofd`）按字母序分别 `systemctl restart`，**绝不批量**（`restart a b` 在任一 unit 未知时会整体失败且不重启存在的那一个）。

**重启集是从发布产物自身推导的**，不是从板上配置读的：一个 `systemd/*.service` 带 `[Install]` 段就算一个"更新要起的 unit"。板上 `updater.toml` 的 `on_apply.units` **只是追加**，用来命名发布不提供的 unit。这个改动修掉了一类历史 bug：板上的 `updater.toml` 若早于某个守护进程存在，那个进程就永远不会被重启而更新还报成功。

**自动回滚默认关闭**（`[update_gate] auto_rollback`）。关掉时门控仍然运行、仍然判定——失败的发布**留在原地**，结果是 `Stuck`，并给出能撤销它的命令。理由是：现在发布以测试过的 Yocto 镜像到达机器人，一次没有人要求的回滚比留下一个坏发布伤害更大。

**健康门控的判据只有"是发布能被归咎的东西"才允许进入裁决**：电池、电机温度、板温、时钟降频**都是描述**，任何自动决策都不许读它们。否则"低电量时更新 → 回滚 → 用同一块低电量评判替代品 → 永远更新不了"。

**它的三个"在坏状态下也不坏"的设计**（对做独立项目很有参考价值）：

1. **`updaterd` 启动时先 `recover_on_start` 再开 socket**——已经启动回退的机器人在任何人问它之前就在回退了。
2. **审判状态下的启动计数器**：`boots >= 2` 时才去问 `robotd`，且**只有既非 healthy 也非 degraded 才回退**。到预算耗尽还没被确认，通常不是坏发布，而是 apply 在门控跑之前被杀掉。
3. **进程外的兜底**：`robot-boot-check.timer` + `robot-rescue`（`/bin/sh`），当 `updaterd` 自己都起不来时把 `current` 切到 `golden` 并重启。它必须在进程外、发布外，因为它要处理的失败正是"`updaterd` 不启动"。

### 3.4 传输与外围

- **`btd`**：BLE。**它是一张每方法的路由允许表**——`robot.move`/`robot.head`/`robot.look`/`robot.pose`/`robot.mouth`/`robot.stop`/`robot.relax`/`robot.subscribe`/`update.pin`/`update.resetToGolden`/`system.pairingPin` 等**被 `btd` 自己拒绝、永不触达守护进程**（错误码 14，"not available over Bluetooth"）。而 `robot.do`/`robot.init`/`robot.enable`/`robot.rebootMotors` **不在此列**：遥操作是"每秒 50 个小更新的流"，20 字节的通知预算装不下；而这些各是**一个请求**。每个拒绝的理由就写在 [btd/src/route.rs](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/btd/src/route.rs) 旁边。
- **`mediad`**：相机 + WebRTC + 内嵌控制台（`:8080` 页面，`:8443` 信令）。控制台页面里的技能菜单是**问 `robot.policies` 得来的**——技能是配置，不能猜。
- **`configd`**：配网、身份、PIN、配对手柄、重启。Linux-only 的是它的 NetworkManager 客户端。
- **`tof`**：`tofd` 还负责在 zero3 上读 BMI088（默认关），beta 上则由 `robotd` 读 LSM6DSV16X（默认开）。
- **MediaPipe 之外的检测**：`duck-detect`（NPU/ONNX）、`pet-detect`（麦克风）。

### 3.5 两个客户端

| | `robotctl`（板上，随发布） | `duckctl`（笔记本，永不发布） |
|---|---|---|
| 传输 | unix socket | BLE（名字刻意不提蓝牙，因为 `mediad` 会给出第二种传输） |
| 定位 | 运维/支持工具 | 手机 App 的替身，也是"从没连过 wifi 的机器人"的唯一入口 |
| 覆盖 | 全量命令域（见 §4.1） | 大部分命令的等价物 + `scan/ip/ssh/scp/open/udp-pad/call` |
| 关键约束 | **不得链接更新器的 http/tar/zstd/crypto 树**（恢复路径上的支持工具要保持单薄） | 一条命令 = 一个连接：找到、必要时配对、验 PIN、问、断开 |

`duckctl scan` 会列出区域内广播 duck 服务的机器人，**并顺带广播自己的 IPv4**，所以"要 ssh 到哪个地址"也来自这里，无需连接、无需 PIN。

### 3.6 一次改动的完整旅程

这是"开发者是怎么完成这个工作的"最直接的答案：

```
改一行源码
  │
  ├─ 1. cargo test --workspace                     ← 本地门：942 个测试（macOS 全量通过），无需硬件/网络/Docker
  ├─ 2. cargo fmt --all &&（Linux-only crate 需要时）按板目标跑 clippy
  ├─ 3. cargo board --bins                         ← 交叉编译验证（zigbuild → aarch64，glibc 2.31 pin）
  └─ 4. scripts/dev-push.sh --name duck-c51b       ← 约一分钟：交叉编译 → xtask package → dev key 签名
                                                    → scp 到板上 ~/duck-sideload → robotctl update apply --from
                                                    → 等所有守护进程确认跑的是新版本
```

`dev-push` 走的是**普通更新路径**——签名、哈希、兼容性、健康门控全都真跑，只是换了签名用的 key。输出长这样：

```
==> building 0.5.1-dev.local.1763400000.g7fc1444 for the board (zigbuild)
==> packaging
==> signing with ~/.duck-keys/team.dev.key
==> applying on microduck@192.168.1.42
==> 0.5.1-dev.local.1763400000.g7fc1444 is live
==> checking every daemon is running it
    [ok] robotd   [ok] configd   [ok] padd   [ok] updaterd   [ok] btd   [ok] mediad   [ok] tofd
```

前置三件事（详见 [dev-push.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/robot/dev-push.md)）：板必须是**dev 板**、`~/.duck-keys/team.dev.key` 存在、有能交叉编译的工具链（`cargo install cargo-zigbuild --locked` + `brew install zig`，或统一加 `--docker`）。让一块已存在的板接受 dev 构建：`sudo robotctl configure --allow-dev-daemon on`。

**这个闭环的设计意图**：让"App / SDK / 任何远程客户端用的那条输入路径"就是开发者**每天都走**的那条路径（手柄是客户端，不是特权进程），所以它不可能悄悄腐烂。

---

## 4. 如何使用

### 4.1 机器人侧：`robotctl` 命令域速查

完整命令见 [cheatsheet.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/robot/cheatsheet.md)。按域划分：

| 域 | 代表命令 |
|---|---|
| 版本与健康 | `robotctl version` / `health` / `monitor` |
| 配置编辑器 | `robotctl configure`（交互式，`--list` 列出所有键；**一个键该"重启才生效"还是"实时生效"是有测试断言的**） |
| 视频 | `media.quality` 四档、`media.source=test`、`media.congestion_control` |
| **策略与技能** | `policy list / check / update / load / reset / search / add / remove`；`robot do <name>`；`pad bind` |
| 手柄 | `pad bindings` / `pad bind` / `pad reset`；`pad_axes` / `pad_head` / `pad_body` / `pad_roller` / `pad_drive` |
| 语音/合唱/Theremin | `sounds ensure-bank`、`robotctl chorale`、`[theremin]` |
| 传感器 | `tofd`、头 IMU 归属、`robotctl frame --output /tmp/frame.uyvy` |
| 网络与账号 | wifi（`--psk-stdin` 优于 `--psk`）、`account login`（RFC 8628 设备码，令牌 30 天自续） |
| 身份与电源 | `system set-name`、`system pin`、`robot init/relax/reboot-motors` |
| **更新与日志** | `update status/check/apply/rollback/log/show/watch/select/pin/reset-to-golden`；`logs robotd -n 300 -b -1` |
| 其他 | `robotctl endurance`、vetbook（含 gallery 与公开数据集） |

两个必须知道的语义细节：

- **`robotctl monitor`** 是实时统一视图：按键 `q`/箭头/`u`/`t`/`c`/`d`/`[`/`]`/`p`，`--json --hz 50 > run.jsonl` 可录制成 jsonl（做回归对比很有用）。
- **`robotctl health` 在 unhealthy 或不可达时退出非零**，所以它可以用来 gate 脚本。而"一个热电机或一个被 pin 的组件是被**报告**的，不是被**判决**的"。

### 4.2 笔记本侧：`duckctl`

```bash
duckctl scan                      # 区域内有哪些鸭子，顺带给出 IPv4
duckctl ip                        # 只打印地址，可直接代入 ssh
duckctl ssh -- sudo robotctl pad pair
duckctl open                      # 用浏览器打开机器人内嵌控制台
duckctl policy search microduck   # 看看 Hub 上别人为这个机器人发布了什么
```

一次命令一条连接；`--verbose` 打印每一行收发，是"卡住了先加什么"的答案。完整命令见 [duckctl.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/robot/duckctl.md)。

### 4.3 策略从哪里来

三个原点，一个槽位（[policy-channel-design.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/policy-channel-design.md) §2）：

| 原点 | 来源 | 签名 | 自动更新 | `reset` 目标 |
|---|---|---|---|---|
| **official** | `pollen-robotics/*`（`policies` 组件） | 是 | 是 | **是** |
| **community** | 任何其他 HF repo | 否 | 否，只报告 | 否 |
| **local** | 板上路径 | 否 | 否 | 否 |

**归谁"官方"由 HF org 决定，是硬编码的**——一个可以被配置信任哪个 org 的机器人，"官方"这个徽章就没有意义了。

**社区策略不做签名验证**，这是全项目唯一的例外：理由是一个策略**不是二进制**——`robotd` 持有总线唯一写句柄，背后有关节夹紧、跌倒→松力反射、意图 deadman，且 `obs[1,61] → actions[1,14]` 的形状门在机器人**站着不动**时就会拒绝错误形状的图。**这个沙箱才是边界，不是签名。**

七个槽位：`walk`、`stand`、`sitstand`、`ground_pick`、`kick_left`、`kick_right`、`roulade`。策略清单契约（schema 2）有两个轴：`kind`（`episodic`/`perpetual`/`scripted`，回答"谁结束它"）与 `command.encoding`（`constant`/`phase`/`posture_flag`，回答"守护进程喂它什么"）。**只有"常量命令 + episodic"才能成为通用一次性技能**（`robot do <name>`）。字段全表见 [policy-manifest.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/policy-manifest.md)。

### 4.4 没有硬件怎么办：仿真

[simulation.md（设计）](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/simulation.md) / [simulation.md（使用）](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/robot/simulation.md)

**接缝就是 `RobotIo`**。它上面的一切——控制环、策略、安全、跌倒检测、运动学、里程计、整个 IPC 面、`robotctl`、`duckctl`——是机器人**跑的同一份代码，且无法分辨**。

```
scripts/duck-sim            # 开窗，鸭子站起来，归你了
scripts/duck-sim drive      # 让它走，并报告"到底走了多远"
scripts/duck-sim ctl health # 或任何别的 robotctl
scripts/duck-sim log / down
```

两种形态：

| | `up`（普通进程） | `boot N`（systemd-nspawn 容器） |
|---|---|---|
| 启动 | 秒级，你的用户，pidfile | 真实 rootfs + overlay，每个鸭子一个容器 |
| 能测 | 控制环、策略、IPC、客户端 | **真实 systemd unit / `User=` / `RuntimeDirectory=` / hardening、更新器、健康门控、回滚、重启顺序** |
| 默认 socket | `~/.cache/duck-sim/duck-a.sock`、`duck-a-tof.sock`；身体在 TCP `7801` | 同名，可寻址为 `duck-a` |

关键环境变量：`DUCK_SIM_RL`、`DUCK_SIM_STATE`、`DUCK_SIM_DUCKS`、`DUCK_SIM_SCENE`、`DUCK_SIM_CAMERAS`、`DUCK_SIM_DUCK`、`DUCK_SIM_KEYFRAME`（默认 `SIT`）、`DUCK_SIM_VIEWER`、`DUCK_SIM_PORT`(7801)、`DUCK_SIM_FRAME_PORT`(7901)。

**五条"会把你坑一小时"的知识**（都来自实测）：

1. **`--sim` ≠ `--fake`**。后者没有物理。
2. **实时比 <1.0× 时策略无法平衡**；**低于 45 Hz 门控转 unhealthy**，容器形态下更新器就开始回滚发布。
3. **鸭子不能热加入**——MuJoCo 要编译模型，改数量会重启模拟器（所以 `RemoteIo` 会重连）。
4. **相机渲染才是伸缩瓶颈**，不是双足；所以相机必须按鸭子 opt-in。
5. **`velstand` 需要 backlash 场景**：默认场景是 `scene_backlash.xml`；`scene.xml`（无 backlash）上它一步不走。而 `alpha_walking` 在 MuJoCo 里**根本不会走**。

**仿真不能告诉你的事**（明确划界）：Dynamixel 总线驱动、BLE 射频、相机 ISP 与 3A、NPU、硬件编码器、热与电池。文档给了一个很好的自检：把一周的真实 bug 拿这个清单过一遍——`videoflip` 破坏编码器零拷贝路径、3A 引擎漏掉 stream-start 事件、INT8 头塌成两个值、舵机总线丢读——**一个都抓不到**。这不是缺陷，是边界。

而 `duck-ether`（故意做得差的假电台）的存在理由很值得学习："完美的电台会掩盖真实电台暴露的 bug"——这是**实测**出来的，四只鸭子每次都在同一小节会合，因为每只鸭子都瞬间无损地看见其它所有鸭子。

---

## 5. 如何测试

### 5.1 三层，从便宜到贵

**第一层：CPU 单元/集成测试（`FakeIo`，无硬件、无网络、无 Docker）**

```bash
cargo test --workspace          # CONTRIBUTING 记录：macOS 全量 942 个测试通过，无排除项
cargo test -p duck-control      # 单个 crate
```

Linux 上需要四个 C 库：`libudev-dev`（`gilrs` 无条件拉 `libudev-sys`）、`libgstreamer1.0-dev`、`libgstreamer-plugins-base1.0-dev`、`libgstreamer-plugins-bad1.0-dev`。**`configd` 的 NM 客户端与 `btd` 的 BlueZ 客户端是 Linux-only**，所以 macOS 上这些代码路径是被条件编译掉的。

`duck-control` 的测试夹具非常规范，值得照抄做法：[duck-control/tests/fixtures/](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-control/tests/fixtures) 里有一整套**故意做坏的** ONNX（`bad_width`、`bad_action_count`、`bad_rank`、`bad_batch`、`bad_state_shape`、`dynamic_batch`、`extra_input`、`missing_state`、`nan_state`、`wrong_type`、`feedforward`、`lstm`、`lstm_changed`）+ 一个 `generate.py`。也就是"每个拒绝理由都有对应的坏输入作为测试"。

**第二层：板级断言（`scripts/board-test.sh`）**

它用 `cargo-zigbuild` 交叉编译，然后在**真实 ARM64 Linux 容器**里跑二进制，在 CI 里跑 **60 条断言**：回滚、篡改拒绝、启动计数恢复、socket 权限模式、peer-credential 授权……**跑的是真实引擎并注入故障，而不是 mock**。它还断言产物的 glibc floor。

容器覆盖不到、必须上硬件的：真实的 `systemctl restart`、eMMC 写入行为与耗时、真实文件系统上的 `statvfs`、以及健康门控超时是否适合一台要几十秒才站起来的机器人。

**第三层：仿真（见 §4.4）**

`up` 形态测控制面与客户端；`boot N` 形态测 systemd/更新器/配置。这是唯一能在没有硬件时**端到端**跑完 `robotctl update apply`（preflight、签名、哈希、兼容性、健康门控、自动回滚）的方式——因为 `dev-push.sh` 就是用它安装的。

### 5.2 静态检查与覆盖率

```bash
cargo fmt --all
# Linux-only crate 需要按板目标跑（否则会被 macOS 的条件编译骗过）
RUSTFLAGS="-D warnings" cargo clippy -p configd --all-targets --target aarch64-unknown-linux-gnu
cargo llvm-cov                  # 覆盖率下限 70%
```

### 5.3 CI：10 个工作流

```
ci.yml                    ← 主门：fmt / clippy / test / 覆盖率 / board-test.sh
dev.yml                   ← 每次分支推送 → daemon-dev-<branch> 预发布
release.yml               ← tag daemon-vX.Y.Z → 正式发布
promote.yml               ← tag daemon-staging-vX.Y.Z → 金丝雀 / 提升
_build-release.yml  _build-duckctl.yml  _promote-release.yml   ← 可复用的内部工作流
duckctl-pypi.yml          ← duckctl 发布到 PyPI（uvx/uv tool 安装的来源）
prune-dev-releases.yml    ← 清理 dev 预发布
turn-endpoint.yml         ← 远程接入的 TURN 端点
```

### 5.4 发布的两种节奏

- `daemon-dev-<branch>`：每个分支的开发构建。语义是 `<crate>-dev.<run>.<sha7>`，**不可成为 latest、不能装到客户机器人**；板子的六小时检查会把它替换掉。
- `daemon-staging-vX.Y.Z`（预发布，金丝雀）→ **promote** 成 `daemon-vX.Y.Z`（复用字节、重新签名）。
- `xtask package` **拒绝** tag 与 `Cargo.toml` 版本不一致；`min_supported` 用于强制低版本更新。

---

## 6. 如何开发：约定与工作流

### 6.1 仓库四条约定（[CONTRIBUTING.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/CONTRIBUTING.md)）

1. **注释解释"为什么"**，不解释"做什么"。
2. **每一个非显然的决策配一个测试，且测试注释写明它防的是哪种失败。** 你在本仓库会反复看到这个句式（"which failure it exists to prevent"）。
3. **先找现成 crate**，再考虑自己写（但注意"板路径上优先纯 Rust crate"——`gilrs` 拉 `libudev-sys` 的代价会在每个 C 依赖上重现）。
4. **commit trailer 用 `Assisted-by:`，而不是 `Co-Authored-By:`。**

### 6.2 文档机制归属规则

`docs/README.md` 给每个机制指定**恰好一页**拥有者。写在别处的事实 → 一句 + 链接。冲突时**非拥有者是 bug**；行为与设计文档冲突时**文档是 bug**。

举例：控制环属于 [robotd-design.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/robotd-design.md)；策略的**通道**（哪个文件填哪个槽、谁发布的）属于 [policy-channel-design.md](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/docs/design/policy-channel-design.md)；策略**是什么、怎么校验、怎么跑**属于 robotd-design §2.3。所以 policy-channel 里只会写一句"§2.3 拥有它"。

这条规则极大降低了你读代码时的认知成本：**要去哪一页找答案，是可以推导的。**

### 6.3 四条"绝不要做"的设计原则（本项目反复回购的教训）

- **不要围绕版本差异做设计。** 一个用户、一台机器人。旧组件的限制是**要提出的问题**，不是要绕开的东西——升 `API_VERSION` 并说明安装后果。skew 被记录并被服务，绝不拒绝。
- **不要把"只在出问题时才跑的代码"当成能信任的代码**（更新器的恢复路径、安全层）。让坏状态**不可表示**，而不是靠记性。
- **不要让一个安全规则需要被恢复流程绕过**——那说明它不是安全规则（`fall_limp` 被删就是因为这个）。
- **用一个测试把"名单两处必须手工保持一致"变成"不可能不一致"**（`JOINT_NAMES` 的 const 断言、`xtask` 里 pin 的字面量与脚本的一致性断言）。

### 6.4 典型开发循环（小步、可工作、可回退）

```
mvn 等价物：cargo test -p <crate>          ← 改一处，先过这一层
          cargo fmt --all
          cargo board --bins               ← 交叉编译验证
          scripts/dev-push.sh --name <robot>   ← 上板，约一分钟
          robotctl monitor / health / logs robotd -n 300
```

无板时把最后两步换成 `scripts/duck-sim` / `scripts/duck-sim boot 1`。

---

## 7. 两个仓库的接缝（做独立项目的落点）

| 方向 | 接缝 | 你要做什么 |
|---|---|---|
| **训练 → 机器人** | HF Hub 策略仓库 + `manifest.json` | 用 `microduck_rl` 的 `uv run publish` 生成符合契约的单策略仓库；机器人侧 `robotctl policy load <slot> <repo>` 或 `policy add` 即成技能 |
| **环境 → 训练** | `mjlab.tasks` entry point | 在你的独立包里注册 `register_mjlab_task(...)`，依赖 `mjlab-microduck`（git 依赖 `microduck_rl`） |
| **仿真 → 控制环** | TCP + NDJSON（`duck_control::sim`） | 由 `microduck_rl` 的 `duck-body` 提供；你不需要改主仓库 |
| **策略形状契约** | `obs_len=61` / `action_len=14` / `model_api` | 形状不对会在**下载前**被 manifest 门拒绝，在**加载时**被形状门拒绝——两条都有明确报错 |

**关键判断**：要接入本项目，**不需要 fork 主仓库**。官方已经把两条即插即用通道开好了（entry point 与 HF 策略集），而且 `duck-control` 这个 crate 的边界是编译器强制的、被明确设计成"将来可以整体搬进另一个仓库"。

---

## 8. 关键数字与契约速查

| 项 | 值 |
|---|---|
| `API_VERSION` | **51**（[duck-ipc-proto/src/lib.rs:525](file:///d:/4_DOCUMENTS/0_Learning/3_Github_Project/microduck/duck-ipc-proto/src/lib.rs#L525)） |
| 关节数 | 协议 15（含 `mouth`）/ 策略 14 |
| obs / action | `[1,61]` / `[1,14]` |
| 控制环 | 50 Hz，`MissedTickBehavior::Skip`，独立 tokio runtime |
| 健康门控阈值 | 低于 **45 Hz** → unhealthy（容器形态下更新器开始回滚） |
| 更新健康门控轮询 | 每 500 ms，最多 30 s |
| 延迟重启 | 回复后 **5 s**（`systemd-run --on-active=5s`） |
| 总线 | `/dev/ttyS2`，1 Mbps，Dynamixel 协议 v2，fast sync read（0x8A，固件 ≥ v46） |
| 寄存器 | tick 读 124–136；电压/温度在 144–146，每秒一次 |
| 跌倒判定去抖 | 200 ms |
| limp-fall 布防 | 倾斜门内 200 ms；倾斜 ≈26°；去抖 3 tick |
| pickup 迟滞 | >0.8 持续 100 ms 暂停；<0.35 持续 80 ms 且满 300 ms 交还 |
| 电池映射 | 6.6 V 空 / 8.2 V 满（NP-F550，2S 直供） |
| Rust | 下限 1.89 / 上限 1.94.1 |
| glibc pin | 2.31 |
| 覆盖率下限 | 70% |
| 板级断言 | 60 条 |
| 测试数（macOS） | 942 |

---

## 9. 建议的学习路径

**第 1 天（建立骨架）**：`AGENTS.md` → `README.md` → `docs/README.md` → `docs/design/architecture.md`（691 行，全读）→ `docs/project/roadmap.md`（看清哪些 done、哪些是"故意不做"）。

**第 2 天（控制核心）**：`docs/design/robotd-design.md`（约 1195 行，全读；这是最深的一页）→ `duck-control/src/{io,model,obs,policy,safety}.rs` → `robotd/src/control.rs`。

**第 3 天（工程与运维）**：`CONTRIBUTING.md` → `docs/design/{updater-design,restart-order}.md` → `docs/robot/{cheatsheet,dev-push,simulation}.md` → 跑一遍 `cargo test --workspace` 与 `scripts/duck-sim`。

**第 4 天（要动策略时）**：`docs/policy-manifest.md` → `docs/design/policy-channel-design.md` → `microduck_rl` 的 `README.md` 与 `src/mjlab_microduck/publish/`。

**一句话总结这个项目的开发逻辑**：*把不变量变成结构（借用检查器、单写者、符号链接原子切换、编译器强制的 crate 边界），把每个非显然的决策连同它防的失败写进注释和一个测试里，把"要去哪一页找答案"变成可推导的规则。*
