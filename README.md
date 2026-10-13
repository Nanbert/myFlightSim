# myFlightSim

一个用 **Rust + Bevy** 写的飞行模拟器外壳，飞行力学由 **JSBSim**（6 自由度
FDM）驱动。Bevy 负责渲染、相机、HUD 和输入；JSBSim 负责真正的气动与运动解算。
默认模型是 `c172p`（Cessna 172P）。

设计取向：**把 JSBSim 和机模已经提供的东西暴露出来就好**——能真实地飞起来
就停手，不去追 FlightGear 那种成熟的整机模拟。因此本项目的控件表是数据驱动的
（见 `src/input.rs`），机模定义了什么通道，我们就接什么通道。

## 前置依赖

本项目**不自带** JSBSim，而是从一个本地源码检出（checkout）编译并链接：

- 一份 **JSBSim 源码**（需要同时提供编译产物与运行期数据目录
  `aircraft/`、`engine/`、`systems/`）
- **CMake**
- 支持 **C++17** 的编译器

`jsbsim-sys/build.rs` 会从该检出编译出 `libJSBSim.so`，链接到本地
`<target>/jsbsim-build/` 下（与具体 profile 无关，便于运行期通过 rpath 找到）。

### 指定 JSBSim 源码位置

按以下优先级解析（`build.rs` 与运行期 `jsbsim_sys::jsbsim_src` 逻辑一致）：

1. 环境变量 **`JSBSIM_SRC`**（若设置则必须有效，否则报错退出）；
2. 依次探测常见位置：`$HOME/gitrep/jsbsim`、`$HOME/jsbsim`、`$HOME/src/jsbsim`、
   `/usr/local/src/jsbsim`、`/opt/jsbsim`。

有效检出的判定标志是目录下存在 `src/FGFDMExec.h`。都找不到时会 panic，并列出
探测过的路径。推荐显式设置：

```sh
export JSBSIM_SRC=$HOME/gitrep/jsbsim
```

## 构建与运行

```sh
cargo run
```

首次运行会先编译 JSBSim（耗时较长），之后增量构建很快。左上角 HUD 显示空速、
高度、姿态、油门、转速等；按 `H` 可切换控制输入叠加层。

## 键位

| 按键 | 作用 | JSBSim 通道 |
| --- | --- | --- |
| `W` / `S` | 俯仰（抬头 / 低头） | `fcs/elevator-cmd-norm` |
| `D` / `A` | 横滚（右滚 / 左滚） | `fcs/aileron-cmd-norm` |
| `Q` / `E` | 偏航（左偏 / 右偏） | `fcs/rudder-cmd-norm` |
| `Shift` / `Ctrl` | 油门加 / 减 | `fcs/throttle-cmd-norm` |
| `Z` / `X` | 俯仰配平 | `fcs/pitch-trim-cmd-norm` |
| `F` / `V` | 襟翼放下 / 收起 | `fcs/flap-cmd-norm` |
| `B` | 机轮刹车（松开回位） | `fcs/{left,right}-brake-cmd-norm` |
| `G` | 切换自动驾驶 | — |
| `R` | 重置到跑道、回到自动驾驶 | — |
| `H` | 显示 / 隐藏输入叠加层 | — |

按键符号约定来自 c172p 的气动导数：`+elevator-cmd-norm` 使机头**下压**、
`+aileron-cmd-norm` 使飞机**右滚**、`+rudder-cmd-norm` 使机头**左偏**。每个键都
映射到能给出直观响应的符号，所以命令值取该键“正向”的相反数。平移类轴
（升降舵、副翼、方向舵、刹车）松开即回中；保持类轴（油门、配平、襟翼）按键
累积、松开保持。

## 自动驾驶

按 `G` 接管，键鼠输入交还给自动驾驶。它跑一条地面起飞 → 爬升 → 巡航的任务：
滑跑对正 → 抬轮 → 爬升到 3000 ft → 稳到 110 kt 平飞巡航（航向 0）。逻辑在
`jsbsim-sys/src/autopilot.rs`，无头示例与游戏里跑的是**同一套控制律**。

## 目录结构

```
.
├── src/
│   ├── main.rs        # Bevy App 组装
│   ├── sim.rs         # JSBSim <-> Bevy 世界桥接、定步长与插值
│   ├── view.rs        # 场景/相机/HUD、操纵面骨骼驱动
│   └── input.rs       # 键位 -> JSBSim 命令（数据驱动绑定表）
├── jsbsim-sys/        # JSBSim 的 Rust 绑定
│   ├── src/lib.rs     # 安全封装 (Fdm) 与源码定位
│   ├── src/autopilot.rs
│   ├── shim/shim.cpp  # C ABI 垫片
│   ├── build.rs       # 编译 libJSBSim、链接垫片
│   └── examples/      # takeoff.rs：无头自动驾驶演示
├── examples/          # confirm_controls.rs、sim_view_shot.rs
├── assets/            # cessna_172.glb
└── CREDITS.md         # 3D 模型署名
```

## 验证与调试

除游戏本体外，仓库里有几个无头/离屏的验证工具：

- `cargo run -p jsbsim-sys --example takeoff` —— 无窗口跑完整自动驾驶任务，
  打印遥测表（高度/空速/升降率/姿态/油门/phase）。用来证明 FDM 本身能飞出一条
  正常的航线，从而把手感问题归因到“操纵”而非“模拟”。
- `cargo run --example confirm_controls` —— 直接驱动 `cessna_172.glb` 的操纵面
  骨骼（不经过 JSBSim），写出一张静置图和一张偏转图，用来核对模型绑定是否正确。
- `cargo run --example sim_view_shot` —— 离屏渲染一帧，用于查看场景与操纵面姿态。

`jsbsim-sys` 里还有一个单元测试 `c172p_climbs_after_throttle_up`：加载 c172p、
加油门、跑 2 秒，断言飞机确实爬升并保持空速。运行：

```sh
cargo test
```

## 署名

3D 模型 `assets/cessna_172.glb` 来自 Sketchfab（作者 Viva_1），采用
**CC BY 4.0** 许可。完整的 TASL 署名见 [CREDITS.md](CREDITS.md)。
