# myFlightSim

用 **Rust + Bevy** 编写的开源飞行模拟器。桌面与浏览器同源，默认使用项目自带的纯 Rust 飞行力学，内置 4 架飞机、可操作的系统与分级飞行课程。

**在线试玩：<https://nanbert.github.io/myFlightSim/>**（模型与音频按需加载，资源合计约 220 MB）

[![License: LGPL-2.1](https://img.shields.io/badge/license-LGPL--2.1-blue.svg)](LICENSE)

## 特性

- **四架飞机** — Cessna 172P、Boeing 737、F-22A、AH-64D，各自拥有独立的键位、座舱、HUD 与飞行特性。
- **双飞行力学后端** — 默认纯 Rust FDM（`crates/fdm`，可在 wasm 中运行）；`FDM_BACKEND=jsbsim` 可切换到 JSBSim。
- **可操作的系统** — 737 的 MCP、自动驾驶、ILS、减速板、反推与操纵台；C172 的冷启动、检查单与灯光；AH-64D 的悬停与街机模式；F-22 的武器与空战。
- **分级课程与评分** — C172 M1–M3、737 M1–M6；滑行、起飞、落地、复飞、中断起飞与自动化进近逐项打分，737 另有航程账本。
- **完整环境与跨平台** — 程序化机场与任务地图、PAPI/ILS、可调天气与时段、雨云与夜航灯光；原生窗口和 Web 均可运行，设置分别持久化到文件与 `localStorage`。

## 快速开始

### 本地运行（纯 Rust，无需 C++ 依赖）

```sh
cargo run --no-default-features
```

首次编译 Bevy 需要几分钟。

### 使用 JSBSim 后端

需要一份 JSBSim 源码、CMake 和支持 C++17 的编译器：

```sh
export JSBSIM_SRC=$HOME/gitrep/jsbsim
FDM_BACKEND=jsbsim cargo run
```

`jsbsim-sys` 会从该目录编译 `libJSBSim.so` 并动态链接。未设置 `JSBSIM_SRC` 时，依次探测 `~/gitrep/jsbsim`、`~/jsbsim`、`~/src/jsbsim`、`/usr/local/src/jsbsim`、`/opt/jsbsim`。

### 浏览器版

```sh
rustup target add wasm32-unknown-unknown
cargo install trunk --locked

trunk serve     # http://localhost:8080/myFlightSim/
trunk build     # 输出到 dist/
```

`Trunk.toml` 已固定 `--no-default-features` 与 `/myFlightSim/` 公共路径；Web 版不包含 JSBSim。

## 机型与课程

| 机型 | 内容 |
| --- | --- |
| C172 | 自由飞行；M1 滑行（冷启动）、M2 起飞、M3 落地 |
| 737 | 自由飞行；M1 滑行、M2 起飞、M3 落地、M4 复飞、M5 中断起飞、M6 自动化进近 |
| F-22 | `FREE FLY` 真实气动；`MISSIONS` 街机空战 |
| AH-64D | 自由飞行；`SETTINGS` 中可切换 Realistic / Arcade |

菜单使用方向键选择、`Enter` 确认、`Esc` 返回；各机型的 `TUTORIAL` 页包含操作说明、真机程序与尚未建模的项目。

## 操作

| 按键 | 作用 |
| --- | --- |
| `W` / `S` | 俯仰 |
| `A` / `D` | 横滚 |
| `Q` / `E` | 偏航 |
| `Shift` / `Ctrl` | 油门（F-22 为 `Space` / `Ctrl`；AH-64D 为总距） |
| `Z` / `X` | 俯仰配平 |
| `F` / `V` | 襟翼（C172 / 737） |
| `B` | 机轮刹车（AH-64D 街机为自动平衡） |
| `U` | 起落架（737 / F-22） |
| `T` | 按当前状态重新配平 |
| `H` | 显示 / 隐藏输入叠加层 |
| `C` | 切换舱外 / 座舱视角 |
| `N` | 座舱视线回正 |
| `O` | 开关 HUD（F-22 / AH-64D） |
| `Esc` | 暂停 / 返回 |

机型专属：

- **C172**：`M` 总开关，按住 `I` 启动机，`L` 灯光。
- **737**：`G` 自动驾驶，`1`–`6` 对应 MCP 的 HDG / ALT / V/S / LVL CHG / A-T / APP，`[` `]` 调节速度，`M` 切换 Speed/Mach，`L` 减速板，`K` 反推，`;` 操纵台。
- **AH-64D**：`J` 自动悬停 / 垂直起降，`B` 自动平衡，`Space` 旋翼配平。
- **F-22**：`J` 推力矢量，`Tab` 锁定 / 切换目标，`Enter` 发射导弹，`K` 切换导弹，按住 `G` 机炮，`L` 热焰弹，`R` 切换雷达距离档。

键位按机型固定，不提供游戏内改键。座舱视角下可移动鼠标环视，左键操作面板与仪表。

## 项目结构

```
src/                  Bevy 应用：菜单、飞行桥接、HUD/座舱、课程与评分
crates/flight-model/  FlightModel 属性总线（后端无关）
crates/fdm/           纯 Rust 飞行力学（默认后端，wasm 可用）
jsbsim-sys/           JSBSim 绑定与可选后端（仅桌面）
assets/               模型、贴图、音频、着色器
examples/             离屏截图与标定工具（--features shots）
```

物理与画面之间由 `FlightModel` 的命名属性总线隔开，因此输入、相机、HUD 和自动驾驶不依赖具体后端。设计取向是把 JSBSim 和机模已经提供的能力真实地暴露出来，而不是复刻一套完整的整机模拟。

## 开发

```sh
cargo test --no-default-features   # 纯 Rust 测试
cargo test                         # 含 JSBSim（需要 JSBSIM_SRC）

cargo run -p jsbsim-sys --example takeoff   # 无头自动驾驶演示
```

截图与标定工具挂在 `shots` feature 上，按需单独构建：

```sh
cargo run --features shots --example menu_shot
```

## 许可与署名

代码以 **LGPL-2.1** 发布，见 [`LICENSE`](LICENSE) 与 [`NOTICE.md`](NOTICE.md)。`crates/fdm` 的气动数据转录自 JSBSim（LGPL-2.1）。

`assets/` 不在上述许可范围内：模型、音频与着色器按各自条款分发，其中两件为 **CC BY-NC-SA**，因此本项目及任何构建**不得商用**。完整署名见 [`CREDITS.md`](CREDITS.md)。
