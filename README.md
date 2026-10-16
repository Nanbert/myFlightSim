# myFlightSim

一个用 **Rust + Bevy** 写的飞行模拟器外壳，飞行力学默认由 **JSBSim**（6 自由度
FDM）驱动。Bevy 负责渲染、相机、HUD 和输入；JSBSim 负责真正的气动与运动解算。
默认模型是 `c172p`（Cessna 172P）。物理与画面之间由一个 `FlightModel` trait
隔开，见「飞行模型后端」。

设计取向：**把 JSBSim 和机模已经提供的东西暴露出来就好**——能真实地飞起来
就停手，不去追 FlightGear 那种成熟的整机模拟。因此本项目的控件表是数据驱动的
（每机型一份档案，见 `src/aircraft_profile.rs`），机模定义了什么通道，我们就为
它声明什么控制。

## 飞行模型后端

物理与画面之间隔着一个 `FlightModel` trait（`crates/flight-model`）：它是一条
**命名属性总线**——App 往里写操纵指令（`fcs/elevator-cmd-norm` …）、读仪表
（`position/h-sl-ft`、`attitude/psi-rad` …），后端负责推进每一步。因此
`view`/`input`/HUD/自动驾驶完全不知道底下是谁在算。

- 默认后端是 **JSBSim**（`jsbsim-sys`），即现在的成熟物理。
- `FDM_BACKEND=rust` 切到 **纯 Rust 引擎**（`crates/fdm`）。它是为浏览器/wasm
  准备的**替代实现**，目前是骨架（还没有物理，飞机停在原点）。

属性名沿用 JSBSim 的拼写（完整契约见 `crates/flight-model/src/lib.rs`），
所以 JSBSim 后端几乎不需要翻译。

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
高度、姿态、油门、转速等；按 `H` 可切换控制输入叠加层，按 `C` 在机外跟随与
座舱内视角之间切换（机模带内饰时才有座舱视角，见下）。

## 键位

| 按键 | 作用 | JSBSim 通道 |
| --- | --- | --- |
| `W` / `S` | 俯仰（抬头 / 低头） | `fcs/elevator-cmd-norm` |
| `D` / `A` | 横滚（右滚 / 左滚） | `fcs/aileron-cmd-norm` |
| `Q` / `E` | 偏航（左偏 / 右偏） | `fcs/rudder-cmd-norm` |
| `Shift` / `Ctrl` | 油门（固定翼）或总距（直升机） | `fcs/throttle-cmd-norm` / `fcs/collective-cmd-norm` |
| `Z` / `X` | 俯仰配平 | `fcs/pitch-trim-cmd-norm` |
| `F` / `V` | 襟翼放下 / 收起（固定翼） | `fcs/flap-cmd-norm` |
| `B` | 机轮刹车（松开回位） | `fcs/{left,right}-brake-cmd-norm` |
| `T` | 按当前状态重新配平（保留空速与位置） | — |
| `Y` | 重启到巡航（离地并按巡航配平） | — |
| `Space` | 直升机悬停配平（仅 AH-1S） | — |
| `G` | 切换自动驾驶（仅 737） | — |
| `U` | 起落架收放（F-22 / 737） | `gear/gear-cmd-norm` |
| `J` | 推力矢量开关（仅 F-22） | `fcs/tvc-enable` |
| `R` | 重置到跑道 | — |
| `H` | 显示 / 隐藏输入叠加层 | — |
| `C` | 切换跟随相机 / 座舱内视角 | — |

每架飞机的操作键**各自独立**（完全解耦），但共有控制项的默认键写得一致，所以
肌肉记忆可以跨机型延续。缺省的机种（如 C172）用通用的一套；F-22 多出起落架与
推力矢量，AH-1S 用手里的总距取代油门、没有襟翼与刹车，737 才有自动驾驶。
想改键：菜单里选中机型 → `SETTINGS`，用方向键选行、`Enter` 后按新键即可（与已
占用的键会**互换**，不会出现重复或落空）。改动写进配置文件（见下），下次启动
自动读回。

按键符号约定来自 c172p 的气动导数：`+elevator-cmd-norm` 使机头**下压**、
`+aileron-cmd-norm` 使飞机**右滚**、`+rudder-cmd-norm` 使机头**左偏**。每个键都
映射到能给出直观响应的符号，所以命令值取该键“正向”的相反数。平移类轴
（升降舵、副翼、方向舵、刹车）松开即回中；保持类轴（油门、配平、襟翼）按键
累积、松开保持。

## 自动驾驶

只有 **737** 有自动驾驶（真实民航如此；C172 / F-22 / AH-1S 上没有，`G` 无反应）。
按 `G` 接管，键鼠输入交还给自动驾驶。它跑一条地面起飞 → 爬升 → 巡航的任务：
滑跑对正 → 抬轮 → 爬升到 3000 ft → 稳到 110 kt 平飞巡航（航向 0）。逻辑在
`jsbsim-sys/src/autopilot.rs`，无头示例与游戏里跑的是**同一套控制律**。

## 天气

天气是**全局设置**（与机型无关），在菜单的 `SETTINGS → GLOBAL` 里用左右方向键在
晴 / 小雨 / 中雨 / 暴雨之间切换，**没有快捷键**。雨是一池固定在**相机周围的雨条**
（细长四边形，逐帧下落并在盒子边界回收），绕竖轴朝向相机，所以池子大小固定、
不用生成新对象，跟天穹的云是同一套“回收”思路。**地面溅水**是另一池贴地的圆环
（`Annulus`，无贴图），出生即快速扩张并淡出，寿命到点就换到相机附近的随机落点
——只在**相机贴近地面**时出现（万米高空看不到地面涟漪，画出来只会像噪点）。
雨量同时驱动**阴天**：天穹换用更暗更灰、云量更大的预烘焙贴图，太阳与照度也
一并压暗。

天气目前只影响**画面**，没有回灌 JSBSim 的 FDM——c172p 的气动导数不受雨影响，
所以不假装它受影响。若要让雨/风真正改变飞行（湍流、侧风、能见度），可以接
JSBSim 的大气模型，那是另一档工作。

## 设置持久化

天气等级与各机型的键位覆盖写在一个文本配置文件里：`myflightsim/config.cfg`，
位于 `$XDG_CONFIG_HOME`（缺省 `~/.config`）下。格式是手写的简单文本（无 serde
依赖），例如：

```
weather = 2

[737]
elevator = KeyS KeyW
gear = KeyU
```

只保存**改动过**的键，所以以后调整默认键位，对没动过该键的玩家依然生效。读写
走 `config.rs` 里的 `ConfigStore` 抽象：原生写文件，将来浏览器（wasm）版只需换
一个 `localStorage` 实现，其余代码不动。

## 目录结构

```
.
├── src/
│   ├── main.rs        # Bevy App 组装
│   ├── app_state.rs   # Menu / Flying 状态
│   ├── sim.rs         # 飞行模型 <-> Bevy 世界桥接、定步长与插值
│   ├── view.rs        # 场景/相机/HUD、操纵面驱动（骨骼旋转 / 绕铰链线旋转）
│   ├── input.rs       # 键位 -> FCS 命令（按机型档案 + 用户覆盖构建）
│   ├── aircraft_profile.rs # 每机型的控制键位档案（数据表）
│   ├── config.rs      # 用户设置持久化（键位覆盖 + 天气；ConfigStore 抽象）
│   ├── menu.rs        # 开始菜单：选机 + 全局设置页 + 按键编辑器
│   ├── gauge/         # 座舱仪表（UI 组件）：7 个实时表，底角布局 / 座舱面板贴附
│   ├── sky.rs         # 程序化天空：跟随相机的渐变天穹 + fBm 云（无贴图素材）
│   ├── weather.rs     # 天气：跟随相机的雨幕粒子 + 阴天（驱动 sky/view 变暗）
│   └── lens.rs        # 镜头雨滴后处理
├── crates/
│   ├── flight-model/  # FlightModel trait：属性总线接口（后端无关的缝）
│   └── fdm/           # 纯 Rust 飞行力学（wasm 目标；当前为骨架）
├── jsbsim-sys/        # JSBSim 的 Rust 绑定（默认物理后端）
│   ├── src/lib.rs     # 安全封装 (Fdm) 与源码定位
│   ├── src/autopilot.rs
│   ├── shim/shim.cpp  # C ABI 垫片
│   ├── build.rs       # 编译 libJSBSim、链接垫片
│   └── examples/      # takeoff.rs：无头自动驾驶演示
├── examples/          # sim_view_shot.rs、cockpit_shot.rs、
│                      # panel_calib.rs、panel_view_shot.rs
├── assets/            # c172_full.glb、f22.glb…（3D 模型，全部入库）
└── CREDITS.md         # 3D 模型署名
```

## 验证与调试

除游戏本体外，仓库里有几个无头/离屏的验证工具：

- `cargo run -p jsbsim-sys --example takeoff` —— 无窗口跑完整自动驾驶任务，
  打印遥测表（高度/空速/升降率/姿态/油门/phase）。用来证明 FDM 本身能飞出一条
  正常的航线，从而把手感问题归因到“操纵”而非“模拟”。
- `cargo run --example sim_view_shot` —— 离屏渲染场景，逐个偏转副翼 / 升降舵 /
  方向舵各出一张图，核对新机模的操纵面接线；`COCKPIT=1 cargo run --example
  sim_view_shot` 则强制座舱视角，只出一张图，核对内饰框景。
- `cargo run --example cockpit_shot` —— 单独加载大机模 + 自由相机，用来手工调
  座舱眼睛位置（见下）。
- `cargo run --example panel_calib` —— 座舱面板标定：把 `src/gauge/mod.rs` 里
  的 7 个表位重新投回屏幕，在画面上撒红球标记，用来核对/测量面板表盘的中心与
  半径（读数取自 `/tmp/dials.txt`，缺省用内置值）。
- `cargo run --example panel_view_shot` —— 带实时仪表的座舱渲染截图，用来验证
  7 个表是否正好贴在模型画好的表盘上（`COCKPIT=0` 则回到屏幕底角布局对照，
  `LOOK=<角度>` 让座舱视角转过去，检查转头时表是否仍跟着贴在表盘上）。

`jsbsim-sys` 里还有一个单元测试 `c172p_climbs_after_throttle_up`：加载 c172p、
加油门、跑 2 秒，断言飞机确实爬升并保持空速。运行：

```sh
cargo test
```

## 机模与座舱视角

C172 用带内饰的大机模 `assets/c172_full.glb`（机外蒙皮 + 座舱内饰 + 仪表面板，
故支持 `C` 座舱视角），随仓库一起入库。大机模没有骨骼也没有动画：舵面是各自独立
的节点、几何烘焙在同一坐标系里，所有节点原点都落在机体重心，所以 `view.rs` 的
`C172_SURFACES` 表用“绕各自前缘线旋转”（`Transform::rotate_around`）把它们铰接
起来——轴与铰链点由绑定姿态顶点量出来。座舱相机的眼睛位置同样写在 `model_spec`
里（模型坐标系，鼻朝 +Z）。

大机模的面板是**静态贴图**，表针不会动。为了不让屏幕底角的 7 个实时表和贴图画好
的表盘打架，座舱视角下这 7 个表会**贴到面板上**：`gauge/mod.rs` 里为每个表存了一份
机体坐标系下的表盘位置（中心 + 两个半径向量，由 `panel_calib` 量出），
`seat_panel` 每帧把它投影到当前相机上，再把整个控件（表盘、数字、指针）移动、
缩放、旋转到位。因为位置来自相机而不是固定屏幕像素，**自由视角、横滚、改窗口大小
都不会错位**；表盘转到相机背后时该表自动隐藏。留作静态的其余表盘（刹车压力、
GYRO/LOC、备用高度表、VOR、备用升降率等）不动——模型上没有转速表，7 个表是“尽量
对上”，不是一一对应。

座舱视角下支持**鼠标自由环视**：移动鼠标即转动视线（机体系下先偏航后俯仰，俯仰限
±80°），`N` 键把视线回正，`C` 退出座舱视角或按 `Esc` 回菜单时也会自动回正。相机
**位置始终钉在飞行员眼睛上**，转头只改变朝向——面板与地平线只是相对头部平移/旋转，
仪表仍由 `seat_panel` 逐帧重投影贴回原表位。进入座舱视角时鼠标会被隐藏并锁定在窗口
内（得到无边界位移），离开座舱视角、回到菜单或窗口失焦时立即释放。

## 署名

3D 模型 `assets/c172_full.glb`（作者 KOG_THORNS）来自 Sketchfab，采用
**CC BY 4.0** 许可。完整的 TASL 署名见 [CREDITS.md](CREDITS.md)。
