# Bug 记录

这里记录**已观察到的 bug**和待验证的回归。功能缺口（例如“混合比未建模”、
“ATC 未做”）仍写在各机型教程的 TODO 里，不混在这里。

状态用 `Open` / `Investigating` / `Fixed` / `Deferred`。

---

## BUG-001 C172 同时开 BANK HOLD 和 TURN COORD 时左右振荡

- **状态**：Open（2026-10-19，按用户要求先不修）
- **机型/后端**：C172 / Rust FDM / 键盘
- **设置**：`BANK HOLD = ON`，`TURN COORD = ON`
- **复现**：
  1. C172 → `SETTINGS`，把 `BANK HOLD` 和 `TURN COORD` 都设成 `ON`；
  2. 起飞或爬升到有速度的空中；
  3. 用 `A/D` 建立约 15–30° 坡度，松手让 bank hold 捕获；
  4. 观察：飞机左右滚转/偏航来回摆，形成持续振荡，而不是一次收敛。
- **期望**：bank hold 稳住坡度，turn coordination 只把滑球回中，二者应该收敛。
- **实际**：两个 assist 同时工作时出现低频左右振荡。单独开其中一个时不明显
  （待确认）。
- **临时规避**：只开一个 assist，或先关掉 `TURN COORD`。
- **可能原因（未验证）**：
  1. 两个闭环耦合：bank hold 动副翼 → adverse yaw / 侧滑 → turn coordination
     动方向舵 → 方向舵的滚转/偏航耦合又改变坡度 → bank hold 再修，形成 limit cycle。
  2. 增益偏大：bank hold `P = 0.008/deg`、`D = 0.012/(deg/s)`；coordination
     `P = 8.0`（对 slip）。
  3. 相位滞后：assist 在 `Update` 里按上一固定步的状态计算，FDM 是 120 Hz；
     `Update` 帧率变化会引入相位，容易激励振荡。
  4. 没有滤波/速率限制：bank、roll rate、slip 直接进 P 项，噪声和延迟会放大。
  5. bank target 捕获后，coordination 的方向舵会改变坡度，bank hold 又追，
     两者可能互相驱动。
- **修复方向（以后）**：
  - 把 assist 控制律移到 `FixedUpdate`，和 FDM 同为 120 Hz；
  - 对 bank / roll rate / slip 做低通，并给 assist 输出加速率限制；
  - 先单独调 bank hold，再调 coordination；降低增益；
  - coordination 只在 bank hold 关闭时工作，或两者之间加解耦/互斥；
  - 用 yaw-rate damping + slip PI，而不是纯 slip P；
  - 加一个 headless 回归：建立 20° 坡度后 20 s 内 `|bank - target|` 收敛，
    且 `|slip|` 有界。
- **相关代码**：`src/assist.rs`、`src/input.rs`（`Binding::assist`）、
  `src/controls.rs`（BANK/COORD 显示）。

---

## 记录模板

```text
## BUG-NNN 标题

- 状态：
- 机型/后端：
- 复现：
  1.
- 期望：
- 实际：
- 临时规避：
- 可能原因：
- 修复方向：
- 相关代码：
```
