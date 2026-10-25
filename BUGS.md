# Bug 记录

这里记录**已观察到的 bug**和待验证的回归。功能缺口（例如“混合比未建模”、
“ATC 未做”）仍写在各机型教程的 TODO 里，不混在这里。

状态用 `Open` / `Investigating` / `Fixed` / `Deferred`。

---

## BUG-002 回主菜单后，飞行中的浮层面板不消失

- **状态**：Fixed（2026-10-25）
- **机型/后端**：全部（UI 层，与机型无关）
- **复现**：
  1. 起飞前让 BEFORE TAKEOFF 卡片显示（C172/737 地面、未推满油门即满足）；
  2. Esc → 回主菜单；
  3. 观察：checklist 卡片仍悬在主菜单画面左下角。
- **期望**：回到主菜单后，所有飞行中的浮层都应隐藏。
- **实际**：BEFORE TAKEOFF 卡片残留。同病的还有两个面板：737 的
  `1000 FT GATE` 稳定/不稳定横幅（approach.rs，越门后数秒内退出即残留）和
  `HOP LEDGER` 面板（hop.rs，737 整个飞行期间都显示，必然残留）。
- **临时规避**：无（重新进入飞行后恢复受控）。
- **可能原因**：这三个面板都在 `Startup` 只生成一次，`display` 只由
  `run_if(unpaused)` 的系统写回——回主菜单后该系统停跑，面板就停在最后一帧
  的显示状态。对照之下，takeoff/landing/taxi/lesson_flow 都有
  `OnExit(AppState::Flying)` 的隐藏系统，hud/gauge/controls 的同步系统不设
  `run_if`、自己判断状态，combat/mcp/pfd 的实体带 `DespawnOnExit`，均无此
  问题。
- **修复方向**：照 takeoff/landing 的模式，给三个模块补
  `OnExit(AppState::Flying)` 的 hide 系统，把 `display` 置回
  `Display::None`。已修：`checklist.rs`（hide_panel）、`approach.rs`
  （hide_banner）、`hop.rs`（hide_ledger）。
- **相关代码**：`src/checklist.rs`、`src/approach.rs`、`src/hop.rs`。

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
