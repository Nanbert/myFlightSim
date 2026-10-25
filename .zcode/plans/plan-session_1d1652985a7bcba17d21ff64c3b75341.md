# INFO 页充实 + 教程收紧

## 1. INFO 页：简史 + 真实规格表（替换 "coming soon" 空壳）

**实现**：`src/tutorial.rs` 加 `pub fn info_lines(id: &str) -> Vec<PageLine>`，内容由静态表 `INFO_PAGES: &[InfoPage]` 驱动（按 normalized id 匹配四机）。结构：heading（机名）→ 简史 3-5 行 → blank → heading "SPECS" → 规格行（`PageLine::option(label, value, false)`，渲染成 FEEL 页那种左右两列，无高亮）。全英文 ASCII（菜单字体限制，与全游戏一致）。

四机内容（公开常识数据，ASCII）：
- **C172 Skyhawk**：1955 首飞、史上产量最高飞机（45,000+ 架）、至今仍在生产、全球航校主力；规格：翼展 36 ft 1 in、长 27 ft 2 in、Lycoming IO-360 160 hp、巡航 122 kt、升限 13,500 ft、航程 ~640 NM、MTOW 2,550 lb。
- **737**：1967 首飞、交付 11,000+（产量最高的喷气客机）、本机为 -800 级（NG，1997，CFM56-7B）；规格：翼展 117 ft 5 in（带翼梢小翼）、长 129 ft 6 in、2×CFM56-7B、巡航 M0.785、升限 41,000 ft、航程 2,935 NM、MTOW 155,500 lb、162-189 座。
- **F-22**：YF-22 1990 / 首飞 1997、2005 服役、187 架后停产；规格：翼展 44 ft 6 in、长 62 ft、2×PW F119（加力 35,000 lbf 级）、最大 M2.25（超巡 1.8）、升限 ~65,000 ft、MTOW 83,500 lb。注明 public figures。
- **AH-64D Apache Longbow**：YAH-64 1975 / 1986 服役、D 型 1997 起、各型 1,200+；规格：旋翼直径 48 ft、机身长 49 ft、2×GE T700-701C（~1,890 shp）、巡航 150 kt、最大 229 kt、升限 21,000 ft、MTOW 23,000 lb、双人机组。

**接线**：`menu.rs` 的 `update_menu_ui` 加 `Some(Page::Info) => tutorial::info_lines(sel.aircraft_id())`（放在 freeform 兜底前）；`page_text` 的 `Page::Info` 占位臂删掉、并入 `String::new()` 组（同 McpGuide 先例，注明专属臂在别处）。

## 2. 737 TUTORIAL：每课段尾加一行 SCORE 线

M1–M6 每段末尾追加一行（前缀 `SCORE:`，88 列内），把满分线/权重点出来，正文不动：
- M1：`SCORE: LINE 0.40 (stop margin) - CENTRELINE 0.30 - SPEED 0.20 - THRUST 0.10.`
- M2：`SCORE: ROTATION 0.30 - CLEAN-UP 0.20 - AUTOMATION 0.20 - RLW 0.15 - CLIMB 0.15.`
- M3：`SCORE: TOUCH 0.45 (rate/point/speed/centre) - ROLLOUT 0.20 - GS 0.15 - LOC 0.10 - SPEED 0.10.`
- M4：`SCORE: REACT <=2 s - PITCH 8-20 deg - GEAR <=3 s of rate - FLAPS 15 by 2,000 ft - LEVEL +-150 ft.`
- M5：`SCORE: REACT <=2 s - PEAK <=110 kt - STOP <=10 s - BUCKETS <=2 s - the centreline.`
- M6：`SCORE: windows set BEFORE the press - stop <=3 s - tight intercept (dev 0.10) - A/T SPD 180.`

## 3. TODO 段全删（四机）

删除 C172 / F-22 / AH-64D / 737 的 "NOT MODELLED YET (TODO)" 段（合计约 35 行）。连带微调：`mcp_guide.rs` Decor 主题末行 "The sim's own TODO list carries them all..." 改为 "The lessons fly the modes the sim has - and fly them well."（引用落空）。

## 触点清单

1. `src/tutorial.rs` — `INFO_PAGES` 表 + `info_lines()`；四机 TODO 段删除；737 六条 SCORE 行。
2. `src/menu.rs` — `Page::Info` 内容臂改 `tutorial::info_lines`；`page_text` 的 Info 臂并入 `String::new()` 组。
3. `src/mcp_guide.rs` — Decor 末行微调。
4. **测试** — `info_lines` 的 ASCII + 行宽 + 每机非空/SPECS 存在测试；现有 ASCII 测试自动覆盖 SCORE 行。

## 明确不做

- C172 等其他机的教程不加 SCORE 行（用户只问 737；其余机课业结构不同）。
- 不做中文、不加"模拟器范围"小节（用户选了纯真实数据）。
- 不动 M1–M6 段正文与 REAL xxx PROCEDURE 段（公开资料，保留）。