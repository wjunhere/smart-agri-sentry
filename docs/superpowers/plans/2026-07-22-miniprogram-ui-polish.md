# 微信小程序四页优化 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 风格不变（复用 app.less 全部 token）优化小程序四页：三态反馈、排版细节、布局微调、纯 CSS 微交互。

**Architecture:** 新增一个共享 `state-block` 组件承载 loading/empty/offline 三态；四页 wxml 只插入组件与调类名；全部动效用 CSS transition/:active；状态判定仅用现有 store 字段。

**Tech Stack:** 微信原生小程序（TS + Less，当前 WebView 渲染，写法兼容 Skyline）；验证用 wechatide 自动化截图 + `npx tsc --noEmit`（基线 46 个既有错误，不得新增）。

**Spec:** `docs/superpowers/specs/2026-07-22-miniprogram-ui-polish-design.md`

**关键约束（评审结论）：**
- **不改 `app.less` 的共享 `.card` 内边距**（会波及 waypoint-editor 弹层）；卡片间距统一通过各页 `.page { gap: 16rpx }` 逐页调整
- 组件文件结构照抄 `components/status-badge/`（4 文件：.ts/.wxml/.less/.json）
- 无数据显示 `·`；`--` 只表示"连接中"
- 验证工具链：`"/b/wechat_devtools/微信web开发者工具/wechatide.cmd" -c Kimi -t <tool> --project E:/smart_agri_sentry/wechat`；截图存 `verify_shots/`

---

### Task 1: state-block 共享组件

**Files:**
- Create: `wechat/miniprogram/components/state-block/state-block.ts`
- Create: `wechat/miniprogram/components/state-block/state-block.wxml`
- Create: `wechat/miniprogram/components/state-block/state-block.less`
- Create: `wechat/miniprogram/components/state-block/state-block.json`

- [ ] **Step 1: 写组件四文件**

`state-block.json`：
```json
{ "component": true, "usingComponents": {} }
```

`state-block.ts`：
```typescript
Component({
  properties: {
    type: { type: String, value: 'empty' },   // loading | empty | offline
    text: { type: String, value: '' },
    subtext: { type: String, value: '' },
  },
})
```

`state-block.wxml`：
```xml
<view class="sb sb-{{type}}">
  <view wx:if="{{type === 'loading'}}" class="sb-spinner"></view>
  <text wx:elif="{{type === 'empty'}}" class="sb-icon">◌</text>
  <text class="sb-text">{{text}}</text>
  <text wx:if="{{subtext}}" class="sb-sub">{{subtext}}</text>
</view>
```

`state-block.less`：
```less
.sb {
  display: flex; flex-direction: column; align-items: center; justify-content: center;
  gap: 8rpx; padding: 32rpx 20rpx;
}
.sb-offline {
  align-items: flex-start;
  border-left: 4rpx solid var(--red);
  padding-left: 20rpx;
}
.sb-spinner {
  width: 40rpx; height: 40rpx; border-radius: 50%;
  border: 4rpx solid var(--border-hi); border-top-color: var(--blue);
  animation: sb-spin 0.9s linear infinite;
}
@keyframes sb-spin { to { transform: rotate(360deg); } }
.sb-icon { font-size: 40rpx; color: var(--text-muted); }
.sb-text { font-size: 24rpx; color: var(--text-dim); }
.sb-offline .sb-text { color: var(--red); }
.sb-sub { font-size: 20rpx; color: var(--text-muted); }
```

- [ ] **Step 2: tsc 检查**

Run: `cd wechat && npx tsc --noEmit -p tsconfig.json 2>&1 | grep -c "error TS"`
Expected: `46`（与基线一致）

- [ ] **Step 3: Commit**

```bash
git add wechat/miniprogram/components/state-block/
git commit -m "feat(miniprogram): add shared state-block component (loading/empty/offline)"
```

---

### Task 2: 控制页——状态条合并 + dpad 加大 + 巡航按钮禁用态 + 按压反馈

**Files:**
- Modify: `wechat/miniprogram/pages/control/control.wxml`
- Modify: `wechat/miniprogram/pages/control/control.less`

- [ ] **Step 1: wxml——合并连接角标进模式卡 + 巡航按钮禁用态**

现状 `control.wxml` 顶部（独立的 `.conn-badge` view + "当前模式"卡）替换为：

```xml
<view class="page">
  <view class="card row-between">
    <text class="label">当前模式</text>
    <view class="status-right">
      <text class="conn-dot {{connected ? 'conn-ok' : 'conn-bad'}}">●</text>
      <text class="mono dim" style="font-size:20rpx">{{carIp}}</text>
      <status-badge text="{{mode}}" type="{{mode === 'AUTO' ? 'green' : 'blue'}}" />
    </view>
  </view>
```

（删除文件顶部独立的 `conn-badge` view。）

巡航按钮组改为禁用态防连点（含 `cruising` 态——后端状态机含 cruising，巡航中禁止重复触发启动脚本）：

```xml
    <view style="display:flex;gap:12rpx">
      <button size="mini" bindtap="onStackPreheat"
              disabled="{{stackState !== 'idle' && stackState !== 'error'}}">预热</button>
      <button size="mini" type="primary" bindtap="onStackStart"
              loading="{{stackState === 'preheating' || stackState === 'starting'}}"
              disabled="{{stackState !== 'idle' && stackState !== 'error'}}">启动巡航</button>
      <button size="mini" type="warn" bindtap="onStackStop"
              loading="{{stackState === 'stopping'}}"
              disabled="{{stackState === 'stopping' || stackState === 'idle'}}">停止巡航</button>
    </view>
```

- [ ] **Step 2: less——dpad 加大、状态条样式、按压反馈、删除 conn-badge 样式**

`control.less` 中：
- `.dpad` 的 `grid-template-columns/rows` `80rpx` → `96rpx`，`gap: 6rpx` → `8rpx`
- `.dpad-btn` 的 `width/height: 80rpx` → `96rpx`
- `.dpad-btn:active` 追加 `transform: scale(0.96); transition: transform 100ms;`
- `.estop-btn:active` 已有 scale(0.95)，补 `transition: transform 100ms;`
- 删除 `.conn-badge/.conn-ok/.conn-bad` 三条规则，替换为：

```less
.status-right { display: flex; align-items: center; gap: 12rpx; }
.conn-dot { font-size: 20rpx; transition: opacity 200ms; }
.conn-ok  { color: var(--green); }
.conn-bad { color: var(--red); }
.pill:active, .tag:active { transform: scale(0.96); transition: transform 100ms; }
```

- `.page` 的 `gap: 12rpx` → `16rpx`

- [ ] **Step 3: tsc + 截图验证**

Run: `cd wechat && npx tsc --noEmit -p tsconfig.json 2>&1 | grep -c "error TS"` → `46`

Run: `"/b/wechat_devtools/微信web开发者工具/wechatide.cmd" -c Kimi -t simulator_open_page --project E:/smart_agri_sentry/wechat --page pages/control/control`，然后 `automation_viewport_action --action screenshot --wait-seconds 3 --path E:/smart_agri_sentry/verify_shots/polish_01_control.png`
Expected: 状态条一行（模式+连接点+IP）、dpad 明显加大、风格与基线一致

- [ ] **Step 4: Commit**

```bash
git add wechat/miniprogram/pages/control/
git commit -m "feat(miniprogram/control): merged status bar, larger dpad, cruise button busy states, press feedback"
```

---

### Task 3: 监测页——视频区 16:9 + 三态 + 数值占位与过渡

**Files:**
- Modify: `wechat/miniprogram/pages/monitor/monitor.wxml`
- Modify: `wechat/miniprogram/pages/monitor/monitor.less`
- Modify: `wechat/miniprogram/pages/monitor/monitor.ts`
- Modify: `wechat/miniprogram/pages/monitor/monitor.json`（注册组件）

- [ ] **Step 1: 确认渲染模式**

Run: `grep -n '"renderer":' wechat/miniprogram/app.json wechat/miniprogram/pages/monitor/monitor.json`
（注意精确匹配 `"renderer":`——`rendererOptions` 不算。已核实：app.json 无 `"renderer": "skyline"`，当前为 **WebView 渲染**，padding hack 与 `aspect-ratio` 均可用。）16:9 用 padding-top hack（兼容最稳）：`.camera-wrap { position: relative; width: 100%; padding-top: 56.25%; }`，内部帧容器 `position: absolute; inset: 0`。

- [ ] **Step 2: monitor.ts——暴露 connected 与数值占位（保留 format 单位）**

`sync()` 的 setData 增加 `connected: s.connected`。数值字段**必须保留现有 `utils/format.ts` 的格式化函数**（`formatTemp/formatHumidity/formatCO2/formatNPK`，附加 °C/%/ppm 单位），只在空值分支改占位：

```typescript
airTemp: s.envAirTemp == null ? (s.connected ? '·' : '--') : formatTemp(s.envAirTemp),
```

对每个传感器字段同样处理（airHumidity/co2/soilTemp/soilN/soilP/soilK/leafWetness，各用其对应的 format 函数）。

- [ ] **Step 3: wxml——视频区三态（保留绝对定位覆盖层）**

`monitor.json` 的 `usingComponents` 加 `"state-block": "/components/state-block/state-block"`（绝对路径，与其他页一致）。

`camera-loading` 块替换为**带覆盖容器的** state-block（组件自身无定位，直接放 in-flow 会被 padding-top 推到视频区下方、撑破 16:9）：

```xml
    <view wx:if="{{!connected}}" class="camera-overlay">
      <state-block type="offline" text="未连接到小车" subtext="请检查控制页的 IP 设置" />
    </view>
    <view wx:elif="{{cameraLoading}}" class="camera-overlay">
      <state-block type="loading" text="视频连接中…" />
    </view>
```

- [ ] **Step 4: less——16:9、覆盖层、数值过渡、卡片间距**

- `.camera-wrap` 按 Step 1 结论改 16:9；删除原固定高度
- 新增 `.camera-overlay { position: absolute; inset: 0; z-index: 10; display: flex; align-items: center; justify-content: center; }`
- `.s-val` 追加 `transition: color 300ms;`
- `.page` gap 统一 16rpx
- 删除被 state-block 取代的 `.camera-loading/.spinner` 旧规则（若已无引用）

- [ ] **Step 5: tsc + 截图验证**

tsc → `46`。截图 `polish_02_monitor.png`：视频区 16:9、无硬件时显示 offline 或 loading 态（取决于小车是否在线）、卡片间距一致。

- [ ] **Step 6: Commit**

```bash
git add wechat/miniprogram/pages/monitor/
git commit -m "feat(miniprogram/monitor): 16:9 video area, state-block tri-states, value transitions"
```

---

### Task 4: 分析页——state-block 替换占位 + 长文排版 + 建议序号色块

**Files:**
- Modify: `wechat/miniprogram/pages/analysis/analysis.wxml`
- Modify: `wechat/miniprogram/pages/analysis/analysis.less`
- Modify: `wechat/miniprogram/pages/analysis/analysis.json`（注册组件）

- [ ] **Step 1: wxml 替换两处占位**

`analysis.json` 注册 `state-block`。

- 分类概率卡：`wx:for="{{probs}}"` 下方加 `<state-block wx:if="{{!probs.length}}" type="empty" text="暂无诊断数据" subtext="启动巡航并检测到植株后生成" />`
- 风险趋势卡：`<view class="trend-placeholder mono muted">数据收集中...</view>` → `<state-block type="empty" text="数据收集中…" />`

- [ ] **Step 2: less——长文排版 + 序号色块**

（`.llm-summary` 现有规则已是 26rpx/1.7/`--text`，**跳过不重复写**。）序号不依赖 WXSS `attr()`（Skyline 兼容性存疑），保留 wxml 里的 `{{index+1}}` 文本节点，仅用 CSS 做色块：

```less
.llm-step {
  display: flex; align-items: flex-start; gap: 12rpx;
  font-size: 24rpx; color: var(--text-dim); line-height: 1.7; margin-top: 12rpx;
}
.step-idx {
  flex-shrink: 0; width: 32rpx; height: 32rpx; border-radius: var(--radius);
  background: var(--blue); color: #000; font-size: 20rpx; font-weight: 700;
  display: flex; align-items: center; justify-content: center;
}
.page { gap: 16rpx; }
```

wxml 中 `.llm-step` 改为 `<view class="llm-step"><text class="step-idx">{{index+1}}</text><text>{{item}}</text></view>`；农艺建议的 steps 列表同样处理。

- [ ] **Step 3: tsc + 截图验证**

tsc → `46`。截图 `polish_03_analysis.png`：空卡显示 state-block、LLM 报告排版改善（可先用 callMethod onDeepAnalysis 触发一次真实报告再截图）。

- [ ] **Step 4: Commit**

```bash
git add wechat/miniprogram/pages/analysis/
git commit -m "feat(miniprogram/analysis): state-block placeholders, readable AI report layout, numbered suggestions"
```

---

### Task 5: 天气页——offline 态 + 占位符

**Files:**
- Modify: `wechat/miniprogram/pages/weather/weather.wxml`
- Modify: `wechat/miniprogram/pages/weather/weather.ts`
- Modify: `wechat/miniprogram/pages/weather/weather.less`
- Modify: `wechat/miniprogram/pages/weather/weather.json`（注册组件）

- [ ] **Step 1: weather.ts——失败标志与占位**

data 增加 `loadFailed: false`。`fetchWeather()` 的 try/catch 中：catch 分支 `this.setData({ loadFailed: true })`，成功分支置 false。当前卡湿度取 `s.weatherDays[0].humidity`（现有 sync 里硬编码 `'--'`，补上映射），空值显示 `·`。

- [ ] **Step 2: wxml——offline 块**

`weather.json` 注册 `state-block`。数据区最上方加：

```xml
  <state-block wx:if="{{loadFailed}}" type="offline"
               text="天气数据获取失败" subtext="检查小车连接后下拉重试" />
```

当前卡的"湿度 {{humidity}} · 东风 3级"中 humidity 空值显示 `·`。

- [ ] **Step 3: less——`.page` gap 16rpx；无需其他改动**

- [ ] **Step 4: tsc + 截图验证**

tsc → `46`。截图 `polish_04_weather.png` 与基线对比（风格不变、间距更匀）。

- [ ] **Step 5: Commit**

```bash
git add wechat/miniprogram/pages/weather/
git commit -m "feat(miniprogram/weather): offline state, placeholder dots, spacing"
```

---

### Task 6: 整体验证 + 文档

- [ ] **Step 1: 四页连续截图**，与 `verify_shots/01-04` 基线逐张对比，确认：风格不变、三态正确、无布局破损

- [ ] **Step 2: offline 态实测**：控制页 IP 改成错误地址保存 → 角标变红、监测页 offline 块出现 → 改回 `10.66.175.213` 恢复（可用 automation_evaluate 写 storage + simulator_refresh 替代手点）

- [ ] **Step 3: 更新上下文**：`.claude/PROJECT_CONTEXT.md` 当前重点加一条（2026-07-22 UI 优化）；`docs/TODO.md` 近期已完成加一条

- [ ] **Step 4: Commit**

```bash
git add .claude/PROJECT_CONTEXT.md docs/TODO.md
git commit -m "docs: update context for miniprogram UI polish"
```

---

## 风险与备注

- **Skyline 兼容**：`aspect-ratio` 在 Skyline 支持不全，16:9 优先 padding-top hack；`animation`/`transition`/`transform: scale` Skyline 均支持
- **`.card` 全局类不动**（waypoint-editor 弹层复用它）；间距统一只改各页 `.page` 的 `gap`
- **tsc 基线 46 错**：任何一步超过 46 即视为本任务引入，需修回
- **截图基线**：`verify_shots/01_control.png`~`04_analysis.png`（2026-07-22 改进前）
