# 微信小程序四页优化 · 设计文档

> 版本 v1.0 · 2026-07-22
> 状态：设计确认，待实现

---

## 1. 目标

在不改变 Grafana 深色工业风（色板、字体、圆角体系全部沿用 `app.less` 现有 token）的前提下，优化小程序四页（控制/监测/分析/天气）的空态反馈、排版细节、布局微调与微交互。零新页面、零新请求、零 JS 定时器动画。

---

## 2. 决策总览

| 决策 | 选择 | 理由 |
|------|------|------|
| 风格 | 完全复用现有 CSS 变量，不加新色系 | 用户明确要求"风格不变" |
| 三态实现 | 共享组件 `state-block`（loading/empty/offline） | 四页复用，一处维护 |
| 状态判定 | 仅用现有 store 字段（`connected` + 数据非空） | 不加新请求/订阅 |
| 动效 | 纯 CSS transition/:active | 小程序 Skyline 下 JS 定时器动画易掉帧 |
| 布局 | 只微调，不重构 wxml 结构 | 降低回归风险 |

---

## 3. 三态规范（`components/state-block/`）

属性：`type: loading|empty|offline`、`text`、`subtext?`。

- `loading`：蓝色（`--blue`）细圈 spinner + 文案
- `empty`：muted 占位符 `◌` + 文案 + 副文案
- `offline`：2rpx 红色（`--red`）左边框 + 文案（含当前 IP）

应用点：

| 页面 | 位置 | 规则 |
|------|------|------|
| 监测 | 视频区 | `!connected`→offline；connected 但无帧→loading「视频连接中…」 |
| 监测 | 传感器卡片 | 无数据显示 `·`（`--` 语义改为"连接中"） |
| 分析 | 风险趋势/分类概率 | 统一换成 state-block empty「数据收集中…」 |
| 天气 | 整页数据区 | `apiGetWeather` 失败→offline 态；无数据字段显示 `·` |

---

## 4. 排版与细节

- 数值 `.mono` 28-32rpx；标签 20rpx `--text-dim` 大写（沿用 card-header 规范）
- 控制页 dpad 按钮 80→96rpx，间距 6→8rpx
- 卡片内边距统一 `20rpx 24rpx`，卡片间距统一 16rpx
- 天气页"当前"卡湿度/风字段无数据显示 `·`

## 5. 布局微调

- 控制页：连接角标并入"当前模式"卡——一行状态条（模式 badge + 连接状态点 + IP）
- 监测页：视频区改 16:9  aspect 比例（`aspect-ratio` 或 padding-top hack，Skyline 兼容优先后者）
- 分析页：AI 报告正文 line-height 1.7、段间距 16rpx；建议列表序号用 `--blue` 色块

## 6. 微交互（纯 CSS）

- 可点元素 `:active`：`transform: scale(0.96)` + 100ms transition（dpad、button、pill、可点卡片）
- 传感器数值：color 300ms transition
- 连接角标：状态切换 200ms 淡入
- 巡航按钮 `starting/stopping` 态 `disabled` + 按钮内置 loading（防连点，WXML 用 `stackState` 判定）

## 7. 明确不做（YAGNI）

- 不改色板/字体/圆角；不加新页面/新组件框架
- 不重构 wxml 结构（只插 state-block、调类名）
- 不做真机动效性能调优（本期模拟器验证即可）

## 8. 验证

- 微信开发者工具自动化逐页截图，与 `verify_shots/` 四张基线对比
- 模拟 `connected=false`（改错 IP）验证 offline 态；改回验证恢复
- tsc 编译无新增错误（基线 46 个既有错误）
