# 海康相机软件自适应曝光设计

日期：2026-07-17
状态：已批准
分支：`feat/adaptive-exposure`（从 `codex/hikrobot-vision-node` 基线开出）

## 背景

相机：Hikrobot MV-CS016-10UC（USB3 Vision），驱动节点
`src/sentry_bringup/sentry_bringup/hikrobot_camera_node.py`。

已确认的事实（来自 codex 现场调试）：

- 硬件自动曝光（ExposureAuto=2）在该相机上不可靠：暗场景只收敛到 5.84ms 且增益顶格，画面严重欠曝。
- `AutoExposureTimeLowerLimit/UpperLimit` 写入返回 0x80000109，该型号不支持，不能依赖硬件 AE 限制策略。
- 相机实际 ExposureTime 范围为 15µs ~ 9,999,813µs。
- 当前板端配置（未入库）：手动 ExposureTime=100ms、GainAuto=True(0~12)、gamma=3.0 LUT。
  结果：暗场景可用，但亮场景严重过曝（整片刷白），巡航时 100ms 曝光产生拖影。

## 目标

- 软件闭环自动曝光：以帧亮度反馈调节曝光/增益，适应明暗场景切换。
- 巡航（车体移动）时压低曝光上限防止拖影；停车分析时允许长曝光保证画质。
- 全程本地可 TDD，控制核心无 ROS 依赖。

非目标：gamma 自适应、硬件 AE 修复、前端 AE 开关 UI。

## 前置步骤：基线入库

codex 的曝光调优成果目前仅以未提交工作区改动存在于板端 `~/dev_ws`（`1f1af6c`
在本地与板端仓库均不存在）。直接开发会导致基线冲突或丢失。

1. 板端：将现有工作区改动作为 baseline 提交到 `codex/hikrobot-vision-node` 并 push。
2. 本地：pull 后从该分支开出 `feat/adaptive-exposure`。
3. 实现完成后 push，板端 pull 实测。

## 架构

```
hikrobot_camera_node（ROS2）
  ├─ capture(): 帧 → 亮度统计（下采样灰度，gamma 之前）
  │     → AdaptiveExposureController → MV_CC_SetFloatValue(ExposureTime / Gain)
  │     （寄存器写入限频：每 0.4s 最多一次）
  ├─ 订阅 /wheel/odom (nav_msgs/Odometry) → moving 标志（迟滞）
  └─ gamma LUT（固定 2.0）在统计之后应用，只影响输出画面

auto_exposure.py（新增，纯 Python，无 ROS 依赖）
  └─ AdaptiveExposureController：update(stats, moving, now) -> 曝光/增益命令
```

亮度统计在 gamma LUT 之前取 raw BGR，避免 LUT 扭曲反馈。

## 控制算法（每帧）

输入：160×120 下采样灰度的 `mean`（0-255）、`sat_ratio`（像素 >250 占比）、
`moving` 标志。

优先级从高到低：

1. **饱和保护**：`sat_ratio > 0.02` → `exposure *= 0.6`，跳过正常步进直接返回。
2. **死区**：`|target/mean - 1| < 0.05` → 不动。
3. **正常步进**：`ratio = clamp(target/mean, 0.7, 1.4)`。
   - 变亮（ratio>1）：曝光先乘 ratio 直到当前动态上限，仍不足再加增益。
   - 变暗（ratio<1）：增益先降到下限，仍过亮再降曝光。（保持低噪点）
4. **动态曝光上限**：moving → 20ms；静止 → 100ms；下限 2ms。

moving 判定（迟滞）：|v| > 0.05 m/s → moving；|v| < 0.02 m/s 持续 1s → 静止。
里程计消息超时（>2s）→ 按 moving 处理（宁可噪点也不拖影）。

控制器每 0.4s 最多输出一次写寄存器命令（曝光生效需 1-2 帧，避免过调振荡）。
该限频对包括饱和保护在内的所有寄存器写入统一生效。

## 参数

节点新增参数（launch 可配）：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `ae_enabled` | `true` | false 时行为与现状完全一致（逃生舱） |
| `ae_target_luma` | `80.0` | raw 亮度目标（gamma 2.0 后约 128） |
| `ae_deadband` | `0.05` | 死区 |
| `ae_max_step` | `1.4` | 单步最大倍率 |
| `ae_sat_limit` | `0.02` | 饱和像素占比阈值 |
| `ae_exp_min_us` | `2000.0` | 曝光下限 |
| `ae_exp_max_moving_us` | `20000.0` | moving 时曝光上限 |
| `ae_exp_max_still_us` | `100000.0` | 静止时曝光上限 |
| `ae_gain_min` / `ae_gain_max` | `0.0` / `12.0` | 增益范围 |
| `ae_move_speed_thresh` | `0.05` | moving 速度阈值 (m/s) |
| `ae_still_speed_thresh` | `0.02` | 静止速度阈值 (m/s) |
| `ae_update_period_s` | `0.4` | 寄存器写入限频 |

launch（`sentry_v2.launch.py`）改动：`gain_auto` → `false`（软件 AE 接管增益）、
`gamma` → `2.0`。

## 错误处理

- 寄存器写失败：节流 warn 日志，保留上次值，节点不崩。
- 启动时回读硬件 ExposureTime/Gain 作为控制器初值（延续"以回读为准"原则）。
- grabbing 状态下写 ExposureTime/Gain 对 USB3 MVS 即时生效（首轮板端验证确认）。
- 里程计缺失/超时：按 moving 上限运行。

## 测试

本地 TDD（off-board，无需相机）：

- 控制器单测：
  - 暗帧 → 曝光递增至 still 上限后再加增益至 gain_max；
  - 亮帧 → 增益先降至 gain_min，再降曝光至 exp_min；
  - 饱和帧（sat_ratio>0.02）→ 曝光 ×0.6 快降；
  - 死区内不变；单步倍率不超过 1.4；
  - moving=true 时曝光被钳制在 20ms；
  - moving↔静止迟滞切换正确；
  - 写入限频：0.4s 内重复 update 不输出新命令。
- 统计函数：合成 numpy 图验证 mean/sat_ratio 计算。
- 节点测试：沿用 `test_hikrobot_camera_node.py` 的 mock MVS SDK 模式，
  验证 ae_enabled=false 时不写寄存器、true 时按控制器输出写寄存器。

板端验收：

- 对准窗户（亮）/角落（暗），观察日志曝光收敛、画面不过曝不欠曝。
- 巡航中确认曝光 ≤20ms，画面无拖影。
- `ae_enabled=false` 回退行为与现状一致。
