# 舵机随巡航换行自动翻转 — 设计文档

日期：2026-07-19（v6，经六轮 spec 评审修订）
状态：已获用户批准（方案 A）；v6 修正：idx==1 时用任务起点位姿构成首航段（默认 waypoints.yaml 的首个航点即第一行行末），确认默认航点文件为合规布局 (b)

## 背景与需求

小车搭载的摄像头固定在舵机（32 脚 PWM6，`pwmchip0/pwm0`）上，巡航时朝向植株行一侧：

- 舵机 0° = 摄像头朝小车**右**侧（初始位置）
- 舵机 180° = 摄像头朝小车**左**侧

蛇形巡航路径中，小车到达行末并转弯进入下一行后，植株行相对小车的方位左右互换。若舵机不翻转，摄像头将拍不到新行。需求：到达换行航点时立即翻转舵机 180°，使进入新行时摄像头已对准植株。

## 已确认的需求决策

| 决策点 | 结论 |
|--------|------|
| 角度映射 | 0° = 朝右，180° = 朝左（实测 500µs = 0°，2500µs = 180°，50Hz） |
| 翻转时机 | 到达行末航点（waypoint 完成回调）立即翻转 |
| 转弯期检测 | 加冷却窗口，窗口内不触发停车诊断（不全程暂停 detector） |
| 方向判定 | 按航段几何计算（heading 变化），不依赖航点序号奇偶 |

## 方案选择

采用**方案 A**：在 `mission_control_node` 的 waypoint 完成回调中做几何判定并发布 `/sentry/servo_cmd`。

- 与状态机天然同步：避障（OBSTACLE_* 状态链）不经过该回调，MANUAL ↔ PATROL 切换不误触发
- 冷却窗口直接挂在现有 `_should_trigger_scan` 去抖逻辑上
- `docs/ROS2.md:105` 已预留 mission_control 作为 `/sentry/servo_cmd` 发布者

否决方案：独立节点订阅 `/mission/status`（航点知识重复、与状态机失同步）；舵机 srv 服务（无实际收益的多余层次）。

## 详细设计

### 1. 改动文件

- `src/sentry_mission/sentry_mission/mission_control_node.py` — 新增舵机翻转逻辑（publisher、`_maybe_flip_servo()`、冷却检查、7 个 `declare_parameter`）
- `src/sentry_bringup/launch/sentry_v2.launch.py` — mission_control_node 的 parameters 字典中显式传入新参数（沿用现有全部参数的传递惯例；该 YAML 字符串参数仅被 `_load_fixed_point_stops` 自定义解析，节点参数不走它）
- `src/sentry_servo/config/servo_config.yaml` — yaw `initial_angle` 由 67.5 改为 0（朝右，与初始位置定义一致）

### 2. 数据流

```
Nav2 到达航点 → tick() 中 isTaskComplete() && TaskResult.SUCCEEDED (mission_control_node.py:816-823)
  → current_wp_idx += 1
  → _maybe_flip_servo()  ← 新增
      → 发布 /sentry/servo_cmd (yaw=0 或 180, pitch=servo_pitch_hold)
  → _send_next_waypoint()
→ servo_driver_node 收指令 → 写 sysfs pwmchip0/pwm0
```

新增 publisher：`create_publisher(ServoCmd, '/sentry/servo_cmd', 10)`。`ServoCmd` 为 `uint8 pitch/yaw`（`ServoCmd.msg:1-2`），0/180 在量程内，无需改消息定义。

pitch 安全性：`servo_driver_node.on_servo_cmd`（`servo_driver_node.py:109-112`）以 `if self.pitch is not None` 守卫——当前 `servo_config.yaml` 无 pitch 舵机配置，`self.pitch` 为 None，pitch 字段被忽略，安全。

### 3. 几何判定 `_maybe_flip_servo()`

在 `current_wp_idx += 1` 之后、`_send_next_waypoint()` 之前调用。

**判定规则（要求拐角分离布点）：**

航点布局有两种理论可能：
- (a) 航点只放在行首/行末，行间走对角线过渡：**不支持**。对角过渡段本身是长航段，其两端边界都是 ≈158° 的反平行几何，局部几何无法区分"该翻"（行末）和"不该翻"（过渡段末端）——固定阈值下会双重触发回弹，auto 推导下会零触发。且对角折返本就不构成左右交替的蛇形覆盖，不是真实作业路径
- (b) 行末与拐角分开布点（换行由两段约 90° 的转弯组成，中间是长度≈行距的拐角短航段）：**本设计支持的布局**，真实蛇形覆盖路径必然是这种结构

规则为：

```
idx = self.current_wp_idx                    # 已递增
wp = self.waypoints                          # 元素为 dict，用 ['x']/['y'] 访问

# 刚完成航段的起点：
#   idx==1（到达首个航点）→ 用任务起点位姿 _mission_start_pose
#     （首个航点可能就是第一行行末，上一航段是小车从起点开到 wp0；
#       若 wp0 是第一行起点，则小车起点 ≈ wp0，航段长 ≈ 0，规则 1 安全跳过）
#   idx>=2 → 用 wp[idx-2]（不会负索引）
if idx == 1:
    x0, y0 = self._mission_start_pose
else:
    x0, y0 = wp[idx-2]['x'], wp[idx-2]['y']

# 1) 刚完成的航段必须是"长航段"（巡行段），否则跳过
#    （到达拐角短航段末端时不判定，避免双重触发）
dx0 = wp[idx-1]['x'] - x0
dy0 = wp[idx-1]['y'] - y0
if hypot(dx0, dy0) < min_row_segment_length: return

# 2) 向后跳过短航段（拐角过渡段），找到下一条长航段
j = idx
while j < len(wp) and hypot(wp[j]['x']-wp[j-1]['x'],
                            wp[j]['y']-wp[j-1]['y']) < min_row_segment_length:
    j += 1
if j >= len(wp): return                      # 后面没有长航段（已到末尾）

# 3) 反平行判定：下一条长航段与刚完成航段 heading 差 ≥ 阈值 → 换行
h_done = atan2(dy0, dx0)
h_next = atan2(wp[j]['y']-wp[j-1]['y'], wp[j]['x']-wp[j-1]['x'])
delta = wrap_to_pi(h_next - h_done)
if abs(delta) < flip_heading_threshold: return

# 4) 换行成立 → 切换侧向并发布
self._servo_side = 'left' if self._servo_side == 'right' else 'right'
yaw = servo_yaw_left if self._servo_side == 'left' else servo_yaw_right
publish ServoCmd(yaw=yaw, pitch=servo_pitch_hold)
self._servo_flip_time = now                  # self.get_clock().now().nanoseconds / 1e9
self._servo_flip_position = (self.odom_x, self.odom_y)
# 同时重置扫描去抖参考点（复用 min_resume_distance）
self.reference_x = self.odom_x; self.reference_y = self.odom_y
self.has_scan_reference = True
```

`_mission_start_pose` 在**每次** `_send_next_waypoint` 发送 idx=0 时记录 `(self.odom_x, self.odom_y)`（任务重跑时 odom 已被 `_prepare_autonomous_start` 重置，重记录恰是正确的起点）；`__init__` 中初始化为 `(0.0, 0.0)`（与 odom 原点一致，即使记录前被调用也安全）。

要点：

- **侧向用切换（toggle）而非转角符号推断**：U 型折返后新行恒在旧行的对侧，追踪 `self._servo_side`（初始 `'right'`）即可，不依赖具体田块几何的符号约定
- 到达行末时向后跳过拐角短航段看到新行，delta ≈ 180° 触发 ✓；到达拐角末端时刚完成的是短航段，规则 1 跳过，不会二次触发 ✓
- 纯直线中间点、L 型拐弯（delta ≈ 90° < 120°）不触发 ✓
- `min_row_segment_length` 默认 **0（auto）**：节点启动加载航点后自动推导，取 `min_seg = 最短航段长`、`max_seg = 最长航段长`，有效值 `min_row_segment_length = (min_seg + max_seg) / 2`。蛇形路径里最短段≈行距（拐角段）、最长段≈行长（巡行段），中点必落在开区间 `行距 < 有效值 < 行长` 内，web 前端改航点调行距后无需同步改参数（与"航点修改重启生效"的语义一致，重启时重新推导）。参数 > 0 时用手动值覆盖自动推导
  - 推导失败回退：航点数 < 2（无航段）→ 记 warn 日志并禁用自动翻转；所有航段等长 → 有效值等于段长，严格 `<` 下全部视为长航段（此时几何上本就无法区分巡行段与拐角段，属可接受退化，不额外处理）
- `flip_heading_threshold` 默认 120°（2.09 rad）：换行反平行 delta ≈ 180°，L 型拐弯 delta ≈ 90°，阈值落在两者之间
- 航点坐标为 dict（`waypoints[i]['x']`），判定基于**计划航点**而非实际轨迹，避障绕行的航向扰动不进入判定
- 新增成员在 `__init__` 初始化：`self._servo_side = 'right'`、`self._servo_flip_time = None`、`self._servo_flip_position = None`（`_should_trigger_scan` 在首个 PATROL tick 就会被调用，不初始化会 AttributeError）、`self._mission_start_pose = (0.0, 0.0)`

### 4. 检测冷却窗口

新增参数：`servo_flip_cooldown_sec`（默认 8.0）、`servo_flip_cooldown_distance`（默认 0.8 m）。

在 `_should_trigger_scan`（`mission_control_node.py:513-522`）开头新增：

```
if self._servo_flip_time is not None:
    dt = now - self._servo_flip_time
    dist = hypot(self.odom_x - self._servo_flip_position[0],
                 self.odom_y - self._servo_flip_position[1])
    if dt < servo_flip_cooldown_sec and dist < servo_flip_cooldown_distance:
        return False
    self._servo_flip_time = None             # 窗口结束，清除
```

**任一条件突破（超时或超距）即退出冷却**——避免低速卡顿时冷却长期有效。翻转时位置单独存 `_servo_flip_position`，不复用 `reference_x/y`（后者会被后续扫描点更新覆盖）。

### 5. 边界、错误处理与已知限制

- `enable_servo_auto_flip`（默认 **false**）总开关，关闭时行为与现状完全一致
- 避障状态链（OBSTACLE_*）不经过 waypoint 完成回调，不受影响
- MANUAL → PATROL 恢复时舵机保持当前角度，不强制归位
- **已知限制：仅支持拐角分离布点（布局 b）**——行末与拐角必须是不同航点，拐角段长度≈行距 < 行长。仅行末布点、对角线过渡的路径（布局 a）不支持：过渡段两端都是反平行几何，局部判定无法区分翻转时机，会出现双重触发或零触发
- **已知限制：假设第一行植株在小车右侧**（`servo_config.yaml` 初始角 0°）。若地块首行在左侧，需手动先把舵机转到 180° 再启动巡航（后续可加 `servo_initial_side` 参数，本期不做）
- **已知限制：web 前端修改的航点只写入文件，节点仅启动时加载一次（`mission_control_node.py:190-200`），运行中 `self.waypoints` 不变，修改需重启生效**
- servo 节点未启动 / 发布失败：仅 warn 日志，不阻断巡航
- launch 启动需 `enable_servo:=true` 才会拉起 `servo_driver_node`（`sentry_v2.launch.py:50,211`）

### 6. 新参数汇总

全部走 `declare_parameter` + launch parameters 字典显式传值（与现有参数一致）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_servo_auto_flip` | false | 舵机自动翻转总开关 |
| `servo_yaw_right` | 0 | 朝右角度（初始侧） |
| `servo_yaw_left` | 180 | 朝左角度 |
| `servo_pitch_hold` | 0 | 发布时携带的 pitch 值（当前被驱动节点忽略） |
| `flip_heading_threshold` | 2.09 | 换行判定转角阈值（rad，120°） |
| `min_row_segment_length` | 0（auto） | 巡行长航段最小长度（m）。0 = 启动时按 `(最短航段+最长航段)/2` 自动推导；>0 = 手动覆盖 |
| `servo_flip_cooldown_sec` | 8.0 | 冷却窗口时长 |
| `servo_flip_cooldown_distance` | 0.8 | 冷却窗口距离（m） |

### 7. 测试

测试框架：pytest + unittest.mock（patch BasicNavigator），现有参考 `src/sentry_mission/tests/test_mission_control_node.py`（770 行）。

- 单元测试 `_maybe_flip_servo` 几何判定（全部基于布局 b 的蛇形路径）：
  - 布局 (b) 完整蛇形（含仓库默认 waypoints.yaml 的两行结构：起点→行末→拐角→行末）：到达行末即触发一次，发布 yaw=180（首次切换 right→left）；到达拐角末端不二次触发
  - 首个航点即第一行行末（idx=1）：用任务起点位姿构成航段，几何满足时正常翻转；小车起点 ≈ wp0 时航段长 ≈ 0，规则 1 安全跳过
  - 任务重跑（MANUAL→AUTO 归零后）：到达 wp0 仍正确翻转（_mission_start_pose 每次发送 idx=0 时重记录）
  - 连续两次换行 → 侧向来回切换（180 → 0）
  - 直线中间点（Δ≈0）、L 型拐弯（Δ≈90°）→ 不发布
  - 刚完成短航段、后方无长航段 → 不发布
  - idx=1（到达首个航点）→ 用任务起点位姿构成航段，无负索引访问
  - 拐角段长度恰等于行距默认值（1.0 m）→ 自动推导有效值 1.75（行距 1.0、行长 2.5）下仍正确判定
  - 自动推导：航段长度集合推导出的有效值落在 `(min_seg, max_seg)` 开区间内；参数 > 0 时手动值优先生效；航点数 < 2 → warn + 禁用自动翻转
  - `enable_servo_auto_flip=false` → 不发布
- 冷却逻辑测试：翻转后窗口内 `_should_trigger_scan` 返回 False；超时或超距后恢复
- 板端实车验证：蛇形路径跑一圈，观察翻转时机、朝向与检测冷却表现。仓库默认 `waypoints.yaml`（起点 (0,0) → (2.5,0) → (2.5,1) → (0,1)）即为合规的布局 (b) 两行结构，可直接用于验证：到达 (2.5,0) 时应翻转
