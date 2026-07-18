# 前端巡航检测消息推送设计

> 日期：2026-07-17 · 状态：已批准（方案 A + 一键清理）
> 实现计划：`docs/superpowers/plans/2026-07-17-mission-message-center.md`

## 1. 目标

巡航任务结束后，操作者在前端顶部状态栏点开消息按钮，即可查看本次巡航中检测到的植株快照与病害类别。多个任务批次的消息保留在 web_remote_node 内存中，直到节点退出。

## 2. 需求确认（brainstorm 结论）

| 决策点 | 结论 |
|---|---|
| 批次边界 | mode MANUAL→AUTO（启动巡航）开批次；AUTO→MANUAL（完成/手动停/急停）关批次 |
| 快照时机 | 检测到植株瞬间（`/mission/status` 出现 PATROL→STOPPED 边沿） |
| 记录范围 | YOLO 检测停车 + 固定点停车都记录；固定点停车仅当时刻有有效检测框才记录 |
| 快照样式 | 后端用 cv2 在 JPEG 上画绿色检测框 + 置信度标签 |
| 提醒形式 | 铃铛按钮 + 红色未读角标（数字 = 未读批次数），点开面板清除 |
| 清理 | 消息面板内"一键清理"按钮，清空全部批次与快照（带确认） |
| 生命周期 | 节点内存，web_remote_node 退出即丢；空批次（0 记录）关闭时丢弃 |

## 3. 架构

```
plant_detector ──/vision/plant_detected──┐
vision_pipeline ──/vision/diagnosis──────┤
mission_control ──/mission/status────────┼──> web_remote_node
camera/republish ──/out/compressed───────┘      │ BatchRecorder (纯 Python, 内存)
                                                │ cv2 画框 → JPEG bytes
                                                ▼
                                   HTTP: GET /api/messages
                                        GET /api/messages/<batch>/<seq>/snapshot
                                        POST /api/messages/read
                                        POST /api/messages/clear
                                        /status.message_unread
                                                ▼
                              static_v2: top-bar 铃铛 + message-center modal
```

## 4. 后端设计（web_remote_node.py + batch_recorder.py）

### 4.1 BatchRecorder（纯 Python，新文件，无 ROS 依赖，可 off-board 测试）

```python
class BatchRecorder:
    def __init__(self, now=time.time):
        self.batches = []        # 已完成批次
        self.current = None      # 进行中的批次
        self.unread = 0
        self._seq = 0            # 批次序号

    def on_mode_change(self, new_mode): ...
        # MANUAL->AUTO: 开批次 {id, name='批次#N · MM-DD HH:MM', started_at, records=[]}
        # AUTO->MANUAL: 关批次；records 为空则丢弃，否则 unread += 1

    def on_stop_trigger(self, plant, jpeg_bytes): ...
        # PATROL->STOPPED 边沿调用。
        # plant 为 None（无有效检测框）或无进行中批次 -> 忽略
        # 追加 record {seq, timestamp, bbox, plant_confidence, jpeg_bytes,
        #              disease_class=None, disease_confidence=None}

    def on_diagnosis(self, disease_class, confidence): ...
        # 回填当前批次最近一条 disease_class is None 且 15s 内的记录

    def mark_read(self): self.unread = 0
    def clear(self):  # 清空全部批次、当前批次记录、unread
```

### 4.2 节点接线

- 新增订阅：`/vision/plant_detected`（缓存最新一条 + 接收时刻）、`/vision/diagnosis`（→ `on_diagnosis`）
- `on_mission_status` 已有 PATROL+完成判定处补充：**PATROL→STOPPED 边沿** → 若最新 plant_detected `detected=true` 且 2s 内 → 取 `latest_camera_jpeg`，`draw_bbox_on_jpeg()` 画框 → `on_stop_trigger`
- mode 变更点（`set_mode`/AUTO 切换处）→ `on_mode_change`
- 快照画框：`draw_bbox_on_jpeg(jpeg_bytes, bbox_norm, label)` → cv2 imdecode → 矩形+文字 → imencode JPEG

### 4.3 HTTP API

| 端点 | 方法 | 返回 |
|---|---|---|
| `/api/messages` | GET | `{batches: [{id, name, started_at, ended_at, records: [{seq, timestamp, disease_class, disease_confidence, plant_confidence, snapshot_url}]}], unread}` |
| `/api/messages/<batch_id>/<seq>/snapshot` | GET | image/jpeg |
| `/api/messages/read` | POST | 清 unread |
| `/api/messages/clear` | POST | 清空全部批次与快照 |
| `/status` | GET | 新增 `message_unread` 字段 |

## 5. 前端设计（static_v2）

- `top-bar.js`：拍摄按钮右侧加铃铛按钮，`store.messageUnread > 0` 时显示红色角标
- `components/message-center.js`（新）：复用 waypoint-editor 的 `.modal-overlay + .modal` 模式
  - 头部：标题 + "一键清理"按钮（confirm 确认）+ 关闭
  - 按批次分组（批次名 + 记录数），倒序（最新批次在上）
  - 每条记录：缩略图、病害类别（复用现有中文名映射，无映射显示原名，未回填显示"未知"）、置信度、HH:MM:SS
  - 点缩略图 → 大图查看
  - 打开时 `POST /api/messages/read`
- `ros.js`：store 增加 `messageBatches`、`messageUnread`；轮询 `/status` 时同步 unread；打开面板时拉 `/api/messages`

## 6. 错误处理

- 无相机帧（latest_camera_jpeg 为 None）→ 跳过该条记录，日志 warn
- JPEG 解码失败 → 跳过画框，存原图
- 批次进行中节点收到 stop_stack → mode 回 MANUAL，批次正常关闭
- 诊断超时未到达 → disease_class 显示"未知"

## 7. 测试

- `tests/test_batch_recorder.py`：开/关批次、空批次丢弃、快照记录（无批次/无检测框忽略）、诊断回填（15s 窗口、超时忽略）、unread 计数、mark_read、clear
- `tests/test_web_remote_node.py` 增补：PATROL→STOPPED 边沿触发快照、固定点无检测框不记录、API 端点（Flask test client）
- 板端实测：巡航一圈 → 消息按钮出现角标 → 查看快照带检测框 → 一键清理

## 8. 不做（YAGNI）

- 磁盘持久化、单条删除、浏览器系统通知、触发类型（检测/固定点）标记、批次导出
