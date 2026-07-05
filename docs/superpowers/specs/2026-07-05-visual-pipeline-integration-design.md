# 视觉管线集成 — 两阶段 YOLO + MobileNet 云台扫描

> 日期：2026-07-05  
> 状态：draft  
> 关联 ADR：无

---

## 1. 动机

当前 `plant_detector_node` 使用 HSV 颜色分割作为植物检测占位，YOLOv8n Crop/Weed 模型已训练并量化为 BPU `.bin`（cosine ≥ 0.997, mAP50=0.860），但未集成到 ROS2 推理节点。病害分类 `vision_diagnosis_node` 独立运行，两节点之间没有级联关系。

本次变更将两阶段管线（YOLO 检测 → MobileNet 分类）完整集成，并通过云台多角度扫描提高病害检出率。

## 2. 架构

### 2.1 节点关系

```
 Patrol 阶段:                           Scan 阶段:
 camera → plant_detector(YOLO BPU)      camera → vision_pipeline_node
              ↓                                      ↓
         /vision/plant_detected              ┌─ YOLO detect (每帧)
              ↓                              ├─ MobileNet classify (有crop)
     mission_control_node                    ├─ servo_cmd 控制云台
              ↓ 同步 service                 ├─ 汇总多角度结果
     vision_pipeline_node ───────────────────┘
              ↓
         /vision/diagnosis (汇总一条)
```

### 2.2 模型 BPU 管理

- **PATROL 阶段**：`plant_detector_node` 独享 BPU，加载 YOLOv8n（~3.8 MB `.bin`）
- **SCANNING 阶段**：mission_control 暂停 plant_detector，vision_pipeline_node 独占 BPU 加载 YOLO + MobileNet（~6 MB `.bin`）
- **SCANNING 结束后**：pipeline 卸载模型，恢复 plant_detector，继续巡航

暂停/恢复通过 plant_detector_node 新增的 `SetBool` service（`/vision/plant_detector/pause`）实现。

## 3. 状态机

```
PATROL ───(YOLO detect crop)──→ STOPPED
  ↑                                │ pause plant_detector
  │                                ↓ trigger pipeline service (sync)
  │                             SCANNING
  │                                │ pub 汇总 Diagnosis
  │                                ↓ resume plant_detector
  │                             ANALYZING ──→ ACTION ──→ RESUME ──→ PATROL
  │
  └── 里程计门控 (min_resume_distance) 防止同株重复触发
```

`APPROACHING` 状态已移除——检测到 crop 直接急停，不再视觉伺服靠近。

### 3.1 同株去重

扫描完成后记录 `reference_pose`（odom）。RESUME 后 plant_detector 继续推理但结果被屏蔽，直到车辆移动超过 `min_resume_distance`（默认 0.5m）才允许 YOLO 再次触发停车。

## 4. vision_pipeline_node（新节点）

### 4.1 职责

- 接收 trigger service 调用，执行一次完整的扫描+推理流程
- 同步返回汇总诊断结果

### 4.2 扫描流程

```
1. 云台回中 (yaw=90, pitch=90, 参数化 initial 角度)
2. 等待 0.5s settle（相机防抖）
3. 加载 YOLO + MobileNet 到 BPU
4. LOOP (最多 max_shots 次):
   a. 取 camera 最新帧
   b. YOLO 推理 → 有 crop？
       无 → break
       有 → MobileNet 分类 → 记录 (bbox, class, confidence)
   c. bbox 中心偏离画面中心 > edge_threshold？
       否 → break
       是 → 云台 yaw/pitch 朝 bbox 方向步进 → goto 4a
5. 汇总所有有效帧结果
6. 卸载 BPU 模型
7. 返回汇总 Diagnosis
```

### 4.3 汇总策略

- `disease_class`：取所有帧中 confidence 最高的类别
- `confidence`：最高值
- `probabilities`：最高帧的完整 softmax 向量
- `bbox`：所有帧的并集

汇总 `Diagnosis` 额外附带 `per_angle_confidences`（float32 数组），供下游 fusion_node 矛盾检测降权。

### 4.4 边界处理

- 所有帧无 crop → `disease_class = "no_crop_detected"`, `confidence = 0.0`
- 全流程超时 15s → 返回已有结果
- 云台到达机械限位 → 停止该方向补拍，继续当前循环
- 相机帧超时 2s 无数据 → 跳过当前角度

## 5. plant_detector_node 升级

### 5.1 变更

- 移除 HSV 颜色分割
- 加载 `yolov8n_crop_weed_bayese_640x640_nv12.bin`（BPU, pyeasy_dnn）
- 输入 NV12 640×640，输出 crop/weed 二分类 + bbox
- 话题和消息格式不变：`/vision/plant_detected` (PlantDetection)
- 新增 `/vision/plant_detector/pause` service（SetBool），暂停/恢复推理循环

### 5.2 推理细节

- 预处理：BGR → resize 640×640 → BGR→NV12
- 后处理：NMS + conf threshold，取最高置信度 bbox
- 当 bbox 面积占比 > min_area_ratio 且 confidence > detection_confidence_threshold 时 `detected=true`

## 6. mission_control_node 变更

### 6.1 状态变更

- 移除 APPROACHING 状态及相关逻辑（视觉伺服、bbox 中心对齐）
- 新增 SCANNING 状态
- STOPPED 状态：同步调用 pipeline trigger service，转入 SCANNING
- SCANNING 状态：接收 pipeline 返回的汇总 Diagnosis，pub 到 `/vision/diagnosis`，转入 ANALYZING

### 6.2 同株去重

- 新增参数 `min_resume_distance`（默认 0.5m）
- 扫描完成后记录 `reference_pose`（订阅 `/odom`）
- PATROL 状态下，plant_detected 回调中先检查里程计距离，不足阈值则忽略

### 6.3 参数变更

| 参数 | 操作 | 默认值 |
|---|---|---|
| `min_resume_distance` | 新增 | 0.5 |
| `bbox_center_tolerance` | 移除 | — |
| `approach_speed` | 移除 | — |
| `stop_distance_tolerance` | 移除 | — |

## 7. 接口定义

### 7.1 新 Service：PipelineTrigger

```ros2
# sentry_interfaces/srv/PipelineTrigger.srv
string crop_type          # tomato / wheat / strawberry
uint8 max_shots           # 最多补拍次数 (默认3)
---
bool success
sentry_interfaces/Diagnosis result
float32[] per_angle_confidences   # 每个有效角度诊断置信度 [0.88, 0.0, 0.72, ...]
```

### 7.2 plant_detector_node pause service

使用标准 `std_srvs/SetBool`：
- `true` → 暂停推理（释放 BPU）
- `false` → 恢复推理（重新加载模型）

## 8. 文件变更清单

| 文件 | 操作 | 说明 |
|---|---|---|
| `src/sentry_vision/sentry_vision/plant_detector_node.py` | 改写 | HSV→YOLOv8n BPU + pause service |
| `src/sentry_vision/sentry_vision/yolo_utils.py` | 新建 | YOLO 前后处理（NV12 转换、NMS） |
| `src/sentry_vision/sentry_vision/vision_pipeline_node.py` | 新建 | 云台扫描编排 + 两阶段推理 + 汇总 |
| `src/sentry_vision/setup.py` | 修改 | 注册 vision_pipeline_node 入口 |
| `src/sentry_interfaces/msg/Diagnosis.msg` | 修改 | 新增 `float32[] per_angle_confidences` 字段 |
| `src/sentry_interfaces/srv/PipelineTrigger.srv` | 新建 | trigger service 定义 |
| `src/sentry_interfaces/CMakeLists.txt` | 修改 | 注册 PipelineTrigger.srv |
| `src/sentry_mission/sentry_mission/mission_control_node.py` | 修改 | 移除 APPROACHING → 新增 SCANNING → 去重门控 |
| `src/sentry_bringup/launch/sentry_v2.launch.py` | 修改 | 注册 vision_pipeline_node |
| `src/sentry_vision/tests/test_yolo_utils.py` | 新建 | YOLO 前后处理单测 |
| `src/sentry_vision/tests/test_vision_pipeline.py` | 新建 | pipeline 状态机测试 |
| `src/sentry_mission/tests/test_mission_control_node.py` | 修改 | 覆盖 SCANNING + 去重逻辑 |

## 9. 测试策略

### 单元测试（本地 / RDK）

| 测试目标 | 方法 | 覆盖 |
|---|---|---|
| YOLO 前后处理 | 已知图片 + 已知输出，验证 NV12 resize + NMS | yolo_utils.py |
| pipeline 状态机 | mock camera/servo/BPU 推理，验证分支 | vision_pipeline_node.py |

### 板端集成测试（RDK X5）

1. `plant_detector_node` 单独启动，验证 YOLO BPU 推理帧率和检出率
2. `vision_pipeline_node` 手动 trigger，验证云台扫描 + 两阶段推理
3. 端到端：`sentry_v2.launch.py` 全链路，waypoint 巡航 → 检测停车 → 扫描 → 诊断 → 恢复

## 10. 不改变的部分

- `vision_diagnosis_node`：保留，可独立用于单帧诊断
- `camera_node`、`servo_driver_node`：无变更
- `fusion_node`、`advisory_node`、`data_logger_node`：无变更
- ServoCmd.msg：复用现有定义
