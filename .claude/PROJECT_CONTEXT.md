# 智农哨兵 · 项目快速概览

> 架构版本 v2.1+ · 更新日期 2026-06-25  
> 详细文档见 [`docs/`](../docs/)。

---

## 项目目标

面向番茄/小麦/草莓多作物病害巡检的嵌入式比赛原型机：

- 底盘自动巡航（当前 mapless Nav2，目标 LiDAR SLAM）
- 植株检测触发停车 → 端侧 AI 病害识别（RDK X5 NPU）
- 移动/固定环境数据融合决策 → 农艺建议
- 本地 ros2 bag 数据记录

---

## 核心硬件

| 模块 | 型号/方案 |
|---|---|
| AI 主控 | RDK X5（8 核 A55, R5 NPU 10 TOPS） |
| 运动控制 | STM32F407ZGT6（FreeRTOS） |
| 雷达 | STL19P / LD19（UART 230400） |
| 摄像头 | IMX219 MIPI-CSI |
| IMU | YB-IMU（CH340 USB） |
| 云台 | 2-DOF 舵机，RDK X5 直接 PWM |
| 环境传感 | 移动七合一空气/土壤 + 固定 LoRa 节点 |

**注意**：GPS 模块已移除，不再使用。

---

## 核心软件栈

- ROS2 Humble on Ubuntu 22.04
- Nav2 + robot_localization EKF
- Python/C++ 混合节点（`sentry_*` 包）
- ONNX 源模型 → RDK X5 运行时 `.bin`

---

## 文档地图

| 文档 | 内容 |
|---|---|
| [`docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) | 系统架构、模块划分、数据流、当前/目标导航 |
| [`docs/HARDWARE.md`](../docs/HARDWARE.md) | 硬件规格、传感器、接线、FreeRTOS、通信协议 |
| [`docs/ROS2.md`](../docs/ROS2.md) | 节点图、话题/服务/参数、消息定义、TF |
| [`docs/DECISIONS.md`](../docs/DECISIONS.md) | 技术决策记录（ADR） |
| [`docs/TODO.md`](../docs/TODO.md) | 当前 Sprint 任务与阻塞项 |
| [`docs/ISSUES.md`](../docs/ISSUES.md) | 已知问题、硬件限制、规避方案 |
| [`docs/SETUP.md`](../docs/SETUP.md) | 环境搭建、编译、启动、STM32 烧录、模型部署 |

开发规范见根目录 [`CLAUDE.md`](../CLAUDE.md)。

---

## 当前重点

1. 导航升级：从 mapless Nav2 迁移到 LiDAR SLAM/mapping。
2. STM32 固件：扩展 `TYPE_CHASSIS` 到 19 字节，接入编码器脉冲。
3. 补齐七合一空气/土壤传感器 UART 协议。
4. 训练/获取小麦、草莓病害模型与植株检测模型。
5. 完成固定环境节点 STM32L072 + LoRa 硬件与固件。
