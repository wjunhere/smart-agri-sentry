# PLAN.md — 舵机随巡航换行自动翻转

分支：`feat/servo-row-switch-flip`
Spec：`docs/superpowers/specs/2026-07-19-servo-row-switch-flip-design.md`（v6, Approved）
计划：`docs/superpowers/plans/2026-07-19-servo-row-switch-flip.md`

- [x] Task 0: 建分支 + PLAN.md 初始化
- [x] Task 1: 参数声明、初始化成员、auto 推导、ServoCmd publisher
- [x] Task 2: `_maybe_flip_servo` 几何判定 + tick 钩子 + 起点位姿记录
- [x] Task 3: 检测冷却窗口
- [x] Task 4: launch 参数接线 + servo 初始角 0 + 板端全量测试
  - 板端结果：新增 20 个测试全部 passed；全量 110 passed + 7 errors（test_wheel_odom_node.py 既有问题，与本改动无关）
  - 备注：start_robot_stack.sh 增加 ENABLE_SERVO / ENABLE_SERVO_AUTO_FLIP 透传；板端已 pull 对齐到 1504535
  - 备注：板端 ~/dev_ws 原有 feat/mission-message-center 未提交改动已 stash（auto-stash before testing feat/servo-row-switch-flip）
- [x] Task 5: 板端实车验证
  - 第一次跑：未翻转。诊断日志定位根因——`_mission_start_pose` 记录的是 EKF 重置传播前的陈旧位姿 (1.58,0)，导致首段被误判为短航段跳过（f634f20 修复：任务起点恒用 odom 原点）
  - 第二次跑（修复后）：到达 (2.5,0) 触发 `Row switch detected (delta=-180.0 deg), servo flipped to left (yaw=180)`；拐角 (2.5,1) 与终点 (0,1) 正确不翻；现场观察舵机实际转动 ✓
- [ ] 收尾：spec/ROS2.md 文档同步、分支合并回 main
