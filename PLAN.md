# PLAN.md — 舵机随巡航换行自动翻转

分支：`feat/servo-row-switch-flip`
Spec：`docs/superpowers/specs/2026-07-19-servo-row-switch-flip-design.md`（v6, Approved）
计划：`docs/superpowers/plans/2026-07-19-servo-row-switch-flip.md`

- [x] Task 0: 建分支 + PLAN.md 初始化
- [ ] Task 1: 参数声明、初始化成员、auto 推导、ServoCmd publisher
- [ ] Task 2: `_maybe_flip_servo` 几何判定 + tick 钩子 + 起点位姿记录
- [ ] Task 3: 检测冷却窗口
- [ ] Task 4: launch 参数接线 + servo 初始角 0 + 板端全量测试
- [ ] Task 5: 板端实车验证
