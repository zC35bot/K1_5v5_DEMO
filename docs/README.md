# K1 5v5 Demo 项目文档

本目录用于补齐仓库级说明，重点覆盖以下内容：

- 项目整体结构与运行链路
- `brain / vision / game_controller` 三个核心节点的代码逻辑
- 常用启动方式、参数入口和配置覆盖关系
- 修改参数时的注意事项、当前仓库的已知限制与易踩坑点

建议阅读顺序：

1. [项目总览](./architecture.md)
2. [Brain 模块说明](./brain.md)
3. [Vision 模块说明](./vision.md)
4. [参数与修改注意事项](./configuration.md)
5. [足球定位校准说明](./ball_localization_calibration.md)
6. [Brain 15 天冲刺规划](./brain_fast_iteration_plan.md)
7. [球定位误差预算与修复顺序](./ball_error_budget_and_fix_plan.md)

如果你只想快速定位：

- 想知道系统怎么启动：看 [项目总览](./architecture.md)
- 想改前锋/守门策略：看 [Brain 模块说明](./brain.md)
- 想改视觉模型、相机、标定：看 [Vision 模块说明](./vision.md)
- 想确认某个 YAML 字段到底有没有生效：看 [参数与修改注意事项](./configuration.md)
- 想调球的初始距离精度、看地面拟合校准：看 [足球定位校准说明](./ball_localization_calibration.md)
- 想分析为什么球会测远、为什么会跳、应该先修哪里：看 [球定位误差预算与修复顺序](./ball_error_budget_and_fix_plan.md)
- 想做守门员队长、传球配合、brain 快速拆模块：看 [Brain 15 天冲刺规划](./brain_fast_iteration_plan.md)
