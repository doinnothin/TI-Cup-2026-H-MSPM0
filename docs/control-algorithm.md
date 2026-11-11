# 控制路径

本页从较高层面描述当前的代码结构。它不能取代赛题任务说明，也不声称每条路径都经本次清理重新验证过。

## 共享数据路径

```text
Vision UART bytes
  -> vision frame parser / freshness and validity checks
  -> ball state estimate and safety checks
  -> selected task policy (Q3 sequence or Q4/Q5/Q6 vehicle controller)
  -> pendulum geometry mapping / actuator command limits
  -> X42S command and feedback
```

视觉模块为外部实现。MCU 侧的解析器与控制边界接受逻辑位于 `user/code/vision_link.*` 以及相应的消费应用模块中。

## 本固件中的各问路径

- **Q2：** 通过既有的任务选择与 `user/bsp/lap_trace.*` 支持实现路线/圈数练习。
- **Q3：** 位于 `user/app/q3/` 的小球位置估计与控制序列。其增益、标定与序列受行为保护；仅格式化的改动在范围内。
- **Q4：** 共享车辆状态机应用 Q4 的路线、停车与小球恢复策略。
- **Q5：** `Q45Vehicle_UpdateBallControl` 选择 Q5 专属的 profile 与增益，应用来自 `user/app/q4/q5_vehicle_model.*` 的路线形状前馈，并叠加有界的第一/第二次弯道反馈与后段直线微调。`q5_dynamic_ball_control.*` 是一个独立的控制器模块；它未被当前车辆任务路径调用。
- **Q6：** `user/app/q6/q6_target.*` 捕获稳定的当前小球位置，或接受一个显式目标；随后共享车辆状态机运行相对于目标的闭环控制，并带有 Q6 门控的自适应。

## Q6 目标捕获

捕获采信位于配置位置边界内、新鲜、高置信、低速的帧。它会收集一个跨时间窗口的样本，拒绝间隙与过大的位置离散度，然后锁定候选目标。正常竞赛路径使用按键驱动的启动流程；仅在台架使用的自动选择、显式目标与静态测试宏在 `q6_config.h` 中默认禁用，除非某次构建将其覆盖。

Q6 仍共享 Q4/Q5 状态机与执行器接口。因此“Q6 模块”指的是一个较窄的目标/配置适配器加上 Q6 门控分支，而非一个完全独立的控制器实现。

Q6 像素行准入谓词由 `q6_target.*` 负责。共享的 Q45 控制器仅在 Q6 模式下、于视觉样本到达就绪或小球估计器之前调用它；Q4/Q5 保留其既有的帧检查，不带这一 Q6 专属限制。它消费既有的 `pixel_cy` 字段，且不改变视觉协议或识别算法。
