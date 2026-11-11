# 架构与模块职责划分

## 范围

本仓库是 2026 年 H 题的 MCU 控制固件。它包含 Q2–Q6 各问的控制路径，以及 MCU 侧的视觉串口消费端。Q1 的视觉识别固件为外部实现，不由 `user/code/vision_link.*` 表示。

## 运行时流程

```text
Startup: SysConfig -> BSP/devices -> application modules
  |
  +-- foreground loop (main.c composition root)
       1. capture current tick -> AppTelemetry_BeginCycle snapshots diagnostic cadence
       2. VisionLink_Poll -> BallActuator_Task
       3. BallPositionControl_Task -> Q3Sequence_Task
       4. Q45Vehicle_Task (Q4/Q5/Q6 shared vehicle state machine)
       5. DebugLog_PublishControlFrame
       6. calibration/identification tasks -> DebugLog_Task
       7. AppTelemetry_PublishDue (slow periodic records)
       8. consume key events / coordinate mutually exclusive modes
       9. process stop/start-pending conditions -> low-rate OLED update

Q45Vehicle_Task
  +-- Q4 route and static recovery policy
  +-- Q5 motion model / dynamic ball controller
  +-- Q6 target capture and mode-gated target-relative behavior
  +-- pendulum geometry -> BallActuator / X42S

TIMG6 ISR -> timebase and bounded chassis tick bookkeeping.
UART/GPIO ISRs -> receive bytes or set event flags; foreground code parses,
controls, formats logs and refreshes the display.
```

该图概括了 `user/main.c` 中的源码级执行顺序，并非逐周期精确的时序图。修改调度器时请保持这一顺序，尤其是视觉/执行器/控制的快速路径，以及每轮循环开始时进行的遥测节拍快照。

## 职责划分

| 层级 | 负责 | 不得负责 |
|---|---|---|
| `user/main.c` | 启动、任务选择、按键协调、顶层轮询、任务排序、模式路由与显示刷新集成。 | 控制器私有状态，或超出既有启动流程的外设寄存器配置。 |
| `user/app/app_key_input.*` | 在前台消费由 ISR 记录的 GPIO 按键事件，以及既有的 15 ms 事件消抖。 | 按键到任务的映射、ISR 事件产生，或控制状态转换。 |
| `user/app/app_status_display.*` | 依据调用方提供的状态快照，在前台渲染 Q4/Q5/Q6 车辆状态主体。 | 读取或改变控制器状态、刷新 OLED，或在 ISR 中做格式化。 |
| `user/app/app_telemetry.*` | 为跨应用模块的既有诊断记录选择周期并控制前台节拍。 | 线格式序列化、直接 UART 发送，或控制决策。 |
| `user/app/q3/` | Q3 估计器、控制器、几何标定与序列。 | Q4/Q5/Q6 调参以及 UART/设备驱动。 |
| `user/app/q4/` | 共享的 Q4/Q5/Q6 车辆状态机、动态 Q5 模型/控制器、Q4 策略/配置。 | 视觉模块固件，或 SysConfig 生成的内容。 |
| `user/app/q6/` | Q6 目标选择/捕获、Q6 视觉行准入策略，以及 Q6 专属参数。 | Q3 行为/配置，或 Q4/Q5 参数归属。 |
| `user/app/ball_actuator/` | 小球摆杆执行器抽象与标定。 | 底盘路线选择。 |
| `user/bsp/` | 时钟、中断粘合代码、底层底盘控制、PID 原语与路线跟踪。 | 竞赛级目标策略。 |
| `user/code/` | 设备驱动、UART 链路、X42S 协议与调试记录序列化。 | 竞赛任务状态机。 |
| TI 生成/厂商文件 | 设备初始化与厂商 API。 | 应用层手工编辑。 |

## 变更边界

- Q3 的控制行为、配置值、标定与序列受保护。格式化与说明性文档可以改进；功能性修改需要证明存在兼容性需求，并提供单独的回归证据。
- 在记录或集成 Q6 时，保持 Q4/Q5 调参与共享控制行为不变。
- 将 `empty.syscfg` 视为可编辑的硬件配置源。仅通过匹配的 SysConfig 版本并经过显式审查后才重新生成生成文件。
- 保持 ISR 工作有界且非阻塞。不得将格式化、浮点控制计算、OLED 更新或阻塞式串口输出移入 ISR。
- 保持 `main.c` 作为启动、任务选择与循环排序的合成根。小型应用适配器可以负责独立的输入消费、状态主体渲染或周期性遥测节拍；但须保留调用方的按键路由、显示的清除/绘制/刷新顺序以及诊断发布顺序。
- 将按键到任务的路由保留在 `main.c` 中；`app_key_input.*` 仅消费既有的 ISR 标志，并保留既有的消抖间隔与时间戳状态。
- `app_telemetry.*` 仅负责周期性日志节拍与到期记录的发布。快速控制帧发布与队列排空的位置仍保留在主循环中，且遥测节拍仍像以前一样在视觉/控制工作之前选定。
- 将 Q6 专属的视觉准入策略保留在 `user/app/q6/` 中；共享的 Q45 控制器决定该策略何时生效，而 Q4/Q5 的帧接受逻辑保持不变。
- 优先复用既有的 C 接口与定长数据结构。本项目无需 RTOS、动态内存分配或新框架。
