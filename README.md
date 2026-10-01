# 工业动作执行框架（虚拟轴）

一个纯仿真的运动控制与动作编排框架，不接触真实硬件。分两层：

- `axis`：底层运动控制。单根虚拟伺服轴，固定采样周期，速度/加速度受限，
  每次设定目标都从**当前真实位置与速度**重新规划梯形速度曲线，改目标不会跳变。
- `action`：动作编排层。单轴 FIFO 队列执行器（`Executor`）与多轴协同组（`Group`），
  管理动作生命周期、停止语义、保存/恢复。

## 快速开始

```bash
go test ./...      # 运行全部测试
go run ./cmd/demo  # 独立演示：队列、改目标、两种软停、急停恢复、多轴协同
```

最小接入示例：

```go
e, _ := action.NewExecutor(axis.Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001})
m := e.Enqueue(10)          // 目标位置 10，进入队列并自动调度
for m.State() != action.Completed {
    e.Step()                // 每个控制周期调用一次
}
// e.Axis().Position() == 10, e.Axis().Velocity() == 0
```

## 动作生命周期

单轴 `Move` 与多轴 `GroupMove` 共用同一套状态机：

```
Pending -> Active -> Completed    正常到位（每个轴都静止在自己的目标上）
                 \-> Canceled     未启动即被移除（队列被清空时）
                 \-> Aborted      已启动但被中断（软停/急停/被取代），
                                  AbortReason 记录原因
```

## 软停的两层含义

“软停”在两层语义不同，调用者按需选择：

**底层运动控制层（`axis`）**
- `Axis.SoftStop()`：丢弃目标，按最大减速度刹到自然停止点，之后可下达新目标。
- `Axis.Pause()`：保留目标刹车，速度归零后**自动继续**朝原目标运动。

**动作编排层（`action.Executor`）**
- `Executor.SoftStop()`：**结束队列**。活动中止（`Aborted/AbortedSoftStop`），
  待执行全部 `Canceled`，轴刹停。
- `Executor.SoftStopKeepTarget()`：**保留目标**。活动动作与队列原样保留，
  轴刹停后自动继续原目标，队列随后照常执行。
- `Executor.EmergencyStop()`：中止活动、清空队列并锁存急停，新目标被拒绝，
  直到 `Reset()`（需先停稳）。

## 多轴协同（`action.Group`）

```go
g, _ := action.NewGroup(action.WaitAndResume, cfgX, cfgY, cfgZ)
gm, _ := g.Enqueue(10, 20, 30)   // 每个轴一个目标
for gm.State() != action.Completed { g.Step() }
```

协同规则：

- **到位判定**：只有当**每根轴**都静止在自己的目标上，组动作才 `Completed`；
  部分轴先到位不会让整组“假装一起到位”。
- **任一轴进入受限状态**（被外部软停/急停）时，整组按策略响应：
  - `TerminateOnLimit`：所有轴受控刹停，活动中止、队列清空（急停则按急停原因中止）。
  - `WaitAndResume`：所有轴保留目标一起刹停，等受限轴解除且全组静止后，
    从各自真实位置/速度重新规划，继续朝原目标运动。
- 等待期间若任一轴急停，等待立即升级为整组终止。
- `Group.SoftStop()` / `Group.EmergencyStop()` 对整组所有轴同时生效。

## 保存与恢复

`Executor.Save/RestoreExecutor`、`Group.Save/RestoreGroup` 序列化完整动态状态
（位置、速度、模式、目标、队列、等待标志）。**停止过程中保存**也按同一规则继续：
恢复后刹停继续到静止、等待中的组继续等待再恢复、被清空的队列不会复活，
任何动作都不会被误认为已到位。

## 目录

- `axis/`：虚拟轴运动学与停止原语
- `action/`：单轴执行器、多轴协同组、生命周期与快照
- `cmd/demo/`：独立演示程序

