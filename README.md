# motion — 工业动作执行框架（虚拟轴）

纯仿真的运动控制与动作编排框架，不驱动真实硬件。分两层：

- `axis`：底层运动控制。单个虚拟伺服轴，固定采样周期，|v| ≤ MaxSpeed、|a| ≤ MaxAccel，每次设定目标都从**当前真实位置/速度**重新规划梯形速度曲线，改目标不会产生跳变。
- `action`：动作编排层。FIFO 队列 + 生命周期状态机，单轴用 `Executor`，多轴协同用 `Group`，多个组共享同一组命名轴用 `Cell`。

运行演示：`go run ./cmd/demo`；测试：`go test ./...`。

## 接入方式

```go
cfg := axis.Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001}
e, _ := action.NewExecutor(cfg)
m := e.Enqueue(100)        // 入队即调度
for m.State() != action.Completed {
    e.Step()               // 每个控制周期调用一次
}
```

多轴协同：

```go
g, _ := action.NewGroup([]axis.Config{cfg, cfg, cfg}, action.ReactDecelerateAndWait)
gm, _ := g.Enqueue([]float64{100, 100, 100}) // 每个轴一个目标
for gm.State() != action.Completed {
    g.Step()
}
```

## 生命周期

每个动作（`Move` / `GroupMove`）走同一条状态机：

```
Pending -> Active -> Completed   正常完成（在目标点静止）
                  \-> Aborted    已开始但被中断（软停 / 急停 / 被取代）
Pending -> Canceled              未开始就被移出队列
```

`AbortReason` 区分 `AbortedSoftStop` / `AbortedEmergencyStop` / `AbortedReplaced`。`Move.StartPos()` 记录动作实际起始位置。

## 两层“软停”的不同含义

- **axis 层** `Axis.SoftStop()`：只负责以最大减速度在当前自然停车点停住，并丢弃轴级目标；不知道队列的存在。
- **action 层**由调用者用 `StopPolicy` 选择队列与目标的命运：
  - `StopEndQueue`（`SoftStop()` 默认）：中止活动动作（Aborted）、取消全部排队动作（Canceled）。
  - `StopKeepGoal`（`SoftStopWithPolicy(action.StopKeepGoal)`）：先刹车停住，**保留目标与队列**，停稳后从真实停车位置重新下发同一目标继续执行，后续队列照常运行。

急停在两层都是锁存故障：`EmergencyStop` 刹车并锁存，新目标被拒绝，`Reset` 在停稳后解除。

## 多轴协同规则

`Group` 的协调铁律：**任一轴进入受限状态（急停，或组外下发的软停），整组一起减速**，绝不让其余轴继续走然后假装整组一起到位。组动作只有在**每个轴都各自 AtTarget** 时才置 Completed。

受限时的反应由 `ConstrainedReaction` 选择：

- `ReactDecelerateAndWait`：所有未受限轴一起软停，活动组动作保持 Active 等待；约束解除且全组停稳后，从真实停车位置恢复原目标继续。
- `ReactAbortAll`：整组减速，活动组动作 Aborted、排队组动作 Canceled。

`RetargetActive` 中途改目标：所有轴从当前位置/速度连续重规划，无跳变。`Group.EmergencyStop` / `Group.Reset` 作用于全部轴。

## 多组共享轴与显式交接（Cell）

`Cell` 是命名虚拟轴的注册表，多个 `Group` 可以各取其中一部分轴，轴集可以交叉（例如 A 组用 `x,y`、B 组用 `y,z`）。铁律是**同一根轴同一时刻只被一个组动作占用**，组之间按明确条件交接，绝不允许两个组同时向同一根轴下发互相覆盖的目标。

```go
c := action.NewCell()
c.AddAxis("y", cfg) // 以及其它轴
ga, _ := c.NewGroup("A", []string{"x", "y"}, action.ReactDecelerateAndWait)
gb, _ := c.NewGroup("B", []string{"y", "z"}, action.ReactDecelerateAndWait)
ga.EnqueueWithHandoff([]float64{10, 10}, nil, nil, nil)
// B 只有在 y 已停稳且当前虚拟位置 >= 10 时才取得 y（以及 z）
gb.EnqueueWithHandoff([]float64{20, 10}, nil,
    &action.HandoffConditions{Axes: []action.AxisCondition{
        {Axis: "y", AtLeast: 10, AtLeastSet: true}}}, nil)
for { c.Step() }
```

- **组级等待**：未取得全部所需轴（或条件未满足）的动作是 `Waiting`，此时一根轴都未占用；轴完全不相交的组不受影响，继续运动。
- **交接记录**：旧组动作释放轴时发布交接记录（旧/新动作 ID、释放时的虚拟位置、是否停稳、时间）；新组取得轴时落到 `GroupMove.Handoffs()`。恢复后按**当前虚拟位置重新核对**条件，不凭旧记录假定已到位。
- **已执行位移不可撤销**：动作一旦占用并移动过轴即为 `Partial`。等待中的动作可由操作者取消（`Group.CancelMove` / `CancelWaiting`）；交接后的新目标可用 `RetargetActive` 修改，旧组已走过的位移不会被回滚。
- **安全停止**：软停/急停把活动动作置为 `Stopping`（区别于尚未取得轴的 `Waiting`），组在停稳前继续占有轴；急停锁存期间**不会自动释放仍未停稳的轴**，必须 `Reset`（且轴已静止）后才释放。其它独立轴上的组照常运行。旧组迟到的完成事件（含停止/完成）只通过其自身动作 ID 进入事件日志（`Cell.Events()` / 完成回调），不会归到新占用者。
- **保存恢复**：`Cell.Save` / `RestoreCell` 保存轴的真实位置/速度、占用关系、等待与停止中的动作、交接条件、门信号和组注册顺序；恢复后组顺序与实际终点一致。

## 保存与恢复

`Executor.Save` / `RestoreExecutor`、`Group.Save` / `RestoreGroup` 序列化完整动态状态（位置、速度、模式、目标、队列、保持标志）。**保存发生在停止过程中也按同一规则继续**：恢复后先完成刹车，再按 StopKeepGoal / 组等待规则从真实位置恢复目标，不会把任何动作误判为已到位。

## 目录

- `axis/`：单轴运动学与软停/急停/快照
- `action/`：生命周期、单轴 Executor、多轴 Group、多组共享轴 Cell 及各自快照持久化
- `cmd/demo/`：独立演示（队列、改目标、两种软停、急停恢复、停止中保存、多轴协同）
