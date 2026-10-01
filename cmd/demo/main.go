// Command demo is a standalone walkthrough of the motion framework: single
// axis lifecycle, both soft-stop policies, save/restore mid-stop, and
// coordinated multi-axis behavior when one axis becomes constrained.
package main

import (
	"fmt"

	"motion/action"
	"motion/axis"
)

var cfg = axis.Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001}

func run(e *action.Executor, cond func() bool) {
	for i := 0; i < 5_000_000 && !cond(); i++ {
		e.Step()
	}
}

func runGroup(g *action.Group, cond func() bool) {
	for i := 0; i < 5_000_000 && !cond(); i++ {
		g.Step()
	}
}

func main() {
	fmt.Println("== 1. single-axis queue and lifecycle ==")
	e, _ := action.NewExecutor(cfg)
	m1 := e.Enqueue(10)
	m2 := e.Enqueue(-5)
	run(e, func() bool { return m2.State() == action.Completed })
	fmt.Printf("m1=%v m2=%v pos=%.3f vel=%.3f\n\n", m1.State(), m2.State(), e.Axis().Position(), e.Axis().Velocity())

	fmt.Println("== 2. mid-motion retarget (continuous replan) ==")
	e2, _ := action.NewExecutor(cfg)
	mr := e2.Enqueue(100)
	for i := 0; i < 400; i++ {
		e2.Step()
	}
	fmt.Printf("cruising at pos=%.3f vel=%.3f -> retarget to 0.2 ahead\n", e2.Axis().Position(), e2.Axis().Velocity())
	e2.RetargetActive(e2.Axis().Position() + 0.2)
	run(e2, func() bool { return mr.State() == action.Completed })
	fmt.Printf("completed at pos=%.6f vel=%.3f\n\n", e2.Axis().Position(), e2.Axis().Velocity())

	fmt.Println("== 3. soft stop: end queue vs keep goal ==")
	e3, _ := action.NewExecutor(cfg)
	ms := e3.Enqueue(100)
	e3.Enqueue(200)
	for i := 0; i < 400; i++ {
		e3.Step()
	}
	e3.SoftStop() // StopEndQueue
	run(e3, func() bool { return e3.Axis().Velocity() == 0 })
	fmt.Printf("end-queue: active=%v stopped-at=%.3f (queue dropped)\n", ms.State(), e3.Axis().Position())

	e4, _ := action.NewExecutor(cfg)
	mk := e4.Enqueue(100)
	mk2 := e4.Enqueue(150)
	for i := 0; i < 400; i++ {
		e4.Step()
	}
	e4.SoftStopWithPolicy(action.StopKeepGoal)
	run(e4, func() bool { return mk2.State() == action.Completed })
	fmt.Printf("keep-goal: resumed after stop, m1=%v m2=%v pos=%.3f\n\n", mk.State(), mk2.State(), e4.Axis().Position())

	fmt.Println("== 4. emergency stop and recovery ==")
	e5, _ := action.NewExecutor(cfg)
	me := e5.Enqueue(100)
	for i := 0; i < 400; i++ {
		e5.Step()
	}
	e5.EmergencyStop()
	run(e5, func() bool { return e5.Axis().Velocity() == 0 })
	fmt.Printf("e-stop latched: move=%v estop=%v\n", me.State(), e5.Axis().EmergencyStopped())
	e5.Reset()
	mr2 := e5.Enqueue(1)
	run(e5, func() bool { return mr2.State() == action.Completed })
	fmt.Printf("after reset: new move completed at pos=%.3f\n\n", e5.Axis().Position())

	fmt.Println("== 5. save during a stop, restore, continue ==")
	e6, _ := action.NewExecutor(cfg)
	mh := e6.Enqueue(100)
	for i := 0; i < 400; i++ {
		e6.Step()
	}
	e6.SoftStopWithPolicy(action.StopKeepGoal)
	for i := 0; i < 10; i++ {
		e6.Step()
	}
	data, _ := e6.Save()
	re, _ := action.RestoreExecutor(data)
	fmt.Printf("saved mid-stop (mode=%v), restored pos=%.3f vel=%.3f\n", e6.Axis().Mode(), re.Axis().Position(), re.Axis().Velocity())
	run(re, func() bool { return mh.State() == action.Completed })
	fmt.Printf("restored executor resumed held goal: pos=%.3f vel=%.3f\n\n", re.Axis().Position(), re.Axis().Velocity())

	fmt.Println("== 6. coordinated group: one axis constrained ==")
	g, _ := action.NewGroup([]axis.Config{cfg, cfg, cfg}, action.ReactDecelerateAndWait)
	gm, _ := g.Enqueue([]float64{100, 100, 100})
	for i := 0; i < 400; i++ {
		g.Step()
	}
	g.Axes()[1].SoftStop() // axis 1 constrained at the axis layer
	runGroup(g, func() bool { return gm.State() == action.Completed })
	fmt.Printf("group decelerated together, waited, resumed: pos=(%.3f, %.3f, %.3f)\n",
		g.Axes()[0].Position(), g.Axes()[1].Position(), g.Axes()[2].Position())

	g2, _ := action.NewGroup([]axis.Config{cfg, cfg, cfg}, action.ReactAbortAll)
	ga, _ := g2.Enqueue([]float64{100, 100, 100})
	for i := 0; i < 400; i++ {
		g2.Step()
	}
	g2.Axes()[0].EmergencyStop()
	runGroup(g2, func() bool { return ga.State() == action.Aborted })
	fmt.Printf("abort-all reaction: move=%v, others braked to vel=(%.3f, %.3f, %.3f)\n",
		ga.State(), g2.Axes()[0].Velocity(), g2.Axes()[1].Velocity(), g2.Axes()[2].Velocity())
}
