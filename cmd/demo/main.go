// Command demo is a standalone walkthrough of the motion framework: single
// axis queueing, retargeting, both soft-stop flavors, emergency-stop
// recovery, save/restore mid-stop, and a coordinated three-axis group.
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
	fmt.Println("== 1. single axis: queue, retarget, soft stops ==")
	e, _ := action.NewExecutor(cfg)
	m1 := e.Enqueue(10)
	e.Enqueue(20)
	for i := 0; i < 400; i++ {
		e.Step()
	}
	fmt.Printf("cruising: pos=%.3f vel=%.3f\n", e.Axis().Position(), e.Axis().Velocity())

	// Retarget mid-motion: no jump, continuous velocity.
	if _, err := e.RetargetActive(-5); err != nil {
		panic(err)
	}
	run(e, func() bool { return m1.State() == action.Completed })
	fmt.Printf("retargeted move done: pos=%.3f vel=%.3f state=%v\n",
		e.Axis().Position(), e.Axis().Velocity(), m1.State())

	// Soft stop that KEEPS the target: brake, then resume automatically.
	e.SoftStopKeepTarget()
	run(e, func() bool { return e.Axis().Velocity() == 0 })
	fmt.Printf("paused at pos=%.3f (target kept, queue intact: %d pending)\n",
		e.Axis().Position(), len(e.Pending()))
	run(e, func() bool { return e.Active() == nil && len(e.Pending()) == 0 })
	fmt.Printf("resumed and finished queue: pos=%.3f\n", e.Axis().Position())

	fmt.Println("\n== 2. soft stop that ENDS the queue + emergency stop ==")
	m2 := e.Enqueue(100)
	m3 := e.Enqueue(200)
	for i := 0; i < 300; i++ {
		e.Step()
	}
	e.SoftStop()
	fmt.Printf("soft stop: active=%v(%v) queued=%v\n", m2.State(), m2.AbortReason(), m3.State())
	run(e, func() bool { return e.Axis().Velocity() == 0 })

	m4 := e.Enqueue(50)
	for i := 0; i < 200; i++ {
		e.Step()
	}
	e.EmergencyStop()
	fmt.Printf("emergency stop: active=%v(%v) latched=%v\n",
		m4.State(), m4.AbortReason(), e.Axis().EmergencyStopped())
	run(e, func() bool { return e.Axis().Velocity() == 0 })
	if err := e.Reset(); err != nil {
		panic(err)
	}
	m5 := e.Enqueue(1)
	run(e, func() bool { return m5.State() == action.Completed })
	fmt.Printf("after reset: new move completed at pos=%.3f\n", e.Axis().Position())

	fmt.Println("\n== 3. save/restore during a stop ==")
	m6 := e.Enqueue(500)
	for i := 0; i < 300; i++ {
		e.Step()
	}
	e.SoftStop()
	data, _ := e.Save()
	r, _ := action.RestoreExecutor(data)
	run(r, func() bool { return r.Axis().Velocity() == 0 })
	fmt.Printf("restored mid-stop: mode=%v active=%v stoppedAt=%.3f\n",
		r.Axis().Mode(), m6.State(), r.Axis().Position())

	fmt.Println("\n== 4. coordinated 3-axis group (WaitAndResume) ==")
	g, _ := action.NewGroup(action.WaitAndResume, cfg, cfg, cfg)
	gm, _ := g.Enqueue(10, 20, 30)
	for i := 0; i < 300; i++ {
		g.Step()
	}
	g.Axis(2).SoftStop() // one axis becomes constrained
	g.Step()
	fmt.Printf("axis 2 constrained: group waiting=%v move=%v\n", g.Waiting(), gm.State())
	runGroup(g, func() bool { return gm.State() == action.Completed })
	p := g.Positions()
	fmt.Printf("group resumed and arrived together: pos=%.3f,%.3f,%.3f\n", p[0], p[1], p[2])

	fmt.Println("\n== 5. coordinated group (TerminateOnLimit) ==")
	g2, _ := action.NewGroup(action.TerminateOnLimit, cfg, cfg)
	gm2, _ := g2.Enqueue(100, 100)
	for i := 0; i < 300; i++ {
		g2.Step()
	}
	g2.Axis(0).EmergencyStop()
	g2.Step()
	runGroup(g2, func() bool {
		return g2.Axis(0).Velocity() == 0 && g2.Axis(1).Velocity() == 0
	})
	fmt.Printf("one axis estop: move=%v(%v) both axes at rest, none at target: %v,%v\n",
		gm2.State(), gm2.AbortReason(), g2.Axis(0).AtTarget(), g2.Axis(1).AtTarget())
}
