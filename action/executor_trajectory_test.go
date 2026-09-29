package action

import (
	"math"
	"testing"

	"motion/axis"
)

// TestExecutorTrajectoryConstraints drives several targets (including a very short move and
// a reversal while cruising) through the executor and checks the FULL trajectory for
// speed/acceleration limits and no overshoot, not just the endpoint.
func TestExecutorTrajectoryConstraints(t *testing.T) {
	const vmax, amag, dt = 10.0, 20.0, 0.001
	cfg := axis.Config{MaxSpeed: vmax, MaxAccel: amag, Step: dt}

	run := func(targets []float64) {
		e, err := NewExecutor(cfg)
		if err != nil {
			t.Fatal(err)
		}
		prevP, prevV := 0.0, 0.0
		goal := targets[0]
		e.Enqueue(goal)
		for ti, tgt := range targets[1:] {
			_ = ti
			for {
				e.Step()
				p, v := e.Axis().Position(), e.Axis().Velocity()
				check(t, prevP, prevV, p, v, goal, vmax, amag, dt)
				prevP, prevV = p, v
				if e.Active() == nil && len(e.Pending()) == 0 {
					break
				}
			}
			// Issue the next goal from the current real state.
			e.Enqueue(tgt)
			goal = tgt
		}
		for e.Active() != nil || len(e.Pending()) != 0 {
			e.Step()
			p, v := e.Axis().Position(), e.Axis().Velocity()
			check(t, prevP, prevV, p, v, goal, vmax, amag, dt)
			prevP, prevV = p, v
		}
		if math.Abs(e.Axis().Position()-goal) > 1e-9 || e.Axis().Velocity() != 0 {
			t.Fatalf("final p=%v v=%v goal=%v", e.Axis().Position(), e.Axis().Velocity(), goal)
		}
	}

	run([]float64{0.05})
	run([]float64{50})
	run([]float64{100, -20})
	run([]float64{30, -0.02, 5})
}

func check(t *testing.T, p0, v0, p1, v1, goal, vmax, amag, dt float64) {
	t.Helper()
	if math.Abs(v1) > vmax+1e-9 {
		t.Fatalf("speed %v > %v near goal %v", v1, vmax, goal)
	}
	if dv := math.Abs(v1 - v0); dv > amag*dt+1e-9 {
		t.Fatalf("acceleration dv=%v > %v", dv, amag*dt)
	}
	if dp := math.Abs(p1 - p0); dp > vmax*dt+amag*dt*dt+1e-9 {
		t.Fatalf("step displacement %v too large", dp)
	}
}
