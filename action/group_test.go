package action

import (
	"math"
	"testing"
	"time"

	"motion/axis"
)

func testGroup(t *testing.T, policy StopPolicy, n int) *Group {
	t.Helper()
	cfgs := make([]axis.Config, n)
	for i := range cfgs {
		cfgs[i] = axis.Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001}
	}
	g, err := NewGroup(policy, cfgs...)
	if err != nil {
		t.Fatal(err)
	}
	clk := time.Unix(1_700_000_000, 0)
	g.now = func() time.Time {
		clk = clk.Add(time.Millisecond)
		return clk
	}
	return g
}

func groupStepUntil(t *testing.T, g *Group, cond func() bool) {
	t.Helper()
	for i := 0; i < 5_000_000; i++ {
		g.Step()
		if cond() {
			return
		}
	}
	t.Fatal("condition never met")
}

// A group move completes only when EVERY axis rests on its own target.
func TestGroupCompletesOnlyWhenAllArrive(t *testing.T) {
	g := testGroup(t, TerminateOnLimit, 3)
	m, err := g.Enqueue(1, 5, 10)
	if err != nil {
		t.Fatal(err)
	}
	sawPartial := false
	for m.State() == Active {
		g.Step()
		// While any axis is still traveling the move must stay Active.
		arrived := 0
		for i := 0; i < 3; i++ {
			if g.Axis(i).AtTarget() {
				arrived++
			}
		}
		if arrived > 0 && arrived < 3 && m.State() != Active {
			t.Fatalf("move %v with only %d/3 axes arrived", m.State(), arrived)
		}
		if arrived > 0 && arrived < 3 {
			sawPartial = true
		}
	}
	if !sawPartial {
		t.Fatal("expected a window where some but not all axes had arrived")
	}
	if m.State() != Completed {
		t.Fatalf("state=%v, want Completed", m.State())
	}
	want := []float64{1, 5, 10}
	for i, w := range want {
		if p := g.Axis(i).Position(); math.Abs(p-w) > 1e-9 {
			t.Fatalf("axis %d pos=%v, want %v", i, p, w)
		}
		if v := g.Axis(i).Velocity(); v != 0 {
			t.Fatalf("axis %d vel=%v, want 0", i, v)
		}
	}
}

// Mid-motion retarget keeps positions/velocities continuous and lands on the
// new targets.
func TestGroupRetargetMidMotion(t *testing.T) {
	g := testGroup(t, TerminateOnLimit, 2)
	m, err := g.Enqueue(100, -100)
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 500; i++ {
		g.Step()
	}
	prev := g.Positions()
	prevV := []float64{g.Axis(0).Velocity(), g.Axis(1).Velocity()}
	m2, err := g.RetargetActive(3, -4)
	if err != nil {
		t.Fatal(err)
	}
	if m2 != m {
		t.Fatal("retarget should reuse the active group move")
	}
	after := g.Positions()
	for i := range prev {
		if after[i] != prev[i] || g.Axis(i).Velocity() != prevV[i] {
			t.Fatalf("axis %d jumped on retarget: p %v->%v v %v->%v",
				i, prev[i], after[i], prevV[i], g.Axis(i).Velocity())
		}
	}
	groupStepUntil(t, g, func() bool { return m.State() == Completed })
	for i, w := range []float64{3, -4} {
		if p := g.Axis(i).Position(); math.Abs(p-w) > 1e-9 {
			t.Fatalf("axis %d pos=%v, want %v", i, p, w)
		}
	}
}

// One axis emergency-stopping terminates the group motion: the other axes
// brake in a controlled way, the active move is aborted and the queue ends.
func TestGroupEmergencyStopCoordinates(t *testing.T) {
	g := testGroup(t, TerminateOnLimit, 3)
	m, _ := g.Enqueue(100, 100, 100)
	queued, _ := g.Enqueue(200, 200, 200)
	for i := 0; i < 500; i++ {
		g.Step()
	}
	g.Axis(1).EmergencyStop()
	g.Step()
	if m.State() != Aborted || m.AbortReason() != AbortedEmergencyStop {
		t.Fatalf("active=%v reason=%v", m.State(), m.AbortReason())
	}
	if queued.State() != Canceled {
		t.Fatalf("queued=%v, want Canceled", queued.State())
	}
	// The healthy axes decelerate to rest instead of continuing to target.
	groupStepUntil(t, g, func() bool {
		return g.Axis(0).Velocity() == 0 && g.Axis(2).Velocity() == 0
	})
	for i := 0; i < 3; i++ {
		if g.Axis(i).AtTarget() {
			t.Fatalf("axis %d must not report arrival after group abort", i)
		}
	}
	// Recovery: reset and run a fresh motion.
	groupStepUntil(t, g, func() bool { return g.Axis(1).Velocity() == 0 })
	if err := g.Reset(); err != nil {
		t.Fatal(err)
	}
	m2, _ := g.Enqueue(1, 2, 3)
	groupStepUntil(t, g, func() bool { return m2.State() == Completed })
	for i, w := range []float64{1, 2, 3} {
		if p := g.Axis(i).Position(); math.Abs(p-w) > 1e-9 {
			t.Fatalf("axis %d pos=%v, want %v", i, p, w)
		}
	}
}

// WaitAndResume: one axis soft-stopping pauses the whole group; once all
// axes are at rest the group resumes and reaches the original targets.
func TestGroupWaitAndResume(t *testing.T) {
	g := testGroup(t, WaitAndResume, 3)
	m, _ := g.Enqueue(10, 20, 30)
	for i := 0; i < 300; i++ {
		g.Step()
	}
	g.Axis(2).SoftStop() // external constraint on one axis
	g.Step()
	if !g.Waiting() {
		t.Fatal("group should be waiting after one axis soft-stopped")
	}
	if m.State() != Active {
		t.Fatalf("active=%v, want Active while waiting", m.State())
	}
	// All axes decelerate to rest as a group.
	groupStepUntil(t, g, func() bool {
		for i := 0; i < 3; i++ {
			if g.Axis(i).Velocity() != 0 {
				return false
			}
		}
		return true
	})
	// Then the group resumes and every axis reaches its own target.
	groupStepUntil(t, g, func() bool { return m.State() == Completed })
	for i, w := range []float64{10, 20, 30} {
		if p := g.Axis(i).Position(); math.Abs(p-w) > 1e-9 {
			t.Fatalf("axis %d pos=%v, want %v", i, p, w)
		}
	}
}

// TerminateOnLimit: one axis soft-stopping ends the queue for the group.
func TestGroupTerminateOnLimit(t *testing.T) {
	g := testGroup(t, TerminateOnLimit, 2)
	m, _ := g.Enqueue(100, 100)
	queued, _ := g.Enqueue(200, 200)
	for i := 0; i < 300; i++ {
		g.Step()
	}
	g.Axis(0).SoftStop()
	g.Step()
	if m.State() != Aborted || m.AbortReason() != AbortedSoftStop {
		t.Fatalf("active=%v reason=%v", m.State(), m.AbortReason())
	}
	if queued.State() != Canceled {
		t.Fatalf("queued=%v, want Canceled", queued.State())
	}
	groupStepUntil(t, g, func() bool {
		return g.Axis(0).Velocity() == 0 && g.Axis(1).Velocity() == 0
	})
	if g.Axis(1).AtTarget() {
		t.Fatal("healthy axis must not report arrival after group termination")
	}
}

// Very short coordinated moves land exactly on every target.
func TestGroupShortMove(t *testing.T) {
	g := testGroup(t, TerminateOnLimit, 3)
	targets := []float64{1e-4, -5e-5, 2e-4}
	m, _ := g.Enqueue(targets...)
	groupStepUntil(t, g, func() bool { return m.State() == Completed })
	for i, w := range targets {
		if p := g.Axis(i).Position(); p != w {
			t.Fatalf("axis %d pos=%v, want exact %v", i, p, w)
		}
	}
}

// Saving while the group is decelerating into a wait restores and resumes
// under the same rules.
func TestGroupSaveRestoreDuringWait(t *testing.T) {
	g := testGroup(t, WaitAndResume, 2)
	firstMove, _ := g.Enqueue(10, 20)
	secondMove, _ := g.Enqueue(30, 40)
	for i := 0; i < 300; i++ {
		g.Step()
	}
	g.Axis(0).SoftStop()
	g.Step()
	if !g.Waiting() {
		t.Fatal("expected waiting")
	}
	// Save mid-deceleration.
	data, err := g.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreGroup(data)
	if err != nil {
		t.Fatal(err)
	}
	if !r.Waiting() {
		t.Fatal("restored group should still be waiting")
	}
	if r.Active() == nil || r.Active().ID != firstMove.ID {
		t.Fatal("restored group lost its active move")
	}
	if len(r.Pending()) != 1 || r.Pending()[0].ID != secondMove.ID {
		t.Fatal("restored group lost its queue")
	}
	// The restored group finishes braking, resumes and completes both moves.
	act := r.Active()
	pend := r.Pending()[0]
	groupStepUntil(t, r, func() bool { return r.Active() == nil && len(r.Pending()) == 0 })
	if act.State() != Completed || pend.State() != Completed {
		t.Fatalf("states %v %v, want both Completed", act.State(), pend.State())
	}
	for i, w := range []float64{30, 40} {
		if p := r.Axis(i).Position(); math.Abs(p-w) > 1e-9 {
			t.Fatalf("axis %d pos=%v, want %v", i, p, w)
		}
	}
}

// Saving while the group is terminating restores the same end state: aborted
// active, empty queue, axes braking to rest.
func TestGroupSaveRestoreDuringTerminate(t *testing.T) {
	g := testGroup(t, TerminateOnLimit, 2)
	m, _ := g.Enqueue(100, 100)
	for i := 0; i < 300; i++ {
		g.Step()
	}
	g.Axis(1).SoftStop()
	g.Step()
	data, err := g.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreGroup(data)
	if err != nil {
		t.Fatal(err)
	}
	if m.State() != Aborted {
		t.Fatalf("active=%v, want Aborted", m.State())
	}
	if len(r.Pending()) != 0 || r.Active() != nil {
		t.Fatal("restored group should have no active or pending moves")
	}
	groupStepUntil(t, r, func() bool {
		return r.Axis(0).Velocity() == 0 && r.Axis(1).Velocity() == 0
	})
	// A fresh motion can be commanded afterwards.
	m2, _ := r.Enqueue(1, 1)
	groupStepUntil(t, r, func() bool { return m2.State() == Completed })
}
