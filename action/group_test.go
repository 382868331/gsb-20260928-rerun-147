package action

import (
	"math"
	"testing"
	"time"

	"motion/axis"
)

func testGroup(t *testing.T, reaction ConstrainedReaction) *Group {
	t.Helper()
	cfg := axis.Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001}
	g, err := NewGroup([]axis.Config{cfg, cfg, cfg}, reaction)
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

func stepGroupUntil(t *testing.T, g *Group, cond func() bool) {
	t.Helper()
	for i := 0; i < 5_000_000; i++ {
		g.Step()
		if cond() {
			return
		}
	}
	t.Fatal("condition never met")
}

func TestGroupCompletesOnlyWhenAllAxesArrive(t *testing.T) {
	g := testGroup(t, ReactDecelerateAndWait)
	m, err := g.Enqueue([]float64{10, -5, 0.02}) // one very short leg
	if err != nil {
		t.Fatal(err)
	}
	if m.State() != Active {
		t.Fatalf("group move not active: %v", m.State())
	}
	// Mid-motion: the short axis arrives first; the group must NOT complete.
	stepGroupUntil(t, g, func() bool { return g.Axes()[2].AtTarget() })
	if m.State() == Completed {
		t.Fatal("group pretended to arrive while other axes still moving")
	}
	stepGroupUntil(t, g, func() bool { return m.State() == Completed })
	want := []float64{10, -5, 0.02}
	for i, ax := range g.Axes() {
		if math.Abs(ax.Position()-want[i]) > 1e-9 || ax.Velocity() != 0 {
			t.Fatalf("axis %d final state wrong: p=%v v=%v", i, ax.Position(), ax.Velocity())
		}
	}
}

func TestGroupRetargetMidMotion(t *testing.T) {
	g := testGroup(t, ReactDecelerateAndWait)
	m, err := g.Enqueue([]float64{100, 100, 100})
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 400; i++ {
		g.Step()
	}
	velBefore := []float64{g.Axes()[0].Velocity(), g.Axes()[1].Velocity(), g.Axes()[2].Velocity()}
	m2, err := g.RetargetActive([]float64{20, -30, 5})
	if err != nil || m2 != m {
		t.Fatalf("retarget wrong: %v %p %p", err, m, m2)
	}
	g.Step()
	for i, ax := range g.Axes() {
		if dv := math.Abs(ax.Velocity() - velBefore[i]); dv > 20*0.001+1e-9 {
			t.Fatalf("axis %d velocity jump on retarget: dv=%v", i, dv)
		}
	}
	stepGroupUntil(t, g, func() bool { return m.State() == Completed })
	want := []float64{20, -30, 5}
	for i, ax := range g.Axes() {
		if math.Abs(ax.Position()-want[i]) > 1e-9 {
			t.Fatalf("axis %d retarget result wrong: p=%v", i, ax.Position())
		}
	}
}

func TestGroupConstrainedAxisDeceleratesAndResumes(t *testing.T) {
	g := testGroup(t, ReactDecelerateAndWait)
	m, err := g.Enqueue([]float64{100, 100, 100})
	if err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 400; i++ {
		g.Step()
	}
	// Axis 1 is constrained externally (soft stop commanded at axis layer).
	g.Axes()[1].SoftStop()
	stepGroupUntil(t, g, func() bool { return g.Holding() })
	// The whole group must decelerate together, not run on.
	stepGroupUntil(t, g, func() bool {
		for _, ax := range g.Axes() {
			if ax.Velocity() != 0 {
				return false
			}
		}
		return true
	})
	if m.State() != Active {
		t.Fatalf("held move must stay active, got %v", m.State())
	}
	// Constraint clears: the group resumes and every axis reaches its target.
	stepGroupUntil(t, g, func() bool { return m.State() == Completed })
	for i, ax := range g.Axes() {
		if math.Abs(ax.Position()-100) > 1e-9 {
			t.Fatalf("axis %d did not reach group target: p=%v", i, ax.Position())
		}
	}
}

func TestGroupConstrainedAxisAbortAll(t *testing.T) {
	g := testGroup(t, ReactAbortAll)
	m1, _ := g.Enqueue([]float64{100, 100, 100})
	m2, _ := g.Enqueue([]float64{200, 200, 200})
	for i := 0; i < 400; i++ {
		g.Step()
	}
	g.Axes()[0].EmergencyStop()
	stepGroupUntil(t, g, func() bool { return m1.State() == Aborted })
	if m2.State() != Canceled {
		t.Fatalf("queued group move must be canceled: %v", m2.State())
	}
	// Unconstrained axes brake to rest instead of continuing.
	stepGroupUntil(t, g, func() bool {
		return g.Axes()[1].Velocity() == 0 && g.Axes()[2].Velocity() == 0
	})
	if g.Active() != nil || len(g.Pending()) != 0 {
		t.Fatal("group must be drained after abort-all")
	}
}

func TestGroupEmergencyStopAndRecovery(t *testing.T) {
	g := testGroup(t, ReactDecelerateAndWait)
	m, _ := g.Enqueue([]float64{100, 100, 100})
	for i := 0; i < 400; i++ {
		g.Step()
	}
	g.EmergencyStop()
	if m.State() != Stopping || m.AbortReason() != AbortedEmergencyStop {
		t.Fatalf("move must be Stopping while the axis is still braking: %v/%v", m.State(), m.AbortReason())
	}
	stepGroupUntil(t, g, func() bool {
		for _, ax := range g.Axes() {
			if ax.Velocity() != 0 {
				return false
			}
		}
		return true
	})
	// Settled but still latched: ownership is not released and new work waits.
	if _, err := g.Enqueue([]float64{1, 1, 1}); err != nil {
		t.Fatal(err)
	}
	if got := len(g.Pending()); got != 1 {
		t.Fatalf("move must wait while e-stop latched, pending=%d", got)
	}
	if err := g.Reset(); err != nil {
		t.Fatal(err)
	}
	stepGroupUntil(t, g, func() bool { return m.State() == Aborted })
	stepGroupUntil(t, g, func() bool { return g.Active() == nil && len(g.Pending()) == 0 })
	for i, ax := range g.Axes() {
		if math.Abs(ax.Position()-1) > 1e-9 {
			t.Fatalf("axis %d post-reset move wrong: p=%v", i, ax.Position())
		}
	}
}

func TestGroupSaveRestoreMidHold(t *testing.T) {
	g := testGroup(t, ReactDecelerateAndWait)
	if _, err := g.Enqueue([]float64{100, 100, 100}); err != nil {
		t.Fatal(err)
	}
	m2, _ := g.Enqueue([]float64{50, 50, 50})
	for i := 0; i < 400; i++ {
		g.Step()
	}
	g.Axes()[2].SoftStop()
	for i := 0; i < 5; i++ {
		g.Step() // group is decelerating together
	}
	if !g.Holding() {
		t.Fatal("expected group to be holding")
	}
	data, err := g.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreGroup(data)
	if err != nil {
		t.Fatal(err)
	}
	if !r.Holding() {
		t.Fatal("restored group lost hold state")
	}
	// Same rules: finish braking, resume the held move, then run the queue.
	// The held move completes first; the queued move is still waiting.
	stepGroupUntil(t, r, func() bool {
		return r.Active() != nil && r.Active().ID == m2.ID
	})
	for i, ax := range r.Axes() {
		if math.Abs(ax.Position()-100) > 1e-9 {
			t.Fatalf("axis %d resumed goal wrong: p=%v", i, ax.Position())
		}
	}
	stepGroupUntil(t, r, func() bool { return r.Active() == nil && len(r.Pending()) == 0 })
	for i, ax := range r.Axes() {
		if math.Abs(ax.Position()-50) > 1e-9 || ax.Velocity() != 0 {
			t.Fatalf("axis %d queued goal wrong: p=%v v=%v", i, ax.Position(), ax.Velocity())
		}
	}
}
