package action

import (
	"math"
	"testing"
	"time"

	"motion/axis"
)

func sharedCfg() axis.Config {
	return axis.Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001}
}

func testCell(t *testing.T) *Cell {
	t.Helper()
	c := NewCell()
	cf := sharedCfg()
	for _, n := range []string{"x", "y", "z"} {
		if err := c.AddAxis(n, cf); err != nil {
			t.Fatal(err)
		}
	}
	clk := time.Unix(1_700_000_000, 0)
	c.now = func() time.Time {
		clk = clk.Add(time.Millisecond)
		return clk
	}
	return c
}

func stepCellUntil(t *testing.T, c *Cell, cond func() bool) {
	t.Helper()
	for i := 0; i < 10_000_000; i++ {
		c.Step()
		if cond() {
			return
		}
	}
	t.Fatal("condition never met")
}

// Disjoint groups run concurrently; a shared axis hands over only once the
// explicit condition (at-rest position window) is met.
func TestCellSharedAxisHandoffByCondition(t *testing.T) {
	c := testCell(t)
	ga, err := c.NewGroup("A", []string{"x", "y"}, ReactDecelerateAndWait)
	if err != nil {
		t.Fatal(err)
	}
	gb, err := c.NewGroup("B", []string{"y", "z"}, ReactDecelerateAndWait)
	if err != nil {
		t.Fatal(err)
	}
	ma, err := ga.EnqueueWithHandoff([]float64{10, 10}, nil, nil, nil)
	if err != nil {
		t.Fatal(err)
	}
	mb, err := gb.EnqueueWithHandoff([]float64{20, 10}, nil, &HandoffConditions{
		Axes: []AxisCondition{{Axis: "y", AtLeast: 10, AtLeastSet: true}},
	}, nil)
	if err != nil {
		t.Fatal(err)
	}
	if ma.State() != Active {
		t.Fatalf("A must start, got %v", ma.State())
	}
	if mb.State() != Waiting {
		t.Fatalf("B must wait for y, got %v", mb.State())
	}
	// While A holds y, B's z (independent) cannot start either: group acquires
	// atomically and owns no axis yet.
	if c.Owner("z") != nil {
		t.Fatal("B must not own z while waiting on y")
	}
	stepCellUntil(t, c, func() bool { return mb.State() == Active })
	if got := c.Owner("y"); got != mb {
		t.Fatal("y ownership must transfer to B")
	}
	if len(mb.Handoffs()) != 1 {
		t.Fatalf("expected one handoff record, got %d", len(mb.Handoffs()))
	}
	h := mb.Handoffs()[0]
	if h.Axis != "y" || h.FromID != ma.ID || h.ToID != mb.ID || !h.AtRest ||
		math.Abs(h.At-10) > 1e-9 {
		t.Fatalf("bad handoff record: %+v", h)
	}
	stepCellUntil(t, c, func() bool {
		return mb.State() == Completed
	})
	if math.Abs(c.Axis("y").Position()-20) > 1e-9 ||
		math.Abs(c.Axis("z").Position()-10) > 1e-9 {
		t.Fatal("B endpoints wrong")
	}
}

// Groups on fully independent axes keep moving while another group waits.
func TestCellIndependentAxesProgressWhileNeighborWaits(t *testing.T) {
	c := testCell(t)
	ga, _ := c.NewGroup("A", []string{"x"}, ReactDecelerateAndWait)
	gb, _ := c.NewGroup("B", []string{"y"}, ReactDecelerateAndWait)
	ma, _ := ga.EnqueueWithHandoff([]float64{10}, nil, &HandoffConditions{
		Gates: []string{"go"},
	}, nil)
	mb, _ := gb.Enqueue([]float64{30})
	if ma.State() != Waiting {
		t.Fatalf("A waits on gate, got %v", ma.State())
	}
	stepCellUntil(t, c, func() bool { return mb.State() == Completed })
	if math.Abs(c.Axis("y").Position()-30) > 1e-9 {
		t.Fatal("independent axis must progress")
	}
	if ma.State() != Waiting {
		t.Fatal("A must still wait")
	}
	c.SetGate("go", true)
	stepCellUntil(t, c, func() bool { return ma.State() == Completed })
}

// A waiting move can be canceled; it never moved and no displacement is lost.
func TestCellCancelWaitingMove(t *testing.T) {
	c := testCell(t)
	ga, _ := c.NewGroup("A", []string{"x"}, ReactDecelerateAndWait)
	gb, _ := c.NewGroup("B", []string{"x"}, ReactDecelerateAndWait)
	_, _ = ga.Enqueue([]float64{10})
	mb, _ := gb.Enqueue([]float64{50})
	if !gb.CancelMove(mb) {
		t.Fatal("waiting move must cancel")
	}
	if mb.State() != Canceled {
		t.Fatalf("got %v", mb.State())
	}
	stepCellUntil(t, c, func() bool {
		return math.Abs(c.Axis("x").Position()-10) < 1e-9
	})
}

// After the handoff the operator can retarget the successor; the executed
// displacement of the old move stays at its real endpoint.
func TestCellRetargetAfterHandoff(t *testing.T) {
	c := testCell(t)
	ga, _ := c.NewGroup("A", []string{"x"}, ReactDecelerateAndWait)
	gb, _ := c.NewGroup("B", []string{"x"}, ReactDecelerateAndWait)
	_, _ = ga.Enqueue([]float64{10})
	mb, _ := gb.Enqueue([]float64{100})
	stepCellUntil(t, c, func() bool { return mb.State() == Active })
	for i := 0; i < 300; i++ {
		c.Step()
	}
	posBefore := c.Axis("x").Position()
	if posBefore <= 10 {
		t.Fatal("B should already have moved past A's endpoint")
	}
	if _, err := gb.RetargetActive([]float64{40}); err != nil {
		t.Fatal(err)
	}
	stepCellUntil(t, c, func() bool { return mb.State() == Completed })
	if math.Abs(c.Axis("x").Position()-40) > 1e-9 {
		t.Fatal("retargeted endpoint wrong")
	}
}

// Soft stop interleaved with an emergency stop on another axis: each keeps
// only its own axes, and neither auto-releases a moving axis.
func TestCellSoftAndEmergencyStopInterleaved(t *testing.T) {
	c := testCell(t)
	ga, _ := c.NewGroup("A", []string{"x", "y"}, ReactDecelerateAndWait)
	gb, _ := c.NewGroup("B", []string{"z"}, ReactDecelerateAndWait)
	ma, _ := ga.Enqueue([]float64{100, 100})
	mb, _ := gb.Enqueue([]float64{100})
	for i := 0; i < 300; i++ {
		c.Step()
	}
	ga.SoftStop()
	if ma.State() != Stopping {
		t.Fatalf("A must be Stopping, got %v", ma.State())
	}
	if c.Owner("x") != ma || c.Owner("y") != ma {
		t.Fatal("soft stop must retain ownership while braking")
	}
	// B continues independently for a few samples, then gets e-stopped.
	zVel := c.Axis("z").Velocity()
	if zVel == 0 {
		t.Fatal("independent group should keep moving")
	}
	c.EmergencyAxisStop("z")
	if mb.State() != Stopping || mb.AbortReason() != AbortedEmergencyStop {
		t.Fatalf("B stopping wrong: %v/%v", mb.State(), mb.AbortReason())
	}
	// A's soft stop finishes first: its axes release; B's e-stop stays latched.
	stepCellUntil(t, c, func() bool { return ma.State() == Aborted })
	if c.Owner("x") != nil || c.Owner("y") != nil {
		t.Fatal("soft-stopped axes must release after settling")
	}
	stepCellUntil(t, c, func() bool { return c.Axis("z").Velocity() == 0 })
	if c.Owner("z") != mb {
		t.Fatal("emergency axis must not auto-release while latched")
	}
	moved := c.Axis("z").Position()
	if moved <= 0 {
		t.Fatal("executed displacement must remain")
	}
	if err := gb.Reset(); err != nil {
		t.Fatal(err)
	}
	stepCellUntil(t, c, func() bool { return mb.State() == Aborted })
	if c.Owner("z") != nil {
		t.Fatal("reset releases the settled axis")
	}
	if math.Abs(c.Axis("z").Position()-moved) > 1e-12 {
		t.Fatal("reset must not undo displacement")
	}
}

// Late completion events stay attributed to the old move.
func TestCellLateEventsBelongToOldMove(t *testing.T) {
	c := testCell(t)
	ga, _ := c.NewGroup("A", []string{"x"}, ReactDecelerateAndWait)
	gb, _ := c.NewGroup("B", []string{"x"}, ReactDecelerateAndWait)
	ma, _ := ga.Enqueue([]float64{10})
	mb, _ := gb.Enqueue([]float64{20})
	stepCellUntil(t, c, func() bool { return mb.State() == Completed })
	ev := c.Events()
	var aDone, bDone int
	for _, e := range ev {
		switch e.MoveID {
		case ma.ID:
			aDone++
			if e.State != Completed {
				t.Fatalf("A event wrong: %+v", e)
			}
		case mb.ID:
			bDone++
			if e.State != Completed {
				t.Fatalf("B event wrong: %+v", e)
			}
		}
	}
	if aDone != 1 || bDone != 1 {
		t.Fatalf("each move must log exactly one terminal event: A=%d B=%d", aDone, bDone)
	}
}

// Save/restore keeps group order, waiting conditions and real endpoints.
func TestCellSaveRestoreOrderAndEndpoints(t *testing.T) {
	c := testCell(t)
	ga, _ := c.NewGroup("A", []string{"x"}, ReactDecelerateAndWait)
	gb, _ := c.NewGroup("B", []string{"x"}, ReactDecelerateAndWait)
	_, _ = ga.Enqueue([]float64{10})
	mb, _ := gb.Enqueue([]float64{20})
	for i := 0; i < 400; i++ {
		c.Step()
	}
	if mb.State() != Waiting {
		t.Fatalf("expected B waiting, got %v", mb.State())
	}
	data, err := c.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreCell(data)
	if err != nil {
		t.Fatal(err)
	}
	if len(r.gorder) != 2 || r.gorder[0] != "A" || r.gorder[1] != "B" {
		t.Fatalf("group order not preserved: %v", r.gorder)
	}
	rmb := r.Group("B").Pending()[0]
	if rmb.State() != Waiting {
		t.Fatalf("waiting state lost: %v", rmb.State())
	}
	stepCellUntil(t, r, func() bool { return rmb.State() == Completed })
	if math.Abs(r.Axis("x").Position()-20) > 1e-9 {
		t.Fatalf("endpoint after restore wrong: %v", r.Axis("x").Position())
	}
}
