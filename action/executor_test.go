package action

import (
	"math"
	"testing"
	"time"

	"motion/axis"
)

func testExecutor(t *testing.T) *Executor {
	t.Helper()
	e, err := NewExecutor(axis.Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001})
	if err != nil {
		t.Fatal(err)
	}
	clk := time.Unix(1_700_000_000, 0)
	e.now = func() time.Time {
		clk = clk.Add(time.Millisecond)
		return clk
	}
	return e
}

func stepUntil(t *testing.T, e *Executor, cond func() bool) {
	t.Helper()
	for i := 0; i < 5_000_000; i++ {
		e.Step()
		if cond() {
			return
		}
	}
	t.Fatal("condition never met")
}

func TestQueueRunsMovesInOrder(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(10)
	m2 := e.Enqueue(-5)
	m3 := e.Enqueue(2)

	if m1.State() != Active || m2.State() != Pending || m3.State() != Pending {
		t.Fatalf("initial scheduling wrong: %v %v %v", m1.State(), m2.State(), m3.State())
	}
	stepUntil(t, e, func() bool { return m1.State() == Completed })
	if math.Abs(e.Axis().Position()-10) > 1e-9 || m2.State() != Active {
		t.Fatalf("m1 completion wrong: p=%v m2=%v", e.Axis().Position(), m2.State())
	}
	stepUntil(t, e, func() bool { return m2.State() == Completed })
	if math.Abs(e.Axis().Position()+5) > 1e-9 {
		t.Fatalf("m2 target wrong: %v", e.Axis().Position())
	}
	stepUntil(t, e, func() bool { return m3.State() == Completed })
	if math.Abs(e.Axis().Position()-2) > 1e-9 || e.Active() != nil {
		t.Fatalf("final state wrong: p=%v active=%v", e.Axis().Position(), e.Active())
	}
}

func TestCancelQueuedKeepsActive(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(10)
	m2 := e.Enqueue(20)
	m3 := e.Enqueue(30)
	canceled := e.CancelQueued()
	if len(canceled) != 2 || m2.State() != Canceled || m3.State() != Canceled {
		t.Fatalf("cancel result wrong: %d %v %v", len(canceled), m2.State(), m3.State())
	}
	if m1.State() != Active {
		t.Fatal("active motion must not be canceled")
	}
	stepUntil(t, e, func() bool { return m1.State() == Completed })
	if e.Active() != nil || len(e.Pending()) != 0 {
		t.Fatal("queue should be drained")
	}
}

func TestRetargetContinuesFromCurrentState(t *testing.T) {
	e := testExecutor(t)
	m := e.Enqueue(100)
	for i := 0; i < 400; i++ {
		e.Step()
	}
	p0, v0 := e.Axis().Position(), e.Axis().Velocity()
	m2, err := e.RetargetActive(p0 + 0.2)
	if err != nil || m != m2 {
		t.Fatalf("retarget identity wrong: %v %p %p", err, m, m2)
	}
	e.Step()
	if e.Axis().Velocity() > v0+1e-12 {
		t.Fatal("retarget caused velocity jump")
	}
	stepUntil(t, e, func() bool { return m.State() == Completed })
	if math.Abs(e.Axis().Position()-(p0+0.2)) > 1e-9 {
		t.Fatalf("retarget target wrong: %v", e.Axis().Position())
	}
}

func TestSoftStopAbortsAndClearsQueue(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(100)
	m2 := e.Enqueue(200)
	for i := 0; i < 400; i++ {
		e.Step()
	}
	e.SoftStop()
	if m1.State() != Aborted || m1.AbortReason() != AbortedSoftStop {
		t.Fatalf("m1 should be aborted: %v/%v", m1.State(), m1.AbortReason())
	}
	if m2.State() != Canceled {
		t.Fatalf("queued move should be canceled: %v", m2.State())
	}
	stepUntil(t, e, func() bool { return e.Axis().Velocity() == 0 })
	if e.Active() != nil || e.Axis().Mode() != axis.Idle {
		t.Fatal("executor should be idle after soft stop")
	}
	// A fresh move starts from the stopped real position.
	stoppedAt := e.Axis().Position()
	m3 := e.Enqueue(stoppedAt + 1)
	if m3.State() != Active {
		t.Fatalf("new move not active after soft stop: %v", m3.State())
	}
	if sp, ok := m3.StartPos(); !ok || math.Abs(sp-stoppedAt) > 1e-12 {
		t.Fatalf("new move start pos wrong: %v ok=%v", sp, ok)
	}
}

func TestEmergencyStopLatches(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(100)
	m2 := e.Enqueue(200)
	for i := 0; i < 400; i++ {
		e.Step()
	}
	e.EmergencyStop()
	if m1.State() != Aborted || m1.AbortReason() != AbortedEmergencyStop {
		t.Fatalf("m1 abort wrong: %v/%v", m1.State(), m1.AbortReason())
	}
	if m2.State() != Canceled {
		t.Fatalf("m2 should be canceled: %v", m2.State())
	}
	stepUntil(t, e, func() bool { return e.Axis().Velocity() == 0 })
	if !e.Axis().EmergencyStopped() {
		t.Fatal("e-stop should be latched")
	}
	if _, err := e.RetargetActive(0); err == nil {
		t.Fatal("new target must be rejected while e-stop latched")
	}
	if err := e.Reset(); err != nil {
		t.Fatal(err)
	}
	m3 := e.Enqueue(1)
	if m3.State() != Active {
		t.Fatalf("motion must run after reset: %v", m3.State())
	}
}

func TestSaveRestoreMidMotion(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(100)
	m2 := e.Enqueue(150)
	for i := 0; i < 300; i++ {
		e.Step()
	}
	p0, v0 := e.Axis().Position(), e.Axis().Velocity()
	data, err := e.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreExecutor(data)
	if err != nil {
		t.Fatal(err)
	}
	if math.Abs(r.Axis().Position()-p0) > 1e-12 || math.Abs(r.Axis().Velocity()-v0) > 1e-12 {
		t.Fatalf("restore changed real state: p=%v/%v v=%v/%v", r.Axis().Position(), p0, r.Axis().Velocity(), v0)
	}
	act := r.Active()
	if act == nil || act.State() != Active || act.ID != m1.ID {
		t.Fatalf("active move not restored: %+v", act)
	}
	pending := r.Pending()
	if len(pending) != 1 || pending[0].ID != m2.ID || pending[0].State() != Pending {
		t.Fatalf("queue not restored: %+v", pending)
	}
	if math.Abs(r.Axis().Position()-100) < 1e-6 {
		t.Fatal("restore must not pretend the axis is already at target")
	}
	stepUntil(t, r, func() bool { return act.State() == Completed })
	stepUntil(t, r, func() bool { return pending[0].State() == Completed })
	if math.Abs(r.Axis().Position()-150) > 1e-9 || r.Axis().Velocity() != 0 {
		t.Fatalf("restored executor did not finish: p=%v v=%v", r.Axis().Position(), r.Axis().Velocity())
	}
}

func TestSaveRestoreDuringEmergencyStop(t *testing.T) {
	e := testExecutor(t)
	e.Enqueue(100)
	for i := 0; i < 500; i++ {
		e.Step()
	}
	e.EmergencyStop()
	for i := 0; i < 10; i++ {
		e.Step() // still braking
	}
	data, err := e.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreExecutor(data)
	if err != nil {
		t.Fatal(err)
	}
	if !r.Axis().EmergencyStopped() {
		t.Fatal("restored axis lost e-stop latch")
	}
	if r.Active() != nil {
		t.Fatal("no motion should be active during e-stop")
	}
	if _, err := r.RetargetActive(0); err == nil {
		t.Fatal("target must stay rejected after restoring e-stop")
	}
}
