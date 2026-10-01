package action

import (
	"math"
	"testing"

	"motion/axis"
)

// SoftStopKeepTarget brakes to rest, keeps the active target and the queue,
// then resumes automatically.
func TestSoftStopKeepTargetResumes(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(10)
	m2 := e.Enqueue(20)
	for i := 0; i < 300; i++ {
		e.Step()
	}
	e.SoftStopKeepTarget()
	if m1.State() != Active || m2.State() != Pending {
		t.Fatalf("keep-target stop must not end the queue: %v %v", m1.State(), m2.State())
	}
	// The axis brakes to rest first.
	stepUntil(t, e, func() bool { return e.Axis().Velocity() == 0 })
	if m1.State() != Active {
		t.Fatalf("m1=%v, want Active while paused", m1.State())
	}
	// Then motion resumes toward the kept target and the queue continues.
	stepUntil(t, e, func() bool { return m1.State() == Completed })
	if p := e.Axis().Position(); math.Abs(p-10) > 1e-9 {
		t.Fatalf("pos=%v, want 10", p)
	}
	stepUntil(t, e, func() bool { return m2.State() == Completed })
	if p := e.Axis().Position(); math.Abs(p-20) > 1e-9 {
		t.Fatalf("pos=%v, want 20", p)
	}
}

// Saving while soft-stopping restores the braking state and the stop
// finishes under the same rules.
func TestSaveRestoreDuringSoftStop(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(100)
	e.Enqueue(200)
	for i := 0; i < 300; i++ {
		e.Step()
	}
	e.SoftStop()
	if e.Axis().Mode() != axis.SoftStopping {
		t.Fatalf("mode=%v, want SoftStopping", e.Axis().Mode())
	}
	data, err := e.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreExecutor(data)
	if err != nil {
		t.Fatal(err)
	}
	if r.Axis().Mode() != axis.SoftStopping {
		t.Fatalf("restored mode=%v, want SoftStopping", r.Axis().Mode())
	}
	if m1.State() != Aborted || m1.AbortReason() != AbortedSoftStop {
		t.Fatalf("active=%v reason=%v", m1.State(), m1.AbortReason())
	}
	// The restored axis finishes braking to rest; nothing is dispatched
	// automatically because the queue was ended by the soft stop.
	stepUntil(t, r, func() bool { return r.Axis().Velocity() == 0 })
	if r.Axis().Mode() != axis.Idle {
		t.Fatalf("mode=%v, want Idle after braking", r.Axis().Mode())
	}
	if r.Active() != nil || len(r.Pending()) != 0 {
		t.Fatal("restored executor should have no active or pending moves")
	}
	// New work can be commanded from the stopped position.
	stoppedAt := r.Axis().Position()
	m3 := r.Enqueue(stoppedAt + 1)
	stepUntil(t, r, func() bool { return m3.State() == Completed })
	if p := r.Axis().Position(); math.Abs(p-(stoppedAt+1)) > 1e-9 {
		t.Fatalf("pos=%v, want %v", p, stoppedAt+1)
	}
}

// A target shorter than one sample of travel lands exactly with no motion.
func TestShortDistanceMove(t *testing.T) {
	e := testExecutor(t)
	m := e.Enqueue(1e-4)
	stepUntil(t, e, func() bool { return m.State() == Completed })
	if p := e.Axis().Position(); p != 1e-4 {
		t.Fatalf("pos=%v, want exact 1e-4", p)
	}
	if v := e.Axis().Velocity(); v != 0 {
		t.Fatalf("vel=%v, want 0", v)
	}
}
