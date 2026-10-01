package action

import (
	"math"
	"testing"

	"motion/axis"
)

func TestSoftStopKeepGoalResumes(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(100)
	m2 := e.Enqueue(150)
	for i := 0; i < 400; i++ {
		e.Step()
	}
	e.SoftStopWithPolicy(StopKeepGoal)
	if m1.State() != Active || m2.State() != Pending {
		t.Fatalf("keep-goal stop must keep work: %v %v", m1.State(), m2.State())
	}
	// The axis brakes to rest, then the SAME goal resumes from the stopped
	// position and the queue continues afterwards.
	stepUntil(t, e, func() bool { return m1.State() == Completed })
	if math.Abs(e.Axis().Position()-100) > 1e-9 || e.Axis().Velocity() != 0 {
		t.Fatalf("resumed goal wrong: p=%v v=%v", e.Axis().Position(), e.Axis().Velocity())
	}
	stepUntil(t, e, func() bool { return m2.State() == Completed })
	if math.Abs(e.Axis().Position()-150) > 1e-9 {
		t.Fatalf("queued move after resume wrong: p=%v", e.Axis().Position())
	}
}

func TestSaveRestoreDuringKeepGoalStop(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(100)
	m2 := e.Enqueue(150)
	for i := 0; i < 400; i++ {
		e.Step()
	}
	e.SoftStopWithPolicy(StopKeepGoal)
	for i := 0; i < 10; i++ {
		e.Step() // still braking
	}
	if e.Axis().Mode() != axis.SoftStopping {
		t.Fatalf("expected mid-stop, mode=%v", e.Axis().Mode())
	}
	data, err := e.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreExecutor(data)
	if err != nil {
		t.Fatal(err)
	}
	// Same rules after restore: brake to rest, resume the held goal, run queue.
	act := r.Active()
	if act == nil || act.ID != m1.ID {
		t.Fatalf("held active move not restored: %+v", act)
	}
	stepUntil(t, r, func() bool { return act.State() == Completed })
	if math.Abs(r.Axis().Position()-100) > 1e-9 {
		t.Fatalf("restored keep-goal stop did not reach goal: p=%v", r.Axis().Position())
	}
	next := r.Active()
	if next == nil || next.ID != m2.ID {
		t.Fatalf("queued move not dispatched after resume: %+v", next)
	}
	stepUntil(t, r, func() bool { return next.State() == Completed })
	if math.Abs(r.Axis().Position()-150) > 1e-9 || r.Axis().Velocity() != 0 {
		t.Fatalf("restored queue result wrong: p=%v v=%v", r.Axis().Position(), r.Axis().Velocity())
	}
}

func TestSaveRestoreDuringEndQueueStop(t *testing.T) {
	e := testExecutor(t)
	m1 := e.Enqueue(100)
	for i := 0; i < 400; i++ {
		e.Step()
	}
	e.SoftStop() // StopEndQueue
	for i := 0; i < 5; i++ {
		e.Step()
	}
	data, err := e.Save()
	if err != nil {
		t.Fatal(err)
	}
	r, err := RestoreExecutor(data)
	if err != nil {
		t.Fatal(err)
	}
	if m1.State() != Aborted || r.Active() != nil {
		t.Fatal("end-queue stop must stay aborted after restore")
	}
	stepUntil(t, r, func() bool { return r.Axis().Velocity() == 0 })
	if r.Axis().Mode() != axis.Idle {
		t.Fatalf("restored axis should settle idle, mode=%v", r.Axis().Mode())
	}
}

func TestShortDistanceMoveExact(t *testing.T) {
	e := testExecutor(t)
	m := e.Enqueue(0.02) // shorter than one cruise sample
	stepUntil(t, e, func() bool { return m.State() == Completed })
	if math.Abs(e.Axis().Position()-0.02) > 1e-12 || e.Axis().Velocity() != 0 {
		t.Fatalf("short move wrong: p=%v v=%v", e.Axis().Position(), e.Axis().Velocity())
	}
	if e.Active() != nil || len(e.Pending()) != 0 {
		t.Fatal("queue should be drained after short move")
	}
}
