package motion

import (
	"encoding/json"
	"math"
	"testing"
)

func runUntilComplete(t *testing.T, c *Controller, maxDuration float64) {
	t.Helper()
	elapsed := 0.0
	for elapsed < maxDuration && c.Mode() != ModeIdle {
		c.Tick(0.001)
		elapsed += 0.001
	}
	if c.Mode() != ModeIdle || math.Abs(c.Velocity()) > 1e-9 {
		t.Fatalf("motion did not finish: mode=%s position=%v velocity=%v", c.Mode(), c.Position(), c.Velocity())
	}
}

func TestQueueExecutesMovesContinuously(t *testing.T) {
	c, _ := NewController(Limits{MaxVelocity: 2, MaxAcceleration: 4}, 0, 0)
	first, err := c.EnqueueMove(1)
	if err != nil {
		t.Fatal(err)
	}
	second, err := c.EnqueueMove(-0.2)
	if err != nil {
		t.Fatal(err)
	}
	runUntilComplete(t, c, 3)
	if first.Status() != StatusCompleted || second.Status() != StatusCompleted {
		t.Fatalf("statuses = %s,%s", first.Status(), second.Status())
	}
	if math.Abs(c.Position()-(-0.2)) > 1e-9 {
		t.Fatalf("final position = %v", c.Position())
	}
}

func TestSetTargetContinuesFromCurrentState(t *testing.T) {
	c, _ := NewController(Limits{MaxVelocity: 2, MaxAcceleration: 4}, 0, 0)
	first, _ := c.EnqueueMove(10)
	pending, _ := c.EnqueueMove(20)
	c.Tick(0.2)
	x, v := c.Position(), c.Velocity()
	second, err := c.SetTarget(-2)
	if err != nil {
		t.Fatal(err)
	}
	if first.Status() != StatusInterrupted || pending.Status() != StatusCanceled || second.Status() != StatusRunning {
		t.Fatalf("statuses = %s,%s,%s", first.Status(), pending.Status(), second.Status())
	}
	if c.Position() != x || c.Velocity() != v {
		t.Fatal("retarget changed state immediately")
	}
	runUntilComplete(t, c, 4)
	if math.Abs(c.Position()-(-2)) > 1e-9 {
		t.Fatalf("position = %v", c.Position())
	}
}

func TestCancelQueuedLeavesActiveRunning(t *testing.T) {
	c, _ := NewController(Limits{MaxVelocity: 2, MaxAcceleration: 4}, 0, 0)
	active, _ := c.EnqueueMove(5)
	pending, _ := c.EnqueueMove(6)
	if canceled := c.CancelQueued(); canceled != 1 {
		t.Fatalf("canceled %d queued actions", canceled)
	}
	if active.Status() != StatusRunning || pending.Status() != StatusCanceled {
		t.Fatalf("statuses = %s,%s", active.Status(), pending.Status())
	}
	runUntilComplete(t, c, 4)
	if active.Status() != StatusCompleted {
		t.Fatalf("active status = %s", active.Status())
	}
}

func TestSoftStopBrakesAndPreservesQueue(t *testing.T) {
	c, _ := NewController(Limits{MaxVelocity: 2, MaxAcceleration: 4}, 0, 0)
	active, _ := c.EnqueueMove(5)
	pending, _ := c.EnqueueMove(-1)
	c.Tick(0.2)
	startVelocity := c.Velocity()
	c.SoftStop()
	moving := c.Mode() == ModeSoftStopping
	for i := 0; i < 300 && c.Mode() == ModeSoftStopping; i++ {
		c.Tick(0.001)
		if math.Abs(c.Velocity()) > math.Abs(startVelocity)+1e-9 {
			t.Fatal("speed increased during soft stop")
		}
	}
	if !moving || c.Mode() != ModeSoftStopped || active.Status() != StatusStopped {
		t.Fatalf("soft stop state = %s,%s", c.Mode(), active.Status())
	}
	if err := c.Resume(); err != nil {
		t.Fatal(err)
	}
	runUntilComplete(t, c, 4)
	if pending.Status() != StatusCompleted || math.Abs(c.Position()+1) > 1e-9 {
		t.Fatalf("pending action after resume = %s position=%v", pending.Status(), c.Position())
	}
}

func TestEmergencyStopZeroesVelocityAndCancelsQueue(t *testing.T) {
	c, _ := NewController(Limits{MaxVelocity: 2, MaxAcceleration: 4}, 0, 0)
	active, _ := c.EnqueueMove(5)
	pending, _ := c.EnqueueMove(6)
	c.Tick(0.2)
	c.EmergencyStop()
	if c.Velocity() != 0 || c.Mode() != ModeEmergencyStop || active.Status() != StatusStopped || pending.Status() != StatusCanceled {
		t.Fatalf("unexpected estop state: mode=%s velocity=%v statuses=%s,%s", c.Mode(), c.Velocity(), active.Status(), pending.Status())
	}
	if _, err := c.EnqueueMove(1); err == nil {
		t.Fatal("enqueue succeeded after emergency stop")
	}
}

func TestSnapshotRestoreDoesNotCompleteInFlightAction(t *testing.T) {
	c, _ := NewController(Limits{MaxVelocity: 2, MaxAcceleration: 4}, 0, 0)
	action, _ := c.EnqueueMove(4)
	c.Tick(0.2)
	data, err := json.Marshal(c)
	if err != nil {
		t.Fatal(err)
	}
	var state State
	if err := json.Unmarshal(data, &state); err != nil {
		t.Fatal(err)
	}
	restored, err := RestoreController(state)
	if err != nil {
		t.Fatal(err)
	}
	if restored.Mode() != ModeMoving || action.Status() != StatusRunning {
		t.Fatalf("snapshot falsely completed motion: mode=%s status=%s", restored.Mode(), action.Status())
	}
	if restored.Position() != c.Position() || restored.Velocity() != c.Velocity() {
		t.Fatal("restored state differs from snapshot")
	}
	runUntilComplete(t, restored, 4)
	if math.Abs(restored.Position()-4) > 1e-9 {
		t.Fatalf("restored motion ended at %v", restored.Position())
	}
}
