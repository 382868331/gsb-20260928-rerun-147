package axis

import "testing"

// Pause keeps the goal: the axis brakes to rest and then resumes toward the
// target on its own, unlike SoftStop which drops the goal.
func TestPauseKeepsGoalAndResumes(t *testing.T) {
	ax, err := New(Config{MaxSpeed: 10, MaxAccel: 20, Step: 0.001})
	if err != nil {
		t.Fatal(err)
	}
	if err := ax.SetTarget(5); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 200; i++ {
		ax.Step()
	}
	ax.Pause()
	if _, ok := ax.Target(); !ok {
		t.Fatal("Pause must keep the goal")
	}
	// Brake to rest.
	for ax.Velocity() != 0 {
		ax.Step()
	}
	if ax.AtTarget() {
		t.Fatal("axis paused mid-travel must not be at target yet")
	}
	// Resume automatically and land exactly on the target.
	for i := 0; i < 1_000_000 && !ax.AtTarget(); i++ {
		ax.Step()
	}
	if !ax.AtTarget() {
		t.Fatal("paused axis never resumed to target")
	}
	ax.SettleIfAtTarget()
	if ax.Position() != 5 || ax.Velocity() != 0 {
		t.Fatalf("pos=%v vel=%v, want exactly 5, 0", ax.Position(), ax.Velocity())
	}
}
