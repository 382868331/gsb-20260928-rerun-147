package axis

import (
	"encoding/json"
	"math"
	"testing"
)

const (
	testV   = 10.0
	testA   = 20.0
	testDt  = 0.001
	testEps = 1e-9
)

type sample struct{ p, v float64 }

func runToSettle(t *testing.T, ax *Axis, maxSteps int) []sample {
	t.Helper()
	out := []sample{{ax.Position(), ax.Velocity()}}
	for i := 0; i < maxSteps; i++ {
		ax.Step()
		cur := sample{ax.Position(), ax.Velocity()}
		prev := out[len(out)-1]
		if math.Abs(cur.v) > testV+testEps {
			t.Fatalf("speed exceeded: |v|=%v at step %d", cur.v, i)
		}
		dv := cur.v - prev.v
		if math.Abs(dv) > testA*testDt+1e-9 {
			t.Fatalf("acceleration exceeded: |dv|=%v > %v at step %d", math.Abs(dv), testA*testDt, i)
		}
		// Trapezoidal integration: displacement is 0.5*(v_prev+v_cur)*dt.
		dp := cur.p - prev.p
		maxStep := testV*testDt + 0.5*testA*testDt*testDt
		if math.Abs(dp) > maxStep+1e-9 {
			t.Fatalf("motion per step exceeded: |dp|=%v at step %d", math.Abs(dp), i)
		}
		out = append(out, cur)
		if cur.v == 0 && ax.Mode() != Moving {
			break
		}
		if ax.Mode() == Moving && cur.v == 0 {
			break
		}
	}
	return out
}

func settle(t *testing.T, ax *Axis) []sample {
	t.Helper()
	return runToSettle(t, ax, 1_000_000)
}

func TestLongMoveTrapezoid(t *testing.T) {
	ax, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	const target = 50.0
	if err := ax.SetTarget(target); err != nil {
		t.Fatal(err)
	}
	traj := settle(t, ax)
	last := traj[len(traj)-1]
	if math.Abs(last.p-target) > testEps || last.v != 0 {
		t.Fatalf("did not settle at target: p=%v v=%v", last.p, last.v)
	}
	// Cruise phase must actually saturate the speed envelope.
	maxV := 0.0
	for _, s := range traj {
		if s.v > maxV {
			maxV = s.v
		}
	}
	if math.Abs(maxV-testV) > 1e-6 {
		t.Fatalf("never reached cruise speed, maxV=%v", maxV)
	}
	// Monotonic approach and no overshoot.
	for _, s := range traj {
		if s.p < -testEps || s.p > target+testEps {
			t.Fatalf("left envelope [0,%v]: p=%v", target, s.p)
		}
	}
}

func TestShortMoveNoOvershoot(t *testing.T) {
	ax, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	// Distance shorter than the distance needed to reach cruise speed.
	const target = 0.05
	if err := ax.SetTarget(target); err != nil {
		t.Fatal(err)
	}
	traj := settle(t, ax)
	last := traj[len(traj)-1]
	if math.Abs(last.p-target) > testEps || last.v != 0 {
		t.Fatalf("short move did not settle: p=%v v=%v", last.p, last.v)
	}
	for _, s := range traj {
		if s.p < -testEps || s.p > target+testEps {
			t.Fatalf("short move overshot: p=%v", s.p)
		}
	}
}

func TestReverseTargetWholeTrajectory(t *testing.T) {
	ax, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	// First get the axis moving in the positive direction at speed.
	if err := ax.SetTarget(100); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 300; i++ {
		ax.Step()
	}
	movingPos, movingVel := ax.Position(), ax.Velocity()
	if movingVel <= 0 {
		t.Fatalf("expected positive velocity before reverse, got %v", movingVel)
	}

	// Command a target behind the current position. Position and velocity must
	// be continuous: decelerate through zero and accelerate the other way.
	const target = -20.0
	if err := ax.SetTarget(target); err != nil {
		t.Fatal(err)
	}
	prevP, prevV := movingPos, movingVel
	sawZero := false
	for i := 0; i < 1_000_000; i++ {
		ax.Step()
		p, v := ax.Position(), ax.Velocity()
		maxStep := testV*testDt + 0.5*testA*testDt*testDt
		if math.Abs(p-prevP) > maxStep+1e-9 {
			t.Fatalf("position discontinuity after reverse: %v at %d", p-prevP, i)
		}
		if math.Abs(v-prevV) > testA*testDt+1e-9 {
			t.Fatalf("velocity discontinuity after reverse: %v at %d", v-prevV, i)
		}
		if math.Abs(v) > testV+testEps {
			t.Fatalf("speed envelope violated after reverse: %v", v)
		}
		if !sawZero && v < 0 {
			sawZero = true
		}
		prevP, prevV = p, v
		if v == 0 && ax.AtTarget() {
			break
		}
	}
	if !sawZero {
		t.Fatal("velocity never reversed sign")
	}
	if math.Abs(ax.Position()-target) > testEps || ax.Velocity() != 0 {
		t.Fatalf("reverse move did not settle at %v: p=%v v=%v", target, ax.Position(), ax.Velocity())
	}
}

func TestRetargetNoJump(t *testing.T) {
	ax, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	if err := ax.SetTarget(100); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 400; i++ {
		ax.Step()
	}
	p0, v0 := ax.Position(), ax.Velocity()
	if err := ax.SetTarget(p0 + 0.1); err != nil {
		t.Fatal(err)
	}
	ax.Step()
	// First step after retarget must start from (p0,v0): no teleport, and with
	// a target only 0.1 away the axis must brake rather than keep accelerating.
	if ax.Velocity() > v0+1e-12 {
		t.Fatalf("velocity jumped up after close retarget: %v -> %v", v0, ax.Velocity())
	}
	traj := settle(t, ax)
	if math.Abs(traj[len(traj)-1].p-(p0+0.1)) > testEps {
		t.Fatalf("did not settle at new target")
	}
}

func TestEmergencyStop(t *testing.T) {
	ax, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	if err := ax.SetTarget(100); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 600; i++ {
		ax.Step()
	}
	v0 := ax.Velocity()
	ax.EmergencyStop()
	if err := ax.SetTarget(0); err == nil {
		t.Fatal("SetTarget must be rejected during latched e-stop")
	}
	// Braking distance for max decel must be bounded by v^2/(2a) plus one step.
	p0 := ax.Position()
	for {
		ax.Step()
		if ax.Velocity() == 0 {
			break
		}
	}
	distance := ax.Position() - p0
	want := v0*v0/(2*testA) + testV*testDt
	if math.Abs(distance) > want+1e-6 {
		t.Fatalf("braked too far: %v > %v", distance, want)
	}
	if ax.Mode() != EmergencyStopping {
		t.Fatalf("mode=%v, want EmergencyStopping", ax.Mode())
	}
	if err := ax.Reset(); err != nil {
		t.Fatal(err)
	}
	if ax.Mode() != Idle || ax.EmergencyStopped() {
		t.Fatal("reset did not clear e-stop")
	}
}

func TestSoftStopResumes(t *testing.T) {
	ax, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	if err := ax.SetTarget(100); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 600; i++ {
		ax.Step()
	}
	ax.SoftStop()
	for ax.Velocity() != 0 {
		ax.Step()
	}
	if ax.Mode() != Idle {
		t.Fatalf("mode after soft stop = %v", ax.Mode())
	}
	if err := ax.SetTarget(-5); err != nil {
		t.Fatalf("new target rejected after soft stop: %v", err)
	}
	settle(t, ax)
	if math.Abs(ax.Position()+5) > testEps || ax.Velocity() != 0 {
		t.Fatalf("did not resume after soft stop: p=%v v=%v", ax.Position(), ax.Velocity())
	}
}

func TestSnapshotRoundTripMidMotion(t *testing.T) {
	ax, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	if err := ax.SetTarget(100); err != nil {
		t.Fatal(err)
	}
	for i := 0; i < 350; i++ {
		ax.Step()
	}
	data, err := json.Marshal(ax.Snapshot())
	if err != nil {
		t.Fatal(err)
	}
	var s Snapshot
	if err := json.Unmarshal(data, &s); err != nil {
		t.Fatal(err)
	}
	restored, _ := New(Config{MaxSpeed: testV, MaxAccel: testA, Step: testDt})
	restored.Restore(s)
	if restored.Position() != ax.Position() || restored.Velocity() != ax.Velocity() {
		t.Fatalf("restore lost mid-motion state: %v,%v vs %v,%v",
			restored.Position(), restored.Velocity(), ax.Position(), ax.Velocity())
	}
	if restored.Mode() != Moving || restored.Velocity() == 0 {
		t.Fatal("restore must not pretend the axis arrived")
	}
	// Continue and finish from the restored state.
	settle(t, restored)
	if math.Abs(restored.Position()-100) > testEps || restored.Velocity() != 0 {
		t.Fatal("restored axis did not finish the move")
	}
}
