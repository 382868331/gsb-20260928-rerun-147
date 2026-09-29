package motion

import (
	"math"
	"testing"
)

func assertPlanConstraints(t *testing.T, plan *Plan, target, maxVelocity, maxAcceleration float64) {
	t.Helper()
	steps := int(math.Ceil(plan.Duration()/0.001)) + 1
	var previous point
	for i := 0; i <= steps; i++ {
		tm := plan.Duration() * float64(i) / float64(steps)
		x, v, a, done := plan.At(tm)
		if math.Abs(v) > maxVelocity+1e-9 {
			t.Fatalf("velocity %.9f exceeds %.9f at t=%.6f", v, maxVelocity, tm)
		}
		if math.Abs(a) > maxAcceleration+1e-9 {
			t.Fatalf("acceleration %.9f exceeds %.9f at t=%.6f", a, maxAcceleration, tm)
		}
		if i == steps && (!done || math.Abs(x-target) > 1e-9 || math.Abs(v) > 1e-9) {
			t.Fatalf("end state = (%v,%v,%v), want (%v,0,true)", x, v, done, target)
		}
		if i > 0 {
			dt := tm - previous.t
			if math.Abs(x-previous.x) > maxVelocity*dt+1e-7 {
				t.Fatalf("position jump near t=%.6f", tm)
			}
			if math.Abs(v-previous.v) > maxAcceleration*dt+1e-7 {
				t.Fatalf("velocity jump near t=%.6f", tm)
			}
		}
		previous = point{t: tm, x: x, v: v, a: a}
	}

	offset := 0.0
	for index, current := range plan.segments {
		for i := 0; i < 8; i++ {
			local := current.duration * float64(i) / 8
			value := current.at(local)
			x, v, a, _ := plan.At(offset + local)
			wantA := value.a
			if math.Abs(x-value.x) > 1e-9 || math.Abs(v-value.v) > 1e-9 || math.Abs(a-wantA) > 1e-9 {
				t.Fatalf("segment formula mismatch at t=%.6f", offset+local)
			}
		}
		end := current.at(current.duration)
		if index+1 < len(plan.segments) {
			next := plan.segments[index+1]
			if math.Abs(end.x-next.start.x) > 1e-9 || math.Abs(end.v-next.start.v) > 1e-9 {
				t.Fatalf("discontinuous segment boundary at t=%.6f", offset)
			}
		}
		offset += current.duration
	}
}

func TestShortDistanceTriangularProfile(t *testing.T) {
	plan := NewMovePlan(0, 0, 0.04, 2, 4)
	assertPlanConstraints(t, plan, 0.04, 2, 4)
	if plan.Duration() >= 0.201 {
		t.Fatalf("short move did not use triangular profile, duration=%v", plan.Duration())
	}
}

func TestReverseTargetStartsFromCurrentVelocity(t *testing.T) {
	plan := NewMovePlan(1, 1, -1, 2, 4)
	x, v, a, done := plan.At(0)
	if done || x != 1 || v != 1 || a != -4 {
		t.Fatalf("profile did not start from actual state: x=%v v=%v a=%v done=%v", x, v, a, done)
	}
	assertPlanConstraints(t, plan, -1, 2, 4)
}

func TestRetargetUsesCurrentMeasuredState(t *testing.T) {
	first := NewMovePlan(0, 0, 10, 2, 4)
	x, v, _, _ := first.At(0.123)
	second := NewMovePlan(x, v, -3, 2, 4)
	nextX, nextV, _, done := second.At(0)
	if done || nextX != x || nextV != v {
		t.Fatalf("retarget jumped: (%v,%v) from (%v,%v)", nextX, nextV, x, v)
	}
	assertPlanConstraints(t, second, -3, 2, 4)
}
