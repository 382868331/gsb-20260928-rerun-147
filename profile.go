package motion

import "math"

const epsilon = 1e-12

type point struct {
	t float64
	x float64
	v float64
	a float64
}

// Point is one point on a constant-acceleration trajectory.
type Point struct {
	Time         float64 `json:"time"`
	Position     float64 `json:"position"`
	Velocity     float64 `json:"velocity"`
	Acceleration float64 `json:"acceleration"`
}

// Segment is one constant-acceleration part of a profile.
type Segment struct {
	Duration float64 `json:"duration"`
	Start    Point   `json:"start"`
}

// ProfileState is enough to reconstruct an in-flight profile exactly.
type ProfileState struct {
	Segments []Segment `json:"segments"`
}

type segment struct {
	duration float64
	start    point
}

func (s segment) at(t float64) point {
	if t < 0 {
		t = 0
	}
	if t > s.duration {
		t = s.duration
	}
	dt := t
	return point{
		t: t,
		x: s.start.x + s.start.v*dt + 0.5*s.start.a*dt*dt,
		v: s.start.v + s.start.a*dt,
		a: s.start.a,
	}
}

// Plan is a piecewise constant-acceleration trajectory. All plans start from
// the measured state supplied when they are constructed, never from an old
// command starting point.
type Plan struct {
	segments []segment
	duration float64
}

func newPlan(segments []segment) *Plan {
	total := 0.0
	for _, item := range segments {
		total += item.duration
	}
	return &Plan{segments: segments, duration: total}
}

func (p *Plan) Duration() float64 { return p.duration }

func (p *Plan) State() ProfileState {
	segments := make([]Segment, 0, len(p.segments))
	for _, item := range p.segments {
		segments = append(segments, Segment{
			Duration: item.duration,
			Start: Point{
				Time:         item.start.t,
				Position:     item.start.x,
				Velocity:     item.start.v,
				Acceleration: item.start.a,
			},
		})
	}
	return ProfileState{Segments: segments}
}

func planFromState(state ProfileState) *Plan {
	segments := make([]segment, 0, len(state.Segments))
	for _, item := range state.Segments {
		segments = append(segments, segment{
			duration: item.Duration,
			start: point{
				t: item.Start.Time,
				x: item.Start.Position,
				v: item.Start.Velocity,
				a: item.Start.Acceleration,
			},
		})
	}
	return newPlan(segments)
}

func (p *Plan) At(elapsed float64) (position, velocity, acceleration float64, done bool) {
	if p == nil || len(p.segments) == 0 || elapsed <= 0 {
		if p == nil || len(p.segments) == 0 {
			return 0, 0, 0, true
		}
		first := p.segments[0].start
		return first.x, first.v, first.a, false
	}
	if elapsed >= p.duration {
		last := p.segments[len(p.segments)-1].at(p.segments[len(p.segments)-1].duration)
		return last.x, last.v, 0, true
	}

	offset := 0.0
	for _, current := range p.segments {
		if elapsed < offset+current.duration {
			value := current.at(elapsed - offset)
			return value.x, value.v, value.a, elapsed >= p.duration
		}
		offset += current.duration
	}
	last := p.segments[len(p.segments)-1].at(p.segments[len(p.segments)-1].duration)
	return last.x, last.v, 0, true
}

func brakePlan(startX, startV, acceleration float64) *Plan {
	if math.Abs(startV) <= epsilon {
		return newPlan([]segment{{duration: 0, start: point{x: startX, v: 0}}})
	}
	a := -math.Copysign(acceleration, startV)
	duration := math.Abs(startV) / acceleration
	return newPlan([]segment{{duration: duration, start: point{x: startX, v: startV, a: a}}})
}

func restMovePlan(startX, target, maxVelocity, acceleration float64) *Plan {
	distance := target - startX
	if math.Abs(distance) <= epsilon {
		return newPlan([]segment{{duration: 0, start: point{x: target, v: 0}}})
	}

	direction := math.Copysign(1, distance)
	vmax := math.Min(maxVelocity, math.Sqrt(acceleration*math.Abs(distance)))
	acceleratingDistance := vmax * vmax / (2 * acceleration)
	cruiseDistance := math.Abs(distance) - 2*acceleratingDistance

	segments := make([]segment, 0, 3)
	t := 0.0
	x := startX
	v := 0.0

	accelDuration := vmax / acceleration
	segments = append(segments, segment{duration: accelDuration, start: point{t: t, x: x, v: v, a: direction * acceleration}})
	end := segments[0].at(accelDuration)
	t += accelDuration
	x, v = end.x, end.v

	if cruiseDistance > epsilon {
		cruiseDuration := cruiseDistance / vmax
		segments = append(segments, segment{duration: cruiseDuration, start: point{t: t, x: x, v: v}})
		end = segments[len(segments)-1].at(cruiseDuration)
		t += cruiseDuration
		x, v = end.x, end.v
	}

	decelDuration := vmax / acceleration
	segments = append(segments, segment{duration: decelDuration, start: point{t: t, x: x, v: v, a: -direction * acceleration}})
	return newPlan(segments)
}

// NewMovePlan first removes any velocity that is incompatible with the new
// command using maximum acceleration, then moves from that actual stopping
// position to the target. This deliberately avoids restarting the profile from
// the previous command's original start state.
func NewMovePlan(startX, startV, target, maxVelocity, acceleration float64) *Plan {
	brake := brakePlan(startX, startV, acceleration)
	endX, _, _, _ := brake.At(brake.Duration())
	move := restMovePlan(endX, target, maxVelocity, acceleration)
	segments := make([]segment, 0, len(brake.segments)+len(move.segments))
	offset := 0.0
	for _, item := range brake.segments {
		if item.duration > epsilon {
			item.start.t = 0
			segments = append(segments, item)
		}
		offset += item.duration
	}
	for _, item := range move.segments {
		if item.duration > epsilon {
			item.start.t = 0
			segments = append(segments, item)
		}
		offset += item.duration
	}
	return newPlan(segments)
}
