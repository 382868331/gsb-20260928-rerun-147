package action

import (
	"encoding/json"
	"errors"
	"fmt"
	"time"

	"motion/axis"
)

// StopPolicy decides how a coordinated axis group reacts when one axis
// enters a constrained state (soft-stopped or emergency-stopped) while a
// group motion is active.
type StopPolicy int

const (
	// TerminateOnLimit brakes every axis to rest, aborts the active group
	// motion and cancels the group queue. Nothing pretends to have arrived.
	TerminateOnLimit StopPolicy = iota
	// WaitAndResume brakes every axis to rest keeping all targets, waits
	// until the constrained axis is released and every axis is at rest,
	// then resumes the whole group toward the original targets.
	WaitAndResume
)

// GroupMove is one coordinated point-to-point motion across all axes of a
// Group. It completes only when EVERY axis is at rest on its own target; a
// subset of axes arriving early never completes the move.
type GroupMove struct {
	ID        string
	Targets   []float64
	CreatedAt time.Time

	state    State
	abort    AbortReason
	startPos []float64
	started  bool
}

// State returns the lifecycle state (same lifecycle as a single-axis Move).
func (m *GroupMove) State() State { return m.state }

// AbortReason returns the interruption reason for an aborted motion.
func (m *GroupMove) AbortReason() AbortReason { return m.abort }

// StartPos records the actual per-axis positions the motion began from.
func (m *GroupMove) StartPos() ([]float64, bool) {
	if !m.started {
		return nil, false
	}
	out := make([]float64, len(m.startPos))
	copy(out, m.startPos)
	return out, true
}

func (m *GroupMove) activate(fromPos []float64) {
	m.state = Active
	m.startPos = append([]float64(nil), fromPos...)
	m.started = true
}

func (m *GroupMove) complete() { m.state = Completed }
func (m *GroupMove) cancel()   { m.state = Canceled }
func (m *GroupMove) abortWith(reason AbortReason) {
	m.state = Aborted
	m.abort = reason
}

// Group coordinates several virtual axes so that constrained states are
// handled as a group: when any single axis can no longer follow its target,
// the whole group decelerates, waits or terminates together according to the
// configured StopPolicy. No axis reports arrival on behalf of the others.
type Group struct {
	axes   []*axis.Axis
	cfgs   []axis.Config
	policy StopPolicy

	active  *GroupMove
	queue   []*GroupMove
	waiting bool // paused as a group, will resume toward kept targets

	now func() time.Time
	seq uint64
}

// NewGroup creates a coordinated group of virtual axes, one per config.
func NewGroup(policy StopPolicy, cfgs ...axis.Config) (*Group, error) {
	if len(cfgs) == 0 {
		return nil, errors.New("action: group needs at least one axis")
	}
	g := &Group{policy: policy, now: time.Now}
	for _, cfg := range cfgs {
		ax, err := axis.New(cfg)
		if err != nil {
			return nil, err
		}
		g.axes = append(g.axes, ax)
		g.cfgs = append(g.cfgs, cfg)
	}
	return g, nil
}

// Len returns the number of axes in the group.
func (g *Group) Len() int { return len(g.axes) }

// Axis exposes one virtual axis for readout.
func (g *Group) Axis(i int) *axis.Axis { return g.axes[i] }

// Positions returns a snapshot of the current position of every axis.
func (g *Group) Positions() []float64 {
	out := make([]float64, len(g.axes))
	for i, ax := range g.axes {
		out[i] = ax.Position()
	}
	return out
}

// Enqueue adds a coordinated motion to the back of the group queue. The
// number of targets must match the number of axes.
func (g *Group) Enqueue(targets ...float64) (*GroupMove, error) {
	if len(targets) != len(g.axes) {
		return nil, fmt.Errorf("action: got %d targets for %d axes", len(targets), len(g.axes))
	}
	m := &GroupMove{
		ID:        g.nextID(),
		Targets:   append([]float64(nil), targets...),
		CreatedAt: g.now(),
		state:     Pending,
	}
	g.queue = append(g.queue, m)
	g.dispatch()
	return m, nil
}

// RetargetActive changes the goals of the active group motion. Every axis
// re-plans from its current position and velocity, so motion stays
// continuous; axes that have not arrived yet never report arrival.
func (g *Group) RetargetActive(targets ...float64) (*GroupMove, error) {
	if len(targets) != len(g.axes) {
		return nil, fmt.Errorf("action: got %d targets for %d axes", len(targets), len(g.axes))
	}
	for _, ax := range g.axes {
		if ax.EmergencyStopped() {
			return nil, errors.New("action: emergency stop is active; call Reset first")
		}
	}
	if g.active != nil && g.active.State() == Active && !g.waiting {
		copy(g.active.Targets, targets)
		for i, ax := range g.axes {
			if err := ax.SetTarget(targets[i]); err != nil {
				return nil, err
			}
		}
		return g.active, nil
	}
	return g.Enqueue(targets...)
}

// SoftStop ends the queue as a group: the active motion is aborted, pending
// motions are canceled and every axis brakes smoothly to rest.
func (g *Group) SoftStop() {
	g.abortActive(AbortedSoftStop)
	g.cancelAll()
	g.waiting = false
	for _, ax := range g.axes {
		ax.SoftStop()
	}
}

// EmergencyStop aborts the active motion, cancels the queue and latches an
// emergency stop on EVERY axis of the group.
func (g *Group) EmergencyStop() {
	g.abortActive(AbortedEmergencyStop)
	g.cancelAll()
	g.waiting = false
	for _, ax := range g.axes {
		ax.EmergencyStop()
	}
}

// Reset clears latched emergency stops on all axes once the group is at rest.
func (g *Group) Reset() error {
	for _, ax := range g.axes {
		if err := ax.Reset(); err != nil {
			return err
		}
	}
	return nil
}

// Active returns the currently executing group motion, if any.
func (g *Group) Active() *GroupMove {
	if g.active != nil && g.active.State() == Active {
		return g.active
	}
	return nil
}

// Pending returns a snapshot slice of motions still waiting in the queue.
func (g *Group) Pending() []*GroupMove {
	out := make([]*GroupMove, len(g.queue))
	copy(out, g.queue)
	return out
}

// Waiting reports whether the group is paused because one axis entered a
// constrained state and the WaitAndResume policy is holding the group.
func (g *Group) Waiting() bool { return g.waiting }

// Step advances every axis one sample and applies the group coordination
// rules: constrained axes drag the whole group into a deceleration, a wait
// or a termination, and a group motion completes only when every axis is at
// rest on its own target.
func (g *Group) Step() {
	for _, ax := range g.axes {
		ax.Step()
	}

	if g.waiting {
		g.stepWaiting()
	} else {
		g.handleConstrainedAxes()
	}

	// Complete only when EVERY axis is at rest on its own target.
	if g.active != nil && g.active.State() == Active && !g.waiting {
		allThere := true
		for _, ax := range g.axes {
			if !ax.AtTarget() {
				allThere = false
				break
			}
		}
		if allThere {
			for _, ax := range g.axes {
				ax.SettleIfAtTarget()
			}
			g.active.complete()
			g.active = nil
			g.dispatch()
		}
	}
	g.dispatch()
}

// constrained reports whether an axis can no longer follow its target on its
// own: externally soft-stopped, emergency-stopping or emergency-latched.
func constrained(ax *axis.Axis) bool {
	return ax.EmergencyStopped() ||
		ax.Mode() == axis.SoftStopping ||
		ax.Mode() == axis.EmergencyStopping
}

// handleConstrainedAxes applies the group policy when any axis entered a
// constrained state outside of a group-initiated wait.
func (g *Group) handleConstrainedAxes() {
	anyEstopped := false
	anyConstrained := false
	for _, ax := range g.axes {
		if ax.EmergencyStopped() || ax.Mode() == axis.EmergencyStopping {
			anyEstopped = true
		}
		if constrained(ax) {
			anyConstrained = true
		}
	}
	if !anyConstrained {
		return
	}
	if anyEstopped {
		// An emergency on one axis terminates the group motion: brake the
		// remaining axes in a controlled way and end the queue.
		g.abortActive(AbortedEmergencyStop)
		g.cancelAll()
		for _, ax := range g.axes {
			if !ax.EmergencyStopped() {
				ax.SoftStop()
			}
		}
		return
	}
	switch g.policy {
	case WaitAndResume:
		// Decelerate together, keep every target, resume when released.
		for _, ax := range g.axes {
			ax.Pause()
		}
		g.waiting = true
	default: // TerminateOnLimit
		g.abortActive(AbortedSoftStop)
		g.cancelAll()
		for _, ax := range g.axes {
			ax.SoftStop()
		}
	}
}

// stepWaiting waits for every axis to be released and at rest, then resumes
// the whole group toward the kept targets from the real positions.
func (g *Group) stepWaiting() {
	for _, ax := range g.axes {
		if ax.EmergencyStopped() || ax.Mode() == axis.EmergencyStopping {
			// The wait turned into an emergency: terminate as a group.
			g.abortActive(AbortedEmergencyStop)
			g.cancelAll()
			g.waiting = false
			for _, other := range g.axes {
				if !other.EmergencyStopped() {
					other.SoftStop()
				}
			}
			return
		}
	}
	for _, ax := range g.axes {
		if ax.Velocity() != 0 || ax.Mode() == axis.SoftStopping {
			return // still braking as a group
		}
	}
	g.waiting = false
	if g.active != nil && g.active.State() == Active {
		for i, ax := range g.axes {
			// Re-plan from the real position and velocity; no jump.
			_ = ax.SetTarget(g.active.Targets[i])
		}
	}
}

func (g *Group) dispatch() {
	if g.active != nil || g.waiting {
		return
	}
	for _, ax := range g.axes {
		if constrained(ax) {
			return
		}
	}
	for len(g.queue) > 0 {
		m := g.queue[0]
		g.queue = g.queue[1:]
		if m.State() != Pending {
			continue
		}
		ok := true
		for i, ax := range g.axes {
			if err := ax.SetTarget(m.Targets[i]); err != nil {
				ok = false
				break
			}
		}
		if !ok {
			g.queue = append([]*GroupMove{m}, g.queue...)
			return
		}
		m.activate(g.Positions())
		g.active = m
		return
	}
}

func (g *Group) abortActive(reason AbortReason) {
	if g.active != nil && g.active.State() == Active {
		g.active.abortWith(reason)
	}
	g.active = nil
}

func (g *Group) cancelAll() {
	for _, m := range g.queue {
		if m.State() == Pending {
			m.cancel()
		}
	}
	g.queue = nil
}

func (g *Group) nextID() string {
	g.seq++
	return fmt.Sprintf("groupmove-%d-%d", g.now().UnixNano(), g.seq)
}

// wireGroupMove is the serialized form of a GroupMove.
type wireGroupMove struct {
	ID        string      `json:"id"`
	Targets   []float64   `json:"targets"`
	CreatedAt time.Time   `json:"created_at"`
	State     State       `json:"state"`
	Abort     AbortReason `json:"abort_reason"`
	StartPos  []float64   `json:"start_pos"`
	Started   bool        `json:"started"`
}

// GroupSnapshot captures every axis mid-motion plus the group queue. Saving
// during a stop is supported: the stop continues under the same rules after
// a restore.
type GroupSnapshot struct {
	Configs []axis.Config    `json:"configs"`
	Policy  StopPolicy       `json:"policy"`
	Axes    []axis.Snapshot  `json:"axes"`
	Active  *wireGroupMove   `json:"active"`
	Queue   []*wireGroupMove `json:"queue"`
	Waiting bool             `json:"waiting"`
}

// Save serializes the full group state, including mid-stop states.
func (g *Group) Save() ([]byte, error) {
	snap := GroupSnapshot{
		Configs: g.cfgs,
		Policy:  g.policy,
		Axes:    make([]axis.Snapshot, len(g.axes)),
		Queue:   make([]*wireGroupMove, 0, len(g.queue)),
		Waiting: g.waiting,
	}
	for i, ax := range g.axes {
		snap.Axes[i] = ax.Snapshot()
	}
	if g.active != nil {
		snap.Active = groupMoveToWire(g.active)
	}
	for _, m := range g.queue {
		snap.Queue = append(snap.Queue, groupMoveToWire(m))
	}
	return json.Marshal(snap)
}

// RestoreGroup decodes a snapshot into a ready-to-run group. A group saved
// while decelerating or waiting resumes under exactly the same coordination
// rules; nothing is assumed to have reached its target.
func RestoreGroup(data []byte) (*Group, error) {
	var snap GroupSnapshot
	if err := json.Unmarshal(data, &snap); err != nil {
		return nil, err
	}
	g, err := NewGroup(snap.Policy, snap.Configs...)
	if err != nil {
		return nil, err
	}
	if len(snap.Axes) != len(g.axes) {
		return nil, errors.New("action: snapshot axis count mismatch")
	}
	for i, s := range snap.Axes {
		g.axes[i].Restore(s)
	}
	g.waiting = snap.Waiting
	if snap.Active != nil {
		g.active = groupMoveFromWire(snap.Active)
	}
	for _, w := range snap.Queue {
		g.queue = append(g.queue, groupMoveFromWire(w))
	}
	return g, nil
}

func groupMoveToWire(m *GroupMove) *wireGroupMove {
	sp, started := m.StartPos()
	return &wireGroupMove{
		ID:        m.ID,
		Targets:   append([]float64(nil), m.Targets...),
		CreatedAt: m.CreatedAt,
		State:     m.State(),
		Abort:     m.AbortReason(),
		StartPos:  sp,
		Started:   started,
	}
}

func groupMoveFromWire(w *wireGroupMove) *GroupMove {
	return &GroupMove{
		ID:        w.ID,
		Targets:   append([]float64(nil), w.Targets...),
		CreatedAt: w.CreatedAt,
		state:     w.State,
		abort:     w.Abort,
		startPos:  append([]float64(nil), w.StartPos...),
		started:   w.Started,
	}
}
