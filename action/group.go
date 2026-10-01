package action

import (
	"encoding/json"
	"errors"
	"fmt"
	"sync"
	"time"

	"motion/axis"
)

// ConstrainedReaction selects how a coordinated group responds when any one
// axis enters a constrained state (emergency stop, or a soft stop commanded
// outside the group). The group never lets the remaining axes run on and
// pretend the whole group arrived together.
type ConstrainedReaction int

const (
	// ReactDecelerateAndWait brakes every unconstrained axis to rest, holds
	// the active group motion, and resumes it from the real stopped positions
	// once the constraint clears.
	ReactDecelerateAndWait ConstrainedReaction = iota
	// ReactAbortAll brakes every unconstrained axis to rest, aborts the
	// active group motion and cancels the queued group motions.
	ReactAbortAll
)

// GroupMove is one coordinated point-to-point motion: every axis i travels to
// Targets[i]. It completes only when EVERY axis is at rest on its own target.
type GroupMove struct {
	ID        string
	Targets   []float64
	CreatedAt time.Time

	state State
	abort AbortReason
}

// State returns the lifecycle state.
func (g *GroupMove) State() State { return g.state }

// AbortReason returns the interruption reason for an aborted group motion.
func (g *GroupMove) AbortReason() AbortReason { return g.abort }

// Group runs a FIFO queue of coordinated motions on several virtual axes.
//
// Coordination rule: if any axis becomes constrained while a group motion is
// active, the whole group decelerates together; depending on the configured
// ConstrainedReaction it then waits and resumes, or terminates the motion and
// the queue. A group motion is Completed only when all axes are individually
// AtTarget, so a partially arrived group is never reported as done.
type Group struct {
	mu       sync.Mutex
	axes     []*axis.Axis
	cfgs     []axis.Config
	reaction ConstrainedReaction

	active *GroupMove
	queue  []*GroupMove

	// holding marks a group-initiated deceleration: axes are braking and the
	// active motion waits to be resumed. It distinguishes our own stop from
	// an externally commanded constraint, and is persisted in snapshots.
	holding bool

	now func() time.Time
	seq uint64
}

// NewGroup creates a coordinated group over one fresh virtual axis per config.
func NewGroup(cfgs []axis.Config, reaction ConstrainedReaction) (*Group, error) {
	if len(cfgs) == 0 {
		return nil, errors.New("action: group needs at least one axis")
	}
	g := &Group{reaction: reaction, now: time.Now}
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

// Axes exposes the underlying virtual axes for readout.
func (g *Group) Axes() []*axis.Axis {
	g.mu.Lock()
	defer g.mu.Unlock()
	out := make([]*axis.Axis, len(g.axes))
	copy(out, g.axes)
	return out
}

// Enqueue adds a coordinated motion to the back of the queue and returns it.
func (g *Group) Enqueue(targets []float64) (*GroupMove, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if len(targets) != len(g.axes) {
		return nil, fmt.Errorf("action: got %d targets for %d axes", len(targets), len(g.axes))
	}
	m := &GroupMove{
		ID:        g.nextIDLocked(),
		Targets:   append([]float64(nil), targets...),
		CreatedAt: g.now(),
		state:     Pending,
	}
	g.queue = append(g.queue, m)
	g.dispatchLocked()
	return m, nil
}

// RetargetActive changes the goals of the active group motion in place. Every
// axis plans from its current position and velocity, so motion is continuous.
func (g *Group) RetargetActive(targets []float64) (*GroupMove, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if len(targets) != len(g.axes) {
		return nil, fmt.Errorf("action: got %d targets for %d axes", len(targets), len(g.axes))
	}
	if g.anyEmergencyLocked() {
		return nil, errors.New("action: emergency stop is active; call Reset first")
	}
	if g.active == nil || g.active.state != Active || g.holding {
		return nil, errors.New("action: no running group motion to retarget")
	}
	for i, ax := range g.axes {
		if err := ax.SetTarget(targets[i]); err != nil {
			return nil, err
		}
	}
	g.active.Targets = append([]float64(nil), targets...)
	return g.active, nil
}

// SoftStop brakes the whole group to rest, aborts the active motion and
// cancels the queue (StopEndQueue semantics at group scope).
func (g *Group) SoftStop() {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.holding = false
	g.abortActiveLocked(AbortedSoftStop)
	g.cancelAllLocked()
	for _, ax := range g.axes {
		ax.SoftStop()
	}
}

// EmergencyStop latches an emergency stop on every axis, aborts the active
// motion and cancels the queue. New targets are rejected until Reset.
func (g *Group) EmergencyStop() {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.holding = false
	g.abortActiveLocked(AbortedEmergencyStop)
	g.cancelAllLocked()
	for _, ax := range g.axes {
		ax.EmergencyStop()
	}
}

// Reset clears latched emergency stops on all axes once they are at rest.
func (g *Group) Reset() error {
	g.mu.Lock()
	defer g.mu.Unlock()
	for _, ax := range g.axes {
		if err := ax.Reset(); err != nil {
			return err
		}
	}
	return nil
}

// Active returns the currently executing group motion, if any.
func (g *Group) Active() *GroupMove {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.active != nil && g.active.state == Active {
		return g.active
	}
	return nil
}

// Pending returns a snapshot slice of group motions still waiting in the queue.
func (g *Group) Pending() []*GroupMove {
	g.mu.Lock()
	defer g.mu.Unlock()
	out := make([]*GroupMove, len(g.queue))
	copy(out, g.queue)
	return out
}

// Holding reports whether the group is decelerating or waiting out a
// constraint before resuming the active motion.
func (g *Group) Holding() bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.holding
}

// Step advances every axis one sample and applies the coordination rules.
func (g *Group) Step() {
	g.mu.Lock()
	defer g.mu.Unlock()
	for _, ax := range g.axes {
		ax.Step()
	}

	if g.active != nil && g.active.state == Active {
		switch {
		case g.constrainedLocked() && !g.holding:
			// An axis became constrained outside our own stop: decelerate
			// the whole group together, then wait or terminate.
			for _, ax := range g.axes {
				if !ax.EmergencyStopped() {
					ax.SoftStop()
				}
			}
			if g.reaction == ReactAbortAll {
				g.abortActiveLocked(AbortedSoftStop)
				g.cancelAllLocked()
			} else {
				g.holding = true
			}
		case g.holding && g.clearedLocked():
			// Constraint cleared and everyone at rest: resume the held
			// motion from the real stopped positions.
			g.holding = false
			for i, ax := range g.axes {
				if err := ax.SetTarget(g.active.Targets[i]); err != nil {
					g.abortActiveLocked(AbortedSoftStop)
					g.cancelAllLocked()
					break
				}
			}
		case !g.holding && g.allAtTargetLocked():
			// Complete only when EVERY axis rests on its own target.
			for _, ax := range g.axes {
				ax.SettleIfAtTarget()
			}
			g.active.state = Completed
			g.active = nil
		}
	}
	g.dispatchLocked()
}

// constrainedLocked reports whether any axis is in a stop state that the group
// did not initiate itself.
func (g *Group) constrainedLocked() bool {
	for _, ax := range g.axes {
		if ax.EmergencyStopped() {
			return true
		}
		if !g.holding && (ax.Mode() == axis.SoftStopping || ax.Mode() == axis.EmergencyStopping) {
			return true
		}
	}
	return false
}

// clearedLocked reports whether every axis is at rest and unconstrained, so a
// held motion can resume.
func (g *Group) clearedLocked() bool {
	for _, ax := range g.axes {
		if ax.EmergencyStopped() || ax.Velocity() != 0 {
			return false
		}
		if ax.Mode() == axis.SoftStopping || ax.Mode() == axis.EmergencyStopping {
			return false
		}
	}
	return true
}

func (g *Group) allAtTargetLocked() bool {
	for _, ax := range g.axes {
		if !ax.AtTarget() {
			return false
		}
	}
	return true
}

func (g *Group) anyEmergencyLocked() bool {
	for _, ax := range g.axes {
		if ax.EmergencyStopped() {
			return true
		}
	}
	return false
}

// dispatchLocked starts the next pending group motion when all axes are free.
func (g *Group) dispatchLocked() {
	if g.active != nil || g.holding {
		return
	}
	for _, ax := range g.axes {
		if ax.EmergencyStopped() || ax.Mode() == axis.SoftStopping || ax.Mode() == axis.EmergencyStopping {
			return
		}
	}
	for len(g.queue) > 0 {
		m := g.queue[0]
		g.queue = g.queue[1:]
		if m.state != Pending {
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
			for _, ax := range g.axes {
				ax.ClearTarget()
			}
			g.queue = append([]*GroupMove{m}, g.queue...)
			return
		}
		m.state = Active
		g.active = m
		return
	}
}

func (g *Group) abortActiveLocked(reason AbortReason) {
	if g.active != nil && g.active.state == Active {
		g.active.state = Aborted
		g.active.abort = reason
	}
	g.active = nil
}

func (g *Group) cancelAllLocked() {
	for _, m := range g.queue {
		if m.state == Pending {
			m.state = Canceled
		}
	}
	g.queue = nil
}

func (g *Group) nextIDLocked() string {
	g.seq++
	return fmt.Sprintf("group-move-%d-%d", g.now().UnixNano(), g.seq)
}

// wireGroupMove is the serialized form of a GroupMove.
type wireGroupMove struct {
	ID        string      `json:"id"`
	Targets   []float64   `json:"targets"`
	CreatedAt time.Time   `json:"created_at"`
	State     State       `json:"state"`
	Abort     AbortReason `json:"abort_reason"`
}

// GroupSnapshot captures every axis mid-motion plus the queue and hold state.
type GroupSnapshot struct {
	Configs  []axis.Config       `json:"configs"`
	Reaction ConstrainedReaction `json:"reaction"`
	Axes     []axis.Snapshot     `json:"axes"`
	Active   *wireGroupMove      `json:"active"`
	Queue    []*wireGroupMove    `json:"queue"`
	Holding  bool                `json:"holding"`
}

// Save serializes the full group state, including an in-progress deceleration.
func (g *Group) Save() ([]byte, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	snap := GroupSnapshot{
		Configs:  append([]axis.Config(nil), g.cfgs...),
		Reaction: g.reaction,
		Axes:     make([]axis.Snapshot, 0, len(g.axes)),
		Queue:    make([]*wireGroupMove, 0, len(g.queue)),
		Holding:  g.holding,
	}
	for _, ax := range g.axes {
		snap.Axes = append(snap.Axes, ax.Snapshot())
	}
	if g.active != nil {
		snap.Active = toWireGroup(g.active)
	}
	for _, m := range g.queue {
		snap.Queue = append(snap.Queue, toWireGroup(m))
	}
	return json.Marshal(snap)
}

// RestoreGroup decodes a snapshot into a ready-to-run group. A motion saved
// mid-deceleration keeps holding and resumes under the same rules.
func RestoreGroup(data []byte) (*Group, error) {
	var snap GroupSnapshot
	if err := json.Unmarshal(data, &snap); err != nil {
		return nil, err
	}
	g, err := NewGroup(snap.Configs, snap.Reaction)
	if err != nil {
		return nil, err
	}
	if len(snap.Axes) != len(g.axes) {
		return nil, errors.New("action: snapshot axis count mismatch")
	}
	for i, s := range snap.Axes {
		g.axes[i].Restore(s)
	}
	g.holding = snap.Holding
	if snap.Active != nil {
		g.active = fromWireGroup(snap.Active)
	}
	for _, w := range snap.Queue {
		g.queue = append(g.queue, fromWireGroup(w))
	}
	return g, nil
}

func toWireGroup(m *GroupMove) *wireGroupMove {
	return &wireGroupMove{
		ID:        m.ID,
		Targets:   append([]float64(nil), m.Targets...),
		CreatedAt: m.CreatedAt,
		State:     m.state,
		Abort:     m.abort,
	}
}

func fromWireGroup(w *wireGroupMove) *GroupMove {
	return &GroupMove{
		ID:        w.ID,
		Targets:   append([]float64(nil), w.Targets...),
		CreatedAt: w.CreatedAt,
		state:     w.State,
		abort:     w.Abort,
	}
}
