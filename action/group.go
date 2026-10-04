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

	state   State
	abort   AbortReason
	partial bool

	// axes maps each group axis index to the shared cell axis name; empty in
	// standalone groups (where axis i is implicit).
	axes []string
	// gates are named handoff conditions that must be open at dispatch.
	gates []string
	// axisConds are per-axis position predicates re-checked at every retry.
	axisConds []AxisCondition

	// acquired lists the cell axis names this move already owns. A move that
	// never acquired anything is Waiting; one with a non-empty acquired set
	// that lost dispatch is reported Partial.
	acquired []string

	// handoffs records one entry per shared axis at the instant the previous
	// owner released it into this move.
	handoffs []HandoffRecord

	// OnDone, when set, is invoked once when the move reaches a terminal
	// state. It is the completion event channel for late events: the record
	// always belongs to this exact move, even if a successor already runs.
	onDone func(*GroupMove)
	done   bool
}

// State returns the lifecycle state.
func (g *GroupMove) State() State { return g.state }

// AbortReason returns the interruption reason for an aborted group motion.
func (g *GroupMove) AbortReason() AbortReason { return g.abort }

// Partial reports whether the move owned and moved at least one axis before
// losing dispatch (waiting on a handoff, or braking in a safe stop). The
// displacement already executed is real and is never rolled back.
func (g *GroupMove) Partial() bool { return g.partial }

// AxisNames returns the shared cell axis names this move addresses (empty for
// a standalone group).
func (g *GroupMove) AxisNames() []string { return append([]string(nil), g.axes...) }

// Gates returns the named handoff gates required before this move starts.
func (g *GroupMove) Gates() []string { return append([]string(nil), g.gates...) }

// Handoffs returns the recorded handoffs for this move, oldest first.
func (g *GroupMove) Handoffs() []HandoffRecord { return append([]HandoffRecord(nil), g.handoffs...) }

// HandoffRecord captures one axis being handed from one group move to the
// next. FromID/ToID identify the moves; At is the virtual position of the
// axis at release; AtRest means it had fully settled; At is time of release.
type HandoffRecord struct {
	Axis     string    `json:"axis"`
	FromID   string    `json:"from_id"`
	ToID     string    `json:"to_id"`
	At       float64   `json:"at"`
	AtRest   bool      `json:"at_rest"`
	HandedAt time.Time `json:"handed_at"`
}

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

	id    string
	names []string // cell axis name per axis index; nil standalone
	cell  *Cell    // non-nil when bound into a shared-axis cell
	bound bool     // when true, Cell.Step drives the axes

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
	axes, err := newAxes(cfgs)
	if err != nil {
		return nil, err
	}
	return &Group{
		reaction: reaction,
		axes:     axes,
		cfgs:     append([]axis.Config(nil), cfgs...),
		now:      time.Now,
	}, nil
}

func newAxes(cfgs []axis.Config) ([]*axis.Axis, error) {
	axes := make([]*axis.Axis, 0, len(cfgs))
	for _, cfg := range cfgs {
		ax, err := axis.New(cfg)
		if err != nil {
			return nil, err
		}
		axes = append(axes, ax)
	}
	return axes, nil
}

// ID returns the group identifier ("" for standalone groups).
func (g *Group) ID() string {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.id
}

// Names returns the shared cell axis names in group index order (empty for a
// standalone group).
func (g *Group) Names() []string {
	g.mu.Lock()
	defer g.mu.Unlock()
	return append([]string(nil), g.names...)
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
	return g.enqueueLocked(targets, nil, nil, nil)
}

// EnqueueWithHandoff adds a coordinated move that only starts once explicit
// handoff conditions are satisfied. axes names the shared cell axis for each
// target (group index order); conditions carry per-axis position predicates
// and named gates. In a standalone group axes/conditions may be nil and the
// move behaves like Enqueue.
func (g *Group) EnqueueWithHandoff(targets []float64, axes []string, conds *HandoffConditions, onDone func(*GroupMove)) (*GroupMove, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.enqueueLocked(targets, axes, conds, onDone)
}

func (g *Group) enqueueLocked(targets []float64, axes []string, conds *HandoffConditions, onDone func(*GroupMove)) (*GroupMove, error) {
	if len(targets) != len(g.axes) {
		return nil, fmt.Errorf("action: got %d targets for %d axes", len(targets), len(g.axes))
	}
	if axes != nil {
		if len(axes) != len(g.axes) {
			return nil, fmt.Errorf("action: got %d axis names for %d axes", len(axes), len(g.axes))
		}
		if g.cell == nil {
			return nil, errors.New("action: handoff axis names require a group bound to a cell")
		}
		for _, n := range axes {
			if n == "" {
				return nil, errors.New("action: handoff axis name must not be empty")
			}
		}
	}
	m := &GroupMove{
		ID:        g.nextIDLocked(),
		Targets:   append([]float64(nil), targets...),
		CreatedAt: g.now(),
		state:     Pending,
		onDone:    onDone,
	}
	if axes != nil {
		m.axes = append([]string(nil), axes...)
	} else if g.names != nil {
		m.axes = append([]string(nil), g.names...)
	}
	if conds != nil {
		m.gates = append([]string(nil), conds.Gates...)
		m.axisConds = append([]AxisCondition(nil), conds.Axes...)
	}
	g.queue = append(g.queue, m)
	g.dispatchLocked(false)
	return m, nil
}

// CancelWaiting removes every queued move that has not acquired an axis yet
// (Pending/Waiting). Moves already in motion and moves that executed part of
// their displacement are left untouched. It returns the canceled moves.
func (g *Group) CancelWaiting() []*GroupMove {
	g.mu.Lock()
	defer g.mu.Unlock()
	return g.cancelWaitingLocked()
}

// CancelMove cancels one specific not-yet-acquired waiting move. It returns
// false if the move already acquired an axis or is not queued on this group.
func (g *Group) CancelMove(m *GroupMove) bool {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.active == m {
		return false
	}
	for i, q := range g.queue {
		if q == m && len(m.acquired) == 0 && (m.state == Pending || m.state == Waiting) {
			g.queue = append(g.queue[:i], g.queue[i+1:]...)
			m.state = Canceled
			g.fireDoneLocked(m)
			return true
		}
	}
	return false
}

func (g *Group) cancelWaitingLocked() []*GroupMove {
	kept := g.queue[:0]
	var canceled []*GroupMove
	for _, m := range g.queue {
		if len(m.acquired) == 0 && (m.state == Pending || m.state == Waiting) {
			m.state = Canceled
			canceled = append(canceled, m)
			g.fireDoneLocked(m)
		} else {
			kept = append(kept, m)
		}
	}
	g.queue = kept
	return canceled
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
	if g.active == nil || (g.active.state != Active && g.active.state != Waiting) || g.holding {
		return nil, errors.New("action: no running group motion to retarget")
	}
	if g.active.state == Waiting {
		return nil, errors.New("action: move is still waiting for an axis handoff")
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
	g.beginStopLocked(AbortedSoftStop)
}

// EmergencyStop latches an emergency stop on every axis, aborts the active
// motion and cancels the queue. New targets are rejected until Reset.
func (g *Group) EmergencyStop() {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.beginStopLocked(AbortedEmergencyStop)
}

// beginStopLocked transitions any active or waiting-with-partial-progress
// move into a safe stop. Axes keep their ownership until fully at rest: an
// emergency stop never auto-releases a still-moving axis, and already
// executed displacement stays executed.
func (g *Group) beginStopLocked(reason AbortReason) {
	g.holding = false
	if g.active != nil {
		m := g.active
		m.partial = len(m.acquired) > 0
		m.state = Stopping
		m.abort = reason
		for i, ax := range g.axes {
			if !g.ownsIndexLocked(i) {
				continue
			}
			if reason == AbortedEmergencyStop {
				ax.EmergencyStop()
			} else if !ax.EmergencyStopped() {
				ax.SoftStop()
			}
		}
	}
	g.cancelAllLocked()
}

// Reset clears latched emergency stops on all axes once they are at rest.
func (g *Group) Reset() error {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.cell != nil {
		g.mu.Unlock()
		err := g.cell.resetGroup(g)
		g.mu.Lock()
		return err
	}
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
	if g.active != nil && (g.active.state == Active || g.active.state == Waiting) {
		return g.active
	}
	return nil
}

// Stopping returns the move currently braking to a safe stop, if any. It
// already executed part of its displacement and retains axis ownership until
// every owned axis is at rest.
func (g *Group) Stopping() *GroupMove {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.active != nil && g.active.state == Stopping {
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
	if !g.bound {
		for _, ax := range g.axes {
			ax.Step()
		}
	}
	g.stepLocked(false)
}

// stepLocked runs one coordination pass over already-advanced axes. When
// cellLocked is true, cell callbacks assume c.mu is held (Cell.Step path).
func (g *Group) stepLocked(cellLocked bool) {
	if g.active != nil {
		switch g.active.state {
		case Stopping:
			// Ownership is released only once every owned axis is fully at rest;
			// an emergency-latched axis is additionally kept until Reset.
			if g.ownedAtRestLocked() {
				m := g.active
				m.state = Aborted
				g.releaseOwnedLocked(m)
				g.fireDoneLocked(m)
				g.active = nil
			}
			return // dispatch stays blocked while stopping
		case Active:
			if g.bound {
				if cellLocked {
					g.cell.tickMoveLocked(g, g.active)
				}
			}
			switch {
			case g.constrainedLocked() && !g.holding:
				// An owned axis became constrained outside our own command: the
				// whole group decelerates together.
				for i, ax := range g.axes {
					if !g.ownsIndexLocked(i) || ax.EmergencyStopped() {
						continue
					}
					ax.SoftStop()
				}
				if g.reaction == ReactAbortAll {
					g.beginStopLocked(AbortedSoftStop)
					return
				}
				g.holding = true
			case g.holding && g.clearedLocked():
				g.holding = false
				if !g.reissueGoalsLocked(g.active) {
					g.beginStopLocked(AbortedSoftStop)
					return
				}
			case !g.holding && g.allAtTargetLocked():
				for i, ax := range g.axes {
					if g.ownsIndexLocked(i) {
						ax.SettleIfAtTarget()
					}
				}
				m := g.active
				m.state = Completed
				g.releaseOwnedLocked(m)
				g.active = nil
				g.fireDoneLocked(m)
			}
		case Waiting:
			if g.bound {
				if cellLocked {
					g.cell.tickMoveLocked(g, g.active)
				}
			}
		}
	}
	g.dispatchLocked(cellLocked)
}

func (g *Group) reissueGoalsLocked(m *GroupMove) bool {
	for i, ax := range g.axes {
		if !g.ownsIndexLocked(i) {
			continue
		}
		if err := ax.SetTarget(m.Targets[i]); err != nil {
			return false
		}
	}
	return true
}

// constrainedLocked reports whether any axis is in a stop state that the group
// did not initiate itself.
func (g *Group) constrainedLocked() bool {
	for i, ax := range g.axes {
		if !g.ownsIndexLocked(i) {
			continue
		}
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
	for i, ax := range g.axes {
		if !g.ownsIndexLocked(i) {
			continue
		}
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
	for i, ax := range g.axes {
		if !g.ownsIndexLocked(i) {
			return false
		}
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

// ownedAtRestLocked reports whether every axis the active move currently owns
// is fully settled (and, for emergency-latched axes, still latched-but-rest is
// not releasable until Reset: callers gate Reset separately through the cell).
func (g *Group) ownedAtRestLocked() bool {
	m := g.active
	for _, name := range m.acquired {
		i := g.indexOfNameLocked(name)
		if i < 0 {
			continue
		}
		ax := g.axes[i]
		if ax.Velocity() != 0 {
			return false
		}
		if ax.Mode() == axis.SoftStopping || ax.Mode() == axis.EmergencyStopping {
			return false
		}
		if ax.EmergencyStopped() {
			// A latched, fully settled axis is at rest but Reset must clear the
			// latch before ownership may change hands.
			return false
		}
	}
	return true
}

func (g *Group) ownsIndexLocked(i int) bool {
	if g.cell == nil || g.active == nil {
		return g.cell == nil
	}
	name := g.axisNameLocked(i)
	for _, n := range g.active.acquired {
		if n == name {
			return true
		}
	}
	return false
}

func (g *Group) axisNameLocked(i int) string {
	if g.names != nil {
		return g.names[i]
	}
	return ""
}

func (g *Group) indexOfNameLocked(name string) int {
	for i, n := range g.names {
		if n == name {
			return i
		}
	}
	return -1
}

// releaseOwnedLocked hands every axis owned by m back to the cell, recording a
// handoff per axis. In a standalone group it is a no-op on ownership.
func (g *Group) releaseOwnedLocked(m *GroupMove) {
	if g.cell == nil {
		return
	}
	g.cell.releaseMoveHeld(g, m)
}

func (g *Group) fireDoneLocked(m *GroupMove) {
	if m.done {
		return
	}
	m.done = true
	if m.onDone != nil {
		m.onDone(m)
	}
}

// dispatchLocked starts the next pending group motion when all axes are free.
func (g *Group) dispatchLocked(cellLocked bool) {
	if g.active != nil || g.holding {
		return
	}
	if g.cell == nil {
		g.dispatchStandaloneLocked()
		return
	}
	if cellLocked {
		g.cell.dispatchGroupHeld(g)
	} else {
		g.cell.dispatchGroupLocked(g)
	}
}

func (g *Group) dispatchStandaloneLocked() {
	for _, ax := range g.axes {
		if ax.EmergencyStopped() || ax.Mode() == axis.SoftStopping || ax.Mode() == axis.EmergencyStopping {
			return
		}
	}
	for len(g.queue) > 0 {
		m := g.queue[0]
		g.queue = g.queue[1:]
		if m.state != Pending && m.state != Waiting {
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

func (g *Group) cancelAllLocked() {
	kept := g.queue[:0]
	for _, m := range g.queue {
		if m.state == Pending || m.state == Waiting {
			if len(m.acquired) > 0 {
				m.partial = true
				kept = append(kept, m)
				continue
			}
			m.state = Canceled
			g.fireDoneLocked(m)
		} else {
			kept = append(kept, m)
		}
	}
	g.queue = kept
}

func (g *Group) nextIDLocked() string {
	g.seq++
	prefix := "group-move"
	if g.id != "" {
		prefix = g.id
	}
	return fmt.Sprintf("%s-%d-%d", prefix, g.now().UnixNano(), g.seq)
}

// wireGroupMove is the serialized form of a GroupMove.
type wireGroupMove struct {
	ID        string          `json:"id"`
	Targets   []float64       `json:"targets"`
	CreatedAt time.Time       `json:"created_at"`
	State     State           `json:"state"`
	Abort     AbortReason     `json:"abort_reason"`
	Partial   bool            `json:"partial"`
	Axes      []string        `json:"axes"`
	Gates     []string        `json:"gates"`
	Acquired  []string        `json:"acquired"`
	Handoffs  []HandoffRecord `json:"handoffs"`
	AxisConds []AxisCondition `json:"axis_conds"`
}

// GroupSnapshot captures every axis mid-motion plus the queue and hold state.
type GroupSnapshot struct {
	ID       string              `json:"id"`
	Configs  []axis.Config       `json:"configs"`
	Reaction ConstrainedReaction `json:"reaction"`
	Axes     []axis.Snapshot     `json:"axes"`
	Names    []string            `json:"names"`
	Active   *wireGroupMove      `json:"active"`
	Queue    []*wireGroupMove    `json:"queue"`
	Holding  bool                `json:"holding"`
	Seq      uint64              `json:"seq"`
}

// Save serializes the full group state, including an in-progress deceleration.
func (g *Group) Save() ([]byte, error) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.cell != nil {
		return nil, errors.New("action: save a cell-bound group through Cell.Save")
	}
	snap := GroupSnapshot{
		ID:       g.id,
		Configs:  append([]axis.Config(nil), g.cfgs...),
		Reaction: g.reaction,
		Axes:     make([]axis.Snapshot, 0, len(g.axes)),
		Queue:    make([]*wireGroupMove, 0, len(g.queue)),
		Holding:  g.holding,
		Seq:      g.seq,
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
	g.seq = snap.Seq
	g.id = snap.ID
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
		Partial:   m.partial,
		Axes:      append([]string(nil), m.axes...),
		Gates:     append([]string(nil), m.gates...),
		Acquired:  append([]string(nil), m.acquired...),
		Handoffs:  append([]HandoffRecord(nil), m.handoffs...),
		AxisConds: append([]AxisCondition(nil), m.axisConds...),
	}
}

func fromWireGroup(w *wireGroupMove) *GroupMove {
	return &GroupMove{
		ID:        w.ID,
		Targets:   append([]float64(nil), w.Targets...),
		CreatedAt: w.CreatedAt,
		state:     w.State,
		abort:     w.Abort,
		partial:   w.Partial,
		axes:      append([]string(nil), w.Axes...),
		gates:     append([]string(nil), w.Gates...),
		acquired:  append([]string(nil), w.Acquired...),
		handoffs:  append([]HandoffRecord(nil), w.Handoffs...),
		axisConds: append([]AxisCondition(nil), w.AxisConds...),
	}
}
