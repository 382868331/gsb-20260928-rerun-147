package action

import (
	"encoding/json"
	"errors"
	"fmt"
	"sync"
	"time"

	"motion/axis"
)

// Executor runs a FIFO queue of motions on one virtual axis.
//
// New motions normally join the queue. A target can also be pushed into the
// active motion (RetargetActive); in that case the axis keeps its current
// position and velocity and only the commanded goal changes, which avoids any
// restart-from-old-start-point jump.
type Executor struct {
	mu  sync.Mutex
	ax  *axis.Axis
	cfg axis.Config

	active *Move
	queue  []*Move

	// holdActive is set by a keep-goal soft stop: the active motion stays
	// Active while the axis brakes, and its goal is re-issued once the axis
	// is at rest. Persisted so a save/restore mid-stop resumes identically.
	holdActive bool

	now func() time.Time
	seq uint64
}

// NewExecutor creates an executor on a new virtual axis.
func NewExecutor(cfg axis.Config) (*Executor, error) {
	ax, err := axis.New(cfg)
	if err != nil {
		return nil, err
	}
	return &Executor{ax: ax, cfg: cfg, now: time.Now}, nil
}

// Axis exposes the underlying virtual axis for readout.
func (e *Executor) Axis() *axis.Axis { return e.ax }

// Enqueue adds a pending motion to the back of the queue and returns it.
func (e *Executor) Enqueue(target float64) *Move {
	e.mu.Lock()
	defer e.mu.Unlock()
	m := NewMove(e.nextIDLocked(), target, e.now())
	e.queue = append(e.queue, m)
	e.dispatchLocked()
	return m
}

// RetargetActive changes the goal of the active motion when one exists, or
// starts a new motion from the current position. The axis plans from its
// current position and velocity, so motion is continuous.
func (e *Executor) RetargetActive(target float64) (*Move, error) {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.ax.EmergencyStopped() {
		return nil, errors.New("action: emergency stop is active; call Reset first")
	}
	if e.active != nil && e.active.State() == Active {
		e.active.Target = target
		if err := e.ax.SetTarget(target); err != nil {
			return nil, err
		}
		return e.active, nil
	}
	// No active motion: behave like an immediate enqueue/start.
	m := NewMove(e.nextIDLocked(), target, e.now())
	e.queue = append([]*Move{m}, e.queue...)
	e.dispatchLocked()
	return m, nil
}

// CancelQueued cancels every not-yet-started motion. The active motion (if
// any) keeps running. It returns the canceled motions.
func (e *Executor) CancelQueued() []*Move {
	e.mu.Lock()
	defer e.mu.Unlock()
	canceled := make([]*Move, 0, len(e.queue))
	kept := e.queue[:0]
	for _, m := range e.queue {
		if m.Cancel() {
			canceled = append(canceled, m)
		} else {
			kept = append(kept, m)
		}
	}
	e.queue = kept
	return canceled
}

// StopPolicy selects what a soft stop does to the commanded work. The two
// layers mean different things by soft stop:
//
//   - axis layer: only decelerate to rest and drop the axis-level goal.
//   - executor layer: decide the fate of the active motion and the queue.
type StopPolicy int

const (
	// StopEndQueue aborts the active motion and cancels every queued motion.
	// This is the historical default of SoftStop.
	StopEndQueue StopPolicy = iota
	// StopKeepGoal brakes to rest but keeps the active motion and the queue:
	// once the axis is at rest the same goal is re-issued from the real
	// stopped position and the remaining queue continues afterwards.
	StopKeepGoal
)

// SoftStop aborts the active motion and cancels queued work, then brakes
// smoothly to rest (StopEndQueue). A new target may be commanded afterwards.
func (e *Executor) SoftStop() {
	e.SoftStopWithPolicy(StopEndQueue)
}

// SoftStopWithPolicy brakes smoothly to rest and applies the given policy to
// the active motion and the queue.
func (e *Executor) SoftStopWithPolicy(p StopPolicy) {
	e.mu.Lock()
	defer e.mu.Unlock()
	if p == StopKeepGoal {
		// Keep the active motion and queue; the axis drops its goal while
		// braking, so remember to re-issue it once at rest.
		e.holdActive = e.active != nil && e.active.State() == Active
		e.ax.SoftStop()
		return
	}
	e.holdActive = false
	e.abortActiveLocked(AbortedSoftStop)
	e.cancelAllLocked()
	e.ax.SoftStop()
}

// EmergencyStop aborts the active motion, cancels queued work and latches an
// emergency stop on the axis. New targets are rejected until Reset.
func (e *Executor) EmergencyStop() {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.holdActive = false
	e.abortActiveLocked(AbortedEmergencyStop)
	e.cancelAllLocked()
	e.ax.EmergencyStop()
}

// Reset clears a latched emergency stop once the axis is at rest.
func (e *Executor) Reset() error {
	e.mu.Lock()
	defer e.mu.Unlock()
	return e.ax.Reset()
}

// Active returns the currently executing motion, if any.
func (e *Executor) Active() *Move {
	e.mu.Lock()
	defer e.mu.Unlock()
	if e.active != nil && e.active.State() == Active {
		return e.active
	}
	return nil
}

// Pending returns a snapshot slice of motions still waiting in the queue.
func (e *Executor) Pending() []*Move {
	e.mu.Lock()
	defer e.mu.Unlock()
	out := make([]*Move, len(e.queue))
	copy(out, e.queue)
	return out
}

// Step advances the axis one sample and processes lifecycle transitions.
func (e *Executor) Step() {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.ax.Step()

	// A keep-goal soft stop re-issues the held goal once the axis is at rest.
	if e.holdActive && e.ax.Mode() == axis.Idle && !e.ax.EmergencyStopped() {
		e.holdActive = false
		if e.active != nil && e.active.State() == Active {
			if err := e.ax.SetTarget(e.active.Target); err != nil {
				e.abortActiveLocked(AbortedSoftStop)
			}
		}
	}

	// If the active move reached its target while Moving, complete it and start
	// the next pending motion.
	if e.active != nil && e.active.State() == Active {
		if e.ax.AtTarget() {
			e.ax.SettleIfAtTarget()
			e.active.Complete()
			e.active = nil
			e.dispatchLocked()
		}
	}
	e.dispatchLocked()
}

// dispatchLocked starts the next pending motion when the axis is free.
func (e *Executor) dispatchLocked() {
	if e.active != nil {
		return
	}
	if e.ax.EmergencyStopped() || e.ax.Mode() == axis.SoftStopping || e.ax.Mode() == axis.EmergencyStopping {
		return
	}
	for len(e.queue) > 0 {
		m := e.queue[0]
		e.queue = e.queue[1:]
		if m.State() != Pending {
			continue
		}
		if err := e.ax.SetTarget(m.Target); err != nil {
			// Re-queue head so the motion is not silently lost.
			e.queue = append([]*Move{m}, e.queue...)
			return
		}
		m.Activate(e.ax.Position())
		e.active = m
		return
	}
}

func (e *Executor) abortActiveLocked(reason AbortReason) {
	if e.active != nil {
		e.active.Abort(reason)
		e.active = nil
	}
}

func (e *Executor) cancelAllLocked() {
	for _, m := range e.queue {
		m.Cancel()
	}
	e.queue = nil
}

// wireMove is the serialized form of a Move including lifecycle internals.
type wireMove struct {
	ID        string      `json:"id"`
	Target    float64     `json:"target"`
	CreatedAt time.Time   `json:"created_at"`
	State     State       `json:"state"`
	Abort     AbortReason `json:"abort_reason"`
	StartPos  float64     `json:"start_pos"`
	Started   bool        `json:"started"`
}

// ExecutorSnapshot captures the axis mid-motion state plus the queue. It does
// not claim any motion has reached its target.
type ExecutorSnapshot struct {
	Config     axis.Config   `json:"config"`
	Axis       axis.Snapshot `json:"axis"`
	Active     *wireMove     `json:"active"`
	Queue      []*wireMove   `json:"queue"`
	HoldActive bool          `json:"hold_active"`
}

// Save serializes the full executor state.
func (e *Executor) Save() ([]byte, error) {
	e.mu.Lock()
	defer e.mu.Unlock()
	snap := ExecutorSnapshot{
		Config:     e.cfg,
		Axis:       e.ax.Snapshot(),
		Queue:      make([]*wireMove, 0, len(e.queue)),
		HoldActive: e.holdActive,
	}
	if e.active != nil {
		snap.Active = toWire(e.active)
	}
	for _, m := range e.queue {
		snap.Queue = append(snap.Queue, toWire(m))
	}
	return json.Marshal(snap)
}

// RestoreExecutor decodes a snapshot into a ready-to-run executor. An active
// motion resumes with the axis at its saved real position and velocity rather
// than being marked complete.
func RestoreExecutor(data []byte) (*Executor, error) {
	var snap ExecutorSnapshot
	if err := json.Unmarshal(data, &snap); err != nil {
		return nil, err
	}
	ax, err := axis.New(snap.Config)
	if err != nil {
		return nil, err
	}
	ax.Restore(snap.Axis)
	e := &Executor{ax: ax, cfg: snap.Config, now: time.Now, holdActive: snap.HoldActive}
	if snap.Active != nil {
		e.active = fromWire(snap.Active)
		if e.active.State() == Active && !snap.Axis.HasGoal && !snap.HoldActive {
			return nil, errors.New("action: active motion restored without an axis goal")
		}
	}
	for _, w := range snap.Queue {
		e.queue = append(e.queue, fromWire(w))
	}
	return e, nil
}

func toWire(m *Move) *wireMove {
	sp, started := m.StartPos()
	return &wireMove{
		ID:        m.ID,
		Target:    m.Target,
		CreatedAt: m.CreatedAt,
		State:     m.State(),
		Abort:     m.AbortReason(),
		StartPos:  sp,
		Started:   started,
	}
}

func (e *Executor) nextIDLocked() string {
	e.seq++
	return fmt.Sprintf("move-%d-%d", e.now().UnixNano(), e.seq)
}

func fromWire(w *wireMove) *Move {
	m := &Move{
		ID:        w.ID,
		Target:    w.Target,
		CreatedAt: w.CreatedAt,
		state:     w.State,
		abort:     w.Abort,
		startPos:  w.StartPos,
		started:   w.Started,
	}
	return m
}
