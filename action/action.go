// Package action defines the lifecycle of a single commanded motion.
//
// A motion follows one lifecycle that is identical whether it runs on one
// virtual axis today or on coordinated multi-axis groups later:
//
//	Pending -> Active -> Completed
//	                    \-> Canceled   (removed before or without starting)
//	                    \-> Aborted    (interrupted by a stop or retarget)
//
// The action package is transport-only; it knows nothing about hardware.
package action

import "time"

// State is the lifecycle state of a motion.
type State int

const (
	// Pending motions are queued but have not started executing.
	Pending State = iota
	// Active motions are currently being executed by the axis.
	Active
	// Completed motions finished at their commanded target.
	Completed
	// Canceled motions were removed from the pending queue.
	Canceled
	// Aborted motions started but were interrupted (stop or replaced target).
	Aborted
)

// IsTerminal reports whether the lifecycle has ended.
func (s State) IsTerminal() bool {
	return s == Completed || s == Canceled || s == Aborted
}

// String names the state for diagnostics.
func (s State) String() string {
	switch s {
	case Pending:
		return "Pending"
	case Active:
		return "Active"
	case Completed:
		return "Completed"
	case Canceled:
		return "Canceled"
	case Aborted:
		return "Aborted"
	default:
		return "Unknown"
	}
}

// AbortReason explains why an active motion was interrupted. The zero value
// means the motion was not aborted.
type AbortReason int

const (
	NotAborted AbortReason = iota
	AbortedSoftStop
	AbortedEmergencyStop
	AbortedReplaced
)

// Move is a single point-to-point motion toward a target position.
//
// It is intentionally a single-axis description for the first round. The same
// lifecycle and scheduling can later carry a multi-axis target without
// changing state transitions.
type Move struct {
	ID        string
	Target    float64
	CreatedAt time.Time

	state    State
	abort    AbortReason
	startPos float64
	started  bool
}

// NewMove creates a pending motion.
func NewMove(id string, target float64, now time.Time) *Move {
	return &Move{ID: id, Target: target, CreatedAt: now, state: Pending}
}

// State returns the lifecycle state.
func (m *Move) State() State { return m.state }

// AbortReason returns the interruption reason for an aborted motion.
func (m *Move) AbortReason() AbortReason { return m.abort }

// StartPos records the actual position the motion began from.
func (m *Move) StartPos() (float64, bool) { return m.startPos, m.started }

// Activate marks a pending motion active, recording where execution began.
func (m *Move) Activate(fromPos float64) bool {
	if m.state != Pending {
		return false
	}
	m.state = Active
	m.startPos = fromPos
	m.started = true
	return true
}

// Complete marks an active motion finished at its target.
func (m *Move) Complete() bool {
	if m.state != Active {
		return false
	}
	m.state = Completed
	return true
}

// Cancel removes a pending motion without starting it.
func (m *Move) Cancel() bool {
	if m.state != Pending {
		return false
	}
	m.state = Canceled
	return true
}

// Abort interrupts an active motion with the given reason.
func (m *Move) Abort(reason AbortReason) bool {
	if m.state != Active || reason == NotAborted {
		return false
	}
	m.state = Aborted
	m.abort = reason
	return true
}
