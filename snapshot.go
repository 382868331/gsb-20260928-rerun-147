package motion

import (
	"encoding/json"
	"fmt"
)

// ActionState is the persisted lifecycle state of one queued or active move.
type ActionState struct {
	ID     string       `json:"id"`
	Target float64      `json:"target"`
	Status ActionStatus `json:"status"`
}

// ActiveState is the persisted state of the currently executing trajectory.
type ActiveState struct {
	ActionID string       `json:"action_id"`
	Elapsed  float64      `json:"elapsed"`
	Slotted  bool         `json:"slotted"`
	Plan     ProfileState `json:"plan"`
}

// State is a portable snapshot. An unfinished active action remains running or
// soft-stopping and keeps its elapsed trajectory; it is never marked complete.
type State struct {
	Limits   Limits        `json:"limits"`
	Position float64       `json:"position"`
	Velocity float64       `json:"velocity"`
	Mode     Mode          `json:"mode"`
	NextID   int           `json:"next_id"`
	Queue    []string      `json:"queue"`
	Actions  []ActionState `json:"actions"`
	Active   *ActiveState  `json:"active,omitempty"`
}

func (c *Controller) Snapshot() State {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.snapshotLocked()
}

func (c *Controller) snapshotLocked() State {

	state := State{
		Limits:   c.limits,
		Position: c.position,
		Velocity: c.velocity,
		Mode:     c.mode,
		NextID:   c.nextID,
		Queue:    make([]string, 0, len(c.queue)),
		Actions:  make([]ActionState, 0, len(c.actions)),
	}
	for _, action := range c.queue {
		state.Queue = append(state.Queue, action.id)
	}
	for _, action := range c.actions {
		state.Actions = append(state.Actions, ActionState{ID: action.id, Target: action.target, Status: action.status})
	}
	if c.active != nil {
		state.Active = &ActiveState{
			ActionID: c.active.action.id,
			Elapsed:  c.active.elapsed,
			Slotted:  c.active.slotted,
			Plan:     c.active.plan.State(),
		}
	}
	return state
}

func RestoreController(state State) (*Controller, error) {
	c := &Controller{actions: make(map[string]*MoveAction)}
	if err := c.Restore(state); err != nil {
		return nil, err
	}
	return c, nil
}

func (c *Controller) Restore(state State) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	if state.Limits.MaxVelocity <= 0 || state.Limits.MaxAcceleration <= 0 {
		return fmt.Errorf("invalid motion limits in snapshot")
	}
	actions := make(map[string]*MoveAction, len(state.Actions))
	for _, saved := range state.Actions {
		actions[saved.ID] = &MoveAction{id: saved.ID, target: saved.Target, status: saved.Status}
	}
	queue := make([]*MoveAction, 0, len(state.Queue))
	for _, id := range state.Queue {
		action, ok := actions[id]
		if !ok {
			return fmt.Errorf("queued action %q is missing from snapshot", id)
		}
		queue = append(queue, action)
	}
	var active *activePlan
	if state.Active != nil {
		action, ok := actions[state.Active.ActionID]
		if !ok {
			return fmt.Errorf("active action %q is missing from snapshot", state.Active.ActionID)
		}
		plan := planFromState(state.Active.Plan)
		x, v, _, done := plan.At(state.Active.Elapsed)
		if abs(x-state.Position) > 1e-9 || abs(v-state.Velocity) > 1e-9 {
			return fmt.Errorf("snapshot trajectory does not match current position and velocity")
		}
		if done && (state.Mode == ModeMoving || state.Mode == ModeSoftStopping) {
			return fmt.Errorf("snapshot marks an unfinished action after its trajectory ended")
		}
		active = &activePlan{action: action, plan: plan, elapsed: state.Active.Elapsed, slotted: state.Active.Slotted}
	}
	c.limits = state.Limits
	c.position = state.Position
	c.velocity = state.Velocity
	c.mode = state.Mode
	c.nextID = state.NextID
	c.actions = actions
	c.queue = queue
	c.active = active
	return nil
}

func (c *Controller) MarshalJSON() ([]byte, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	return json.Marshal(c.snapshotLocked())
}
