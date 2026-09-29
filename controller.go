package motion

import (
	"fmt"
	"sync"
)

type Mode string

const (
	ModeIdle          Mode = "idle"
	ModeMoving        Mode = "moving"
	ModeSoftStopping  Mode = "soft_stopping"
	ModeSoftStopped   Mode = "soft_stopped"
	ModeEmergencyStop Mode = "emergency_stopped"
)

type ActionStatus string

const (
	StatusPending     ActionStatus = "pending"
	StatusRunning     ActionStatus = "running"
	StatusCompleted   ActionStatus = "completed"
	StatusCanceled    ActionStatus = "canceled"
	StatusInterrupted ActionStatus = "interrupted"
	StatusStopped     ActionStatus = "stopped"
)

type Limits struct {
	MaxVelocity     float64
	MaxAcceleration float64
}

type MoveAction struct {
	id     string
	target float64
	status ActionStatus
}

func (a *MoveAction) ID() string           { return a.id }
func (a *MoveAction) Target() float64      { return a.target }
func (a *MoveAction) Status() ActionStatus { return a.status }

type activePlan struct {
	action  *MoveAction
	plan    *Plan
	elapsed float64
	slotted bool
}

type Controller struct {
	mu       sync.Mutex
	limits   Limits
	position float64
	velocity float64
	mode     Mode
	queue    []*MoveAction
	active   *activePlan
	actions  map[string]*MoveAction
	nextID   int
}

func NewController(limits Limits, initialPosition, initialVelocity float64) (*Controller, error) {
	if limits.MaxVelocity <= 0 || limits.MaxAcceleration <= 0 {
		return nil, fmt.Errorf("velocity and acceleration limits must be positive")
	}
	return &Controller{
		limits:   limits,
		position: initialPosition,
		velocity: initialVelocity,
		mode:     ModeIdle,
		actions:  make(map[string]*MoveAction),
	}, nil
}

func (c *Controller) Position() float64 {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.position
}

func (c *Controller) Velocity() float64 {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.velocity
}

func (c *Controller) Mode() Mode {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.mode
}

func (c *Controller) EnqueueMove(target float64) (*MoveAction, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.mode == ModeEmergencyStop || c.mode == ModeSoftStopping || c.mode == ModeSoftStopped {
		return nil, fmt.Errorf("controller is not accepting moves in mode %s", c.mode)
	}
	action := c.addAction(target)
	c.queue = append(c.queue, action)
	c.startNextIfIdle()
	return action, nil
}

// SetTarget starts a fresh command from the current measured position and
// velocity. It interrupts the active action and replaces queued work.
func (c *Controller) SetTarget(target float64) (*MoveAction, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.mode == ModeEmergencyStop || c.mode == ModeSoftStopping || c.mode == ModeSoftStopped {
		return nil, fmt.Errorf("controller cannot retarget in mode %s", c.mode)
	}
	if c.active != nil {
		c.active.action.status = StatusInterrupted
		c.cancelPendingLocked()
		c.active = nil
	} else {
		c.cancelPendingLocked()
	}
	c.queue = nil
	action := c.addAction(target)
	c.queue = append(c.queue, action)
	c.mode = ModeIdle
	c.startNextIfIdle()
	return action, nil
}

func (c *Controller) CancelQueued() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	n := len(c.queue)
	c.cancelPendingLocked()
	c.queue = nil
	return n
}

func (c *Controller) SoftStop() {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.mode == ModeEmergencyStop || c.mode == ModeSoftStopping || c.mode == ModeSoftStopped {
		return
	}
	if c.active == nil || abs(c.velocity) <= epsilon {
		c.velocity = 0
		c.mode = ModeSoftStopped
		if c.active != nil {
			c.active.action.status = StatusStopped
			c.active = nil
		}
		return
	}
	plan := brakePlan(c.position, c.velocity, c.limits.MaxAcceleration)
	c.active = &activePlan{action: c.active.action, plan: plan, slotted: true}
	c.mode = ModeSoftStopping
}

func (c *Controller) EmergencyStop() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.velocity = 0
	if c.active != nil {
		c.active.action.status = StatusStopped
	}
	c.cancelPendingLocked()
	c.queue = nil
	c.active = nil
	c.mode = ModeEmergencyStop
}

func (c *Controller) Resume() error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if c.mode != ModeSoftStopped {
		return fmt.Errorf("resume is only valid after a soft stop")
	}
	c.velocity = 0
	c.mode = ModeIdle
	c.startNextIfIdle()
	return nil
}

func (c *Controller) Reset(position float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.position = position
	c.velocity = 0
	c.mode = ModeIdle
	c.queue = nil
	c.active = nil
	c.actions = make(map[string]*MoveAction)
	c.nextID = 0
}

func (c *Controller) Tick(dt float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if dt < 0 {
		panic("negative tick duration")
	}
	if c.active == nil || dt == 0 {
		return
	}
	c.active.elapsed += dt
	x, v, _, done := c.active.plan.At(c.active.elapsed)
	c.position, c.velocity = x, v
	if !done {
		return
	}

	action := c.active.action
	if c.active.slotted {
		action.status = StatusStopped
		c.active = nil
		c.mode = ModeSoftStopped
		return
	}
	action.status = StatusCompleted
	c.active = nil
	c.mode = ModeIdle
	c.velocity = 0
	c.startNextIfIdle()
}

func (c *Controller) addAction(target float64) *MoveAction {
	c.nextID++
	action := &MoveAction{id: fmt.Sprintf("move-%d", c.nextID), target: target, status: StatusPending}
	c.actions[action.id] = action
	return action
}

func (c *Controller) cancelPendingLocked() {
	for _, action := range c.queue {
		action.status = StatusCanceled
	}
}

func (c *Controller) startNextIfIdle() {
	if c.active != nil || c.mode == ModeSoftStopped || c.mode == ModeEmergencyStop {
		return
	}
	for len(c.queue) > 0 && c.queue[0].status != StatusPending {
		c.queue = c.queue[1:]
	}
	if len(c.queue) == 0 {
		c.mode = ModeIdle
		return
	}
	action := c.queue[0]
	c.queue = c.queue[1:]
	action.status = StatusRunning
	plan := NewMovePlan(c.position, c.velocity, action.target, c.limits.MaxVelocity, c.limits.MaxAcceleration)
	c.active = &activePlan{action: action, plan: plan}
	c.mode = ModeMoving
}

func abs(v float64) float64 {
	if v < 0 {
		return -v
	}
	return v
}
