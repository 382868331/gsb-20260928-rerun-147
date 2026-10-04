package action

import (
	"errors"
	"fmt"
	"sync"
	"time"

	"motion/axis"
)

// AxisCondition is one explicit handoff predicate on a shared axis. The
// successor acquires the axis only when the axis is at rest and its current
// virtual position lies inside the requested window.
type AxisCondition struct {
	Axis       string  `json:"axis"`
	AtLeast    float64 `json:"at_least"`
	AtLeastSet bool    `json:"at_least_set"`
	AtMost     float64 `json:"at_most"`
	AtMostSet  bool    `json:"at_most_set"`
}

// HandoffConditions are the explicit gates a move waits for: per-axis position
// predicates (re-evaluated against the CURRENT virtual position on every
// dispatch retry) and named boolean gates toggled by the operator.
type HandoffConditions struct {
	Axes  []AxisCondition `json:"axes"`
	Gates []string        `json:"gates"`
}

// Event is a terminal lifecycle event attributed to one exact group move. A
// completion arriving late (after a successor already owns the axis) is still
// recorded against the old move, never the new owner.
type Event struct {
	GroupID string      `json:"group_id"`
	MoveID  string      `json:"move_id"`
	State   State       `json:"state"`
	Abort   AbortReason `json:"abort_reason"`
	At      time.Time   `json:"at"`
}

type offeredHandoff struct {
	fromID   string
	pos      float64
	atRest   bool
	handedAt time.Time
}

type cellAxis struct {
	cfg axis.Config
	ax  *axis.Axis
}

// Cell is the shared virtual axis registry. Multiple groups address subsets of
// its named axes; an axis is owned by at most one active group move at a time,
// and ownership changes only through explicit, re-verified handoff conditions.
type Cell struct {
	mu sync.Mutex

	axes   map[string]*cellAxis
	order  []string
	groups map[string]*Group
	gorder []string

	owner   map[string]*GroupMove
	offered map[string]offeredHandoff
	gates   map[string]bool

	events []Event
	now    func() time.Time
}

// NewCell creates an empty shared-axis cell.
func NewCell() *Cell {
	return &Cell{
		axes:    map[string]*cellAxis{},
		groups:  map[string]*Group{},
		owner:   map[string]*GroupMove{},
		offered: map[string]offeredHandoff{},
		gates:   map[string]bool{},
		now:     time.Now,
	}
}

// AddAxis registers a named virtual axis.
func (c *Cell) AddAxis(name string, cfg axis.Config) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	if name == "" {
		return errors.New("action: axis name must not be empty")
	}
	if _, ok := c.axes[name]; ok {
		return fmt.Errorf("action: axis %q already registered", name)
	}
	ax, err := axis.New(cfg)
	if err != nil {
		return err
	}
	c.axes[name] = &cellAxis{cfg: cfg, ax: ax}
	c.order = append(c.order, name)
	return nil
}

// AxisNames returns axis names in registration order.
func (c *Cell) AxisNames() []string {
	c.mu.Lock()
	defer c.mu.Unlock()
	return append([]string(nil), c.order...)
}

// Axis returns a named axis for readout.
func (c *Cell) Axis(name string) *axis.Axis {
	c.mu.Lock()
	defer c.mu.Unlock()
	if ca, ok := c.axes[name]; ok {
		return ca.ax
	}
	return nil
}

// Owner returns the move currently owning a named axis.
func (c *Cell) Owner(name string) *GroupMove {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.owner[name]
}

// SetGate toggles a named handoff gate.
func (c *Cell) SetGate(name string, open bool) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.gates[name] = open
}

// Gate reports a gate state (closed when unset).
func (c *Cell) Gate(name string) bool {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.gates[name]
}

// Events returns a copy of the terminal event log.
func (c *Cell) Events() []Event {
	c.mu.Lock()
	defer c.mu.Unlock()
	return append([]Event(nil), c.events...)
}

// NewGroup registers a group addressing the named axes (one target per name
// when enqueueing). Groups on disjoint axis sets run concurrently; sharing an
// axis forces handoff ordering.
func (c *Cell) NewGroup(id string, axisNames []string, reaction ConstrainedReaction) (*Group, error) {
	c.mu.Lock()
	defer c.mu.Unlock()
	if id == "" {
		return nil, errors.New("action: group id must not be empty")
	}
	if _, ok := c.groups[id]; ok {
		return nil, fmt.Errorf("action: group %q already registered", id)
	}
	if len(axisNames) == 0 {
		return nil, errors.New("action: group needs at least one axis")
	}
	sharedAxes := make([]*axis.Axis, len(axisNames))
	cfgs := make([]axis.Config, len(axisNames))
	for i, name := range axisNames {
		ca, ok := c.axes[name]
		if !ok {
			return nil, fmt.Errorf("action: unknown axis %q", name)
		}
		sharedAxes[i] = ca.ax
		cfgs[i] = ca.cfg
	}
	g := &Group{
		id:       id,
		names:    append([]string(nil), axisNames...),
		cell:     c,
		bound:    true,
		axes:     sharedAxes,
		cfgs:     cfgs,
		reaction: reaction,
		now:      c.now,
	}
	c.groups[id] = g
	c.gorder = append(c.gorder, id)
	return g, nil
}

// Group returns a registered group by id.
func (c *Cell) Group(id string) *Group {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.groups[id]
}

// Step advances every shared axis once and runs one coordination pass per
// group. Groups on independent axes keep moving while a neighbor waits/brakes.
func (c *Cell) Step() {
	for _, id := range c.gorder {
		c.groups[id].mu.Lock()
	}
	c.mu.Lock()
	for _, name := range c.order {
		c.axes[name].ax.Step()
	}
	for _, id := range c.gorder {
		c.groups[id].stepLocked(true)
	}
	c.mu.Unlock()
	for i := len(c.gorder) - 1; i >= 0; i-- {
		c.groups[c.gorder[i]].mu.Unlock()
	}
}

// EmergencyAxisStop latches an emergency stop on one named axis. The owning
// move (if any) enters a safe stop and keeps the axis until it settles and is
// reset; groups on other axes continue unaffected.
func (c *Cell) EmergencyAxisStop(name string) error {
	c.mu.Lock()
	if _, ok := c.axes[name]; !ok {
		c.mu.Unlock()
		return fmt.Errorf("action: unknown axis %q", name)
	}
	m := c.owner[name]
	c.axes[name].ax.EmergencyStop()
	c.mu.Unlock()
	if m == nil {
		return nil
	}
	c.mu.Lock()
	var owner *Group
	for _, g := range c.groups {
		if g.active == m {
			owner = g
			break
		}
	}
	c.mu.Unlock()
	if owner != nil {
		owner.mu.Lock()
		owner.beginStopLocked(AbortedEmergencyStop)
		owner.mu.Unlock()
	}
	return nil
}

// resetGroup clears latched emergency stops on the group's axes once each is at
// rest; a still-braking axis is never released.
func (c *Cell) resetGroup(g *Group) error {
	g.mu.Lock()
	defer g.mu.Unlock()
	c.mu.Lock()
	defer c.mu.Unlock()
	for i, ax := range g.axes {
		if ax.EmergencyStopped() && ax.Velocity() != 0 {
			return fmt.Errorf("action: axis %q still braking", g.names[i])
		}
		if err := ax.Reset(); err != nil {
			return err
		}
	}
	return nil
}

// dispatchGroupLocked attempts to start g's head move. Caller holds g.mu but
// not c.mu.
func (c *Cell) dispatchGroupLocked(g *Group) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.dispatchGroupHeld(g)
}

// dispatchGroupHeld is dispatchGroupLocked with c.mu already held.
func (c *Cell) dispatchGroupHeld(g *Group) {
	if g.active != nil || g.holding || len(g.queue) == 0 {
		return
	}
	m := g.queue[0]
	if m.state != Pending && m.state != Waiting {
		g.queue = g.queue[1:]
		return
	}
	for _, name := range m.axes {
		if c.owner[name] != nil {
			m.state = Waiting
			return
		}
		ca := c.axes[name]
		if ca.ax.EmergencyStopped() ||
			ca.ax.Mode() == axis.SoftStopping ||
			ca.ax.Mode() == axis.EmergencyStopping {
			m.state = Waiting
			return
		}
	}
	if !c.conditionsMetLocked(m) {
		m.state = Waiting
		return
	}

	// Acquire every axis atomically and command targets from real positions.
	for _, name := range m.axes {
		if err := c.axes[name].ax.SetTarget(m.Targets[axisIndex(g, name)]); err != nil {
			c.rollbackGoalsLocked(m)
			m.state = Waiting
			return
		}
	}
	g.queue = g.queue[1:]
	for _, name := range m.axes {
		c.owner[name] = m
		m.acquired = append(m.acquired, name)
		if off, ok := c.offered[name]; ok {
			m.handoffs = append(m.handoffs, HandoffRecord{
				Axis:     name,
				FromID:   off.fromID,
				ToID:     m.ID,
				At:       off.pos,
				AtRest:   off.atRest,
				HandedAt: off.handedAt,
			})
			delete(c.offered, name)
		}
	}
	m.state = Active
	g.active = m
}

func axisIndex(g *Group, name string) int {
	for i, n := range g.names {
		if n == name {
			return i
		}
	}
	return -1
}

func (c *Cell) conditionsMetLocked(m *GroupMove) bool {
	for _, gate := range m.gates {
		if !c.gates[gate] {
			return false
		}
	}
	for _, cond := range m.axisConds {
		ca := c.axes[cond.Axis]
		if ca.ax.Velocity() != 0 {
			return false
		}
		pos := ca.ax.Position()
		if cond.AtLeastSet && pos < cond.AtLeast {
			return false
		}
		if cond.AtMostSet && pos > cond.AtMost {
			return false
		}
	}
	return true
}

func (c *Cell) rollbackGoalsLocked(m *GroupMove) {
	for _, name := range m.axes {
		if c.owner[name] == nil {
			c.axes[name].ax.ClearTarget()
		}
	}
}

// tickMoveLocked re-evaluates a waiting move every sample so it starts the
// instant its axes are free and conditions hold at the CURRENT positions.
func (c *Cell) tickMoveLocked(g *Group, m *GroupMove) {
	if m.state != Waiting {
		return
	}
	c.dispatchGroupHeld(g)
}

// releaseMoveLocked returns m's axes to the pool and publishes a handoff per
// axis. Caller holds g.mu but not c.mu.
func (c *Cell) releaseMoveLocked(g *Group, m *GroupMove) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.releaseMoveHeld(g, m)
}

// releaseMoveHeld is releaseMoveLocked with c.mu already held.
func (c *Cell) releaseMoveHeld(g *Group, m *GroupMove) {
	for _, name := range m.acquired {
		if c.owner[name] != m {
			continue
		}
		ca := c.axes[name]
		c.offered[name] = offeredHandoff{
			fromID:   m.ID,
			pos:      ca.ax.Position(),
			atRest:   ca.ax.Velocity() == 0,
			handedAt: c.now(),
		}
		delete(c.owner, name)
	}
	m.acquired = nil
	c.events = append(c.events, Event{
		GroupID: g.id,
		MoveID:  m.ID,
		State:   m.state,
		Abort:   m.abort,
		At:      c.now(),
	})
}
