package action

import (
	"encoding/json"
	"errors"
	"time"

	"motion/axis"
)

// wireCellGroup serializes one registered group, keyed by id.
type wireCellGroup struct {
	ID       string              `json:"id"`
	Reaction ConstrainedReaction `json:"reaction"`
	Names    []string            `json:"names"`
	Active   *wireGroupMove      `json:"active"`
	Queue    []*wireGroupMove    `json:"queue"`
	Holding  bool                `json:"holding"`
	Seq      uint64              `json:"seq"`
}

// CellSnapshot captures every shared axis mid-motion, ownership and pending
// handoffs, gates, the terminal event log and the group order/queues.
type CellSnapshot struct {
	Axes   map[string]axisSnapshotWire `json:"axes"`
	Order  []string                    `json:"order"`
	Gates  map[string]bool             `json:"gates"`
	Owner  map[string]string           `json:"owner"`  // axis -> move ID
	Groups []*wireCellGroup            `json:"groups"` // registration order
	Events []Event                     `json:"events"`
}

type axisSnapshotWire = struct {
	Config axis.Config   `json:"config"`
	State  axis.Snapshot `json:"state"`
}

// Save serializes the complete cell state, including mid-braking axes,
// waiting/stopping moves and unpublished handoffs.
func (c *Cell) Save() ([]byte, error) {
	for _, id := range c.gorder {
		c.groups[id].mu.Lock()
	}
	c.mu.Lock()
	defer func() {
		c.mu.Unlock()
		for i := len(c.gorder) - 1; i >= 0; i-- {
			c.groups[c.gorder[i]].mu.Unlock()
		}
	}()

	snap := CellSnapshot{
		Axes:   map[string]axisSnapshotWire{},
		Gates:  map[string]bool{},
		Owner:  map[string]string{},
		Events: append([]Event(nil), c.events...),
	}
	snap.Order = append(snap.Order, c.order...)
	for _, name := range c.order {
		ca := c.axes[name]
		snap.Axes[name] = axisSnapshotWire{Config: ca.cfg, State: ca.ax.Snapshot()}
	}
	for k, v := range c.gates {
		snap.Gates[k] = v
	}
	for name, m := range c.owner {
		snap.Owner[name] = m.ID
	}
	for _, id := range c.gorder {
		g := c.groups[id]
		wg := &wireCellGroup{
			ID:       g.id,
			Reaction: g.reaction,
			Names:    append([]string(nil), g.names...),
			Holding:  g.holding,
			Seq:      g.seq,
		}
		if g.active != nil {
			wg.Active = toWireGroup(g.active)
		}
		for _, m := range g.queue {
			wg.Queue = append(wg.Queue, toWireGroup(m))
		}
		snap.Groups = append(snap.Groups, wg)
	}
	return json.Marshal(snap)
}

// RestoreCell rebuilds a cell from a snapshot. Moves resume from the saved
// real positions: handoff conditions are re-verified against those positions,
// and stopping moves keep their axes until the brakes finish.
func RestoreCell(data []byte) (*Cell, error) {
	var snap CellSnapshot
	if err := json.Unmarshal(data, &snap); err != nil {
		return nil, err
	}
	c := NewCell()
	c.now = time.Now
	moves := map[string]*GroupMove{}

	for _, name := range snap.Order {
		w, ok := snap.Axes[name]
		if !ok {
			return nil, errors.New("action: snapshot missing axis " + name)
		}
		if err := c.AddAxis(name, w.Config); err != nil {
			return nil, err
		}
		c.axes[name].ax.Restore(w.State)
	}
	for k, v := range snap.Gates {
		c.gates[k] = v
	}
	for _, wg := range snap.Groups {
		g, err := c.NewGroup(wg.ID, wg.Names, wg.Reaction)
		if err != nil {
			return nil, err
		}
		g.holding = wg.Holding
		g.seq = wg.Seq
		if wg.Active != nil {
			m := fromWireGroup(wg.Active)
			g.active = m
			moves[m.ID] = m
		}
		for _, w := range wg.Queue {
			m := fromWireGroup(w)
			g.queue = append(g.queue, m)
			moves[m.ID] = m
		}
	}
	c.events = append(c.events, snap.Events...)
	c.mu.Lock()
	defer c.mu.Unlock()
	for name, id := range snap.Owner {
		m := moves[id]
		if m == nil {
			return nil, errors.New("action: snapshot owner move not found: " + id)
		}
		c.owner[name] = m
	}
	return c, nil
}
