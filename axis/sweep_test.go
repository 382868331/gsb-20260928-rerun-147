package axis

import (
	"math"
	"testing"
)

func TestParameterSweep(t *testing.T) {
	configs := []Config{
		{1, 1, 0.01}, {10, 20, 0.001}, {5, 50, 0.002},
		{3.5, 7.25, 0.0005}, {100, 1000, 0.001}, {0.1, 0.5, 0.01},
	}
	targets := []float64{0, 0.0001, 0.001, 0.01, 0.05, 0.37, 1, -1.7, -13.5, 42.0}
	for _, c := range configs {
		for _, target := range targets {
			ax, err := New(c)
			if err != nil {
				t.Fatal(err)
			}
			if err := ax.SetTarget(target); err != nil {
				t.Fatal(err)
			}
			prevP, prevV := 0.0, 0.0
			for i := 0; i < 5_000_000; i++ {
				ax.Step()
				p, v := ax.Position(), ax.Velocity()
				if math.Abs(v) > c.MaxSpeed+1e-9 {
					t.Fatalf("v limit cfg=%+v tgt=%v v=%v", c, target, v)
				}
				if dv := math.Abs(v - prevV); dv > c.MaxAccel*c.Step+1e-9 {
					t.Fatalf("a limit cfg=%+v tgt=%v dv=%v", c, target, dv)
				}
				if dp := math.Abs(p - prevP); dp > c.MaxSpeed*c.Step+c.MaxAccel*c.Step*c.Step+1e-9 {
					t.Fatalf("step too far cfg=%+v tgt=%v dp=%v", c, target, dp)
				}
				prevP, prevV = p, v
				if ax.AtTarget() {
					ax.SettleIfAtTarget()
					break
				}
			}
			if math.Abs(ax.Position()-target) > 1e-9 || ax.Velocity() != 0 {
				t.Fatalf("settle cfg=%+v tgt=%v p=%v v=%v", c, target, ax.Position(), ax.Velocity())
			}
		}
	}
}
