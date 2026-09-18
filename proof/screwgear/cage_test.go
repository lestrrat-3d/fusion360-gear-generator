package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// The frame is two collars, one threaded on each gear, fused where they meet.
// It is the frame Segerman's model uses, and the shape is not decoration: the
// two rings are what the mechanism has instead of bearings.
//
// A collar is a disc standing across its gear's axis with an opening cut to the
// ribbon's own cross-section. The opening is what does the work. A round hole
// would let its ribbon turn freely as it slid, leaving the mechanism three
// degrees of freedom instead of one; an opening shaped like the ribbon forces
// the ribbon to turn as it advances, the way a twisted-bar screwdriver does.
// The opening is a twisted channel rather than a straight one, because the
// ribbon turns while it is inside the collar — over the default 1.5 mm of depth
// it turns 13.5 degrees, which is far more than a straight hole would pass.

// inCollarOpening answers whether a point lies in the channel cut through a
// gear's collar, which is that gear's own ribbon plus the clearance.
func inCollarOpening(g Gear, pt r3.Vec) bool {
	u, v, _ := g.local(pt)
	return math.Abs(u) <= g.P.Width/2+g.P.Clearance && math.Abs(v) <= g.P.Thickness/2+g.P.Clearance
}

// inCollar answers whether a point is inside a collar's material: within its
// rim, within its depth, and not in the opening.
func inCollar(g Gear, pt r3.Vec) bool {
	d := pt.Sub(collarCentre(g))
	along := d.Dot(g.Ez)
	if math.Abs(along) > g.P.CollarDepth/2 {
		return false
	}
	if d.Sub(g.Ez.Scale(along)).Len() > g.P.CollarOuter {
		return false
	}
	return !inCollarOpening(g, pt)
}

// The two collars have to meet, or the frame is two loose rings and holds
// nothing. This is the measurement behind that: how far their rims overlap.
func TestCollarsMeetEachOther(t *testing.T) {
	ga, gb := defaultPair()
	gap := collarCentre(ga).Sub(collarCentre(gb)).Len()
	overlap := 2*ga.P.CollarOuter - gap

	if overlap <= 0 {
		t.Fatalf("the collar centres stand %.2f mm apart and each rim reaches %.2f mm, so the two "+
			"rings never touch and the frame is not one body", gap, ga.P.CollarOuter)
	}
	if overlap < ga.P.CollarDepth {
		t.Errorf("the rims overlap by only %.2f mm, which is less than a collar is thick", overlap)
	}
	t.Logf("collar centres %.2f mm apart, rims overlapping %.2f mm", gap, overlap)
}

// A collar must not foul the gear it does not hold.
func TestCollarsClearTheOtherGear(t *testing.T) {
	ga, gb := defaultPair()
	half := ga.P.Length() / 2

	for _, pairing := range []struct {
		collar, ribbon Gear
		label          string
	}{{ga, gb, "A's collar against gear B"}, {gb, ga, "B's collar against gear A"}} {
		g := pairing.ribbon
		w, th := g.P.Width/2, g.P.Thickness/2
		for s := -half; s <= half; s += 0.02 {
			e := g.edge(s)
			for i := range 5 {
				v := -th + 2*th*float64(i)/4
				for _, u := range []float64{e, -w, (e - w) / 2} {
					if inCollar(pairing.collar, g.world(u, v, s)) {
						t.Fatalf("%s: the ribbon meets it at station %.2f, (u,v)=(%.2f,%.2f)",
							pairing.label, s, u, v)
					}
				}
			}
		}
	}
}

// The collar's own gear goes through it, at every position that gear takes.
//
// One static pass settles every position. The opening is cut to the ribbon's
// blank — the full-width rectangle, before the teeth are taken out of one edge
// — and that blank is invariant under the gear's own screw motion, so a ribbon
// that clears the collar at one phase clears it at all of them. The teeth
// cannot change that either: they are cut INTO the edge, so the material only
// retreats from the opening's face.
func TestEachGearPassesThroughItsCollar(t *testing.T) {
	ga, gb := defaultPair()
	half := ga.P.Length() / 2

	for _, g := range []Gear{ga, gb} {
		w, th := g.P.Width/2, g.P.Thickness/2
		for s := -half; s <= half; s += 0.02 {
			e := g.edge(s)
			for i := range 5 {
				v := -th + 2*th*float64(i)/4
				for _, u := range []float64{e, -w, (e - w) / 2} {
					if inCollar(g, g.world(u, v, s)) {
						t.Fatalf("a gear meets its own collar at station %.2f, (u,v)=(%.2f,%.2f)",
							s, u, v)
					}
				}
			}
		}
	}
}

// This is the collar's whole reason for being: it admits the screw motion and
// nothing else. The test turns a gear out of step with its own advance and
// finds the angle at which it jams in its collar.
//
// The slack that is left is the clearance divided by the collar's reach, and it
// is small: a gear cannot turn far without advancing to match. A frame of round
// holes would report no jam at any angle, which is the case this rules out.
func TestCollarAdmitsOnlyTheScrewMotion(t *testing.T) {
	ga, _ := defaultPair()

	// A gear turned by extra about its own axis, with its advance unchanged.
	turned := func(extra float64) Gear {
		g := ga
		g.Mount += extra
		return g
	}
	fits := func(extra float64) bool {
		g := turned(extra)
		w, th := g.P.Width/2, g.P.Thickness/2
		for s := g.P.CollarAt - g.P.CollarDepth; s <= g.P.CollarAt+g.P.CollarDepth; s += 0.01 {
			for i := range 5 {
				v := -th + 2*th*float64(i)/4
				for _, u := range []float64{w, -w} {
					// The collar is the one built for the gear in its own place.
					if inCollar(ga, g.world(u, v, s)) {
						return false
					}
				}
			}
		}
		return true
	}

	if !fits(0) {
		t.Fatal("the gear does not pass its own collar even in step, so nothing below means anything")
	}

	var slack float64
	for extra := 0.0; extra < 0.6; extra += 0.002 {
		if !fits(extra) {
			slack = extra
			break
		}
	}
	if slack == 0 {
		t.Fatal("the gear turns freely inside its collar: the opening is not holding it to the " +
			"screw motion, which is the one thing the frame is for")
	}
	if got := slack * 180 / math.Pi; got > 12 {
		t.Errorf("the collar lets the gear turn %.1f degrees out of step, which is more play than "+
			"a mechanism of one degree of freedom can be said to have", got)
	}
	t.Logf("the collar jams the gear %.2f degrees out of step, on %.2f mm of clearance",
		slack*180/math.Pi, ga.P.Clearance)
}
