package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// The frame is one tube with a slot through its wall for each gear, and the
// slot is what makes it a frame rather than a pair of bearings: a round hole
// would let its ribbon turn freely as it slid, leaving the mechanism three
// degrees of freedom instead of one.
//
// Two things can go wrong with that, and neither shows up in the meshing proof.
// The four places the ribbons pierce the wall can run into each other and cut
// the tube in half, and a slot can reach the rim and open into a notch. Both
// are decided by the tube's diameter against the crossing angle, so both are
// checked here on the wall itself rather than argued from a formula.

// wallGrid marks which cells of the tube's wall survive the slots. It is the
// same test the picture uses: a cell is gone when its own midpoint falls inside
// a gear's clearance rectangle.
func wallGrid(p Params, ga, gb Gear, nAz, nZ int) [][]bool {
	mid := (p.CageInner() + p.CageOuter()) / 2
	half := p.CageHalfHeight()
	keep := make([][]bool, nAz)
	for i := range nAz {
		keep[i] = make([]bool, nZ)
		for j := range nZ {
			a := 2 * math.Pi * (float64(i) + 0.5) / float64(nAz)
			z := -half + 2*half*(float64(j)+0.5)/float64(nZ)
			pt := r3.NewVec(mid*math.Cos(a), mid*math.Sin(a), z)
			open := false
			for _, g := range []Gear{ga, gb} {
				u, v, _ := g.local(pt)
				if math.Abs(u) <= p.Width/2+p.Clearance && math.Abs(v) <= p.Thickness/2+p.Clearance {
					open = true
					break
				}
			}
			keep[i][j] = !open
		}
	}
	return keep
}

// components counts the connected regions of cells whose flag equals want,
// walking the grid four ways and wrapping around in azimuth.
func components(grid [][]bool, want bool) int {
	nAz, nZ := len(grid), len(grid[0])
	seen := make([][]bool, nAz)
	for i := range seen {
		seen[i] = make([]bool, nZ)
	}
	count := 0
	for i := range nAz {
		for j := range nZ {
			if seen[i][j] || grid[i][j] != want {
				continue
			}
			count++
			stack := [][2]int{{i, j}}
			for len(stack) > 0 {
				c := stack[len(stack)-1]
				stack = stack[:len(stack)-1]
				ci, cj := (c[0]+nAz)%nAz, c[1]
				if cj < 0 || cj >= nZ || seen[ci][cj] || grid[ci][cj] != want {
					continue
				}
				seen[ci][cj] = true
				stack = append(stack, [2]int{ci + 1, cj}, [2]int{ci - 1, cj},
					[2]int{ci, cj + 1}, [2]int{ci, cj - 1})
			}
		}
	}
	return count
}

func TestCageSlotsLeaveOneTubeAndFourHoles(t *testing.T) {
	p := defaultParams()
	ga, gb := defaultPair()
	grid := wallGrid(p, ga, gb, 360, 240)

	if got := components(grid, true); got != 1 {
		t.Errorf("the slots cut the wall into %d pieces; the frame has to be one body", got)
	}
	if got := components(grid, false); got != 4 {
		t.Errorf("the wall carries %d openings, want 4: each gear pierces it twice", got)
	}
}

// The point of a slot is that its own gear goes through it. This walks both
// ribbons end to end and asserts that no part of either is ever inside the
// tube's wall.
//
// One static pass settles it for every position the gear takes. A slot is cut
// to the ribbon's blank — the full-width rectangle, before the teeth are taken
// out of one edge — and that blank is invariant under the gear's own screw
// motion, so a ribbon that clears the wall at one phase clears it at all of
// them. The teeth cannot change that either: they are cut INTO the edge, so the
// material only ever retreats from the slot's face.
func TestRibbonsPassThroughTheirSlots(t *testing.T) {
	p := defaultParams()
	ga, gb := defaultPair()
	half := p.Length() / 2

	inWall := func(pt r3.Vec) bool {
		r := math.Hypot(pt.X, pt.Y)
		if r < p.CageInner() || r > p.CageOuter() || math.Abs(pt.Z) > p.CageHalfHeight() {
			return false
		}
		for _, g := range []Gear{ga, gb} {
			u, v, _ := g.local(pt)
			if math.Abs(u) <= p.Width/2+p.Clearance && math.Abs(v) <= p.Thickness/2+p.Clearance {
				return false // inside a slot, which is cut away
			}
		}
		return true
	}

	for _, g := range []Gear{ga, gb} {
		w, th := g.P.Width/2, g.P.Thickness/2
		for s := -half; s <= half; s += 0.02 {
			e := g.edge(s)
			for i := range 5 {
				v := -th + 2*th*float64(i)/4
				for _, u := range []float64{e, -w, (e - w) / 2} {
					if pt := g.world(u, v, s); inWall(pt) {
						t.Fatalf("a ribbon meets the cage wall at station %.2f, (u,v)=(%.2f,%.2f), "+
							"radius %.2f", s, u, v, math.Hypot(pt.X, pt.Y))
					}
				}
			}
		}
	}
}

// A slot that reaches the tube's rim is a notch rather than a hole, and the
// ribbon would fall out of it sideways.
func TestCageSlotsStayOffTheRim(t *testing.T) {
	p := defaultParams()
	ga, gb := defaultPair()
	grid := wallGrid(p, ga, gb, 360, 240)

	nZ := len(grid[0])
	for i := range grid {
		if !grid[i][0] || !grid[i][nZ-1] {
			t.Fatalf("a slot reaches the tube's rim at azimuth %.1f degrees",
				360*float64(i)/float64(len(grid)))
		}
	}
}

// How small the tube can go, measured on the wall rather than argued.
//
// There are two separate floors and they are far apart. A narrow tube is cut
// into pieces by its own slots and stops being a frame at all. A wider one
// stays in one piece, but each gear's two piercings still reach around and join
// into a single opening, which leaves the wall standing on two arms rather than
// four. The spec carries both numbers and the default clears both.
//
// The spec first tried to settle this with an arc — the two gears pierce the
// wall Sigma apart, so the wall survives while (CageDiameter/2)*Sigma exceeds a
// slot's width — and that rule is wrong twice over. A slot's width around the
// tube is not the ribbon's thickness, because the ribbon crosses the wall at
// whatever station puts it at the tube's radius and the cross-section angle
// there decides how much of the opening lies across the tube. And the rule is
// about the wrong pair of openings: what merges first is one gear's own two
// piercings, not one gear's against the other's.
func TestCageDiameterFloorsAreMeasured(t *testing.T) {
	p := defaultParams()

	wall := func(diameter float64) (pieces, holes int) {
		q := p
		q.CageDiameter = diameter
		ga, gb := pair(q, q.Sigma(), 0, q.ToothPitch/2)
		grid := wallGrid(q, ga, gb, 360, 240)
		return components(grid, true), components(grid, false)
	}

	var onePiece, fourHoles float64
	for d := 4.0; d <= p.CageDiameter; d += 0.25 {
		pieces, holes := wall(d)
		if onePiece == 0 && pieces == 1 {
			onePiece = d
		}
		if fourHoles == 0 && pieces == 1 && holes == 4 {
			fourHoles = d
		}
	}
	if onePiece == 0 || fourHoles == 0 {
		t.Fatalf("no tube up to the default holds together: one piece from %.2f, four holes from %.2f",
			onePiece, fourHoles)
	}
	if pieces, _ := wall(onePiece - 0.5); pieces == 1 {
		t.Errorf("the wall is still one piece at %.2f mm, below the %.2f mm called the floor",
			onePiece-0.5, onePiece)
	}
	if p.CageDiameter < fourHoles+4 {
		t.Errorf("the default tube is %.2f mm across and its openings merge below %.2f mm, "+
			"which is less than 4 mm of margin", p.CageDiameter, fourHoles)
	}
	t.Logf("one body from %.2f mm across, four distinct openings from %.2f mm; default %.2f mm",
		onePiece, fourHoles, p.CageDiameter)
}
