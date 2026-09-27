package screwgear_test

import (
	"math"
	"sort"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// The frame is the one in Segerman's video: an open skeleton of round rods,
// with no wall, no plate and no post. A round wire RING stands at one end, a
// smaller LOOP with straight sides at the other, four thin RODS run between
// them, and a short smooth COLLAR sits round each ribbon where it crosses the
// frame. The ring is at gear B's end and the loop at gear A's, so one gear's
// collars sit low and the other's high, and the gears meet in the middle.
//
// The collar is what holds a gear to its screw motion. Its BORE is the ribbon's
// own cross-section, turned to the angle the ribbon has there and twisted
// through the collar at the ribbon's own lead, so a gear that turns without
// advancing jams in it. A round hole would not, and the mechanism would have
// three degrees of freedom instead of one. The collar's outside is that bore
// grown by the wall in every direction of its own section, which rounds every
// corner off.
//
// What passes through a collar is never a tooth. The ribbon carries a smooth
// boss at each of the two places it crosses the frame, the bore is cut to that
// boss, and the teeth stay outside it.
//
// A rod cannot stand where its ribbon crosses the frame, because the ribbon
// runs on through that point. It stands beside its collar instead, on the
// ring's own circle, turned round the ring by the least angle at which it
// clears both ribbons over the whole stroke, and the collar's wall is what
// joins the two. All four are turned the same way round, so they land in the
// gaps between the ribbons rather than against each other, and the loop that
// joins their feet is a rectangle with a corner at each rod.

// crossing is one place a ribbon passes through the frame: its collar, and the
// rod that serves it.
type crossing struct {
	gear    int     // 0 for gear A, 1 for gear B
	g       Gear    // that gear, at the assembly phase
	station float64 // where the boss sits on the ribbon's own axis at that phase
	azimuth float64 // of that point round the frame's axis
	rod     float64 // azimuth of the rod, on the ring's circle
}

// frame is the whole skeleton, with the rods placed.
type frame struct {
	p     Params
	gears [2]Gear
	cross [4]crossing
}

// collarStations are where a gear's collars sit on its own axis: where its
// bosses are at the assembly phase. Gear B is assembled a fraction of a pitch
// along its axis from gear A, and its bosses go with it, so its collars are
// that far from the ring's circle along its axis.
func collarStations(g Gear) [2]float64 {
	s := boreStations(g.P)
	return [2]float64{s[0] + g.Phase, s[1] + g.Phase}
}

func azimuthOf(g Gear, station float64) float64 {
	c := g.Origin.Add(g.Ez.Scale(station))
	return math.Atan2(c.Y, c.X)
}

// newFrame places the frame round the pair, deriving where each rod stands.
func newFrame(ga, gb Gear) frame {
	f := frame{p: ga.P, gears: [2]Gear{ga, gb}}
	i := 0
	for gi, g := range f.gears {
		for _, station := range collarStations(g) {
			f.cross[i] = crossing{gear: gi, g: g, station: station, azimuth: azimuthOf(g, station)}
			i++
		}
	}
	for i := range f.cross {
		f.cross[i].rod = f.cross[i].azimuth + f.rodShift(f.cross[i])
	}
	return f
}

// rodShift is the least angle round the ring, in the frame's positive sense,
// at which a rod on the ring's circle clears BOTH ribbons over the whole stroke
// by the clearance. Searching from zero is what puts the rod as close beside
// its collar as it can stand.
//
// The search never finishes at a full turn: a rod that clears nowhere on the
// circle means the ring is too small for the ribbon, and that is reported by
// the tests rather than hidden in a placement.
func (f frame) rodShift(c crossing) float64 {
	const step = 0.25 * math.Pi / 180
	for shift := 0.0; shift <= 2*math.Pi; shift += step {
		if f.rodClears(c.azimuth + shift) {
			return shift
		}
	}
	return math.NaN()
}

// rodClears answers whether a rod at the given azimuth stays the clearance away
// from everything either ribbon reaches over the stroke. The rod runs the whole
// height of the frame, so only the horizontal distance counts.
func (f frame) rodClears(azimuth float64) bool {
	p := f.p
	rx, ry := p.RingRadius*math.Cos(azimuth), p.RingRadius*math.Sin(azimuth)
	need := p.RodRadius() + p.Clearance
	reach := p.RingRadius + p.Width // further out along a ribbon nothing can touch the ring's circle
	for _, g := range f.gears {
		clear := true
		eachEnvelopePoint(g, -reach, reach, 0.05, func(pt r3.Vec) {
			if clear && math.Hypot(pt.X-rx, pt.Y-ry) < need {
				clear = false
			}
		})
		if !clear {
			return false
		}
	}
	return true
}

// rodPoint is a point on a rod's axis at height z.
func (f frame) rodPoint(c crossing, z float64) r3.Vec {
	return r3.NewVec(f.p.RingRadius*math.Cos(c.rod), f.p.RingRadius*math.Sin(c.rod), z)
}

// loopBars are the loop's straight sides: one bar from each rod's foot to the
// next rod's foot round the ring.
func (f frame) loopBars() [4][2]r3.Vec {
	order := []int{0, 1, 2, 3}
	sort.Slice(order, func(a, b int) bool {
		return wrap(f.cross[order[a]].rod) < wrap(f.cross[order[b]].rod)
	})
	var bars [4][2]r3.Vec
	for i := range 4 {
		a, b := f.cross[order[i]], f.cross[order[(i+1)%4]]
		bars[i] = [2]r3.Vec{f.rodPoint(a, -f.p.CageRise), f.rodPoint(b, -f.p.CageRise)}
	}
	return bars
}

func wrap(a float64) float64 { return math.Mod(a+4*math.Pi, 2*math.Pi) }

// segmentDistance is the distance from a point to a straight bar's axis.
func segmentDistance(pt, a, b r3.Vec) float64 {
	ab := b.Sub(a)
	t := pt.Sub(a).Dot(ab) / ab.Dot(ab)
	t = math.Max(0, math.Min(1, t))
	return pt.Sub(a.Add(ab.Scale(t))).Len()
}

// ringGap, loopGap and rodGap are how far a point is from the surface of each
// round part, negative inside it.
func (f frame) ringGap(pt r3.Vec) float64 {
	p := f.p
	return math.Hypot(math.Hypot(pt.X, pt.Y)-p.RingRadius, pt.Z-p.CageRise) - p.WireRadius()
}

func (f frame) loopGap(pt r3.Vec) float64 {
	worst := math.Inf(1)
	for _, bar := range f.loopBars() {
		worst = math.Min(worst, segmentDistance(pt, bar[0], bar[1])-f.p.WireRadius())
	}
	return worst
}

func (f frame) rodGap(pt r3.Vec) float64 {
	worst := math.Inf(1)
	for _, c := range f.cross {
		d := segmentDistance(pt, f.rodPoint(c, -f.p.CageRise), f.rodPoint(c, f.p.CageRise))
		worst = math.Min(worst, d-f.p.RodRadius())
	}
	return worst
}

// inBore answers whether a point is inside the channel cut for this gear. The
// channel follows the ribbon, so it is twisted rather than straight.
func inBore(g Gear, pt r3.Vec) bool {
	d, _ := boreGap(g, pt)
	return d == 0
}

// boreGap is how far outside the bore's rectangle a point lies, measured in the
// ribbon's own section at that station with the twist undone, and zero inside.
// The collar is the bore grown by this distance, so its outline is a rounded
// rectangle that turns with the ribbon.
func boreGap(g Gear, pt r3.Vec) (float64, float64) {
	p := g.P
	u, v, s := g.local(pt)
	du := math.Max(0, math.Abs(u)-p.BoreHalfWidth())
	dv := math.Max(0, math.Abs(v)-p.BoreHalfThickness())
	return math.Hypot(du, dv), s
}

// inCollar answers whether a point is in the material of one collar.
func (f frame) inCollar(c crossing, pt r3.Vec) bool {
	d, s := boreGap(c.g, pt)
	if math.Abs(s-c.station) > f.p.CollarHalf {
		return false
	}
	return d > 0 && d <= f.p.CollarWall
}

// inFrame answers whether a point is inside any material of the frame.
func (f frame) inFrame(pt r3.Vec) bool {
	if f.ringGap(pt) <= 0 || f.loopGap(pt) <= 0 || f.rodGap(pt) <= 0 {
		return true
	}
	for _, c := range f.cross {
		if f.inCollar(c, pt) {
			return true
		}
	}
	return false
}

// eachRibbonPoint walks a ribbon's whole surface at one tooth phase.
func eachRibbonPoint(g Gear, step float64, fn func(pt r3.Vec, u, s float64)) {
	half := g.P.Length() / 2
	for s := -half; s <= half; s += step {
		uHi, uLo, t := g.profile(s)
		for i := range 5 {
			v := -t + 2*t*float64(i)/4
			for _, u := range []float64{uHi, uLo, (uHi + uLo) / 2} {
				fn(g.world(u, v, s), u, s)
			}
		}
	}
}

// eachEnvelopePoint walks the boundary of everything a ribbon reaches over the
// whole stroke, between two stations.
func eachEnvelopePoint(g Gear, from, to, step float64, fn func(pt r3.Vec)) {
	for s := from; s <= to; s += step {
		uHi, uLo, t := g.envelope(s)
		for i := range 5 {
			k := float64(i) / 4
			v := -t + 2*t*k
			fn(g.world(uHi, v, s))
			fn(g.world(uLo, v, s))
			u := uLo + (uHi-uLo)*k
			fn(g.world(u, t, s))
			fn(g.world(u, -t, s))
		}
	}
}

func defaultFrame() frame {
	ga, gb := defaultPair()
	return newFrame(ga, gb)
}

// One gear's collars sit low and the other's high, so the two ribbons pass the
// frame at different heights and their teeth meet in the middle.
func TestBoresSitOnOppositeSidesOfTheMiddle(t *testing.T) {
	f := defaultFrame()
	for _, c := range f.cross {
		at := c.g.Origin.Add(c.g.Ez.Scale(c.station))
		if c.gear == 0 && at.Z >= 0 {
			t.Errorf("gear A's collar at station %.1f sits at height %.2f, not below the middle",
				c.station, at.Z)
		}
		if c.gear == 1 && at.Z <= 0 {
			t.Errorf("gear B's collar at station %.1f sits at height %.2f, not above the middle",
				c.station, at.Z)
		}
	}
}

// Each rod stands beside its own collar, on the ring's circle, and is joined
// to it: some stretch of the rod runs inside the collar's wall. The four are
// turned the same way round from their crossings, and none touches another.
//
// The angle is derived rather than chosen, and this records what it came to.
// A frame whose ring is too small for its ribbon has no place to put a rod at
// all, and that fails here before any picture is drawn.
func TestRodsStandBesideTheirCollars(t *testing.T) {
	f := defaultFrame()
	p := f.p

	for i, c := range f.cross {
		shift := c.rod - c.azimuth
		if math.IsNaN(shift) {
			t.Fatalf("no rod on the ring's circle clears the ribbons for the collar at %.0f degrees: "+
				"the ring is too small for the ribbon", c.azimuth*180/math.Pi)
		}
		if shift <= 0 || shift >= math.Pi/2 {
			t.Errorf("the rod for the collar at %.0f degrees stands %.1f degrees round from it, "+
				"outside the quarter turn a rod beside its collar can take",
				c.azimuth*180/math.Pi, shift*180/math.Pi)
		}
		// How far the rod's axis runs from the bore, where the collar's wall
		// takes it in: under the wall's thickness, or nothing joins the two.
		joined, nearest := false, math.Inf(1)
		for z := -p.CageRise; z <= p.CageRise; z += 0.05 {
			pt := f.rodPoint(c, z)
			if !f.inCollar(c, pt) {
				continue
			}
			joined = true
			gap, _ := boreGap(c.g, pt)
			nearest = math.Min(nearest, gap)
		}
		if !joined {
			t.Errorf("the rod for the collar at %.0f degrees runs nowhere inside that collar's wall, "+
				"so nothing joins the two", c.azimuth*180/math.Pi)
		}
		chord := 2 * p.RingRadius * math.Sin(shift/2)
		t.Logf("rod %d stands %.1f degrees round the ring from its collar, %.1f mm from the crossing, "+
			"its axis %.2f mm outside the bore where the wall takes it in",
			i, shift*180/math.Pi, chord, nearest)
	}

	for i := range f.cross {
		for j := i + 1; j < 4; j++ {
			d := f.rodPoint(f.cross[i], 0).Sub(f.rodPoint(f.cross[j], 0)).Len()
			if d < p.RodDiameter+p.Clearance {
				t.Errorf("rods %d and %d stand %.2f mm apart, which is touching", i, j, d)
			}
		}
	}
}

// The frame is one body. Every rod runs from the ring to the loop, and each
// collar is held by its rod, so the ring reaches everything.
func TestFrameIsOnePiece(t *testing.T) {
	f := defaultFrame()
	p := f.p

	// The pieces, and which ones meet.
	const ring, loop = 0, 1
	rod := func(i int) int { return 2 + i }
	collar := func(i int) int { return 6 + i }
	joined := make([][]int, 10)
	join := func(a, b int) { joined[a] = append(joined[a], b); joined[b] = append(joined[b], a) }

	for i, c := range f.cross {
		if f.ringGap(f.rodPoint(c, p.CageRise)) <= 0 {
			join(ring, rod(i))
		} else {
			t.Errorf("rod %d does not reach the ring", i)
		}
		if f.loopGap(f.rodPoint(c, -p.CageRise)) <= 0 {
			join(loop, rod(i))
		} else {
			t.Errorf("rod %d does not reach the loop", i)
		}
		for z := -p.CageRise; z <= p.CageRise; z += 0.05 {
			if f.inCollar(c, f.rodPoint(c, z)) {
				join(rod(i), collar(i))
				break
			}
		}
	}

	seen := make([]bool, 10)
	stack := []int{ring}
	seen[ring] = true
	for len(stack) > 0 {
		here := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		for _, next := range joined[here] {
			if !seen[next] {
				seen[next] = true
				stack = append(stack, next)
			}
		}
	}
	names := []string{"the ring", "the loop", "rod 0", "rod 1", "rod 2", "rod 3",
		"collar 0", "collar 1", "collar 2", "collar 3"}
	for i, ok := range seen {
		if !ok {
			t.Errorf("%s is not joined to the ring", names[i])
		}
	}

	bars := f.loopBars()
	sides := make([]float64, 0, 4)
	for _, bar := range bars {
		sides = append(sides, bar[1].Sub(bar[0]).Len())
	}
	t.Logf("the loop's sides run %.1f, %.1f, %.1f and %.1f mm between the rods' feet",
		sides[0], sides[1], sides[2], sides[3])
}

// Everything either ribbon reaches over the whole stroke has to miss every part
// of the frame but its own bore, by the clearance. This walks the envelope of
// both ribbons — the crest on the toothed side and the boss at its fullest for
// each station — against the ring, the loop, the rods and the collars.
func TestRibbonsClearTheFrameOverTheStroke(t *testing.T) {
	f := defaultFrame()
	p := f.p
	half := p.Length() / 2

	ring, loop, rod := math.Inf(1), math.Inf(1), math.Inf(1)
	for _, g := range f.gears {
		eachEnvelopePoint(g, -half, half, 0.02, func(pt r3.Vec) {
			ring = math.Min(ring, f.ringGap(pt))
			loop = math.Min(loop, f.loopGap(pt))
			rod = math.Min(rod, f.rodGap(pt))
			for _, c := range f.cross {
				if f.inCollar(c, pt) {
					t.Fatalf("a ribbon reaches into the collar at %.0f degrees at (%.2f, %.2f, %.2f)",
						c.azimuth*180/math.Pi, pt.X, pt.Y, pt.Z)
				}
			}
		})
	}
	for name, gap := range map[string]float64{"ring": ring, "loop": loop, "rods": rod} {
		if gap < p.Clearance {
			t.Errorf("a ribbon comes within %.3f mm of the %s over the stroke, under the %.2f mm clearance",
				gap, name, p.Clearance)
		}
	}
	t.Logf("over a %.2f mm stroke the ribbons keep %.2f mm from the ring, %.2f mm from the loop and "+
		"%.2f mm from the rods", p.Stroke(), ring, loop, rod)
}

// Inside a collar the ribbon both clears the bore and fills it, over the whole
// stroke: what is in the bore is always the boss's flat top, the clearance
// away from the wall all round. A stroke longer than the boss allows would put
// the boss's taper in the bore at the ends of the travel, and the ribbon would
// then be loose in the frame there rather than held.
func TestCollarBoresHoldTheBossOverTheStroke(t *testing.T) {
	f := defaultFrame()
	p := f.p

	least, most := math.Inf(1), 0.0
	for _, c := range f.cross {
		for d := -p.Stroke() / 2; d <= p.Stroke()/2+1e-9; d += p.Stroke() / 20 {
			g := c.g
			g.Phase = c.g.Phase + d
			for s := c.station - p.CollarHalf; s <= c.station+p.CollarHalf+1e-9; s += 0.02 {
				uHi, uLo, vHalf := g.profile(s)
				gap := math.Min(p.BoreHalfWidth()-uHi, math.Min(p.BoreHalfWidth()+uLo,
					p.BoreHalfThickness()-vHalf))
				least, most = math.Min(least, gap), math.Max(most, gap)
			}
		}
	}
	if least < p.Clearance-1e-6 {
		t.Errorf("the ribbon comes within %.4f mm of a bore's wall over the stroke, under the %.2f mm clearance",
			least, p.Clearance)
	}
	if most > p.Clearance+1e-6 {
		t.Errorf("the ribbon falls %.4f mm short of a bore's wall somewhere in the stroke: the boss "+
			"does not fill the bore over the whole travel", most)
	}
	t.Logf("over the %.2f mm stroke every bore holds the boss at %.3f-%.3f mm all round",
		p.Stroke(), least, most)
}

// The bore is not the twisted channel the model describes. Fusion has no twist
// for a solid, so the build CUTS it with a loft through a handful of rotated
// rectangles, and a loft is flat between its sections: the wall is faceted, and
// every facet stands a little inside the true channel. What that costs is
// clearance, because the facet takes its bite out of the gap the boss passes
// through, and enough of it would bind the gear.
//
// This measures the bite. It builds the lofted opening over the collar's span
// as the build would, and asks how much room is left round the boss.
func TestBoreLoftKeepsItsClearance(t *testing.T) {
	f := defaultFrame()
	p := f.p

	worst := math.Inf(1)
	for _, c := range f.cross {
		lo, hi := boreSpan(p, c.station)
		turn := (hi - lo) / p.Lambda()
		n := boreSections(turn)
		left := boreLoftClearance(c.g, lo, hi, n)
		if left < 0.95*p.Clearance {
			t.Errorf("the bore at station %+.1f lofts through %d sections %.1f degrees apart and "+
				"leaves the boss %.4f mm, under the %.4f mm that is 95%% of the clearance",
				c.station, n, turn/float64(n-1)*180/math.Pi, left, 0.95*p.Clearance)
		}
		worst = math.Min(worst, left)
	}

	// What the spec once fixed at five sections, for the record.
	c := f.cross[0]
	lo, hi := boreSpan(p, c.station)
	t.Logf("the lofted bore turns %.0f degrees through %d sections and leaves the boss %.4f mm of "+
		"the %.2f mm clearance; five sections would leave %.4f mm",
		(hi-lo)/p.Lambda()*180/math.Pi, boreSections((hi-lo)/p.Lambda()), worst, p.Clearance,
		boreLoftClearance(c.g, lo, hi, 5))
}

// boreSections is the count the build lofts a bore through: enough that no two
// neighbours are more than five degrees of twist apart. The ribbon's own cell
// is held to two degrees, because there the departure is measured against the
// backlash; here it is measured against a clearance twenty times larger, and
// five degrees already spends under a fiftieth of it.
func boreSections(turn float64) int {
	n := int(math.Ceil(turn/(5*math.Pi/180))) + 1
	if n < 3 {
		n = 3
	}
	return n
}

// boreSpan is the stretch of a gear's own axis a bore's loft covers: the
// collar's length, with a millimetre of margin at each end so the cut runs
// clean through.
func boreSpan(p Params, station float64) (float64, float64) {
	return station - p.CollarHalf - 1, station + p.CollarHalf + 1
}

// boreLoftClearance is the least room the lofted opening leaves round the boss,
// over the whole span. The loft's corners run straight from one section to the
// next, so at a station between two sections the opening is the four corners
// interpolated, and the boss has to sit inside that quadrilateral.
func boreLoftClearance(g Gear, lo, hi float64, n int) float64 {
	p := g.P
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	bw, bt := p.Width/2+p.BossGrow, p.Thickness/2+p.BossGrow
	corner := func(s float64, i int) r3.Vec {
		u, v := hw, ht
		if i == 1 || i == 2 {
			u = -hw
		}
		if i >= 2 {
			v = -ht
		}
		return g.world(u, v, s)
	}

	worst := math.Inf(1)
	for k := range n - 1 {
		sa := lo + (hi-lo)*float64(k)/float64(n-1)
		sb := lo + (hi-lo)*float64(k+1)/float64(n-1)
		for f := 0.0; f <= 1.0; f += 0.02 {
			var quad [4][2]float64
			for i := range 4 {
				a, b := corner(sa, i), corner(sb, i)
				u, v, _ := g.local(a.Add(b.Sub(a).Scale(f)))
				quad[i] = [2]float64{u, v}
			}
			for _, bc := range [][2]float64{{bw, bt}, {-bw, bt}, {-bw, -bt}, {bw, -bt}} {
				for i := range 4 {
					a, b := quad[i], quad[(i+1)%4]
					ex, ey := b[0]-a[0], b[1]-a[1]
					d := ((bc[0]-a[0])*ey - (bc[1]-a[1])*ex) / math.Hypot(ex, ey)
					worst = math.Min(worst, -d)
				}
			}
		}
	}
	return worst
}

// The middle of the frame has to stay open, or the mesh cannot be seen and the
// teeth have nothing to meet in.
func TestTheMiddleStaysOpen(t *testing.T) {
	f := defaultFrame()
	window := axialWindow(f.p)

	for x := -window; x <= window; x += 0.2 {
		for y := -window; y <= window; y += 0.2 {
			for z := -window; z <= window; z += 0.2 {
				if f.inFrame(r3.NewVec(x, y, z)) {
					t.Fatalf("the frame reaches into the meshing space at (%.1f, %.1f, %.1f)", x, y, z)
				}
			}
		}
	}
	t.Logf("nothing of the frame is within %.2f mm of the middle", window)
}

// This is the frame's own proof: it admits the screw motion and nothing else.
// TestRibbonsClearTheFrameOverTheStroke is the first half, that the gear moves
// freely through its stroke. This is the second: the test turns a gear out of
// step with its own advance and finds the angle at which its boss jams in the
// collars.
//
// A frame of round holes would report no jam at any angle, and that is the case
// this rules out.
func TestBoresAdmitOnlyTheScrewMotion(t *testing.T) {
	f := defaultFrame()
	p := f.p
	ga := f.gears[0]

	fits := func(extra float64) bool {
		g := ga
		g.Mount += extra
		clear := true
		eachRibbonPoint(g, 0.05, func(pt r3.Vec, _, _ float64) {
			if clear && f.inFrame(pt) {
				clear = false
			}
		})
		return clear
	}

	if !fits(0) {
		t.Fatal("the gear does not pass its own bores even in step, so nothing below means anything")
	}
	var slack float64
	for extra := 0.0; extra < 0.5; extra += 0.002 {
		if !fits(extra) {
			slack = extra
			break
		}
	}
	if slack == 0 {
		t.Fatal("the gear turns freely in the frame: the bores are not holding it to the screw " +
			"motion, which is the one thing the frame is for")
	}
	if got := slack * 180 / math.Pi; got > 12 {
		t.Errorf("the frame lets the gear turn %.1f degrees out of step, which is more play than a "+
			"mechanism of one degree of freedom can be said to have", got)
	}
	t.Logf("the frame jams the gear %.2f degrees out of step, on %.2f mm of clearance",
		slack*180/math.Pi, p.Clearance)
}

// The boss travels with its gear, so its length is the stroke: the mechanism
// runs only while the boss's flat top still fills the collars, and a collar's
// length comes straight out of the travel.
func TestStrokeIsTheBossLength(t *testing.T) {
	p := defaultParams()

	stroke := p.Stroke()
	if stroke <= 0 {
		t.Fatalf("the boss is %.2f mm long and the collar %.2f mm, so there is no travel",
			2*p.BossHalf, 2*p.CollarHalf)
	}
	if teeth := stroke / p.ToothPitch; teeth < 2 {
		t.Errorf("the stroke is %.2f mm, only %.1f teeth, too short to show a gear working",
			stroke, teeth)
	}
	t.Logf("collar %.2f mm along the ribbon, stroke %.2f mm, which is %.1f teeth",
		2*p.CollarHalf, stroke, stroke/p.ToothPitch)
}

// A tooth must never reach a bore, or the frame would need an opening shaped
// like a tooth and the boss would be pointless. Both gears are walked, because
// gear B's collars sit where ITS bosses are at the assembly phase.
func TestTeethNeverReachABore(t *testing.T) {
	f := defaultFrame()
	p := f.p

	for _, c := range f.cross {
		for d := -p.Stroke() / 2; d <= p.Stroke()/2+1e-9; d += 0.05 {
			for s := c.station - p.CollarHalf; s <= c.station+p.CollarHalf+1e-9; s += 0.02 {
				g := c.g
				g.Phase = c.g.Phase + d
				if b := g.boss(s); b < p.BossGrow-1e-9 {
					t.Fatalf("at travel %+.2f mm the ribbon inside the collar at %.1f stands only "+
						"%.3f mm proud, so a tooth is in the bore", d, c.station, b)
				}
			}
		}
	}
}

// The frame puts each boss a set distance from the middle, and a smaller frame
// puts it nearer the mesh. Outside the engaged zone the two ribbons have to
// clear each other with their bosses on, at every phase of the stroke; the
// mesh proof walks the bare ribbons, and the boss is the frame's business.
func TestBossesClearTheOtherRibbon(t *testing.T) {
	f := defaultFrame()
	p := f.p
	half := p.Length() / 2
	window := axialWindow(p)

	worst, worstAt := math.Inf(-1), 0.0
	for i, g := range f.gears {
		other := f.gears[1-i]
		for _, side := range [][2]float64{{-half, -window}, {window, half}} {
			eachEnvelopePoint(g, side[0], side[1], 0.02, func(pt r3.Vec) {
				if m := other.envelopeMargin(pt); m > worst {
					_, _, s := g.local(pt)
					worst, worstAt = m, s
				}
			})
		}
	}
	if worst > -p.Clearance {
		t.Errorf("outside the engaged zone the ribbons come within %.3f mm of each other at station "+
			"%.2f, under the %.2f mm clearance, once the boss is on", -worst, worstAt, p.Clearance)
	}
	if got := p.CageRadius - p.BossHalf; got < window {
		t.Errorf("a boss starts %.2f mm from the middle, inside the %.2f mm the teeth engage over", got, window)
	}
	t.Logf("with the bosses on, the ribbons keep %.2f mm from each other outside the engaged zone, "+
		"closest at station %.2f; a boss starts %.2f mm from the middle", -worst, worstAt,
		p.CageRadius-p.BossHalf)
}
