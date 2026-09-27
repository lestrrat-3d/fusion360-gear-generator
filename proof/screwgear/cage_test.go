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
// The collar is what holds a gear to its screw motion. Its BORE is the
// ribbon's crest rectangle plus the clearance, turned to the angle the ribbon
// has there and twisted through the collar at the ribbon's own lead, so a gear
// that turns without advancing jams in it. A round hole would not, and the
// mechanism would have three degrees of freedom instead of one. The collar's
// outside is that bore grown by the wall in every direction of its own
// section, which rounds every corner off.
//
// What passes through a collar is the plain ribbon, teeth and all. The crest
// of a cosine rack is the ribbon's outer edge, so the crest rectangle holds
// every point of the ribbon and the bore needs no tooth-shaped cut; the crests
// are what bear on its toothed side, as in the video, where the teeth run
// straight through the collars.
//
// A rod cannot stand where its ribbon crosses the frame, because the ribbon
// runs on through that point. It stands beside its collar instead, on the
// ring's own circle, turned round the ring by the least angle at which it
// clears both ribbons at every phase of the travel, and the collar's wall is
// what joins the two. All four are turned the same way round, so they land in
// the gaps between the ribbons rather than against each other, and the loop
// that joins their feet is a rectangle with a corner at each rod.

// crossing is one place a ribbon passes through the frame: its collar, and the
// rod that serves it.
type crossing struct {
	gear    int     // 0 for gear A, 1 for gear B
	g       Gear    // that gear, at the assembly phase
	station float64 // where the collar sits on the ribbon's own axis
	azimuth float64 // of that point round the frame's axis
	rod     float64 // azimuth of the rod, on the ring's circle
}

// frame is the whole skeleton, with the rods placed.
type frame struct {
	p     Params
	gears [2]Gear
	cross [4]crossing
}

func azimuthOf(g Gear, station float64) float64 {
	c := g.Origin.Add(g.Ez.Scale(station))
	return math.Atan2(c.Y, c.X)
}

// newFrame places the frame round the pair, deriving where each rod stands.
//
// Every collar sits where its ribbon's axis crosses the circle of radius
// CageRadius about the frame's axis, which is station +/-CageRadius of that
// axis for both gears alike. The assembly phase moves gear B's ribbon along
// its axis and not its collars: the ribbon is the same twisted rack from end
// to end, so any stretch of it fits a collar, and the frame has no way to
// know the phase, since the ribbon can go in at any phase a pitch apart.
func newFrame(ga, gb Gear) frame {
	f := frame{p: ga.P, gears: [2]Gear{ga, gb}}
	i := 0
	for gi, g := range f.gears {
		for _, station := range boreStations(g.P) {
			f.cross[i] = crossing{gear: gi, g: g, station: station, azimuth: azimuthOf(g, station)}
			i++
		}
	}
	for i := range f.cross {
		f.cross[i].rod = f.cross[i].azimuth + f.rodShift(f.cross[i])
	}
	return f
}

// travelLimits is how far the pair can advance from the assembly position,
// backward and forward, before an end of either ribbon leaves one of that
// ribbon's collars. Advancing a gear moves its ends with it, so a gear whose
// far end is D past a collar's far face can advance D that way. Gear B is
// assembled a fraction of a pitch along its axis, so its two limits differ by
// that much, and the pair's limit each way is the tighter gear's.
func (f frame) travelLimits() (float64, float64) {
	back, fwd := math.Inf(1), math.Inf(1)
	for _, g := range f.gears {
		lo, hi := g.span()
		for _, station := range boreStations(g.P) {
			back = math.Min(back, hi-(station+f.p.CollarHalf))
			fwd = math.Min(fwd, (station-f.p.CollarHalf)-lo)
		}
	}
	return back, fwd
}

// rodShift is the least angle round the ring, in the frame's positive sense,
// at which a rod on the ring's circle clears BOTH ribbons at every phase of the
// travel by the clearance. Searching from zero is what puts the rod as close
// beside its collar as it can stand.
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
// from everything either ribbon reaches at any phase. The rod runs the whole
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

// eachRibbonPoint walks a ribbon's whole surface at one tooth phase, from one
// end of the ribbon to the other.
func eachRibbonPoint(g Gear, step float64, fn func(pt r3.Vec, u, s float64)) {
	from, to := g.span()
	for s := from; s <= to; s += step {
		uHi, uLo, t := g.profile(s)
		for i := range 5 {
			v := -t + 2*t*float64(i)/4
			for _, u := range []float64{uHi, uLo, (uHi + uLo) / 2} {
				fn(g.world(u, v, s), u, s)
			}
		}
	}
}

// eachEnvelopePoint walks the boundary of everything a ribbon reaches at any
// tooth phase, between two stations.
func eachEnvelopePoint(g Gear, from, to, step float64, fn func(pt r3.Vec)) {
	uHi, uLo, t := g.envelope()
	for s := from; s <= to; s += step {
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

// travelSpan is every station a ribbon covers at some phase of the travel:
// its span at the assembly phase, stretched half the travel each way.
func travelSpan(g Gear) (float64, float64) {
	lo, hi := g.span()
	return lo - g.P.Travel()/2, hi + g.P.Travel()/2
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
		// And the whole rod runs within the collar's length, not just its
		// axis: the collar's ends are square to the ribbon, so the rod's
		// station along the ribbon has to sit its own radius inside either end.
		_, along := boreGap(c.g, f.rodPoint(c, 0))
		if off := math.Abs(along - c.station); off+p.RodRadius() > p.CollarHalf+1e-9 {
			t.Errorf("the rod for the collar at %.0f degrees stands %.2f mm along the ribbon from the "+
				"collar's middle, so part of its %.1f mm diameter misses the collar's %.1f mm length",
				c.azimuth*180/math.Pi, off, p.RodDiameter, 2*p.CollarHalf)
		}
		chord := 2 * p.RingRadius * math.Sin(shift/2)
		t.Logf("rod %d stands %.1f degrees round the ring from its collar, %.1f mm from the crossing, "+
			"its axis %.2f mm outside the bore where the wall takes it in and %.2f mm along the ribbon "+
			"from the collar's middle",
			i, shift*180/math.Pi, chord, nearest, along-c.station)
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

// Everything either ribbon reaches at any phase of the travel has to miss every
// part of the frame but its own bore, by the clearance. This walks the crest
// rectangle of both ribbons, over every station the travel carries them
// through, against the ring, the loop, the rods and the collars.
func TestRibbonsClearTheFrameOverTheTravel(t *testing.T) {
	f := defaultFrame()
	p := f.p

	ring, loop, rod := math.Inf(1), math.Inf(1), math.Inf(1)
	for _, g := range f.gears {
		lo, hi := travelSpan(g)
		eachEnvelopePoint(g, lo, hi, 0.02, func(pt r3.Vec) {
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
			t.Errorf("a ribbon comes within %.3f mm of the %s over the travel, under the %.2f mm clearance",
				gap, name, p.Clearance)
		}
	}
	behind, ahead := f.travelLimits()
	t.Logf("over the %.1f mm travel the ribbons keep %.2f mm from the ring, %.2f mm from the loop and "+
		"%.2f mm from the rods", behind+ahead, ring, loop, rod)
}

// Inside a collar every point of the ribbon, teeth included, stays inside the
// bore by the clearance, at every phase of the travel; and the bore is no
// looser than that. The back edge and both faces run the clearance from the
// wall at every station, and on the toothed side the crests come to the
// clearance whenever the travel brings one into the collar, which is what
// "the crests bear on the bore's toothed side" means. Between crests the
// toothed side falls away by the tooth height, and nothing bears there.
func TestRibbonsStayInsideTheirBoresOverTheTravel(t *testing.T) {
	f := defaultFrame()
	p := f.p

	// The least gap on each side of the bore, over every collar, phase and
	// station: the crest side, the back edge and the faces.
	behind, ahead := f.travelLimits()
	crest, back, face := math.Inf(1), math.Inf(1), math.Inf(1)
	for _, c := range f.cross {
		for d := -behind; d <= ahead+1e-9; d += 0.05 {
			g := c.g
			g.Phase = c.g.Phase + d
			for s := c.station - p.CollarHalf; s <= c.station+p.CollarHalf+1e-9; s += 0.02 {
				uHi, uLo, vHalf := g.profile(s)
				crest = math.Min(crest, p.BoreHalfWidth()-uHi)
				back = math.Min(back, p.BoreHalfWidth()+uLo)
				face = math.Min(face, p.BoreHalfThickness()-vHalf)
			}
		}
	}
	for name, gap := range map[string]float64{"crests": crest, "back edge": back, "faces": face} {
		if gap < p.Clearance-1e-6 {
			t.Errorf("the ribbon's %s come within %.4f mm of a bore's wall over the travel, under the "+
				"%.2f mm clearance", name, gap, p.Clearance)
		}
	}
	// And no looser: the bore is cut to the ribbon, not merely round it.
	for name, gap := range map[string]float64{"crests": crest, "back edge": back, "faces": face} {
		if gap > p.Clearance+5e-3 {
			t.Errorf("the ribbon's %s never come nearer a bore's wall than %.4f mm: the bore is cut "+
				"looser than the %.2f mm clearance", name, gap, p.Clearance)
		}
	}
	t.Logf("over the %.1f mm travel the bores hold the ribbon at %.3f mm on the crests, %.3f mm on "+
		"the back edge and %.3f mm on the faces; the bore is %.1f by %.1f mm",
		behind+ahead, crest, back, face, 2*p.BoreHalfWidth(), 2*p.BoreHalfThickness())
}

// The bore is not the twisted channel the model describes. Fusion has no twist
// for a solid, so the build CUTS it with a loft through a handful of rotated
// rectangles, and a loft is flat between its sections: the wall is faceted, and
// every facet stands a little inside the true channel. What that costs is
// clearance, because the facet takes its bite out of the gap the ribbon passes
// through, and enough of it would bind the gear.
//
// This measures the bite. It builds the lofted opening over the collar's span
// as the build would, and asks how much room is left round the ribbon's crest
// rectangle.
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
				"leaves the ribbon %.4f mm, under the %.4f mm that is 95%% of the clearance",
				c.station, n, turn/float64(n-1)*180/math.Pi, left, 0.95*p.Clearance)
		}
		worst = math.Min(worst, left)
	}

	// What the spec once fixed at five sections, for the record.
	c := f.cross[0]
	lo, hi := boreSpan(p, c.station)
	t.Logf("the lofted bore turns %.0f degrees through %d sections and leaves the ribbon %.4f mm of "+
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

// boreLoftClearance is the least room the lofted opening leaves round the
// ribbon's crest rectangle, over the whole span. The loft's corners run
// straight from one section to the next, so at a station between two sections
// the opening is the four corners interpolated, and the crest rectangle has to
// sit inside that quadrilateral.
func boreLoftClearance(g Gear, lo, hi float64, n int) float64 {
	p := g.P
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	bw, bt := p.Width/2, p.Thickness/2
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
// TestRibbonsClearTheFrameOverTheTravel is the first half, that the gear moves
// freely through its travel. This is the second: the test turns a gear out of
// step with its own advance and finds the angle at which its crests and back
// corners jam in the collars.
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

// Nothing on the ribbon limits the travel: it is the same twisted rack from
// end to end, so any stretch of it fits a collar. What limits it is the
// ribbon's length. A gear has to keep both its collars full, and it has to
// keep the engaged zone covered or the teeth stop meshing; whichever of the
// two an end reaches first is the limit. This walks each gear out of the
// assembly position both ways to find both, holds Travel to the collars, and
// records what the pair comes to once gear B's assembly phase is counted.
func TestTravelIsTheRibbonBetweenItsCollars(t *testing.T) {
	f := defaultFrame()
	p := f.p
	window := axialWindow(p)

	// covered answers whether the ribbon, advanced by d, still spans [lo, hi]
	// of its own axis.
	covered := func(g Gear, d, lo, hi float64) bool {
		g.Phase += d
		from, to := g.span()
		return from <= lo+1e-9 && to >= hi-1e-9
	}
	// firstUncovered is how far the gear can advance, in one direction, before
	// [lo, hi] is no longer inside it.
	firstUncovered := func(g Gear, sign, lo, hi float64) float64 {
		for d := 0.0; d <= p.Length(); d += 0.01 {
			if !covered(g, sign*d, lo, hi) {
				return d
			}
		}
		return math.Inf(1)
	}

	// Each gear's own limits, each way, from the collars and from the mesh.
	var collars, mesh [2]float64 // [0] backward, [1] forward, the tighter gear's
	for i := range collars {
		collars[i], mesh[i] = math.Inf(1), math.Inf(1)
	}
	for _, g := range f.gears {
		for i, sign := range []float64{-1, 1} {
			for _, station := range boreStations(p) {
				collars[i] = math.Min(collars[i],
					firstUncovered(g, sign, station-p.CollarHalf, station+p.CollarHalf))
			}
			mesh[i] = math.Min(mesh[i], firstUncovered(g, sign, -window, window))
		}
	}

	for i, way := range []string{"backward", "forward"} {
		if mesh[i] <= collars[i] {
			t.Errorf("%s, an end leaves the engaged zone at %.2f mm of advance, before it leaves a "+
				"collar at %.2f mm: the collars are not what limits the travel", way, mesh[i], collars[i])
		}
	}
	// A ribbon alone runs Travel through its own collars. Gear B sits a
	// fraction of a pitch along its axis, so it reaches one collar that much
	// sooner than gear A does, and the pair loses exactly that.
	pairTravel := collars[0] + collars[1]
	if got, want := pairTravel, p.Travel()-math.Abs(assemblyPhase); math.Abs(got-want) > 0.02 {
		t.Errorf("walking the ends finds a travel of %.2f mm; Travel less the assembly phase is %.2f",
			got, want)
	}
	if behind, ahead := f.travelLimits(); math.Abs(behind-collars[0]) > 0.011 || math.Abs(ahead-collars[1]) > 0.011 {
		t.Errorf("walking the ends finds limits of %.2f mm back and %.2f mm forward; travelLimits "+
			"derives %.2f and %.2f", collars[0], collars[1], behind, ahead)
	}
	if teeth := pairTravel / p.ToothPitch; teeth < 2 {
		t.Errorf("the travel is %.2f mm, only %.1f teeth, too short to show a gear working",
			pairTravel, teeth)
	}
	t.Logf("the pair travels %.1f mm, %.1f teeth, %.0f%% of the ribbon: %.1f mm back and %.1f mm "+
		"forward of the assembly position, where an end reaches its collar's far face; a ribbon alone "+
		"runs %.1f mm through its collars, and gear B, assembled %.2f mm along its axis, reaches one "+
		"that much sooner; an end would leave the engaged zone at %.1f mm",
		pairTravel, pairTravel/p.ToothPitch, 100*pairTravel/p.Length(), collars[0], collars[1],
		p.Travel(), math.Abs(assemblyPhase), math.Min(mesh[0], mesh[1]))
}

// Outside the engaged zone the two ribbons have to clear each other at every
// phase of the travel. The mesh proof walks the bare ribbons at the assembly
// phases; this walks everything each ribbon reaches at any phase, over every
// station the travel carries it through, against the other, and holds the
// collars outside the engaged zone as well.
func TestRibbonsClearEachOtherOutsideTheEngagement(t *testing.T) {
	f := defaultFrame()
	p := f.p
	window := axialWindow(p)

	worst, worstAt := math.Inf(-1), 0.0
	for i, g := range f.gears {
		other := f.gears[1-i]
		lo, hi := travelSpan(g)
		for _, side := range [][2]float64{{lo, -window}, {window, hi}} {
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
			"%.2f, under the %.2f mm clearance", -worst, worstAt, p.Clearance)
	}
	if got := p.CageRadius - p.CollarHalf; got < window {
		t.Errorf("a collar starts %.2f mm from the middle, inside the %.2f mm the teeth engage over", got, window)
	}
	t.Logf("outside the engaged zone the ribbons keep %.2f mm from each other at every phase of the "+
		"travel, closest at station %.2f; a collar starts %.2f mm from the middle", -worst, worstAt,
		p.CageRadius-p.CollarHalf)
}
