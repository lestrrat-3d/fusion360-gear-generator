package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// The frame is a short tube with most of its wall gone: a flat plate at each
// end, and four posts standing between them. Each post sits where one ribbon
// crosses the tube and carries a BORE that ribbon passes through. The two posts
// of one gear are bored low, the two of the other high, and the gears meet in
// the middle, where nothing of the frame stands in the way of seeing them.
//
// The bore is what holds a gear to its screw motion. It is the ribbon's own
// cross-section, turned to the angle the ribbon has there and twisted through
// the post at the ribbon's own lead, so a gear that turns without advancing
// jams in it. A round hole would not, and the mechanism would have three
// degrees of freedom instead of one.
//
// What passes through a bore is never a tooth. The ribbon carries a smooth boss
// at each of the two places it crosses the cage, the bore is cut to that boss,
// and the teeth stay outside it.
//
// Every outside face of the frame lies on ONE cylinder. The plates, the posts
// and the blocks are all pieces of the same wall, differing only in how far
// round and how far up each runs, so nothing stands proud of anything else and
// the whole outside is a single turned surface. The plates' top and bottom
// faces are flat and level, and they are what a print stands on. Only the bore
// inside is skewed, which is the skew the mechanism actually needs.

// postAzimuth is where a gear's post stands, for the bore at the given station.
func postAzimuth(g Gear, station float64) float64 {
	c := g.Origin.Add(g.Ez.Scale(station))
	return math.Atan2(c.Y, c.X)
}

// inBore answers whether a point is inside the channel cut for this gear. The
// channel follows the ribbon, so it is twisted rather than straight, and it
// runs the whole height of the post: a post is solid bar above and below its
// block, and the ribbon has to get past that too.
func inBore(g Gear, pt r3.Vec) bool {
	p := g.P
	u, v, _ := g.local(pt)
	return math.Abs(u) <= p.BoreHalfWidth() && math.Abs(v) <= p.BoreHalfThickness()
}

// boreReach is how far the bore cuts each way from a post's centre line at one
// height, and whether it is there at all. The post is built from it and the
// wall is checked against it, so both read the same measurement.
func boreReach(g Gear, azimuth, z float64) (left, right float64, ok bool) {
	p := g.P
	for r := p.CageInner(); r <= p.CageOuter(); r += 0.1 {
		for dt := 0.0; dt <= p.Width; dt += 0.05 {
			for _, sign := range []float64{1, -1} {
				ang := azimuth + sign*dt/p.CageRadius
				if !inBore(g, r3.NewVec(r*math.Cos(ang), r*math.Sin(ang), z)) {
					continue
				}
				ok = true
				if sign > 0 {
					right = math.Max(right, dt)
				} else {
					left = math.Max(left, dt)
				}
			}
		}
	}
	return left, right, ok
}

// ptAt is the point an arc offset from a post's centre line, at the middle of
// the wall's thickness.
func ptAt(p Params, azimuth, t, z float64) r3.Vec {
	ang := azimuth + t/p.CageRadius
	return r3.NewVec(p.CageRadius*math.Cos(ang), p.CageRadius*math.Sin(ang), z)
}

// inBoreAt asks the same question about a point given as an arc offset from a
// post's centre line and a height, at any depth through the wall.
func inBoreAt(g Gear, azimuth, t, z float64) bool {
	p := g.P
	ang := azimuth + t/p.CageRadius
	for r := p.CageInner(); r <= p.CageOuter(); r += 0.1 {
		if inBore(g, r3.NewVec(r*math.Cos(ang), r*math.Sin(ang), z)) {
			return true
		}
	}
	return false
}

// piece is one part of the frame's wall: a stretch of height, and at each
// height a reach to each side round the cage. Everything is a piece of the same
// wall, so the only things that differ between a plate and a post are how tall
// they are and how far round they run.
//
// The two sides are kept apart because a post's bore is not centred on it. The
// channel crosses the wall diagonally, so it wants material to one side low
// down and to the other side higher up, and a shape that reaches equally both
// ways carries the worse of the two at every height for nothing.
type piece struct {
	azimuth     float64
	zLo, zHi    float64
	step        float64
	left, right []float64 // angle reached each way at zLo, zLo+step, ...
	g           *Gear     // the gear bored through it, nil for a plate
}

func (q piece) sideAt(side []float64, z float64) float64 {
	if z < q.zLo || z > q.zHi {
		return 0
	}
	i := (z - q.zLo) / q.step
	lo := int(math.Floor(i))
	if lo >= len(side)-1 {
		return side[len(side)-1]
	}
	f := i - float64(lo)
	return side[lo]*(1-f) + side[lo+1]*f
}

// widthAt is what the piece spans at one height, both sides together.
func (q piece) widthAt(z float64) float64 {
	return q.sideAt(q.left, z) + q.sideAt(q.right, z)
}

func (q piece) maxHalfAngle() float64 {
	worst := 0.0
	for _, side := range [][]float64{q.left, q.right} {
		for _, h := range side {
			worst = math.Max(worst, h)
		}
	}
	return worst
}

func (q piece) holds(p Params, pt r3.Vec) bool {
	r := math.Hypot(pt.X, pt.Y)
	if r > p.CageOuter() || r < p.CageInner() {
		return false
	}
	if pt.Z < q.zLo || pt.Z > q.zHi {
		return false
	}
	d := math.Mod(math.Atan2(pt.Y, pt.X)-q.azimuth+3*math.Pi, 2*math.Pi) - math.Pi
	if d >= 0 {
		return d <= q.sideAt(q.right, pt.Z)
	}
	return -d <= q.sideAt(q.left, pt.Z)
}

// postPiece is one post: a column running the full height, which widens where
// its bore needs it and narrows to PostWidth everywhere else, so it meets the
// plates at both ends with no step.
//
// The block round the bore is SCULPTED TO THE CHANNEL rather than squared off
// round it. The channel crosses the post diagonally — far to one side low down,
// as far to the other side higher up, narrow in between — so one upright box
// big enough for all of it is half again as wide as any single height asks for,
// and that extra is what made the frame look heavy.
//
// What the block must not become is a wall cut to the bore's own outline. That
// outline wiggles, and a shape following it is a row of notches. Three things
// keep this one smooth: each side is grown from the bore by a DISC, which
// rounds every corner off; each side then rises to ONE widest stretch and comes
// back, so there is a single bulge and no second one; and each side is finally
// limited to 45 degrees, which is the steepest a printer will build. Every face
// is either upright or a 45 degree ramp.
//
// What the block has to clear is measured rather than derived, because the bore
// is a twisted channel through a wall it is not aligned with.
func postPiece(g Gear, station float64) piece {
	p := g.P
	const step = 0.1
	a := postAzimuth(g, station)
	n := int(math.Round(2*p.CageRise/step)) + 1

	// How far the bore reaches each way from the post's centre line, height by
	// height, and whether it is there at all.
	left, right := make([]float64, n), make([]float64, n)
	bored := make([]bool, n)
	for j := range n {
		left[j], right[j], bored[j] = boreReach(g, a, -p.CageRise+float64(j)*step)
	}

	// Grow each side out of the bore by a disc of the wall thickness. Adding the
	// wall sideways alone would leave less than it where the bore's edge runs
	// diagonally, since what a wall has to be thick in is the direction across
	// itself, not the direction the measurement happened to be taken in. A disc
	// leaves the full wall whichever way it is measured, and it cannot produce a
	// corner sharper than the disc.
	// The disc is grown by a hair more than the wall. The profile is held at
	// heights one step apart and read as a straight line between them, and a
	// straight line between two points of a circle runs inside it, so the wall
	// would come out a shade thin between two sampled heights. The margin below
	// is the sag of a step of that length off a circle of this radius; the
	// deepest sag TestBoresKeepTheirWall finds without it is 0.5 um, and with it
	// the 3 mm holds outright.
	wall := p.BlockWall + step*step/(2*p.BlockWall)
	grow := func(side []float64) []float64 {
		out := make([]float64, n)
		for j := range n {
			out[j] = p.PostWidth / 2
			for k := range n {
				if !bored[k] {
					continue
				}
				dz := math.Abs(float64(k-j)) * step
				if dz > wall {
					continue
				}
				out[j] = math.Max(out[j], side[k]+math.Sqrt(wall*wall-dz*dz))
			}
		}
		return out
	}

	// One bulge per side: out to the widest stretch, and back. Filling in
	// anything that dips between two wider heights is what rules out a second
	// bulge, which is the shape that reads as a notch.
	single := func(side []float64) []float64 {
		out := make([]float64, n)
		run := 0.0
		for j := range n { // rising to the widest stretch, filling any dip on the way
			run = math.Max(run, side[j])
			out[j] = run
		}
		run = 0
		for j := n - 1; j >= 0; j-- { // and falling away from it
			run = math.Max(run, side[j])
			out[j] = math.Min(out[j], run)
		}
		return out
	}

	// Slant what is left at 45 degrees.
	//
	// Only the underside has to be slanted. A print is built upward, so what
	// will not bridge is material appearing above nothing: widening as the post
	// rises is that case, while narrowing again rests on what is under it and
	// would print as a square shelf. The top is slanted to match the bottom
	// because the part reads better for it, and it costs only height.
	//
	// Taking each height's reach as the largest any other height demands, less
	// the distance between them, is exactly a 45 degree slant either side.
	ramp := func(side []float64) []float64 {
		out := make([]float64, n)
		for j := range n {
			want := side[j]
			for k := range n {
				want = math.Max(want, side[k]-math.Abs(float64(k-j))*step)
			}
			out[j] = want / p.CageRadius
		}
		return out
	}

	shape := func(side []float64) []float64 { return ramp(single(grow(side))) }
	return piece{azimuth: a, zLo: -p.CageRise, zHi: p.CageRise, step: step,
		left: shape(left), right: shape(right), g: &g}
}

// platePieces are the two end plates: the whole way round, on the same wall.
func platePieces(p Params) [2]piece {
	full := []float64{math.Pi, math.Pi}
	return [2]piece{
		{azimuth: 0, zLo: -p.CageRise, zHi: -p.CageRise + p.PlateThick,
			step: p.PlateThick, left: full, right: full},
		{azimuth: 0, zLo: p.CageRise - p.PlateThick, zHi: p.CageRise,
			step: p.PlateThick, left: full, right: full},
	}
}

// cagePieces is the whole frame: two plates and four posts.
func cagePieces(ga, gb Gear) []piece {
	p := ga.P
	out := make([]piece, 0, 6)
	out = append(out, platePieces(p)[0], platePieces(p)[1])
	for i := range 2 {
		g := []Gear{ga, gb}[i]
		for _, station := range boreStations(p) {
			out = append(out, postPiece(g, station))
		}
	}
	return out
}

// inCage answers whether a point is inside any material of the frame.
func inCage(ga, gb Gear, pieces []piece, pt r3.Vec) bool {
	p := ga.P
	for _, q := range pieces {
		if !q.holds(p, pt) {
			continue
		}
		if q.g != nil && inBore(*q.g, pt) {
			continue
		}
		if q.g == nil && (inBore(ga, pt) || inBore(gb, pt)) {
			continue
		}
		return true
	}
	return false
}

// eachRibbonPoint walks a ribbon's whole surface.
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

// One gear's posts are bored low and the other's high, so the two ribbons pass
// the cage at different heights and their teeth meet in the middle.
func TestBoresSitOnOppositeSidesOfTheMiddle(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	for _, station := range boreStations(p) {
		a := ga.Origin.Add(ga.Ez.Scale(station))
		b := gb.Origin.Add(gb.Ez.Scale(station))
		if a.Z >= 0 {
			t.Errorf("gear A's bore at station %.1f sits at height %.2f, not below the middle",
				station, a.Z)
		}
		if b.Z <= 0 {
			t.Errorf("gear B's bore at station %.1f sits at height %.2f, not above the middle",
				station, b.Z)
		}
	}
}

// A post runs from plate to plate with no break in it.
//
// It does NOT have to be back to its plain width by the time it reaches a
// plate. A plate runs the whole way round, so a post still widening where it
// meets one merges into material that is already there: no step, no gap, and
// nothing unsupported. Requiring the ramp to finish first only made the cage
// taller for nothing.
func TestPostsRunUnbrokenIntoThePlates(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P
	floor := p.PostWidth / 2 / p.CageRadius

	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			q := postPiece(g, station)
			var wideLo, wideHi float64
			for z := -p.CageRise; z <= p.CageRise; z += 0.05 {
				h := math.Min(q.sideAt(q.left, z), q.sideAt(q.right, z))
				if h < floor-1e-9 {
					t.Fatalf("the post at %.0f degrees reaches only %.3f mm to one side at height "+
						"%.2f, which is a gap in it", q.azimuth*180/math.Pi, h*p.CageRadius, z)
				}
				if q.widthAt(z) > 2*floor+1e-9 {
					if wideLo == 0 {
						wideLo = z
					}
					wideHi = z
				}
			}
			_, _ = wideLo, wideHi
		}
	}
	t.Logf("each post runs the full %.1f mm, widening only round its bore", 2*p.CageRise)
}

// Nothing on the frame may hang off nothing. A filament printer will not bridge
// a surface shallower than 45 degrees, and the one place the frame could offer
// one is where a post widens as it rises.
//
// Only widening counts. A print is built upward, so what will not bridge is
// material appearing above nothing; where a bulge ends and the post narrows
// again, what is left rests on what is under it, and that shelf prints.
func TestPostsNeverOverhang(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			q := postPiece(g, station)
			for _, side := range [][]float64{q.left, q.right} {
				for j := 1; j < len(side); j++ {
					rise := q.step
					run := (side[j] - side[j-1]) * p.CageRadius
					if run > rise+1e-9 {
						t.Fatalf("the post at %.0f degrees widens %.3f mm over %.3f mm of height, "+
							"which is steeper than 45 degrees", q.azimuth*180/math.Pi, run, rise)
					}
				}
			}
		}
	}
	t.Logf("no post widens faster than 45 degrees")
}

// A bore needs material round it, or the frame is a shell where it is most
// worked. The wall has to be there IN EVERY DIRECTION, not only sideways: what
// a wall is thick in is the direction across itself, and the bore's edge runs
// diagonally over most of its height, so a sideways measurement there reports
// more than the material really is. This walks a disc of the wall thickness
// round the bore's edge and requires every point of it to be material, which is
// the same disc the post is grown by.
func TestBoresKeepTheirWall(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	if p.BlockWall < 3 {
		t.Fatalf("the wall round a bore is %.2f mm, under the 3 mm a printed frame needs",
			p.BlockWall)
	}

	plates := platePieces(p)
	worst := math.Inf(1)
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			q := postPiece(g, station)
			material := func(t float64, z float64) bool {
				pt := ptAt(p, q.azimuth, t, z)
				return q.holds(p, pt) || plates[0].holds(p, pt) || plates[1].holds(p, pt)
			}
			for j := range int(math.Round((q.zHi-q.zLo)/q.step)) + 1 {
				z := q.zLo + float64(j)*q.step
				l, r, ok := boreReach(g, q.azimuth, z)
				if !ok {
					continue
				}
				// A hair inside the wall, because the post is grown by exactly
				// this disc: its edge and the disc agree to the last bit over
				// the stretch where the bore's own edge is steepest, and a
				// strict comparison there is a coin toss on rounding.
				probe := p.BlockWall - 1e-9
				for _, edge := range []float64{-l, r} {
					for a := 0.0; a < 2*math.Pi; a += math.Pi / 60 {
						dt, dz := probe*math.Cos(a), probe*math.Sin(a)
						if inBoreAt(g, q.azimuth, edge+dt, z+dz) {
							continue // still the bore itself
						}
						if !material(edge+dt, z+dz) {
							t.Fatalf("at height %.2f the bore's edge has under %.2f mm of material "+
								"%.0f degrees round it", z, p.BlockWall, a*180/math.Pi)
						}
					}
				}
				// How much wall there really is, taken as the nearest material
				// edge to the bore's edge at this height.
				for _, pair := range [][2]float64{{-l, -q.sideAt(q.left, z) * p.CageRadius},
					{r, q.sideAt(q.right, z) * p.CageRadius}} {
					worst = math.Min(worst, math.Abs(pair[1]-pair[0]))
				}
			}
		}
	}
	t.Logf("every bore keeps at least %.1f mm of wall, %.2f mm of it sideways", p.BlockWall, worst)
}

// Each side of a post widens once and narrows once. That is what keeps the
// block a block: a side that went out, came back and went out again would read
// as a notch cut into the post, which is the shape a wall following the bore's
// own wiggling outline produces and the reason this one is grown from the bore
// rather than traced round it.
func TestPostSidesHaveOneBulge(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			q := postPiece(g, station)
			for what, side := range map[string][]float64{"left": q.left, "right": q.right} {
				turns := 0
				for j := 2; j < len(side); j++ {
					was, now := side[j-1]-side[j-2], side[j]-side[j-1]
					if was > 1e-12 && now < -1e-12 {
						turns++
					}
				}
				if turns > 1 {
					t.Errorf("the %s side of the post at %.0f degrees widens and narrows %d times "+
						"over its height, which is a notch rather than a block",
						what, q.azimuth*180/math.Pi, turns+1)
				}
			}
		}
	}
	t.Logf("every post side carries one bulge")
}

// The bore is not the twisted channel the model describes. Fusion has no twist
// for a solid, so the build CUTS it with a loft through a handful of rotated
// rectangles, and a loft is flat between its sections: the wall is faceted, and
// every facet stands a little inside the true channel. What that costs is
// clearance, because the facet takes its bite out of the gap the boss passes
// through, and enough of it would bind the gear.
//
// This measures the bite. It walks the stations where the ribbon meets that
// post's material, builds the lofted opening there as the build would, and asks
// how much room is left round the boss.
//
// SPANNING ONLY THE MATERIAL IS WHAT MAKES THIS CHEAP. The ribbon crosses a post
// over about 4 mm of its own length and turns some 44 degrees doing it. Lofting
// the whole post's height instead would be 36 mm and 392 degrees, and the same
// section count over that span leaves the channel pinched shut.
func TestBoreLoftKeepsItsClearance(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	worst := math.Inf(1)
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			lo, hi := boreSpan(g, station)
			turn := (hi - lo) / p.Lambda()
			n := boreSections(turn)
			left := boreLoftClearance(g, lo, hi, n)
			if left < 0.95*p.Clearance {
				t.Errorf("the bore at station %+.1f lofts through %d sections %.1f degrees apart and "+
					"leaves the boss %.4f mm, under the %.4f mm that is 95%% of the clearance",
					station, n, turn/float64(n-1)*180/math.Pi, left, 0.95*p.Clearance)
			}
			worst = math.Min(worst, left)
		}
	}

	// What the spec used to fix at five sections, for the record.
	g := ga
	lo, hi := boreSpan(g, boreStations(p)[0])
	t.Logf("the lofted bore leaves the boss %.4f mm of the %.2f mm clearance; five sections would "+
		"leave %.4f mm", worst, p.Clearance, boreLoftClearance(g, lo, hi, 5))
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

// boreSpan is the stretch of a gear's own axis over which its bore has post
// material to cut, with a millimetre of margin at each end. Outside it the
// ribbon is in open air and there is nothing to remove.
func boreSpan(g Gear, station float64) (lo, hi float64) {
	p := g.P
	q := postPiece(g, station)
	lo, hi = math.Inf(1), math.Inf(-1)
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	for s := station - 4*p.Width; s <= station+4*p.Width; s += 0.01 {
		for cu := -hw; cu <= hw; cu += 0.25 {
			for cv := -ht; cv <= ht; cv += 0.25 {
				if q.holds(p, g.world(cu, cv, s)) {
					lo, hi = math.Min(lo, s), math.Max(hi, s)
				}
			}
		}
	}
	return lo - 1, hi + 1
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

// The widening has to earn its material: a post that is as wide at its ends as
// it is at its bore is carrying weight for nothing.
//
// This also reports what the four posts take out of the cage wall altogether,
// which is the measure of the shaping. A block squared off round the whole
// channel, which is what this frame carried before, comes to 1804 mm2 against
// the 1181 mm2 here, and is 19.40 mm at its widest against 17.23 mm.
func TestPostsAreNarrowAwayFromTheirBores(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	atEnd, widest, area := 0.0, 0.0, 0.0
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			q := postPiece(g, station)
			end := q.widthAt(p.CageRise-p.PlateThick) * p.CageRadius
			mine := 0.0
			for z := q.zLo; z <= q.zHi; z += q.step {
				w := q.widthAt(z) * p.CageRadius
				mine = math.Max(mine, w)
				area += w * q.step
			}
			if end > mine/2 {
				t.Errorf("the post at %.0f degrees is %.2f mm wide at the plate against %.2f at its "+
					"bore, which is not much of a saving", q.azimuth*180/math.Pi, end, mine)
			}
			atEnd, widest = math.Max(atEnd, end), math.Max(widest, mine)
		}
	}
	t.Logf("a post runs %.2f mm wide, widening to at most %.2f mm round its bore; the four take "+
		"%.0f mm2 out of the wall", atEnd, widest, area)
}

// Nothing may stand outside the frame's one cylinder. That surface is what
// makes the outside read as turned rather than assembled, and a corner poking
// through it is the defect this rules out.
func TestNothingStandsProudOfTheShell(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P
	pieces := cagePieces(ga, gb)

	for _, q := range pieces {
		for da := -q.maxHalfAngle(); da <= q.maxHalfAngle(); da += 0.01 {
			a := q.azimuth + da
			for z := q.zLo; z <= q.zHi; z += 0.5 {
				for _, r := range []float64{p.CageOuter() + 0.001, p.CageInner() - 0.001} {
					pt := r3.NewVec(r*math.Cos(a), r*math.Sin(a), z)
					if inCage(ga, gb, pieces, pt) {
						t.Fatalf("frame material sits at radius %.3f, outside the wall %.3f to %.3f",
							r, p.CageInner(), p.CageOuter())
					}
				}
			}
		}
	}
	t.Logf("the whole frame lies between radius %.2f and %.2f", p.CageInner(), p.CageOuter())
}

// Each post has to reach both plates, or the frame is not one body.
func TestPostsReachBothPlates(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			a := postAzimuth(g, station)
			post := postPiece(g, station)
			for _, z := range []float64{-p.CageRise + p.PlateThick/2, p.CageRise - p.PlateThick/2} {
				pt := r3.NewVec(p.CageRadius*math.Cos(a), p.CageRadius*math.Sin(a), z)
				if !post.holds(p, pt) {
					t.Errorf("the post at %.1f degrees does not reach the plate at height %.1f",
						a*180/math.Pi, z)
				}
				met := false
				for _, plate := range platePieces(p) {
					if plate.holds(p, pt) {
						met = true
					}
				}
				if !met {
					t.Errorf("no plate stands at height %.1f where the post reaches it", z)
				}
			}
		}
	}
}

// Every part of both ribbons has to miss every part of the frame. The bores are
// the only places they come near, and even there they must not touch.
func TestRibbonsClearTheCage(t *testing.T) {
	ga, gb := defaultPair()
	pieces := cagePieces(ga, gb)

	for _, g := range []Gear{ga, gb} {
		var hit bool
		var at r3.Vec
		var atS float64
		eachRibbonPoint(g, 0.02, func(pt r3.Vec, _, s float64) {
			if !hit && inCage(ga, gb, pieces, pt) {
				hit, at, atS = true, pt, s
			}
		})
		if hit {
			t.Fatalf("a ribbon meets the frame at station %.2f, (%.2f, %.2f, %.2f)",
				atS, at.X, at.Y, at.Z)
		}
	}
}

// The middle of the cage has to stay open, or the mesh cannot be seen and the
// teeth have nothing to meet in.
func TestTheMiddleStaysOpen(t *testing.T) {
	ga, gb := defaultPair()
	pieces := cagePieces(ga, gb)
	window := axialWindow(ga.P)

	for x := -window; x <= window; x += 0.2 {
		for y := -window; y <= window; y += 0.2 {
			for z := -window; z <= window; z += 0.2 {
				if inCage(ga, gb, pieces, r3.NewVec(x, y, z)) {
					t.Fatalf("the frame reaches into the meshing space at (%.1f, %.1f, %.1f)", x, y, z)
				}
			}
		}
	}
	t.Logf("nothing of the frame is within %.2f mm of the middle", window)
}

// This is the frame's own proof: it admits the screw motion and nothing else.
// The test turns a gear out of step with its own advance and finds the angle at
// which its boss jams in the bores.
//
// A frame of round holes would report no jam at any angle, and that is the case
// this rules out.
func TestBoresAdmitOnlyTheScrewMotion(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P
	pieces := cagePieces(ga, gb)

	fits := func(extra float64) bool {
		g := ga
		g.Mount += extra
		clear := true
		eachRibbonPoint(g, 0.05, func(pt r3.Vec, _, _ float64) {
			if clear && inCage(ga, gb, pieces, pt) {
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
		t.Fatal("the gear turns freely in the cage: the bores are not holding it to the screw " +
			"motion, which is the one thing the frame is for")
	}
	if got := slack * 180 / math.Pi; got > 12 {
		t.Errorf("the cage lets the gear turn %.1f degrees out of step, which is more play than a "+
			"mechanism of one degree of freedom can be said to have", got)
	}
	t.Logf("the cage jams the gear %.2f degrees out of step, on %.2f mm of clearance",
		slack*180/math.Pi, p.Clearance)
}

// How far a bore stands from upright, which is what decides whether it prints.
//
// A bore is a hole through a post, and the posts stand along the cage axis,
// which is the axis a print stands on. A bore is therefore a horizontal hole
// whose ceiling has to be bridged, and a bore whose opening is TALL and narrow
// bridges a short span where a wide flat one leaves a ceiling as wide as the
// ribbon.
//
// There are four of them: each gear crosses twice, at plus and minus the cage
// radius, and those two are turned in opposite directions. Upright at all four
// needs the two mounting angles equal AND the cage radius a whole number of
// half turns of the ribbon, and even then the four sit at plus and minus the
// mounting angle. So the best any radius can do is the mounting angle itself.
func TestBoresStandNearlyUpright(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	worst := 0.0
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			theta := g.angle(station)
			off := math.Mod(math.Abs(theta), math.Pi)
			worst = math.Max(worst, math.Min(off, math.Pi-off))
		}
	}
	if got := worst * 180 / math.Pi; got > 20 {
		t.Errorf("a bore stands %.1f degrees off upright, which leaves a ceiling too wide to "+
			"bridge on a filament printer", got)
	}
	best := math.Pi
	for x := 0.0; x < math.Pi; x += math.Pi / 3600 {
		d := 0.0
		for _, phi := range []float64{p.MountAngleA, p.MountAngleB} {
			for _, sign := range []float64{1, -1} {
				off := math.Mod(math.Abs(sign*x+phi), math.Pi)
				d = math.Max(d, math.Min(off, math.Pi-off))
			}
		}
		best = math.Min(best, d)
	}
	if worst > best+0.5*math.Pi/180 {
		t.Errorf("the bores stand %.1f degrees off upright where %.1f is available: the cage "+
			"radius is not where it should be", worst*180/math.Pi, best*180/math.Pi)
	}
	t.Logf("bores stand %.1f degrees off upright, against %.1f the mounting angles allow",
		worst*180/math.Pi, best*180/math.Pi)
}

// The boss travels with its gear, so its length is the stroke: the mechanism
// runs only while the boss still fills the bores. The bore's length along the
// ribbon is the block's radial depth taken on the slant, because a gear's axis
// is not quite radial where it crosses.
func TestStrokeIsTheBossLength(t *testing.T) {
	ga, _ := defaultPair()
	p := ga.P

	bore := p.ShellThick / math.Cos(math.Asin(p.AxisOffset()/2/p.CageRadius))
	stroke := 2 * (p.BossHalf - p.BossTaper - bore/2)
	if stroke <= 0 {
		t.Fatalf("the boss is %.2f mm long and the bore %.2f mm deep, so there is no travel",
			2*p.BossHalf, bore)
	}
	if teeth := stroke / p.ToothPitch; teeth < 2 {
		t.Errorf("the stroke is %.2f mm, only %.1f teeth, too short to show a gear working",
			stroke, teeth)
	}
	t.Logf("bore %.2f mm along the ribbon, stroke %.2f mm, which is %.1f teeth",
		bore, stroke, stroke/p.ToothPitch)
}

// A tooth must never reach a bore, or the frame would need an opening shaped
// like a tooth and the boss would be pointless.
func TestTeethNeverReachABore(t *testing.T) {
	ga, _ := defaultPair()
	p := ga.P

	bore := p.ShellThick / math.Cos(math.Asin(p.AxisOffset()/2/p.CageRadius))
	stroke := p.BossHalf - p.BossTaper - bore/2
	for _, station := range boreStations(p) {
		for d := -stroke; d <= stroke; d += 0.05 {
			for s := station - bore/2; s <= station+bore/2; s += 0.02 {
				g := ga
				g.Phase = d
				if b := g.boss(s); b < p.BossGrow-1e-9 {
					t.Fatalf("at travel %.2f mm the ribbon inside the bore at %.1f stands only "+
						"%.3f mm proud, so a tooth is in the bore", d, station, b)
				}
			}
		}
	}
}
