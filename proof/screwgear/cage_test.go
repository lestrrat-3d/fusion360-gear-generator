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

// piece is one part of the frame's wall: a stretch of height, and at each
// height a half-width round the cage. Everything is a piece of the same wall,
// so the only things that differ between a plate and a post are how tall they
// are and how wide they run.
type piece struct {
	azimuth  float64
	zLo, zHi float64
	step     float64
	half     []float64 // half-angle at zLo, zLo+step, ...
	g        *Gear     // the gear bored through it, nil for a plate
}

func (q piece) halfAngleAt(z float64) float64 {
	if z < q.zLo || z > q.zHi {
		return 0
	}
	i := (z - q.zLo) / q.step
	lo := int(math.Floor(i))
	if lo >= len(q.half)-1 {
		return q.half[len(q.half)-1]
	}
	f := i - float64(lo)
	return q.half[lo]*(1-f) + q.half[lo+1]*f
}

func (q piece) maxHalfAngle() float64 {
	worst := 0.0
	for _, h := range q.half {
		worst = math.Max(worst, h)
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
	d := math.Mod(math.Abs(math.Atan2(pt.Y, pt.X)-q.azimuth), 2*math.Pi)
	return math.Min(d, 2*math.Pi-d) <= q.halfAngleAt(pt.Z)
}

// postPiece is one post: a column running the full height, which widens where
// its bore needs it and narrows to PostWidth everywhere else, so it meets the
// plates at both ends with no step.
//
// The widening is a PLAIN BOX: one width, held over the bore's whole height,
// with a 45 degree ramp at each end down to the plain post. It does not follow
// the bore's own outline height by height. A twisted bore's outline zig-zags,
// and a wall cut to it would be a row of notches — weaker, uglier, and harder
// to print than the straight wall that costs a little more material.
//
// What the box has to be is measured rather than derived, because the bore is a
// twisted channel through a wall it is not aligned with.
func postPiece(g Gear, station float64) piece {
	p := g.P
	const step = 0.1
	a := postAzimuth(g, station)
	n := int(math.Round(2*p.CageRise/step)) + 1
	half := make([]float64, n)
	for j := range n {
		z := -p.CageRise + float64(j)*step
		widest := 0.0
		for r := p.CageInner(); r <= p.CageOuter(); r += 0.1 {
			for dt := 0.0; dt <= p.Width; dt += 0.05 {
				ang := a + dt/p.CageRadius
				if inBore(g, r3.NewVec(r*math.Cos(ang), r*math.Sin(ang), z)) {
					widest = math.Max(widest, dt)
				}
				ang = a - dt/p.CageRadius
				if inBore(g, r3.NewVec(r*math.Cos(ang), r*math.Sin(ang), z)) {
					widest = math.Max(widest, dt)
				}
			}
		}
		half[j] = 0
		if widest > 0 {
			half[j] = widest + p.BlockWall
		}
	}

	// Square the bulge off: one width over one stretch of height, both taken
	// from what the bore needs at its worst.
	wide, lo, hi := 0.0, math.Inf(1), math.Inf(-1)
	for j, h := range half {
		if h == 0 {
			continue
		}
		z := -p.CageRise + float64(j)*step
		wide = math.Max(wide, h)
		lo, hi = math.Min(lo, z), math.Max(hi, z)
	}
	for j := range half {
		z := -p.CageRise + float64(j)*step
		half[j] = p.PostWidth / 2
		if z >= lo-p.BlockWall && z <= hi+p.BlockWall {
			half[j] = math.Max(half[j], wide)
		}
	}

	// Hold the widening to 45 degrees, on the underside only.
	//
	// A print is built upward, so material that appears above nothing is what
	// will not bridge. Widening as the post rises is that case and is ramped;
	// narrowing again is not, because what is left rests on what is under it,
	// so a bulge may end in a flat shelf. Taking each height's width as the
	// largest any height ABOVE demands, less the distance up to it, is exactly
	// a 45 degree ramp under every bulge and a square top on it.
	ramped := make([]float64, n)
	for j := range n {
		want := half[j]
		for k := j; k < n; k++ {
			want = math.Max(want, half[k]-float64(k-j)*step)
		}
		ramped[j] = want / p.CageRadius
	}
	return piece{azimuth: a, zLo: -p.CageRise, zHi: p.CageRise, step: step, half: ramped, g: &g}
}

// platePieces are the two end plates: the whole way round, on the same wall.
func platePieces(p Params) [2]piece {
	full := []float64{math.Pi, math.Pi}
	return [2]piece{
		{azimuth: 0, zLo: -p.CageRise, zHi: -p.CageRise + p.PlateThick,
			step: p.PlateThick, half: full},
		{azimuth: 0, zLo: p.CageRise - p.PlateThick, zHi: p.CageRise,
			step: p.PlateThick, half: full},
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
				h := q.halfAngleAt(z)
				if h < floor-1e-9 {
					t.Fatalf("the post at %.0f degrees is only %.3f wide at height %.2f, which is "+
						"a gap in it", q.azimuth*180/math.Pi, h*p.CageRadius*2, z)
				}
				if h > floor+1e-9 {
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
			for j := 1; j < len(q.half); j++ {
				rise := q.step
				run := (q.half[j] - q.half[j-1]) * p.CageRadius
				if run > rise+1e-9 {
					t.Fatalf("the post at %.0f degrees widens %.3f mm over %.3f mm of height, "+
						"which is steeper than 45 degrees", q.azimuth*180/math.Pi, run, rise)
				}
			}
		}
	}
	t.Logf("no post widens faster than 45 degrees")
}

// A bore needs material round it, or the frame is a shell where it is most
// worked.
func TestBoresKeepTheirWall(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	if p.BlockWall < 3 {
		t.Fatalf("the wall round a bore is %.2f mm, under the 3 mm a printed frame needs",
			p.BlockWall)
	}
	// And the post really is that much wider than its bore at every height.
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			q := postPiece(g, station)
			for j, h := range q.half {
				z := q.zLo + float64(j)*q.step
				widest := 0.0
				for r := p.CageInner(); r <= p.CageOuter(); r += 0.1 {
					for dt := 0.0; dt <= p.Width; dt += 0.05 {
						for _, sign := range []float64{1, -1} {
							ang := q.azimuth + sign*dt/p.CageRadius
							if inBore(g, r3.NewVec(r*math.Cos(ang), r*math.Sin(ang), z)) {
								widest = math.Max(widest, dt)
							}
						}
					}
				}
				if widest == 0 {
					continue
				}
				if wall := h*p.CageRadius - widest; wall < p.BlockWall-1e-9 {
					t.Fatalf("at height %.2f the post leaves %.2f mm round its bore, under %.2f",
						z, wall, p.BlockWall)
				}
			}
		}
	}
	t.Logf("every bore keeps at least %.1f mm of wall", p.BlockWall)
}

// The widening has to earn its material: a post that is as wide at its ends as
// it is at its bore is carrying weight for nothing.
func TestPostsAreNarrowAwayFromTheirBores(t *testing.T) {
	ga, _ := defaultPair()
	p := ga.P
	q := postPiece(ga, boreStations(p)[1])

	atEnd := q.halfAngleAt(p.CageRise-p.PlateThick) * p.CageRadius * 2
	widest := q.maxHalfAngle() * p.CageRadius * 2
	if atEnd > widest/2 {
		t.Errorf("a post is %.2f mm wide at the plate against %.2f at its bore, which is not much "+
			"of a saving", atEnd, widest)
	}
	t.Logf("a post runs %.2f mm wide, widening to %.2f mm round its bore", atEnd, widest)
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
