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
// Everything OUTSIDE is square to the cage, even though the bore inside is
// not. The end plates have flat faces, and a block is a brick whose faces are
// radial, tangential and level. A part whose outside is square to the build
// plate prints better than one whose every face is skewed, and the skew the
// mechanism needs is all inside the bore.

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

// block is one post's brick, squared to the cage: radial, tangential and level.
type block struct {
	centre         r3.Vec
	radial, tangen r3.Vec
	hr, ht, hz     float64
}

// blockAt sizes the brick around a bore by MEASURING what the bore occupies
// over the block's radial run and adding the wall, rather than by projecting
// the opening through the angles by hand. The bore is a twisted channel through
// a brick that is not aligned with it, so what it occupies in the brick's own
// directions is not a closed form worth trusting.
func blockAt(g Gear, station float64) block {
	p := g.P
	a := postAzimuth(g, station)
	radial := r3.NewVec(math.Cos(a), math.Sin(a), 0)
	tangen := r3.NewVec(-math.Sin(a), math.Cos(a), 0)
	centre := r3.NewVec(p.CageRadius*math.Cos(a), p.CageRadius*math.Sin(a),
		g.Origin.Z)

	var ht, hz float64
	for dr := -p.BlockDepth / 2; dr <= p.BlockDepth/2; dr += 0.05 {
		for dt := -p.Width; dt <= p.Width; dt += 0.05 {
			for dz := -p.Width; dz <= p.Width; dz += 0.05 {
				pt := centre.Add(radial.Scale(dr)).Add(tangen.Scale(dt)).
					Add(r3.NewVec(0, 0, dz))
				if !inBore(g, pt) {
					continue
				}
				ht = math.Max(ht, math.Abs(dt))
				hz = math.Max(hz, math.Abs(dz))
			}
		}
	}
	return block{
		centre: centre, radial: radial, tangen: tangen,
		hr: p.BlockDepth / 2, ht: ht + p.BlockWall, hz: hz + p.BlockWall,
	}
}

// holds answers whether a point is inside the brick's outside shape.
func (b block) holds(pt r3.Vec) bool {
	d := pt.Sub(b.centre)
	return math.Abs(d.Dot(b.radial)) <= b.hr &&
		math.Abs(d.Dot(b.tangen)) <= b.ht &&
		math.Abs(d.Z) <= b.hz
}

// inPlate answers whether a point is inside one of the two end plates: a flat
// annulus, level top and bottom, reaching PlateWall in from the cage radius.
func inPlate(p Params, pt r3.Vec) bool {
	z := math.Abs(pt.Z)
	if z > p.CageRise || z < p.CageRise-p.PlateThick {
		return false
	}
	r := math.Hypot(pt.X, pt.Y)
	return r <= p.CageRadius+p.PostBar/2 && r >= p.CageRadius+p.PostBar/2-p.PlateWall
}

// inPost answers whether a point is inside the slender part of a post: a square
// bar standing at the cage radius, squared to the cage like everything else
// outside, running from the bottom plate to the top.
func inPost(p Params, azimuth float64, pt r3.Vec) bool {
	if math.Abs(pt.Z) > p.CageRise {
		return false
	}
	radial := r3.NewVec(math.Cos(azimuth), math.Sin(azimuth), 0)
	tangen := r3.NewVec(-math.Sin(azimuth), math.Cos(azimuth), 0)
	d := pt.Sub(r3.NewVec(p.CageRadius*math.Cos(azimuth), p.CageRadius*math.Sin(azimuth), 0))
	return math.Abs(d.Dot(radial)) <= p.PostBar/2 && math.Abs(d.Dot(tangen)) <= p.PostBar/2
}

// cageBlocks is every brick on the frame.
func cageBlocks(ga, gb Gear) []struct {
	g Gear
	b block
} {
	var out []struct {
		g Gear
		b block
	}
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(g.P) {
			out = append(out, struct {
				g Gear
				b block
			}{g, blockAt(g, station)})
		}
	}
	return out
}

// inCage answers whether a point is inside any material of the frame.
func inCage(ga, gb Gear, blocks []struct {
	g Gear
	b block
}, pt r3.Vec) bool {
	p := ga.P
	if inPlate(p, pt) {
		return true
	}
	for _, bl := range blocks {
		if bl.b.holds(pt) && !inBore(bl.g, pt) {
			return true
		}
	}
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			if inPost(p, postAzimuth(g, station), pt) && !inBore(g, pt) {
				return true
			}
		}
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

// A block must not push through an end plate. Sticking out past the plate's own
// face leaves a lump on what should be a flat surface, and a print stands on
// that surface.
func TestBlocksStayInsideThePlates(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P
	inner := p.CageRise - p.PlateThick

	for _, bl := range cageBlocks(ga, gb) {
		top := bl.b.centre.Z + bl.b.hz
		bottom := bl.b.centre.Z - bl.b.hz
		if top > inner || bottom < -inner {
			t.Errorf("a block runs from %.2f to %.2f where the plates leave only %.2f either side",
				bottom, top, inner)
		}
	}
	t.Logf("plates leave +/-%.2f mm; the blocks reach +/-%.2f mm",
		inner, blockReach(cageBlocks(ga, gb)))
}

func blockReach(blocks []struct {
	g Gear
	b block
}) float64 {
	worst := 0.0
	for _, bl := range blocks {
		worst = math.Max(worst, math.Abs(bl.b.centre.Z)+bl.b.hz)
	}
	return worst
}

// Each post has to reach both plates, or the frame is not one body.
func TestPostsReachBothPlates(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			a := postAzimuth(g, station)
			for _, z := range []float64{-p.CageRise + p.PlateThick/2, p.CageRise - p.PlateThick/2} {
				pt := r3.NewVec(p.CageRadius*math.Cos(a), p.CageRadius*math.Sin(a), z)
				if !inPost(p, a, pt) || !inPlate(p, pt) {
					t.Errorf("the post at %.1f degrees does not meet the plate at height %.1f",
						a*180/math.Pi, z)
				}
			}
		}
	}
}

// Every part of both ribbons has to miss every part of the frame. The bores are
// the only places they come near, and even there they must not touch.
func TestRibbonsClearTheCage(t *testing.T) {
	ga, gb := defaultPair()
	blocks := cageBlocks(ga, gb)

	for _, g := range []Gear{ga, gb} {
		var hit bool
		var at r3.Vec
		var atS float64
		eachRibbonPoint(g, 0.02, func(pt r3.Vec, _, s float64) {
			if !hit && inCage(ga, gb, blocks, pt) {
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
	blocks := cageBlocks(ga, gb)
	window := axialWindow(ga.P)

	for x := -window; x <= window; x += 0.2 {
		for y := -window; y <= window; y += 0.2 {
			for z := -window; z <= window; z += 0.2 {
				if inCage(ga, gb, blocks, r3.NewVec(x, y, z)) {
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
	blocks := cageBlocks(ga, gb)

	fits := func(extra float64) bool {
		g := ga
		g.Mount += extra
		clear := true
		eachRibbonPoint(g, 0.05, func(pt r3.Vec, _, _ float64) {
			if clear && inCage(ga, gb, blocks, pt) {
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

	bore := p.BlockDepth / math.Cos(math.Asin(p.AxisOffset()/2/p.CageRadius))
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

	bore := p.BlockDepth / math.Cos(math.Asin(p.AxisOffset()/2/p.CageRadius))
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
