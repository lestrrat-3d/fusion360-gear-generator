package screwgear_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// The frame is a cylinder with most of its wall gone: a ring at the top, a ring
// at the bottom, and four posts standing between them. Each post sits where one
// ribbon crosses the cylinder, and each carries a BORE that ribbon passes
// through. The two posts of one gear are bored low, the two of the other high,
// and the gears meet in the middle of the cylinder.
//
// The bore is what holds a gear to its screw motion. It is the ribbon's own
// cross-section, turned to the angle the ribbon has there and twisted through
// the post at the ribbon's own lead, so a gear that turns without advancing
// jams in it. A round hole would not, and the mechanism would have three
// degrees of freedom instead of one.
//
// What passes through a bore is never a tooth. The ribbon carries a smooth boss
// at each of the two places it crosses the cage, the bore is cut to that boss,
// and the teeth stay outside it. That is what keeps the frame free of any
// opening shaped like a tooth, and why nothing bears on a crest.

// postAzimuth is where a gear's post stands, for the bore at the given station.
func postAzimuth(g Gear, station float64) float64 {
	c := g.Origin.Add(g.Ez.Scale(station))
	return math.Atan2(c.Y, c.X)
}

// inBore answers whether a point is inside the channel cut through a post for
// this gear. The channel follows the ribbon, so it is twisted rather than
// straight, and it runs the whole height of the post rather than only the
// depth of the block: a post is solid bar above and below its block, and the
// ribbon has to get past that too.
func inBore(g Gear, pt r3.Vec) bool {
	p := g.P
	u, v, _ := g.local(pt)
	return math.Abs(u) <= p.BoreHalfWidth() && math.Abs(v) <= p.BoreHalfThickness()
}

// inBlock answers whether a point is inside the material of the block a post
// widens into around its bore.
func inBlock(g Gear, station float64, pt r3.Vec) bool {
	p := g.P
	u, v, s := g.local(pt)
	if math.Abs(s-station) > p.BlockDepth/2 {
		return false
	}
	if math.Abs(u) > p.BoreHalfWidth()+p.BlockWall || math.Abs(v) > p.BoreHalfThickness()+p.BlockWall {
		return false
	}
	return !inBore(g, pt)
}

// inPost answers whether a point is inside the slender part of a post: a round
// bar standing at the cage radius, from the bottom ring to the top.
func inPost(p Params, azimuth float64, pt r3.Vec) bool {
	if math.Abs(pt.Z) > p.CageRise {
		return false
	}
	c := r3.NewVec(p.CageRadius*math.Cos(azimuth), p.CageRadius*math.Sin(azimuth), pt.Z)
	return pt.Sub(c).Len() <= p.PostBar/2
}

func inRing(p Params, pt r3.Vec) bool {
	if d := math.Abs(math.Abs(pt.Z) - p.CageRise); d > p.RingBar/2 {
		return false
	}
	return math.Abs(math.Hypot(pt.X, pt.Y)-p.CageRadius) <= p.RingBar/2
}

// inCage answers whether a point is inside any material of the frame.
func inCage(ga, gb Gear, pt r3.Vec) bool {
	p := ga.P
	if inRing(p, pt) {
		return true
	}
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			if inBlock(g, station, pt) {
				return true
			}
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
	t.Logf("bores at heights %.2f and %.2f, on a cage %.1f mm tall",
		-p.AxisOffset()/2, p.AxisOffset()/2, 2*p.CageRise)
}

// Each post has to reach both rings, or the frame is not one body.
func TestPostsReachBothRings(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			a := postAzimuth(g, station)
			for _, z := range []float64{-p.CageRise, p.CageRise} {
				pt := r3.NewVec(p.CageRadius*math.Cos(a), p.CageRadius*math.Sin(a), z)
				if !inPost(p, a, pt) || !inRing(p, pt) {
					t.Errorf("the post at %.1f degrees does not meet the ring at height %.1f",
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

	for _, g := range []Gear{ga, gb} {
		var hit bool
		var at r3.Vec
		var atS float64
		eachRibbonPoint(g, 0.02, func(pt r3.Vec, _, s float64) {
			if !hit && inCage(ga, gb, pt) {
				hit, at, atS = true, pt, s
			}
		})
		if hit {
			t.Fatalf("a ribbon meets the frame at station %.2f, (%.2f, %.2f, %.2f)",
				atS, at.X, at.Y, at.Z)
		}
	}
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

	fits := func(extra float64) bool {
		g := ga
		g.Mount += extra
		clear := true
		eachRibbonPoint(g, 0.05, func(pt r3.Vec, _, _ float64) {
			if clear && inCage(ga, gb, pt) {
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
// A bore is a hole through a post, and the post stands along the cage axis. A
// model like this is printed with that axis vertical, so the bore is a
// horizontal hole and its ceiling has to be bridged. A bore whose opening is
// TALL and narrow bridges a short span; one whose opening is wide and flat
// leaves a ceiling as wide as the ribbon, and that sags.
//
// The opening's angle is the ribbon's cross-section angle where it crosses the
// cage, which is CageRadius/Lambda plus or minus that gear's mounting angle,
// and there are FOUR of them: each gear crosses twice, at plus and minus the
// cage radius, and those two are turned in opposite directions.
//
// Upright at all four needs the two mounting angles equal AND the cage radius a
// whole number of half turns of the ribbon, and even then the four sit at plus
// and minus the mounting angle. So the best any radius can do is the mounting
// angle itself, and the smallest equal mounting angle that drives at this twist
// is 15 degrees. CageRadius = Lambda*pi is what puts the four there.
func TestBoresStandNearlyUpright(t *testing.T) {
	ga, gb := defaultPair()
	p := ga.P

	worst := 0.0
	for _, g := range []Gear{ga, gb} {
		for _, station := range boreStations(p) {
			theta := g.angle(station)
			off := math.Mod(math.Abs(theta), math.Pi)
			off = math.Min(off, math.Pi-off)
			worst = math.Max(worst, off)
		}
	}
	if got := worst * 180 / math.Pi; got > 20 {
		t.Errorf("a bore stands %.1f degrees off upright, which leaves a ceiling too wide to "+
			"bridge on a filament printer", got)
	}
	// What the best cage radius could do, searched rather than argued: the four
	// bores sit at plus and minus CageRadius/Lambda off each mounting angle, so
	// moving the radius trades one pair against the other.
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
// runs only while the boss still fills the bores.
func TestStrokeIsTheBossLength(t *testing.T) {
	ga, _ := defaultPair()
	p := ga.P

	stroke := 2 * (p.BossHalf - p.BossTaper - p.BlockDepth/2)
	if stroke <= 0 {
		t.Fatalf("the boss is %.2f mm long and the bore %.2f mm deep, so there is no travel",
			2*p.BossHalf, p.BlockDepth)
	}
	if teeth := stroke / p.ToothPitch; teeth < 2 {
		t.Errorf("the stroke is %.2f mm, only %.1f teeth, too short to show a gear working",
			stroke, teeth)
	}
	t.Logf("stroke %.2f mm, which is %.1f teeth", stroke, stroke/p.ToothPitch)
}

// A tooth must never reach a bore, or the frame would need an opening shaped
// like a tooth and the boss would be pointless.
func TestTeethNeverReachABore(t *testing.T) {
	ga, _ := defaultPair()
	p := ga.P

	stroke := p.BossHalf - p.BossTaper - p.BlockDepth/2
	for _, station := range boreStations(p) {
		for d := -stroke; d <= stroke; d += 0.05 {
			for s := station - p.BlockDepth/2; s <= station+p.BlockDepth/2; s += 0.02 {
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
