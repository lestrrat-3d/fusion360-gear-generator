package screwgear_test

import (
	"math"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// ---------------------------------------------------------------------------
// The printable sleeve: the frame proposed to replace the video's.
//
// The video's frame (cage_test.go) was printed several times and reported on
// 2026-09-30 as not practically printable: its thin rods, wire ring and the
// loop hanging on them wobble while printing. The bores cannot change, since
// they are what holds each gear to its screw motion, and the mesh has to stay
// visible from the top and the bottom. Everything else about the frame may
// change.
//
// The sleeve is one thick-walled tube about the frame's axis, inner radius
// CageRadius - CollarHalf and outer radius CageRadius + CollarHalf, standing
// CageRise either side of the middle plane, with the four twisted bores cut
// straight through its wall and nothing else. Its wall is the collar: the
// bore runs through every bit of material the ribbon passes, so the wall's
// thickness along the bore's centre line is the collar's length, and the far
// face of the bore stays where the video frame's collar had it. The mesh is
// seen along the axis through the hollow from either end, and the frame
// prints standing on either end with no support: every outside face is a
// vertical cylinder or a level end face, and the only downward faces are the
// ceilings of the four bores.
//
// The spec and the add-in still build the video's frame. This file proves the
// sleeve so that the spec can adopt it; nothing here changes defaultParams,
// and every test in cage_test.go still describes the frame that is built.
//
// Three checks the sleeve needs read nothing of the frame but the bores and
// where they sit, and TestSleeveBoresAreTheSameChannels holds those to the
// video frame's: TestRibbonsStayInsideTheirBoresOverTheTravel,
// TestTravelIsTheRibbonBetweenItsCollars and
// TestRibbonsClearEachOtherOutsideTheEngagement hold for the sleeve as they
// stand, and are not repeated here.
// ---------------------------------------------------------------------------

// sleeveCageRise is the sleeve's CageRise: half its height, to its flat end
// faces. The video frame's 20.25 mm is measured to the centre of the ring's
// wire and would cost 3 mm of print height for 1.5 mm more end wall and
// nothing else; 18.75 mm, 1.25 widths, leaves the 3.74 mm end wall
// TestSleeveIsOnePiece measures, against the 18.01 mm the channels need for a
// CollarWall of end wall and the 18.41 mm the build's closed form asks for.
const sleeveCageRise = 18.75

// sleeveParams is the spec's default table with the sleeve's own CageRise.
// Nothing else differs, and TestSleeveBoresAreTheSameChannels holds that.
// RingRadius, RingWire and RodDiameter are the video frame's inputs and the
// sleeve reads none of them.
func sleeveParams() Params {
	p := defaultParams()
	p.CageRise = sleeveCageRise
	return p
}

// SleeveInner and SleeveOuter are the sleeve's radii: the collar's two ends,
// measured on the ribbon's axis from the middle, turned into a tube.
func (p Params) SleeveInner() float64 { return p.CageRadius - p.CollarHalf }
func (p Params) SleeveOuter() float64 { return p.CageRadius + p.CollarHalf }

// BoreCorner is how far the bore's corner stands from the ribbon's axis: the
// radius the channel sweeps out as it twists.
func (p Params) BoreCorner() float64 { return math.Hypot(p.BoreHalfWidth(), p.BoreHalfThickness()) }

// sleeveCutMargin is how far each bore's cut runs past the material, at both
// ends, so the cut starts and finishes in air.
const sleeveCutMargin = 1

// SleeveCut is the span each bore's cut covers on its own axis, as a distance
// from the middle: the +R bore runs over [sIn, sOut] and the -R bore over
// [-sOut, -sIn]. The sleeve's inner face is a cylinder rather than a plane
// square to the ribbon, so the channel's corners reach into the wall before
// its centre line does. sqrt(Ri^2 - c^2) is the station at which the corner
// first touches the inner cylinder, and a millimetre before that the whole
// section is in the hollow; a millimetre past the outer radius the whole
// section is outside the sleeve.
func (p Params) SleeveCut() (float64, float64) {
	ri, c := p.SleeveInner(), p.BoreCorner()
	return math.Sqrt(ri*ri-c*c) - sleeveCutMargin, p.SleeveOuter() + sleeveCutMargin
}

// meshFootprintRadius is a bound on how far from the frame's axis the mesh
// zone reaches: a point of either ribbon within axialWindow of the crossing
// is at most axialWindow along its axis, which is radial in projection, and
// at most half the crest rectangle's diagonal across it.
func meshFootprintRadius(p Params) float64 {
	return math.Hypot(axialWindow(p), math.Hypot(p.Width/2, p.Thickness/2))
}

// The build's range checks for the sleeve, in the order it runs them. Each is
// a closed form the build can evaluate from its inputs alone.
const (
	refuseChannelInWall = "the bore's corner reaches past the sleeve's inner radius, so the channel would start in the wall"
	refuseMeshHidden    = "the mesh zone and its clearance reach past the sleeve's inner radius, so the mesh would not be visible along the axis"
	refuseEndWall       = "the channels come nearer the sleeve's end faces than CollarWall"
)

// sleeveRefusal is the first of the build's checks the inputs fail, or ""
// when they pass every one.
func sleeveRefusal(p Params) string {
	ri := p.SleeveInner()
	if p.BoreCorner() >= ri {
		return refuseChannelInWall
	}
	if meshFootprintRadius(p)+p.Clearance > ri {
		return refuseMeshHidden
	}
	// No point of a channel is further from the middle plane than its axis
	// is, plus the bore's corner radius.
	if p.CageRise < p.AxisOffset()/2+p.BoreCorner()+p.CollarWall {
		return refuseEndWall
	}
	return ""
}

// sleeve is the proposed frame, in the implicit style of the rest of this
// package: a point is in the frame when it is in the tube and in no bore's
// channel.
type sleeve struct {
	p          Params
	gears      [2]Gear
	ri, ro, zb float64
	sIn, sOut  float64 // each bore's cut, as a distance from the middle on its own axis
}

func newSleeve(ga, gb Gear) sleeve {
	p := ga.P
	sIn, sOut := p.SleeveCut()
	return sleeve{
		p:     p,
		gears: [2]Gear{ga, gb},
		ri:    p.SleeveInner(),
		ro:    p.SleeveOuter(),
		zb:    p.CageRise,
		sIn:   sIn,
		sOut:  sOut,
	}
}

func defaultSleeve() sleeve {
	p := sleeveParams()
	ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
	return newSleeve(ga, gb)
}

// inShell answers whether a point is in the uncut tube.
func (f sleeve) inShell(pt r3.Vec) bool {
	r := math.Hypot(pt.X, pt.Y)
	return r >= f.ri && r <= f.ro && math.Abs(pt.Z) <= f.zb
}

// inChannel answers whether a point is inside one of the four bores' cuts:
// the bore's rectangle, turned with the ribbon, over the cut's span. The
// channel is the same boreGap the video frame's collars are cut with.
func (f sleeve) inChannel(pt r3.Vec) bool {
	for _, g := range f.gears {
		if inSleeveChannel(g, f.sIn, f.sOut, pt) {
			return true
		}
	}
	return false
}

func inSleeveChannel(g Gear, sIn, sOut float64, pt r3.Vec) bool {
	d, s := boreGap(g, pt)
	return d == 0 && math.Abs(s) >= sIn && math.Abs(s) <= sOut
}

// inFrame answers whether a point is in the sleeve's material.
func (f sleeve) inFrame(pt r3.Vec) bool {
	return f.inShell(pt) && !f.inChannel(pt)
}

// shellDistance is how far a point is from the uncut tube, zero inside it.
func (f sleeve) shellDistance(pt r3.Vec) float64 {
	r := math.Hypot(pt.X, pt.Y)
	dr := math.Max(0, math.Max(f.ri-r, r-f.ro))
	dz := math.Max(0, math.Abs(pt.Z)-f.zb)
	return math.Hypot(dr, dz)
}

// sectionReach is how far a bore's rectangle, turned to theta, reaches
// straight up (along the section's u direction at theta = 0, which is the
// frame's axis) and sideways across the frame's plane. The extremes are
// corners.
func sectionReach(hw, ht, theta float64) (float64, float64) {
	up := hw*math.Abs(math.Cos(theta)) + ht*math.Abs(math.Sin(theta))
	return up, shadowHalfWidth(hw, ht, theta)
}

// inWall answers whether any of the bore's section at station s lies in the
// tube's radii. A point of the section is at least |s| from the frame's axis,
// since the axis runs radially in projection and the section is square to it,
// and at most hypot(s, sideways reach) from it.
func (f sleeve) inWall(g Gear, s float64) bool {
	_, side := sectionReach(f.p.BoreHalfWidth(), f.p.BoreHalfThickness(), g.angle(s))
	return math.Abs(s) <= f.ro && math.Hypot(s, side) >= f.ri
}

// channelTop is the highest any channel reaches inside the wall, measured
// from the middle plane, and the station it reaches it at. The two gears are
// the same either way up, so this is also the lowest.
func (f sleeve) channelTop() (float64, float64) {
	top, at := 0.0, 0.0
	for _, g := range f.gears {
		for _, sign := range []float64{-1, 1} {
			for s := sign * f.sIn; math.Abs(s) <= f.sOut; s += sign * 0.01 {
				if !f.inWall(g, s) {
					continue
				}
				up, _ := sectionReach(f.p.BoreHalfWidth(), f.p.BoreHalfThickness(), g.angle(s))
				if z := math.Abs(g.Origin.Z) + up; z > top {
					top, at = z, s
				}
			}
		}
	}
	return top, at
}

// eachGrownEnvelopePoint walks the crest rectangle grown by grow on every
// side, nine points to a side, at every station from one to the other.
func eachGrownEnvelopePoint(g Gear, grow, from, to, step float64, fn func(pt r3.Vec, s float64)) {
	uHi, uLo, t := g.envelope()
	uHi, uLo, t = uHi+grow, uLo-grow, t+grow
	for s := from; s <= to; s += step {
		for i := range 9 {
			k := float64(i) / 8
			v := -t + 2*t*k
			fn(g.world(uHi, v, s), s)
			fn(g.world(uLo, v, s), s)
			u := uLo + (uHi-uLo)*k
			fn(g.world(u, t, s), s)
			fn(g.world(u, -t, s), s)
		}
	}
}

// The bores are what the frame may not change: their section, their twist and
// where they sit. This pins the numbers a frame change must not move, and
// holds the sleeve's inputs to the spec's defaults in everything but CageRise,
// which is what lets the video frame's bore, travel and ribbon-to-ribbon
// tests stand for the sleeve unchanged.
func TestSleeveBoresAreTheSameChannels(t *testing.T) {
	f := defaultSleeve()
	p := f.p

	same := p
	same.CageRise = defaultParams().CageRise
	if same != defaultParams() {
		t.Fatalf("the sleeve's inputs differ from the spec's defaults in more than CageRise: %+v", p)
	}
	if got := p.BoreHalfWidth(); math.Abs(got-7.95) > 1e-12 {
		t.Errorf("the bore is %.4f mm across, want 15.9", 2*got)
	}
	if got := p.BoreHalfThickness(); math.Abs(got-2.325) > 1e-12 {
		t.Errorf("the bore is %.4f mm through, want 4.65", 2*got)
	}
	if st := boreStations(p); st[0] != -15 || st[1] != 15 {
		t.Errorf("the bores sit at stations %v, want -15 and +15", st)
	}
	if got, want := p.Lambda(), 49.5/(2*math.Pi); math.Abs(got-want) > 1e-12 {
		t.Errorf("the bore twists at %.6f mm per radian, want %.6f", got, want)
	}
	for gi, g := range f.gears {
		for _, c := range []struct{ station, deg float64 }{{15, 124.1}, {-15, -94.1}} {
			if got := g.angle(c.station) * 180 / math.Pi; math.Abs(got-c.deg) > 0.05 {
				t.Errorf("gear %d's bore at station %+.0f stands at %.2f degrees, want %.1f",
					gi, c.station, got, c.deg)
			}
		}
	}
	// The collar's span is where the wall is complete round the channel, and
	// where the video frame's bore tests walk it. The cut has to cover it.
	if f.sIn > p.CageRadius-p.CollarHalf || f.sOut < p.CageRadius+p.CollarHalf {
		t.Errorf("the cut runs over [%.3f, %.3f] and misses part of the collar's span [%.1f, %.1f]",
			f.sIn, f.sOut, p.CageRadius-p.CollarHalf, p.CageRadius+p.CollarHalf)
	}
	// One gear's bores sit below the middle and the other's above.
	for gi, g := range f.gears {
		for _, station := range boreStations(p) {
			z := g.Origin.Add(g.Ez.Scale(station)).Z
			if (gi == 0) != (z < 0) {
				t.Errorf("gear %d's bore at station %+.0f sits at height %.3f, on the wrong side of the middle",
					gi, station, z)
			}
		}
	}
	t.Logf("the sleeve runs from radius %.1f to %.1f mm, a %.1f mm wall, %.2f mm tall; each bore is "+
		"%.1f by %.2f mm, cut over stations %.3f to %.1f mm of its axis, %.1f mm, turning %.2f degrees",
		f.ri, f.ro, f.ro-f.ri, 2*f.zb, 2*p.BoreHalfWidth(), 2*p.BoreHalfThickness(), f.sIn, f.sOut,
		f.sOut-f.sIn, (f.sOut-f.sIn)/p.Lambda()*180/math.Pi)
}

// The build checks each twisted cut after it is made, by probing two points it
// knows must be open. For the sleeve the probes sit at the crossing itself,
// station +/-CageRadius, on the bore's long axis a clearance's half inside
// each end. Under the right twist sense both are in the channel; under the
// wrong one the channel at the crossing is turned 2*(s - start)/Lambda from
// them, where start is the station the sweep's profile sits at, and both land
// in the wall. So the check tells the senses apart on its own.
func TestSleeveBoreProbesTellTheTwistSense(t *testing.T) {
	f := defaultSleeve()
	p := f.p
	reach := p.Width/2 + p.Clearance/2

	for gi, g := range f.gears {
		for _, station := range boreStations(p) {
			// The sweep runs along +Ez from the path's start, with the profile
			// there: +sIn for the +R bore and -sOut for the -R bore.
			start := f.sIn
			if station < 0 {
				start = -f.sOut
			}
			// The wrong sense turns the other way from the same profile.
			wrong := g
			wrong.Hand = -g.Hand
			wrong.Mount = g.angle(start) + start/g.lambda()
			if d := math.Abs(wrong.angle(start) - g.angle(start)); d > 1e-12 {
				t.Fatalf("the wrong-sense channel does not start at the right profile: %.3e rad apart", d)
			}
			for _, side := range []float64{-1, 1} {
				probe := g.world(side*reach, 0, station)
				if !f.inShell(probe) {
					t.Errorf("gear %d's probe at station %+.0f, radius %.2f and height %.2f, is not in the "+
						"sleeve's wall, so a wrong twist would leave it open too",
						gi, station, math.Hypot(probe.X, probe.Y), probe.Z)
				}
				if f.inFrame(probe) {
					t.Errorf("gear %d's probe at station %+.0f is in the wall under the right twist", gi, station)
				}
				if inSleeveChannel(wrong, f.sIn, f.sOut, probe) || inSleeveChannel(f.gears[1-gi], f.sIn, f.sOut, probe) {
					t.Errorf("gear %d's probe at station %+.0f is open under the wrong twist as well, so "+
						"the check cannot tell the senses apart", gi, station)
				}
			}
			if gi == 0 {
				turn := 2 * (station - start) / g.lambda()
				probe := g.world(reach, 0, station)
				t.Logf("at station %+.0f both probes stand at radius %.2f mm; the wrong sense turns the "+
					"channel %.0f degrees from them and puts them %.2f mm across it, against its %.3f mm "+
					"half thickness", station, math.Hypot(probe.X, probe.Y),
					math.Abs(turn)*180/math.Pi, reach*math.Abs(math.Sin(turn)), p.BoreHalfThickness())
			}
		}
	}
}

// The frame's own proof, for the sleeve: it admits the screw motion and
// nothing else. A gear turned out of step with its own advance jams in the
// bores' walls, as it does in the video frame's collars.
func TestSleeveAdmitsOnlyTheScrewMotion(t *testing.T) {
	f := defaultSleeve()
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
		t.Fatal("the gear does not pass the sleeve's bores even in step, so nothing below means anything")
	}
	var slack float64
	for extra := 0.0; extra < 0.5; extra += 0.002 {
		if !fits(extra) {
			slack = extra
			break
		}
	}
	if slack == 0 {
		t.Fatal("the gear turns freely in the sleeve: the bores are not holding it to the screw motion")
	}
	if got := slack * 180 / math.Pi; got > 12 {
		t.Errorf("the sleeve lets the gear turn %.1f degrees out of step", got)
	}
	t.Logf("the sleeve jams the gear %.2f degrees out of step, on %.2f mm of clearance",
		slack*180/math.Pi, f.p.Clearance)
}

// Everything either ribbon reaches at any phase of the travel misses the
// sleeve by the clearance. Inside a bore's cut the crest rectangle grown by
// the clearance, less a micron, is the bore shrunk by that micron, so walking
// it against the frame is exact there. Outside the cut the grown rectangle has
// to miss the uncut tube, and the least distance from the ungrown rectangle to
// the tube is logged as a bound on the gap and held to the clearance.
func TestRibbonsClearTheSleeveOverTheTravel(t *testing.T) {
	f := defaultSleeve()
	p := f.p

	hits, samples := 0, 0
	var first r3.Vec
	outside, outsideAt := math.Inf(1), 0.0
	for _, g := range f.gears {
		lo, hi := travelSpan(g)
		eachGrownEnvelopePoint(g, p.Clearance-1e-3, lo, hi, 0.02, func(pt r3.Vec, _ float64) {
			samples++
			if f.inFrame(pt) {
				if hits == 0 {
					first = pt
				}
				hits++
			}
		})
		eachGrownEnvelopePoint(g, 0, lo, hi, 0.02, func(pt r3.Vec, s float64) {
			if math.Abs(s) >= f.sIn && math.Abs(s) <= f.sOut {
				return
			}
			if d := f.shellDistance(pt); d < outside {
				outside, outsideAt = d, s
			}
		})
	}
	if hits > 0 {
		t.Errorf("%d of %d points of the ribbons' crest rectangles grown by the clearance lie in the "+
			"sleeve, the first at (%.2f, %.2f, %.2f)", hits, samples, first.X, first.Y, first.Z)
	}
	if outside < p.Clearance {
		t.Errorf("outside the bores' cuts a ribbon comes within %.3f mm of the tube, under the %.2f mm "+
			"clearance", outside, p.Clearance)
	}
	t.Logf("over the travel no point of %d on the ribbons grown by the clearance touches the sleeve; "+
		"outside the cuts they keep %.3f mm from the tube, nearest at station %+.2f", samples, outside, outsideAt)
}

// sleeveVoxel is the voxel size every whole-body check of the sleeve walks.
const sleeveVoxel = 0.25

// voxels is the sleeve sampled at the centres of a cubic grid that covers
// [-Ro, Ro]^2 x [-Zb, Zb] with one empty layer of cells beyond on every side.
type voxels struct {
	f      sleeve
	h      float64
	n, nz  int
	x0, z0 float64 // the centre of cell 0 on the X and Y axes, and on the Z axis
	solid  []bool
	count  int
}

func (v *voxels) idx(i, j, k int) int { return (k*v.n+j)*v.n + i }

func (v *voxels) at(i, j, k int) r3.Vec {
	return r3.NewVec(v.x0+float64(i)*v.h, v.x0+float64(j)*v.h, v.z0+float64(k)*v.h)
}

func (v *voxels) solidAt(i, j, k int) bool {
	if i < 0 || j < 0 || k < 0 || i >= v.n || j >= v.n || k >= v.nz {
		return false
	}
	return v.solid[v.idx(i, j, k)]
}

func voxelize(f sleeve, h float64) *voxels {
	v := &voxels{
		f:  f,
		h:  h,
		n:  int(math.Ceil(2*f.ro/h)) + 2,
		nz: int(math.Ceil(2*f.zb/h)) + 2,
		x0: -f.ro - h/2,
		z0: -f.zb - h/2,
	}
	v.solid = make([]bool, v.n*v.n*v.nz)
	// Layers are independent, so they are filled in parallel; each goroutine
	// writes only its own layers' cells.
	counts := make([]int, v.nz)
	var wg sync.WaitGroup
	for k := range v.nz {
		wg.Go(func() {
			for j := range v.n {
				for i := range v.n {
					if f.inFrame(v.at(i, j, k)) {
						v.solid[v.idx(i, j, k)] = true
						counts[k]++
					}
				}
			}
		})
	}
	wg.Wait()
	for _, c := range counts {
		v.count += c
	}
	return v
}

// defaultVoxels is the default sleeve's grid, built once for the tests that
// share it.
var defaultVoxels = sync.OnceValue(func() *voxels { return voxelize(defaultSleeve(), sleeveVoxel) })

// The sleeve is one body. The tube is, and the bores are what could cut it
// apart: the grid is flood-filled from one solid cell across shared faces, and
// every solid cell has to be reached. The closed form that makes it true is
// that no channel comes within CollarWall of either end face inside the wall,
// so both end bands are whole rings, and every bit of the wall between the
// bores reaches one of them.
func TestSleeveIsOnePiece(t *testing.T) {
	v := defaultVoxels()
	f := v.f
	p := f.p

	start := -1
	for i, s := range v.solid {
		if s {
			start = i
			break
		}
	}
	if start < 0 {
		t.Fatal("the sleeve has no material at all")
	}
	seen := make([]bool, len(v.solid))
	seen[start] = true
	stack := []int{start}
	reached := 0
	for len(stack) > 0 {
		c := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		reached++
		i, j, k := c%v.n, (c/v.n)%v.n, c/(v.n*v.n)
		for _, d := range [6][3]int{{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}} {
			ii, jj, kk := i+d[0], j+d[1], k+d[2]
			if !v.solidAt(ii, jj, kk) {
				continue
			}
			if w := v.idx(ii, jj, kk); !seen[w] {
				seen[w] = true
				stack = append(stack, w)
			}
		}
	}
	if reached != v.count {
		t.Errorf("a flood fill from one cell of the sleeve reaches %d of its %d cells: the bores cut it "+
			"into more than one piece", reached, v.count)
	}

	top, at := f.channelTop()
	if wall := f.zb - top; wall < p.CollarWall {
		t.Errorf("a channel reaches %.3f mm from the middle at station %+.2f, leaving %.3f mm of end "+
			"wall under the %.1f mm CollarWall", top, at, wall, p.CollarWall)
	}
	// The closed form checked against the grid: no cell of the tube that a
	// channel cuts stands higher than the closed form says a channel reaches.
	// A cell of the tube that is not solid is one a channel cuts.
	highest := 0.0
	for k := range v.nz {
		for j := range v.n {
			for i := range v.n {
				if pt := v.at(i, j, k); f.inShell(pt) && !v.solidAt(i, j, k) {
					highest = math.Max(highest, math.Abs(pt.Z))
				}
			}
		}
	}
	if highest > top {
		t.Errorf("the grid finds a channel in the wall at %.3f mm from the middle, past the %.3f mm the "+
			"closed form gives", highest, top)
	}
	volume := float64(v.count) * v.h * v.h * v.h
	t.Logf("the sleeve is one piece of %d cells at %.2f mm, %.0f mm^3, about %.0f g of PLA; the channels "+
		"reach %.3f mm from the middle (the grid finds %.3f), leaving %.2f mm of end wall",
		v.count, v.h, volume, volume*1.24e-3, top, highest, f.zb-top)
}

// The mesh has to stay visible from the top and the bottom. Everything either
// ribbon reaches within the engaged zone, grown by the clearance, is projected
// onto the plane square to the frame's axis, and the line through every point
// of that footprint along the axis has to miss the frame at every height.
// The frame's material never comes inside the inner radius, so this walks the
// footprint's outline: the open region is a disc, and a disc holds the inside
// of any outline it holds.
//
// The video frame's cube about the crossing stays open as well, which is what
// TestTheMiddleStaysOpen holds for that frame.
func TestSleeveKeepsTheMeshVisibleAlongTheAxis(t *testing.T) {
	f := defaultSleeve()
	p := f.p
	window := axialWindow(p)

	// The footprint itself, and the same grown by the clearance: the crest
	// rectangle grown on every side and the stations stretched at both ends,
	// which holds the clearance's disc round every point of the footprint.
	radius, grown, blocked := 0.0, 0.0, 0
	for _, g := range f.gears {
		eachGrownEnvelopePoint(g, 0, -window, window, 0.05, func(pt r3.Vec, _ float64) {
			radius = math.Max(radius, math.Hypot(pt.X, pt.Y))
		})
		eachGrownEnvelopePoint(g, p.Clearance, -window-p.Clearance, window+p.Clearance, 0.05,
			func(pt r3.Vec, _ float64) {
				grown = math.Max(grown, math.Hypot(pt.X, pt.Y))
				for z := -f.zb; z <= f.zb; z += 0.1 {
					if f.inFrame(r3.NewVec(pt.X, pt.Y, z)) {
						blocked++
						return
					}
				}
			})
	}
	if blocked > 0 {
		t.Errorf("%d points of the mesh's footprint are hidden by the sleeve along its axis", blocked)
	}
	if bound := meshFootprintRadius(p); radius > bound {
		t.Errorf("the footprint reaches %.3f mm from the axis, past the %.3f mm the build's closed form "+
			"bounds it by", radius, bound)
	}

	for x := -window; x <= window; x += 0.2 {
		for y := -window; y <= window; y += 0.2 {
			for z := -window; z <= window; z += 0.2 {
				if f.inFrame(r3.NewVec(x, y, z)) {
					t.Fatalf("the sleeve reaches into the meshing space at (%.1f, %.1f, %.1f)", x, y, z)
				}
			}
		}
	}
	t.Logf("the mesh's footprint reaches %.2f mm from the axis (the closed form bounds it by %.2f), %.2f "+
		"grown by the clearance, inside a hollow of radius %.1f mm, and is open along the axis from end "+
		"to end; nothing of the sleeve is within %.2f mm of the middle",
		radius, meshFootprintRadius(p), grown, f.ri, window)
}

// unsupportedCells walks the grid from the bed up, the bed under the lowest
// layer when up is true and over the highest when it is false, and returns
// every solid cell above the first layer that has no solid cell under it
// within one cell either way: the 45 degree rule at the grid's size. An
// unsupported cell is where the printer would lay material on air, which is
// a bridge or a worse overhang.
func unsupportedCells(v *voxels, up bool) [][3]int {
	below := -1
	first, last, step := 0, v.nz, 1
	if !up {
		below, first, last, step = 1, v.nz-1, -1, -1
	}
	var out [][3]int
	base := true
	for k := first; k != last; k += step {
		layerSolid := false
		for j := range v.n {
			for i := range v.n {
				if !v.solidAt(i, j, k) {
					continue
				}
				layerSolid = true
				if base {
					continue
				}
				supported := false
				for dj := -1; dj <= 1 && !supported; dj++ {
					for di := -1; di <= 1 && !supported; di++ {
						supported = v.solidAt(i+di, j+dj, k+below)
					}
				}
				if !supported {
					out = append(out, [3]int{i, j, k})
				}
			}
		}
		if layerSolid {
			base = false
		}
	}
	return out
}

// inABore answers whether an unsupported cell is part of a bore's ceiling:
// its column along the frame's axis meets a channel, and a channel lies
// within two cells of it.
func inABore(v *voxels, c [3]int) bool {
	f := v.f
	column := false
	for k := range v.nz {
		if f.inChannel(v.at(c[0], c[1], k)) {
			column = true
			break
		}
	}
	if !column {
		return false
	}
	for dk := -2; dk <= 2; dk++ {
		for dj := -2; dj <= 2; dj++ {
			for di := -2; di <= 2; di++ {
				if f.inChannel(v.at(c[0]+di, c[1]+dj, c[2]+dk)) {
					return true
				}
			}
		}
	}
	return false
}

// The sleeve prints standing on either end, with no support. Each end is a
// flat, level ring, which is the whole of the bed's contact; no layer lays
// material on air except in the roof of a bore, which is the hole itself and
// which no frame can move; and nothing in the wall is thinner than
// CollarWall. What the printer makes of a bore's roof is not something a
// proof reaches, so the flattest roof in each bore and the span the printer
// bridges there are logged for the record.
func TestSleevePrintsStandingOnEitherEnd(t *testing.T) {
	v := defaultVoxels()
	f := v.f
	p := f.p

	// The bed. The first and last layers of the grid lie beyond the ends and
	// have to be empty; the layers just inside them are the two ends.
	ring := math.Pi * (f.ro*f.ro - f.ri*f.ri)
	layerArea := func(k int) float64 {
		n := 0
		for j := range v.n {
			for i := range v.n {
				if v.solidAt(i, j, k) {
					n++
				}
			}
		}
		return float64(n) * v.h * v.h
	}
	if a, b := layerArea(0), layerArea(v.nz-1); a > 0 || b > 0 {
		t.Errorf("the sleeve has %.2f mm^2 below its bottom face and %.2f mm^2 above its top face", a, b)
	}
	bottom, top := layerArea(1), layerArea(v.nz-2)
	for name, a := range map[string]float64{"bottom": bottom, "top": top} {
		if math.Abs(a-ring)/ring > 0.01 {
			t.Errorf("the %s layer has %.1f mm^2 of material, not the %.1f mm^2 ring of a flat end", name, a, ring)
		}
	}

	// Overhangs, printed either way up.
	for _, up := range []bool{true, false} {
		way := "on its bottom end"
		if !up {
			way = "on its top end"
		}
		cells := unsupportedCells(v, up)
		stray := 0
		for _, c := range cells {
			if inABore(v, c) {
				continue
			}
			if stray < 5 {
				pt := v.at(c[0], c[1], c[2])
				t.Errorf("printed %s, the sleeve lays material on air outside any bore at radius %.2f, "+
					"height %.2f", way, math.Hypot(pt.X, pt.Y), pt.Z)
			}
			stray++
		}
		if stray > 0 {
			t.Errorf("printed %s, %d unsupported cells lie outside the bores", way, stray)
		}
		t.Logf("printed %s, %d cells (%.0f mm^2) lay material on air, every one in a bore's roof",
			way, len(cells), float64(len(cells))*v.h*v.h)
	}

	// Each bore's roof, for the record. Of each pair of opposite walls of the
	// channel, the one whose normal into the channel points down is a roof,
	// and it overhangs upright by the arcsine of that normal's vertical
	// component: zero is an upright wall and ninety a flat ceiling.
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	for gi, g := range f.gears {
		for _, sign := range []float64{-1, 1} {
			worst, at, span := 0.0, 0.0, 0.0
			for s := sign * f.sIn; math.Abs(s) <= f.sOut; s += sign * 0.01 {
				if !f.inWall(g, s) {
					continue
				}
				th := g.angle(s)
				// The section's u direction is the frame's axis turned by
				// theta, so a wall across u has a vertical normal component of
				// cos(theta) and a wall across v one of sin(theta).
				for _, w := range []struct{ nz, span float64 }{
					{math.Abs(math.Cos(th)), 2 * ht}, {math.Abs(math.Sin(th)), 2 * hw},
				} {
					if over := math.Asin(math.Min(1, w.nz)) * 180 / math.Pi; over > worst {
						worst, at, span = over, s, w.span
					}
				}
			}
			if gi == 0 {
				t.Logf("the bore at station %+.0f has its flattest roof at station %+.2f, %.1f degrees "+
					"from upright, and bridges %.1f mm there over the wall; not enforced",
					sign*p.CageRadius, at, worst, span)
			}
		}
	}

	// Thin features. The end wall, the wall between neighbouring bores, and
	// the edge each bore's mouth leaves where it meets a cylinder.
	topReach, _ := f.channelTop()
	if wall := f.zb - topReach; wall < p.CollarWall {
		t.Errorf("the end wall is %.2f mm, under the %.1f mm CollarWall", wall, p.CollarWall)
	}
	nearest, pairName := nearestChannels(f)
	if nearest < p.CollarWall {
		t.Errorf("the channels of %s come within %.2f mm of each other, under the %.1f mm CollarWall",
			pairName, nearest, p.CollarWall)
	}
	c := p.BoreCorner()
	inner := 90 - math.Asin(c/f.ri)*180/math.Pi
	outer := 90 - math.Asin(c/f.ro)*180/math.Pi
	if inner < minMouthWedge || outer < minMouthWedge {
		t.Errorf("a bore's mouth leaves an edge of %.1f degrees on the inner face and %.1f on the outer, "+
			"under the %.0f degree floor", inner, outer, minMouthWedge)
	}
	t.Logf("the base is a %.1f mm^2 ring (%.1f mm^2 on the grid, %.1f on top); the end wall is %.2f mm; "+
		"the nearest two channels, %s, are %.2f mm apart; the mouths leave %.1f and %.1f degree edges",
		ring, bottom, top, f.zb-topReach, pairName, nearest, inner, outer)
}

// minMouthWedge is the least angle the material may come to where a bore's
// side meets the inner or outer cylinder. The mouths are logged rather than
// held to a bound the print has shown matters; the floor is there so that an
// input which leaves a feather edge is caught.
const minMouthWedge = 30.0

// nearestChannels is the least distance between the outlines of two
// different bores' channels, where they run through the wall, and which two.
func nearestChannels(f sleeve) (float64, string) {
	p := f.p
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	type channel struct {
		name string
		pts  []r3.Vec
	}
	var chans []channel
	for gi, g := range f.gears {
		for _, sign := range []float64{-1, 1} {
			side := " +R"
			if sign < 0 {
				side = " -R"
			}
			ch := channel{name: [2]string{"gear A", "gear B"}[gi] + side}
			for s := sign * f.sIn; math.Abs(s) <= f.sOut; s += sign * 0.1 {
				for i := range 17 {
					k := float64(i) / 16
					for _, q := range [4][2]float64{
						{hw, -ht + 2*ht*k}, {-hw, -ht + 2*ht*k}, {-hw + 2*hw*k, ht}, {-hw + 2*hw*k, -ht},
					} {
						pt := g.world(q[0], q[1], s)
						if r := math.Hypot(pt.X, pt.Y); r < f.ri-0.5 || r > f.ro+0.5 {
							continue
						}
						ch.pts = append(ch.pts, pt)
					}
				}
			}
			chans = append(chans, ch)
		}
	}
	best, name := math.Inf(1), ""
	for i := range chans {
		for j := i + 1; j < len(chans); j++ {
			for _, a := range chans[i].pts {
				for _, b := range chans[j].pts {
					if d := a.Sub(b).Len(); d < best {
						best, name = d, chans[i].name+" and "+chans[j].name
					}
				}
			}
		}
	}
	return best, name
}

// The compiled step proof will stand a ruled loft in for each bore's twisted
// sweep, as it does for the video frame's collars, and the sleeve's cut is
// longer: it runs over [sIn, sOut] rather than the collar's length and a
// millimetre, so the loft turns further. This measures what the stand-in
// costs over that span, at the section count the stand-in derives, at the
// same clearances TestBoreSubstituteKeepsItsClearance runs.
func TestSleeveBoreSubstituteKeepsItsClearance(t *testing.T) {
	for _, clearance := range []float64{0.05, 0.1, 0.45, 0.9} {
		p := sleeveParams()
		p.Clearance = clearance
		ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
		sIn, sOut := p.SleeveCut()
		turn := (sOut - sIn) / p.Lambda()
		n := boreSections(p, turn)

		worst := math.Inf(1)
		for _, g := range [2]Gear{ga, gb} {
			for _, span := range [2][2]float64{{-sOut, -sIn}, {sIn, sOut}} {
				left := boreLoftClearance(g, span[0], span[1], n)
				if left < 0.95*p.Clearance {
					t.Errorf("at a clearance of %.2f mm the stand-in for the bore over [%.2f, %.2f] lofts "+
						"through %d sections and leaves the ribbon %.4f mm, under 95%% of the clearance",
						clearance, span[0], span[1], n, left)
				}
				worst = math.Min(worst, left)
			}
		}
		t.Logf("at a clearance of %.2f mm the sleeve's cut turns %.2f degrees through %d sections %.2f "+
			"degrees apart and the stand-in leaves the ribbon %.4f mm", clearance, turn*180/math.Pi, n,
			turn/float64(n-1)*180/math.Pi, worst)
	}
}

// The build refuses an input the sleeve cannot be made from, with three
// closed forms: a bore's corner that reaches past the inner radius would
// start the channel in the wall; a mesh zone that reaches past it would be
// hidden; channels that come within CollarWall of the end faces would leave
// them too thin. This holds that the defaults pass all three, and that each
// refusal is reached by an input that passes every check before it.
func TestSleeveInputsAreChecked(t *testing.T) {
	if why := sleeveRefusal(sleeveParams()); why != "" {
		t.Fatalf("the build refuses the defaults: %s", why)
	}
	p := sleeveParams()
	t.Logf("at the defaults the bore's corner stands %.3f mm from its axis against a %.1f mm inner radius; "+
		"the mesh reaches %.2f mm and its clearance %.2f more; the end wall needs a rise of %.2f mm and has %.2f",
		p.BoreCorner(), p.SleeveInner(), meshFootprintRadius(p), p.Clearance,
		p.AxisOffset()/2+p.BoreCorner()+p.CollarWall, p.CageRise)

	cases := []struct {
		name string
		edit func(*Params)
		want string
	}{
		{"a 4 mm clearance", func(p *Params) { p.Clearance = 4 }, refuseChannelInWall},
		{"a 1.5 mm engagement", func(p *Params) { p.Engagement = 1.5 }, refuseMeshHidden},
		{"an 18 mm rise", func(p *Params) { p.CageRise = 18 }, refuseEndWall},
	}
	for _, c := range cases {
		p := sleeveParams()
		c.edit(&p)
		if got := sleeveRefusal(p); got != c.want {
			t.Errorf("%s: the build says %q, want %q", c.name, got, c.want)
			continue
		}
		t.Logf("%s is refused: %s", c.name, c.want)
	}

	// The mesh check is the one an older, simpler rule would miss: at a 1.5 mm
	// engagement the collar still starts outside the engaged zone, which is
	// all the video frame asks, while the footprint reaches the wall.
	q := sleeveParams()
	q.Engagement = 1.5
	if zone := axialWindow(q); q.SleeveInner() <= zone {
		t.Errorf("at a 1.5 mm engagement the engaged zone reaches %.2f mm, so the video frame's rule "+
			"refuses it too and the sleeve's own check is not what is reached", zone)
	}
	// And the rise check is not over-cautious there: at 18 mm the channels
	// really do leave less than CollarWall at the ends.
	r := sleeveParams()
	r.CageRise = 18
	ga, gb := pair(r, r.Sigma(), 0, assemblyPhase)
	topReach, _ := newSleeve(ga, gb).channelTop()
	if wall := r.CageRise - topReach; wall >= r.CollarWall {
		t.Errorf("at an 18 mm rise the channels leave %.3f mm of end wall, which is enough; the refusal "+
			"is stricter than the geometry", wall)
	} else {
		t.Logf("at an 18 mm rise the channels leave %.3f mm of end wall", wall)
	}
}

// The video's frame no longer binds the design, since the user dropped its
// look as a requirement, but the sleeve still lands inside the widened ranges
// TestProportionsFollowTheVideo holds the ring and the frame to, reading the
// ring's outer diameter as the sleeve's and the frame's height as its own.
func TestSleeveProportionsStayNearTheVideo(t *testing.T) {
	p := sleeveParams()
	across := 2 * p.SleeveOuter() / p.Width
	tall := 2 * p.CageRise / (2 * p.SleeveOuter())
	if across < 1.76 || across > 3.36 {
		t.Errorf("the sleeve is %.2f widths across; the video's ring reads 2.2-2.8", across)
	}
	if tall < 0.64 || tall > 1.44 {
		t.Errorf("the sleeve is %.2f of its own diameter tall; the video's frame reads about one", tall)
	}
	t.Logf("the sleeve is %.2f ribbon widths across and %.2f of its diameter tall", across, tall)
}
