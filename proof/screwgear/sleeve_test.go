package screwgear_test

import (
	"fmt"
	"math"
	"sort"
	"strings"
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
// straight through its wall and two slanted windows cut between them. Its wall
// is the collar: the bore runs through every bit of material the ribbon
// passes, so the wall's thickness along the bore's centre line is the
// collar's length, and the far face of the bore stays where the video frame's
// collar had it. The mesh is seen along the axis through the hollow from
// either end, and from the side through the windows. The frame prints
// standing on either end with no support: every outside face is a vertical
// cylinder or a level end face, every face of a window is upright or at 45
// degrees, and the only faces that need more are the ceilings of the four
// bores.
//
// spec/screwgear/instructions.md builds the sleeve; the add-in still builds
// the video's frame until it is regenerated. Nothing here changes
// defaultParams, which cage_test.go reads for the video frame it proves;
// sleeveParams is the spec's default table.
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

// The build's range checks for the sleeve, in the order it runs them. The
// first three are closed forms the build can evaluate from its inputs alone;
// the fourth is channelSeparation, which samples the bores' channels.
const (
	refuseChannelInWall = "the bore's corner reaches past the sleeve's inner radius, so the channel would start in the wall"
	refuseMeshHidden    = "the mesh zone and its clearance reach past the sleeve's inner radius, so the mesh would not be visible along the axis"
	refuseEndWall       = "the channels come nearer the sleeve's end faces than CollarWall"
	refuseChannelsClose = "two neighbouring bores' channels come nearer each other than CollarWall"
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
	ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
	if sep, _ := channelSeparation(plainSleeve(ga, gb)); sep < p.CollarWall {
		return refuseChannelsClose
	}
	return ""
}

// sleeve is the proposed frame, in the implicit style of the rest of this
// package: a point is in the frame when it is in the tube, in no bore's
// channel and in no window.
type sleeve struct {
	p          Params
	gears      [2]Gear
	ri, ro, zb float64
	sIn, sOut  float64             // each bore's cut, as a distance from the middle on its own axis
	sections   [4][]channelStation // each bore's channel in the tube, as wallGap walks it
	windows    []sleeveWindow
}

// newSleeve builds the sleeve for a pair. Its windows are placed from the
// tube's radii and height and from the bores' channels, so they follow
// whatever inputs the sleeve is built from.
func newSleeve(ga, gb Gear) sleeve {
	f := plainSleeve(ga, gb)
	for _, b := range f.bores() {
		f.sections[b.index] = f.channelSections(b)
	}
	for _, facing := range windowFacings(f.p) {
		if w := f.newWindow(facing); w.room() == "" {
			f.windows = append(f.windows, w)
		}
	}
	return f
}

// plainSleeve is the tube and its four bores, with no window.
func plainSleeve(ga, gb Gear) sleeve {
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

// windowFacings are the two level directions the windows are cut toward:
// across the two wider gaps between neighbouring bores. Neighbouring bores
// are Sigma apart across +X and -X, and 180 - Sigma apart across +Y and -Y,
// so the windows face +Y and -Y up to a right-angled crossing and +X and -X
// past it.
func windowFacings(p Params) [2]r3.Vec {
	if p.Sigma() <= math.Pi/2 {
		return [2]r3.Vec{r3.NewVec(0, 1, 0), r3.NewVec(0, -1, 0)}
	}
	return [2]r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(-1, 0, 0)}
}

// defaultSleeve is the sleeve at the defaults, built once: finding the
// windows' ends walks the far bores, and most tests read the same sleeve.
var defaultSleeve = sync.OnceValue(func() sleeve {
	p := sleeveParams()
	ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
	return newSleeve(ga, gb)
})

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

// inWindow answers whether a point is inside one of the windows' cuts.
func (f sleeve) inWindow(pt r3.Vec) bool {
	for _, w := range f.windows {
		if w.contains(pt) {
			return true
		}
	}
	return false
}

// inFrame answers whether a point is in the sleeve's material.
func (f sleeve) inFrame(pt r3.Vec) bool {
	return f.inShell(pt) && !f.inChannel(pt) && !f.inWindow(pt)
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

// The sleeve is one body. The tube is, and the bores and windows are what
// could cut it apart: the grid is flood-filled from one solid cell across
// shared faces, and every solid cell has to be reached. The closed form that
// makes it true is that no channel comes within CollarWall of either end face
// inside the wall and no window reaches past the channels, so both end bands
// are whole rings, and every bit of the wall between the openings reaches one
// of them.
func TestSleeveIsOnePiece(t *testing.T) {
	checkSleeveIsOnePiece(t, defaultVoxels())
}

// checkSleeveIsOnePiece is TestSleeveIsOnePiece for any sleeve's grid.
func checkSleeveIsOnePiece(t testing.TB, v *voxels) {
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
	// The windows cut the tube too, and are held under the same height by
	// TestSleeveWindowsKeepTheirWalls, so only the channels are asked here.
	highest := 0.0
	for k := range v.nz {
		for j := range v.n {
			for i := range v.n {
				if pt := v.at(i, j, k); f.inShell(pt) && !v.solidAt(i, j, k) && f.inChannel(pt) {
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
	checkSleevePrints(t, defaultVoxels())
}

// checkSleevePrints is TestSleevePrintsStandingOnEitherEnd for any sleeve's
// grid.
func checkSleevePrints(t testing.TB, v *voxels) {
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
	bores := f.bores()
	var pts [4][]r3.Vec
	for _, b := range bores {
		pts[b.index] = f.channelOutline(b)
	}
	best, name := math.Inf(1), ""
	for i := range bores {
		for j := i + 1; j < len(bores); j++ {
			for _, a := range pts[i] {
				for _, b := range pts[j] {
					if d := a.Sub(b).Len(); d < best {
						best, name = d, bores[i].name+" and "+bores[j].name
					}
				}
			}
		}
	}
	return best, name
}

// channelOutline samples a bore's channel where it runs through the wall:
// the rectangle's four sides at seventeen points each, at stations 0.1 mm
// apart from the cut's inner end outward, kept where they stand within half a
// millimetre of the tube's radii.
func (f sleeve) channelOutline(b bore) []r3.Vec {
	p := f.p
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	var out []r3.Vec
	for s := b.sign * f.sIn; math.Abs(s) <= f.sOut; s += b.sign * 0.1 {
		for i := range 17 {
			k := float64(i) / 16
			for _, q := range [4][2]float64{
				{hw, -ht + 2*ht*k}, {-hw, -ht + 2*ht*k}, {-hw + 2*hw*k, ht}, {-hw + 2*hw*k, -ht},
			} {
				pt := b.g.world(q[0], q[1], s)
				if r := math.Hypot(pt.X, pt.Y); r < f.ri-0.5 || r > f.ro+0.5 {
					continue
				}
				out = append(out, pt)
			}
		}
	}
	return out
}

// channelSeparation is the build's check on the wall between neighbouring
// bores, and the gap it is least across. Round the tube the bores alternate
// between the gears, so the four gaps between neighbours face +Y (gear A's
// +R bore and gear B's -R), -X (the two -R bores), -Y (gear A's -R and gear
// B's +R) and +X (the two +R bores). For each gap both bores' outlines, as
// channelOutline samples them, are projected onto the plane through the
// frame's axis square to the direction the gap faces, and the convex hull of
// each projection is taken. The separation is the most, over the directions
// square to the two hulls' edges, by which one hull's projection onto that
// direction clears the other's. A projection brings no two points nearer
// together, so the separation never exceeds the least distance between the
// two outlines that nearestChannels measures.
func channelSeparation(f sleeve) (float64, string) {
	b := f.bores()
	gaps := []struct {
		facing r3.Vec
		x, y   bore
	}{
		{r3.NewVec(0, 1, 0), b[1], b[2]},
		{r3.NewVec(-1, 0, 0), b[2], b[0]},
		{r3.NewVec(0, -1, 0), b[0], b[3]},
		{r3.NewVec(1, 0, 0), b[3], b[1]},
	}
	least, which := math.Inf(1), ""
	for _, gap := range gaps {
		across := r3.NewVec(0, 0, 1).Cross(gap.facing)
		hull := func(bb bore) [][2]float64 {
			var pts [][2]float64
			for _, pt := range f.channelOutline(bb) {
				pts = append(pts, [2]float64{pt.Dot(across), pt.Z})
			}
			return convexHull(pts)
		}
		hx, hy := hull(gap.x), hull(gap.y)
		sep := math.Inf(-1)
		for _, h := range [][][2]float64{hx, hy} {
			for i := range h {
				a, c := h[i], h[(i+1)%len(h)]
				n := [2]float64{c[1] - a[1], a[0] - c[0]}
				l := math.Hypot(n[0], n[1])
				if l == 0 {
					continue
				}
				n[0], n[1] = n[0]/l, n[1]/l
				loX, hiX, loY, hiY := math.Inf(1), math.Inf(-1), math.Inf(1), math.Inf(-1)
				for _, q := range hx {
					d := q[0]*n[0] + q[1]*n[1]
					loX, hiX = math.Min(loX, d), math.Max(hiX, d)
				}
				for _, q := range hy {
					d := q[0]*n[0] + q[1]*n[1]
					loY, hiY = math.Min(loY, d), math.Max(hiY, d)
				}
				sep = math.Max(sep, math.Max(loY-hiX, loX-hiY))
			}
		}
		if sep < least {
			least, which = sep, gap.x.name+" and "+gap.y.name
		}
	}
	return least, which
}

// convexHull is the convex hull of a set of points, counter-clockwise, by
// Andrew's monotone chain.
func convexHull(pts [][2]float64) [][2]float64 {
	sort.Slice(pts, func(i, j int) bool {
		if pts[i][0] != pts[j][0] {
			return pts[i][0] < pts[j][0]
		}
		return pts[i][1] < pts[j][1]
	})
	cross := func(o, a, b [2]float64) float64 {
		return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
	}
	var h [][2]float64
	for pass := 0; pass < 2; pass++ {
		start := len(h)
		for _, q := range pts {
			for len(h) >= start+2 && cross(h[len(h)-2], h[len(h)-1], q) <= 0 {
				h = h[:len(h)-1]
			}
			h = append(h, q)
		}
		h = h[:len(h)-1]
		for i, j := 0, len(pts)-1; i < j; i, j = i+1, j-1 {
			pts[i], pts[j] = pts[j], pts[i]
		}
	}
	return h
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
// closed forms and one sampled check: a bore's corner that reaches past the
// inner radius would start the channel in the wall; a mesh zone that reaches
// past it would be hidden; channels that come within CollarWall of the end
// faces would leave them too thin; and two neighbouring bores whose channels
// channelSeparation cannot hold CollarWall apart would leave too thin a wall
// between them. This holds that the defaults pass all four, and that each
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
		{"a 4 mm CollarWall at a 70 degree crossing", func(p *Params) {
			p.CollarWall, p.CrossAngle = 4, 70*math.Pi/180
			p.CageRise = leastRise(*p)
		}, refuseChannelsClose},
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
	// The separation is a bound under the distance nearestChannels measures,
	// and at the defaults it clears CollarWall with room to spare.
	sep, at := channelSeparation(defaultSleeve())
	near, _ := nearestChannels(defaultSleeve())
	if sep > near {
		t.Errorf("the separation between %s is %.3f mm, past the %.3f mm nearestChannels measures", at, sep, near)
	}
	t.Logf("at the defaults the channels are at least %.3f mm apart by the build's check, least across the "+
		"gap between %s; nearestChannels measures %.3f mm", sep, at, near)
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

// ---------------------------------------------------------------------------
// The windows.
//
// Down the hollow is the only way into the plain sleeve. Two slanted windows
// through the wall show the mesh from the side as well.
//
// The four bores sit round the tube at azimuths Sigma/2 and 180 - Sigma/2 on
// the +Y side and at their images under the half turn about X on the -Y side:
// 40, 140, 220 and 320 degrees at the defaults. Across +X and -X neighbouring
// bores are Sigma apart, and their channels come within 6.81 and 4.83 mm of
// each other, which is less than a CollarWall either side of any window.
// Across +Y and -Y they are 180 - Sigma apart, one bore low and the other
// high, and the wall between them is a band that runs at about 45 degrees from
// above the low bore down to below the high one. Each window is cut along that
// band, one facing +Y and one facing -Y. Past a right-angled crossing the
// wider gaps are the ones across +X and -X, and the windows face those
// instead (windowFacings).
//
// A window is a prism: a hexagon drawn on the plane through the frame's axis
// square to the direction it faces, pushed straight out through the wall on
// that side. On the plane, t runs across the window toward increasing azimuth
// and z up the frame's axis. The hexagon is the band lo <= z + lean*t <= hi,
// whose sides are 45 degree lines, cut off by two upright ends at t = left and
// t = right, with the two long corners trimmed by 45 degree lines. Every edge
// is upright or at 45 degrees, so standing on either end the printer lays
// each window's roof on the layer below it and bridges nothing.
//
// Where a 45 degree roof meets the tube's inner face, the line they meet on
// descends more gently than the roof, at atan(cos psi), psi being how far
// round the face the line is from the window's facing direction. That matters
// only where the layer below lies toward the window's middle, which on the
// inner face is toward the hollow: there the layer's outline steps along the
// face by 1/cos psi of a layer per layer. Up to psi = 45 degrees that is one
// cell diagonally, which the voxel layer check accepts as its 45 degree rule;
// past it the check finds material laid on air. Each side of the band rises
// toward the middle on one half of the window, so each trim also cuts its side
// off where the side would meet the inner face past psi = 45 degrees, at
// |t| = SleeveInner/sqrt(2). On the outer face the same happens where the
// layer below lies away from the middle, and a window that stays within
// SleeveOuter/sqrt(2) of the middle never reaches it, so neither end stands
// further out than that.
//
// Every dimension comes from the sleeve's own geometry: the sides sit
// CollarWall beyond the reach of the two bores flanking the window, the trims
// keep the window inside the height the channels already reach and its roofs
// within 45 degrees of the middle on the inner face, and each end stands where
// its corner lines through the wall, and its upright edges on the inner and
// the outer face, come CollarWall, and the sampling slack, from the other two
// bores. TestSleeveWindowsFollowTheSize holds that the rule keeps every
// minimum at sizes well away from the defaults; a window the bores leave no
// room for is not cut (room).
// ---------------------------------------------------------------------------

// windowStep is the spacing, in mm, at which the window checks sample the
// windows' faces and each end's corner lines through the wall. windowSlack is
// the most the true least distance from a face to a channel can fall below
// the least sampled one: half the diagonal of a sample cell, and 5 microns for
// the stations the channel is walked at (wallGap). The ends keep it, and the
// check holds the sampled distance to CollarWall plus it.
const (
	windowStep  = 0.1
	windowSlack = windowStep/math.Sqrt2 + 5e-3
)

// postSlenderness is the tallest a post beside a window may stand where it is
// narrower than two CollarWalls, as a multiple of its narrowest width there.
// The video frame's 3 mm rods ran about 40 mm between the ring and the loop,
// over 13 times their width, and wobbled in print.
const postSlenderness = 2.0

// bore is one of the four channels: a gear and which side of the middle its
// cut is on.
type bore struct {
	g     Gear
	sign  float64 // +1 for the +R bore, -1 for the -R bore
	name  string
	index int // its place in bores()
}

func (f sleeve) bores() [4]bore {
	var out [4]bore
	for gi, g := range f.gears {
		for si, sign := range []float64{-1, 1} {
			name := [2]string{"gear A", "gear B"}[gi] + [2]string{" -R", " +R"}[si]
			out[gi*2+si] = bore{g: g, sign: sign, name: name, index: gi*2 + si}
		}
	}
	return out
}

// span is the stretch of the gear's axis the bore's cut covers.
func (b bore) span(f sleeve) (float64, float64) {
	if b.sign > 0 {
		return f.sIn, f.sOut
	}
	return -f.sOut, -f.sIn
}

// crossing is where the bore's axis meets the frame's circle.
func (b bore) crossing() r3.Vec {
	return b.g.Origin.Add(b.g.Ez.Scale(b.sign * b.g.P.CageRadius))
}

// flat is a convex polygon of at most eight corners, counter-clockwise, which
// is all a rectangle cut by four lines can come to.
type flat struct {
	n int
	v [8][2]float64
}

// clip keeps the part of a convex polygon where a*x + b*y <= c.
func (q flat) clip(a, b, c float64) flat {
	var out flat
	for i := range q.n {
		p, r := q.v[i], q.v[(i+1)%q.n]
		fp, fr := c-(a*p[0]+b*p[1]), c-(a*r[0]+b*r[1])
		if fp >= 0 {
			out.v[out.n] = p
			out.n++
		}
		if (fp >= 0) != (fr >= 0) {
			k := fp / (fp - fr)
			out.v[out.n] = [2]float64{p[0] + k*(r[0]-p[0]), p[1] + k*(r[1]-p[1])}
			out.n++
		}
	}
	return out
}

// gap2 is the squared distance from (x, y) to the polygon, zero inside.
func (q flat) gap2(x, y float64) float64 {
	inside := q.n >= 3
	best := math.Inf(1)
	for i := range q.n {
		p, r := q.v[i], q.v[(i+1)%q.n]
		ex, ey := r[0]-p[0], r[1]-p[1]
		if ex*(y-p[1])-ey*(x-p[0]) < 0 {
			inside = false
		}
		k := 0.0
		if l2 := ex*ex + ey*ey; l2 > 0 {
			k = math.Max(0, math.Min(1, ((x-p[0])*ex+(y-p[1])*ey)/l2))
		}
		dx, dy := x-p[0]-k*ex, y-p[1]-k*ey
		best = math.Min(best, dx*dx+dy*dy)
	}
	if inside {
		return 0
	}
	return best
}

// sectionInWall is a bore's channel at station s cut down to the part that
// lies in the tube, in the gear's own section coordinates: x along Ex and y
// along Ey. The section plane is square to the gear's axis, which is level and
// meets the frame's axis, so a point at (x, y) in it stands at radius
// hypot(s, y) and at height Origin.Z + x*Ex.Z. The part in the tube is the
// turned rectangle clipped to the strips of y that lie between the tube's
// radii, one on each side of the gear's axis, and to the heights between its
// end faces.
func (f sleeve) sectionInWall(g Gear, s float64) [2]flat {
	var out [2]flat
	if math.Abs(s) >= f.ro {
		return out
	}
	p := g.P
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	c, sn := math.Cos(g.angle(s)), math.Sin(g.angle(s))
	rect := flat{n: 4}
	for i, uv := range [4][2]float64{{-hw, -ht}, {hw, -ht}, {hw, ht}, {-hw, ht}} {
		rect.v[i] = [2]float64{uv[0]*c - uv[1]*sn, uv[0]*sn + uv[1]*c}
	}
	xa, xb := (-f.zb-g.Origin.Z)/g.Ex.Z, (f.zb-g.Origin.Z)/g.Ex.Z
	if xa > xb {
		xa, xb = xb, xa
	}
	rect = rect.clip(1, 0, xb).clip(-1, 0, -xa)
	near := math.Sqrt(math.Max(0, f.ri*f.ri-s*s))
	far := math.Sqrt(f.ro*f.ro - s*s)
	for i, side := range []float64{-1, 1} {
		out[i] = rect.clip(0, side, far).clip(0, -side, -near)
	}
	return out
}

// wallGap is the distance from q to the part of a bore's channel that lies in
// the tube, or reach when that part is at least reach away. The channel is
// the union of its sections, so the distance is the least, over stations, of
// the distance along the axis to the station's plane combined with the
// distance within that plane to the section. Every point of a section lies
// within BoreCorner of the gear's axis, which rules most points out before
// any station is walked.
//
// The stations are those of channelSections: every 2 microns along the bore's
// cut, from its first station. A section's points move at most
// hypot(1, BoreCorner/Lambda), 1.45, per unit of station, and the tube's faces
// clip them at up to about 4 away from the few stations where a face is
// tangent to the section's plane, so between two stations the distance dips at
// most 4 microns under the nearer. The exception is where a piece of the
// section enters or leaves the tube, which is where the least distance often
// is: the channel's corner first reaching into the wall. Wherever a piece
// appears or vanishes between two stations, the stretch between them is
// sampled again at a tenth of a micron.
//
// The walk starts at the station nearest q's own and goes outward both ways.
// A station further along the axis than the least distance found so far
// cannot come nearer, and neither can any beyond it, so each way stops there;
// a piece whose bounding circle is further off than that is passed over. The
// least distance is the same one a walk of every station would find.
func (f sleeve) wallGap(b bore, q r3.Vec, reach float64) float64 {
	g := b.g
	lo, hi := b.span(f)
	d := q.Sub(g.Origin)
	x, y, sq := d.Dot(g.Ex), d.Dot(g.Ey), d.Dot(g.Ez)
	axis := math.Hypot(math.Hypot(x, y), sq-math.Max(lo, math.Min(hi, sq)))
	if axis-g.P.BoreCorner() >= reach {
		return reach
	}
	best := reach * reach
	st := f.sections[b.index]
	// visit takes one station into account and says whether the walk goes on
	// past it: whether a station that far along the axis could still come
	// nearer than best.
	visit := func(c *channelStation) bool {
		ds := (sq - c.s) * (sq - c.s)
		if ds >= best {
			return false
		}
		for i := range c.pieces {
			if c.pieces[i].n == 0 {
				continue
			}
			if o := math.Hypot(x-c.cx[i], y-c.cy[i]) - c.r[i]; o > 0 && ds+o*o >= best {
				continue
			}
			best = math.Min(best, ds+c.pieces[i].gap2(x, y))
		}
		return true
	}
	at := sort.Search(len(st), func(i int) bool { return st[i].s >= sq })
	for i := at; i < len(st) && visit(&st[i]); i++ {
	}
	for i := at - 1; i >= 0 && visit(&st[i]); i-- {
	}
	return math.Sqrt(best)
}

// channelStation is one station of a bore's channel in the tube: its pieces,
// and a circle round each that holds all of it.
type channelStation struct {
	s         float64
	pieces    [2]flat
	cx, cy, r [2]float64
}

// channelSections is a bore's channel in the tube at every station wallGap
// walks, in order: every 2 microns from the cut's first station to its last,
// and every tenth of a micron between two of those where a piece appears or
// vanishes.
func (f sleeve) channelSections(b bore) []channelStation {
	const step, fine = 0.002, 0.0001
	lo, hi := b.span(f)
	make1 := func(s float64) channelStation {
		c := channelStation{s: s, pieces: f.sectionInWall(b.g, s)}
		for i, piece := range c.pieces {
			if piece.n == 0 {
				continue
			}
			for k := range piece.n {
				c.cx[i] += piece.v[k][0] / float64(piece.n)
				c.cy[i] += piece.v[k][1] / float64(piece.n)
			}
			for k := range piece.n {
				c.r[i] = math.Max(c.r[i], math.Hypot(piece.v[k][0]-c.cx[i], piece.v[k][1]-c.cy[i]))
			}
		}
		return c
	}
	has := func(c channelStation) [2]bool { return [2]bool{c.pieces[0].n > 0, c.pieces[1].n > 0} }
	var out []channelStation
	for k := 0; ; k++ {
		s := lo + float64(k)*step
		if s > hi {
			break
		}
		c := make1(s)
		if k > 0 && has(c) != has(out[len(out)-1]) {
			prev := out[len(out)-1].s
			for j := 1; prev+float64(j)*fine < s; j++ {
				out = append(out, make1(prev+float64(j)*fine))
			}
		}
		out = append(out, c)
	}
	return out
}

// wallCorners calls fn with every corner of every section of a bore's channel
// in the tube, in world space, at stations a micron apart. Any linear measure
// of the channel's part in the tube is extreme at one of them.
func (f sleeve) wallCorners(b bore, fn func(r3.Vec)) {
	g := b.g
	lo, hi := b.span(f)
	for s := lo; s <= hi; s += 0.001 {
		for _, piece := range f.sectionInWall(g, s) {
			for i := range piece.n {
				fn(g.Origin.Add(g.Ex.Scale(piece.v[i][0])).Add(g.Ey.Scale(piece.v[i][1])).Add(g.Ez.Scale(s)))
			}
		}
	}
}

// sleeveWindow is one window: the hexagon, and the prism it is pushed out as.
type sleeveWindow struct {
	facing, across r3.Vec  // the level direction it is cut toward, and t's direction: Z x facing
	lean           float64 // +1 or -1: the band is lo <= z + lean*t <= hi
	lo, hi         float64
	bottom, top    float64 // the trims: bottom <= z - lean*t <= top
	left, right    float64 // the upright ends
	zLimit         float64 // the height the channels reach, which the trims keep it within
	corners        [][2]float64
}

// plane is a point's place on the window's plane, and how far it stands in
// front of the plane through the frame's axis.
func (w sleeveWindow) plane(pt r3.Vec) (t, z, a float64) {
	return pt.Dot(w.across), pt.Z, pt.Dot(w.facing)
}

// inside answers whether a point of the window's plane is in its hexagon.
func (w sleeveWindow) inside(t, z float64) bool {
	band, trim := z+w.lean*t, z-w.lean*t
	return band >= w.lo && band <= w.hi && trim >= w.bottom && trim <= w.top && t >= w.left && t <= w.right
}

// contains answers whether a point is in the window's cut: in front of the
// plane through the frame's axis, on the hexagon's prism.
func (w sleeveWindow) contains(pt r3.Vec) bool {
	t, z, a := w.plane(pt)
	return a > 0 && w.inside(t, z)
}

// at is the point of the prism at (t, z) on the window's plane, a in front.
func (w sleeveWindow) at(t, z, a float64) r3.Vec {
	return w.across.Scale(t).Add(w.facing.Scale(a)).Add(r3.NewVec(0, 0, z))
}

// chord is the stretch of a's over which the prism's line at t is in the tube:
// from the inner face to the outer.
func (f sleeve) chord(t float64) (float64, float64) {
	return math.Sqrt(math.Max(0, f.ri*f.ri-t*t)), math.Sqrt(math.Max(0, f.ro*f.ro-t*t))
}

// newWindow finds the window facing a level direction from the bores.
func (f sleeve) newWindow(facing r3.Vec) sleeveWindow {
	p := f.p
	w := sleeveWindow{facing: facing, across: r3.NewVec(0, 0, 1).Cross(facing)}

	// The flanking bores are the two whose crossings lie on the window's side.
	var flanks, far []bore
	for _, b := range f.bores() {
		if b.crossing().Dot(facing) > 0 {
			flanks = append(flanks, b)
		} else {
			far = append(far, b)
		}
	}
	if len(flanks) != 2 {
		panic("a window's side of the tube does not hold two bores")
	}
	low, high := flanks[0], flanks[1]
	if low.crossing().Z > high.crossing().Z {
		low, high = high, low
	}
	w.lean = 1
	if high.crossing().Dot(w.across) < low.crossing().Dot(w.across) {
		w.lean = -1
	}

	// The sides: CollarWall across the band beyond each flanking bore's reach.
	// The band's measure z + lean*t grows by sqrt(2) per mm across it.
	lowReach, highReach := math.Inf(-1), math.Inf(1)
	f.wallCorners(low, func(pt r3.Vec) {
		t, z, _ := w.plane(pt)
		lowReach = math.Max(lowReach, z+w.lean*t)
	})
	f.wallCorners(high, func(pt r3.Vec) {
		t, z, _ := w.plane(pt)
		highReach = math.Min(highReach, z+w.lean*t)
	})
	w.lo = lowReach + math.Sqrt2*p.CollarWall
	w.hi = highReach - math.Sqrt2*p.CollarWall

	// The trims: each long corner no higher than the channels reach, and each
	// side cut off where it would meet the inner face past 45 degrees round.
	// The side lo meets the trim bottom at t = lean*(lo - bottom)/2, and hi
	// meets top at t = lean*(hi - top)/2.
	w.zLimit, _ = f.channelTop()
	w.top = math.Min(2*w.zLimit-w.hi, w.hi+math.Sqrt2*f.ri)
	w.bottom = math.Max(-2*w.zLimit-w.lo, w.lo-math.Sqrt2*f.ri)

	// The ends: as far out as both corners of the end keep their distance.
	w.left = -f.windowEnd(w, far, -1)
	w.right = f.windowEnd(w, far, 1)

	// The hexagon's corners, clipped from a square that holds the tube.
	q := [][2]float64{{-2 * f.ro, -2 * f.ro}, {2 * f.ro, -2 * f.ro}, {2 * f.ro, 2 * f.ro}, {-2 * f.ro, 2 * f.ro}}
	for _, h := range [6][3]float64{
		{w.lean, 1, w.hi}, {-w.lean, -1, -w.lo},
		{-w.lean, 1, w.top}, {w.lean, -1, -w.bottom},
		{1, 0, w.right}, {-1, 0, -w.left},
	} {
		q = clipCorners(q, h[0], h[1], h[2])
	}
	w.corners = q
	return w
}

// clipCorners keeps the part of a convex polygon of any size where
// a*t + b*z <= c.
func clipCorners(q [][2]float64, a, b, c float64) [][2]float64 {
	var out [][2]float64
	for i := range q {
		p, r := q[i], q[(i+1)%len(q)]
		fp, fr := c-(a*p[0]+b*p[1]), c-(a*r[0]+b*r[1])
		if fp >= 0 {
			out = append(out, p)
		}
		if (fp >= 0) != (fr >= 0) {
			k := fp / (fp - fr)
			out = append(out, [2]float64{p[0] + k*(r[0]-p[0]), p[1] + k*(r[1]-p[1])})
		}
	}
	return out
}

// windowEnd is how far out on the side dir the window's upright end can
// stand: the farthest t, no further out than SleeveInner or SleeveOuter/sqrt(2),
// at which every point below keeps CollarWall and the slack from each bore
// that does not flank the window. The points are both of the end's corners,
// walked along their lines through the wall at windowStep, and the end's
// upright edges on the inner and the outer face at the heights the full
// check (eachWindowFacePoint) samples them at: its walk along the edge, and
// its walk of the openings, which steps up from -zLimit and lands on an end
// that stands on its grid. The corners alone held at the defaults, but at a
// 0.2 mm clearance they let an end's edge on the inner face come 2.60 mm from
// a far bore against a 3 mm CollarWall. An end
// the band and the trims have already closed off is taken as too far. The
// search bisects 24 times between 0 and the limit.
func (f sleeve) windowEnd(w sleeveWindow, far []bore, dir float64) float64 {
	need := f.p.CollarWall + windowSlack
	clear := func(te float64) bool {
		t := dir * te
		zLow := math.Max(w.lo-w.lean*t, w.bottom+w.lean*t)
		zHigh := math.Min(w.hi-w.lean*t, w.top+w.lean*t)
		if zLow > zHigh {
			return false
		}
		a0, a1 := f.chord(t)
		near := func(z, a float64) bool {
			for _, b := range far {
				if f.wallGap(b, w.at(t, z, a), need) < need {
					return true
				}
			}
			return false
		}
		for _, z := range [2]float64{zLow, zHigh} {
			for a := a0; ; a = math.Min(a+windowStep, a1) {
				if near(z, a) {
					return false
				}
				if a >= a1 {
					break
				}
			}
		}
		// The end's upright edges on the inner and the outer face, between
		// the two corners, at the heights eachWindowFacePoint walks the end
		// at: its walk along the edge, and its walk of the openings, which
		// steps up from -zLimit.
		n := int(math.Ceil((zHigh - zLow) / windowStep))
		for _, a := range [2]float64{a0, a1} {
			for k := 1; k < n; k++ {
				if near(zLow+(zHigh-zLow)*float64(k)/float64(n), a) {
					return false
				}
			}
			for z := -w.zLimit; z <= w.zLimit; z += windowStep {
				if z > zLow && z < zHigh && near(z, a) {
					return false
				}
			}
		}
		return true
	}
	lo, hi := 0.0, math.Min(f.ri, f.ro/math.Sqrt2)*(1-1e-9)
	for range 24 {
		m := (lo + hi) / 2
		if clear(m) {
			lo = m
		} else {
			hi = m
		}
	}
	return lo
}

// eachWindowFacePoint walks, at step, every face of a window's cut that lies
// in the tube: each edge of the hexagon, along its line through the wall from
// the inner face to the outer, and the hexagon's inside on the inner and the
// outer face, which are the window's two openings. The least distance from
// the cut to anything outside it is reached on one of these.
func (f sleeve) eachWindowFacePoint(w sleeveWindow, step float64, fn func(r3.Vec)) {
	line := func(t, z float64) {
		if math.Abs(t) >= f.ri {
			return
		}
		a0, a1 := f.chord(t)
		for a := a0; ; a = math.Min(a+step, a1) {
			fn(w.at(t, z, a))
			if a >= a1 {
				break
			}
		}
	}
	for i := range w.corners {
		p, q := w.corners[i], w.corners[(i+1)%len(w.corners)]
		n := int(math.Ceil(math.Hypot(q[0]-p[0], q[1]-p[1]) / step))
		for k := range n {
			line(p[0]+(q[0]-p[0])*float64(k)/float64(n), p[1]+(q[1]-p[1])*float64(k)/float64(n))
		}
	}
	for t := w.left; t <= w.right; t += step {
		if math.Abs(t) >= f.ri {
			continue
		}
		a0, a1 := f.chord(t)
		for z := -w.zLimit; z <= w.zLimit; z += step {
			if w.inside(t, z) {
				fn(w.at(t, z, a0))
				fn(w.at(t, z, a1))
			}
		}
	}
}

// area is the hexagon's area, by the shoelace formula.
func (w sleeveWindow) area() float64 {
	a := 0.0
	for i := range w.corners {
		p, q := w.corners[i], w.corners[(i+1)%len(w.corners)]
		a += p[0]*q[1] - q[0]*p[1]
	}
	return a / 2
}

// room is why the window is not cut, or "" when it is.
func (w sleeveWindow) room() string {
	if w.hi <= w.lo {
		return "the flanking bores leave no band between them"
	}
	if w.right <= w.left || len(w.corners) < 3 || w.area() <= 0 {
		return "the far bores leave the band no length"
	}
	return ""
}

// The windows are cut where the wall has room for them, and nowhere they cost
// the frame a minimum it had.
//
// Each window keeps a CollarWall of solid wall round every channel. For each
// bore the check first looks for a side of the hexagon that the whole of the
// bore's channel in the tube lies CollarWall beyond, measured on the window's
// plane. A channel point that projects that far from the hexagon is at least
// that far from any point of the prism, so such a side settles the bore
// exactly; it is how the two bores flanking a window are held off, and it is
// how the window's sides were placed. A bore no side holds off is measured in
// space: every face of the window's cut in the tube is walked at windowStep and
// its distance to the bore's channel in the tube taken, and the least has to
// clear CollarWall by windowSlack, which is what the sampling can miss.
//
// The window stays inside the height the channels reach, so the end bands keep
// the 3.74 mm end wall the channels leave; every edge of the hexagon is upright
// or at 45 degrees or steeper; and where a window's face meets the tube's inner
// or outer face the material comes to an edge no sharper than the bores'
// mouths are held to.
func TestSleeveWindowsKeepTheirWalls(t *testing.T) {
	f := defaultSleeve()
	if len(f.windows) != 2 {
		t.Fatalf("the sleeve has %d windows, want 2", len(f.windows))
	}
	checkWindowWalls(t, f)
}

// checkWindowWalls is TestSleeveWindowsKeepTheirWalls for every window any
// sleeve has.
func checkWindowWalls(t testing.TB, f sleeve) {
	p := f.p
	cw := p.CollarWall
	top, _ := f.channelTop()

	for wi, w := range f.windows {
		if len(w.corners) < 3 || w.area() <= 0 {
			t.Fatalf("window %d is empty: the bores leave no band for it", wi)
		}
		name := fmt.Sprintf("the window facing (%+.0f, %+.0f)", w.facing.X, w.facing.Y)

		// The end bands and the edges' slopes.
		zMax, steepest := 0.0, 90.0
		for i, c := range w.corners {
			zMax = math.Max(zMax, math.Abs(c[1]))
			q := w.corners[(i+1)%len(w.corners)]
			dt, dz := math.Abs(q[0]-c[0]), math.Abs(q[1]-c[1])
			if dt < 1e-9 && dz < 1e-9 {
				continue
			}
			rise := math.Atan2(dz, dt) * 180 / math.Pi
			steepest = math.Min(steepest, rise)
			if rise < 45-1e-9 {
				t.Errorf("%s has an edge from (%.2f, %.2f) to (%.2f, %.2f) rising at %.1f degrees, under 45",
					name, c[0], c[1], q[0], q[1], rise)
			}
		}
		if zMax > top+1e-9 {
			t.Errorf("%s reaches %.3f mm from the middle, past the %.3f mm the channels reach, so an end band "+
				"is thinner than the %.2f mm end wall", name, zMax, top, f.zb-top)
		}

		// The bores.
		for _, b := range f.bores() {
			held, by := false, 0.0
			var pts []r3.Vec
			f.wallCorners(b, func(pt r3.Vec) { pts = append(pts, pt) })
			for i, c := range w.corners {
				q := w.corners[(i+1)%len(w.corners)]
				// The outward normal of a counter-clockwise polygon's edge.
				nt, nz := q[1]-c[1], -(q[0] - c[0])
				l := math.Hypot(nt, nz)
				if l < 1e-9 {
					continue
				}
				nt, nz = nt/l, nz/l
				least := math.Inf(1)
				for _, pt := range pts {
					pt2, z, _ := w.plane(pt)
					least = math.Min(least, (pt2-c[0])*nt+(z-c[1])*nz)
				}
				if least > by {
					by = least
				}
				if least >= cw-1e-9 {
					held = true
				}
			}
			if held {
				t.Logf("%s: %s's channel lies %.3f mm beyond one of its sides", name, b.name, by)
				continue
			}
			need := cw + windowSlack
			var mu sync.Mutex
			least, where := need+1, r3.Vec{}
			var wg sync.WaitGroup
			var batch []r3.Vec
			flush := func(pts []r3.Vec) {
				wg.Go(func() {
					lb, lw := need+1, r3.Vec{}
					for _, q := range pts {
						if d := f.wallGap(b, q, need+1); d < lb {
							lb, lw = d, q
						}
					}
					mu.Lock()
					if lb < least {
						least, where = lb, lw
					}
					mu.Unlock()
				})
			}
			f.eachWindowFacePoint(w, windowStep, func(q r3.Vec) {
				batch = append(batch, q)
				if len(batch) == 4096 {
					flush(batch)
					batch = nil
				}
			})
			flush(batch)
			wg.Wait()
			if least < need {
				t.Errorf("%s comes %.6f mm from %s's channel at radius %.2f, height %.2f, azimuth %.1f: under "+
					"the %.1f mm CollarWall and the %.3f mm slack", name, least, b.name, math.Hypot(where.X, where.Y),
					where.Z, math.Atan2(where.Y, where.X)*180/math.Pi, cw, windowSlack)
			}
			t.Logf("%s: no side holds off %s, and its faces keep %.3f mm from its channel, nearest at radius "+
				"%.2f, height %.2f, azimuth %.1f", name, b.name, least, math.Hypot(where.X, where.Y), where.Z,
				math.Atan2(where.Y, where.X)*180/math.Pi)
		}

		// Where the window's faces meet the tube's. The material between a face
		// and the tube's inner face comes to an edge of 180 degrees less the
		// angle between the two faces' normals into the material.
		sharpest := 180.0
		for i, c := range w.corners {
			q := w.corners[(i+1)%len(w.corners)]
			nt, nz := q[1]-c[1], -(q[0] - c[0])
			l := math.Hypot(nt, nz)
			if l < 1e-9 {
				continue
			}
			into := w.across.Scale(nt / l).Add(r3.NewVec(0, 0, nz/l))
			for k := 0.0; k <= 1; k += 0.01 {
				tt := c[0] + (q[0]-c[0])*k
				if math.Abs(tt) >= f.ri {
					continue
				}
				a0, a1 := f.chord(tt)
				for _, face := range []struct{ a, r, out float64 }{{a0, f.ri, 1}, {a1, f.ro, -1}} {
					radial := w.across.Scale(tt).Add(w.facing.Scale(face.a)).Scale(face.out / face.r)
					edge := 180 - math.Acos(math.Max(-1, math.Min(1, into.Dot(radial))))*180/math.Pi
					sharpest = math.Min(sharpest, edge)
				}
			}
		}
		if sharpest < minMouthWedge {
			t.Errorf("%s leaves a %.1f degree edge where it meets the tube, under the %.0f degree floor",
				name, sharpest, minMouthWedge)
		}

		// Where a roof meets the tube's faces. A roof is an edge that is not
		// upright; the layer that carries it lies toward sign(nt) across the
		// window in either orientation, and on the inner face that is toward
		// the hollow where it points at the middle, on the outer face where it
		// points away. There the line the roof meets the face on descends at
		// atan(slope*cos psi), and has to keep to the 45 degree rule measured
		// one cell diagonally: atan(1/sqrt(2)), 35.26 degrees.
		flattest := 90.0
		for i, c := range w.corners {
			q := w.corners[(i+1)%len(w.corners)]
			nt, nz := q[1]-c[1], -(q[0] - c[0])
			if math.Abs(nz) < 1e-9 || math.Hypot(nt, nz) < 1e-9 {
				continue
			}
			slope := math.Abs(nt / nz)
			for k := range 101 {
				tt := c[0] + (q[0]-c[0])*float64(k)/100
				for _, face := range []struct{ r, toward float64 }{{f.ri, -1}, {f.ro, 1}} {
					if math.Abs(tt) >= face.r || math.Copysign(1, nt)*tt*face.toward <= 0 {
						continue
					}
					cosPsi := math.Sqrt(1 - tt*tt/(face.r*face.r))
					flattest = math.Min(flattest, math.Atan(slope*cosPsi)*180/math.Pi)
				}
			}
		}
		if limit := math.Atan(1/math.Sqrt2) * 180 / math.Pi; flattest < limit-1e-6 {
			t.Errorf("%s has a roof meeting the tube on a line that descends at %.1f degrees, flatter than the "+
				"%.2f degrees one cell diagonally allows", name, flattest, limit)
		}

		// The material the window takes: the hexagon times the wall's depth
		// along the prism at each t.
		removed := 0.0
		const h = 0.02
		for tt := w.left + h/2; tt < w.right; tt += h {
			a0, a1 := f.chord(tt)
			for z := -w.zLimit + h/2; z < w.zLimit; z += h {
				if w.inside(tt, z) {
					removed += (a1 - a0) * h * h
				}
			}
		}

		var cs []string
		for _, c := range w.corners {
			cs = append(cs, fmt.Sprintf("(%.2f, %.2f)", c[0], c[1]))
		}
		band, trim := "z + t", "z - t"
		if w.lean < 0 {
			band, trim = trim, band
		}
		t.Logf("%s: %.3f <= %s <= %.3f, %.2f mm across its band (%.2f tall), ends at t = %.3f "+
			"and %.3f, trims %.3f <= %s <= %.3f, reaching |z| = %.2f against the channels' %.2f; corners %s; "+
			"%.1f mm^2 on its plane; takes %.0f mm^3, about %.1f g of PLA; edges rise at %.0f degrees or more; "+
			"its faces leave edges of %.1f degrees or more on the tube, and its roofs meet the tube on lines "+
			"descending at %.1f degrees or more", name, w.lo, band, w.hi, (w.hi-w.lo)/math.Sqrt2,
			w.hi-w.lo, w.left, w.right, w.bottom, trim, w.top, zMax, top, strings.Join(cs, " "), w.area(),
			removed, removed*1.24e-3, steepest, sharpest, flattest)
	}
}

// The wall a window leaves beside it stands firm. Level sections are cut
// through the sleeve every 0.1 mm over the windows' height, and each is walked
// round at the inner face, the middle of the wall and the outer face at 0.1
// degree steps. The material between two openings, a window on at least one
// side, is a post at that height, and its width is the arc it spans. Every
// post has to be at least CollarWall wide. Where a post is narrower than two
// CollarWalls it is a slender member rather than wall, and the heights over
// which the same two openings keep it that narrow, without a break, may run to
// no more than postSlenderness times its narrowest width there.
func TestSleeveWindowPostsStandFirm(t *testing.T) {
	checkWindowPosts(t, defaultSleeve())
}

// checkWindowPosts is TestSleeveWindowPostsStandFirm for any sleeve.
func checkWindowPosts(t testing.TB, f sleeve) {
	p := f.p
	bores := f.bores()
	names := make([]string, 0, len(bores)+len(f.windows))
	for _, b := range bores {
		names = append(names, b.name)
	}
	for _, w := range f.windows {
		names = append(names, fmt.Sprintf("the window facing (%+.0f, %+.0f)", w.facing.X, w.facing.Y))
	}
	// opening names what a point is cut by, or -1 for material.
	opening := func(pt r3.Vec) int {
		for wi, w := range f.windows {
			if w.contains(pt) {
				return len(bores) + wi
			}
		}
		for bi, b := range bores {
			if lo, hi := b.span(f); inSleeveChannel(b.g, f.sIn, f.sOut, pt) {
				if s := pt.Sub(b.g.Origin).Dot(b.g.Ez); s >= lo && s <= hi {
					return bi
				}
			}
		}
		return -1
	}

	const steps = 3600
	zTop := 0.0
	for _, w := range f.windows {
		zTop = math.Max(zTop, w.zLimit)
	}
	type arc struct {
		a, b  int // the openings either side, in order of increasing azimuth
		width float64
	}
	radii := []float64{f.ri + 1e-3, (f.ri + f.ro) / 2, f.ro - 1e-3}
	heights := int(math.Round(2*zTop/0.1)) + 1
	arcs := make([][][]arc, heights) // by height, then radius
	var wg sync.WaitGroup
	for k := range heights {
		wg.Go(func() {
			z := -zTop + 0.1*float64(k)
			arcs[k] = make([][]arc, len(radii))
			for ri, r := range radii {
				var labels [steps]int
				first := -1
				for i := range steps {
					a := 2 * math.Pi * float64(i) / steps
					labels[i] = opening(r3.NewVec(r*math.Cos(a), r*math.Sin(a), z))
					if first < 0 && labels[i] >= 0 {
						first = i
					}
				}
				if first < 0 {
					continue
				}
				for n := 0; n < steps; {
					i := (first + n) % steps
					if labels[i] >= 0 {
						n++
						continue
					}
					run := 0
					for labels[(i+run)%steps] < 0 {
						run++
					}
					before, after := labels[(i-1+steps)%steps], labels[(i+run)%steps]
					if before >= len(bores) || after >= len(bores) {
						arcs[k][ri] = append(arcs[k][ri], arc{before, after, float64(run) * 2 * math.Pi / steps * r})
					}
					n += run
				}
			}
		})
	}
	wg.Wait()

	narrowest, narrowestAt, narrowestZ := math.Inf(1), "", 0.0
	type stretch struct {
		from, to, width float64
	}
	worst, worstRatio, worstName := stretch{}, 0.0, ""
	for ri := range radii {
		open := map[[2]int]*stretch{}
		closeStretch := func(key [2]int, s *stretch) {
			if tall := s.to - s.from + 0.1; tall/s.width > worstRatio {
				worst, worstRatio, worstName = *s, tall/s.width, names[key[0]]+" and "+names[key[1]]
			}
		}
		for k := range heights {
			z := -zTop + 0.1*float64(k)
			seen := map[[2]int]float64{}
			for _, a := range arcs[k][ri] {
				if a.width < narrowest {
					narrowest, narrowestAt, narrowestZ = a.width, names[a.a]+" and "+names[a.b], z
				}
				if a.width < 2*p.CollarWall {
					key := [2]int{a.a, a.b}
					if w, ok := seen[key]; !ok || a.width < w {
						seen[key] = a.width
					}
				}
			}
			for key, s := range open {
				if _, ok := seen[key]; !ok {
					closeStretch(key, s)
					delete(open, key)
				}
			}
			for key, width := range seen {
				if s, ok := open[key]; ok {
					s.to, s.width = z, math.Min(s.width, width)
					continue
				}
				open[key] = &stretch{from: z, to: z, width: width}
			}
		}
		for key, s := range open {
			closeStretch(key, s)
		}
	}
	if narrowest < p.CollarWall {
		t.Errorf("the post between %s is %.2f mm wide at height %.2f, under the %.1f mm CollarWall",
			narrowestAt, narrowest, narrowestZ, p.CollarWall)
	}
	tall := worst.to - worst.from + 0.1
	if worstRatio > postSlenderness {
		t.Errorf("the post between %s stays under %.0f mm wide from height %.2f to %.2f, %.1f mm tall on a "+
			"%.2f mm width: %.1f times, over the %.0f allowed", worstName, 2*p.CollarWall, worst.from, worst.to,
			tall, worst.width, worstRatio, postSlenderness)
	}
	t.Logf("beside the windows the narrowest post is %.2f mm, between %s at height %.2f; the most slender "+
		"stretch under %.0f mm is between %s, %.1f mm tall on %.2f mm, %.2f times its width against the %.0f allowed",
		narrowest, narrowestAt, narrowestZ, 2*p.CollarWall, worstName, tall, worst.width, worstRatio, postSlenderness)
}

// The windows show the mesh from the side. The mesh zone is the same set of
// points TestSleeveKeepsTheMeshVisibleAlongTheAxis projects into the
// footprint: both ribbons' crest rectangles within the engaged zone, here at
// every 0.25 mm of station. A point is seen through a window when a straight
// line from it leaves the tube through the window and misses both ribbons'
// crest rectangles on the way. The crest rectangle holds the whole ribbon, so
// a line that misses it misses the teeth too, and the count is a floor.
//
// The window is a convex prism, so a line that crosses the inner face inside
// it and the outer face inside it stays inside it through the wall, and the
// hollow holds no material. The test tries lines from the point to the outer
// opening every 0.5 mm, and every such line counts that crosses the inner face
// inside the window and clears the ribbons, walked at 0.1 mm. Each window has
// to show some of the mesh; the fractions are logged, with the lines that are
// level, as a person beside the frame at the mesh's height would look.
func TestSleeveWindowsShowTheMeshFromTheSide(t *testing.T) {
	checkSideView(t, defaultSleeve())
}

// checkSideView is TestSleeveWindowsShowTheMeshFromTheSide for any sleeve. It
// returns the share of the mesh zone seen through either window.
func checkSideView(t testing.TB, f sleeve) float64 {
	window := axialWindow(f.p)
	var points []r3.Vec
	for _, g := range f.gears {
		eachGrownEnvelopePoint(g, 0, -window, window, 0.25, func(pt r3.Vec, _ float64) {
			points = append(points, pt)
		})
	}
	// clearOf answers whether the line from m to q misses both ribbons'
	// crest rectangles, leaving m's own surface.
	clearOf := func(m, q r3.Vec) bool {
		d := q.Sub(m)
		l := d.Len()
		for s := 0.05; s < l; s += 0.1 {
			x := m.Add(d.Scale(s / l))
			for _, g := range f.gears {
				if g.envelopeMargin(x) > 1e-9 {
					return false
				}
			}
		}
		return true
	}
	// sees answers whether a line from m to the window's outer opening at
	// (tt, z) passes the wall inside the window and clears the ribbons.
	sees := func(w sleeveWindow, m r3.Vec, tt, z float64) bool {
		_, a1 := f.chord(tt)
		out := w.at(tt, z, a1)
		d := out.Sub(m)
		// Where the line crosses the inner face: r = ri, the larger root.
		qa := d.X*d.X + d.Y*d.Y
		qb := 2 * (m.X*d.X + m.Y*d.Y)
		qc := m.X*m.X + m.Y*m.Y - f.ri*f.ri
		disc := qb*qb - 4*qa*qc
		if disc < 0 {
			return false
		}
		in := m.Add(d.Scale((-qb + math.Sqrt(disc)) / (2 * qa)))
		if !w.contains(in) {
			return false
		}
		return clearOf(m, in)
	}
	type tally struct{ seen, level int }
	counts := make([]tally, len(f.windows))
	seenAny, seenLevel := make([]bool, len(points)), make([]bool, len(points))
	var mu sync.Mutex
	var wg sync.WaitGroup
	for wi, w := range f.windows {
		for from := 0; from < len(points); from += 256 {
			wg.Go(func() {
				var local tally
				for pi := from; pi < min(from+256, len(points)); pi++ {
					m := points[pi]
					seen, level := false, false
					for tt := w.left; tt <= w.right && !seen; tt += 0.5 {
						for z := -w.zLimit; z <= w.zLimit && !seen; z += 0.5 {
							seen = w.inside(tt, z) && sees(w, m, tt, z)
						}
					}
					for tt := w.left; tt <= w.right && !level; tt += 0.1 {
						level = w.inside(tt, m.Z) && sees(w, m, tt, m.Z)
					}
					if seen {
						local.seen++
					}
					if level {
						local.level++
					}
					mu.Lock()
					seenAny[pi] = seenAny[pi] || seen
					seenLevel[pi] = seenLevel[pi] || level
					mu.Unlock()
				}
				mu.Lock()
				counts[wi].seen += local.seen
				counts[wi].level += local.level
				mu.Unlock()
			})
		}
	}
	wg.Wait()
	pct := func(n int) float64 { return 100 * float64(n) / float64(len(points)) }
	for wi, w := range f.windows {
		c := counts[wi]
		if c.seen == 0 {
			t.Errorf("no line from the mesh zone leaves through the window facing (%+.0f, %+.0f)",
				w.facing.X, w.facing.Y)
		}
		t.Logf("through the window facing (%+.0f, %+.0f): %d of %d points of the mesh zone (%.1f%%), %d (%.1f%%) "+
			"along a level line", w.facing.X, w.facing.Y, c.seen, len(points), pct(c.seen), c.level, pct(c.level))
	}
	both, level := 0, 0
	for i := range points {
		if seenAny[i] {
			both++
		}
		if seenLevel[i] {
			level++
		}
	}
	t.Logf("through either window: %d of %d points of the mesh zone (%.1f%%), %d (%.1f%%) along a level line",
		both, len(points), pct(both), level, pct(level))
	return pct(both)
}

// leastRise is the least CageRise the build accepts for these inputs, to the
// quarter millimetre above the closed form sleeveRefusal holds it to.
func leastRise(p Params) float64 {
	return math.Ceil((p.AxisOffset()/2+p.BoreCorner()+p.CollarWall)*4) / 4
}

// sleeveSize is one input TestSleeveWindowsFollowTheSize builds the sleeve at.
type sleeveSize struct {
	name string
	p    Params
}

// windowSizes is the spread of inputs TestSleeveWindowsFollowTheSize builds
// the sleeve at: the whole ribbon and frame scaled, with CollarWall and the
// clearance held, since they are print lengths rather than sizes; single
// inputs moved off their defaults; and a 4 mm CollarWall beside the inputs
// that bring the bores nearest each other. Where an input asks for a taller
// sleeve than the default's, CageRise is raised to the least the build
// accepts.
func windowSizes() []sleeveSize {
	var out []sleeveSize
	add := func(name string, edit func(*Params)) {
		p := sleeveParams()
		edit(&p)
		p.CageRise = math.Max(p.CageRise, leastRise(p))
		out = append(out, sleeveSize{name, p})
	}
	for _, k := range []float64{2.0 / 3, 0.75, 0.8, 1.25, 1.5, 1.75} {
		add(fmt.Sprintf("everything scaled by %.3g", k), func(p *Params) {
			for _, v := range []*float64{&p.Width, &p.Thickness, &p.ToothHeight, &p.ToothPitch, &p.TwistLead,
				&p.Engagement, &p.CageRadius, &p.CollarHalf, &p.CageRise} {
				*v *= k
			}
		})
	}
	for _, v := range []float64{10, 12} {
		add(fmt.Sprintf("ribbon width %g", v), func(p *Params) { p.Width = v })
	}
	for _, v := range []float64{2.5, 5} {
		add(fmt.Sprintf("ribbon thickness %g", v), func(p *Params) { p.Thickness = v })
	}
	for _, v := range []float64{40, 60} {
		add(fmt.Sprintf("twist lead %g", v), func(p *Params) { p.TwistLead = v })
	}
	for _, v := range []float64{14, 17, 20, 25} {
		add(fmt.Sprintf("cage radius %g", v), func(p *Params) { p.CageRadius = v })
	}
	for _, v := range []float64{18.5, 25} {
		add(fmt.Sprintf("cage rise %g", v), func(p *Params) { p.CageRise = v })
	}
	for _, v := range []float64{0.2, 0.9} {
		add(fmt.Sprintf("clearance %g", v), func(p *Params) { p.Clearance = v })
	}
	for _, v := range []float64{2, 4} {
		add(fmt.Sprintf("collar half length %g", v), func(p *Params) { p.CollarHalf = v })
	}
	for _, v := range []float64{2, 4, 5} {
		add(fmt.Sprintf("collar wall %g", v), func(p *Params) { p.CollarWall = v })
	}
	for _, v := range []float64{70, 90, 100, 120} {
		add(fmt.Sprintf("crossing angle %g", v), func(p *Params) { p.CrossAngle = v * math.Pi / 180 })
	}
	add("mounting angles 0 and 30", func(p *Params) { p.MountAngleA, p.MountAngleB = 0, 30*math.Pi/180 })
	add("collar wall 4, clearance 0.9", func(p *Params) { p.CollarWall, p.Clearance = 4, 0.9 })
	add("collar wall 4, cage radius 14", func(p *Params) { p.CollarWall, p.CageRadius = 4, 14 })
	add("collar wall 4, crossing angle 70", func(p *Params) { p.CollarWall, p.CrossAngle = 4, 70*math.Pi/180 })
	return out
}

// The windows are sized from the sleeve they are cut in, so at every size the
// dialog accepts they have to keep every minimum the default windows keep, or
// be left out. This builds the sleeve at each input of windowSizes. An input
// the build refuses is logged with the reason; every refusal in the spread
// has to be the one for the wall between two bores, since the spread raises
// CageRise and keeps to the ranges the other three checks set. For every
// window of an accepted input it runs every check the default windows pass:
// the walls round every bore and the end bands
// (TestSleeveWindowsKeepTheirWalls), the posts (TestSleeveWindowPostsStandFirm),
// one piece and no material laid on air outside a bore on the same 0.25 mm
// grid, with the end wall, the wall between bores and the bores' mouths
// (TestSleeveIsOnePiece, TestSleevePrintsStandingOnEitherEnd); and it logs how
// much of the mesh zone the windows show
// (TestSleeveWindowsShowTheMeshFromTheSide).
//
// No accepted input in the spread leaves a window out. The rule that does is
// exercised last, at a 7 mm CollarWall, which leaves no band between the
// flanking bores; the build refuses that input for its bores anyway.
func TestSleeveWindowsFollowTheSize(t *testing.T) {
	sizes := windowSizes()
	var mu sync.Mutex
	accepted, cut := 0, 0
	t.Run("sizes", func(t *testing.T) {
		for _, size := range sizes {
			t.Run(size.name, func(t *testing.T) {
				t.Parallel()
				p := size.p
				ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
				if why := sleeveRefusal(p); why != "" {
					if why != refuseChannelsClose {
						t.Errorf("the build refuses this input because %s, which the spread should not reach", why)
					}
					plain := plainSleeve(ga, gb)
					sep, at := channelSeparation(plain)
					near, _ := nearestChannels(plain)
					t.Logf("the build refuses it: %s (%.3f mm by its check across the gap between %s; "+
						"nearestChannels measures %.3f mm)", why, sep, at, near)
					return
				}
				f := newSleeve(ga, gb)
				for _, facing := range windowFacings(p) {
					if why := f.newWindow(facing).room(); why != "" {
						t.Errorf("no window facing (%+.0f, %+.0f): %s", facing.X, facing.Y, why)
					}
				}
				for _, w := range f.windows {
					t.Logf("the window facing (%+.0f, %+.0f) is %.2f mm across its band, its ends %.2f mm apart, "+
						"%.1f mm^2 on its plane", w.facing.X, w.facing.Y, (w.hi-w.lo)/math.Sqrt2, w.right-w.left,
						w.area())
				}
				mu.Lock()
				accepted++
				cut += len(f.windows)
				mu.Unlock()
				checkWindowWalls(t, f)
				checkWindowPosts(t, f)
				v := voxelize(f, sleeveVoxel)
				checkSleeveIsOnePiece(t, v)
				checkSleevePrints(t, v)
				sep, _ := channelSeparation(f)
				t.Logf("sleeve %.2f to %.2f mm, %.2f mm tall; the build's check holds the bores %.3f mm apart; "+
					"the windows show %.1f%% of the mesh zone", f.ri, f.ro, 2*f.zb, sep, checkSideView(t, f))
			})
		}
	})
	t.Logf("%d sizes, %d accepted, %d windows cut", len(sizes), accepted, cut)

	p := sleeveParams()
	p.CollarWall = 7
	p.CageRise = leastRise(p)
	ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
	f := newSleeve(ga, gb)
	for _, facing := range windowFacings(p) {
		w := f.newWindow(facing)
		if w.room() == "" {
			t.Errorf("at a 7 mm CollarWall the window facing (%+.0f, %+.0f) still has room: band %.3f to %.3f",
				facing.X, facing.Y, w.lo, w.hi)
			continue
		}
		t.Logf("at a 7 mm CollarWall the window facing (%+.0f, %+.0f) is left out: %s", facing.X, facing.Y, w.room())
	}
	if len(f.windows) != 0 {
		t.Errorf("at a 7 mm CollarWall the sleeve still has %d windows", len(f.windows))
	}
}
