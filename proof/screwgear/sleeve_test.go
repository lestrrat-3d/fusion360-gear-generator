package screwgear_test

import (
	"fmt"
	"math"
	"slices"
	"sort"
	"strings"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// ---------------------------------------------------------------------------
// The frame: the printable sleeve.
//
// The frame used to be the one in Segerman's video, a skeleton of a wire ring,
// a loop, four thin rods and a twisted collar round each ribbon. It was printed
// several times and reported on 2026-09-30 as not practically printable: the
// rods, the ring and the loop hanging on them wobble while printing. The sleeve
// replaced it under two rules: the bores may not change, since they are what
// holds each gear to its screw motion, and the mesh has to stay visible from
// the top and the bottom.
//
// The sleeve is one thick-walled tube about the frame's axis, inner radius
// CageRadius - CollarHalf and outer radius CageRadius + CollarHalf, standing
// CageRise either side of the middle plane, with the four twisted bores cut
// straight through its wall and two slanted windows cut between them. On each
// bore's centre line the wall runs from station CageRadius - CollarHalf to
// CageRadius + CollarHalf of its gear's axis, so the far face of each bore is
// CageRadius + CollarHalf from the middle, which is what sets the travel. The
// mesh is seen along the axis through the hollow from either end, and from the
// side through the windows. The frame prints standing on either end with no
// support: every outside face is a vertical cylinder or a level end face, every
// face of a window is upright or at 45 degrees, and the only faces that need
// more are the ceilings of the four bores.
//
// spec/screwgear/instructions.md §4 builds the sleeve, and defaultParams is
// that spec's default table.
// ---------------------------------------------------------------------------

// SleeveInner and SleeveOuter are the sleeve's radii: CollarHalf either side
// of the bore's station, measured on the ribbon's axis from the middle, turned
// into a tube.
func (p Params) SleeveInner() float64 { return p.CageRadius - p.CollarHalf }
func (p Params) SleeveOuter() float64 { return p.CageRadius + p.CollarHalf }

// BoreCorner is how far the furthest corner of any bore stands from the
// ribbon's axis: the radius the channel sweeps out as it twists. The level
// bore's two corners on its roof side stand RoofAllowance further across the
// thickness than the rest, so they are the furthest.
func (p Params) BoreCorner() float64 {
	return math.Hypot(p.BoreHalfWidth(), p.BoreHalfThickness()+p.RoofAllowance)
}

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
	refuseChannelInWall = "the bore's corner, with the cut's margin, reaches past the sleeve's inner radius, so the cut would start in the wall"
	refuseMeshHidden    = "the mesh zone and its clearance reach past the sleeve's inner radius, so the mesh would not be visible along the axis"
	refuseEndWall       = "the channels come nearer the sleeve's end faces than CollarWall"
	refuseChannelsClose = "two neighbouring bores' channels come nearer each other than CollarWall"
)

// sleeveRefusal is the first of the build's checks the inputs fail, or ""
// when they pass every one.
func sleeveRefusal(p Params) string {
	ri := p.SleeveInner()
	// Each cut starts sleeveCutMargin before the channel's corner reaches the
	// inner face, at sqrt(ri^2 - c^2) - sleeveCutMargin along its axis. That
	// station has to be past the middle, or the two cuts of one gear overlap.
	if math.Hypot(p.BoreCorner(), sleeveCutMargin) >= ri {
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
	p := defaultParams()
	ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
	return newSleeve(ga, gb)
})

// inShell answers whether a point is in the uncut tube.
func (f sleeve) inShell(pt r3.Vec) bool {
	r := math.Hypot(pt.X, pt.Y)
	return r >= f.ri && r <= f.ro && math.Abs(pt.Z) <= f.zb
}

// inChannel answers whether a point is inside one of the four bores' cuts:
// the bore's rectangle, turned with the ribbon, over the cut's span.
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

// boreGap is how far outside the bore's opening a point lies, measured in the
// ribbon's own section at that station with the twist undone, and zero inside,
// with the station. The channel is the set where it is zero, a rectangle that
// turns with the ribbon. Which of the gear's two bores the point is measured
// against is the sign of its station.
func boreGap(g Gear, pt r3.Vec) (float64, float64) {
	u, v, s := g.local(pt)
	hw, vLo, vHi := boreOpening(g, math.Copysign(1, s))
	du := math.Max(0, math.Abs(u)-hw)
	dv := math.Max(0, math.Max(vLo-v, v-vHi))
	return math.Hypot(du, dv), s
}

// boreOpening is one bore's opening in its gear's own section, before the
// twist: u from -hw to hw and v from vLo to vHi. sign is -1 for the -R bore and
// +1 for the +R bore. Every bore is the crest rectangle plus Clearance all
// round, and the level bore (levelBore) also takes RoofAllowance on the long
// face that is its roof when the sleeve stands on its -n end (roofSide).
func boreOpening(g Gear, sign float64) (float64, float64, float64) {
	p := g.P
	hw, ht := p.BoreHalfWidth(), p.BoreHalfThickness()
	vLo, vHi := -ht, ht
	if sign != levelBore(g) {
		return hw, vLo, vHi
	}
	if roofSide(g, sign) > 0 {
		return hw, vLo, vHi + p.RoofAllowance
	}
	return hw, vLo - p.RoofAllowance, vHi
}

// openingCorners is boreOpening's four corners, counter-clockwise about +v x
// +u's normal: (-hw, vLo), (hw, vLo), (hw, vHi), (-hw, vHi).
func openingCorners(g Gear, sign float64) [4][2]float64 {
	hw, vLo, vHi := boreOpening(g, sign)
	return [4][2]float64{{-hw, vLo}, {hw, vLo}, {hw, vHi}, {-hw, vHi}}
}

// levelBore is which of a gear's two bores takes the roof allowance: the one
// whose long faces come nearest level over the wall's span on its centre line,
// CollarHalf either side of its station, -1 for the -R bore and +1 for the +R
// bore, the -R bore when the two tie. A long face runs along the section's u,
// which stands theta from the gear's Ex, and Ex is along the frame's axis, so
// the face is level where cos(theta) is zero. At the defaults, both mounting
// angles zero since 2026-10-03, both bores' faces pass through level inside the
// wall, at stations -12.37 and +12.37, so the two tie and the -R bore takes the
// allowance; the +R bore's roof is bridged with the clearance alone
// (TestSleevePrintsStandingOnEitherEnd logs both). At the 14 degrees of the
// third print the -R bore's faces passed through level at station -14.30 and
// the +R bore's came no nearer than 11.3 degrees.
func levelBore(g Gear) float64 {
	p := g.P
	tilt := func(sign float64) float64 {
		lo := g.angle(sign * (p.CageRadius - p.CollarHalf))
		hi := g.angle(sign * (p.CageRadius + p.CollarHalf))
		if lo > hi {
			lo, hi = hi, lo
		}
		if level := math.Pi/2 + math.Ceil((lo-math.Pi/2)/math.Pi)*math.Pi; level <= hi {
			return 0
		}
		return math.Min(math.Abs(math.Cos(lo)), math.Abs(math.Cos(hi)))
	}
	if tilt(1) < tilt(-1) {
		return 1
	}
	return -1
}

// roofSide is which long face of a gear's bore is its roof when the sleeve
// stands on its -n end: +1 when the +v face is the upper one at the bore's
// station, -1 when the -v face is. v's component along the frame's axis is
// -sin(theta) Ex.Z + cos(theta) Ey.Z.
func roofSide(g Gear, sign float64) float64 {
	th := g.angle(sign * g.P.CageRadius)
	if -math.Sin(th)*g.Ex.Z+math.Cos(th)*g.Ey.Z > 0 {
		return 1
	}
	return -1
}

// eachRibbonPoint walks a ribbon's whole surface at one tooth phase, from one
// end of the ribbon to the other.
func eachRibbonPoint(g Gear, step float64, fn func(pt r3.Vec, u, s float64)) {
	from, to := g.span()
	uLo, t := -g.P.Width/2, g.P.Thickness/2
	for s := from; s <= to; s += step {
		for i := range 5 {
			v := -t + 2*t*float64(i)/4
			uHi := g.edgeAt(v, s)
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

// travelLimits is how far the pair can advance from the assembly position,
// backward and forward, before an end of either ribbon leaves the wall's span
// of one of that ribbon's bores, CollarHalf either side of the bore's station
// on its centre line. Advancing a gear moves its ends with it, so a gear whose
// far end is D past a bore's far face can advance D that way. Gear B is
// assembled a fraction of a pitch along its axis, so its two limits differ by
// that much, and the pair's limit each way is the tighter gear's.
func (f sleeve) travelLimits() (float64, float64) {
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

// sectionReach is how far a bore's opening at station s, turned to its angle
// there, reaches from the gear's axis along Ex, which is the frame's axis, and
// along Ey, sideways across the frame's plane, each either way. The extremes
// are corners.
func sectionReach(g Gear, s float64) (float64, float64, float64, float64) {
	c, sn := math.Cos(g.angle(s)), math.Sin(g.angle(s))
	alongLo, sideLo := math.Inf(1), math.Inf(1)
	alongHi, sideHi := math.Inf(-1), math.Inf(-1)
	for _, q := range openingCorners(g, math.Copysign(1, s)) {
		x, y := q[0]*c-q[1]*sn, q[0]*sn+q[1]*c
		alongLo, alongHi = math.Min(alongLo, x), math.Max(alongHi, x)
		sideLo, sideHi = math.Min(sideLo, y), math.Max(sideHi, y)
	}
	return alongLo, alongHi, sideLo, sideHi
}

// inWall answers whether any of the bore's section at station s lies in the
// tube's radii. A point of the section is at least |s| from the frame's axis,
// since the axis runs radially in projection and the section is square to it,
// and at most hypot(s, sideways reach) from it.
func (f sleeve) inWall(g Gear, s float64) bool {
	_, _, lo, hi := sectionReach(g, s)
	return math.Abs(s) <= f.ro && math.Hypot(s, math.Max(-lo, hi)) >= f.ri
}

// channelTop is the furthest any channel reaches from the middle plane inside
// the wall, up or down, and the station it reaches it at. The roof allowance
// makes the sleeve differ either way up, so both ways are taken.
func (f sleeve) channelTop() (float64, float64) {
	top, at := 0.0, 0.0
	for _, g := range f.gears {
		for _, sign := range []float64{-1, 1} {
			for s := sign * f.sIn; math.Abs(s) <= f.sOut; s += sign * 0.01 {
				if !f.inWall(g, s) {
					continue
				}
				lo, hi, _, _ := sectionReach(g, s)
				for _, x := range []float64{lo, hi} {
					if z := math.Abs(g.Origin.Z + x*g.Ex.Z); z > top {
						top, at = z, s
					}
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
// where they sit. They are the channels the video frame's collars were cut
// with, and this pins the numbers a frame change must not move. The section
// and the angles moved once, on 2026-10-02, when the clearance went from 0.45
// to 0.20 mm and the mounting angles from 15 to 14 degrees because the printed
// pair did not mesh over the play the wider bores allowed
// (spec/screwgear/fusion.md [SCREW-F-PRINT-MESH]); the stations and the twist
// did not move. The section moved again the same day, when the second
// sleeve's bridged -R roofs printed too tight and each gear's level bore took
// a roof allowance on its roof face (spec/screwgear/fusion.md
// [SCREW-F-PRINT-2]). The angles moved again on 2026-10-03, from 123.1 and
// -95.1 degrees, when both mounting angles went to zero with the leaned tooth
// (spec/screwgear/fusion.md [SCREW-F-PRINT-3]).
func TestSleeveBoresAreTheSameChannels(t *testing.T) {
	f := defaultSleeve()
	p := f.p

	if got := p.BoreHalfWidth(); math.Abs(got-7.70) > 1e-12 {
		t.Errorf("the bore is %.4f mm across, want 15.4", 2*got)
	}
	if got := p.BoreHalfThickness(); math.Abs(got-2.075) > 1e-12 {
		t.Errorf("the bore is %.4f mm through, want 4.15", 2*got)
	}
	// The level bore of each gear is the -R bore, and its roof allowance is on
	// the face that is up when the sleeve stands on its -n end: +v for gear A,
	// whose Ex points up, and -v for gear B.
	for gi, g := range f.gears {
		if got := levelBore(g); got != -1 {
			t.Errorf("gear %d's level bore is its %+.0fR bore, want -R", gi, got)
		}
		want := [2][2]float64{{-2.075, 2.375}, {-2.375, 2.075}}[gi]
		if _, lo, hi := boreOpening(g, -1); math.Abs(lo-want[0]) > 1e-12 || math.Abs(hi-want[1]) > 1e-12 {
			t.Errorf("gear %d's -R bore spans v from %.3f to %.3f, want %.3f to %.3f", gi, lo, hi, want[0], want[1])
		}
		if _, lo, hi := boreOpening(g, 1); lo != -2.075 || hi != 2.075 {
			t.Errorf("gear %d's +R bore spans v from %.3f to %.3f, want +/-2.075", gi, lo, hi)
		}
	}
	if st := boreStations(p); st[0] != -15 || st[1] != 15 {
		t.Errorf("the bores sit at stations %v, want -15 and +15", st)
	}
	if got, want := p.Lambda(), 49.5/(2*math.Pi); math.Abs(got-want) > 1e-12 {
		t.Errorf("the bore twists at %.6f mm per radian, want %.6f", got, want)
	}
	for gi, g := range f.gears {
		for _, c := range []struct{ station, deg float64 }{{15, 109.09}, {-15, -109.09}} {
			if got := g.angle(c.station) * 180 / math.Pi; math.Abs(got-c.deg) > 0.005 {
				t.Errorf("gear %d's bore at station %+.0f stands at %.2f degrees, want %.2f",
					gi, c.station, got, c.deg)
			}
		}
	}
	// The wall's span on the bore's centre line is where the wall is complete
	// round the channel, and where the bore and travel tests walk it. The cut
	// has to cover it.
	if f.sIn > p.CageRadius-p.CollarHalf || f.sOut < p.CageRadius+p.CollarHalf {
		t.Errorf("the cut runs over [%.3f, %.3f] and misses part of the wall's span [%.1f, %.1f]",
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

// This is the frame's own proof: it admits the screw motion and nothing else.
// TestRibbonsClearTheSleeveOverTheTravel is the first half, that the gear
// moves freely through its travel. This is the second: the test turns a gear
// out of step with its own advance and finds the angle at which its crests and
// back corners jam in the bores' walls.
//
// A frame of round holes would report no jam at any angle, and that is the case
// this rules out.
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

// Inside a bore every point of the ribbon, teeth included, stays inside it by
// the clearance, at every phase of the travel; and the bore is no looser than
// that, but for its roof allowance. The back edge and both faces run the
// clearance from the wall at every station, and on the toothed side the
// crests come to the clearance whenever the travel brings one into the wall,
// which is what "the crests bear on the bore's toothed side" means. Between
// crests the toothed side falls away by the tooth height, and nothing bears
// there. The level bore's roof face runs the clearance and the roof allowance
// from the ribbon instead, and the floor face opposite it the clearance alone.
// Each bore is walked over the wall's span on its centre line, CollarHalf
// either side of its station.
func TestRibbonsStayInsideTheirBoresOverTheTravel(t *testing.T) {
	f := defaultSleeve()
	p := f.p

	// The least gap on each side of the bore, over every bore, phase and
	// station: the crest side, the back edge and the faces, the level bore's
	// roof face apart.
	behind, ahead := f.travelLimits()
	crest, back, face, roof := math.Inf(1), math.Inf(1), math.Inf(1), math.Inf(1)
	for _, b := range f.bores() {
		hw, vLo, vHi := boreOpening(b.g, b.sign)
		roofV := 0.0
		if b.sign == levelBore(b.g) {
			roofV = roofSide(b.g, b.sign)
		}
		station := b.sign * p.CageRadius
		for d := -behind; d <= ahead+1e-9; d += 0.05 {
			g := b.g
			g.Phase = b.g.Phase + d
			for s := station - p.CollarHalf; s <= station+p.CollarHalf+1e-9; s += 0.02 {
				uLo, vHalf := -p.Width/2, p.Thickness/2
				for i := range toothSplinePoints {
					v := -vHalf + 2*vHalf*float64(i)/float64(toothSplinePoints-1)
					crest = math.Min(crest, hw-g.edgeAt(v, s))
				}
				back = math.Min(back, hw+uLo)
				for _, side := range []float64{-1, 1} {
					gap := vHi - vHalf
					if side < 0 {
						gap = -vLo - vHalf
					}
					if side == roofV {
						roof = math.Min(roof, gap)
					} else {
						face = math.Min(face, gap)
					}
				}
			}
		}
	}
	gaps := map[string]float64{"crests": crest, "back edge": back, "faces": face}
	for name, gap := range gaps {
		if gap < p.Clearance-1e-6 {
			t.Errorf("the ribbon's %s come within %.4f mm of a bore's wall over the travel, under the "+
				"%.2f mm clearance", name, gap, p.Clearance)
		}
	}
	// And no looser: the bore is cut to the ribbon, not merely round it.
	for name, gap := range gaps {
		if gap > p.Clearance+5e-3 {
			t.Errorf("the ribbon's %s never come nearer a bore's wall than %.4f mm: the bore is cut "+
				"looser than the %.2f mm clearance", name, gap, p.Clearance)
		}
	}
	if want := p.Clearance + p.RoofAllowance; math.Abs(roof-want) > 5e-3 {
		t.Errorf("the level bores' roofs stand %.4f mm off the ribbon's face, want the clearance and the "+
			"roof allowance, %.2f mm", roof, want)
	}
	t.Logf("over the %.1f mm travel the bores hold the ribbon at %.3f mm on the crests, %.3f mm on "+
		"the back edge and %.3f mm on the faces, and the level bores' roofs stand %.3f mm off it; the "+
		"bore is %.1f by %.2f mm, %.2f mm through at the level bore",
		behind+ahead, crest, back, face, roof, 2*p.BoreHalfWidth(), 2*p.BoreHalfThickness(),
		2*p.BoreHalfThickness()+p.RoofAllowance)
}

// Nothing on the ribbon limits the travel: it is the same twisted rack from
// end to end, so any stretch of it fits a bore. What limits it is the
// ribbon's length. A gear has to keep both its bores full over the wall's
// span, and it has to keep the engaged zone covered or the teeth stop
// meshing; whichever of the two an end reaches first is the limit. This walks
// each gear out of the assembly position both ways to find both, holds Travel
// to the bores, and records what the pair comes to once gear B's assembly
// phase is counted.
func TestTravelIsTheRibbonBetweenItsBores(t *testing.T) {
	f := defaultSleeve()
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

	// Each gear's own limits, each way, from the bores and from the mesh.
	var bores, mesh [2]float64 // [0] backward, [1] forward, the tighter gear's
	for i := range bores {
		bores[i], mesh[i] = math.Inf(1), math.Inf(1)
	}
	for _, g := range f.gears {
		for i, sign := range []float64{-1, 1} {
			for _, station := range boreStations(p) {
				bores[i] = math.Min(bores[i],
					firstUncovered(g, sign, station-p.CollarHalf, station+p.CollarHalf))
			}
			mesh[i] = math.Min(mesh[i], firstUncovered(g, sign, -window, window))
		}
	}

	for i, way := range []string{"backward", "forward"} {
		if mesh[i] <= bores[i] {
			t.Errorf("%s, an end leaves the engaged zone at %.2f mm of advance, before it leaves a "+
				"bore at %.2f mm: the bores are not what limits the travel", way, mesh[i], bores[i])
		}
	}
	// A ribbon alone runs Travel through its own bores. Gear B sits a
	// fraction of a pitch along its axis, so it reaches one bore's end that
	// much sooner than gear A does, and the pair loses exactly that.
	pairTravel := bores[0] + bores[1]
	if got, want := pairTravel, p.Travel()-math.Abs(assemblyPhase); math.Abs(got-want) > 0.02 {
		t.Errorf("walking the ends finds a travel of %.2f mm; Travel less the assembly phase is %.2f",
			got, want)
	}
	if behind, ahead := f.travelLimits(); math.Abs(behind-bores[0]) > 0.011 || math.Abs(ahead-bores[1]) > 0.011 {
		t.Errorf("walking the ends finds limits of %.2f mm back and %.2f mm forward; travelLimits "+
			"derives %.2f and %.2f", bores[0], bores[1], behind, ahead)
	}
	if teeth := pairTravel / p.ToothPitch; teeth < 2 {
		t.Errorf("the travel is %.2f mm, only %.1f teeth, too short to show a gear working",
			pairTravel, teeth)
	}
	t.Logf("the pair travels %.1f mm, %.1f teeth, %.0f%% of the ribbon: %.1f mm back and %.1f mm "+
		"forward of the assembly position, where an end reaches its bore's far face; a ribbon alone "+
		"runs %.1f mm through its bores, and gear B, assembled %.2f mm along its axis, reaches one "+
		"that much sooner; an end would leave the engaged zone at %.1f mm",
		pairTravel, pairTravel/p.ToothPitch, 100*pairTravel/p.Length(), bores[0], bores[1],
		p.Travel(), math.Abs(assemblyPhase), math.Min(mesh[0], mesh[1]))
}

// Outside the engaged zone the two ribbons have to clear each other at every
// phase of the travel. The mesh proof walks the bare ribbons at the assembly
// phases; this walks everything each ribbon reaches at any phase, over every
// station the travel carries it through, against the other, and holds the
// sleeve's inner face outside the engaged zone as well.
func TestRibbonsClearEachOtherOutsideTheEngagement(t *testing.T) {
	f := defaultSleeve()
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
	if got := p.SleeveInner(); got < window {
		t.Errorf("the sleeve's inner face stands %.2f mm from the middle, inside the %.2f mm the teeth "+
			"engage over", got, window)
	}
	t.Logf("outside the engaged zone the ribbons keep %.2f mm from each other at every phase of the "+
		"travel, closest at station %.2f; the sleeve's inner face stands %.2f mm from the middle",
		-worst, worstAt, p.SleeveInner())
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
// of any outline it holds. The cube about the crossing that the engaged zone
// spans stays open as well, or the teeth would have nothing to meet in.
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
		"grown by the clearance as a box, inside a hollow of radius %.1f mm, and is open along the axis from end "+
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
// bridges there are logged for the record. Only standing on its bottom end,
// the -n end, puts the roof allowance on the faces the printer bridges
// (levelBore); stood on its top end the sleeve still prints, with the
// allowance on the floors and the bridged roofs at the bare clearance.
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
	for gi, g := range f.gears {
		for _, sign := range []float64{-1, 1} {
			hw, vLo, vHi := boreOpening(g, sign)
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
					{math.Abs(math.Cos(th)), vHi - vLo}, {math.Abs(math.Sin(th)), 2 * hw},
				} {
					if over := math.Asin(math.Min(1, w.nz)) * 180 / math.Pi; over > worst {
						worst, at, span = over, s, w.span
					}
				}
			}
			roof := "no roof allowance"
			if sign == levelBore(g) {
				roof = fmt.Sprintf("the %.2f mm roof allowance on its %+.0fv face", p.RoofAllowance, roofSide(g, sign))
			}
			t.Logf("gear %c's bore at station %+.0f has its flattest roof at station %+.2f, %.1f degrees "+
				"from upright, and bridges %.1f mm there over the wall; it carries %s",
				'A'+gi, sign*p.CageRadius, at, worst, span, roof)
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
	hw, vLo, vHi := boreOpening(b.g, b.sign)
	var out []r3.Vec
	for s := b.sign * f.sIn; math.Abs(s) <= f.sOut; s += b.sign * 0.1 {
		for i := range 17 {
			k := float64(i) / 16
			v := vLo + (vHi-vLo)*k
			for _, q := range [4][2]float64{
				{hw, v}, {-hw, v}, {-hw + 2*hw*k, vHi}, {-hw + 2*hw*k, vLo},
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

// The bore Fusion cuts is the twisted channel the model describes: a sweep of
// the clearance rectangle along the axis with a twist, which turns it rigidly
// at 1/Lambda and has no facets (spec/screwgear/fusion.md
// [SCREW-F-TWISTED-SLOT]). The proof's solid engine has no twisted sweep, so
// the compiled step proof stands in a RULED loft through a handful of rotated
// rectangles for each bore, over the cut's span [sIn, sOut]. A ruled loft is
// flat between its sections: the wall is faceted, and every facet stands a
// little inside the true channel. What that costs is clearance, because the
// facet takes its bite out of the gap the ribbon passes through.
//
// This measures that bite, so that a measurement made on the stand-in is known
// to hold for the swept channel to within it. It builds the ruled opening
// through the sections the stand-in lofts and asks how much room is left round
// the ribbon's crest rectangle. The bite does not shrink with the clearance, so
// the section count has to grow as the clearance does; this runs the rule at a
// clearance far below the default as well as above it, and at the default
// itself.
//
// What it does not measure is the body Fusion builds: the sweep's twist has
// the sense and the linearity Fusion gives it, which the diagnostic of
// 2026-09-28 measured and the build re-checks with its probes
// (spec/screwgear/fusion.md [SCREW-F-SWEEP-CHECK]).
//
// Nor is the ruled channel what decad builds. decad lofts the stand-in two
// sections at a time and walls each cell with two flat triangles, not with the
// ruled patch through its four corners, and the triangles depart from that
// patch by up to a quarter of the cell's twist vector (boreTriangleDeparture).
// This logs that departure beside the ruled figure and holds nothing to it: at
// the defaults it is about 0.32 mm on the long faces, more than the clearance,
// so the 95% this case holds is a bound on a ruled loft only. The compiled
// step proof reads the cage's volume off decad's stand-in, to 1%, and the
// build's probes, which stand clearance/2 from a short face where the
// triangles move by up to about 0.09 mm; it reads no clearance off it.
func TestSleeveBoreSubstituteKeepsItsClearance(t *testing.T) {
	for _, clearance := range []float64{0.05, 0.1, 0.2, 0.45, 0.9} {
		p := defaultParams()
		p.Clearance = clearance
		ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
		sIn, sOut := p.SleeveCut()
		turn := (sOut - sIn) / p.Lambda()
		n := boreSections(p, turn)

		worst := math.Inf(1)
		long, short := 0.0, 0.0
		for _, g := range [2]Gear{ga, gb} {
			for _, span := range [2][2]float64{{-sOut, -sIn}, {sIn, sOut}} {
				left := boreLoftClearance(g, span[0], span[1], n)
				if left < 0.95*p.Clearance {
					t.Errorf("at a clearance of %.2f mm the stand-in for the bore over [%.2f, %.2f] lofts "+
						"through %d sections and leaves the ribbon %.4f mm, under 95%% of the clearance",
						clearance, span[0], span[1], n, left)
				}
				worst = math.Min(worst, left)
				l, s := boreTriangleDeparture(g, span[0], span[1], n)
				long, short = math.Max(long, l), math.Max(short, s)
			}
		}
		t.Logf("at a clearance of %.2f mm the sleeve's cut turns %.2f degrees through %d sections %.2f "+
			"degrees apart and the stand-in leaves the ribbon %.4f mm; decad's two triangles a cell depart "+
			"from that ruled wall by up to %.3f mm on the long faces and %.3f mm on the short ones",
			clearance, turn*180/math.Pi, n, turn/float64(n-1)*180/math.Pi, worst, long, short)
	}
}

// boreTriangleDeparture is how far the walls decad builds for the stand-in
// over [lo, hi], through n sections, can depart from the ruled walls
// boreLoftClearance measures, on the opening's long faces and on its short
// ones. decad walls each cell, the stretch of one face between two
// neighbouring sections, with two flat triangles split along a diagonal. With
// a and b one face's corners on the lower section and c and d the same
// corners on the higher, the ruled patch passes through the mean of the four
// corners at the cell's middle and the diagonal through the mean of two of
// them, and those two points are |T|/4 apart, T = a - b - c + d the cell's
// twist vector.
func boreTriangleDeparture(g Gear, lo, hi float64, n int) (float64, float64) {
	hw, vLo, vHi := boreOpening(g, math.Copysign(1, lo+hi))
	corners := [4][2]float64{{-hw, vLo}, {hw, vLo}, {hw, vHi}, {-hw, vHi}}
	long, short := 0.0, 0.0
	for k := range n - 1 {
		sa := lo + (hi-lo)*float64(k)/float64(n-1)
		sb := lo + (hi-lo)*float64(k+1)/float64(n-1)
		for i := range 4 {
			j := (i + 1) % 4
			a, b := g.world(corners[i][0], corners[i][1], sa), g.world(corners[j][0], corners[j][1], sa)
			c, d := g.world(corners[i][0], corners[i][1], sb), g.world(corners[j][0], corners[j][1], sb)
			dep := a.Sub(b).Sub(c).Add(d).Len() / 4
			if i%2 == 0 {
				long = math.Max(long, dep)
			} else {
				short = math.Max(short, dep)
			}
		}
	}
	return long, short
}

// boreSections is the count the compiled proof's stand-in lofts a bore
// through; the build itself derives no count, since its sweep has no sections.
// No two neighbours are more than five degrees of twist apart, and no more
// than the angle at which the facets between them take four percent of the
// clearance. The facet at the bore's corner, which is R = hypot(W/2 + c,
// T/2 + c) from the axis, falls R*(1 - cos(step/2)) inside the true channel, so
// the second bound is step <= 2*acos(1 - 0.04*c/R). At the defaults that is
// 5.1 degrees and the five-degree bound governs; at a clearance of 0.05 mm it
// is 2.5 degrees and governs instead. The ribbon's own cell is held to two
// degrees, because there the departure is measured against the backlash.
func boreSections(p Params, turn float64) int {
	return int(math.Ceil(turn/boreStep(p))) + 1
}

// boreStep is the largest twist the stand-in allows between neighbouring bore
// sections: the smaller of five degrees and the facet bound above.
func boreStep(p Params) float64 {
	facet := 2 * math.Acos(1-0.04*p.Clearance/p.BoreCorner())
	return math.Min(5*math.Pi/180, facet)
}

// boreLoftClearance is the least room a RULED loft of the opening leaves round
// the ribbon's crest rectangle, over the whole span. A ruled loft's corners run
// straight from one section to the next, so at a station between two sections
// the opening is the four corners interpolated, and the crest rectangle has to
// sit inside that quadrilateral. The swept channel Fusion cuts has no such
// facets; see TestSleeveBoreSubstituteKeepsItsClearance for what this bounds.
func boreLoftClearance(g Gear, lo, hi float64, n int) float64 {
	p := g.P
	hw, vLo, vHi := boreOpening(g, math.Copysign(1, lo+hi))
	bw, bt := p.Width/2, p.Thickness/2
	corner := func(s float64, i int) r3.Vec {
		u, v := hw, vHi
		if i == 1 || i == 2 {
			u = -hw
		}
		if i >= 2 {
			v = vLo
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

// The build refuses an input the sleeve cannot be made from, with three
// closed forms and one sampled check: a bore's corner that reaches past the
// inner radius would start the channel in the wall; a mesh zone that reaches
// past it would be hidden; channels that come within CollarWall of the end
// faces would leave them too thin; and two neighbouring bores whose channels
// channelSeparation cannot hold CollarWall apart would leave too thin a wall
// between them. This holds that the defaults pass all four, and that each
// refusal is reached by an input that passes every check before it.
func TestSleeveInputsAreChecked(t *testing.T) {
	if why := sleeveRefusal(defaultParams()); why != "" {
		t.Fatalf("the build refuses the defaults: %s", why)
	}
	p := defaultParams()
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
		// The corner inside the inner radius, but by less than the cut's margin
		// allows: the cut would start on the far side of the middle.
		{"a cage radius 0.02 mm past the bore's corner", func(p *Params) {
			p.CageRadius = p.CollarHalf + p.BoreCorner() + 0.02
		}, refuseChannelInWall},
		{"a 1.5 mm engagement", func(p *Params) { p.Engagement = 1.5 }, refuseMeshHidden},
		{"a 16.5 mm rise", func(p *Params) { p.CageRise = 16.5 }, refuseEndWall},
		{"a 4 mm CollarWall with a 0.55 mm clearance", func(p *Params) {
			p.CollarWall, p.Clearance = 4, 0.55
			p.CageRise = leastRise(*p)
		}, refuseChannelsClose},
	}
	for _, c := range cases {
		p := defaultParams()
		c.edit(&p)
		if got := sleeveRefusal(p); got != c.want {
			t.Errorf("%s: the build says %q, want %q", c.name, got, c.want)
			continue
		}
		t.Logf("%s is refused: %s", c.name, c.want)
	}

	// The channel check is stricter than the corner alone: just inside the
	// inner radius the corner passes c < Ri, yet the cut would start before
	// the middle and the two cuts of one gear would overlap.
	m := defaultParams()
	m.CageRadius = m.CollarHalf + m.BoreCorner() + 0.02
	if sIn, _ := m.SleeveCut(); m.BoreCorner() >= m.SleeveInner() || sIn > 0 {
		t.Errorf("at a %.3f mm cage radius the corner stands %.3f mm against a %.3f mm inner radius and the cut "+
			"starts at %.3f mm, so the case does not reach the margin", m.CageRadius, m.BoreCorner(), m.SleeveInner(), sIn)
	} else {
		t.Logf("at a %.3f mm cage radius the corner passes c < Ri and the cut would start at %.3f mm", m.CageRadius, sIn)
	}

	// The mesh check is the one an older, simpler rule would miss: at a 1.5 mm
	// engagement the wall's inner face still stands outside the engaged zone,
	// which is all that rule asks, while the footprint reaches the wall.
	q := defaultParams()
	q.Engagement = 1.5
	if zone := axialWindow(q); q.SleeveInner() <= zone {
		t.Errorf("at a 1.5 mm engagement the engaged zone reaches %.2f mm, so a rule on the engaged zone "+
			"alone refuses it too and the sleeve's own check is not what is reached", zone)
	}
	// And the rise check is not over-cautious there: at 16.5 mm the channels
	// really do leave less than CollarWall at the ends. The closed form is a
	// bound, not the gap, so it also refuses the rises from about 16.81 mm up
	// to its own 18.03 mm, which the channels would allow.
	r := defaultParams()
	r.CageRise = 16.5
	ga, gb := pair(r, r.Sigma(), 0, assemblyPhase)
	topReach, _ := newSleeve(ga, gb).channelTop()
	if wall := r.CageRise - topReach; wall >= r.CollarWall {
		t.Errorf("at a 16.5 mm rise the channels leave %.3f mm of end wall, which is enough; the refusal "+
			"is stricter than the geometry", wall)
	} else {
		t.Logf("at a 16.5 mm rise the channels leave %.3f mm of end wall", wall)
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

// ---------------------------------------------------------------------------
// The windows.
//
// Down the hollow is the only way into the plain sleeve. Two slanted windows
// through the wall show the mesh from the side as well.
//
// The four bores sit round the tube at azimuths Sigma/2 and 180 - Sigma/2 on
// the +Y side and at their images under the half turn about X on the -Y side:
// 40, 140, 220 and 320 degrees at the defaults. Across +X and -X neighbouring
// bores are Sigma apart, and their channels come within 5.07 mm of each other
// across -X, which is less than a CollarWall either side of any window.
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
	c, sn := math.Cos(g.angle(s)), math.Sin(g.angle(s))
	rect := flat{n: 4}
	for i, uv := range openingCorners(g, math.Copysign(1, s)) {
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
// hypot(1, BoreCorner/Lambda), 1.42, per unit of station, and the tube's faces
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
// that stands on its grid. The corners alone held at the 0.45 mm clearance
// the defaults had until 2026-10-02, but at a 0.2 mm clearance, with the
// mounting angles and engagement of that time, they let an end's edge on the
// inner face come 2.60 mm from a far bore against a 3 mm CollarWall. An end
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
// the 4.94 mm end wall the channels leave; every edge of the hexagon is upright
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
		p := defaultParams()
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
	for _, v := range []float64{14.75, 17, 20, 25} {
		add(fmt.Sprintf("cage radius %g", v), func(p *Params) { p.CageRadius = v })
	}
	for _, v := range []float64{18.5, 25} {
		add(fmt.Sprintf("cage rise %g", v), func(p *Params) { p.CageRise = v })
	}
	for _, v := range []float64{0.45, 0.55} {
		add(fmt.Sprintf("clearance %g", v), func(p *Params) { p.Clearance = v })
	}
	for _, v := range []float64{2, 3.25} {
		add(fmt.Sprintf("collar half length %g", v), func(p *Params) { p.CollarHalf = v })
	}
	for _, v := range []float64{2, 4, 5} {
		add(fmt.Sprintf("collar wall %g", v), func(p *Params) { p.CollarWall = v })
	}
	for _, v := range []float64{70, 90, 100, 110} {
		add(fmt.Sprintf("crossing angle %g", v), func(p *Params) { p.CrossAngle = v * math.Pi / 180 })
	}
	add("mounting angles 0 and 30", func(p *Params) { p.MountAngleA, p.MountAngleB = 0, 30*math.Pi/180 })
	add("collar wall 4, clearance 0.55", func(p *Params) { p.CollarWall, p.Clearance = 4, 0.55 })
	add("collar wall 4, cage radius 14.75", func(p *Params) { p.CollarWall, p.CageRadius = 4, 14.75 })
	add("collar wall 4, crossing angle 70", func(p *Params) { p.CollarWall, p.CrossAngle = 4, 70*math.Pi/180 })
	return out
}

// quickWindowSizes are the inputs of windowSizes TestSleeveWindowsFollowTheSize
// builds unless SCREWGEAR_FULL=1. Each stands for one thing the spread holds:
//   - everything scaled by 0.75, the smallest sleeve the build accepts, for
//     the scaled inputs;
//   - a 110 degree crossing, past a right angle, where the windows face ±X
//     rather than ±Y;
//   - a 5 mm CollarWall, the thickest the spread tries on its own, so the
//     wall each window keeps from every bore is the thickest of the spread;
//   - the two inputs the build refuses for the wall between two bores, which
//     are the only refusals in the spread and so the only inputs that reach
//     channelSeparation's refusal.
var quickWindowSizes = []string{
	"everything scaled by 0.75",
	"crossing angle 110",
	"collar wall 5",
	"everything scaled by 0.667",
	"collar wall 4, clearance 0.55",
}

// quickSizes picks quickWindowSizes out of sizes, and fails the test when one
// is missing, so that renaming an input cannot quietly shrink the sample.
func quickSizes(t *testing.T, sizes []sleeveSize) []sleeveSize {
	t.Helper()
	var out []sleeveSize
	for _, name := range quickWindowSizes {
		i := slices.IndexFunc(sizes, func(s sleeveSize) bool { return s.name == name })
		if i < 0 {
			t.Fatalf("quickWindowSizes names %q, which windowSizes no longer holds", name)
		}
		out = append(out, sizes[i])
	}
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
//
// The full sample (SCREWGEAR_FULL=1) builds every input of windowSizes, 33.
// The default sample builds the five that quickWindowSizes names and leaves
// out the other 28, every one of them an input the build accepts.
func TestSleeveWindowsFollowTheSize(t *testing.T) {
	t.Parallel()
	sizes := windowSizes()
	if !fullSample(t, fmt.Sprintf("%d of the %d inputs of windowSizes", len(sizes)-len(quickWindowSizes),
		len(sizes))) {
		sizes = quickSizes(t, sizes)
	}
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

	p := defaultParams()
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
