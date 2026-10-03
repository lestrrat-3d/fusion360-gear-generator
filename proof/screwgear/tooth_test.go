package screwgear_test

import (
	"math"
	"testing"
)

// ---------------------------------------------------------------------------
// The leaned tooth as the build draws it.
//
// Since 2026-10-03 the toothed edge leans across the thickness (ToothSlant) and
// is lowered toward the faces (ToothBow), so a section's toothed side is no
// longer a straight line between two corners. The build draws it as a fitted
// spline through toothSplinePoints points of the edge at the section's station
// (spec §2), and these cases hold what that costs and what the build can check
// about it. What Fusion's loft does between the splines is not reachable here
// (spec "What the proof cannot reach").
// ---------------------------------------------------------------------------

// naturalSpline is the natural cubic spline through the points (t[i], y[i]),
// t increasing: zero second derivative at both ends. It returns the spline as
// a function of t.
func naturalSpline(t, y []float64) func(float64) float64 {
	n := len(t)
	m := make([]float64, n) // second derivatives at the knots
	c, d := make([]float64, n), make([]float64, n)
	for i := 1; i < n-1; i++ {
		h0, h1 := t[i]-t[i-1], t[i+1]-t[i]
		a, b := h0/6, (h0+h1)/3
		r := (y[i+1]-y[i])/h1 - (y[i]-y[i-1])/h0
		w := b - a*c[i-1]
		c[i] = (h1 / 6) / w
		d[i] = (r - a*d[i-1]) / w
	}
	for i := n - 2; i >= 1; i-- {
		m[i] = d[i] - c[i]*m[i+1]
	}
	return func(x float64) float64 {
		i := 0
		for i < n-2 && x > t[i+1] {
			i++
		}
		h := t[i+1] - t[i]
		a, b := (t[i+1]-x)/h, (x-t[i])/h
		return a*y[i] + b*y[i+1] + ((a*a*a-a)*m[i]+(b*b*b-b)*m[i+1])*h*h/6
	}
}

// splineDeparture is how far, along u, a natural cubic spline through n points
// of the toothed edge at station s, evenly across the thickness, ends
// included, and parametrised by chord length as a fitted spline is, departs
// from the edge between them.
func splineDeparture(g Gear, s float64, n int) float64 {
	t := g.P.Thickness / 2
	us, vs, ks := make([]float64, n), make([]float64, n), make([]float64, n)
	for i := range n {
		vs[i] = -t + 2*t*float64(i)/float64(n-1)
		us[i] = g.edgeAt(vs[i], s)
		if i > 0 {
			ks[i] = ks[i-1] + math.Hypot(us[i]-us[i-1], vs[i]-vs[i-1])
		}
	}
	u, v := naturalSpline(ks, us), naturalSpline(ks, vs)
	worst := 0.0
	for j := 0; j <= 400; j++ {
		k := ks[n-1] * float64(j) / 400
		worst = math.Max(worst, math.Abs(u(k)-g.edgeAt(v(k), s)))
	}
	return worst
}

// maxSplineDeparture is the most the toothed side's spline may depart from the
// edge, along u.
const maxSplineDeparture = 0.02

// The spec draws each section's toothed side as a fitted spline through eleven
// points of the edge, 0.375 mm apart across the 3.75 mm thickness at the
// defaults. Across the thickness the edge runs through tan(ToothSlant)*T of
// cosine phase, 0.69 of a pitch at the defaults, so the count has to resolve
// most of a wave. This fits a natural cubic spline through the points at
// every station of a pitch, the end condition that departs most at the ends
// of the usual ones, and holds it within 0.02 mm of the edge along u: under
// the 0.05 mm TestTeethTouchAlongALine holds the contact to, and 2% of the
// backlash. Nine points depart by 0.030 mm, which is why the count is eleven;
// other counts are logged beside it.
//
// Fusion's fitted spline is its own fit, with end conditions the API
// reference does not state, so this bounds a spline Fusion may not draw
// exactly; it is the figure the count is bought with.
func TestToothSplineHoldsTheEdge(t *testing.T) {
	g, _ := defaultPair()
	p := g.P
	at := func(n int) (float64, float64) {
		worst, worstAt := 0.0, 0.0
		for i := range 105 {
			s := p.ToothPitch * float64(i) / 105
			if d := splineDeparture(g, s, n); d > worst {
				worst, worstAt = d, s
			}
		}
		return worst, worstAt
	}
	for _, n := range []int{5, 7, 9, 13} {
		d, s := at(n)
		t.Logf("a spline through %d points departs from the edge by up to %.4f mm, at station %.3f", n, d, s)
	}
	d, s := at(toothSplinePoints)
	if d > maxSplineDeparture {
		t.Errorf("a spline through %d points departs from the edge by %.4f mm at station %.3f, past %.2f mm",
			toothSplinePoints, d, s, maxSplineDeparture)
	}
	t.Logf("the build's spline through %d points, %.3f mm apart across the thickness, departs from the edge "+
		"by up to %.4f mm, at station %.3f; the edge runs through %.2f pitches of phase across the thickness",
		toothSplinePoints, p.Thickness/float64(toothSplinePoints-1), d, s, g.Slant*p.Thickness/p.ToothPitch)
}

// ToothBow is what makes a leaned ridge run straight in the world. A ridge is
// a line of constant s + Slant*v in the section's chart, and the twist turns
// its two ends by the twist across the stretch of axis it covers, so without
// the bow the crest ridge bows out from its own chord. This walks the crest
// ridge through the crossing and holds it within 0.01 mm of its chord, against
// 0.17 mm with no bow. It also holds the ridge's direction at the crossing
// within 3 degrees of the bisector of the obtuse angle between the two axes,
// 50 degrees from the ribbon's own axis at the 80 degree crossing, which is
// what the slant was chosen for: it stands 48.2 degrees there, and where the
// teeth actually touch, a millimetre or two along the axes, the two ribbons'
// ridges lie within 0.3 degrees of each other (TestTeethTouchAlongALine).
func TestToothRidgesRunStraight(t *testing.T) {
	ga, _ := defaultPair()
	p := ga.P
	th := p.Thickness / 2
	bowOff := func(g Gear) float64 {
		a, b := g.surf(-th, g.Slant*th), g.surf(th, -g.Slant*th)
		chord, _ := b.Sub(a).Normalize()
		worst := 0.0
		for i := 0; i <= 40; i++ {
			v := -th + 2*th*float64(i)/40
			d := g.surf(v, -g.Slant*v).Sub(a)
			worst = math.Max(worst, d.Sub(chord.Scale(d.Dot(chord))).Len())
		}
		return worst
	}
	straight := ga
	straight.Bow = 0
	if got := bowOff(ga); got > 0.01 {
		t.Errorf("the crest ridge bows %.4f mm off its chord, past 0.01 mm", got)
	} else {
		t.Logf("the crest ridge stands %.4f mm off its chord at most, against %.4f mm with no bow", got, bowOff(straight))
	}
	want := 90 - p.Sigma()*180/math.Pi/2
	got := math.Acos(math.Abs(ga.ridgeDir(0, 0).Dot(ga.Ez))) * 180 / math.Pi
	if math.Abs(got-want) > 3 {
		t.Errorf("the crest ridge at the crossing stands %.2f degrees from the axis, want %.0f", got, want)
	}
	t.Logf("the crest ridge at the crossing stands %.2f degrees from the ribbon's axis, against the %.0f of the "+
		"obtuse bisector", got, want)
}

// toothProbe is one of the build's handedness probes: a point in the
// section's (u, v, s) chart, and whether the cell has to contain it.
type toothProbe struct {
	u, v, s float64
	inside  bool
}

// The build's probes stand probeDepth inside the ribbon's crest and face.
const probeDepth = 0.25

// handProbes are the probes of spec §2's handedness check on gear g's cell: at
// each face, probeDepth in from it, one on the crest ridge through the cell's
// first crest at or past half a pitch from its start, and one where that crest
// would be under the opposite slant, probeDepth under the crest's height.
func handProbes(g Gear) []toothProbe {
	p := g.P
	s0 := g.Phase - p.Length()/2
	sc := g.Phase + math.Ceil((s0+p.ToothPitch/2-g.Phase)/p.ToothPitch)*p.ToothPitch
	var out []toothProbe
	for _, side := range []float64{-1, 1} {
		v := side * (p.Thickness/2 - probeDepth)
		u := p.Width/2 - p.ToothBow*v*v - probeDepth
		out = append(out,
			toothProbe{u, v, sc - g.Slant*v, true},
			toothProbe{u, v, sc + g.Slant*v, false})
	}
	return out
}

// probeMargin is how far inside the body a probe lies along u: positive
// inside.
func probeMargin(g Gear, q toothProbe) float64 { return g.edgeAt(q.v, q.s) - q.u }

// The slant's sign has to follow the (u, v, s) chart of the spec's §1, or the
// ridges lean the wrong way and the two ribbons' ridges cross at about 100
// degrees where they meet instead of lying along each other. Nothing in the
// sketches shows which way a ridge leans, so the build probes the cell after
// its loft (spec §2): at each face, a point on the crest ridge has to be inside
// and the point where the crest would be under the opposite sign outside.
// This holds that the probes tell the two signs apart by more than 0.1 mm
// each way at the defaults, so a wrong sign fails loudly, and that the probes
// lie in the cell. It also measures what the wrong sign does to the mesh's
// ridges: their angle where the two ribbons cross.
func TestToothSlantProbesTellTheHand(t *testing.T) {
	ga, gb := defaultPair()
	for _, g := range []Gear{ga, gb} {
		wrong := g
		wrong.Slant = -g.Slant
		s0 := g.Phase - g.P.Length()/2
		for _, q := range handProbes(g) {
			right, flipped := probeMargin(g, q), probeMargin(wrong, q)
			if (right > 0) != q.inside || math.Abs(right) < 0.1 {
				t.Errorf("the probe at u=%.3f v=%+.3f s=%.3f reads %+.3f mm under the right slant, want %s by 0.1 mm",
					q.u, q.v, q.s, right, map[bool]string{true: "inside", false: "outside"}[q.inside])
			}
			if (flipped > 0) == q.inside || math.Abs(flipped) < 0.1 {
				t.Errorf("the probe at u=%.3f v=%+.3f s=%.3f reads %+.3f mm under the wrong slant, so it cannot "+
					"tell the two apart", q.u, q.v, q.s, flipped)
			}
			if q.s < s0 || q.s > s0+float64(cellTeeth)*g.P.ToothPitch {
				t.Errorf("the probe at station %.3f is outside the cell, %.3f to %.3f", q.s, s0,
					s0+float64(cellTeeth)*g.P.ToothPitch)
			}
			if g.Phase == 0 {
				t.Logf("probe at u=%.3f v=%+.3f, %.3f mm from the cell's start: %+.3f mm inside under the right "+
					"slant, %+.3f under the wrong one", q.u, q.v, q.s-s0, right, flipped)
			}
		}
	}
	wa, wb := ga, gb
	wa.Slant, wb.Slant = -ga.Slant, -gb.Slant
	right := math.Acos(math.Abs(ga.ridgeDir(0, 0).Dot(gb.ridgeDir(0, 0)))) * 180 / math.Pi
	wrong := math.Acos(math.Abs(wa.ridgeDir(0, 0).Dot(wb.ridgeDir(0, 0)))) * 180 / math.Pi
	t.Logf("at the crossing the two crest ridges stand %.1f degrees apart, and %.1f with the slant's sign "+
		"flipped on both", right, wrong)
	if wrong < 60 {
		t.Errorf("the flipped slant puts the ridges only %.1f degrees apart", wrong)
	}
}
