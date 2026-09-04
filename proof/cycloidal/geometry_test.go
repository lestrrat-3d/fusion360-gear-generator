// Package cycloidal_test proves the cycloidal drive's build, step by step.
//
// Units. The generator works in Fusion-internal centimetres; this proof works in
// millimetres throughout, because both engines are millimetre-native (the sketch
// engine defaults to units.Metric() and decad's bare coordinates are millimetres
// by convention). The profile math is scale-free — disk_point is a homogeneous
// function of R, E, Rr and c — so the choice changes no shape, only the number a
// dimension carries. Every case table below is therefore written in the dialog's
// own display units, which is also how the spec states its defaults.
//
// This file holds the geometry the steps share: the rotor point function, the
// adaptive sampler, the swept-envelope contour, the undercut guard, and the
// resolve-and-check the dialog runs. Nothing here touches either engine.
package cycloidal_test

import (
	"math"
	"testing"
)

// Case-table parameter keys. Each names the dialog input it carries, in the
// dialog's display units (mm, or a bare count), plus the two the proof needs
// that the dialog does not have: the disc index a per-disc step is building.
const (
	pPinCount              = "pinCount"
	pPinCircleDiameter     = "pinCircleDiameter"
	pPinDiameter           = "pinDiameter"
	pEccentricity          = "eccentricity"
	pDiskClearance         = "diskClearance"
	pDiscThickness         = "discThickness"
	pDiscGap               = "discGap"
	pCenterBearingDiameter = "centerBearingDiameter"
	pInputShaftDiameter    = "inputShaftDiameter"
	pBearingClearance      = "bearingClearance"
	pOutputPinCircleDiam   = "outputPinCircleDiameter"
	pOutputPinCount        = "outputPinCount"
	pOutputPinDiameter     = "outputPinDiameter"
	pWall                  = "wall"
	pBaseThickness         = "baseThickness"
	pOutputPlateThickness  = "outputPlateThickness"
	pChamferSize           = "chamferSize"
	pDiscCount             = "discCount"
	pDiscIndex             = "discIndex"
)

// Fixed sampling constants the spec pins by number.
const (
	fineSteps      = 2000 // epitrochoid-trace.md "Sampling" step 1, and the curvature scan
	turnThreshold  = 5.0  // degrees, epitrochoid-trace.md "Sampling" step 2
	sweepSteps     = 240  // N_theta = N_t, epitrochoid-trace.md "Pinless ring casing"
	contourBins    = 80   // nbins
	undercutRounds = 40   // bisection iterations for E*
)

// pt is a plane-local point in millimetres.
type pt struct{ X, Y float64 }

// dims is the resolved dimension set: what _resolveDimensions stashes, for one
// disc of the stack. Every field is millimetres unless it is a count.
type dims struct {
	N  int // Pin Count
	L  int // Lobes = N - 1
	M  int // Output Pin Count
	D  int // Disc Count
	D0 int // this disc's index d

	R     float64 // pin circle radius
	E     float64 // eccentricity, unsigned
	C     float64 // disk clearance
	Rr    float64 // resolved ring-pin radius
	RrEff float64 // Rr + c
	Rv    float64 // root/valley radius = R - Rr_eff - E
	Rop   float64 // output pin circle radius

	DPin  float64 // resolved output pin diameter
	DHole float64 // output hole diameter = DPin + 2E

	T      float64 // disc thickness
	G      float64 // disc gap
	Wall   float64
	Base   float64 // base thickness
	PlateT float64 // output plate thickness
	Cham   float64 // chamfer size

	CBD float64 // centre bearing diameter (= cam outer diameter)
	ISD float64 // input shaft diameter, 0 = no bore
	Clr float64 // bearing clearance, diametral

	S   float64 // this disc's eccentric sign: +1 for d=0, -1 for d=1
	Phi float64 // this disc's clocking, d*pi
}

// derive resolves a case's raw dialog values the way _resolveDimensions does.
func derive(p map[string]float64) dims {
	d := dims{
		N:      int(math.Round(p[pPinCount])),
		M:      int(math.Round(p[pOutputPinCount])),
		D:      int(math.Round(p[pDiscCount])),
		D0:     int(math.Round(p[pDiscIndex])),
		R:      p[pPinCircleDiameter] / 2,
		E:      p[pEccentricity],
		C:      p[pDiskClearance],
		Rop:    p[pOutputPinCircleDiam] / 2,
		T:      p[pDiscThickness],
		G:      p[pDiscGap],
		Wall:   p[pWall],
		Base:   p[pBaseThickness],
		PlateT: p[pOutputPlateThickness],
		Cham:   p[pChamferSize],
		CBD:    p[pCenterBearingDiameter],
		ISD:    p[pInputShaftDiameter],
		Clr:    p[pBearingClearance],
	}
	d.L = d.N - 1
	if d.D == 0 {
		d.D = 1
	}
	// Pin / hole sizes: 0 means auto-derive to the mean of the theoretical
	// bounds, any non-zero value is the user's explicit override.
	if pd := p[pPinDiameter]; pd > 0 {
		d.Rr = pd / 2
	} else {
		d.Rr = 0.5 * (d.E + d.R*math.Sin(math.Pi/float64(d.N)))
	}
	d.RrEff = d.Rr + d.C
	d.Rv = d.R - d.RrEff - d.E
	if od := p[pOutputPinDiameter]; od > 0 {
		d.DPin = od
	} else {
		d.DPin = d.Rop*math.Sin(math.Pi/float64(d.M)) - d.E
	}
	d.DHole = d.DPin + 2*d.E
	d.S = 1
	if d.D0 == 1 {
		d.S = -1
	}
	d.Phi = float64(d.D0) * math.Pi
	return d
}

// centre is this disc's centre Od_d = O + s_d*E*Xhat.
func (d dims) centre() pt { return pt{X: d.S * d.E, Y: 0} }

// stackTop is the top of the disc stack: (D-1)*(T+G) + T.
func (d dims) stackTop() float64 { return float64(d.D-1)*(d.T+d.G) + d.T }

// zBase is this disc's own plane height, z_d = d*(T+G).
func (d dims) zBase() float64 { return float64(d.D0) * (d.T + d.G) }

// outerWall is the casing's outer radius: the contour peak plus Wall.
func (d dims) outerWall() float64 { return d.R - d.Rr + 2*d.E + d.Wall }

// innerFloor is the housing base annulus's inner radius: Wall inside the
// contour valley R - Rr.
func (d dims) innerFloor() float64 { return d.R - d.Rr - d.Wall }

// plateRadius is OutputPlateDiameter/2 = Rop + DPin/2 + Wall.
func (d dims) plateRadius() float64 { return d.Rop + d.DPin/2 + d.Wall }

// diskPoint is the rotor profile point function, reproduced exactly from
// epitrochoid-trace.md "The point function", for a rotor centred at (cx, cy)
// with clocking phi.
func (d dims) diskPoint(t, cx, cy, phi float64) pt {
	n := float64(d.N)
	num := math.Sin((1 - n) * t)
	den := d.R/(d.E*n) - math.Cos((1-n)*t)
	psi := math.Atan2(num, den)
	x0 := d.R*math.Cos(t) - d.RrEff*math.Cos(t+psi) - d.E*math.Cos(n*t)
	y0 := -d.R*math.Sin(t) + d.RrEff*math.Sin(t+psi) + d.E*math.Sin(n*t)
	return pt{
		X: cx + x0*math.Cos(phi) - y0*math.Sin(phi),
		Y: cy + x0*math.Sin(phi) + y0*math.Cos(phi),
	}
}

// lobePoint is diskPoint on this disc's own centre and clocking.
func (d dims) lobePoint(t float64) pt {
	c := d.centre()
	return d.diskPoint(t, c.X, c.Y, d.Phi)
}

// lobeSamples returns the adaptively sampled fit points of one lobe,
// t in [0, 2*pi/L], per epitrochoid-trace.md "Sampling": a 2000-step fine
// trace, greedily kept whenever the accumulated turn angle reaches 5.0 degrees,
// first and last always kept. Uniform sampling overshoots into rabbit-ear loops
// near the undercut limit, which is why this is not a uniform trace.
func (d dims) lobeSamples() []pt {
	params := d.lobeParams()
	out := make([]pt, len(params))
	for i, t := range params {
		out[i] = d.lobePoint(t)
	}
	return out
}

// lobeParams is the sampler proper: the curve parameters the adaptive rule
// keeps, first and last included, so a caller that repeats the lobe around the
// disc can re-evaluate at the same parameters shifted by whole lobe pitches.
func (d dims) lobeParams() []float64 {
	span := 2 * math.Pi / float64(d.L)
	fine := make([]pt, fineSteps+1)
	for i := range fine {
		fine[i] = d.lobePoint(span * float64(i) / float64(fineSteps))
	}
	kept := []float64{0}
	acc := 0.0
	for i := 1; i < fineSteps; i++ {
		ax, ay := fine[i].X-fine[i-1].X, fine[i].Y-fine[i-1].Y
		bx, by := fine[i+1].X-fine[i].X, fine[i+1].Y-fine[i].Y
		acc += math.Abs(math.Atan2(ax*by-ay*bx, ax*bx+ay*by)) * 180 / math.Pi
		if acc >= turnThreshold {
			kept = append(kept, span*float64(i)/float64(fineSteps))
			acc = 0
		}
	}
	return append(kept, span)
}

// lobeOutline returns the whole rotor outline: the L lobes of one disc as one
// closed chain of sampled points, each lobe the previous one advanced by a lobe
// pitch. The chain's last point is the one before the start point returns, so
// the caller closes the loop itself.
//
// The base curve is L-fold symmetric, P(t + 2*pi/L) = Rot(-2*pi/L)*P(t), so
// repeating the lobe is exactly what the L-times circular pattern of the lobe
// sector about the disc axis produces in Fusion.
func (d dims) lobeOutline() []pt {
	params := d.lobeParams()
	span := 2 * math.Pi / float64(d.L)
	out := make([]pt, 0, len(params)*d.L)
	for lobe := range d.L {
		for i, t := range params {
			if i == len(params)-1 {
				continue // the next lobe's first point is this one's last
			}
			out = append(out, d.lobePoint(t+span*float64(lobe)))
		}
	}
	return out
}

// tipRadius is the lobe tip's radius about the disc centre: R - Rr_eff + E.
// The lobe's furthest reach. The casing's outer wall is sized from it, one
// eccentricity further out, so the proof measures it rather than assuming it.
func (d dims) tipRadius() float64 { return d.R - d.RrEff + d.E }

// tracedTipRadius is the same reach measured on the fine trace the sampler runs
// over, which is the profile itself rather than the polyline through the kept
// points.
func (d dims) tracedTipRadius() float64 {
	span := 2 * math.Pi / float64(d.L)
	c := d.centre()
	best := 0.0
	for i := 0; i <= fineSteps; i++ {
		best = math.Max(best, radiusOf(d.lobePoint(span*float64(i)/float64(fineSteps)), c))
	}
	return best
}

// outputHoleCentres are the M output-hole centres of disc d: the seed hole on
// the +X ray from the disc centre, then one per M-th of a turn about the disc
// axis, which is what the M-times circular pattern produces.
func (d dims) outputHoleCentres() []pt {
	c := d.centre()
	out := make([]pt, 0, d.M)
	for k := range d.M {
		a := 2 * math.Pi * float64(k) / float64(d.M)
		out = append(out, pt{X: c.X + d.Rop*math.Cos(a), Y: c.Y + d.Rop*math.Sin(a)})
	}
	return out
}

// outputPinCentres are the M output-pin centres, on the drive axis O rather than
// the disc centre: the seed pin on the +X ray from O, then one per M-th of a
// turn, which is what the M-times pattern about the drive axis produces.
func (d dims) outputPinCentres() []pt {
	out := make([]pt, 0, d.M)
	for k := range d.M {
		a := 2 * math.Pi * float64(k) / float64(d.M)
		out = append(out, pt{X: d.Rop * math.Cos(a), Y: d.Rop * math.Sin(a)})
	}
	return out
}

// contour returns one pin-pitch of the ring casing's inner wall,
// contour(phi) = env(phi) + c, emitted at bin EDGES so the first point lands
// exactly on -pi/N and the last exactly on +pi/N.
//
// Bin centres would inset both ends by half a bin, leaving an angular gap
// between every pair of patterned sectors, so the N sectors would not touch and
// would not Join into one casing.
func (d dims) contour() []pt {
	n := float64(d.N)
	half := math.Pi / n
	binMax := make([]float64, contourBins)
	hit := make([]bool, contourBins)
	for i := range sweepSteps {
		theta := 2 * math.Pi * float64(i) / float64(sweepSteps)
		cx, cy := d.E*math.Cos(theta), d.E*math.Sin(theta)
		phi := -theta / float64(d.L)
		for j := range sweepSteps {
			t := 2 * math.Pi * float64(j) / float64(sweepSteps)
			p := d.diskPoint(t, cx, cy, phi)
			a := math.Atan2(p.Y, p.X)
			if a < -half || a > half {
				continue
			}
			b := int((a + half) / (2 * half) * float64(contourBins))
			b = min(max(b, 0), contourBins-1)
			r := math.Hypot(p.X, p.Y)
			if r > binMax[b] {
				binMax[b] = r
			}
			hit[b] = true
		}
	}
	out := make([]pt, 0, contourBins+1)
	for i := 0; i <= contourBins; i++ {
		phi := -half + 2*half*float64(i)/float64(contourBins)
		peak := 0.0
		if i-1 >= 0 && hit[i-1] {
			peak = max(peak, binMax[i-1])
		}
		if i < contourBins && hit[i] {
			peak = max(peak, binMax[i])
		}
		r := d.C + peak
		out = append(out, pt{X: r * math.Cos(phi), Y: r * math.Sin(phi)})
	}
	return out
}

// contourRing returns the casing's whole inner wall: the pin-pitch contour
// repeated N times about O. Adjacent copies share their end point, so the ring
// is one closed chain — which is exactly what the N-times pattern plus Join
// produces when the contour's ends sit at exactly +/-pi/N.
func (d dims) contourRing() []pt {
	one := d.contour()
	out := make([]pt, 0, len(one)*d.N)
	for k := range d.N {
		a := 2 * math.Pi * float64(k) / float64(d.N)
		sa, ca := math.Sin(a), math.Cos(a)
		for i, p := range one {
			if i == len(one)-1 {
				continue // shared with the next sector's first point
			}
			out = append(out, pt{X: p.X*ca - p.Y*sa, Y: p.X*sa + p.Y*ca})
		}
	}
	return out
}

// rhoMinO is the smallest radius of curvature of the base trochoid at the
// points whose centre of curvature lies toward O — the convex flanks an inward
// offset can overrun. epitrochoid-trace.md "No-undercut guard".
func (d dims) rhoMinO() float64 {
	n := float64(d.N)
	best := math.Inf(1)
	for i := range fineSteps {
		t := 2 * math.Pi * float64(i) / float64(fineSteps)
		bx := d.R*math.Cos(t) - d.E*math.Cos(n*t)
		by := -d.R*math.Sin(t) + d.E*math.Sin(n*t)
		xp := -d.R*math.Sin(t) + d.E*n*math.Sin(n*t)
		yp := -d.R*math.Cos(t) + d.E*n*math.Cos(n*t)
		xpp := -d.R*math.Cos(t) + d.E*n*n*math.Cos(n*t)
		ypp := d.R*math.Sin(t) - d.E*n*n*math.Sin(n*t)
		k := xp*ypp - yp*xpp
		if math.Abs(k) < 1e-12 {
			continue
		}
		s2 := xp*xp + yp*yp
		rho := math.Pow(s2, 1.5) / k
		s := math.Sqrt(s2)
		cx := bx + rho*(-yp/s)
		cy := by + rho*(xp/s)
		if cx*cx+cy*cy < bx*bx+by*by {
			best = math.Min(best, math.Abs(rho))
		}
	}
	return best
}

// undercutCeiling is the binding eccentricity limit itself: the largest E for
// which the inward equidistant still clears the base trochoid's curvature,
// searched over the whole range the loose cusp limit R/N allows. The spec calls
// this the binding constraint and R/N the loose one, so this must come out
// below R/N.
func undercutCeiling(p map[string]float64) float64 {
	q := make(map[string]float64, len(p))
	for k, v := range p {
		q[k] = v
	}
	d := derive(q)
	q[pEccentricity] = d.R / float64(d.N)
	return undercutLimit(q)
}

// undercutLimit is E*, the largest eccentricity in (0, E] for which the inward
// equidistant still clears the base trochoid's curvature, by exactly 40
// bisection rounds. It returns 0 when no positive E' satisfies the guard.
func undercutLimit(p map[string]float64) float64 {
	trial := func(e float64) bool {
		q := make(map[string]float64, len(p))
		for k, v := range p {
			q[k] = v
		}
		q[pEccentricity] = e
		t := derive(q)
		return t.RrEff < t.rhoMinO()
	}
	hi := p[pEccentricity]
	lo := 1e-9
	if !trial(lo) {
		return 0
	}
	for range undercutRounds {
		mid := (lo + hi) / 2
		if trial(mid) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

// problems is evaluate_problems: the whole authoritative validity list, run on
// the resolved values, in the order the spec's table gives. An empty result
// means the case is inside the regime the design must hold across.
func (d dims) problems() []string {
	var out []string
	n, m := float64(d.N), float64(d.M)
	if d.D == 2 && (d.N%2 != 0 || d.M%2 != 0) {
		out = append(out, "two discs require an even Pin Count and an even Output Pin Count")
	}
	if !(d.E < d.Rr && d.Rr < d.R*math.Sin(math.Pi/n)) {
		out = append(out, "pin geometry out of range: E < Rr < R*sin(pi/N) fails")
	}
	if d.DPin <= 0 {
		out = append(out, "output pins vanish: resolved diameter <= 0")
	}
	if !(d.DHole < 2*d.Rop*math.Sin(math.Pi/m)) {
		out = append(out, "output holes overlap")
	}
	if !(d.E < d.R/n) {
		out = append(out, "eccentricity too large: E >= R/N")
	}
	if !(d.Rop < d.Rv) {
		out = append(out, "output pin circle too large: Rop >= Rv")
	}
	if !(d.RrEff < d.rhoMinO()) {
		out = append(out, "eccentricity too large: the rotor profile undercuts")
	}
	if !(d.ISD < d.CBD) {
		out = append(out, "input shaft diameter must be less than centre bearing diameter")
	}
	if !(d.E+d.ISD/2 < d.CBD/2) {
		out = append(out, "input bore does not fit inside the cam")
	}
	if !((d.CBD+d.Clr)/2 < d.Rop-d.DHole/2) {
		out = append(out, "disk centre bore overlaps the output holes")
	}
	return out
}

// requireInRegime fails the case when its parameters fall outside the validity
// table. The dialog rejects such a case before any geometry is built, so a
// proof case that violates it would be proving a build the generator refuses.
func requireInRegime(t testing.TB, d dims) {
	t.Helper()
	if problems := d.problems(); len(problems) > 0 {
		t.Fatalf("case is outside the validity table, which the dialog rejects: %v", problems)
	}
}

// polygonArea is the signed area of a closed polygon, positive counter-clockwise.
func polygonArea(points []pt) float64 {
	sum := 0.0
	for i := range points {
		j := (i + 1) % len(points)
		sum += points[i].X*points[j].Y - points[j].X*points[i].Y
	}
	return sum / 2
}

// radiusOf is the distance from c to p.
func radiusOf(p, c pt) float64 { return math.Hypot(p.X-c.X, p.Y-c.Y) }

// nearly reports whether a and b agree to within tol.
func nearly(a, b, tol float64) bool { return math.Abs(a-b) <= tol }
