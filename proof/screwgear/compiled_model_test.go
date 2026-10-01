package screwgear_test

// The compiled step proof's shared model: the dialog inputs, the numbers the
// build derives from them in processInputs, the mechanism frame of the spec's
// §1, and the section and schedule arithmetic the build steps share. Every
// name here carries the `cp` prefix so it cannot collide with the hand-written
// mechanism proof that shares this package.

import (
	"maps"
	"math"

	"github.com/lestrrat-3d/r3"
)

// cpDefaults is the spec's dialog table ("Variables"), in millimetres and
// degrees. toothCount is a count.
var cpDefaults = map[string]float64{
	"ribbonWidth":     15,
	"toothCount":      68,
	"twistLead":       49.5,
	"ribbonThickness": 3.75,
	"toothPitch":      2.625,
	"toothHeight":     2.625,
	"cageRadius":      15,
	"cageRise":        18.75,
	"clearance":       0.45,
	"collarHalf":      3,
	"collarWall":      3,
	"crossAngle":      80,
	"engagement":      0.75,
	"mountAngleA":     15,
	"mountAngleB":     15,
	"assemblyPhase":   -1.31,
}

// cpCellTeeth is the module constant CELL_TEETH.
const cpCellTeeth = 4

// cpWith returns the defaults with the given overrides applied.
func cpWith(over map[string]float64) map[string]float64 {
	p := maps.Clone(cpDefaults)
	maps.Copy(p, over)
	return p
}

// cpModel is everything processInputs derives, in millimetres and radians.
type cpModel struct {
	W, T, P, H     float64
	N              int
	Lead, Lam      float64
	Sigma, Eng, A  float64
	PhiA, PhiB     float64
	Phase          float64
	CageRadius     float64
	Rise           float64
	Clearance      float64
	CollarHalf     float64
	CollarWall     float64
	Ri, Ro, Hw, Ht float64
	Corner         float64 // c, the bore's corner radius
	SIn, SOut      float64
	Cell           int // teeth in the lofted cell, min(cellTeeth, N)
	Steps          int // n, sections per tooth
	Q, R           int // N = Q*Cell + R
}

func cpModelOf(p map[string]float64) cpModel {
	m := cpModel{
		W:          p["ribbonWidth"],
		T:          p["ribbonThickness"],
		P:          p["toothPitch"],
		H:          p["toothHeight"],
		N:          int(math.Round(p["toothCount"])),
		Lead:       p["twistLead"],
		Sigma:      p["crossAngle"] * math.Pi / 180,
		Eng:        p["engagement"],
		PhiA:       p["mountAngleA"] * math.Pi / 180,
		PhiB:       p["mountAngleB"] * math.Pi / 180,
		Phase:      p["assemblyPhase"],
		CageRadius: p["cageRadius"],
		Rise:       p["cageRise"],
		Clearance:  p["clearance"],
		CollarHalf: p["collarHalf"],
		CollarWall: p["collarWall"],
	}
	m.Lam = m.Lead / (2 * math.Pi)
	m.A = m.W - m.Eng
	m.Ri = m.CageRadius - m.CollarHalf
	m.Ro = m.CageRadius + m.CollarHalf
	m.Hw = m.W/2 + m.Clearance
	m.Ht = m.T/2 + m.Clearance
	m.Corner = math.Hypot(m.Hw, m.Ht)
	m.SIn = math.Sqrt(m.Ri*m.Ri-m.Corner*m.Corner) - 1
	m.SOut = m.Ro + 1
	m.Cell = min(cpCellTeeth, m.N)
	twist := math.Ceil((m.P / m.Lam) / (2 * math.Pi / 180))
	m.Steps = max(int(twist), 8)
	m.Q = m.N / m.Cell
	m.R = m.N % m.Cell
	return m
}

// cpGear is one gear's frame of §1: its axis origin and direction, its
// unrotated section axes û and v̂, its mounting angle and its tooth phase.
type cpGear struct {
	Label     string
	Origin    r3.Vec
	Dir, U, V r3.Vec
	Phi       float64
	Z0        float64
}

// gear returns gear 0 (A) or 1 (B) in the proof's world: C at the origin,
// ê on X, k̂ = n̂ × ê on Y, n̂ on Z.
func (m cpModel) gear(g int) cpGear {
	n := r3.NewVec(0, 0, 1)
	if g == 0 {
		dir := r3.NewVec(math.Cos(m.Sigma/2), math.Sin(m.Sigma/2), 0)
		return cpGear{Label: "Gear A", Origin: n.Scale(-m.A / 2), Dir: dir, U: n, V: dir.Cross(n), Phi: m.PhiA, Z0: 0}
	}
	dir := r3.NewVec(math.Cos(m.Sigma/2), -math.Sin(m.Sigma/2), 0)
	u := n.Scale(-1)
	return cpGear{Label: "Gear B", Origin: n.Scale(m.A / 2), Dir: dir, U: u, V: dir.Cross(u), Phi: m.PhiB, Z0: m.Phase}
}

// theta is the section angle at station s, s/Lambda + Phi.
func (m cpModel) theta(g cpGear, s float64) float64 { return s/m.Lam + g.Phi }

// rot turns the section coordinates (u, v) by theta.
func cpRot(u, v, theta float64) (float64, float64) {
	c, s := math.Cos(theta), math.Sin(theta)
	return u*c - v*s, u*s + v*c
}

// world is §1's point of gear g at station s with section coordinates (u, v).
func (m cpModel) world(g cpGear, s, u, v float64) r3.Vec {
	x, y := cpRot(u, v, m.theta(g, s))
	return g.Origin.Add(g.Dir.Scale(s)).Add(g.U.Scale(x)).Add(g.V.Scale(y))
}

// utooth is the toothed edge Utooth(s) for the gear's tooth phase z0.
func (m cpModel) utooth(s, z0 float64) float64 {
	return m.W/2 - m.H/2 + (m.H/2)*math.Cos(2*math.Pi*(s-z0)/m.P)
}

// ribbonStart is s0 = Z0 - L/2, the station of the ribbon's negative end.
func (m cpModel) ribbonStart(g cpGear) float64 { return g.Z0 - float64(m.N)*m.P/2 }

// cpQuad is one rectangle section: its station and its four corners, in the
// order the step list draws them: (uB, -hv), (uF, -hv), (uF, hv), (uB, hv),
// each already turned by the station's angle into section coordinates
// (x along û, y along v̂).
type cpQuad struct {
	S       float64
	Corners [4][2]float64
}

func (m cpModel) quad(g cpGear, s, uB, uF, hv float64) cpQuad {
	th := m.theta(g, s)
	q := cpQuad{S: s}
	for i, c := range [4][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}} {
		x, y := cpRot(c[0], c[1], th)
		q.Corners[i] = [2]float64{x, y}
	}
	return q
}

// ribbonSections is the section list of the teeth [from, from+teeth) of gear
// g's ribbon, counted from its negative end: teeth*n + 1 rectangles of the
// tooth profile, the cell recipe of §2.
func (m cpModel) ribbonSections(g cpGear, from, teeth int) []cpQuad {
	s0 := m.ribbonStart(g) + float64(from)*m.P
	out := make([]cpQuad, 0, teeth*m.Steps+1)
	for k := 0; k <= teeth*m.Steps; k++ {
		s := s0 + float64(k)*m.P/float64(m.Steps)
		out = append(out, m.quad(g, s, -m.W/2, m.utooth(s, g.Z0), m.T/2))
	}
	return out
}

// boreSpan is the station span a bore's cut covers on its gear's axis:
// [sIn, sOut] for a +R bore and [-sOut, -sIn] for a -R bore.
func (m cpModel) boreSpan(sign int) (float64, float64) {
	if sign > 0 {
		return m.SIn, m.SOut
	}
	return -m.SOut, -m.SIn
}

// boreSectionCount is the stand-in's section count of "What the proof's
// stand-in costs": no two neighbours more than 5° apart, nor more than the
// facet angle 2*acos(1 - 0.04*clearance/c).
func (m cpModel) boreSectionCount() int {
	turn := (m.SOut - m.SIn) / m.Lam
	step := min(5*math.Pi/180, 2*math.Acos(1-0.04*m.Clearance/m.Corner))
	return int(math.Ceil(turn/step)) + 1
}

// cpJoin is one join of the doubling schedule of §3: the body holds the
// cells [0, Body) and the piece placed against it holds Piece cells, moved
// there by Step(Body*cell) from the ribbon's first cells.
type cpJoin struct{ Body, Piece int }

// cpSchedule runs §3's doubling schedule for q cells and returns its joins in
// order, plus the asides taken, in the order they were taken.
func cpSchedule(q int) ([]cpJoin, []int) {
	var joins []cpJoin
	var asides []int
	if q < 1 {
		return nil, nil
	}
	top := 0
	for (q >> (top + 1)) > 0 {
		top++
	}
	m := 1
	for bit := range top {
		if q&(1<<bit) != 0 {
			asides = append(asides, m)
		}
		joins = append(joins, cpJoin{Body: m, Piece: m})
		m *= 2
	}
	for i := len(asides) - 1; i >= 0; i-- {
		joins = append(joins, cpJoin{Body: m, Piece: asides[i]})
		m += asides[i]
	}
	return joins, asides
}

// cpSimpson is the volume of the ruled solid between two corresponding
// quadrilaterals on parallel planes h apart: the section area is quadratic
// along the slab, so Simpson's rule is exact for it.
func cpSimpson(a, b cpQuad) float64 {
	var mid [4][2]float64
	for i := range 4 {
		mid[i] = [2]float64{(a.Corners[i][0] + b.Corners[i][0]) / 2, (a.Corners[i][1] + b.Corners[i][1]) / 2}
	}
	h := math.Abs(b.S - a.S)
	return h / 6 * (cpShoelace(a.Corners) + 4*cpShoelace(mid) + cpShoelace(b.Corners))
}

func cpShoelace(c [4][2]float64) float64 {
	var a float64
	for i := range 4 {
		j := (i + 1) % 4
		a += c[i][0]*c[j][1] - c[j][0]*c[i][1]
	}
	return math.Abs(a) / 2
}

// cpFacetSlack bounds how far a solid whose skew walls are each split into two
// flat triangles can differ in volume from the bilinear ruled solid: per wall,
// the volume between the two splittings of a skew quadrilateral is the volume
// of the tetrahedron on its four corners, and either splitting lies within it.
func cpFacetSlack(a, b cpQuad) float64 {
	var total float64
	for i := range 4 {
		j := (i + 1) % 4
		p0 := r3.NewVec(a.Corners[i][0], a.Corners[i][1], a.S)
		p1 := r3.NewVec(a.Corners[j][0], a.Corners[j][1], a.S)
		p2 := r3.NewVec(b.Corners[j][0], b.Corners[j][1], b.S)
		p3 := r3.NewVec(b.Corners[i][0], b.Corners[i][1], b.S)
		total += math.Abs(p1.Sub(p0).Cross(p2.Sub(p0)).Dot(p3.Sub(p0))) / 6
	}
	return total
}

// cpRuledVolume is the bilinear ruled volume through the sections, and the
// slack cpFacetSlack allows a faceted build of it.
func cpRuledVolume(secs []cpQuad) (float64, float64) {
	var v, slack float64
	for i := 1; i < len(secs); i++ {
		v += cpSimpson(secs[i-1], secs[i])
		slack += cpFacetSlack(secs[i-1], secs[i])
	}
	return v, slack
}

// cpLocalBox is the bounding box of sections given in a gear-local frame whose
// Z runs along the axis from station zero at zOrigin.
func cpLocalBox(secs []cpQuad, zOrigin float64) (r3.Vec, r3.Vec) {
	lo := r3.NewVec(math.Inf(1), math.Inf(1), math.Inf(1))
	hi := r3.NewVec(math.Inf(-1), math.Inf(-1), math.Inf(-1))
	for _, q := range secs {
		for _, c := range q.Corners {
			p := r3.NewVec(c[0], c[1], q.S-zOrigin)
			lo = r3.NewVec(min(lo.X, p.X), min(lo.Y, p.Y), min(lo.Z, p.Z))
			hi = r3.NewVec(max(hi.X, p.X), max(hi.Y, p.Y), max(hi.Z, p.Z))
		}
	}
	return lo, hi
}

// cpWindow is one window of §4: its facing direction and the six numbers its
// search settles, with lean.
type cpWindow struct {
	Name                             string
	D                                r3.Vec
	Lean                             float64
	Lo, Hi, Bottom, Top, Left, Right float64
}

// cpDefaultWindows are the two windows the search settles at the defaults.
// The spec quotes the +k̂ window's six numbers; the -k̂ window is the +k̂ one
// turned half a turn about ê, which carries (t, z) to (-t, -z): lo' = -hi,
// hi' = -lo, bottom' = -top, top' = -bottom, left' = -right, right' = -left.
// Both flanking pairs put the high bore at the larger t, so lean is +1 for
// both. The search itself runs in processInputs and is not rebuilt here; the
// compiled proof takes its results as numbers, as the spec directs.
func cpDefaultWindows() []cpWindow {
	pk := cpWindow{Name: "+k", D: r3.NewVec(0, 1, 0), Lean: 1,
		Lo: -3.335, Hi: 6.495, Bottom: -20.306, Top: 23.466, Left: -11.404, Right: 11.663}
	mk := cpWindow{Name: "-k", D: r3.NewVec(0, -1, 0), Lean: 1,
		Lo: -pk.Hi, Hi: -pk.Lo, Bottom: -pk.Top, Top: -pk.Bottom, Left: -pk.Right, Right: -pk.Left}
	return []cpWindow{pk, mk}
}

// across is n̂ × d, the window plane's t direction.
func (w cpWindow) across() r3.Vec { return r3.NewVec(0, 0, 1).Cross(w.D) }

// corners clips the square -2*Ro <= t, z <= 2*Ro to the six half-planes of
// §4's hexagon in the spec's order (clipCorners) and drops a corner within
// 0.001 mm of the one before it.
func (w cpWindow) corners(ro float64) [][2]float64 {
	poly := [][2]float64{{-2 * ro, -2 * ro}, {2 * ro, -2 * ro}, {2 * ro, 2 * ro}, {-2 * ro, 2 * ro}}
	// Each half-plane is a*t + b*z <= c.
	for _, hp := range [][3]float64{
		{w.Lean, 1, w.Hi},
		{-w.Lean, -1, -w.Lo},
		{-w.Lean, 1, w.Top},
		{w.Lean, -1, -w.Bottom},
		{1, 0, w.Right},
		{-1, 0, -w.Left},
	} {
		poly = cpClip(poly, hp[0], hp[1], hp[2])
	}
	var out [][2]float64
	for _, p := range poly {
		if len(out) > 0 && math.Hypot(p[0]-out[len(out)-1][0], p[1]-out[len(out)-1][1]) < 0.001 {
			continue
		}
		out = append(out, p)
	}
	if len(out) > 1 && math.Hypot(out[0][0]-out[len(out)-1][0], out[0][1]-out[len(out)-1][1]) < 0.001 {
		out = out[:len(out)-1]
	}
	return out
}

// cpClip keeps the part of a convex polygon where a*t + b*z <= c, walking its
// edges in order (Sutherland–Hodgman against one line).
func cpClip(poly [][2]float64, a, b, c float64) [][2]float64 {
	var out [][2]float64
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		fp := a*p[0] + b*p[1] - c
		fq := a*q[0] + b*q[1] - c
		if fp <= 0 {
			out = append(out, p)
		}
		if (fp < 0 && fq > 0) || (fp > 0 && fq < 0) {
			k := fp / (fp - fq)
			out = append(out, [2]float64{p[0] + k*(q[0]-p[0]), p[1] + k*(q[1]-p[1])})
		}
	}
	return out
}

// cpPolygonArea is the shoelace area of a polygon of any corner count.
func cpPolygonArea(poly [][2]float64) float64 {
	var a float64
	for i := range poly {
		j := (i + 1) % len(poly)
		a += poly[i][0]*poly[j][1] - poly[j][0]*poly[i][1]
	}
	return a / 2
}

// cpInsideConvex reports how far (x, y) lies inside a counter-clockwise convex
// polygon: the least distance to an edge line, negative when outside.
func cpInsideConvex(poly [][2]float64, x, y float64) float64 {
	best := math.Inf(1)
	for i := range poly {
		j := (i + 1) % len(poly)
		ex, ey := poly[j][0]-poly[i][0], poly[j][1]-poly[i][1]
		l := math.Hypot(ex, ey)
		d := (ex*(y-poly[i][1]) - ey*(x-poly[i][0])) / l
		best = min(best, d)
	}
	return best
}
