package screwgear_test

// The model the compiled step proof builds from: the spec's frame of §1, the
// ribbon's section of §2, the bores of §4 and the two searches §4 runs before
// any feature. Every length here is in millimetres and every angle the
// parameter maps carry is in degrees; the build works in centimetres and
// radians and divides by ten where this multiplies by nothing. The names carry
// an `sg` prefix so they cannot collide with the hand-written mechanism proof
// that shares this package.

import (
	"fmt"
	"math"
	"sort"
	"strings"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// sgDefaults is the spec's default table ("Defaults, and what they were
// measured to do"), keyed by the dialog's input ids. centreX and centreY put the
// selected point off the world origin, so no step can lean on C being zero.
func sgDefaults() map[string]float64 {
	return map[string]float64{
		"ribbonWidth":     15,
		"toothCount":      68,
		"twistLead":       49.5,
		"ribbonThickness": 3.75,
		"toothPitch":      2.625,
		"toothHeight":     2.625,
		"toothSlant":      25.8,
		"toothBow":        0.048,
		"cageRadius":      15,
		"cageRise":        18.75,
		"clearance":       0.20,
		"roofAllowance":   0.30,
		"collarHalf":      3,
		"collarWall":      3,
		"crossAngle":      80,
		"engagement":      1.05,
		"mountAngleA":     0,
		"mountAngleB":     0,
		"assemblyPhase":   -1.31,
		"centreX":         7,
		"centreY":         -4,
	}
}

// sgWith returns the defaults with the given entries replaced or added.
func sgWith(over map[string]float64) map[string]float64 {
	p := sgDefaults()
	for k, v := range over {
		p[k] = v
	}
	return p
}

// sgThirdPrint is the fit the third sleeve was printed at: the straight
// ridge, 14 degrees on both gears and a 0.90 mm engagement. It keeps a case
// whose bores are not all level and whose angle dimensions take the other
// reference line.
func sgThirdPrint(over map[string]float64) map[string]float64 {
	p := sgWith(map[string]float64{
		"toothSlant": 0, "toothBow": 0, "mountAngleA": 14, "mountAngleB": 14,
		"engagement": 0.90, "assemblyPhase": -1.30,
	})
	for k, v := range over {
		p[k] = v
	}
	return p
}

const (
	sgSplinePoints = 11 // TOOTH_SPLINE_POINTS
	sgCellTeeth    = 4  // CELL_TEETH
	sgMinSteps     = 8
	sgMaxStepDeg   = 2.0
)

// sgModel is one parameter case resolved into the frame of §1. The frame puts
// ê, k̂ and n̂ on the world X, Y and Z axes, as the selected plane is the
// world XY plane in every case here, with C at (centreX, centreY, 0).
type sgModel struct {
	W, T, P, H, Lead, Lambda    float64
	N                           int
	Slant, TanSlant, Bow        float64
	Sigma, Engage, A            float64
	Phi, Z0                     [2]float64
	CageR, Rise, Clear, Roof    float64
	CollarHalf, CollarWall      float64
	Ri, Ro, Hw, Ht, Corner      float64
	SIn, SOut                   float64
	Centre, Ex, Kx, Nx          r3.Vec
	Dir, Origin, U, V           [2]r3.Vec
	Cell, Steps, Whole, Remains int
}

func sgModelOf(p map[string]float64) *sgModel {
	m := &sgModel{
		W: p["ribbonWidth"], T: p["ribbonThickness"], P: p["toothPitch"], H: p["toothHeight"],
		Lead: p["twistLead"], N: int(math.Round(p["toothCount"])),
		Slant: p["toothSlant"] * math.Pi / 180, Bow: p["toothBow"],
		Sigma: p["crossAngle"] * math.Pi / 180, Engage: p["engagement"],
		CageR: p["cageRadius"], Rise: p["cageRise"], Clear: p["clearance"], Roof: p["roofAllowance"],
		CollarHalf: p["collarHalf"], CollarWall: p["collarWall"],
	}
	m.Lambda = m.Lead / (2 * math.Pi)
	m.TanSlant = math.Tan(m.Slant)
	m.A = m.W - m.Engage
	m.Phi = [2]float64{p["mountAngleA"] * math.Pi / 180, p["mountAngleB"] * math.Pi / 180}
	m.Z0 = [2]float64{0, p["assemblyPhase"]}
	m.Ri = m.CageR - m.CollarHalf
	m.Ro = m.CageR + m.CollarHalf
	m.Hw = m.W/2 + m.Clear
	m.Ht = m.T/2 + m.Clear
	m.Corner = math.Hypot(m.Hw, m.Ht+m.Roof)
	m.SIn = math.Sqrt(math.Max(0, m.Ri*m.Ri-m.Corner*m.Corner)) - 1
	m.SOut = m.Ro + 1
	m.Centre = r3.NewVec(p["centreX"], p["centreY"], 0)
	m.Ex, m.Kx, m.Nx = r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	half := m.Sigma / 2
	m.Dir[0] = r3.NewVec(math.Cos(half), math.Sin(half), 0)
	m.Dir[1] = r3.NewVec(math.Cos(half), -math.Sin(half), 0)
	m.Origin[0] = m.Centre.Sub(m.Nx.Scale(m.A / 2))
	m.Origin[1] = m.Centre.Add(m.Nx.Scale(m.A / 2))
	m.U[0] = m.Nx
	m.U[1] = m.Nx.Scale(-1)
	for g := 0; g < 2; g++ {
		m.V[g] = m.Dir[g].Cross(m.U[g])
	}
	m.Cell = min(sgCellTeeth, m.N)
	m.Steps = max(int(math.Ceil((m.P/m.Lambda)/(sgMaxStepDeg*math.Pi/180)-1e-12)), sgMinSteps)
	m.Whole = m.N / m.Cell
	m.Remains = m.N % m.Cell
	return m
}

// theta is the cross-section angle of gear g at station s.
func (m *sgModel) theta(g int, s float64) float64 { return s/m.Lambda + m.Phi[g] }

// turn rotates section coordinates (u, v) by theta into the gear's (x, y),
// x along û_g and y along v̂_g.
func sgTurn(u, v, th float64) (float64, float64) {
	c, s := math.Cos(th), math.Sin(th)
	return u*c - v*s, u*s + v*c
}

// world is the world point of gear g at station s and section coordinates
// (u, v), as §1 writes it.
func (m *sgModel) world(g int, s, u, v float64) r3.Vec {
	x, y := sgTurn(u, v, m.theta(g, s))
	return m.Origin[g].Add(m.Dir[g].Scale(s)).Add(m.U[g].Scale(x)).Add(m.V[g].Scale(y))
}

// local maps a world point into gear g's frame: x along û_g, y along v̂_g, and
// the station along dir_g.
func (m *sgModel) local(g int, q r3.Vec) (x, y, s float64) {
	d := q.Sub(m.Origin[g])
	return d.Dot(m.U[g]), d.Dot(m.V[g]), d.Dot(m.Dir[g])
}

// utooth is the toothed edge of "The part" for gear g. sign is +1 for the
// input slant and -1 for its negation, which only the slant's sign check reads.
func (m *sgModel) utooth(g int, v, s, sign float64) float64 {
	arg := 2 * math.Pi * (s + sign*m.TanSlant*v - m.Z0[g]) / m.P
	return m.W/2 - m.H/2 + (m.H/2)*math.Cos(arg) - m.Bow*v*v
}

// cellStart is s0, the first station of gear g's cell.
func (m *sgModel) cellStart(g int) float64 { return m.Z0[g] - float64(m.N)*m.P/2 }

// station is s_k, the k-th section's station of gear g's cell.
func (m *sgModel) station(g, k int) float64 { return m.cellStart(g) + float64(k)*m.P/float64(m.Steps) }

// sgSectionPoints are a section's M + 2 points in section coordinates (u, v),
// in the order the sketch adds them: B0, the toothed points F_0 … F_(M-1), B1.
func (m *sgModel) sectionUV(g int, s float64) (b0, b1 [2]float64, f [][2]float64) {
	hv := m.T / 2
	b0 = [2]float64{-m.W / 2, -hv}
	b1 = [2]float64{-m.W / 2, hv}
	for j := 0; j < sgSplinePoints; j++ {
		v := -hv + float64(j)*m.T/float64(sgSplinePoints-1)
		f = append(f, [2]float64{m.utooth(g, v, s, 1), v})
	}
	return b0, b1, f
}

// sectionPolygon is the section's outline in section-plane coordinates (x
// along û_g, y along v̂_g), turned to the station's angle: B0, F_0 … F_(M-1),
// B1, counter-clockwise.
func (m *sgModel) sectionPolygon(g int, s float64) [][2]float64 {
	b0, b1, f := m.sectionUV(g, s)
	th := m.theta(g, s)
	var out [][2]float64
	add := func(q [2]float64) {
		x, y := sgTurn(q[0], q[1], th)
		out = append(out, [2]float64{x, y})
	}
	add(b0)
	for _, q := range f {
		add(q)
	}
	add(b1)
	return out
}

// sgScrewStep is Step(k) of §3 about gear g's axis: k teeth of advance and
// k*P/Lambda of turn, right-handed about +dir_g.
func (m *sgModel) screwStep(g, k int) (r3.Transform, error) {
	turn, err := r3.RotationAround(m.Origin[g], m.Dir[g], units.Radians(float64(k)*m.P/m.Lambda))
	if err != nil {
		return r3.Transform{}, err
	}
	shift, err := r3.Translation(m.Dir[g].Scale(float64(k) * m.P))
	if err != nil {
		return r3.Transform{}, err
	}
	return turn.Then(shift)
}

// sgDoubling is the schedule of §3 for q whole cells: the asides taken on the
// way up, as the number of cells each holds, and the number of doublings.
type sgRound struct {
	Aside   bool // a copy kept unmoved, before the doubling at this size
	Cells   int  // cells the body holds before this round
	MoveBy  int  // cells the copy is moved by, for a doubling or an aside's placement
	Placing bool // an aside being moved into place after the last doubling
}

func sgDoubling(q int) []sgRound {
	if q <= 1 {
		return nil
	}
	top := 0
	for (q >> (top + 1)) > 0 {
		top++
	}
	var rounds []sgRound
	var asides []int
	cells := 1
	for bit := 0; bit < top; bit++ {
		if q&(1<<bit) != 0 {
			asides = append(asides, cells)
		}
		rounds = append(rounds, sgRound{Cells: cells, MoveBy: cells})
		cells *= 2
	}
	for i := len(asides) - 1; i >= 0; i-- {
		rounds = append(rounds, sgRound{Placing: true, Cells: cells, MoveBy: cells, Aside: true})
		cells += asides[i]
	}
	if cells != q {
		panic(fmt.Sprintf("doubling schedule reached %d cells, want %d", cells, q))
	}
	return rounds
}

// sgBore is one of the four bores of §4, in the build's cut order.
type sgBore struct {
	Name           string
	Gear           int
	Sigma          float64 // -1 for the -R bore, +1 for the +R bore
	Level          bool
	RoofPlus       bool // the roof is the +v face
	VLo, VHi       float64
	SpanLo, SpanHi float64 // the cut's stations on the gear's axis
}

// tilt is how far the bore's long faces stay from level over the wall's span
// on its centre line ("The roof allowance").
func (m *sgModel) tilt(g int, sigma float64) float64 {
	t1 := m.theta(g, sigma*(m.CageR-m.CollarHalf))
	t2 := m.theta(g, sigma*(m.CageR+m.CollarHalf))
	lo, hi := math.Min(t1, t2), math.Max(t1, t2)
	k := math.Ceil((lo - math.Pi/2) / math.Pi)
	if math.Pi/2+k*math.Pi <= hi {
		return 0
	}
	return math.Min(math.Abs(math.Cos(t1)), math.Abs(math.Cos(t2)))
}

// bores returns the four bores in cut order: gear A -R, gear A +R, gear B -R,
// gear B +R.
func (m *sgModel) bores() []sgBore {
	var out []sgBore
	for g := 0; g < 2; g++ {
		levelSigma := -1.0
		if m.tilt(g, +1) < m.tilt(g, -1) {
			levelSigma = +1
		}
		for _, sigma := range []float64{-1, +1} {
			b := sgBore{Gear: g, Sigma: sigma, VLo: -m.Ht, VHi: m.Ht}
			b.Name = fmt.Sprintf("%s %s", sgGearLabel(g), map[float64]string{-1: "-R", 1: "+R"}[sigma])
			if sigma < 0 {
				b.SpanLo, b.SpanHi = -m.SOut, -m.SIn
			} else {
				b.SpanLo, b.SpanHi = m.SIn, m.SOut
			}
			if sigma == levelSigma {
				b.Level = true
				sc := sigma * m.CageR
				up := -math.Sin(m.theta(g, sc)) * m.U[g].Dot(m.Nx)
				b.RoofPlus = up > 0
				if b.RoofPlus {
					b.VHi = m.Ht + m.Roof
				} else {
					b.VLo = -m.Ht - m.Roof
				}
			}
			out = append(out, b)
		}
	}
	return out
}

func sgGearLabel(g int) string { return []string{"Gear A", "Gear B"}[g] }

// planeStation is the station the bore's section plane and profile stand at:
// the negative end of its cut span.
func (b sgBore) planeStation() float64 { return b.SpanLo }

// crossing is the bore's crossing station, ±cageRadius.
func (m *sgModel) crossing(b sgBore) float64 { return b.Sigma * m.CageR }

// sgRefusal runs the range checks of "Variables" in the build's order and
// returns the id the first failing check names, or "" when every check passes.
// The fourth sleeve check needs the separation, which the caller passes.
func (m *sgModel) refusal(p map[string]float64) string {
	switch {
	case m.W <= 0:
		return "ribbonWidth"
	case m.T <= 0:
		return "ribbonThickness"
	case m.P <= 0:
		return "toothPitch"
	case m.Lead <= 0:
		return "twistLead"
	case m.CollarHalf <= 0:
		return "collarHalf"
	case m.CollarWall <= 0:
		return "collarWall"
	case m.Clear <= 0:
		return "clearance"
	case m.Roof < 0:
		return "roofAllowance"
	case p["toothCount"] != math.Round(p["toothCount"]) || m.N < 4:
		return "toothCount"
	case m.H <= 0 || m.H >= m.W/2:
		return "toothHeight"
	case math.Abs(p["toothSlant"]) >= 90:
		return "toothSlant"
	case m.Bow < 0 || m.H+m.Bow*(m.T/2)*(m.T/2) >= m.W/2:
		return "toothBow"
	case m.Engage <= 0 || m.Engage > m.H:
		return "engagement"
	case p["crossAngle"] <= 0 || p["crossAngle"] >= 180:
		return "crossAngle"
	case math.Abs(m.Z0[1]) >= m.P:
		return "assemblyPhase"
	case m.CageR+m.CollarHalf+1 >= float64(m.N)*m.P/2:
		return "cageRadius"
	case math.Hypot(m.Corner, 1) >= m.Ri:
		return "cageRadius"
	case math.Hypot(m.axialWindow(), math.Hypot(m.W/2, m.T/2))+m.Clear > m.Ri:
		return "cageRadius"
	case m.Rise < m.A/2+m.Corner+m.CollarWall:
		return "cageRise"
	}
	return ""
}

func (m *sgModel) axialWindow() float64 {
	return 1.5 * math.Sqrt(m.W*m.W-m.A*m.A) / math.Sin(m.Sigma)
}

// sgClip keeps the part of a convex polygon with a*x + b*y <= c, walking its
// edges in order and adding the point where an edge crosses the line.
func sgClip(poly [][2]float64, a, b, c float64) [][2]float64 {
	var out [][2]float64
	n := len(poly)
	for i := 0; i < n; i++ {
		p, q := poly[i], poly[(i+1)%n]
		fp := a*p[0] + b*p[1] - c
		fq := a*q[0] + b*q[1] - c
		if fp <= 0 {
			out = append(out, p)
		}
		if (fp < 0 && fq > 0) || (fp > 0 && fq < 0) {
			t := fp / (fp - fq)
			out = append(out, [2]float64{p[0] + t*(q[0]-p[0]), p[1] + t*(q[1]-p[1])})
		}
	}
	return out
}

// sectionInWall is §4's two pieces of the bore's section in the wall at
// station s, in the section plane's (x, y).
func (m *sgModel) sectionInWall(b sgBore, s float64) [2][][2]float64 {
	var pieces [2][][2]float64
	if math.Abs(s) >= m.Ro {
		return pieces
	}
	th := m.theta(b.Gear, s)
	var poly [][2]float64
	for _, c := range [][2]float64{{-m.Hw, b.VLo}, {m.Hw, b.VLo}, {m.Hw, b.VHi}, {-m.Hw, b.VHi}} {
		x, y := sgTurn(c[0], c[1], th)
		poly = append(poly, [2]float64{x, y})
	}
	zg := m.Origin[b.Gear].Sub(m.Centre).Dot(m.Nx)
	un := m.U[b.Gear].Dot(m.Nx)
	poly = sgClip(poly, un, 0, m.Rise-zg)
	poly = sgClip(poly, -un, 0, m.Rise+zg)
	near := math.Sqrt(math.Max(0, m.Ri*m.Ri-s*s))
	far := math.Sqrt(m.Ro*m.Ro - s*s)
	pieces[0] = sgClip(sgClip(poly, 0, -1, -near), 0, 1, far)
	pieces[1] = sgClip(sgClip(poly, 0, 1, -near), 0, -1, far)
	return pieces
}

// planeCoords are a world point's (t, z, depth) for a window facing d.
func (m *sgModel) planeCoords(q, d r3.Vec) (t, z, a float64) {
	across := m.Nx.Cross(d)
	rel := q.Sub(m.Centre)
	return rel.Dot(across), rel.Dot(m.Nx), rel.Dot(d)
}

// channelOutline is step 1 of "The wall between the bores".
func (m *sgModel) channelOutline(b sgBore) []r3.Vec {
	var kept []r3.Vec
	for k := 0; ; k++ {
		s := b.Sigma * (m.SIn + 0.1*float64(k))
		if math.Abs(s) > m.SOut+1e-12 {
			break
		}
		th := m.theta(b.Gear, s)
		emit := func(u, v float64) {
			x, y := sgTurn(u, v, th)
			q := m.Origin[b.Gear].Add(m.Dir[b.Gear].Scale(s)).Add(m.U[b.Gear].Scale(x)).Add(m.V[b.Gear].Scale(y))
			rel := q.Sub(m.Centre)
			r := math.Hypot(rel.Dot(m.Ex), rel.Dot(m.Kx))
			if r >= m.Ri-0.5 && r <= m.Ro+0.5 {
				kept = append(kept, q)
			}
		}
		for i := 0; i <= 16; i++ {
			f := float64(i) / 16
			v := b.VLo + (b.VHi-b.VLo)*f
			u := -m.Hw + 2*m.Hw*f
			emit(m.Hw, v)
			emit(-m.Hw, v)
			emit(u, b.VHi)
			emit(u, b.VLo)
		}
	}
	return kept
}

// sgHull is Andrew's monotone chain, counter-clockwise, without collinear
// points.
func sgHull(pts [][2]float64) [][2]float64 {
	p := append([][2]float64(nil), pts...)
	sort.Slice(p, func(i, j int) bool {
		if p[i][0] != p[j][0] {
			return p[i][0] < p[j][0]
		}
		return p[i][1] < p[j][1]
	})
	if len(p) < 3 {
		return p
	}
	cross := func(o, a, b [2]float64) float64 {
		return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
	}
	var h [][2]float64
	for _, q := range p {
		for len(h) >= 2 && cross(h[len(h)-2], h[len(h)-1], q) <= 0 {
			h = h[:len(h)-1]
		}
		h = append(h, q)
	}
	lower := len(h) + 1
	for i := len(p) - 2; i >= 0; i-- {
		q := p[i]
		for len(h) >= lower && cross(h[len(h)-2], h[len(h)-1], q) <= 0 {
			h = h[:len(h)-1]
		}
		h = append(h, q)
	}
	return h[:len(h)-1]
}

// sgGap is one of the four gaps between neighbouring bores, named by the
// direction it faces and the indices of its two bores in cut order.
type sgGap struct {
	Name string
	D    r3.Vec
	B    [2]int
}

func (m *sgModel) gaps() []sgGap {
	// Cut order: 0 A -R, 1 A +R, 2 B -R, 3 B +R.
	return []sgGap{
		{"+k", m.Kx, [2]int{1, 2}},
		{"-e", m.Ex.Scale(-1), [2]int{0, 2}},
		{"-k", m.Kx.Scale(-1), [2]int{0, 3}},
		{"+e", m.Ex, [2]int{1, 3}},
	}
}

// channelSeparation is the fourth sleeve check: the least separation over
// the four gaps, and the gap it is across.
func (m *sgModel) channelSeparation() (float64, string) {
	bores := m.bores()
	outlines := make([][]r3.Vec, len(bores))
	for i, b := range bores {
		outlines[i] = m.channelOutline(b)
	}
	least, where := math.Inf(1), ""
	for _, gap := range m.gaps() {
		across := m.Nx.Cross(gap.D)
		var hulls [2][][2]float64
		for side, bi := range gap.B {
			var pts [][2]float64
			for _, q := range outlines[bi] {
				rel := q.Sub(m.Centre)
				pts = append(pts, [2]float64{rel.Dot(across), rel.Dot(m.Nx)})
			}
			hulls[side] = sgHull(pts)
		}
		best := math.Inf(-1)
		for _, h := range hulls {
			for i := range h {
				a, b := h[i], h[(i+1)%len(h)]
				ex, ey := b[0]-a[0], b[1]-a[1]
				l := math.Hypot(ex, ey)
				if l == 0 {
					continue
				}
				mx, my := -ey/l, ex/l
				proj := func(hh [][2]float64) (lo, hi float64) {
					lo, hi = math.Inf(1), math.Inf(-1)
					for _, q := range hh {
						d := mx*q[0] + my*q[1]
						lo, hi = math.Min(lo, d), math.Max(hi, d)
					}
					return lo, hi
				}
				plo, phi := proj(hulls[0])
				qlo, qhi := proj(hulls[1])
				best = math.Max(best, math.Max(qlo-phi, plo-qhi))
			}
		}
		if best < least {
			least, where = best, gap.Name
		}
	}
	return least, where
}

// channelTop is zLimit, the furthest any channel reaches from the middle
// plane inside the wall.
func (m *sgModel) channelTop() float64 {
	z := 0.0
	for _, b := range m.bores() {
		zg := m.Origin[b.Gear].Sub(m.Centre).Dot(m.Nx)
		un := m.U[b.Gear].Dot(m.Nx)
		corners := [][2]float64{{-m.Hw, b.VLo}, {m.Hw, b.VLo}, {m.Hw, b.VHi}, {-m.Hw, b.VHi}}
		for k := 0; ; k++ {
			s := b.Sigma * (m.SIn + 0.01*float64(k))
			if math.Abs(s) > m.SOut+1e-12 {
				break
			}
			if math.Abs(s) > m.Ro {
				continue
			}
			th := m.theta(b.Gear, s)
			ymax := 0.0
			for _, c := range corners {
				_, y := sgTurn(c[0], c[1], th)
				ymax = math.Max(ymax, math.Abs(y))
			}
			if math.Hypot(s, ymax) < m.Ri {
				continue
			}
			for _, c := range corners {
				x, _ := sgTurn(c[0], c[1], th)
				z = math.Max(z, math.Abs(zg+x*un))
			}
		}
	}
	return z
}

// sgStation is one row of a bore's station table for the distance walk.
type sgStation struct {
	S      float64
	Pieces [][][2]float64 // the non-empty pieces
	Circle [][3]float64   // per piece: centre x, centre y, radius
}

type sgTable struct {
	Bore sgBore
	Rows []sgStation
}

func (m *sgModel) stationRow(b sgBore, s float64) sgStation {
	row := sgStation{S: s}
	for _, pc := range m.sectionInWall(b, s) {
		if len(pc) == 0 {
			continue
		}
		cx, cy := 0.0, 0.0
		for _, q := range pc {
			cx += q[0]
			cy += q[1]
		}
		cx /= float64(len(pc))
		cy /= float64(len(pc))
		r := 0.0
		for _, q := range pc {
			r = math.Max(r, math.Hypot(q[0]-cx, q[1]-cy))
		}
		row.Pieces = append(row.Pieces, pc)
		row.Circle = append(row.Circle, [3]float64{cx, cy, r})
	}
	return row
}

func (m *sgModel) stationTable(b sgBore) *sgTable {
	tab := &sgTable{Bore: b}
	var coarse []sgStation
	for k := 0; ; k++ {
		s := b.SpanLo + float64(k)*0.002
		if s > b.SpanHi+1e-12 {
			break
		}
		coarse = append(coarse, m.stationRow(b, s))
	}
	signature := func(s float64) [2]bool {
		pc := m.sectionInWall(b, s)
		return [2]bool{len(pc[0]) > 0, len(pc[1]) > 0}
	}
	for i, row := range coarse {
		if i > 0 && signature(coarse[i-1].S) != signature(row.S) {
			for k := 1; ; k++ {
				s := coarse[i-1].S + float64(k)*0.0001
				if s >= row.S-1e-12 {
					break
				}
				tab.Rows = append(tab.Rows, m.stationRow(b, s))
			}
		}
		tab.Rows = append(tab.Rows, row)
	}
	return tab
}

// sgPointInPiece is true when (x, y) is on the inner side of every edge of a
// counter-clockwise polygon of three or more corners.
func sgPointInPiece(pc [][2]float64, x, y float64) bool {
	if len(pc) < 3 {
		return false
	}
	for i := range pc {
		a, b := pc[i], pc[(i+1)%len(pc)]
		if (b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]) < 0 {
			return false
		}
	}
	return true
}

func sgSegDist2(x, y float64, a, b [2]float64) float64 {
	ex, ey := b[0]-a[0], b[1]-a[1]
	l2 := ex*ex + ey*ey
	t := 0.0
	if l2 > 0 {
		t = math.Max(0, math.Min(1, ((x-a[0])*ex+(y-a[1])*ey)/l2))
	}
	dx, dy := a[0]+t*ex-x, a[1]+t*ey-y
	return dx*dx + dy*dy
}

// wallGap is the distance from q to the bore's channel in the wall, capped at
// reach.
func (m *sgModel) wallGap(tab *sgTable, q r3.Vec, reach float64) float64 {
	b := tab.Bore
	x, y, sq := m.local(b.Gear, q)
	clamped := math.Min(math.Max(sq, b.SpanLo), b.SpanHi)
	if math.Hypot(math.Hypot(x, y), sq-clamped)-m.Corner >= reach {
		return reach
	}
	best := reach * reach
	visit := func(row sgStation) bool {
		ds := sq - row.S
		if ds*ds >= best {
			return false
		}
		for i, pc := range row.Pieces {
			c := row.Circle[i]
			o := math.Hypot(x-c[0], y-c[1]) - c[2]
			if o > 0 && ds*ds+o*o >= best {
				continue
			}
			d2 := 0.0
			if !sgPointInPiece(pc, x, y) {
				d2 = math.Inf(1)
				for j := range pc {
					d2 = math.Min(d2, sgSegDist2(x, y, pc[j], pc[(j+1)%len(pc)]))
				}
			}
			best = math.Min(best, ds*ds+d2)
		}
		return true
	}
	start := sort.Search(len(tab.Rows), func(i int) bool { return tab.Rows[i].S >= sq })
	for i := start; i < len(tab.Rows); i++ {
		if !visit(tab.Rows[i]) {
			break
		}
	}
	for i := start - 1; i >= 0; i-- {
		if !visit(tab.Rows[i]) {
			break
		}
	}
	return math.Sqrt(best)
}

// sgWindow is one window the search found room for.
type sgWindow struct {
	Facing                           string
	D, Across                        r3.Vec
	Lean                             float64
	Lo, Hi, Bottom, Top, Left, Right float64
	Corners                          [][2]float64 // (t, z), counter-clockwise
	Reason                           string       // why there is no window, when Corners is nil
}

// windowFacings is the pair of directions the windows face, d before -d.
func (m *sgModel) windowFacings() []struct {
	Name string
	D    r3.Vec
} {
	if m.Sigma <= math.Pi/2+1e-12 {
		return []struct {
			Name string
			D    r3.Vec
		}{{"+k", m.Kx}, {"-k", m.Kx.Scale(-1)}}
	}
	return []struct {
		Name string
		D    r3.Vec
	}{{"+e", m.Ex}, {"-e", m.Ex.Scale(-1)}}
}

// newWindow is the window search of §4 for the window facing d.
func (m *sgModel) newWindow(name string, d r3.Vec, zLimit float64, tables []*sgTable) sgWindow {
	w := sgWindow{Facing: name, D: d, Across: m.Nx.Cross(d)}
	bores := m.bores()
	var flank, far []int
	for i, b := range bores {
		crossing := m.Origin[b.Gear].Add(m.Dir[b.Gear].Scale(m.crossing(b)))
		if crossing.Sub(m.Centre).Dot(d) > 0 {
			flank = append(flank, i)
		} else {
			far = append(far, i)
		}
	}
	if len(flank) != 2 || len(far) != 2 {
		w.Reason = fmt.Sprintf("%d flanking bores", len(flank))
		return w
	}
	cross := func(i int) r3.Vec {
		b := bores[i]
		return m.Origin[b.Gear].Add(m.Dir[b.Gear].Scale(m.crossing(b)))
	}
	low, high := flank[0], flank[1]
	if cross(high).Sub(m.Centre).Dot(m.Nx) < cross(low).Sub(m.Centre).Dot(m.Nx) {
		low, high = high, low
	}
	w.Lean = -1
	tl, _, _ := m.planeCoords(cross(low), d)
	th, _, _ := m.planeCoords(cross(high), d)
	if th > tl {
		w.Lean = 1
	}
	reach := func(i int, highest bool) float64 {
		b := bores[i]
		best := math.Inf(1)
		if highest {
			best = math.Inf(-1)
		}
		for k := 0; ; k++ {
			s := b.SpanLo + float64(k)*0.001
			if s > b.SpanHi+1e-12 {
				break
			}
			for _, pc := range m.sectionInWall(b, s) {
				for _, c := range pc {
					q := m.Origin[b.Gear].Add(m.U[b.Gear].Scale(c[0])).Add(m.V[b.Gear].Scale(c[1])).Add(m.Dir[b.Gear].Scale(s))
					t, z, _ := m.planeCoords(q, d)
					mm := z + w.Lean*t
					if highest {
						best = math.Max(best, mm)
					} else {
						best = math.Min(best, mm)
					}
				}
			}
		}
		return best
	}
	w.Lo = reach(low, true) + math.Sqrt2*m.CollarWall
	w.Hi = reach(high, false) - math.Sqrt2*m.CollarWall
	w.Top = math.Min(2*zLimit-w.Hi, w.Hi+math.Sqrt2*m.Ri)
	w.Bottom = math.Max(-2*zLimit-w.Lo, w.Lo-math.Sqrt2*m.Ri)
	if w.Hi <= w.Lo {
		w.Reason = fmt.Sprintf("the flanking bores leave no band (lo %.3f mm, hi %.3f mm)", w.Lo, w.Hi)
		return w
	}
	need := m.CollarWall + 0.1/math.Sqrt2 + 0.005
	a0 := func(t float64) float64 { return math.Sqrt(math.Max(0, m.Ri*m.Ri-t*t)) }
	a1 := func(t float64) float64 { return math.Sqrt(math.Max(0, m.Ro*m.Ro-t*t)) }
	at := func(t, z, a float64) r3.Vec {
		return m.Centre.Add(w.Across.Scale(t)).Add(m.Nx.Scale(z)).Add(d.Scale(a))
	}
	clear := func(t float64) bool {
		zLow := math.Max(w.Lo-w.Lean*t, w.Bottom+w.Lean*t)
		zHigh := math.Min(w.Hi-w.Lean*t, w.Top+w.Lean*t)
		if zLow > zHigh {
			return false
		}
		var pts []r3.Vec
		for _, z := range []float64{zLow, zHigh} {
			for a := a0(t); ; a += 0.1 {
				if a >= a1(t) {
					pts = append(pts, at(t, z, a1(t)))
					break
				}
				pts = append(pts, at(t, z, a))
			}
		}
		n := int(math.Ceil((zHigh - zLow) / 0.1))
		var zs []float64
		for k := 1; k < n; k++ {
			zs = append(zs, zLow+(zHigh-zLow)*float64(k)/float64(n))
		}
		for j := 0; ; j++ {
			z := -zLimit + float64(j)*0.1
			if z > zLimit {
				break
			}
			if z > zLow && z < zHigh {
				zs = append(zs, z)
			}
		}
		for _, z := range zs {
			pts = append(pts, at(t, z, a0(t)), at(t, z, a1(t)))
		}
		for _, q := range pts {
			for _, fi := range far {
				if m.wallGap(tables[fi], q, need) < need {
					return false
				}
			}
		}
		return true
	}
	end := func(delta float64) float64 {
		lo, hi := 0.0, math.Min(m.Ri, m.Ro/math.Sqrt2)*(1-1e-9)
		for i := 0; i < 24; i++ {
			mid := (lo + hi) / 2
			if clear(delta * mid) {
				lo = mid
			} else {
				hi = mid
			}
		}
		return lo
	}
	w.Right = end(+1)
	w.Left = -end(-1)
	if w.Right <= w.Left {
		w.Reason = fmt.Sprintf("the far bores leave the band no length (left %.3f mm, right %.3f mm)", w.Left, w.Right)
		return w
	}
	corners := [][2]float64{{-2 * m.Ro, -2 * m.Ro}, {2 * m.Ro, -2 * m.Ro}, {2 * m.Ro, 2 * m.Ro}, {-2 * m.Ro, 2 * m.Ro}}
	corners = sgClip(corners, w.Lean, 1, w.Hi)
	corners = sgClip(corners, -w.Lean, -1, -w.Lo)
	corners = sgClip(corners, -w.Lean, 1, w.Top)
	corners = sgClip(corners, w.Lean, -1, -w.Bottom)
	corners = sgClip(corners, 1, 0, w.Right)
	corners = sgClip(corners, -1, 0, -w.Left)
	var kept [][2]float64
	for _, c := range corners {
		if len(kept) > 0 && math.Hypot(c[0]-kept[len(kept)-1][0], c[1]-kept[len(kept)-1][1]) < 0.001 {
			continue
		}
		kept = append(kept, c)
	}
	for len(kept) > 1 && math.Hypot(kept[0][0]-kept[len(kept)-1][0], kept[0][1]-kept[len(kept)-1][1]) < 0.001 {
		kept = kept[:len(kept)-1]
	}
	if len(kept) < 3 || sgArea(kept) <= 0 {
		w.Reason = fmt.Sprintf("the hexagon has %d distinct corners and no area", len(kept))
		return w
	}
	w.Corners = kept
	return w
}

// sgArea is the signed area of a polygon, positive counter-clockwise.
func sgArea(poly [][2]float64) float64 {
	a := 0.0
	for i := range poly {
		p, q := poly[i], poly[(i+1)%len(poly)]
		a += p[0]*q[1] - q[0]*p[1]
	}
	return a / 2
}

// windows runs the window search for both facings, d before -d.
func (m *sgModel) windows() []sgWindow {
	bores := m.bores()
	tables := make([]*sgTable, len(bores))
	for i, b := range bores {
		tables[i] = m.stationTable(b)
	}
	zLimit := m.channelTop()
	var out []sgWindow
	for _, f := range m.windowFacings() {
		out = append(out, m.newWindow(f.Name, f.D, zLimit, tables))
	}
	return out
}

// windowProbe is the point the build probes before and after a window's cut:
// the middle of the wall at the average of the hexagon's corners.
func (m *sgModel) windowProbe(w sgWindow) r3.Vec {
	tc, zc := 0.0, 0.0
	for _, c := range w.Corners {
		tc += c[0]
		zc += c[1]
	}
	tc /= float64(len(w.Corners))
	zc /= float64(len(w.Corners))
	a0 := math.Sqrt(math.Max(0, m.Ri*m.Ri-tc*tc))
	a1 := math.Sqrt(math.Max(0, m.Ro*m.Ro-tc*tc))
	return m.Centre.Add(w.Across.Scale(tc)).Add(m.Nx.Scale(zc)).Add(w.D.Scale((a0 + a1) / 2))
}

// sgFormatCorners renders corners for a failure message.
func sgFormatCorners(c [][2]float64) string {
	var parts []string
	for _, q := range c {
		parts = append(parts, fmt.Sprintf("(%.3f, %.3f)", q[0], q[1]))
	}
	return strings.Join(parts, " ")
}
