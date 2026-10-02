package screwgear_test

// The compiled step proof for the screw gear: one function per step of
// spec/screwgear/steps.md, built in the sketch and decad engines. This file
// holds the model every step shares — the dialog's parameters, the frame of
// the spec's §1, the derived counts and the four bores — and the case tables.
// The step functions are in compiled_sketches_test.go and
// compiled_solids_test.go; their registrations are generated into
// zz_registrations_test.go from the step list.
//
// Units. Every length here is millimetres and every angle in a case map is
// degrees, the dialog's own display units. The build works in centimetres and
// radians; the conversion is the build's ([PB-DIALOG-DEFAULT-UNITS]) and does
// not reach this proof.
//
// The frame. The proof puts the mechanism's centre C at the world origin, the
// Anchor Line's direction ê on +X, k̂ = n̂ × ê on +Y and the selected plane's
// normal n̂ on +Z, as the hand-written proof's pair does. The build reads ê and
// n̂ from Fusion; every number it then computes is the same function of them
// that the helpers below compute of X and Z.

// The steps the proof does not build, and why. Each is [PROSE] in the step
// list.
//
//   - The dialog, processInputs and the component tree build no geometry.
//   - The wall-between-the-bores check and the window search run in
//     processInputs; the spec has the compiled step proof take their results
//     as numbers (sgDefaultWindows), and the hand-written channelSeparation,
//     newWindow and TestSleeveWindowsFollowTheSize are their proof.
//   - The Gear A and Gear B Axis Planes and the Window Plane are construction
//     planes. The proof's sketches are drawn on those planes' coordinates
//     directly (the frame above), so a plane is a coordinate choice here, not
//     a feature with a result to verify; the build checks the one thing about
//     them that can go wrong, the sign of n̂ read back from Fusion, at run time.
//     The Window Plane for ±ê windows, made by setByDistanceOnPath at 0.5, is
//     not reached at all: no window numbers past a 90° crossing are stated.
//   - The doubling schedule is control flow over the copy, move and join steps,
//     which are built; the hand-written TestDoublingScheduleCoversTheRibbon
//     proves the schedule tiles the ribbon.
//   - Relocating the bodies (moveToComponent) and hiding construction geometry
//     have no engine counterpart and change no geometry.

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/r3"
)

// sgCellTeeth is the module constant CELL_TEETH of the spec's "Variables".
const sgCellTeeth = 4

// sgDefaults is the spec's default table ("Defaults", and the dialog table of
// "Variables"), keyed by dialog input id. The numbers come from the
// hand-written proof's sleeveParams, which the spec names as the table the
// compiled step proof calls, so a default that moves there moves here.
func sgDefaults() map[string]float64 {
	d := sleeveParams()
	return map[string]float64{
		"ribbonWidth":     d.Width,
		"toothCount":      float64(d.ToothCount),
		"twistLead":       d.TwistLead,
		"ribbonThickness": d.Thickness,
		"toothPitch":      d.ToothPitch,
		"toothHeight":     d.ToothHeight,
		"cageRadius":      d.CageRadius,
		"cageRise":        d.CageRise,
		"clearance":       d.Clearance,
		"roofAllowance":   d.RoofAllowance,
		"collarHalf":      d.CollarHalf,
		"collarWall":      d.CollarWall,
		"crossAngle":      d.CrossAngle * 180 / math.Pi,
		"engagement":      d.Engagement,
		"mountAngleA":     d.MountAngleA * 180 / math.Pi,
		"mountAngleB":     d.MountAngleB * 180 / math.Pi,
		// The hand-written proof keeps the assembly phase as a constant of its
		// own rather than a field of its table.
		"assemblyPhase": assemblyPhase,
	}
}

// sgWith returns the defaults with the given overrides applied. An override
// key that is not a dialog input (a gear index, a section, a bore) is added.
func sgWith(over map[string]float64) map[string]float64 {
	p := sgDefaults()
	for k, v := range over {
		p[k] = v
	}
	return p
}

// sgGear is one gear's frame of the spec's §1: its axis through origin along
// dir, its unrotated section axes u (toward the other gear) and v = dir × u,
// its mounting angle phi in radians and its tooth phase z0.
type sgGear struct {
	label   string
	origin  r3.Vec
	dir     r3.Vec
	u, v    r3.Vec
	phi     float64
	z0      float64
	uDotN   float64 // û_g·n̂: +1 for gear A, -1 for gear B
	lambda  float64
	toothed func(s float64) float64
}

// sgModel is every number processInputs derives, in millimetres and radians.
type sgModel struct {
	W, T, P, H, lead float64
	N                int
	sigma            float64
	engagement       float64
	cageRadius       float64
	cageRise         float64
	collarHalf       float64
	collarWall       float64
	clearance        float64
	roof             float64

	lambda float64 // TwistLead / 2π
	A      float64 // distance between the axes, W - Engagement
	n      int     // steps to the tooth
	c      int     // teeth in the cell, min(CELL_TEETH, N)
	q, r   int     // whole cells and remainder teeth
	L      float64 // ribbon length N*P

	Ri, Ro    float64 // sleeve radii
	hw, ht    float64 // bore half-width and half-thickness
	corner    float64 // c of §4, hypot(hw, ht + roofAllowance)
	sIn, sOut float64 // a +R bore's cut span on its axis

	gears [2]sgGear
}

func sgRad(deg float64) float64 { return deg * math.Pi / 180 }

func newModel(p map[string]float64) sgModel {
	m := sgModel{
		W: p["ribbonWidth"], T: p["ribbonThickness"], P: p["toothPitch"],
		H: p["toothHeight"], lead: p["twistLead"], N: int(math.Round(p["toothCount"])),
		sigma: sgRad(p["crossAngle"]), engagement: p["engagement"],
		cageRadius: p["cageRadius"], cageRise: p["cageRise"], collarHalf: p["collarHalf"],
		collarWall: p["collarWall"], clearance: p["clearance"], roof: p["roofAllowance"],
	}
	m.lambda = m.lead / (2 * math.Pi)
	m.A = m.W - m.engagement
	// §2: the larger of the 2°-twist count and the floor of eight.
	twistSteps := int(math.Ceil((m.P / m.lambda) / sgRad(2) * (1 - 1e-12)))
	m.n = max(twistSteps, 8)
	m.c = min(sgCellTeeth, m.N)
	m.q = m.N / m.c
	m.r = m.N % m.c
	m.L = float64(m.N) * m.P
	m.Ri = m.cageRadius - m.collarHalf
	m.Ro = m.cageRadius + m.collarHalf
	m.hw = m.W/2 + m.clearance
	m.ht = m.T/2 + m.clearance
	m.corner = math.Hypot(m.hw, m.ht+m.roof)
	m.sIn = math.Sqrt(m.Ri*m.Ri-m.corner*m.corner) - 1
	m.sOut = m.Ro + 1

	nHat := r3.NewVec(0, 0, 1)
	dirA := r3.NewVec(math.Cos(m.sigma/2), math.Sin(m.sigma/2), 0)
	dirB := r3.NewVec(math.Cos(m.sigma/2), -math.Sin(m.sigma/2), 0)
	ga := sgGear{label: "Gear A", origin: nHat.Scale(-m.A / 2), dir: dirA, u: nHat,
		phi: sgRad(p["mountAngleA"]), z0: 0, uDotN: 1, lambda: m.lambda}
	gb := sgGear{label: "Gear B", origin: nHat.Scale(m.A / 2), dir: dirB, u: nHat.Scale(-1),
		phi: sgRad(p["mountAngleB"]), z0: p["assemblyPhase"], uDotN: -1, lambda: m.lambda}
	ga.v = ga.dir.Cross(ga.u)
	gb.v = gb.dir.Cross(gb.u)
	for i, g := range []sgGear{ga, gb} {
		z0 := g.z0
		g.toothed = func(s float64) float64 {
			return m.W/2 - m.H/2 + (m.H/2)*math.Cos(2*math.Pi*(s-z0)/m.P)
		}
		m.gears[i] = g
	}
	return m
}

// theta is a section's angle at station s, s/Lambda + Phi_g.
func (g sgGear) theta(s float64) float64 { return s/g.lambda + g.phi }

// world is the point of §1 at station s with section coordinates (u, v).
func (g sgGear) world(s, u, v float64) r3.Vec {
	th := g.theta(s)
	x, y := sgTurn(u, v, th)
	return g.origin.Add(g.dir.Scale(s)).Add(g.u.Scale(x)).Add(g.v.Scale(y))
}

// sgTurn turns (u, v) by th: the section's coordinates on the plane square to
// the axis, x along û_g and y along v̂_g.
func sgTurn(u, v, th float64) (float64, float64) {
	return u*math.Cos(th) - v*math.Sin(th), u*math.Sin(th) + v*math.Cos(th)
}

// cellStart is s0 = Z0 - L/2, the ribbon's negative end, where the cell starts.
func (m sgModel) cellStart(g sgGear) float64 { return g.z0 - m.L/2 }

// cellCorners is the four corners of section k of a cell of `teeth` teeth that
// starts at station from, as plane coordinates on the section's own plane
// (x along û_g, y along v̂_g), with the station they stand at: the corners
// (uB, -hv), (uF, -hv), (uF, hv), (uB, hv) of §2 turned by theta_k.
func (m sgModel) cellCorners(g sgGear, from float64, k int) (s float64, xy [4][2]float64) {
	s = from + float64(k)*m.P/float64(m.n)
	uB, uF, hv := -m.W/2, g.toothed(s), m.T/2
	th := g.theta(s)
	for i, c := range [4][2]float64{{uB, -hv}, {uF, -hv}, {uF, hv}, {uB, hv}} {
		xy[i][0], xy[i][1] = sgTurn(c[0], c[1], th)
	}
	return s, xy
}

// sgBore is one of the four bores, in the order the build cuts them.
type sgBore struct {
	name     string
	gear     int
	sigma    float64 // -1 for the -R bore, +1 for the +R bore
	from, to float64 // the cut's span on the gear's axis, profile end first
	vLo, vHi float64 // the rectangle's extent across the thickness
	level    bool
}

// tilt is §4's "how near level" of a bore at sign sg: zero when some
// pi/2 + k*pi lies between the angles at the wall's two faces on the bore's
// centre line, else the smaller |cos theta| of the two.
func (m sgModel) tilt(g sgGear, sg float64) float64 {
	a := g.theta(sg * (m.cageRadius - m.collarHalf))
	b := g.theta(sg * (m.cageRadius + m.collarHalf))
	lo, hi := math.Min(a, b), math.Max(a, b)
	k := math.Ceil((lo - math.Pi/2) / math.Pi)
	if math.Pi/2+k*math.Pi <= hi {
		return 0
	}
	return math.Min(math.Abs(math.Cos(a)), math.Abs(math.Cos(b)))
}

// levelSign is the sign of a gear's level bore: the one with the smaller
// tilt, the -R bore on a tie.
func (m sgModel) levelSign(g sgGear) float64 {
	if m.tilt(g, 1) < m.tilt(g, -1) {
		return 1
	}
	return -1
}

// roofIsPlusV reports whether the level bore's roof is its +v face: the face
// up with the sleeve standing on its -n̂ end, where
// v̂(sc)·n̂ = -sin(theta(sc))*(û_g·n̂) is positive at sc = sigma*cageRadius.
func (m sgModel) roofIsPlusV(g sgGear, sg float64) bool {
	return -math.Sin(g.theta(sg*m.cageRadius))*g.uDotN > 0
}

// bores is the four bores in the build's order: gear A -R, gear A +R,
// gear B -R, gear B +R.
func (m sgModel) bores() [4]sgBore {
	var out [4]sgBore
	i := 0
	for gi, g := range m.gears {
		lvl := m.levelSign(g)
		for _, sg := range []float64{-1, 1} {
			b := sgBore{gear: gi, sigma: sg, vLo: -m.ht, vHi: m.ht}
			b.name = fmt.Sprintf("%s Bore %s", g.label, map[float64]string{-1: "-R", 1: "+R"}[sg])
			if sg < 0 {
				b.from, b.to = -m.sOut, -m.sIn
			} else {
				b.from, b.to = m.sIn, m.sOut
			}
			if sg == lvl {
				b.level = true
				if m.roofIsPlusV(g, sg) {
					b.vHi = m.ht + m.roof
				} else {
					b.vLo = -m.ht - m.roof
				}
			}
			out[i] = b
			i++
		}
	}
	return out
}

// boreSections is the stand-in's count from "What the proof's stand-in
// costs": no two neighbouring sections more than 5° of twist apart, nor more
// than the facet bound 2*acos(1 - 0.04*clearance/c), ceil(turn/step) + 1.
func (m sgModel) boreSections() int {
	turn := (m.sOut - m.sIn) / m.lambda
	step := math.Min(sgRad(5), 2*math.Acos(1-0.04*m.clearance/m.corner))
	return int(math.Ceil(turn/step*(1-1e-12))) + 1
}

// sgWindow is one window's numbers from the window search of §4, which the
// build runs in processInputs and the compiled step proof takes as numbers
// (the spec's "What the proof checks"). Only the defaults' numbers are
// stated in the spec, so the window steps are proved at the defaults alone;
// the hand-written TestSleeveWindowsFollowTheSize runs the search itself at
// 33 inputs, ±ê windows included.
type sgWindow struct {
	name                             string
	d                                r3.Vec // the facing direction
	lean                             float64
	lo, hi, bottom, top, left, right float64
	area                             float64 // mm² on the plane, as the spec quotes it
}

// sgDefaultWindows is §4's "The hexagon" at the defaults, the +k̂ window first.
func sgDefaultWindows() [2]sgWindow {
	return [2]sgWindow{
		{name: "Window +k", d: r3.NewVec(0, 1, 0), lean: 1,
			lo: -3.780, hi: 6.730, bottom: -20.751, top: 22.352, left: -11.523, right: 11.703, area: 220.0},
		{name: "Window -k", d: r3.NewVec(0, -1, 0), lean: 1,
			lo: -6.436, hi: 3.780, bottom: -22.645, top: 20.751, left: -11.670, right: 11.523, area: 215.1},
	}
}

// across is n̂ × d, the window plane's t direction.
func (w sgWindow) across() r3.Vec { return r3.NewVec(0, 0, 1).Cross(w.d) }

// corners is clipCorners of §4: the square |t|, |z| <= 2*Ro clipped, in this
// order, to lean*t + z <= hi, -lean*t - z <= -lo, -lean*t + z <= top,
// lean*t - z <= -bottom, t <= right and -t <= -left, dropping a corner
// within 0.001 mm of the one before it.
func (w sgWindow) corners(ro float64) [][2]float64 {
	q := [][2]float64{{-2 * ro, -2 * ro}, {2 * ro, -2 * ro}, {2 * ro, 2 * ro}, {-2 * ro, 2 * ro}}
	q = sgClip(q, w.lean, 1, w.hi)
	q = sgClip(q, -w.lean, -1, -w.lo)
	q = sgClip(q, -w.lean, 1, w.top)
	q = sgClip(q, w.lean, -1, -w.bottom)
	q = sgClip(q, 1, 0, w.right)
	q = sgClip(q, -1, 0, -w.left)
	var out [][2]float64
	for _, c := range q {
		if len(out) > 0 && math.Hypot(c[0]-out[len(out)-1][0], c[1]-out[len(out)-1][1]) < 0.001 {
			continue
		}
		out = append(out, c)
	}
	if len(out) > 1 && math.Hypot(out[0][0]-out[len(out)-1][0], out[0][1]-out[len(out)-1][1]) < 0.001 {
		out = out[:len(out)-1]
	}
	return out
}

// sgClip keeps the part of a convex polygon where a*t + b*z <= c, walking its
// edges in order, keeping each corner on the kept side and adding the point
// where an edge crosses the line.
func sgClip(q [][2]float64, a, b, c float64) [][2]float64 {
	var out [][2]float64
	for i := range q {
		p0, p1 := q[i], q[(i+1)%len(q)]
		f0 := a*p0[0] + b*p0[1] - c
		f1 := a*p1[0] + b*p1[1] - c
		if f0 <= 0 {
			out = append(out, p0)
		}
		if (f0 < 0 && f1 > 0) || (f0 > 0 && f1 < 0) {
			s := f0 / (f0 - f1)
			out = append(out, [2]float64{p0[0] + s*(p1[0]-p0[0]), p0[1] + s*(p1[1]-p0[1])})
		}
	}
	return out
}

// sgPolygonArea is the shoelace area of a polygon, positive counter-clockwise.
func sgPolygonArea(q [][2]float64) float64 {
	a := 0.0
	for i := range q {
		p0, p1 := q[i], q[(i+1)%len(q)]
		a += p0[0]*p1[1] - p1[0]*p0[1]
	}
	return a / 2
}

// sgDoublingRounds is the schedule of §3 for q cells: the moves, each the
// number of teeth Step moves the copy by, and whether the copy is an aside.
// It is the build's control flow, not a feature; the hand-written
// TestDoublingScheduleCoversTheRibbon proves it tiles the ribbon.
type sgRound struct {
	moveTeeth int // the copy moves by Step(moveTeeth)
	bodyCells int // cells the body holds before the join
	aside     bool
}

func sgDoublingRounds(q, c int) []sgRound {
	var out []sgRound
	m := 1
	var asides []int
	top := 0
	for (q >> (top + 1)) > 0 {
		top++
	}
	for bit := 0; bit < top; bit++ {
		if q&(1<<bit) != 0 {
			asides = append(asides, m)
		}
		out = append(out, sgRound{moveTeeth: m * c, bodyCells: m})
		m *= 2
	}
	for i := len(asides) - 1; i >= 0; i-- {
		out = append(out, sgRound{moveTeeth: m * c, bodyCells: m, aside: true})
		m += asides[i]
	}
	return out
}
