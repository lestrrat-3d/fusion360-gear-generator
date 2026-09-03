// Package bevelgear_test proves the bevel gear pair's geometry against
// spec/bevelgear/instructions.md, spec/bevelgear/fusion.md and
// spec/bevelgear/spiral-tooth-trace.md.
//
// This file holds the closed-form model every step shares: the input
// resolution with its bounds, the §2 lattice in the Gear Profiles sketch's own
// 2-D frame, and the parameter case tables. Lengths here are millimetres, the
// unit the spec states its formulas in; the generated module converts to
// Fusion's internal centimetres, which is a units question the spec pins in its
// Units note and no geometry question at all.
package bevelgear_test

import (
	"fmt"
	"math"
	"testing"
)

// vec2 is a point in the Gear Profiles sketch's own 2-D frame, millimetres.
type vec2 struct{ X, Y float64 }

func v2add(a, b vec2) vec2           { return vec2{a.X + b.X, a.Y + b.Y} }
func v2sub(a, b vec2) vec2           { return vec2{a.X - b.X, a.Y - b.Y} }
func v2scale(a vec2, s float64) vec2 { return vec2{a.X * s, a.Y * s} }
func v2dot(a, b vec2) float64        { return a.X*b.X + a.Y*b.Y }
func v2cross(a, b vec2) float64      { return a.X*b.Y - a.Y*b.X }
func v2len(a vec2) float64           { return math.Hypot(a.X, a.Y) }
func v2unit(a vec2) vec2             { n := v2len(a); return vec2{a.X / n, a.Y / n} }
func v2perp(a vec2) vec2             { return vec2{-a.Y, a.X} }

func v2rot(a vec2, t float64) vec2 {
	s, c := math.Sin(t), math.Cos(t)
	return vec2{a.X*c - a.Y*s, a.X*s + a.Y*c}
}

// v2distToLine is the perpendicular distance from p to the infinite line
// through origin along dir (dir must be a unit vector).
func v2distToLine(p, origin, dir vec2) float64 {
	return math.Abs(v2cross(dir, v2sub(p, origin)))
}

// v2intersect is the intersection of the lines (p1 along d1) and (p2 along d2).
func v2intersect(p1, d1, p2, d2 vec2) vec2 {
	den := v2cross(d1, d2)
	return v2add(p1, v2scale(d1, v2cross(v2sub(p2, p1), d2)/den))
}

// gearSide carries one gear of the pair: its own inputs, its pitch cone angle,
// and the §2 lattice points that belong to it. The pinion's points are named
// A, C, E, G, H, K, K', M, N in the spec; the driving gear's twins are
// B, D, F, I, J, L, L', O, P and are held in the same fields.
type gearSide struct {
	Label        string
	Teeth        float64
	PitchDia     float64
	Gamma        float64 // pitch cone half angle, radians
	BaseHeight   float64 // resolved
	MinBase      float64
	MaxBase      float64
	MinTeeth     float64
	Bore         float64 // resolved bore diameter
	VirtualTeeth float64

	AxisDir vec2 // unit Apex->A (pinion) / Apex->B (driving)
	Axis    vec2 // A / B
	DedDir  vec2 // unit Apex2->C (pinion) / Apex2->D (driving)
	Ded     vec2 // C / D
	E       vec2 // E / F
	G       vec2 // G / I
	H       vec2 // H / J
	K       vec2 // K / L
	KPrime  vec2 // K' / L'
	M       vec2 // M / O — toe edge, on the root axis
	N       vec2 // N / P — toe edge, on the drop line
}

// RootDir is the unit Apex->C (pinion) / Apex->D (driving) root cone element,
// which §3a calls coneVec.
func (g gearSide) RootDir(apex vec2) vec2 { return v2unit(v2sub(g.Ded, apex)) }

// config is one resolved parameter set plus the whole §2 figure.
type config struct {
	Module       float64
	Sigma        float64 // radians
	SigmaDeg     float64
	MaxSigmaDeg  float64
	ConeDistance float64
	R            float64 // Pitch Cone Distance
	Center       vec2
	Perp         vec2
	Apex         vec2
	Apex2        vec2
	FaceWidth    float64
	MaxFaceWidth float64
	ToothSpacing float64
	SpiralAngle  float64 // radians
	HandSign     float64 // +1 Right, -1 Left, as read from the dialog
	CutterRadius float64
	BoreEnable   bool
	GrowSign     float64 // +1 grows toward the target plane's normal
	Pinion       gearSide
	Driving      gearSide
	Problems     []string
}

func param(p map[string]float64, key string, def float64) float64 {
	if v, ok := p[key]; ok {
		return v
	}
	return def
}

// maxShaftAngleDeg is the Maximum Shaft Angle: the cone-angle singularity
// acos(-smaller/larger), which is exclusive, capped at an inclusive 150.
func maxShaftAngleDeg(ppd, dpd float64) (limit float64, coneLimited bool) {
	cone := math.Acos(-math.Min(ppd, dpd)/math.Max(ppd, dpd)) * 180 / math.Pi
	if cone <= 150 {
		return cone, true
	}
	return 150, false
}

// resolve runs the whole input pass: range checks, the two cone angles, the
// Minimum Teeth floor, the base-height window and the bore diameters, in the
// order the spec's Variables section fixes. It then builds the §2 lattice and
// resolves the Face Width against the Maximum Face Width read off that figure.
func resolve(p map[string]float64) config {
	var c config
	c.Module = param(p, "module", 1)
	drivingTeeth := math.Round(param(p, "drivingTeeth", 31))
	pinionTeeth := math.Round(param(p, "pinionTeeth", 31))
	c.SigmaDeg = param(p, "shaftAngleDeg", 90)
	c.Sigma = c.SigmaDeg * math.Pi / 180
	c.ToothSpacing = param(p, "toothSpacing", 0)
	c.SpiralAngle = param(p, "spiralAngleDeg", 0) * math.Pi / 180
	c.HandSign = param(p, "handSign", 1)
	c.CutterRadius = param(p, "cutterRadius", 0)
	c.BoreEnable = param(p, "boreEnable", 1) != 0
	c.GrowSign = param(p, "growSign", 1)

	problem := func(format string, args ...any) {
		c.Problems = append(c.Problems, fmt.Sprintf(format, args...))
	}
	if c.Module <= 0 {
		problem("Module must be greater than 0")
	}
	if drivingTeeth < 3 {
		problem("Driving Gear Teeth must be at least 3")
	}
	if pinionTeeth < 3 {
		problem("Pinion Gear Teeth must be at least 3")
	}
	ppd := c.Module * pinionTeeth
	dpd := c.Module * drivingTeeth
	c.ConeDistance = math.Hypot(dpd, ppd)

	limit, coneLimited := maxShaftAngleDeg(ppd, dpd)
	c.MaxSigmaDeg = limit
	if c.SigmaDeg < 30 {
		problem("Shaft Angle must be at least 30 deg")
	}
	if coneLimited && c.SigmaDeg >= limit {
		problem("Shaft Angle must be below %.2f deg", limit)
	}
	if !coneLimited && c.SigmaDeg > limit {
		problem("Shaft Angle must be at most %.2f deg", limit)
	}

	gammaP := math.Atan2(math.Sin(c.Sigma)*ppd, dpd+ppd*math.Cos(c.Sigma))
	gammaG := c.Sigma - gammaP
	c.R = (ppd / 2) / math.Sin(gammaP)

	c.Pinion = gearSide{Label: "Pinion", Teeth: pinionTeeth, PitchDia: ppd, Gamma: gammaP}
	c.Driving = gearSide{Label: "Driving", Teeth: drivingTeeth, PitchDia: dpd, Gamma: gammaG}

	for _, g := range []*gearSide{&c.Pinion, &c.Driving} {
		r := g.PitchDia / 2
		g.MinTeeth = 5.27 * math.Cos(g.Gamma)
		g.MaxBase = 0.95 * (r - 1.25*c.Module*math.Cos(g.Gamma)) * math.Tan(g.Gamma)
		g.MinBase = 1.05 * 1.25 * c.Module * math.Sin(g.Gamma)
		g.VirtualTeeth = math.Floor(2 * (r / math.Cos(g.Gamma)) / c.Module)
		if g.Teeth < g.MinTeeth {
			problem("%s Gear Teeth must be at least %.2f", g.Label, g.MinTeeth)
		}
	}

	// Base heights: the driving gear resolves first because the pinion's
	// fallback is the RESOLVED driving height scaled by the tooth ratio.
	drivingIn := param(p, "drivingBaseHeight", 0)
	pinionIn := param(p, "pinionBaseHeight", 0)
	resolveBase := func(g *gearSide, in, fallback float64) {
		v := in
		if v == 0 {
			v = fallback
			if v < g.MinBase {
				v = g.MinBase
			}
			if v > g.MaxBase {
				v = g.MaxBase
			}
		} else {
			if v > g.MaxBase {
				problem("%s Gear Base Height must be at most %.3f mm", g.Label, g.MaxBase)
			}
			if v < g.MinBase {
				problem("%s Gear Base Height must be at least %.3f mm", g.Label, g.MinBase)
			}
		}
		g.BaseHeight = v
	}
	resolveBase(&c.Driving, drivingIn, c.Module*drivingTeeth/8)
	resolveBase(&c.Pinion, pinionIn, c.Driving.BaseHeight*(pinionTeeth/drivingTeeth))

	c.Pinion.Bore = param(p, "pinionBore", 0)
	if c.Pinion.Bore == 0 {
		c.Pinion.Bore = ppd / 4
	}
	c.Driving.Bore = param(p, "drivingBore", 0)
	if c.Driving.Bore == 0 {
		c.Driving.Bore = dpd / 4
	}

	c.buildLattice()

	// Maximum Face Width, from the solved A, B, C, D, H, J.
	pinDist := v2distToLine(c.Pinion.Axis, c.Pinion.Ded, c.Pinion.DedDir)
	drvDist := v2distToLine(c.Driving.Axis, c.Driving.Ded, c.Driving.DedDir)
	c.MaxFaceWidth = 0.95 * math.Min(pinDist, drvDist)
	fw := param(p, "faceWidth", 0)
	if fw == 0 {
		fw = math.Min(c.ConeDistance/6, c.MaxFaceWidth)
	} else if fw > c.MaxFaceWidth {
		problem("Face Width must be at most %.3f mm", c.MaxFaceWidth)
	}
	c.FaceWidth = fw
	c.buildToe()
	return c
}

// buildLattice places the whole §2 figure in the Gear Profiles sketch's own
// frame: the projected centre at the origin, the projected anchor line along
// +X, and the grow direction (the target-plane normal, [BEVEL-F-GROW-SIDE])
// along +Y. Everything below is the closed form the spec seeds the sketch with.
func (c *config) buildLattice() {
	c.Center = vec2{0, 0}
	c.Perp = vec2{0, c.GrowSign}
	c.Apex = v2add(c.Center, v2scale(c.Perp, c.R*math.Cos(c.Driving.Gamma)+c.Driving.BaseHeight))
	c.Driving.Axis = v2add(c.Center, v2scale(c.Perp, c.Driving.BaseHeight))
	c.Driving.AxisDir = v2unit(v2sub(c.Driving.Axis, c.Apex))

	// The pinion shaft is the driving shaft rotated about the apex by the
	// Shaft Angle. Of the two senses keep the endpoint with the greater X.
	reach := c.R * math.Cos(c.Pinion.Gamma)
	plus := v2rot(c.Driving.AxisDir, c.Sigma)
	minus := v2rot(c.Driving.AxisDir, -c.Sigma)
	c.Pinion.AxisDir = plus
	if v2add(c.Apex, v2scale(minus, reach)).X > v2add(c.Apex, v2scale(plus, reach)).X {
		c.Pinion.AxisDir = minus
	}
	c.Pinion.Axis = v2add(c.Apex, v2scale(c.Pinion.AxisDir, reach))

	// Apex 2 sits in the interior wedge: the drop from A points at B.
	drop := v2perp(c.Pinion.AxisDir)
	if v2dot(drop, v2sub(c.Driving.Axis, c.Pinion.Axis)) < 0 {
		drop = v2scale(drop, -1)
	}
	c.Apex2 = v2add(c.Pinion.Axis, v2scale(drop, c.Pinion.PitchDia/2))

	// The two dedendum points sit on one line through Apex 2, perpendicular to
	// the Pitch Line. C is the one that moves toward the pinion shaft axis.
	pitchDir := v2unit(v2sub(c.Apex2, c.Apex))
	n := v2perp(pitchDir)
	if v2distToLine(v2add(c.Apex2, n), c.Apex, c.Pinion.AxisDir) > v2distToLine(c.Apex2, c.Apex, c.Pinion.AxisDir) {
		n = v2scale(n, -1)
	}
	c.Pinion.DedDir = n
	c.Driving.DedDir = v2scale(n, -1)

	for _, g := range []*gearSide{&c.Pinion, &c.Driving} {
		g.Ded = v2add(c.Apex2, v2scale(g.DedDir, 1.25*c.Module))
		// H is the point on the dedendum line whose along-shaft offset from the
		// A->Apex2 drop is this gear's base height; walking out from Apex 2 the
		// along-shaft coordinate rises at sin(gamma) per unit.
		g.H = v2add(c.Apex2, v2scale(g.DedDir, g.BaseHeight/math.Sin(g.Gamma)))
		foot := func(p vec2) vec2 {
			return v2add(c.Apex, v2scale(g.AxisDir, v2dot(v2sub(p, c.Apex), g.AxisDir)))
		}
		g.E = foot(g.Ded) // C->E perpendicular to the shaft axis
		g.G = foot(g.H)   // G->H perpendicular to the shaft axis
		// K is where the dedendum line crosses the shaft axis: the distance to
		// the axis falls at cos(gamma) per unit from its pitch radius.
		g.K = v2add(c.Apex2, v2scale(g.DedDir, (g.PitchDia/2)/math.Cos(g.Gamma)))
		g.KPrime = v2add(g.K, v2scale(g.DedDir, c.ToothSpacing))
	}
}

// buildToe places M->N and O->P: the dedendum line offset toward the apex by
// the resolved Face Width, with M on the root axis and N on the A->Apex2 drop.
func (c *config) buildToe() {
	for _, g := range []*gearSide{&c.Pinion, &c.Driving} {
		toward := v2perp(g.DedDir)
		if v2dot(toward, v2sub(c.Apex, g.Ded)) < 0 {
			toward = v2scale(toward, -1)
		}
		base := v2add(g.Ded, v2scale(toward, c.FaceWidth))
		g.M = v2intersect(base, g.DedDir, c.Apex, g.RootDir(c.Apex))
		g.N = v2intersect(base, g.DedDir, g.Axis, v2unit(v2sub(c.Apex2, g.Axis)))
	}
}

// gearOf picks the gear a case names: 0 (the default) is the pinion, which the
// spec builds first, and 1 is the driving gear.
func (c *config) gearOf(p map[string]float64) *gearSide {
	if param(p, "gear", 0) != 0 {
		return &c.Driving
	}
	return &c.Pinion
}

func mustResolve(t testing.TB, p map[string]float64) config {
	t.Helper()
	c := resolve(p)
	if len(c.Problems) != 0 {
		t.Fatalf("input resolution rejected a case the table expects to build: %v", c.Problems)
	}
	return c
}

func near(t testing.TB, got, want, tol float64, what string, args ...any) {
	t.Helper()
	if math.Abs(got-want) > tol {
		t.Errorf("%s: got %.9f, want %.9f (tolerance %.1e)", fmt.Sprintf(what, args...), got, want, tol)
	}
}
