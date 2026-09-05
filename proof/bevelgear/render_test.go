package bevelgear_test

import (
	"flag"
	"math"
	"path/filepath"
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/render"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

// ---------------------------------------------------------------------------
// The README's bevel gear picture.
//
// This is not a proof and it is skipped unless -render.out names a directory.
// It is here, in the proof's own package, because the alternative is a second
// description of the same gear: the picture is built from the lattice
// gearLattice resolves and the tooth newToothOutline maps, trimmed on the cones
// section 2's own toe and heel edges lie on, so a change that moves the proved
// geometry moves the picture with it.
//
// What it does NOT reuse is the SOLID model. Every body in this gear is a cone,
// which decad builds as a Loft, and a union of two lofts that meet face to face
// is a tangent contact its exact predicates refuse to classify — the frustum's
// own bands meet that way. So the picture is meshed directly, the way
// cmd/genexamples meshes the spur gear's sweep, and the two conical trims that
// the solid proof substitutes for are meshed exactly instead of performed:
// every section of the tooth is the heel section scaled about the Apex, so a
// section point's own trim station is where its ray from the Apex crosses the
// trim cone, which is a closed form per point.
//
// The gear is drawn at Mean Spiral Angle 0 — a straight bevel, which is the
// path the dialog takes when the angle is left at zero. The spiral tooth is
// built by slicing the tooth body and rotating each slice, and the slice
// geometry is not part of what this file's lattice holds.
// ---------------------------------------------------------------------------

var renderOut = flag.String("render.out", "",
	"directory the README example image is written to; the render is skipped when it is empty")

// renderCase is the gear the README shows: the 16-tooth PINION of a 24/16 pair
// at a right angle in module 2, with the face width left to resolve and the
// bore on. The pinion is the steeper of the two cones — its pitch cone half
// angle is 33.7 degrees against the driving gear's 56.3 — which is the half of
// the pair a picture of one gear should be.
func renderCase() map[string]float64 {
	return params(map[string]float64{
		keyModule:       2,
		keyDrivingTeeth: 24,
		keyPinionTeeth:  16,
		keyShaftAngle:   90,
		keySpiralAngle:  0,
		keyGearSide:     0,
	})
}

// renderSegments is how many facets a revolved surface is chorded to. It is set
// by how round the frustum has to look at the output size and by nothing else:
// no measurement is taken off this mesh.
const renderSegments = 240

// renderArcSamples is how many segments each of the tooth section's tip and
// root arcs is drawn with.
const renderArcSamples = 10

// gearColor is the bevel gear's own colour in the README table, chosen to sit
// apart from the spur, helical and herringbone examples.
var gearColor = solidlens.RGB(0.13, 0.45, 0.35)

func TestRenderExample(t *testing.T) {
	if *renderOut == "" {
		t.Skip("no -render.out directory; the example image is not being regenerated")
	}
	p := renderCase()
	d := newDesign(t, p)
	g, f := sideOf(d, p)

	body, err := render.Revolve(frustumProfile(g, f), renderSegments)
	if err != nil {
		t.Fatalf("frustum: %v", err)
	}
	teeth, err := toothRing(t, d, g, f)
	if err != nil {
		t.Fatalf("teeth: %v", err)
	}
	if body, err = render.Placed(body, facing(t)); err != nil {
		t.Fatalf("turn the frustum to face the camera: %v", err)
	}
	if teeth, err = render.Placed(teeth, facing(t)); err != nil {
		t.Fatalf("turn the teeth to face the camera: %v", err)
	}

	scene := render.Scene(renderView(t, body, teeth),
		render.Part{Mesh: body, Color: gearColor},
		render.Part{Mesh: teeth, Color: gearColor})
	path := filepath.Join(*renderOut, "bevel.png")
	if err := render.WritePNG(t.Context(), path, scene, solidlens.Settings{Width: 960, Height: 720}); err != nil {
		t.Fatalf("write %s: %v", path, err)
	}
	t.Logf("wrote %s", path)
}

// frustumProfile is the Gear Body's revolve profile: the section 2 hexagon,
// with the bore taken out of it as a clip rather than as a boolean.
//
// The hexagon is the untruncated one. The toe stub the solid proof revolves
// instead is there because a LOFT cannot start at a degenerate ring; a
// revolution has no such end, so the picture carries the real toe corner.
func frustumProfile(g gear, f gearFrame) []render.Vec2 {
	poly := make([]render.Vec2, 0, len(f.hexagon()))
	for _, v := range f.hexagon() {
		poly = append(poly, render.Vec2{X: v.X, Y: v.Y})
	}
	return render.ClipRadius(poly, g.Bore/2)
}

// toothRing meshes the gear's whole ring of teeth as one mesh.
func toothRing(t *testing.T, d design, g gear, f gearFrame) (*solidlens.Mesh, error) {
	t.Helper()
	o := newToothOutline(d, g)
	section := toothSection(o)
	ends, err := render.EarClip(section)
	if err != nil {
		return nil, err
	}
	n := int(g.Teeth)
	meshes := make([]solidlens.TriangleSource, 0, n)
	for i := range n {
		mesh, err := tooth(g, f, rotated(section, 2*math.Pi*float64(i)/g.Teeth), ends)
		if err != nil {
			return nil, err
		}
		meshes = append(meshes, mesh)
	}
	return render.Merge(meshes...)
}

// tooth meshes one tooth, trimmed flush at both ends by the cones the gear
// body's own toe and heel faces lie on.
//
// The trim is exact and needs no boolean. The tooth's section at cone-distance
// fraction k is its heel section scaled by k, so a section point q travels the
// ray from the Apex through (q, Ded.X) and reaches radius k|q| at height
// k*Ded.X. A trim cone is generated by a line parallel to the back cone, so its
// radius is (axis - z) / tan gamma with axis its own apex on the shaft, and the
// two meet where k|q| * tan gamma = axis - k*Ded.X. That is one k per section
// point, which is why the rings below lie ON the cones rather than near them,
// and why the tooth's toe and heel ends come out conical rather than flat.
//
// The cones are the EXACT ones — the toe edge M->N and the heel edge C->H swept
// about the shaft — where cutConeStations offsets each by a sliver so a boolean
// has two surfaces to separate. Nothing is cut here, so nothing needs the
// sliver.
func tooth(g gear, f gearFrame, section []render.Vec2, ends [][3]int) (*solidlens.Mesh, error) {
	tanGamma := math.Tan(g.Gamma)
	heelAxis := f.Ded.X + f.Ded.Y*tanGamma
	toeAxis := f.Toe.X + f.Toe.Y*tanGamma

	near := make([]solidlens.Vec, len(section))
	far := make([]solidlens.Vec, len(section))
	for i, q := range section {
		// k * rate = axis, solved from the crossing above.
		rate := f.Ded.X + math.Hypot(q.X, q.Y)*tanGamma
		toe, heel := toeAxis/rate, heelAxis/rate
		near[i] = solidlens.Vec{X: toe * q.X, Y: toe * q.Y, Z: toe * f.Ded.X}
		far[i] = solidlens.Vec{X: heel * q.X, Y: heel * q.Y, Z: heel * f.Ded.X}
	}
	return render.Prism(near, far, ends)
}

// toothSection is the tooth's heel cross-section as one closed loop, walked
// counter-clockwise: up the right flank, across the tip, down the left flank,
// and back under the tooth along the root arc. newToothOutline puts the flank
// at the smaller angle in Right, which is what makes that walk the
// counter-clockwise one.
//
// The tooth is at the sink newToothOutline applies, a twentieth of the tooth
// height below the gear body's root cone. In the generated module the tooth
// seats exactly on that cone; here the sink also keeps the tooth's root face
// off the body's root face, which two coincident surfaces would otherwise
// fight over pixel by pixel.
func toothSection(o toothOutline) []render.Vec2 {
	right, left, rootR, tipR := o.section(1, 0, 1)
	loop := make([]render.Vec2, 0, 2*len(right)+2*renderArcSamples+2)

	rightFoot := right[0].unit().scale(rootR)
	leftFoot := left[0].unit().scale(rootR)
	if !o.Embedded {
		// The flank starts outside the root circle, so a radial line connects
		// its first sample to the root arc.
		loop = append(loop, planar(rightFoot))
	}
	loop = append(loop, points(right)...)
	loop = append(loop, arcSamples(tipR, bearingOf(right[len(right)-1]), bearingOf(left[len(left)-1]))...)
	for i := len(left) - 1; i >= 0; i-- {
		loop = append(loop, planar(left[i]))
	}
	if !o.Embedded {
		loop = append(loop, planar(leftFoot))
	}
	return append(loop, arcSamples(rootR, bearingOf(leftFoot), bearingOf(rightFoot))...)
}

func planar(p vec2) render.Vec2 { return render.Vec2{X: p.X, Y: p.Y} }

func points(src []vec2) []render.Vec2 {
	out := make([]render.Vec2, 0, len(src))
	for _, p := range src {
		out = append(out, planar(p))
	}
	return out
}

func bearingOf(p vec2) float64 { return math.Atan2(p.Y, p.X) }

// arcSamples returns the INTERIOR points of the arc of the given radius from
// bearing a to bearing b, the short way round. The endpoints are left out
// because the caller already holds them.
func arcSamples(radius, a, b float64) []render.Vec2 {
	sweep := wrapAngle(b - a)
	out := make([]render.Vec2, 0, renderArcSamples-1)
	for i := 1; i < renderArcSamples; i++ {
		th := a + sweep*float64(i)/renderArcSamples
		out = append(out, render.Vec2{X: radius * math.Cos(th), Y: radius * math.Sin(th)})
	}
	return out
}

// rotated turns a section about the shaft axis, which is how the tooth is
// patterned round the gear.
func rotated(section []render.Vec2, angle float64) []render.Vec2 {
	sin, cos := math.Sin(angle), math.Cos(angle)
	out := make([]render.Vec2, len(section))
	for i, p := range section {
		out[i] = render.Vec2{X: p.X*cos - p.Y*sin, Y: p.X*sin + p.Y*cos}
	}
	return out
}

// facing turns the gear a half turn about X, which brings the toe end — the
// end the teeth taper to, and the end that meets the other gear of the pair —
// round to the camera. The gear is built in the proof's own frame, with the
// Apex at the origin and the shaft along +Z, and that frame presents the flat
// back of the gear to a camera standing above it.
func facing(t *testing.T) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.NewVec(1, 0, 0), units.Radians(math.Pi))
	if err != nil {
		t.Fatalf("half turn about X: %v", err)
	}
	return turn
}

// renderView frames the gear on what it actually occupies, which is not
// anything the lattice names: the teeth stand proud of every point in it.
func renderView(t *testing.T, meshes ...solidlens.TriangleSource) solidlens.Camera {
	t.Helper()
	view, err := render.View{
		Distance:     2.3,
		ElevationDeg: 34,
		AzimuthDeg:   -58,
		FOV:          32,
	}.FitTo(meshes...)
	if err != nil {
		t.Fatalf("frame the gear: %v", err)
	}
	return view.Camera()
}
