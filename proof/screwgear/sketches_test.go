package screwgear_test

import (
	"testing"

	"github.com/lestrrat-3d/fusion360-gear-generator/proof/proofkit"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/sketch/sketchtest"
)

var anchorCases = []proofkit.Case{
	{Name: "origin", Params: map[string]float64{"x": 0, "y": 0}},
	{Name: "translated", Params: map[string]float64{"x": 13, "y": -7}},
}

func stepAnchor(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	proofkit.Step(t, "project selected centre and constrain the Anchor Line")
	centre := s.CreateReferencePoint(p["x"], p["y"], "selected-centre")
	a := s.CreatePoint(p["x"]-5, p["y"])
	b := s.CreatePoint(p["x"]+5, p["y"])
	line := s.CreateLine(a, b)
	// The midpoint includes Fusion's separate point-on-line coincidence row.
	mid := sketch.NewMidpoint(centre, line)
	horizontal := sketch.NewHorizontal(line)
	length := sketch.NewHorizontalDistance(a, b, 10)
	s.AddConstraint(mid, horizontal, length)
	sketchtest.Solve(t, s)
	// The exact 10 mm formula has only floating point roundoff.
	sketchtest.Satisfies(t, mid, sketchtest.Within(1e-8))
	sketchtest.Satisfies(t, horizontal, sketchtest.Within(1e-8))
	sketchtest.Satisfies(t, length, sketchtest.Within(1e-8))
	sketchtest.MeasuresPoint(t, a, p["x"]-5, p["y"], sketchtest.Within(1e-8))
	sketchtest.MeasuresPoint(t, b, p["x"]+5, p["y"], sketchtest.Within(1e-8))
}
