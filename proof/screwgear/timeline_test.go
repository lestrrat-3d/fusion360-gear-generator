package screwgear_test

import (
	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"testing"
)

// Each declaration corresponds to one Fusion timeline entry in the step list.
// The shared builders hold the geometry; every run gets a fresh harness case.
// The PROSE component entries (00 through 04) and relocations (78 through 80)
// have no sketch or decad counterpart: these engines own documents, not Fusion
// occurrences. Their creation context and unchanged world placement need Fusion.
// The PROSE planes (06, 07, 48, 51, 54, 57, 60 and 65) are represented by exact
// mathematical frames in the adjacent sketch and solid builders. This cannot
// establish Fusion's setByOffset, setByAngle or setByDistanceOnPath placement.
// Window numbers are the spec's rounded default search outputs. The compile
// proof has no exact per-case search results for the broader 33-input regime.
// The checked harness view and the public decad Body API provide no point
// containment reading. The required roof/floor and twist probes remain Fusion
// runtime checks; the independent solid substitutes assert volume and topology.
func stepEntry05Anchorsketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	buildAnchor(t, s, p)
}

func stepEntry08GearApathssketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	buildPaths(t, s, p)
}

func stepEntry9GearAcellsectionssketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	buildCellSections(t, s, p)
}

func stepEntry10GearAcellloft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	return buildCellLoft(t, doc, p)
}

func stepEntry11GearAasidecopy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	return buildCopyCell(t, doc, p)
}

func stepEntry12GearAdoubling1copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	return buildCopyCell(t, doc, p)
}

func stepEntry13GearAdoubling1move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	return buildMoveCell(t, doc, p)
}

func stepEntry14GearAdoubling1join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	return buildJoinCell(t, doc, p)
}

func stepEntry15GearAdoubling2copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 8
	p["moveTeeth"] = 8
	p["toolTeeth"] = 8
	p["N"] = 68
	p["phase"] = 0
	return buildCopyCell(t, doc, p)
}

func stepEntry16GearAdoubling2move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 8
	p["moveTeeth"] = 8
	p["toolTeeth"] = 8
	p["N"] = 68
	p["phase"] = 0
	return buildMoveCell(t, doc, p)
}

func stepEntry17GearAdoubling2join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 8
	p["moveTeeth"] = 8
	p["toolTeeth"] = 8
	p["N"] = 68
	p["phase"] = 0
	return buildJoinCell(t, doc, p)
}

func stepEntry18GearAdoubling3copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 16
	p["moveTeeth"] = 16
	p["toolTeeth"] = 16
	p["N"] = 68
	p["phase"] = 0
	return buildCopyCell(t, doc, p)
}

func stepEntry19GearAdoubling3move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 16
	p["moveTeeth"] = 16
	p["toolTeeth"] = 16
	p["N"] = 68
	p["phase"] = 0
	return buildMoveCell(t, doc, p)
}

func stepEntry20GearAdoubling3join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 16
	p["moveTeeth"] = 16
	p["toolTeeth"] = 16
	p["N"] = 68
	p["phase"] = 0
	return buildJoinCell(t, doc, p)
}

func stepEntry21GearAdoubling4copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 32
	p["moveTeeth"] = 32
	p["toolTeeth"] = 32
	p["N"] = 68
	p["phase"] = 0
	return buildCopyCell(t, doc, p)
}

func stepEntry22GearAdoubling4move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 32
	p["moveTeeth"] = 32
	p["toolTeeth"] = 32
	p["N"] = 68
	p["phase"] = 0
	return buildMoveCell(t, doc, p)
}

func stepEntry23GearAdoubling4join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 32
	p["moveTeeth"] = 32
	p["toolTeeth"] = 32
	p["N"] = 68
	p["phase"] = 0
	return buildJoinCell(t, doc, p)
}

func stepEntry24GearAasidemove(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 64
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	return buildMoveCell(t, doc, p)
}

func stepEntry25GearAasidejoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 64
	p["moveTeeth"] = 64
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	return buildJoinCell(t, doc, p)
}

func stepEntry26r1GearAremaindersectionssketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 1
	p["moveTeeth"] = 68
	p["toolTeeth"] = 1
	p["N"] = 69
	p["phase"] = 0
	p["start"] = p["phase"] - p["N"]*p["P"]/2 + 68*p["P"]
	buildCellSections(t, s, p)
}

func stepEntry26r2GearAremainderloft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 1
	p["moveTeeth"] = 68
	p["toolTeeth"] = 1
	p["N"] = 69
	p["phase"] = 0
	p["start"] = p["phase"] - p["N"]*p["P"]/2 + 68*p["P"]
	return buildCellLoft(t, doc, p)
}

func stepEntry26r3GearAremainderjoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 68
	p["moveTeeth"] = 68
	p["toolTeeth"] = 1
	p["N"] = 69
	p["phase"] = 0
	return buildJoinCell(t, doc, p)
}

func stepEntry27GearBpathssketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	buildPaths(t, s, p)
}

func stepEntry28GearBcellsectionssketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	buildCellSections(t, s, p)
}

func stepEntry29GearBcellloft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	return buildCellLoft(t, doc, p)
}

func stepEntry30GearBasidecopy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	return buildCopyCell(t, doc, p)
}

func stepEntry31GearBdoubling1copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	return buildCopyCell(t, doc, p)
}

func stepEntry32GearBdoubling1move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	return buildMoveCell(t, doc, p)
}

func stepEntry33GearBdoubling1join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	return buildJoinCell(t, doc, p)
}

func stepEntry34GearBdoubling2copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 8
	p["moveTeeth"] = 8
	p["toolTeeth"] = 8
	p["N"] = 68
	p["phase"] = -1.31
	return buildCopyCell(t, doc, p)
}

func stepEntry35GearBdoubling2move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 8
	p["moveTeeth"] = 8
	p["toolTeeth"] = 8
	p["N"] = 68
	p["phase"] = -1.31
	return buildMoveCell(t, doc, p)
}

func stepEntry36GearBdoubling2join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 8
	p["moveTeeth"] = 8
	p["toolTeeth"] = 8
	p["N"] = 68
	p["phase"] = -1.31
	return buildJoinCell(t, doc, p)
}

func stepEntry37GearBdoubling3copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 16
	p["moveTeeth"] = 16
	p["toolTeeth"] = 16
	p["N"] = 68
	p["phase"] = -1.31
	return buildCopyCell(t, doc, p)
}

func stepEntry38GearBdoubling3move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 16
	p["moveTeeth"] = 16
	p["toolTeeth"] = 16
	p["N"] = 68
	p["phase"] = -1.31
	return buildMoveCell(t, doc, p)
}

func stepEntry39GearBdoubling3join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 16
	p["moveTeeth"] = 16
	p["toolTeeth"] = 16
	p["N"] = 68
	p["phase"] = -1.31
	return buildJoinCell(t, doc, p)
}

func stepEntry40GearBdoubling4copy(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 32
	p["moveTeeth"] = 32
	p["toolTeeth"] = 32
	p["N"] = 68
	p["phase"] = -1.31
	return buildCopyCell(t, doc, p)
}

func stepEntry41GearBdoubling4move(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 32
	p["moveTeeth"] = 32
	p["toolTeeth"] = 32
	p["N"] = 68
	p["phase"] = -1.31
	return buildMoveCell(t, doc, p)
}

func stepEntry42GearBdoubling4join(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 32
	p["moveTeeth"] = 32
	p["toolTeeth"] = 32
	p["N"] = 68
	p["phase"] = -1.31
	return buildJoinCell(t, doc, p)
}

func stepEntry43GearBasidemove(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 64
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	return buildMoveCell(t, doc, p)
}

func stepEntry44GearBasidejoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 64
	p["moveTeeth"] = 64
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	return buildJoinCell(t, doc, p)
}

func stepEntry45r1GearBremaindersectionssketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 1
	p["moveTeeth"] = 68
	p["toolTeeth"] = 1
	p["N"] = 69
	p["phase"] = -1.31
	p["start"] = p["phase"] - p["N"]*p["P"]/2 + 68*p["P"]
	buildCellSections(t, s, p)
}

func stepEntry45r2GearBremainderloft(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 1
	p["moveTeeth"] = 68
	p["toolTeeth"] = 1
	p["N"] = 69
	p["phase"] = -1.31
	p["start"] = p["phase"] - p["N"]*p["P"]/2 + 68*p["P"]
	return buildCellLoft(t, doc, p)
}

func stepEntry45r3GearBremainderjoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 68
	p["moveTeeth"] = 68
	p["toolTeeth"] = 1
	p["N"] = 69
	p["phase"] = -1.31
	return buildJoinCell(t, doc, p)
}

func stepEntry46Sleevesketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	buildSleeveSketch(t, s, p)
}

func stepEntry47Sleeveextrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	return buildSleeveExtrude(t, doc, p)
}

func stepEntry49GearAboreRsketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = -1
	buildBoreSketch(t, s, p)
}

func stepEntry50GearAboreRsweepcut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = -1
	return buildBoreCut(t, doc, p)
}

func stepEntry52GearAboreRsketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = 1
	buildBoreSketch(t, s, p)
}

func stepEntry53GearAboreRsweepcut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = 1
	return buildBoreCut(t, doc, p)
}

func stepEntry55GearBboreRsketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = -1
	buildBoreSketch(t, s, p)
}

func stepEntry56GearBboreRsweepcut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = -1
	return buildBoreCut(t, doc, p)
}

func stepEntry58GearBboreRsketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = 1
	buildBoreSketch(t, s, p)
}

func stepEntry59GearBboreRsweepcut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = 1
	return buildBoreCut(t, doc, p)
}

func stepEntry61Windowdsketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	buildWindowSketch(t, s, p)
}

func stepEntry62Windowdextrudecut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	return buildWindowCut(t, doc, p)
}

func stepEntry63Windowdsketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	buildWindowSketch(t, s, p)
}

func stepEntry64Windowdextrudecut(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	return buildWindowCut(t, doc, p)
}

func stepEntry66GearAboreRsquaremarkersketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = -1
	buildMarkerSketch(t, s, p)
}

func stepEntry67GearAboreRmarkerextrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = -1
	return buildMarkerExtrude(t, doc, p)
}

func stepEntry68GearAboreRmarkerjoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = -1
	return buildMarkerJoin(t, doc, p)
}

func stepEntry69GearAboreRcirclemarkersketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = 1
	buildMarkerSketch(t, s, p)
}

func stepEntry70GearAboreRmarkerextrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = 1
	return buildMarkerExtrude(t, doc, p)
}

func stepEntry71GearAboreRmarkerjoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = 0
	p["gear"] = 0
	p["sigma"] = 1
	return buildMarkerJoin(t, doc, p)
}

func stepEntry72GearBboreRsquaremarkersketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = -1
	buildMarkerSketch(t, s, p)
}

func stepEntry73GearBboreRmarkerextrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = -1
	return buildMarkerExtrude(t, doc, p)
}

func stepEntry74GearBboreRmarkerjoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = -1
	return buildMarkerJoin(t, doc, p)
}

func stepEntry75GearBboreRcirclemarkersketch(t testing.TB, s *sketch.Sketch, p map[string]float64) {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = 1
	buildMarkerSketch(t, s, p)
}

func stepEntry76GearBboreRmarkerextrude(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = 1
	return buildMarkerExtrude(t, doc, p)
}

func stepEntry77GearBboreRmarkerjoin(t *testing.T, doc *decad.Document, p map[string]float64) []*decad.Body {
	delete(p, "start")
	p["cell"] = 4
	p["moveTeeth"] = 4
	p["toolTeeth"] = 4
	p["N"] = 68
	p["phase"] = -1.31
	p["gear"] = 1
	p["sigma"] = 1
	return buildMarkerJoin(t, doc, p)
}
