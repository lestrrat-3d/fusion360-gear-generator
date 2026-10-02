package screwgear_test

import (
	"fmt"
	"math"
	"runtime"
	"sort"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// ---------------------------------------------------------------------------
// The mesh under the play the bores allow.
//
// TestPairDrivesOneToOne measures the mesh at one pose: each ribbon on the
// axis the spec gives it. A printed ribbon is not held there. Each bore is the
// crest rectangle plus Clearance all round, so a ribbon can move inside its two
// bores toward or away from the other ribbon, sideways, and roll about its own
// axis, and the mesh sees every one of those moves.
//
// The print of 2026-10-02 is what showed it (spec/screwgear/fusion.md
// [SCREW-F-PRINT-MESH]). At the defaults of that day, a 0.45 mm clearance, a
// 0.75 mm engagement and 15 degrees on both mounting angles, the ribbons
// screwed through their bores and the teeth did not mesh. The proof passed,
// because it measured the nominal pose and nothing else, and at that pose the
// pair drives. Over the poses the bores allowed, 128 of the 324 pose pairs a
// scratch study ran, at the resolution below with the roll-0 poses listed
// twice, jammed or let the teeth pass without boxing each other.
//
// Could the proof have caught it? Yes: the bores and the mesh were both in
// this package, and nothing joined them. This file joins them.
//
// What the moves do to the mesh. Moving a ribbon by tx along its own Ex, which
// points at the other ribbon, adds tx to the engagement. Rolling it by an angle
// about its own axis adds that angle to its mounting angle. Moving it by ty
// along its own Ey, square to its axis and to the common perpendicular, is a
// shift along both ribbons' axes, and on a helicoid a shift along the axis is a
// roll plus a change of tooth phase; the tooth phase is free, so a sideways
// move acts on the mesh as a roll of both ribbons, mostly of the other one.
// The tests move the Gear itself (movedInBores), so none of that is assumed.
// ---------------------------------------------------------------------------

// borePose is how far one ribbon sits from its nominal place inside its bores:
// tx along its own Ex (toward the other ribbon when positive), ty along its
// own Ey, and roll about its own axis.
type borePose struct{ tx, ty, roll float64 }

func (q borePose) String() string {
	return fmt.Sprintf("(toward %+.3f, side %+.3f, roll %+.2f deg)", q.tx, q.ty, q.roll*180/math.Pi)
}

// movedInBores is the ribbon carried by the pose: an exact rigid motion, the
// roll about the axis through Origin.
func movedInBores(g Gear, q borePose) Gear {
	g.Origin = g.Origin.Add(g.Ex.Scale(q.tx)).Add(g.Ey.Scale(q.ty))
	g.Mount += q.roll
	return g
}

// playStationStep is the station step the bore fit is sampled at.
const playStationStep = 0.25

// fitsInBores answers whether the ribbon, carried by the pose, still lies in
// its two bores wherever it is inside the tube's wall. The bores are f's,
// which stay where the nominal ribbon put them. The crest rectangle is what is
// tested, since some phase of the travel puts a crest at every station.
func fitsInBores(f sleeve, g Gear, q borePose) bool {
	m := movedInBores(g, q)
	fits := true
	for _, sign := range []float64{-1, 1} {
		eachEnvelopePoint(m, sign*f.sIn, sign*f.sOut, playStationStep, func(pt r3.Vec) {
			if !fits || !f.inShell(pt) {
				return
			}
			if d, _ := boreGap(g, pt); d > 0 {
				fits = false
			}
		})
		if !fits {
			return false
		}
	}
	return true
}

// shiftLimit is how far the ribbon, rolled by roll, can move along the
// direction (cos a) Ex + (sin a) Ey before a bore stops it, to a micron.
func shiftLimit(f sleeve, g Gear, a, roll float64) float64 {
	at := func(d float64) borePose { return borePose{d * math.Cos(a), d * math.Sin(a), roll} }
	if !fitsInBores(f, g, at(0)) {
		return 0
	}
	lo, hi := 0.0, 3.0
	for hi-lo > 1e-3 {
		mid := (lo + hi) / 2
		if fitsInBores(f, g, at(mid)) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return lo
}

// rollLimit is how far the ribbon, not moved otherwise, can roll the way sign
// says before a bore stops it, to 1e-4 rad.
func rollLimit(f sleeve, g Gear, sign float64) float64 {
	lo, hi := 0.0, 0.3
	for hi-lo > 1e-4 {
		mid := (lo + hi) / 2
		if fitsInBores(f, g, borePose{0, 0, sign * mid}) {
			lo = mid
		} else {
			hi = mid
		}
	}
	return sign * lo
}

// playRollStep is the roll step the reachable poses are sampled at.
const playRollStep = math.Pi / 180

// reachPoses samples the edge of the poses one ribbon's bores allow in
// engagement and roll: at every whole degree of roll short of the limit, the
// furthest the ribbon can move toward the other ribbon and away from it, and
// the two roll limits themselves. Each pose appears once; the scratch study
// this was ported from listed the roll-0 poses twice.
func reachPoses(f sleeve, g Gear) []borePose {
	var out []borePose
	for _, sign := range []float64{-1, 1} {
		limit := rollLimit(f, g, sign)
		for k := 0; float64(k)*playRollStep < math.Abs(limit); k++ {
			if k == 0 && sign > 0 {
				continue
			}
			roll := sign * float64(k) * playRollStep
			out = append(out,
				borePose{shiftLimit(f, g, 0, roll), 0, roll},
				borePose{-shiftLimit(f, g, math.Pi, roll), 0, roll})
		}
		out = append(out, borePose{0, 0, limit})
	}
	return out
}

// sidewaysPoses are the ribbon at its sideways limits, with no other move.
func sidewaysPoses(f sleeve, g Gear) []borePose {
	return []borePose{
		{0, shiftLimit(f, g, math.Pi/2, 0), 0},
		{0, -shiftLimit(f, g, -math.Pi/2, 0), 0},
	}
}

// playMeshSamples is the number of phases of A per pitch the mesh is judged at
// under play, half TestPairDrivesOneToOne's twelve, so the many pose pairs run
// in the time the suite has. The scratch study the counts below come from used
// the same six.
const playMeshSamples = 6

// How a pose pair fails to drive.
const (
	playDrives = iota
	playJams   // at some phase of A no phase of B clears it
	playLoose  // the free window reaches a pitch, so the teeth never box B in
	playSlips  // B does not advance one pitch per pitch of A
)

// playMesh is what one pose pair does over a pitch of A.
type playMesh struct {
	kind                 int
	at                   float64 // the phase of A it failed at
	narrowest, widest    float64
	winding, departure   float64
	ribbonA, ribbonB     borePose
	towardA, towardTotal float64
}

func (m playMesh) String() string {
	switch m.kind {
	case playJams:
		return fmt.Sprintf("jams at phase %.3f of A", m.at)
	case playLoose:
		return fmt.Sprintf("does not box B in at phase %.3f of A, a %.3f mm window", m.at, m.widest)
	case playSlips:
		return fmt.Sprintf("winds %.3f pitches per pitch, not 1:1", m.winding)
	}
	return fmt.Sprintf("drives: window %.3f-%.3f mm, departure %.3f mm", m.narrowest, m.widest, m.departure)
}

// meshUnderPlay judges two placed gears the way TestPairDrivesOneToOne judges
// the defaults, at playMeshSamples phases of A. Each window is searched from
// where the last one's middle would be after 1:1 advance, which is where it
// lies when the pair drives; a pair that does not drive still ends where the
// search finds it.
func meshUnderPlay(ga, gb Gear) playMesh {
	pitch := ga.P.ToothPitch
	step := pitch / playMeshSamples
	lo, hi, ok := freeWindow(ga, gb, 0, gb.Phase)
	if !ok {
		return playMesh{kind: playJams}
	}
	m := playMesh{narrowest: hi - lo, widest: hi - lo}
	if hi-lo >= pitch {
		m.kind = playLoose
		return m
	}
	start := (lo + hi) / 2
	middle := start
	centres := make([]float64, 0, playMeshSamples)
	for i := 1; i <= playMeshSamples; i++ {
		za := float64(i) * step
		lo, hi, ok := freeWindow(ga, gb, za, middle+step)
		if !ok {
			m.kind, m.at = playJams, za
			return m
		}
		m.narrowest, m.widest = math.Min(m.narrowest, hi-lo), math.Max(m.widest, hi-lo)
		if hi-lo >= pitch {
			m.kind, m.at = playLoose, za
			return m
		}
		middle = (lo + hi) / 2
		centres = append(centres, middle)
	}
	m.winding = (middle - start) / pitch
	for i, c := range centres {
		want := start + (middle-start)*float64(i+1)/playMeshSamples
		m.departure = math.Max(m.departure, math.Abs(c-want))
	}
	if math.Abs(m.winding-1) > 0.02 {
		m.kind = playSlips
	}
	return m
}

// meshOverPoses runs meshUnderPlay over every pair of a pose of A and a pose
// of B, on as many goroutines as the process has CPUs.
func meshOverPoses(ga, gb Gear, pairs [][2]borePose) []playMesh {
	out := make([]playMesh, len(pairs))
	work := make(chan int)
	var wg sync.WaitGroup
	for range runtime.GOMAXPROCS(0) {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for i := range work {
				a, b := pairs[i][0], pairs[i][1]
				m := meshUnderPlay(movedInBores(ga, a), movedInBores(gb, b))
				m.ribbonA, m.ribbonB = a, b
				m.towardA, m.towardTotal = math.Min(a.tx, b.tx), a.tx+b.tx
				out[i] = m
			}
		}()
	}
	for i := range pairs {
		work <- i
	}
	close(work)
	wg.Wait()
	return out
}

// everyPair is every pose of A against every pose of B.
func everyPair(as, bs []borePose) [][2]borePose {
	var out [][2]borePose
	for _, a := range as {
		for _, b := range bs {
			out = append(out, [2]borePose{a, b})
		}
	}
	return out
}

// playVerdict sorts a run's pose pairs into those that drive, the jams this
// file accepts, and the failures it does not.
//
// The accepted failure is a jam with both ribbons pushed toward each other and
// the axes brought together by at least one clearance: past that the crests
// overlap too deeply to pass, which is the cost of an engagement deep enough
// that the opposite corner, both ribbons pulled apart, still boxes the teeth
// in. Every other failure is a defect: a pair that lets the teeth pass without
// boxing each other is a pair that slips, and a jam that does not need both
// ribbons pushed together is a jam the user will meet in ordinary handling.
type playVerdict struct {
	drives, accepted  int
	refused           []playMesh
	narrowest, widest float64
	departure         float64
}

func judgePlay(p Params, runs []playMesh) playVerdict {
	v := playVerdict{narrowest: math.Inf(1)}
	for _, m := range runs {
		switch {
		case m.kind == playDrives:
			v.drives++
			v.narrowest = math.Min(v.narrowest, m.narrowest)
			v.widest = math.Max(v.widest, m.widest)
			v.departure = math.Max(v.departure, m.departure)
		case m.kind == playJams && m.towardA > 0 && m.towardTotal >= p.Clearance:
			v.accepted++
		default:
			v.refused = append(v.refused, m)
		}
	}
	return v
}

func logPlay(t *testing.T, label string, runs []playMesh, v playVerdict) {
	t.Helper()
	t.Logf("%s: %d of %d pose pairs drive, window %.3f-%.3f mm, worst departure %.3f mm; "+
		"%d jam with both ribbons pushed together; %d fail otherwise",
		label, v.drives, len(runs), v.narrowest, v.widest, v.departure, v.accepted, len(v.refused))
	sorted := append([]playMesh(nil), runs...)
	sort.SliceStable(sorted, func(i, j int) bool { return sorted[i].kind > sorted[j].kind })
	for _, m := range sorted {
		if m.kind == playDrives {
			break
		}
		t.Logf("  A %s, B %s: %s", m.ribbonA, m.ribbonB, m)
	}
}

// maxPlayJams is how many of TestPairDrivesUnderBorePlay's pose pairs may jam.
// The one that does at the defaults is both ribbons pushed the whole clearance
// toward each other with no roll, which the scratch study, listing the roll-0
// poses twice, counted as 4 of 100.
const maxPlayJams = 1

// The pair has to drive over the play its bores allow, not only at the nominal
// pose. This samples each ribbon's reach in engagement and roll, every whole
// degree of roll with the furthest move toward and away at each, and runs the
// mesh at every pair of those poses.
//
// What the sampling gives up. It moves each ribbon along Ex and rolls it, and
// leaves the sideways move to TestPairDrivesUnderSidewaysPlay, which takes it
// alone; diagonal moves, and sideways moves combined with a roll, are not run
// here. An offline run on 2026-10-02 sampled those too, each ribbon at every
// whole degree of roll and eight directions of move across its axis, 26 poses
// a ribbon and 676 pose pairs: 6 of 676 jammed, every one with both ribbons
// pushed toward each other by 0.204 mm or more in all, and none failed any
// other way. It took 3 min 22 s on 24 CPUs, too long for the suite; this
// case takes about 20 s there, about four CPU-minutes. Tilts of a ribbon about
// the two axes square to its own are not sampled at all.
func TestPairDrivesUnderBorePlay(t *testing.T) {
	t.Parallel()
	p := defaultParams()
	ga, gb := defaultPair()
	f := plainSleeve(ga, gb)

	if m := meshUnderPlay(ga, gb); m.kind != playDrives {
		t.Fatalf("at the nominal pose the pair %s", m)
	}

	posesA, posesB := reachPoses(f, ga), reachPoses(f, gb)
	runs := meshOverPoses(ga, gb, everyPair(posesA, posesB))
	v := judgePlay(p, runs)
	logPlay(t, fmt.Sprintf("%d poses of A by %d of B", len(posesA), len(posesB)), runs, v)

	for _, m := range v.refused {
		t.Errorf("with A at %s and B at %s the pair %s", m.ribbonA, m.ribbonB, m)
	}
	if v.accepted > maxPlayJams {
		t.Errorf("%d pose pairs jam with both ribbons pushed together, want at most %d",
			v.accepted, maxPlayJams)
	}
	for i, g := range []Gear{ga, gb} {
		t.Logf("gear %c moves %.3f mm toward and %.3f mm away, and rolls %.2f and %+.2f deg",
			'A'+i, shiftLimit(f, g, 0, 0), shiftLimit(f, g, math.Pi, 0),
			rollLimit(f, g, -1)*180/math.Pi, rollLimit(f, g, 1)*180/math.Pi)
	}
}

// A sideways move acts on the mesh as a roll of both ribbons, mostly of the
// other one (the comment at the top of this file says why), so it is checked
// on its own: each ribbon at either sideways limit, against the other at its
// nominal pose, at either limit along Ex, and at either sideways limit.
func TestPairDrivesUnderSidewaysPlay(t *testing.T) {
	t.Parallel()
	p := defaultParams()
	ga, gb := defaultPair()
	f := plainSleeve(ga, gb)

	others := func(g Gear) []borePose {
		return []borePose{
			{},
			{shiftLimit(f, g, 0, 0), 0, 0},
			{-shiftLimit(f, g, math.Pi, 0), 0, 0},
		}
	}
	sideA, sideB := sidewaysPoses(f, ga), sidewaysPoses(f, gb)
	pairs := everyPair(sideA, append(others(gb), sideB...))
	pairs = append(pairs, everyPair(others(ga), sideB)...)
	runs := meshOverPoses(ga, gb, pairs)
	v := judgePlay(p, runs)
	logPlay(t, "sideways", runs, v)
	for _, m := range v.refused {
		t.Errorf("with A at %s and B at %s the pair %s", m.ribbonA, m.ribbonB, m)
	}
	for i, q := range [][]borePose{sideA, sideB} {
		t.Logf("gear %c moves %+.3f and %+.3f mm sideways", 'A'+i, q[0].ty, q[1].ty)
	}
}

// printedFit is the defaults the ribbons and the first sleeve were printed at,
// before 2026-10-02: the same ribbon, a 0.45 mm clearance, a 0.75 mm
// engagement and 15 degrees on both mounting angles, gear B built at -1.31 mm.
func printedFit() (Params, Gear, Gear) {
	p := defaultParams()
	p.Clearance = 0.45
	p.Engagement = 0.75
	p.MountAngleA = 15 * math.Pi / 180
	p.MountAngleB = 15 * math.Pi / 180
	ga, gb := pair(p, p.Sigma(), 0, -1.31)
	return p, ga, gb
}

// The check above has to fail the arrangement that failed in print, or it is
// not the check that print asked for. At the printed values the nominal pose
// drives, which is all the proof used to ask, and the play breaks it. To keep
// the run short this takes each ribbon at its two limits along Ex and its two
// roll limits, sixteen pose pairs, and asks for at least one failure the check
// does not accept. On 2026-10-02 eight of the sixteen were such failures: six
// let the teeth pass without boxing each other and two jammed with only one
// ribbon pushed toward the other. The scratch study's full reach at these
// values failed 128 of 324.
func TestPrintedFitFailsUnderBorePlay(t *testing.T) {
	t.Parallel()
	p, ga, gb := printedFit()
	f := plainSleeve(ga, gb)

	if m := meshUnderPlay(ga, gb); m.kind != playDrives {
		t.Fatalf("at the printed values the nominal pose %s; the print's failure was the play, "+
			"not the nominal mesh", m)
	}
	limits := func(g Gear) []borePose {
		return []borePose{
			{shiftLimit(f, g, 0, 0), 0, 0},
			{-shiftLimit(f, g, math.Pi, 0), 0, 0},
			{0, 0, rollLimit(f, g, -1)},
			{0, 0, rollLimit(f, g, 1)},
		}
	}
	runs := meshOverPoses(ga, gb, everyPair(limits(ga), limits(gb)))
	v := judgePlay(p, runs)
	logPlay(t, "printed values", runs, v)
	if len(v.refused) == 0 {
		t.Error("at the printed values every pose pair drives or jams with both ribbons pushed " +
			"together; the check accepts the arrangement the print showed not meshing")
	}
}
