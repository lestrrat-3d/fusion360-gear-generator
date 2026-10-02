package screwgear_test

import (
	"fmt"
	"math"
	"os"
	"runtime"
	"sort"
	"sync"
	"testing"

	"github.com/lestrrat-3d/r3"
)

// fullSampleEnv names the environment variable that selects the full sample in
// the cases that judge the mesh over many poses and in
// TestSleeveWindowsFollowTheSize. Unset or 0, each of those runs the smaller
// sample its own comment names, which is what CI runs; 1 runs the full one:
//
//	SCREWGEAR_FULL=1 proof/run.sh --package ./screwgear -- -count=1
//
// It is an environment variable rather than a test flag because go test hands
// every argument after a flag it does not know to the test binary, and run.sh
// puts the package list last, so a flag would take the package list with it.
const fullSampleEnv = "SCREWGEAR_FULL"

// fullSample reports whether SCREWGEAR_FULL asks for the full sample, and logs
// which sample the case runs, naming what the default leaves out. Any value
// but empty, 0 or 1 fails the test, so a mistyped value cannot quietly run the
// default.
func fullSample(t *testing.T, dropped string) bool {
	t.Helper()
	switch v := os.Getenv(fullSampleEnv); v {
	case "", "0":
		t.Logf("default sample: leaves out %s; %s=1 runs them", dropped, fullSampleEnv)
		return false
	case "1":
		t.Logf("full sample (%s=1)", fullSampleEnv)
		return true
	default:
		t.Fatalf("%s=%q: want 1 for the full sample, or 0 or unset for the default", fullSampleEnv, v)
		return false
	}
}

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
// The second sleeve, printed the same day at the defaults below, showed two
// more things (spec/screwgear/fusion.md [SCREW-F-PRINT-2]). Its two -R bores,
// whose roofs the printer bridges, came out too tight to pass the ribbons,
// which is why the level bore of each gear now carries a roof allowance and so
// lets its ribbon tilt (tiltReach). And with the bores chiselled open, the
// teeth meshed only sometimes. A printed tooth's tip comes out rounded and
// short, which the model's exact cosine does not, so every case here that
// judges the defaults blunts both ribbons' tips by printTipLoss.
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

// printTipLoss is how far this file takes a printed tooth's tip to fall short
// of the model's crest: the crest cut flat that far down. A 0.4 mm nozzle lays
// a bead about 0.45 mm wide, and the cosine's crest is narrower than that bead
// for its last 0.19 mm; a slicer drops or rounds what one bead cannot draw, and
// an outer wall comes out a tenth or two of a millimetre off on top of that.
// 0.35 mm takes both. TestSecondPrintMeshIsMarginal records where the mesh
// stops boxing as the loss grows.
const printTipLoss = 0.35

// borePose is how far one ribbon sits from its nominal place inside its bores:
// tx along its own Ex (toward the other ribbon when positive), ty along its
// own Ey, roll about its own axis, and tilt about its own Ey through the
// crossing, which turns +Ez toward +Ex and so moves the ribbon's -R end along
// -Ex and its +R end along +Ex.
type borePose struct{ tx, ty, roll, tilt float64 }

func (q borePose) String() string {
	if q.tilt == 0 {
		return fmt.Sprintf("(toward %+.3f, side %+.3f, roll %+.2f deg)", q.tx, q.ty, q.roll*180/math.Pi)
	}
	return fmt.Sprintf("(toward %+.3f, side %+.3f, roll %+.2f deg, tilt %+.3f deg)",
		q.tx, q.ty, q.roll*180/math.Pi, q.tilt*180/math.Pi)
}

// movedInBores is the ribbon carried by the pose: an exact rigid motion, the
// tilt and the roll about axes through Origin, which is the ribbon's station 0
// where the axes cross.
func movedInBores(g Gear, q borePose) Gear {
	if q.tilt != 0 {
		c, sn := math.Cos(q.tilt), math.Sin(q.tilt)
		g.Ez, g.Ex = g.Ez.Scale(c).Add(g.Ex.Scale(sn)), g.Ex.Scale(c).Sub(g.Ez.Scale(sn))
	}
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
//
// Each bore's cut is walked from its lower station to its higher, -sOut to
// -sIn and sIn to sOut. Until 2026-10-02 the -R cut was walked from -sIn
// toward -sOut, a loop that never ran, so the play this file measured was the
// +R bores' alone; the -R bores turned out to allow the same moves and rolls
// to a micron, and the counts recorded before then stand.
func fitsInBores(f sleeve, g Gear, q borePose) bool {
	m := movedInBores(g, q)
	fits := true
	for _, span := range [2][2]float64{{-f.sOut, -f.sIn}, {f.sIn, f.sOut}} {
		eachEnvelopePoint(m, span[0], span[1], playStationStep, func(pt r3.Vec) {
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
	at := func(d float64) borePose { return borePose{tx: d * math.Cos(a), ty: d * math.Sin(a), roll: roll} }
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
		if fitsInBores(f, g, borePose{roll: sign * mid}) {
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
				borePose{tx: shiftLimit(f, g, 0, roll), roll: roll},
				borePose{tx: -shiftLimit(f, g, math.Pi, roll), roll: roll})
		}
		out = append(out, borePose{roll: limit})
	}
	return out
}

// limitPoses are the ribbon at its two limits along Ex and its two roll
// limits, each with no other move: the furthest it moves toward the other
// ribbon, the furthest away, and the furthest it rolls either way.
func limitPoses(f sleeve, g Gear) []borePose {
	return []borePose{
		{tx: shiftLimit(f, g, 0, 0)},
		{tx: -shiftLimit(f, g, math.Pi, 0)},
		{roll: rollLimit(f, g, -1)},
		{roll: rollLimit(f, g, 1)},
	}
}

// sidewaysPoses are the ribbon at its sideways limits, with no other move.
func sidewaysPoses(f sleeve, g Gear) []borePose {
	return []borePose{
		{ty: shiftLimit(f, g, math.Pi/2, 0)},
		{ty: -shiftLimit(f, g, -math.Pi/2, 0)},
	}
}

// playTiltStep is the tilt step tiltReach samples at.
const playTiltStep = 0.02 * math.Pi / 180

// tiltReach is what a tilt in the bores does at the crossing. At every tilt
// the bores admit, sampled every playTiltStep with the ribbon moved along Ex
// as far as the bores let it each way, it returns the pose that puts the
// crossing furthest toward the other ribbon and the one that puts it furthest
// away, and the largest tilt either way, as a tiltPlay. A tilt about Ey moves the crossing
// only by the move along Ex that goes with it, so those two poses are the
// tilt's whole effect on the engagement. The moves along Ex are found by
// scanning every 0.02 mm and bisecting each end to a micron.
func tiltReach(f sleeve, g Gear) tiltPlay {
	r := tiltPlay{toward: borePose{tx: math.Inf(-1)}, away: borePose{tx: math.Inf(1)}}
	fits := func(tilt, tx float64) bool { return fitsInBores(f, g, borePose{tx: tx, tilt: tilt}) }
	edge := func(tilt, in, out float64) float64 {
		for math.Abs(out-in) > 1e-3 {
			mid := (in + out) / 2
			if fits(tilt, mid) {
				in = mid
			} else {
				out = mid
			}
		}
		return in
	}
	for _, sign := range []float64{-1, 1} {
		for k := 0; ; k++ {
			tilt := sign * float64(k) * playTiltStep
			seed, found := 0.0, false
			for tx := -1.0; tx <= 1 && !found; tx += 0.02 {
				seed, found = tx, fits(tilt, tx)
			}
			if !found {
				break
			}
			if hi := edge(tilt, seed, seed+1); hi > r.toward.tx {
				r.toward = borePose{tx: hi, tilt: tilt}
			}
			if lo := edge(tilt, seed, seed-1); lo < r.away.tx {
				r.away = borePose{tx: lo, tilt: tilt}
			}
			if sign < 0 {
				r.lo = tilt
			} else {
				r.hi = tilt
			}
		}
	}
	return r
}

// tiltPlay is tiltReach's answer: the poses that put the crossing furthest
// toward and away from the other ribbon, and the tilt limits, lo <= 0 <= hi.
type tiltPlay struct {
	toward, away borePose
	lo, hi       float64
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

// samePose pairs each pose of A with the pose of B at the same index: both
// ribbons pushed toward each other, both pulled apart, both rolled the same
// way, both tilted to carry their crossings toward each other or apart. In
// those pairs the two moves add up, in the engagement or in the mounting
// angles' sum, rather than cancelling.
func samePose(as, bs []borePose) [][2]borePose {
	out := make([][2]borePose, 0, len(as))
	for i := range min(len(as), len(bs)) {
		out = append(out, [2]borePose{as[i], bs[i]})
	}
	return out
}

// playVerdict sorts a run's pose pairs into those that drive, the jams this
// file accepts, and the failures it does not.
//
// The accepted failure is a jam with the axes brought together by at least one
// clearance in all and neither ribbon pulled away from the other: past that
// the crests overlap too deeply to pass, which is the cost of an engagement
// deep enough that the opposite corner, both ribbons pulled apart, still boxes
// the teeth in. A jam is a pose the two bodies cannot both be in at that
// phase, so the teeth push the ribbons out of it; the user meets one only by
// pressing the ribbons together. Every other failure is a defect: a pair that
// lets the teeth pass without boxing each other is a pair that slips, and a
// jam with a ribbon pulled away, or with the axes closed by less than a
// clearance, is a jam the user will meet in ordinary handling. Until the roof
// allowance came in, both ribbons had to be pushed toward each other; a
// ribbon tilted into its roof allowance closes the axes by more than a
// clearance on its own, against the other at its roll limit and so at no move
// along Ex, and that jam is the same kind.
type playVerdict struct {
	drives, accepted  int
	refused           []playMesh
	narrowest, widest float64
	departure         float64
}

// playLimitSlack is how far under the clearance a move found by shiftLimit may
// fall: it bisects each limit to a micron from below.
const playLimitSlack = 1e-3

func judgePlay(p Params, runs []playMesh) playVerdict {
	v := playVerdict{narrowest: math.Inf(1)}
	for _, m := range runs {
		switch {
		case m.kind == playDrives:
			v.drives++
			v.narrowest = math.Min(v.narrowest, m.narrowest)
			v.widest = math.Max(v.widest, m.widest)
			v.departure = math.Max(v.departure, m.departure)
		case m.kind == playJams && m.towardA >= 0 && m.towardTotal >= p.Clearance-playLimitSlack:
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
		"%d jam with the ribbons pushed together; %d fail otherwise",
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
// In the full sample at the defaults four do, every one with gear A tilted
// into its roof allowance so that its crossing stands 0.273 mm toward gear B,
// against gear B pushed the whole 0.20 mm toward it at no roll, at 0.199 mm,
// rolled 1 degree and pushed 0.069 mm, or at its roll limit of 1.53 degrees.
// The default sample meets one of them, against gear B at 0.199 mm. With the
// tips short by printTipLoss, both ribbons pushed the whole clearance together
// no longer jams, as it did with the model's own tips.
const maxPlayJams = 4

// The pair has to drive over the play its bores allow, not only at the nominal
// pose, with tips as a print makes them. The full sample (SCREWGEAR_FULL=1)
// takes each ribbon's reach in engagement and roll, every whole degree of roll
// with the furthest move toward and away at each, and the two roll limits,
// adds the two tilts that put the crossing furthest toward and away
// (tiltReach), and runs the mesh at every pair of those poses with both
// ribbons blunted by printTipLoss: 10 poses a ribbon, 100 pose pairs.
//
// The default sample takes six poses a ribbon, the furthest move toward and
// away at no roll, the two roll limits and the two tilts, and runs each pose
// of A against the same pose of B (samePose): 6 pose pairs. It leaves out the
// whole degrees of roll short of the limits, and every pair of two different
// poses, among them three of the four jams maxPlayJams counts. It keeps the
// nominal pose, every kind of move this case makes, and the two pairs at the
// edges of the full sample's windows: both ribbons pushed together, the
// narrowest window, 0.026 mm, and both pulled apart with gear B tilted, the
// widest, 1.654 mm, with the worst departure, 0.315 mm.
//
// What the sampling gives up. It moves each ribbon along Ex, rolls it and
// tilts it about Ey, and leaves the sideways move to
// TestPairDrivesUnderSidewaysPlay, which takes it alone; diagonal moves,
// sideways moves combined with a roll, and tilts about Ex are not run here.
// The roof allowance lets a ribbon tilt about Ey and nothing else
// (TestRoofAllowanceAddsOnlyATilt). An offline run on 2026-10-02, before the
// roof allowance and with the model's own tips, sampled the diagonal moves
// too, 26 poses a ribbon and 676 pose pairs: 6 of 676 jammed, every one with
// both ribbons pushed toward each other by 0.204 mm or more in all, and none
// failed any other way. It took 3 min 22 s on 24 CPUs, too long for the
// suite.
func TestPairDrivesUnderBorePlay(t *testing.T) {
	t.Parallel()
	p := defaultParams()
	ga, gb := defaultPair()
	f := plainSleeve(ga, gb)

	full := fullSample(t, "the whole degrees of roll short of the limits and every pair of two different poses")
	ta, tb := tiltReach(f, ga), tiltReach(f, gb)
	var pairs [][2]borePose
	label := fmt.Sprintf("tips %.2f mm short", printTipLoss)
	if full {
		posesA := append(reachPoses(f, ga), ta.toward, ta.away)
		posesB := append(reachPoses(f, gb), tb.toward, tb.away)
		pairs = everyPair(posesA, posesB)
		label += fmt.Sprintf(", %d poses of A by %d of B", len(posesA), len(posesB))
	} else {
		pairs = samePose(append(limitPoses(f, ga), ta.toward, ta.away), append(limitPoses(f, gb), tb.toward, tb.away))
		label += ", each pose the same on both ribbons"
	}
	ga.Blunt, gb.Blunt = printTipLoss, printTipLoss
	// The nominal pose runs in the same batch, so that its mesh shares the
	// CPUs with the others rather than running alone before them.
	runs := meshOverPoses(ga, gb, append([][2]borePose{{}}, pairs...))
	if m := runs[0]; m.kind != playDrives {
		t.Fatalf("at the nominal pose, with tips %.2f mm short, the pair %s", printTipLoss, m)
	}
	runs = runs[1:]
	v := judgePlay(p, runs)
	logPlay(t, label, runs, v)

	for _, m := range v.refused {
		t.Errorf("with A at %s and B at %s the pair %s", m.ribbonA, m.ribbonB, m)
	}
	if v.accepted > maxPlayJams {
		t.Errorf("%d pose pairs jam with the ribbons pushed together, want at most %d",
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
// on its own, with both ribbons' tips blunted by printTipLoss. The full sample
// (SCREWGEAR_FULL=1) takes each ribbon at either sideways limit, against the
// other at its nominal pose, at either limit along Ex, and at either sideways
// limit: 16 pose pairs.
//
// The default sample takes both ribbons at the same sideways limit, and each
// ribbon at either sideways limit against the other pushed the whole way
// toward it: 6 pose pairs. It leaves out the other at its nominal pose, the
// other pulled away, and the two ribbons at opposite sideways limits. It keeps
// both ways sideways for each ribbon, and the full sample's narrowest window,
// 0.092 mm with the other pushed toward, and its widest, 1.339 mm with the
// worst departure, 0.249 mm, both ribbons at the same limit.
func TestPairDrivesUnderSidewaysPlay(t *testing.T) {
	t.Parallel()
	p := defaultParams()
	ga, gb := defaultPair()
	f := plainSleeve(ga, gb)

	full := fullSample(t, "the other ribbon at its nominal pose or pulled away, and the two at opposite sideways limits")
	sideA, sideB := sidewaysPoses(f, ga), sidewaysPoses(f, gb)
	var pairs [][2]borePose
	if full {
		others := func(g Gear) []borePose {
			return []borePose{
				{},
				{tx: shiftLimit(f, g, 0, 0)},
				{tx: -shiftLimit(f, g, math.Pi, 0)},
			}
		}
		pairs = everyPair(sideA, append(others(gb), sideB...))
		pairs = append(pairs, everyPair(others(ga), sideB)...)
	} else {
		towardA, towardB := borePose{tx: shiftLimit(f, ga, 0, 0)}, borePose{tx: shiftLimit(f, gb, 0, 0)}
		pairs = samePose(sideA, sideB)
		pairs = append(pairs, everyPair(sideA, []borePose{towardB})...)
		pairs = append(pairs, everyPair([]borePose{towardA}, sideB)...)
	}
	ga.Blunt, gb.Blunt = printTipLoss, printTipLoss
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

// The roof allowance widens one face of one bore of each gear, and the other
// bore and the level bore's floor still hold the ribbon at the clearance. So
// no move of the whole ribbon along Ex or Ey and no roll gains anything: each
// is stopped by a face the allowance does not touch. What it does let a ribbon
// do is tilt about its Ey, its level bore's end rising into the allowance
// while its other bore holds, which moves the crossing along Ex. This holds
// the first, to a micron and to 1e-4 rad, against the same sleeve with no
// allowance, and measures the second: at the defaults gear A's crossing goes
// 0.273 mm toward gear B and gear B's 0.273 mm away from gear A, against
// 0.200 mm without the allowance, at a tilt of 0.60 degrees. The tilt is
// capped by the other bore rather than by the allowance, so a roof left with
// far more room, such as one chiselled open, lets the crossing go no further.
func TestRoofAllowanceAddsOnlyATilt(t *testing.T) {
	t.Parallel()
	p := defaultParams()
	ga, gb := defaultPair()
	f := plainSleeve(ga, gb)
	q := p
	q.RoofAllowance = 0
	qa, qb := pair(q, q.Sigma(), 0, assemblyPhase)
	fq := plainSleeve(qa, qb)

	for i, gs := range [][2]Gear{{ga, qa}, {gb, qb}} {
		g, h := gs[0], gs[1]
		if got := levelBore(g); got != -1 {
			t.Errorf("gear %c's roof allowance is on its %+.0fR bore, want the -R bore", 'A'+i, got)
		}
		for _, a := range []float64{0, math.Pi, math.Pi / 2, -math.Pi / 2} {
			if got, want := shiftLimit(f, g, a, 0), shiftLimit(fq, h, a, 0); math.Abs(got-want) > 1e-3 {
				t.Errorf("gear %c moves %.4f mm along %.0f degrees with the allowance and %.4f without",
					'A'+i, got, a*180/math.Pi, want)
			}
		}
		for _, sign := range []float64{-1, 1} {
			if got, want := rollLimit(f, g, sign), rollLimit(fq, h, sign); math.Abs(got-want) > 1e-4 {
				t.Errorf("gear %c rolls %.4f deg with the allowance and %.4f without",
					'A'+i, got*180/math.Pi, want*180/math.Pi)
			}
		}
		with, without := tiltReach(f, g), tiltReach(fq, h)
		if without.toward.tx > p.Clearance+1e-3 || -without.away.tx > p.Clearance+1e-3 {
			t.Errorf("without the allowance gear %c's crossing reaches %+.3f and %+.3f mm, past the clearance",
				'A'+i, without.toward.tx, without.away.tx)
		}
		gain := math.Max(with.toward.tx-without.toward.tx, without.away.tx-with.away.tx)
		if gain > p.RoofAllowance/2 {
			t.Errorf("gear %c's tilt moves its crossing %.3f mm further, more than half the %.2f mm allowance",
				'A'+i, gain, p.RoofAllowance)
		}
		t.Logf("gear %c: moves and rolls as without the allowance; tilts %.2f to %+.2f deg against %.2f to %+.2f "+
			"without, and its crossing reaches %+.3f to %+.3f mm (%s and %s) against %+.3f to %+.3f",
			'A'+i, with.lo*180/math.Pi, with.hi*180/math.Pi, without.lo*180/math.Pi, without.hi*180/math.Pi,
			with.away.tx, with.toward.tx, with.away, with.toward, without.away.tx, without.toward.tx)
	}
}

// printedFit is the defaults the ribbons and the first sleeve were printed at,
// before 2026-10-02: the same ribbon, a 0.45 mm clearance and no roof
// allowance, a 0.75 mm engagement and 15 degrees on both mounting angles, gear
// B built at -1.31 mm.
func printedFit() (Params, Gear, Gear) {
	p := defaultParams()
	p.Clearance = 0.45
	p.RoofAllowance = 0
	p.Engagement = 0.75
	p.MountAngleA = 15 * math.Pi / 180
	p.MountAngleB = 15 * math.Pi / 180
	ga, gb := pair(p, p.Sigma(), 0, -1.31)
	return p, ga, gb
}

// The check above has to fail the arrangement that failed in print, or it is
// not the check that print asked for. At the printed values the nominal pose
// drives, which is all the proof used to ask, and the play breaks it. This
// takes each ribbon at its two limits along Ex and its two roll limits
// (limitPoses) and asks for at least one failure the check does not accept.
// The tips here are the model's own: the first print failed without any tip
// loss. The scratch study's full reach at these values failed 128 of 324.
//
// The full sample (SCREWGEAR_FULL=1) runs every pair of those poses, sixteen.
// Six of the sixteen let the teeth pass without boxing each other. Two more
// jam with one ribbon pushed the whole 0.45 mm toward the other and the other
// at its roll limit; the check refused those until the roof allowance came in
// and accepts them now, since they close the axes by a clearance with neither
// ribbon pulled away (playVerdict). A third, both ribbons pushed together,
// jams and has always been accepted.
//
// The default sample runs each pose of A against the same pose of B
// (samePose): 4 pose pairs. Two of them let the teeth pass, both ribbons
// pulled 0.45 mm apart and both rolled +3.46 degrees, so the default still
// fails the print, once on a move along Ex and once on a roll. It leaves out
// every pair of two different poses, among them the other four that let the
// teeth pass.
func TestPrintedFitFailsUnderBorePlay(t *testing.T) {
	t.Parallel()
	p, ga, gb := printedFit()
	f := plainSleeve(ga, gb)

	posesA, posesB := limitPoses(f, ga), limitPoses(f, gb)
	pairs := samePose(posesA, posesB)
	if fullSample(t, "every pair of two different poses") {
		pairs = everyPair(posesA, posesB)
	}
	// The nominal pose runs in the same batch, as in TestPairDrivesUnderBorePlay.
	runs := meshOverPoses(ga, gb, append([][2]borePose{{}}, pairs...))
	if m := runs[0]; m.kind != playDrives {
		t.Fatalf("at the printed values the nominal pose %s; the print's failure was the play, "+
			"not the nominal mesh", m)
	}
	runs = runs[1:]
	v := judgePlay(p, runs)
	logPlay(t, "printed values", runs, v)
	if len(v.refused) == 0 {
		t.Error("at the printed values every pose pair drives or jams with the ribbons pushed " +
			"together; the check accepts the arrangement the print showed not meshing")
	}
}

// The second sleeve was printed at the defaults' mesh, a 0.20 mm clearance
// with no roof allowance, and the ribbons printed before 2026-10-02, the same
// part the defaults describe. Its two -R bores came out too tight and were
// chiselled open, and then the teeth meshed only sometimes, even with the
// ribbons pressed together (spec/screwgear/fusion.md [SCREW-F-PRINT-2]).
//
// Over its bores as drawn the model passes TestPairDrivesUnderBorePlay's
// judgement with the tips as much as 0.40 mm short: every one of its 64 pose
// pairs drives, the free window up to 1.247 mm wide. At 0.45 mm both ribbons
// pulled the whole clearance apart let the teeth pass, and at 0.50 mm four of
// the 64 do. So the mesh holds a tip loss of 0.40 mm and not 0.45 mm, and
// printTipLoss sits 0.05 mm inside that edge. This holds the edge, at the one
// pose pair that fails first, so a change that moves it is caught.
//
// Could the proof have caught the print? Partly. The chiselled -R bores were
// no longer the bores drawn, and a ribbon in a bore cut by hand moves in ways
// no pose of the drawn bore reaches; the proof cannot model the chisel. What
// it can say is that the drawn fit has 0.05 mm of tip loss in hand past the
// 0.35 mm it takes, which is thin, and that a roof which sags into the bore
// tightens the fit rather than loosening it. Whether the teeth slip in the
// reprinted sleeve only a print settles.
//
// The case is one pose pair at two tip losses, the least that shows an edge,
// so it has no smaller sample and runs the same whatever SCREWGEAR_FULL says.
// Each loss is a subtest of its own, run in parallel: each mesh judgement
// takes 5 to 6 s of one CPU, and run one after the other they were the
// longest single-threaded stretch in the package.
func TestSecondPrintMeshIsMarginal(t *testing.T) {
	t.Parallel()
	p := defaultParams()
	p.RoofAllowance = 0
	ga, gb := pair(p, p.Sigma(), 0, assemblyPhase)
	f := plainSleeve(ga, gb)
	apartA := borePose{tx: -shiftLimit(f, ga, math.Pi, 0)}
	apartB := borePose{tx: -shiftLimit(f, gb, math.Pi, 0)}
	for _, c := range []struct {
		loss  float64
		boxes bool
	}{{0.40, true}, {0.45, false}} {
		t.Run(fmt.Sprintf("tips %.2f mm short", c.loss), func(t *testing.T) {
			t.Parallel()
			a, b := movedInBores(ga, apartA), movedInBores(gb, apartB)
			a.Blunt, b.Blunt = c.loss, c.loss
			m := meshUnderPlay(a, b)
			t.Logf("tips %.2f mm short, both ribbons pulled %.3f and %.3f mm apart: the pair %s",
				c.loss, -apartA.tx, -apartB.tx, m)
			if c.boxes && m.kind != playDrives {
				t.Errorf("with tips %.2f mm short and both ribbons pulled apart the pair %s; the second "+
					"print's fit drove there", c.loss, m)
			}
			if !c.boxes && m.kind != playLoose {
				t.Errorf("with tips %.2f mm short and both ribbons pulled apart the pair %s; the second "+
					"print's fit let the teeth pass there", c.loss, m)
			}
		})
	}
}
