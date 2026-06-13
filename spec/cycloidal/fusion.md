# Cycloidal rotor — Fusion-API realization (minimal: one lobe)

The gear-specific *how*: binding Fusion-API recipes cited by anchor from `instructions.md`. Shared
conventions are in `PLAYBOOK.md` (`[PB-…]`); reuse `lib/geargen/spurgear.py`'s idioms where noted. The
generator works in internal **cm** — convert mm inputs with `misc.to_cm`. (Full-drive recipes are
archived at `.tmp/cycloidal-spec-full-archive/fusion.md`; this minimal file covers only the one-lobe
sketch.)

## [CYCLOIDAL-F-ANCHOR-CHAIN] Anchor projection

Same shape as `[SPUR-F-ANCHOR-CHAIN]`. The sketch projects the user's Anchor
(`sketch.project(anchorEntity).item(0)`) and constrains its own **local origin** to the result with
`geometricConstraints.addCoincident(localOrigin, projected)`. The local origin is a fresh
`sketchPoints.add(Point3D.create(0,0,0))` (`[SPUR-F-LOCAL-ORIGIN]`), **not** `sketch.originPoint`. All
geometry is drawn relative to that local origin, so anchoring it drags the drawing onto the user's
Anchor. Keep the sketch visible.

## [CYCLOIDAL-F-DISK-CENTER] Drive axis vs disk centre (the eccentric offset)

The anchor `O` (local origin, `[CYCLOIDAL-F-ANCHOR-CHAIN]`) is the **drive axis** — the fixed centre the
pin ring is built on. The cycloidal **disk is eccentric**: its centre `Od` is offset from `O` by the
**Eccentricity** along `+X̂`, i.e. `Od = O + (E, 0)`. In every **disk** sketch, build this offset
explicitly so `E` is a visible, driving dimension:
1. `diskCentre = sketch.sketchPoints.add(Point3D.create(E, 0, 0))`.
2. `eccLine = sketchCurves.sketchLines.addByTwoPoints(localOrigin, diskCentre)`; `eccLine.isConstruction =
   True`; `geometricConstraints.addHorizontal(eccLine)`; and a driving **distance** dimension between
   `localOrigin` and `diskCentre` (`sketchDimensions.addDistanceDimension(...)`), then **set its
   `.parameter.expression` to the registered `Eccentricity` parameter's name** (the "parameter-referenced
   dimensions" rule in `instructions.md` Sketch Discipline — NOT a numeric value). This pins `diskCentre`
   at `(E, 0)` and keeps it tied to `Eccentricity`.

Everything that belongs to the **disk** (root circle, lobe, spokes, angle dim, output holes) is centred on
**`diskCentre`**; everything **fixed** (the pin circle) is centred on **`O`**. So the disk sits
eccentrically inside the pin ring — the real meshing snapshot, with `E` shown as the offset dimension.

## [CYCLOIDAL-F-DISK-LOBE] One open, fully-constrained lobe about the disk centre

**Units.** `disk_point` returns internal **cm**; add points **as-is — never `to_cm`**. Use
`Rr_eff = Rr + c` as the roller offset. Build the lobe about **`Od = (E, 0)`**: `disk_point(t, cx=E, cy=0,
phi=0)`, valley radius `Rv = R − Rr_eff − E`. After `[CYCLOIDAL-F-DISK-CENTER]` has made `diskCentre`:

**Naming the circles (along-path text label).** Each construction circle gets a text label wrapped along
it — exactly the `spurgear.py` "Gear Profile" idiom: `t = sketch.sketchTexts.createInput2(name, height)`;
`t.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`;
`sketch.sketchTexts.add(t)`. Use `height = Rr` (the resolved pin radius — readable). The `name` is the
circle's name string below.

1. **Pin circle (construction, on `O`).** `addByCenterRadius(Point3D.create(0,0,0), R)`, `isConstruction =
   True`, centre coincident to **`localOrigin`**, diameter dim referencing **`PinCircleDiameter`** (the
   "parameter-referenced dimensions" rule, `instructions.md` Sketch Discipline), and an **along-path label
   `'Pin Circle'`**. The fixed ring — on the drive axis, **not** the disk centre.

2. **Output-pin circle (construction, on the disk centre `Od`).** `addByCenterRadius(Point3D.create(E,0,0),
   Rop)` (`Rop = Output Pin Circle Radius`), `isConstruction = True`, centre coincident to **`diskCentre`**
   (the disk centre `Od`, **NOT** `O`), diameter dim referencing **`OutputPinCircleDiameter`**, and an
   **along-path label `'Output Pin Circle'`**. The disk's output holes sit on this circle, so it is part of
   the **disk** and centred on `Od` — **concentric with the root circle, not the pin circle**. It is the
   **innermost** reference circle (`Rop < Rv`). (Same circle as the Output Hole sketch's, about `Od`.)

3. **Root circle (construction, on the disk centre `Od`).** `addByCenterRadius(Point3D.create(E,0,0), Rv)`,
   `isConstruction = True`, centre coincident to **`diskCentre`**, diameter dim referencing
   **`2 * (PinCircleRadius - PinRadius - DiskClearance - Eccentricity)`** (= `2·Rv`, per the rule), and an
   **along-path label `'Root Circle'`**. The valleys sit on it.

4. **Lobe spline (open, adaptively sampled) — NO arc.** Sample one lobe via `disk_point(t, E, 0, 0)` using
   the **adaptive (bounded-turn-angle) sampling** of `epitrochoid-trace.md` "Sampling" — **not** uniform
   `t` (uniform overshoots into rabbit-ears near the undercut limit). Add each kept `Point3D.create(x,y,0)`
   to an `ObjectCollection`, `spline = sketchCurves.sketchFittedSplines.add(coll)` (`[PB-SKETCHCURVES]`).
   **Leave open — no `isClosed`, no closing arc.** `startPt = spline.fitPoints.item(0)` (start valley
   `Od+(Rv,0)`); `endPt = spline.fitPoints.item(spline.fitPoints.count−1)` (end valley
   `Od+(Rv·cos 2π/L, −Rv·sin 2π/L)`). The first/last kept points are exactly `t=0` and `t=2π/L`.

5. **Lock the spline (full constraint).** ⚠️ Pin the **interior** fit points and let the frame pin the ends
   — do NOT fix the *whole* spline (that makes the angle dim redundant → solver error):
   - `for i in range(1, spline.fitPoints.count − 1): spline.fitPoints.item(i).isFixed = True` — locks the
     lobe **shape**.
   - Coincide each **end onto the root circle**: `addCoincident(startPt, rootCircle)` and
     `addCoincident(endPt, rootCircle)` (point-on-curve → pins each end's **radius** to `Rv`).
   The endpoint **angles** are pinned by the two spokes (below); shape by the fixes → fully constrained, no
   redundancy.

6. **Spoke line 1 + horizontal (from the disk centre).** `line1 = sketchLines.addByTwoPoints(diskCentre,
   Point3D.create(E+Rv, 0, 0))` — start **shares** `diskCentre`. `addCoincident(line1.endSketchPoint,
   startPt)` (distinct points → real coincident). `addHorizontal(line1)` — pins the start valley on the
   `+X̂` ray from the disk centre (collinear with `eccLine`).

7. **Spoke line 2 (from the disk centre).** `line2 = sketchLines.addByTwoPoints(diskCentre, Point3D.create(
   E + Rv·cos(2π/L), −Rv·sin(2π/L), 0))` — start shares `diskCentre`; `addCoincident(line2.endSketchPoint,
   endPt)`. (No horizontal.)

8. **Lobe-pitch angle dim (driving).** `angDim = addAngularDimension(line1, line2, textPoint)`, `textPoint`
   in the **minor wedge** (below the spokes at bisector `−π/L`, e.g. `Point3D.create(E + 0.4·Rv·cos(π/L),
   −0.4·Rv·sin(π/L), 0)`) so Fusion takes the minor angle (≈24°), not the reflex; set
   `angDim.parameter.expression = '360 deg / {}'.format(lobesParam.name)` (the `Lobes` param). This pins the
   end-valley angle.

After 1–8 the sketch is **fully constrained** (pin circle on `O`, ecc offset dim'd, root circle on `Od`,
interior points fixed, both valleys pinned by radius + angle). Leave **visible**; do **not** extrude. The
output hole is a **separate** sketch so its profile never shares this one.

## [CYCLOIDAL-F-DISK-AXIS] Construction axis ⟂ the sketch at the disk centre — from a BODY FACE

The construction axis is **perpendicular to the sketch plane through the Root-circle centre `Od`** (the
disk's rotation axis, the pattern axis). The generator keeps the Rotor-Lobe sketch's `diskCentre` sketch
point as `self.lobeDiskCentre`.

⚠️ **`setByLine` does NOT work — confirmed.** A face-less axis (`setByLine` with an `InfiniteLine3D`)
raises `RuntimeError: 3 : Environment is not supported` in the parametric design environment, and
`occurrence.activate()` does **not** fix it (tested — it still raised on `constructionAxes.add`). The axis
**must be anchored to a BRep face**, so it is created **AFTER the lobe sector is extruded**
(`[CYCLOIDAL-F-DISK-BODY]` step 1) — never before any body. Do **not** call `activate()`. Recipe
(`buildDiskAxis(capFace)`, called from `buildDisk` after the extrude):
1. `capFace` = **a planar cap of the extrude — selected by NORMAL direction**, i.e. a face whose surface
   is `PlaneSurfaceType` **and whose normal is parallel to the sketch-plane normal** (the cap is ⟂ the
   extrude direction). Use **`extrude.startFaces.item(0)`** (the cap in the sketch plane); only if that is
   somehow not such a face, fall back to `extrude.endFaces.item(0)`. The axis direction comes from the
   cap's normal, so **either cap yields the same vertical axis** — you only need *a cap*; its location
   through `Od` is set by the point arg in step 2.
   ⚠️ **Do NOT pick the cap by "nearest planar face to `Od`" or "the planar face whose plane contains
   `Od`."** The extruded pie-sector's **two spoke side-faces are ALSO planar and ALSO contain `Od`** — `Od`
   is the sector **apex**, a vertex/edge shared by both spokes and both caps. A proximity-/containment-based
   search can therefore select a **spoke** face, whose normal lies **in** the sketch plane → a **horizontal**
   axis → the circular pattern spins the lobes about a sideways axis and the whole disk is garbage. Choose
   the cap by **normal ∥ sketch normal**, never by distance to / containment of `Od`. Likewise, do **not**
   evaluate `getParameterAtPoint(Od)` to score faces: `Od` is a degenerate boundary vertex on every
   candidate, so that test is unreliable.
2. `axInput = component.constructionAxes.createInput()`;
   `axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentre)` — direction = the cap's (vertical)
   normal, location = the disk-centre point `Od`. (The cap need not *contain* `Od`; the face supplies only
   the direction.)
3. `axis = component.constructionAxes.add(axInput)`; `axis.name = 'Disk Axis'`; **`self.diskAxis = axis`**.

Face-anchored axis methods (`setByPerpendicularAtPoint`, `setByCircularFace`) work on a **non-active**
component — only the line/point methods need a body face. If `setByPerpendicularAtPoint` rejects the
sketch point, pass the body **vertex** at `Od` instead (search `body.vertices` for the one nearest `Od`).

## [CYCLOIDAL-F-DISK-BODY] Extrude the lobe sector + circular-pattern into the disk

The Rotor Lobe sketch has **one closed profile**: the **lobe pie-sector** bounded by spoke line 1, the lobe
spline, and spoke line 2 (a closed loop `Od → valley1 → tip → valley2 → Od`, since both spokes are solid
and meet the spline's endpoints). Extrude it and pattern it ×`L` to tile the full disk.

⚠️ **Profile selection.** Do **NOT** use `sketch.profiles.item(0)` — the along-path **text labels** add
their own letter outline profiles. Select the lobe sector as the profile whose loop **contains the lobe
spline**: iterate `sketch.profiles`, and for each, scan its `profileLoops[*].profileCurves[*].sketchEntity`
for the saved `spline` object (identity match). (Equivalently `find_profile_by_curve_counts` for a single
loop of **2 lines + 1 fitted spline**, if that helper is available.)

1. **Extrude.** `ext = extrudeFeatures.createInput(sectorProfile,
   adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`; `ext.setOneSideExtent(
   adsk.fusion.DistanceExtentDefinition.create(ValueInput.createByString(discThicknessParam.name)),
   adsk.fusion.ExtentDirections.PositiveExtentDirection)`; `extrude = extrudeFeatures.add(ext)`. Name the
   body `'Cycloidal Disk'`. (Use the `DiscThickness` param **by name** for the extent; the value is already
   cm. Features build fine non-active.)
2. **Disk Axis (now that a body face exists).** Call `self.buildDiskAxis(...)` with a planar cap face of
   this extrude → the construction axis ⟂ the sketch at `Od`, stashed on `self.diskAxis`
   (`[CYCLOIDAL-F-DISK-AXIS]`). This MUST come after step 1 — `setByLine` (pre-body) is unsupported.
3. **Circular-pattern the EXTRUDE FEATURE ×`L` about the Disk Axis.** Pattern the **extrude feature**, not
   its body: `coll = adsk.core.ObjectCollection.create(); coll.add(extrude)` (the `ExtrudeFeature` from
   step 1); `pat = circularPatternFeatures.createInput(coll, self.diskAxis)`;
   **`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`**
   (`[CYCLOIDAL-F-OUTPUT-HOLES]` ⚠️); `pat.quantity = ValueInput.createByReal(L)` (`= N − 1`);
   `pat.totalAngle = ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
   `circularPatternFeatures.add(pat)`. (`createInput` accepts features **or** bodies; pass the **feature** —
   `[PB-PATTERN-BODIES]`.) The `L` sectors tile the full rotor disk.
4. **Combine ×`L` → one disk body.** The component now holds exactly the `L` lobe-sector bodies. Join them:
   `target = component.bRepBodies.item(0)`; `tools = ObjectCollection` of `component.bRepBodies.item(i)` for
   `i in 1..count−1`; `ci = combineFeatures.createInput(target, tools)`;
   `ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`; `combineFeatures.add(ci)`. Name the
   resulting body `'Cycloidal Disk'` and stash **`self.diskBody`** (the output-hole cut needs it).

## [CYCLOIDAL-F-OUTPUT-HOLE] Separate output-hole sketch

A **new** sketch (`Output Hole`), independent of the lobe sketch so the lobe and hole profiles never
interfere. Anchor it (`[CYCLOIDAL-F-ANCHOR-CHAIN]`) and rebuild the disk centre
(`[CYCLOIDAL-F-DISK-CENTER]`) — output holes belong to the **disk**, so they are centred on `Od = (E,0)`:

1. **Output-hole circle (construction, on the disk centre).** `addByCenterRadius(Point3D.create(E,0,0),
   Rop)` (`Rop = Output Pin Circle Radius`), `isConstruction = True`, centre coincident to `diskCentre`,
   diameter dim referencing **`OutputPinCircleDiameter`** (the parameter-references rule), and an
   **along-path label `'Output Hole Circle'`** (same `sketchTexts`/`setAsAlongPath` idiom, height `Rr`).
2. **One hole (solid).** `hole = addByCenterRadius(Point3D.create(E+Rop, 0, 0), D_hole/2)` — **solid** (no
   `isConstruction`). `D_hole = D_pin + 2E` (`epitrochoid-trace.md`; cm as-is). Pin its **diameter** with a
   driving diameter dim whose `.parameter.expression` references **`OutputHoleDiameter`** (the rule). Pin
   its **position**: `addCoincident(
   hole.centerSketchPoint, outputHoleCircle)` (on the `Rop` circle) **and** a horizontal construction line
   `addByTwoPoints(diskCentre, hole.centerSketchPoint)` + `addHorizontal(...)` (hole on the `+X̂` ray from
   `Od`). → hole fully constrained. (One of `M`; the pattern comes later.) Stash the solid hole circle on
   **`self.outputHole`** — the cut step selects its profile by identity.

Leave the sketch **visible**. The solid hole is cut through the disk next (`[CYCLOIDAL-F-OUTPUT-HOLES]`).

## [CYCLOIDAL-F-OUTPUT-HOLES] Cut the output hole through the disk + pattern ×M

After `buildOutputHoleSketch` has drawn the hole (`self.outputHole`, the solid circle on the `Rop` circle
about `Od`) and `buildDisk` has produced the combined `self.diskBody`, cut one hole and pattern it ×`M`:

1. **Select the hole profile by identity.** ⚠️ **Not** `profiles.item(0)` — the construction circle and the
   `'Output Hole Circle'` text label add other (non-cut / letter) profiles. Pick the profile whose loop
   contains `self.outputHole` (the solid hole circle), the same identity trick the lobe sector uses.
2. **Extrude-cut through the disk.** `ci = extrudeFeatures.createInput(holeProfile,
   adsk.fusion.FeatureOperations.CutFeatureOperation)`; same extent idiom as the lobe extrude
   (`[CYCLOIDAL-F-DISK-BODY]` step 1): `ci.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(
   ValueInput.createByString(discThicknessParam.name)), adsk.fusion.ExtentDirections.PositiveExtentDirection)`
   (the `Output Hole` sketch is on the target plane `z=0`, the disk spans `[0, DiscThickness]`, so the cut
   passes through it). ⚠️ `setDistanceExtent` is a **`HoleFeatureInput`** method — do **not** use it on an
   extrude. Set **`ci.participantBodies = [self.diskBody]`** (restrict the cut to the disk); `cut =
   extrudeFeatures.add(ci)`. Stash `self.outputHoleCut = cut`.
3. **Pattern the CUT FEATURE ×`M` about the Disk Axis.** `coll = ObjectCollection.create(); coll.add(cut)`
   (the **`ExtrudeFeature`**, not a body — same as the lobe pattern); `pat = circularPatternFeatures.
   createInput(coll, self.diskAxis)`; **`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.
   AdjustPatternCompute`** (see ⚠️ below); `pat.quantity = ValueInput.createByReal(M)` (`M = Output Pin
   Count`); `pat.totalAngle = ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
   `circularPatternFeatures.add(pat)`. The `M` holes orbit `Od` (the Disk Axis is at `Od`).

⚠️ **`AdjustPatternCompute` is mandatory on EVERY circular pattern here.** This is a **lone CUT** pattern
(no body-creating feature to anchor it). With the default optimized/identical "paste" compute, Fusion
copies the cut's edges instead of recomputing each instance against the body, and a patterned cut fails with
`RuntimeError: 3 … NO_TARGET_BODY … PATTERN_FEATURES_NO_PASTE_INT_EDGES` ("no pattern instances could
intersect the original body"). Setting `patternComputeOption = AdjustPatternCompute` forces a full
per-instance recompute of the cut, which succeeds. Apply this to the lobe pattern, the output-hole pattern,
the ring-pin pattern, and the output-pin pattern alike.

## [CYCLOIDAL-F-RING-PINS] Ring pins + Housing Ring base (on `O`, the fixed ring)

The ring pins and their base are the **fixed** member, centred on the drive axis **`O`** (NOT `Od`). The
Housing Ring sits on its **own construction plane, `1 mm` below the disk on the side OPPOSITE the disk
extrude**, so it never fouls the disk; the ring pins are **full-length posts** that run from the bottom of
the housing all the way up to the top of the disk. Everything is built from **one** sketch. Build
`buildRingPins()` after the disk + output holes:

1. **Housing construction plane (`1 mm`, opposite the disk).** The disk extrudes `+normal`
   (`PositiveExtentDirection`, `[CYCLOIDAL-F-DISK-BODY]`), so the housing plane is on `−normal`:
   `pi = component.constructionPlanes.createInput()`; `pi.setByOffset(self.plane, ValueInput.createByString(
   '-1 mm'))`; `housingPlane = constructionPlanes.add(pi)`; name `'Ring Housing Plane'`. (A negative offset
   is measured along `−normal` = away from the disk.)
2. **One sketch on that plane.** Create sketch `Housing Ring` on `housingPlane`; anchor a local origin to
   `O` (`[CYCLOIDAL-F-ANCHOR-CHAIN]`). In it:
   - **Annulus:** two concentric **solid** circles on `O` — outer `addByCenterRadius(O, R + Rr + Wall)`,
     inner `addByCenterRadius(O, R − Rr − Wall)` — diameter dims → `HousingOuterDiameter` /
     `HousingInnerDiameter`. Keep handles `outerCircle`, `innerCircle`.
   - **Projected pin circle (construction).** `projected = sketch.project(self.lobePinCircle)` (the Rotor
     Lobe sketch's pin circle, radius `R` on `O` — stashed there). Set each returned curve
     `isConstruction = True`; keep `projPinCircle = projected.item(0)`. This links the pin placement to
     `PinCircleDiameter` through the lobe sketch.
   - **Ring pin (solid).** `pin = addByCenterRadius(Point3D.create(R, 0, 0), Rr)`; diameter dim →
     `2 * PinRadius`; pin its centre: `addCoincident(pin.centerSketchPoint, projPinCircle)` + a horizontal
     construction line `O → pin centre` with `addHorizontal`. Keep `pin`.
3. **Housing extrude — the SOLID ring, including the pin footprint, `−normal`.** Adding the pin circle on
   the annulus splits it, so the sketch now has these profiles: the **central hole** (1 loop = `innerCircle`),
   the **pin disc** (1 loop = `pin`), and the **ring-with-pin-bite** (3 loops = outer + inner + pin). The
   housing is the full solid ring **including** the pin footprint, i.e. **every profile EXCEPT the central
   hole**. Collect them: iterate `sketch.profiles`, **skip** the one that is the central hole (the profile
   with `profileLoops.count == 1` whose single loop curve is `innerCircle`, by identity), `coll.add` the
   rest (the 3-loop ring + the pin disc). `ext = extrudeFeatures.createInput(coll, NewBodyFeatureOperation)`;
   `ext.setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByString(baseThicknessParam.name)),
   adsk.fusion.ExtentDirections.NegativeExtentDirection)`; name the body `'Housing Ring'`; stash
   **`self.housingRing`**. (Including the pin disc is why the ring has no pin-shaped hole — "extrude the
   profile **including the ring pin profile**".)
   ⚠️ **Direction — AWAY from the disk is `NegativeExtentDirection`.** The disk extrudes
   `PositiveExtentDirection` (`+normal`) on `self.plane`; this offset plane inherits the same normal, so the
   housing must extrude `−normal` = **`NegativeExtentDirection`** to sit on the side opposite the disk. (Do
   **not** use Positive — that drives the housing toward / into the disk.)
4. **Drive axis at `O`.** The disk-axis recipe (`[CYCLOIDAL-F-DISK-AXIS]`) anchored at `O`:
   `capFace = housingExtrude.startFaces.item(0)` (cap **selected by NORMAL**, never a proximity search);
   `axInput.setByPerpendicularAtPoint(capFace, originPoint)` (the `Housing Ring` sketch's `O` point); name
   `'Drive Axis'`; stash **`self.driveAxis`**.
5. **Ring pin extrude — TWO-SIDED, full-length post.** ⚠️ **Select the pin DISC, not the surrounding ring.**
   The ring-pin circle `pin` bounds **two** profiles: the **pin disc** (the small disc *inside* it — 1 loop,
   `pin` as the outer boundary) and the **ring-with-pin-bite** (3 loops, where `pin` is an inner hole-loop).
   A generic "first profile whose any loop contains `pin`" returns the **ring** (the area *outside* the pin)
   — wrong. Pick the pin disc deterministically: the profile with **`profileLoops.count == 1` whose single
   loop's curve is `pin`** (by identity). Then `pinExt = extrudeFeatures.createInput(pinProfile,
   NewBodyFeatureOperation)`; extrude **both sides** with `setTwoSidesExtent(sideOne, sideTwo)` — this offset
   plane shares `self.plane`'s normal, so `+normal` is toward the disk:
   - `sideOne` (toward the disk, `+normal`/`PositiveExtentDirection`) reaches the disk top:
     `DistanceExtentDefinition.create(ValueInput.createByString('{} + 1 mm'.format(discThicknessParam.name)))`
     (disk top is `DiscThickness` above the target plane = `DiscThickness + 1 mm` above this `−1 mm` sketch);
   - `sideTwo` (away from the disk, toward / through the housing) reaches the housing's far face:
     `DistanceExtentDefinition.create(ValueInput.createByString(baseThicknessParam.name))`.
   `pinFeature = extrudeFeatures.add(pinExt)`; name the body `'Ring Pin'`; keep the `Ring Pin` **body** as
   `pinBody = pinFeature.bodies.item(0)`. The post spans `[−1mm − BaseThickness, +DiscThickness]` — flush
   with the housing bottom and the disk top, a **separate dowel** body.
6. **Combine-cut a socket hole in the housing (keep the pin).** Cut the pin's shape out of the Housing Ring
   so the dowel passes through a matching hole instead of overlapping solid material:
   `combines = component.features.combineFeatures`; `tools = ObjectCollection.create(); tools.add(pinBody)`;
   `ci = combines.createInput(self.housingRing, tools)`; `ci.operation =
   adsk.fusion.FeatureOperations.CutFeatureOperation`; **`ci.isKeepToolBodies = True`** (the pin must
   survive the cut); `combineFeature = combines.add(ci)`. → a pin-shaped through-hole in the housing, the
   `Ring Pin` body intact and seated in it.
7. **Chamfer the pin ends** (`[CYCLOIDAL-F-CHAMFERS]`): `chamferFeature = self._chamferCapRims(pinBody)`
   (the `Ring Pin` body) — `None` when `Chamfer Size == 0`.
8. **Pattern the pin, hole AND chamfer ×`N` about `self.driveAxis`.** `coll = ObjectCollection.create();
   coll.add(pinFeature)` (the pin `ExtrudeFeature`); `coll.add(combineFeature)` (the socket `CombineFeature`);
   **and `if chamferFeature: coll.add(chamferFeature)`** (the pin-end `ChamferFeature`) — patterned in the
   **same** input so each patterned pin gets its matching hole and chamfers; `pat =
   circularPatternFeatures.createInput(coll, self.driveAxis)`; **`pat.patternComputeOption =
   adsk.fusion.PatternComputeOptions.AdjustPatternCompute`** (`[CYCLOIDAL-F-OUTPUT-HOLES]` ⚠️);
   `pat.quantity = ValueInput.createByReal(N)`
   (`N = Pin Count`); `pat.totalAngle = '360 deg'`; `pat.isSymmetric = False`; `add(pat)`. → `N` full-length
   ring posts, each through a matching hole in the single Housing Ring.

## [CYCLOIDAL-F-CAM] Eccentric cam + disk center bore

The cam is the **input** — an eccentric cylinder the disk rides on (a plain journal bearing). Outer centred
on the disk centre `Od`, input-shaft bore centred on the drive axis `O`; the `E` offset between them is the
eccentricity. The disk's center bore is **enlarged** over the cam by `Bearing Clearance` (a concentric
running gap, both on `Od`) so the cam turns freely. Use **two sketches** so the bore-cut and cam profiles
stay simple. Build `buildCam()` after the ring pins:

1. **Disk center bore — sketch `Disk Bore`** on `self.plane`; anchor a local origin to `O`
   (`[CYCLOIDAL-F-ANCHOR-CHAIN]`), rebuild `Od` (`[CYCLOIDAL-F-DISK-CENTER]`). One **solid** circle
   `addByCenterRadius(Od, (CenterBearingDiameter + BearingClearance)/2)`, centre coincident to `Od`, diameter
   dim `.parameter.expression = '{} + {}'.format(centerBearingParam.name, bearingClearanceParam.name)`. Cut
   it through the disk: `coll = ObjectCollection.create()`; add **every** profile in this sketch (just the
   disc); `ci = extrudeFeatures.createInput(coll, CutFeatureOperation)`;
   `ci.setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByString(discThicknessParam.name)),
   PositiveExtentDirection)`; `ci.participantBodies = [self.diskBody]`; `extrudeFeatures.add(ci)`. → a clean
   round center bore `Bearing Clearance` wider than the cam.
2. **Eccentric Cam — sketch `Eccentric Cam`** on `self.plane`; anchor a local origin to `O`, rebuild `Od`.
   Add:
   - **cam-outer circle** `addByCenterRadius(Od, CenterBearingDiameter/2)`, **solid**, centre coincident to
     `Od`, diameter dim `.parameter.expression = CenterBearingDiameter`. Keep `camOuter`.
   - **only if `Input Shaft Diameter > 0`:** **input-bore circle** `addByCenterRadius(O,
     InputShaftDiameter/2)`, **solid**, centre coincident to the local origin `O`, diameter dim →
     `InputShaftDiameter`. Keep `inputBore`. (It lies inside `camOuter`, offset `E` from `Od`, so it splits
     the cam disc into a small **bore disc** + the **cam annulus**.)
   Extrude the **cam cross-section** by `Disc Thickness` as a New Body, name it `'Eccentric Cam'`
   (`PositiveExtentDirection` → `[0, DiscThickness]`, co-level with the disk). The cross-section is the **cam
   annulus** when `Input Shaft Diameter > 0` — the **2-loop** profile (outer loop `camOuter`, inner hole-loop
   `inputBore`); select it by `profileLoops.count == 2`. When `Input Shaft Diameter == 0` there is no bore,
   so the cross-section is the **1-loop** cam disc — select the only profile. Stash **`self.cam`**. The cam
   outer is on `Od`; the disk's bore (on `Od`, larger by `Bearing Clearance`) rides on it with the running
   gap; the input bore runs through the full cam height on `O`.

## [CYCLOIDAL-F-OUTPUT-PINS] Output pins + plate (on `O`, the output member)

The output member, mirror of the ring housing on the `+normal` side. A solid plate above the disk carries
`M` output pins that hang down through the disk's `M` output holes. All on the drive axis `O`. Build
`buildOutputPins()` after the cam:

1. **Output plate plane (`1 mm` above the disk, opposite the housing).** `pi =
   component.constructionPlanes.createInput()`; `pi.setByOffset(self.plane, ValueInput.createByString(
   '{} + 1 mm'.format(discThicknessParam.name)))` — a positive offset is `+normal` (the disk side), so this
   sits `DiscThickness + 1 mm` along `+normal`, i.e. `1 mm` above the disk top. Name `'Output Plate Plane'`.
   ⚠️ **Direction note:** this plane is **above** the disk, so on its sketch `PositiveExtentDirection`
   (`+normal`) points **away** from the disk and `Negative` points **toward** it — the *mirror* of the
   Housing-Ring plane (which is below, where away = `Negative`).
2. **One sketch `Output Plate`** on that plane; anchor a local origin to `O` (`[CYCLOIDAL-F-ANCHOR-CHAIN]`).
   Add:
   - **plate-outer circle** `addByCenterRadius(O, OutputPlateDiameter/2)`, **solid**, centre coincident to
     the local origin, diameter dim → `OutputPlateDiameter`.
   - **output-pin circle** `addByCenterRadius(O, Rop)`, **construction**, centre coincident to `O`, diameter
     dim → `OutputPinCircleDiameter`. Keep `outPinCircle`.
   - **output pin** `addByCenterRadius(Point3D.create(Rop, 0, 0), D_pin/2)`, **solid**, diameter dim
     `.parameter.expression = '{} - 2 * {}'.format(outputHoleDiameterParam.name, eccentricityParam.name)`
     (= `D_pin`); centre pinned: `addCoincident(pin.centerSketchPoint, outPinCircle)` + a horizontal
     construction line `O → pin centre` with `addHorizontal`. Keep `pin`.
3. **Output plate body — the SOLID disc including the pin footprint, away from the disk.** The pin splits the
   plate disc into a pin disc + a 2-loop plate-with-pin-bite; the plate is **all** of it. `coll =
   ObjectCollection.create()`; add **every** profile in the sketch; `ext = extrudeFeatures.createInput(coll,
   NewBodyFeatureOperation)`; `ext.setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByString(
   outputPlateThicknessParam.name)), adsk.fusion.ExtentDirections.PositiveExtentDirection)` (away from the
   disk); name the body `'Output Plate'`; stash **`self.outputPlate`**.
4. **Output pin extrude — TWO-SIDED.** Select the **pin disc** (the `profileLoops.count == 1` profile whose
   loop curve is `pin` — not any-loop-contains, which returns the surrounding plate ring). `pinExt =
   extrudeFeatures.createInput(pinProfile, NewBodyFeatureOperation)`; `setTwoSidesExtent(sideOne, sideTwo)`:
   - `sideOne` (`PositiveExtentDirection`, away from the disk, into the plate) =
     `DistanceExtentDefinition.create(ValueInput.createByString(outputPlateThicknessParam.name))`;
   - `sideTwo` (toward the disk) = `DistanceExtentDefinition.create(ValueInput.createByString(
     '{} + 1 mm'.format(discThicknessParam.name)))` (reaches the disk bottom — the pin runs the full
     output-hole height).
   `pinFeature = extrudeFeatures.add(pinExt)`; name the body `'Output Pin'`; keep `pinBody =
   pinFeature.bodies.item(0)`.
5. **Socket (combine-cut, keep the pin).** `tools = ObjectCollection.create(); tools.add(pinBody)`; `ci =
   combineFeatures.createInput(self.outputPlate, tools)`; `ci.operation = CutFeatureOperation`;
   `ci.isKeepToolBodies = True`; `combineFeature = combineFeatures.add(ci)`. → a matching hole in the plate.
6. **Chamfer the pin ends** (`[CYCLOIDAL-F-CHAMFERS]`): `chamferFeature = self._chamferCapRims(pinBody)`
   (the `Output Pin` body) — `None` when `Chamfer Size == 0`.
7. **Pattern pin, socket AND chamfer ×`M` about `self.driveAxis`.** Reuse the drive axis at `O`
   (`buildRingPins`). `coll = ObjectCollection.create(); coll.add(pinFeature); coll.add(combineFeature)`;
   **`if chamferFeature: coll.add(chamferFeature)`**; `pat = circularPatternFeatures.createInput(coll,
   self.driveAxis)`; **`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`**
   (`[CYCLOIDAL-F-OUTPUT-HOLES]` ⚠️); `pat.quantity = ValueInput.createByReal(M)` (`M = Output Pin Count`); `pat.totalAngle =
   '360 deg'`; `pat.isSymmetric = False`; `add(pat)`. → `M` output pins, each in a plate socket, aligned at
   angle `0` with the disk's output holes (holes on `Od`, pins on `O`, both starting at the `+X̂` point, so
   each pin sits in its hole offset by `E`).

## [CYCLOIDAL-F-CHAMFERS] Outer-rim / pin-end chamfers

One helper chamfers the **outer rim** of a disc-like body on **both** flat faces; applied to a pin body it
chamfers the pin's two **ends** (same geometry — a cap-face outer loop). Skip entirely when
`Chamfer Size == 0`.

`_chamferCapRims(self, body)` → returns the `ChamferFeature`, or `None`:
- If `self.chamferSize <= 0`: return `None`.
- `axis = self.plane.geometry.normal` (the disc/pin axis; `self.plane` is a `ConstructionPlane`).
- `edges = adsk.core.ObjectCollection.create()`. For each `face` in `body.faces`:
  - skip unless `face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType`;
  - `n = face.geometry.normal`; skip unless `abs(n.dotProduct(axis)) > 0.999` (a top/bottom **cap** face,
    not a side wall);
  - for each `loop` in `face.loops` with **`loop.isOuter`** (the rim — skip inner hole loops), add every
    `edge` in `loop.edges` to `edges`.
- If `edges.count == 0`: return `None`.
- Modern chamfer API (same as `spurgear.chamferTooth`): `chamfers =
  self.getComponent().features.chamferFeatures`; `ci = chamfers.createInput2()`;
  `ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByString(
  self.parameterName(chamferSizeParamName)), False)`; `return chamfers.add(ci)`.

⚠️ Only the **outer** loop is chamfered (`loop.isOuter`), so bores / output holes / sockets stay sharp. For
the **rotor disc** the outer loop is the **lobe profile** (a small chamfer follows it; keep `Chamfer Size`
well below the lobe size so it does not fail at the sharp valleys). For the housing / plate / pins it is a
circle.

**Pin chamfers ride in the pattern.** `buildRingPins`/`buildOutputPins` call `_chamferCapRims(pinBody)`
**after** the socket combine and **before** the circular pattern, then add the returned `ChamferFeature` to
the pattern's `ObjectCollection` (alongside the pin `ExtrudeFeature` and socket `CombineFeature`) — so every
patterned pin instance carries its end chamfers. (Pattern of features may include chamfer features.)
