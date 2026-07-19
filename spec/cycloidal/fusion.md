# Cycloidal drive — Fusion-API realization

The gear-specific *how*: binding Fusion-API recipes cited by anchor from `instructions.md`. Shared
conventions are in `PLAYBOOK.md` (`[PB-…]`); reuse `lib/geargen/spurgear.py`'s idioms where noted. The
generator works in internal **cm** — convert mm inputs with `misc.to_cm`. This file covers the full
drive: rotor disc(s), pinless housing, eccentric cam, output plate + pins, chamfers, sub-components.

## [CYCLOIDAL-F-ANCHOR-CHAIN] Anchor projection

The same anchor-projection recipe the spur generator uses. The sketch projects the user's Anchor
(`sketch.project(anchorEntity).item(0)`) and constrains its own **local origin** to the result with
`geometricConstraints.addCoincident(localOrigin, projected)`. The local origin is a fresh
`sketchPoints.add(Point3D.create(0,0,0))` (spur's local-origin idiom), **not** `sketch.originPoint`. All
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

The construction axis is **perpendicular to the sketch plane through the Root-circle centre `Od_d`** (the
disc's rotation axis, the pattern axis). The generator keeps each Rotor-Lobe sketch's `diskCentre` sketch
point as `self.lobeDiskCentres[d]`.

⚠️ **`setByLine` does NOT work — confirmed.** A face-less axis (`setByLine` with an `InfiniteLine3D`)
raises `RuntimeError: 3 : Environment is not supported` in the parametric design environment, and
`occurrence.activate()` does **not** fix it (tested — it still raised on `constructionAxes.add`). The axis
**must be anchored to a BRep face**, so it is created **AFTER the lobe sector is extruded**
(`[CYCLOIDAL-F-DISK-BODY]` step 1) — never before any body. Do **not** call `activate()`. Recipe
(`buildDiskAxis(capFace, d)`, called from `buildDisk(d)` after the extrude):
1. `capFace` = **a planar cap of the extrude — selected by NORMAL direction**, i.e. a face whose surface
   is `PlaneSurfaceType` **and whose normal is parallel to the sketch-plane normal** (the cap is ⟂ the
   extrude direction). Use **`extrude.startFaces.item(0)`** unconditionally (the cap in the sketch plane;
   no `endFaces` fallback). The axis direction comes from the
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
   `axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])` — direction = the cap's (vertical)
   normal, location = the disc-centre point `Od_d`. (The cap need not *contain* `Od_d`; the face supplies
   only the direction.)
3. `axis = component.constructionAxes.add(axInput)`; `axis.name = 'Disk Axis {d+1}'`;
   **`self.diskAxes[d] = axis`**.

Face-anchored axis methods (`setByPerpendicularAtPoint`, `setByCircularFace`) work on a **non-active**
component — only the line/point methods need a body face. Pass the stashed disk-centre sketch point
directly (no nearest-vertex fallback).

## [CYCLOIDAL-F-DISK-BODY] Extrude the lobe sector + circular-pattern into the disk

The Rotor Lobe sketch has **one closed profile**: the **lobe pie-sector** bounded by spoke line 1, the lobe
spline, and spoke line 2 (a closed loop `Od → valley1 → tip → valley2 → Od`, since both spokes are solid
and meet the spline's endpoints). Extrude it and pattern it ×`L` to tile the full disk.

⚠️ **Profile selection.** Do **NOT** use `sketch.profiles.item(0)` — the along-path **text labels** add
their own letter outline profiles. Select the lobe sector as the profile whose loop **contains the lobe
spline**: iterate `sketch.profiles`, and for each, scan its `profileLoops[*].profileCurves[*].sketchEntity`
for the saved `spline` object (identity match). (Equivalently `find_profile_by_curve_counts` for a single
loop of **2 lines + 1 fitted spline**, if that helper is available.)

⚠️ **`find_profile_by_curve_counts` works ONLY for NURBS/arc/line-bounded profiles.** It counts
`NurbsCurve3D` / `Arc3D` / `Line3D` per loop and treats everything else as "other". A **full circle is a
`Circle3DCurveType`** — it is NOT an arc, so it counts as "other" — and any **annulus** has its two circles
in **separate loops**. Therefore **circle-bounded profiles (annulus, plain disc, bore) MUST be selected by
`profileLoops.count` (and/or loop-identity), NEVER by `find_profile_by_curve_counts`**. This applies to the
**cam annulus** (`[CYCLOIDAL-F-CAM]`), the **housing/casing annulus** (`[CYCLOIDAL-F-RING-PINS]`), and the
**output-plate annulus** (`[CYCLOIDAL-F-OUTPUT-PINS]`).

1. **Extrude.** `ext = extrudeFeatures.createInput(sectorProfile,
   adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`; `ext.setOneSideExtent(
   adsk.fusion.DistanceExtentDefinition.create(ValueInput.createByString(discThicknessParam.name)),
   adsk.fusion.ExtentDirections.PositiveExtentDirection)`; `extrude = extrudeFeatures.add(ext)`. Name the
   body `'Cycloidal Disk {d+1}'`. (Use the `DiscThickness` param **by name** for the extent; the value is
   already cm. Features build fine non-active.)
2. **Disk Axis (now that a body face exists).** Call `self.buildDiskAxis(capFace, d)` with a planar cap face
   of this extrude → the construction axis ⟂ the sketch at `Od_d`, stashed on `self.diskAxes[d]`
   (`[CYCLOIDAL-F-DISK-AXIS]`). This MUST come after step 1 — `setByLine` (pre-body) is unsupported.
3. **Circular-pattern the EXTRUDE FEATURE ×`L` about the Disk Axis.** Pattern the **extrude feature**, not
   its body: `coll = adsk.core.ObjectCollection.create(); coll.add(extrude)` (the `ExtrudeFeature` from
   step 1); `pat = circularPatternFeatures.createInput(coll, self.diskAxes[d])`;
   **`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`**
   (`[CYCLOIDAL-F-OUTPUT-HOLES]` ⚠️); `pat.quantity = ValueInput.createByReal(L)` (`= N − 1`);
   `pat.totalAngle = ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
   `circularPatternFeatures.add(pat)`. (`createInput` accepts features **or** bodies; pass the **feature** —
   `[PB-PATTERN-BODIES]`.) The `L` sectors tile the full rotor disk.
4. **Combine ×`L` → one disk body.** ⚠️ Join ONLY disc `d`'s **own** `L` sectors — with two discs, earlier
   bodies already exist, so `bRepBodies.item(0)` is the WRONG target. Record `base =
   component.bRepBodies.count` **BEFORE** step 1's extrude; disc `d`'s sectors are then
   `bRepBodies.item(base) … item(base + L − 1)`. Join them: `target = component.bRepBodies.item(base)`;
   `tools = ObjectCollection` of `component.bRepBodies.item(i)` for `i in base+1 .. base+L−1`;
   `ci = combineFeatures.createInput(target, tools)`;
   `ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`; `combineFeatures.add(ci)`. Name the
   resulting body `'Cycloidal Disk {d+1}'` and stash **`self.diskBodies[d]`** (the output-hole cut needs it).
   (`[CYCLOIDAL-F-TWO-DISC]` restates the baseline rule.)

## [CYCLOIDAL-F-OUTPUT-HOLE] Separate output-hole sketch

A **new** sketch (`Output Hole {d+1}`, per disc, on `plane(d)`), independent of the lobe sketch so the lobe
and hole profiles never interfere. Anchor it (`[CYCLOIDAL-F-ANCHOR-CHAIN]`) and rebuild the disc centre
(`[CYCLOIDAL-F-DISK-CENTER]`) — output holes belong to the **disc**, so they are centred on
`Od_d = (s_d·E, 0)` (signed `E` per `[CYCLOIDAL-F-TWO-DISC]`; the text below shows the `d = 0` case):

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
   **`self.outputHoles[d]`** — the cut step selects its profile by identity.

Leave the sketch **visible**. The solid hole is cut through the disk next (`[CYCLOIDAL-F-OUTPUT-HOLES]`).

## [CYCLOIDAL-F-OUTPUT-HOLES] Cut the output hole through the disk + pattern ×M

After `buildOutputHoleSketch(d)` has drawn the hole (`self.outputHoles[d]`, the solid circle on the `Rop`
circle about `Od_d`) and `buildDisk(d)` has produced the combined `self.diskBodies[d]`, cut one hole and
pattern it ×`M`:

1. **Select the hole profile by identity.** ⚠️ **Not** `profiles.item(0)` — the construction circle and the
   `'Output Hole Circle'` text label add other (non-cut / letter) profiles. Pick the profile whose loop
   contains `self.outputHoles[d]` (the solid hole circle), the same identity trick the lobe sector uses.
2. **Extrude-cut through the disk.** `ci = extrudeFeatures.createInput(holeProfile,
   adsk.fusion.FeatureOperations.CutFeatureOperation)`; same extent idiom as the lobe extrude
   (`[CYCLOIDAL-F-DISK-BODY]` step 1): `ci.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(
   ValueInput.createByString(discThicknessParam.name)), adsk.fusion.ExtentDirections.PositiveExtentDirection)`
   (the `Output Hole` sketch is on the target plane `z=0`, the disk spans `[0, DiscThickness]`, so the cut
   passes through it). ⚠️ `setDistanceExtent` is a **`HoleFeatureInput`** method — do **not** use it on an
   extrude. Set **`ci.participantBodies = [self.diskBodies[d]]`** (restrict the cut to the disc); `cut =
   extrudeFeatures.add(ci)` (kept local — nothing else needs the feature after the pattern).
3. **Pattern the CUT FEATURE ×`M` about the Disk Axis.** `coll = ObjectCollection.create(); coll.add(cut)`
   (the **`ExtrudeFeature`**, not a body — same as the lobe pattern); `pat = circularPatternFeatures.
   createInput(coll, self.diskAxes[d])`; **`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.
   AdjustPatternCompute`** (see ⚠️ below); `pat.quantity = ValueInput.createByReal(M)` (`M = Output Pin
   Count`); `pat.totalAngle = ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
   `circularPatternFeatures.add(pat)`. The `M` holes orbit `Od_d` (the Disk Axis is at `Od_d`).

⚠️ **`AdjustPatternCompute` is mandatory on EVERY circular pattern here.** This is a **lone CUT** pattern
(no body-creating feature to anchor it). With the default optimized/identical "paste" compute, Fusion
copies the cut's edges instead of recomputing each instance against the body, and a patterned cut fails with
`RuntimeError: 3 … NO_TARGET_BODY … PATTERN_FEATURES_NO_PASTE_INT_EDGES` ("no pattern instances could
intersect the original body"). Setting `patternComputeOption = AdjustPatternCompute` forces a full
per-instance recompute of the cut, which succeeds. Apply this to the lobe pattern, the output-hole pattern,
the ring-pin pattern, and the output-pin pattern alike.

## [CYCLOIDAL-F-RING-PINS] Housing base + ring casing (on `O`; pinless contour)

The fixed reaction member is **ONE `Housing` body** on `O` (NOT `Od`), built as two extrusions that are then
**Combined into a single printable part**: a base annulus `1 mm` below the disc, and the ring casing
surrounding the disc stack whose inner wall is the integral rolling contour (`epitrochoid-trace.md` "Pinless
ring casing"). The two extrusions are made to **meet** (the casing reaches down to the base top — no floating
gap) and Joined. **No discrete pins.** Build `buildRingPins()` after the discs + cam:

1. **Housing plane.** `pi = constructionPlanes.createInput(); pi.setByOffset(self.plane,
   ValueInput.createByString('-1 mm')); housingPlane = constructionPlanes.add(pi)`; name `'Ring Housing
   Plane'` (`−normal`, away from the disk).
2. **Housing base — sketch `Housing Ring`** on `housingPlane`, local origin on `O`: a plain **annulus**,
   outer `addByCenterRadius(O, R − Rr + 2·E + Wall)` (the thin pinless wall = contour peak + `Wall`; dim →
   `HousingOuterDiameter`), inner `addByCenterRadius(O, R − Rr − Wall)` (dim → `HousingInnerDiameter`).
   ⚠️ Constrain each circle's centre **coincident to the local origin ONLY** (`addCoincident(circle.
   centerSketchPoint, localOrigin)`) — do **NOT** also set the centre `isFixed` (coincident-to-the-anchored-
   origin already pins it; adding `isFixed` is redundant and risks an over-constrained solver — `[PB-SHARE-
   XOR-COINCIDENT]`). Extrude the **annulus profile** (`profileLoops.count ==
   2`) `ext.setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByString(baseThicknessParam
   .name)), adsk.fusion.ExtentDirections.NegativeExtentDirection)` (away from the disk) as a New Body
   `'Housing Ring'`; stash **`self.housingRing`**. ⚠️ Away from the disk = `NegativeExtentDirection` (the
   offset plane shares `self.plane`'s normal).
3. **Drive axis at `O`.** `capFace = housingExtrude.startFaces.item(0)` (cap by NORMAL);
   `axInput.setByPerpendicularAtPoint(capFace, originPoint)`; name `'Drive Axis'`; stash **`self.driveAxis`**.
4. **Ring casing — one section, patterned ×`N`** (mirrors the disc's lobe-sector → pattern → join). The inner
   wall is the disc's **swept envelope** `contour(φ) = env(φ) + c` (`epitrochoid-trace.md` "Pinless ring
   casing") — a smooth conjugate curve, NOT a constant circle.
   - **Compute one pin-pitch of the contour (Python).** `env = {}`; for `θ` in `linspace(0, 2π, Nθ = 240)`:
     `cx, cy = E*cos θ, E*sin θ`; `phi = -θ/L`; for `t` in `linspace(0, 2π, Nt = 240)`:
     `x, y = disk_point(t, cx, cy, phi)`; `a = atan2(y, x)`; **if `a ∈ [−π/N, π/N]`**: bin `a` into
     **`nbins = 80`** bins, keep `max(r=hypot(x,y))` per bin (`binMax`). (Empty bins: per
     `epitrochoid-trace.md` — an edge uses only its hit neighbours; both-unhit edges fall back to radius `c`.) ⚠️ **Emit the contour points at bin EDGES so the FIRST
     lands exactly on `−π/N` and the LAST exactly on `+π/N`** — `nbins+1` points at `φ_i = −π/N + (2π/N)·i/nbins`
     (`i = 0 … nbins`), radius `(c + max(binMax[i−1], binMax[i]))` (single neighbour at the two ends), as
     `[(r_i·cos φ_i, r_i·sin φ_i)]` ordered by angle (all cm; `disk_point` returns cm, use as-is). **Do NOT use
     bin CENTRES `(b+0.5)/nbins`** — that insets the endpoints by half a bin, leaving an angular gap between
     every patterned sector so the ×`N` sectors don't touch and won't Join (see `epitrochoid-trace.md` "Build
     one section" ⚠️, and the spoke note below).
   - **Section sketch `Ring Casing`** on `self.plane`, local origin on `O`: **outer**
     `addByCenterRadius(O, R − Rr + 2·E + Wall)` (= contour peak + `Wall`, the thin pinless wall; matches the
     base outer so the two Join flush) — **leave SOLID, do NOT set `isConstruction`** (it forms the
     sector's outer arc; a construction circle would leave the wedge open → `profileContainingCurve` finds
     nothing) — (dim → `HousingOuterDiameter`); a **fitted spline**
     `sketchFittedSplines.add(ObjectCollection of the contour Point3D's)` (open, `isClosed` off); and **two
     radial spokes** — `sketchLines.addByTwoPoints` from each spline end (the first/last contour points, at
     `φ = ±π/N`) out to a point on the outer circle at the same angle; keep the spline handle `contour`.
   - **Extrude the SECTOR + pattern + join.** ⚠️ **Select the wedge by MINIMUM AREA among the profiles whose
     loop contains `contour` — "contains `contour`" alone is AMBIGUOUS here and picking wrong silently makes a
     plain cylinder.** Unlike the disc's lobe sector (whose root circle is *construction*, so the lobe spline
     bounds only one finite profile), this sketch's outer circle is **SOLID**, so the open `contour` spline is
     a shared edge of **TWO** closed profiles: (a) the thin **annular wedge** between `contour` (inner) and the
     outer-circle arc (outer), bounded by the two spokes — *this* is the casing section; and (b) the large
     **complement** (the rest of the disc: the entire central region inside `contour` plus the annulus over the
     other `N−1` pitches). **Both loops contain `contour`**, so a first-match "containing-curve" search (and
     `profiles.item(0)`) may return the big complement (b) — extruding it gives a near-full disc that, patterned
     ×`N` and joined, **overlaps into a solid cylinder with the scallops erased** (the reported "extruding a
     cylinder, not the contour" bug). Disambiguate by **area**: among the profiles whose loop contains
     `contour`, take the one with the **smallest** `areaProperties(LowCalculationAccuracy).area` — the wedge (a)
     is far smaller than the complement (b). Then: `ext = extrudeFeatures.createInput(sectorProfile,
     NewBodyFeatureOperation)`; extrude it **TWO-SIDED so it spans from the housing base top up to the stack
     top** (closing the `1 mm` gap so it can Join the base into one connected solid): `ext.setTwoSidesExtent(
     DistanceExtentDefinition.create(ValueInput.createByString(stackTopExpr)),  # +normal: up to stackTop` and
     `DistanceExtentDefinition.create(ValueInput.createByString('1 mm')))  # −normal: 1 mm down to the base top`.
     ⚠️ The `'1 mm'` negative side MUST match the housing plane's `'-1 mm'` offset (step 1) so the casing
     **bottom face is coincident with the base top face** → the Join below yields ONE connected solid, not two
     lumps. `sectorFeature = extrudeFeatures.add(ext)`. `coll = ObjectCollection.create();
     coll.add(sectorFeature)`; `pat = circularPatternFeatures.createInput(
     coll, self.driveAxis)`; **`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.
     AdjustPatternCompute`** (`[CYCLOIDAL-F-OUTPUT-HOLES]` ⚠️); `pat.quantity = ValueInput.createByReal(N)`
     (`N = Pin Count`); `pat.totalAngle = '360 deg'`; `pat.isSymmetric = False`; `add(pat)`. Then
     **`combineFeatures` Join** the `N` sector bodies into one casing body (collect them by a pre-extrude
     `base = component.bRepBodies.count` baseline, as the disc join does). The section ends sit at valley
     midpoints (tangential by symmetry), so the joined inner wall is smooth.
5. **Combine the housing into ONE body.** ⚠️ **Do NOT leave the base and casing as two separate bodies** — the
   housing is a single printable part. `combineFeatures` **Join** the casing body (tool) into `self.housingRing`
   (target, the base from step 2): `ci = combineFeatures.createInput(self.housingRing, ObjectCollection
   containing the casing body); ci.operation = JoinFeatureOperation; combineFeatures.add(ci)`. Because the
   casing bottom is coincident with the base top (step 4), the result is one **connected** solid. Rename it
   `self.housingRing.name = 'Housing'`; keep **`self.housingRing`** = the combined body, and set
   **`self.ringCasing = None`** (it is consumed by the Join). `buildChamfers` already skips a `None`
   `self.ringCasing`, so the combined housing is chamfered exactly once via `self.housingRing`.

## [CYCLOIDAL-F-CAM] Eccentric cam + disk center bore

The cam is the **input** — an eccentric cylinder the disk rides on (a plain journal bearing). Outer centred
on the disk centre `Od`, input-shaft bore centred on the drive axis `O`; the `E` offset between them is the
eccentricity. The disk's center bore is **enlarged** over the cam by `Bearing Clearance` (a concentric
running gap, both on `Od`) so the cam turns freely. Use **two sketches** so the bore-cut and cam profiles
stay simple. `buildCam()` runs after the per-disc loop and **before** `buildRingPins()` (the call-graph
order; `buildDiskBore(d)` runs inside the per-disc loop):

1. **Disc center bore — `buildDiskBore(d)`, sketch `Disc Bore {d+1}`** on `plane(d)` (runs per disc, inside
   the §0 loop); anchor a local origin to `O`
   (`[CYCLOIDAL-F-ANCHOR-CHAIN]`), rebuild `Od_d` (`[CYCLOIDAL-F-DISK-CENTER]`). One **solid** circle
   `addByCenterRadius(Od_d, (CenterBearingDiameter + BearingClearance)/2)`, centre coincident to `Od_d`,
   diameter
   dim `.parameter.expression = '{} + {}'.format(centerBearingParam.name, bearingClearanceParam.name)`. Cut
   it through the disc: `coll = ObjectCollection.create()`; add **every** profile in this sketch (just the
   disc); `ci = extrudeFeatures.createInput(coll, CutFeatureOperation)`;
   `ci.setOneSideExtent(DistanceExtentDefinition.create(ValueInput.createByString(discThicknessParam.name)),
   PositiveExtentDirection)`; `ci.participantBodies = [self.diskBodies[d]]`; `extrudeFeatures.add(ci)`. → a
   clean round center bore `Bearing Clearance` wider than the cam.
2. **Eccentric Cam — per-disc section sketch `Eccentric Cam {d+1}`** on `plane(d)` (in `buildCam()`'s own
   loop over `d`); anchor a local origin to `O`, rebuild `Od_d`.
   Add:
   - **cam-outer circle** `addByCenterRadius(Od, CenterBearingDiameter/2)`, **solid**, centre coincident to
     `Od`, diameter dim `.parameter.expression = CenterBearingDiameter`. Keep `camOuter`.
   - **only if `Input Shaft Diameter > 0`:** **input-bore circle** `addByCenterRadius(O,
     InputShaftDiameter/2)`, **solid**, centre coincident to the local origin `O`, diameter dim →
     `InputShaftDiameter`. Keep `inputBore`. (It lies inside `camOuter`, offset `E` from `Od`, so it splits
     the cam disc into a small **bore disc** + the **cam annulus**.)
   Extrude the **cam cross-section** as a New Body named `'Eccentric Cam {d+1}'`
   (`PositiveExtentDirection`; extent `T + g` for `d < D−1`, `T` for the last — the `[CYCLOIDAL-F-TWO-DISC]`
   "Cam" paragraph owns the extent expressions). The cross-section is the **cam
   annulus** when `Input Shaft Diameter > 0` — the **2-loop** profile (outer loop `camOuter`, inner hole-loop
   `inputBore`); select it by **`profileLoops.count == 2`**. ⚠️ **Do NOT use `find_profile_by_curve_counts`
   for the cam annulus** — a full circle is a single `Circle3DCurveType` curve (that helper counts it as
   "other", not an arc) and the annulus's two circles are in **separate loops**, so `find_profile_by_curve_counts(arcs=2)`
   raises `Could not find profile`. Same gotcha as the housing annulus (`[CYCLOIDAL-F-RING-PINS]`). When
   `Input Shaft Diameter == 0` there is no bore, so the cross-section is the **1-loop** cam disc — select the
   only profile (`profiles.item(0)`). After all `D` sections: **Join** them into one body named
   `'Eccentric Cam'` and stash **`self.cam`** (per `[CYCLOIDAL-F-TWO-DISC]`). The cam
   outer is on `Od_d`; the disc's bore (on `Od_d`, larger by `Bearing Clearance`) rides on it with the
   running gap; the input bore runs through the full cam height on `O`.

## [CYCLOIDAL-F-OUTPUT-PINS] Output pins + plate (on `O`, the output member)

The output member, mirror of the ring housing on the `+normal` side. A solid plate above the disk carries
`M` output pins that hang down through the disk's `M` output holes. All on the drive axis `O`. Build
`buildOutputPins()` after the cam:

1. **Output plate plane (`1 mm` above the TOP disc, opposite the housing).** `pi =
   component.constructionPlanes.createInput()`; `pi.setByOffset(self.plane, ValueInput.createByString(
   stackTopExpr + ' + 1 mm'))` — `stackTopExpr` is the **prefixed** stack-top string (`[CYCLOIDAL-F-TWO-DISC]`),
   so this sits `1 mm` above the top disc (= disc top for `D=1`). A positive offset is `+normal` (the disk
   side). Name `'Output Plate Plane'`.
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
     stackTopExpr + ' + 1 mm'))` (reaches **disc-0's bottom** `z=0` — the pin runs through **all** discs'
     output holes; `stackTopExpr` is the **prefixed** stack-top string).
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
- `axis = self.plane.geometry.normal` (the disc/pin axis; `self.plane` is a `ConstructionPlane`);
  `ref = self.plane.geometry.origin`.
- **First pass — collect the cap faces with their axial heights.** `capFaces = []`. For each `face` in
  `body.faces`:
  - skip unless `face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType`;
  - `n = face.geometry.normal`; skip unless `abs(n.dotProduct(axis)) > 0.999` (a **cap-normal** face — flat,
    perpendicular to the axis — not a side wall);
  - axial height `h = (face.geometry.origin − ref) · axis` (dot of the component differences with `axis`);
    append `(h, face)` to `capFaces`.
- If `capFaces` is empty: return `None`. Else `hmin = min(h)`, `hmax = max(h)` over `capFaces`.
- ⚠️ **Second pass — chamfer ONLY the two axially-EXTREME caps** (`h ≈ hmin` or `h ≈ hmax`, tolerance
  `~1e-4` cm), **NOT every cap-normal face.** `edges = adsk.core.ObjectCollection.create()`. For each
  `(h, face)` in `capFaces` with `h` at an extreme, for each `loop` in `face.loops` with **`loop.isOuter`**
  (the rim — skip inner hole loops), add every `edge` in `loop.edges`. **Why the extreme filter:** the
  combined `Housing` (base + casing in one body) has an **internal horizontal ledge** at the base/casing
  junction (`z = −1 mm`), where the base annulus extends inward past the casing's inner contour. That ledge is
  a cap-normal face too, and its **outer** loop is the **scalloped inner contour** — chamfering that complex
  edge throws `RuntimeError ... ASM_BL_CAP_COMPLEX (面取りを要求されたサイズで作成できませんでした)`. A
  uniform-thickness disc/plate/pin has exactly two cap faces (both extreme), so this filter is a no-op for
  them and only drops the housing's interior ledge.
- If `edges.count == 0`: return `None`.
- Modern chamfer API (same as `spurgear.chamferTooth`): `chamfers =
  self.getComponent().features.chamferFeatures`; `ci = chamfers.createInput2()`;
  `ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByString(
  self.parameterName(chamferSizeParamName)), False)`; `return chamfers.add(ci)`.

⚠️ Only the **outer** loop is chamfered (`loop.isOuter`), so bores / output holes / sockets stay sharp. For
the **rotor disc** the outer loop is the **lobe profile** (a small chamfer follows it; keep `Chamfer Size`
well below the lobe size so it does not fail at the sharp valleys). For the housing / plate / pins it is a
circle.

**Output-pin chamfers ride in the pattern.** `buildOutputPins` calls `_chamferCapRims(pinBody)` **after** the
socket combine and **before** the circular pattern, then adds the returned `ChamferFeature` to the pattern's
`ObjectCollection` (alongside the pin `ExtrudeFeature` and socket `CombineFeature`) — so every patterned
output pin carries its end chamfers. (Pattern of features may include chamfer features.) The **ring casing's
pins are integral bumps** — no separate chamfer; `buildChamfers` chamfers the casing's outer rim.

## [CYCLOIDAL-F-TWO-DISC] Two-disc stack (Disc Count = 2)

`D = Disc Count` (`1`|`2`, from the dropdown). The per-disc steps run in a `for d in range(D)` loop; the
stack-spanning steps (cam, ring pins, output pins, chamfers) run once. Per-disc stashes are **lists** indexed
by `d` (`self.diskBodies`, `self.diskAxes`, `self.lobeSplines`, `self.outputHoles`, `self.lobeDiskCentres`,
`self.discPlanes`); `self.lobePinCircle` stays scalar (disc 0; stashed but unused — legacy). For `D = 1` the loop runs once and
everything is identical to the single-disc build.

⚠️ **Prefix EVERY parameter in EVERY `createByString` expression** (offsets AND extents) with
`self.parameterName(PARAM_…)` — the params register under `CycloidalDrive<N>_`, so a bare `'DiscThickness'`
raises `RuntimeError: invalid expression`. Below, `nT = parameterName(PARAM_DISC_THICKNESS)`, `nG =
parameterName(PARAM_DISC_GAP)`.

**Per-disc geometry.** Let `T = DiscThickness`, `g = DiscGap`, `s_d = +1 if d == 0 else -1`.
- **Plane** `plane(d)`: `d == 0` → `self.plane`; else `pi = constructionPlanes.createInput();
  pi.setByOffset(self.plane, ValueInput.createByString('{} * ({} + {})'.format(d, nT, nG)));
  constructionPlanes.add(pi)` (**prefixed** `nT`/`nG`). Name `'Disc Plane {d+1}'`; stash `self.discPlanes[d]`.
  Disc `d` spans `[z_d, z_d + T]`, `z_d = d·(T+g)`, all extrudes `PositiveExtentDirection` from `plane(d)`.
- **Centre** `Od_d = O + s_d·E·X̂`: in `[CYCLOIDAL-F-DISK-CENTER]` place `diskCentre` at `Point3D.create(
  s_d * E, 0, 0)` (signed); keep the `Eccentricity` distance-dim expression (the dim is a magnitude — the
  sign is in the point's position + the horizontal line direction).
- **Clocking** `phi_d = d·π`: the lobe is `disk_point(t, s_d * E, 0, d * math.pi)`. ⚠️ Verified identity:
  `disk_point(t, -E, 0, π) == (-x, -y)` of `disk_point(t, +E, 0, 0)` — disc 1 is **disc 0 rotated 180° about
  `O`**, which meshes the second disc with the **same** ring pins at the opposite eccentric (rigorous for
  **even `N`**: a π-rotation about `O` maps the `N`-pin ring onto itself). Its `M` output holes are likewise
  π-rotated, landing on the same output pins (rigorous for **even `M`**). Hence the even-`N`/even-`M`
  Validity gate for `D = 2`.

⚠️ **Join only the disc's OWN sectors** (`[CYCLOIDAL-F-DISK-BODY]` / §2·A step 4). Capture `base =
component.bRepBodies.count` **before** disc `d`'s sector extrude; after the ×`L` pattern, disc `d`'s sectors
are `bRepBodies.item(base .. base+L-1)`. Join those (target = `item(base)`, tools = the rest of that range).
Using `item(0)` would wrongly fold disc 0 into disc 1.

**`stackTop` expression (PREFIXED).** `DiscCount` is a dropdown (not a param), so build the string for the
current `D` with prefixed names: `stackTopExpr = nT` for `D = 1`, `'2 * {} + {}'.format(nT, nG)` for `D = 2`.
Used by the casing/output-pin span (`stackTopExpr + ' + 1 mm'`) and the output-plate plane
(`setByOffset(self.plane, ValueInput.createByString(stackTopExpr + ' + 1 mm'))`).

**Cam — `D` sections joined** (`buildCam`, `[CYCLOIDAL-F-CAM]`). For each `d`, sketch the eccentric cam
cross-section on `plane(d)` (outer on `Od_d`, input bore on `O`) and extrude `PositiveExtentDirection` as a
New Body by **`'{} + {}'.format(nT, nG)`** for `d < D-1` (fills the inter-disc gap so adjacent sections abut)
and **`nT`** for the last (both **prefixed**). Then `combineFeatures` **Join** all section bodies into one
`'Eccentric Cam'` (target = section 0's body, tools = the rest). The `±E` sections share a large central
overlap (centres `2E` apart, radius `CenterBearingDiameter/2`), so the joined cam is one continuous solid
with the input bore through it. (`D = 1` → the single section, unchanged.)

## [CYCLOIDAL-F-SUBCOMPONENTS] Organize finished bodies into sub-components (`buildSubComponents`)

The **last** step of `generate`. Everything so far built its bodies directly in the **Cycloidal Drive
component** (`self.getComponent()`), giving a flat body list. Group them into **four child sub-components**,
one per part type, so the browser tree reads like an assembly. **Build-then-organize, not build-in-place:** the
earlier steps and `buildChamfers` need every body in the root component, and `moveToComponent` invalidates the
moved body — so this MUST run after all geometry + chamfers.

`buildSubComponents(self)`:
- `component = self.getComponent()` (the Cycloidal Drive component — `Component.occurrences` holds its
  children, exactly as `base.getOccurrence` uses `parentComponent.occurrences.addNewComponent`).
- ⚠️ **Snapshot first, move second.** Iterate `component.bRepBodies` **once** and bucket each body **by name**
  into four Python lists (do NOT move while iterating — `moveToComponent` mutates `bRepBodies` and the loop
  would skip bodies):
  - name `.startswith('Cycloidal Disk')` → **`Rotor Discs`**
  - name `== 'Housing'` → **`Housing`**
  - name `== 'Eccentric Cam'` → **`Eccentric Cam`**
  - name `== 'Output Plate'` **or** `.startswith('Output Pin')` → **`Output`**
  Name-keying (not stashed `self.*` handles or body indices) is deliberate: chamfers/combines may have
  refreshed the stashed proxies, and `moveToComponent` shifts every later index. `buildOutputPins` step 8
  guarantees every pin is named `'Output Pin k'`, and `buildRingPins`/`buildCam` leave the final bodies named
  `'Housing'`/`'Eccentric Cam'` — so every body falls into exactly one bucket.
- For each group **in a fixed order** (`Rotor Discs`, `Housing`, `Eccentric Cam`, `Output`), skip if empty,
  else: `occ = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())` (⚠️ **identity** transform —
  so the body keeps its world position; a non-identity transform would shift it), `occ.component.name =
  <group name>`, then for each `body` in the list `body.moveToComponent(occ)` (target is the **occurrence**;
  the body moves into `occ.component` at the same world position). `moveToComponent` returns the relocated
  `BRepBody` or `None` on failure — you may ignore it (the relocation already happened); do not reuse the
  pre-move reference afterward.
- No new parameters, sketches, or dims. Construction geometry (planes/axes) stays in the root Cycloidal Drive
  component; only solid bodies are relocated.
- **Final cleanup — hide construction geometry.** After the moves, call
  **`solids.hide_construction_geometry(component)`** (import from `.solids`; it recursively walks the component
  + its sub-occurrences and sets `isLightBulbOn = False` on every sketch, construction plane, and construction
  axis). ⚠️ Use the shared helper — re-implementing it is a helper-shadow rejection. This supersedes any
  per-axis `isLightBulbOn = False` (e.g. the drive axis); one call hides them all. Only solid bodies stay
  visible.
