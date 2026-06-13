# Cycloidal Rotor Creation Instructions (minimal: one open rotor-lobe section + reference frame)

> ⚠️ **SCOPE — incremental step 1.** This generator currently produces **only one Fusion sketch
> containing a single OPEN lobe section** of the cycloidal rotor profile, plus its reference frame (three construction circles, two spoke lines, an angle dim) and one output hole — no bodies, no pins, no
> cam, no output, no assembly. This is the deliberate minimal starting point; we grow it one verified
> piece at a time. The full prior spec (pins/cam/output/assembly + every `[PB-…]`/`[CYCLOIDAL-F-…]`
> gotcha) is archived at `.tmp/cycloidal-spec-full-archive/` and is the reference as we grow back.

The **design & geometry intent** is here; the **Fusion-API realization** is in the sidecar `fusion.md`
(cited by anchor `[CYCLOIDAL-F-…]`); the **profile math** is in `epitrochoid-trace.md` (cited by name).
Cross-gear conventions are `[PB-…]` in `PLAYBOOK.md`. Read all three together.

This generator does **not** subclass any gear class and shares no involute math. It subclasses
`base.Generator` directly and uses the generic occurrence / prefixed-user-parameter plumbing only.

## Architecture

The module `lib/geargen/cycloidal.py` defines exactly these public classes (the command wiring binds to
them by name; exported via `lib/geargen/__init__.py`):

- **`CycloidalDriveCommandInputsConfigurator`** — classmethod `configure(cls, command)` adds the dialog
  inputs (the "Exact input ids" table, in display order). No `handle_input_changed` (no conditional
  visibility).
- **`CycloidalDriveGenerator(base.Generator)`** — the generator. 1-arg constructor `(design)`
  (inherited); implements `generate(self, inputs)` and the call-graph methods below; relies on inherited
  `deleteComponent()` for error cleanup. Overrides `prefixBase()` to return `'CycloidalDrive'`.

**Generation Context: none.** The build is a single sketch; carry the few handles as locals. (No
`CycloidalDriveGenerationContext` class in this minimal step.)

**Dependencies: none.** Imports only the framework (`base`, `misc`, `utilities`, `fusion360utils`).

## Component Setup

One command invocation creates a new child component (named per `generateName`) of the user-selected
Parent Component. It draws six sketches — `Rotor Lobe` (one fully-constrained open lobe section + its
reference frame), `Output Hole` (one output hole), `Housing Ring` (the base annulus + projected pin circle
+ ring pin, on a construction plane `1 mm` below the disk), `Disk Bore` (the enlarged center bore),
`Eccentric Cam` (cam outer + input bore), and `Output Plate` (the output disc + one output pin, on a plane
`1 mm` above the disk) — and builds the **rotor disk** (with its **center bore**), **`M` output holes**, the
**Housing Ring base**, **`N` full-length ring pins**, the **eccentric input cam**, and the **output plate +
`M` output pins**. The **Anchor is the drive axis** `O`; the
**cycloidal disk is eccentric**, its centre
`Od = O + E·X̂` offset by the Eccentricity. So the parts on the fixed axes (pin circle, ring pins, Housing
Ring, the cam's input bore, **the output plate + output pins**) are built on `O`, while the **disk**
geometry (root circle, lobe, output holes, center bore) and the **cam outer** are built on `Od` — the disk
sits eccentrically inside the pin ring (`[CYCLOIDAL-F-DISK-CENTER]`). The **output pins are on `O`** but the
disk's **output holes are on `Od`** and oversized by `2E`, so the pins pass through as the disk orbits. Each
profile lives in its own sketch so they never interfere.

## Variables

User inputs in dialog order; all linear inputs are mm. Derived parameters are registered as Fusion user
parameters under the `CycloidalDrive<N>_` prefix.

- **Target Plane** — user-specified plane; the lobe sketch sits here. Any non-`ConstructionPlane`
  selection (e.g. a planar face) is normalized to a coplanar construction plane at generation time.
- **Anchor Point** — user-specified `ConstructionPoint` or `SketchPoint`; the lobe is centred on its
  in-plane projection `O`.
- **Pin Count** — integer ≥ 4 (`N`). Sets `Lobes = N − 1`. Default **16**.
- **Pin Circle Diameter** — mm > 0. `R = Pin Circle Radius` (pitch radius). Default 90 mm.
- **Pin Diameter** — mm ≥ 0. The ring-pin diameter, **`0` = auto** (derive `Rr` from the bounds-mean).
  Default **0**. If > 0, `Rr = Pin Diameter / 2` (an explicit override).
- **Eccentricity** — mm > 0 (`E`). Default 1.5 mm. (Validity: `E < R/N`; reject otherwise.)
- **Disk Clearance** — mm ≥ 0 (`c`). Backlash cut into the profile (`Rr_eff = Rr + c`). Default 0.3 mm.
- **Disc Thickness** — mm > 0. Axial thickness of the rotor disk (the lobe-sector extrude depth). Default 8 mm.
- **Center Bearing Diameter** — mm > 0. The disk **center-bore** diameter **=** the eccentric **cam outer
  diameter** (the disk rides on the cam). Centred on the disk centre `Od`. Default **30 mm**.
- **Input Shaft Diameter** — mm ≥ 0. Bore through the cam centred on the **drive axis `O`** for the input
  shaft. **`0` = no bore** (solid cam). Default **8 mm**.
- **Bearing Clearance** — mm ≥ 0 (diametral). Running clearance of the disk-bore-on-cam plain bearing: the
  disk's **center bore** is enlarged to `Center Bearing Diameter + Bearing Clearance` (the cam stays at
  `Center Bearing Diameter`), a concentric gap so the cam spins freely in the bore. Default **0.2 mm**
  (≈ 0.1 mm radial; tune to your printer).
- **Output Pin Circle Diameter** — mm > 0. `Rop = Output Pin Circle Radius`; the circle the fixed output
  pins (and the disk's output holes) are centred on. Default 50 mm. (Validity: `Rop < Rv`; reject otherwise.)
- **Output Pin Count** — integer ≥ 3 (`M`). Number of output pins/holes. Default **6**.
- **Output Pin Diameter** — mm ≥ 0. The output-pin diameter, **`0` = auto** (derive from the bounds-mean).
  Default **0**. If > 0, it is the explicit pin diameter. The output **hole** is always this pin `+ 2E`.
- **Housing Wall** — mm > 0 (`wall`). Radial wall thickness of the ring-pin base annulus (the housing
  ring extends `wall` beyond the pins on both sides of the pin circle). Default **5 mm**.
- **Base Thickness** — mm > 0. Axial thickness of the ring-pin base (Housing Ring); the ring pins run from
  its far face up to the disk top. Default **5 mm**.
- **Output Plate Thickness** — mm > 0. Axial thickness of the **output plate** (the disc carrying the `M`
  output pins, on a plane `1 mm` above the disk, opposite the housing). Default **5 mm**.
- **Chamfer Size** — mm ≥ 0. A 45° equal-distance chamfer on the **outer rim** (both flat faces) of the
  rotor disc, Housing Ring and Output Plate, and on the two **ends** of every ring pin and output pin.
  **`0` = no chamfer**. Default **0.5 mm**.
- **Parent Component** — component, default root (pre-selected); listed last.

**`Rr` and the output pin/hole auto-derive (mean of bounds) when their `…Diameter` input is `0`, else use
the override** — see `epitrochoid-trace.md` "Pin / hole sizes". Resolve: `Rr = (Pin Diameter/2)` if
`Pin Diameter > 0` else `½·(E + R·sin(π/N))`; `D_pin = Output Pin Diameter` if `Output Pin Diameter > 0`
else `Rop·sin(π/M) − E`; `D_hole = D_pin + 2E`.

Derived (Python-precomputed, internal cm): `Lobes = N − 1`; `PinCircleRadius`, `OutputPinCircleRadius`;
**`PinRadius`** (resolved per above); `Rr_eff = PinRadius + c`; `Rv = R − Rr_eff − E`;
**`OutputHoleDiameter = D_hole`** (resolved per above); and the **ring-pin base annulus** diameters
**`HousingInnerDiameter = 2·(R − PinRadius − Wall)`** and **`HousingOuterDiameter = 2·(R + PinRadius +
Wall)`** (the housing surrounds the pins by `Wall` on each side of the pin circle); and the **output-plate**
diameter **`OutputPlateDiameter = OutputPinCircleDiameter + D_pin + 2·Wall`** (= `2·Rop + D_pin + 2·Wall`,
the plate covers the output pins by `Wall`). The resolved **output-pin diameter** is `D_pin = OutputHoleDiameter − 2E`. Validity (reject with a clear message, on the
**resolved** values): `E < PinRadius < R·sin(π/N)`; `D_pin > 0`; `D_hole < 2·Rop·sin(π/M)`; `E < R/N`;
`Rop < Rv`; **and the no-undercut guard `Rr_eff < ρ_min^O`** (`epitrochoid-trace.md` "No-undercut guard"
— the *binding* eccentricity limit, ≈ 2.5 mm for defaults; computed numerically from the base-trochoid
curvature, far tighter than `R/N`). This is the check that stops the "sketch breaks at high E" failure.
Also (cam/bore): `Input Shaft Diameter < Center Bearing Diameter`; the input bore must sit inside the cam
(`E + InputShaftDiameter/2 < CenterBearingDiameter/2`); and the **enlarged** disk center bore must clear the
output holes (`(CenterBearingDiameter + BearingClearance)/2 < Rop − D_hole/2`). Reject with a clear message
otherwise.

### Exact input ids and parameter-name strings (verbatim)

| Dialog input | input id | registered user-parameter |
|---|---|---|
| Parent Component (selection) | `parentComponent` | — |
| Target Plane (selection) | `plane` | — |
| Anchor Point (selection) | `anchorPoint` | — |
| Pin Count | `pinCount` | `PinCount` |
| Pin Circle Diameter | `pinCircleDiameter` | `PinCircleDiameter` |
| Pin Diameter | `pinDiameter` | `PinDiameter` |
| Eccentricity | `eccentricity` | `Eccentricity` |
| Disk Clearance | `diskClearance` | `DiskClearance` |
| Disc Thickness | `discThickness` | `DiscThickness` |
| Output Pin Circle Diameter | `outputPinCircleDiameter` | `OutputPinCircleDiameter` |
| Output Pin Count | `outputPinCount` | `OutputPinCount` |
| Output Pin Diameter | `outputPinDiameter` | `OutputPinDiameter` |
| Center Bearing Diameter | `centerBearingDiameter` | `CenterBearingDiameter` |
| Input Shaft Diameter | `inputShaftDiameter` | `InputShaftDiameter` |
| Bearing Clearance | `bearingClearance` | `BearingClearance` |
| Housing Wall | `wall` | `Wall` |
| Base Thickness | `baseThickness` | `BaseThickness` |
| Output Plate Thickness | `outputPlateThickness` | `OutputPlateThickness` |
| Chamfer Size | `chamferSize` | `ChamferSize` |

`Pin Diameter` and `Output Pin Diameter` are `addValueInput` defaulting to **`0`** (`0` = auto-derive).
Derived parameter names (kept verbatim): `Lobes`, `PinCircleRadius`, `OutputPinCircleRadius`,
`PinRadius` (resolved), `OutputHoleDiameter` (resolved), `HousingInnerDiameter`, `HousingOuterDiameter`,
`OutputPlateDiameter`.

**Dialog display order (`configure()` adds inputs in exactly this sequence):** Target Plane, Anchor
Point, Pin Count, Pin Circle Diameter, Pin Diameter, Eccentricity, Disk Clearance, Disc Thickness, Center
Bearing Diameter, Input Shaft Diameter, Bearing Clearance, Output Pin Circle Diameter, Output Pin Count,
Output Pin Diameter, Housing Wall, Base Thickness, Output Plate Thickness, Chamfer Size, **Parent Component
(last)**. Selections first two; Parent last. All numerics are `addValueInput` (read with `get_value`);
selections via `get_selection`. `Pin Count` (`N`) and `Output Pin Count` (`M`) are integer counts —
register the parameters but **read the counts from the dialog value rounded to int** for the Python
formulas (`sin(π/N)`, `sin(π/M)`; Fusion user params are floats).

## Sketch Discipline

- **The sketch follows the user's Anchor** — project the Anchor and constrain a fresh local origin to it
  (`[CYCLOIDAL-F-ANCHOR-CHAIN]`); all geometry is drawn relative to the local origin.
- **The lobe is one OPEN fitted spline** — never set `isClosed`; **no closing arc** at this step
  (`[CYCLOIDAL-F-DISK-LOBE]`,
  `[PB-SHARE-XOR-COINCIDENT]` — share *xor* coincide, never both).
- **Dimensions are driving by default** — never `isDriven=True` (`[PB-DRIVING-DIM]`).
- **Every driving dimension is PARAMETER-REFERENCED (coded, not a numeric fluke).** Set each driving
  dimension's `.parameter.expression` to reference the registered user parameter(s) **by name** (the
  generator registers all of them and keeps handles), so the sketch is genuinely parametric and the
  references are deterministic every regen:
  | dimension | `.parameter.expression` |
  |---|---|
  | eccentricity offset (`O→Od`) | `Eccentricity` |
  | lobe-pitch angle | `360 deg / Lobes` |
  | pin-circle diameter | `PinCircleDiameter` |
  | root-circle diameter | `2 * (PinCircleRadius - PinRadius - DiskClearance - Eccentricity)` (= `2·Rv`) |
  | output-pin-circle diameter | `OutputPinCircleDiameter` |
  | output-hole diameter | `OutputHoleDiameter` |
  | ring-pin diameter | `2 * PinRadius` |
  | housing inner diameter | `HousingInnerDiameter` |
  | housing outer diameter | `HousingOuterDiameter` |
  | cam outer diameter | `CenterBearingDiameter` |
  | disk center-bore (enlarged) | `CenterBearingDiameter + BearingClearance` |
  | cam input-shaft bore | `InputShaftDiameter` |
  | output-pin-circle (plate, on O) | `OutputPinCircleDiameter` |
  | output-pin diameter | `OutputHoleDiameter - 2 * Eccentricity` (= `D_pin`) |
  | output-plate diameter | `OutputPlateDiameter` |

  ⚠️ The **lobe spline points** remain Python-computed numeric snapshots (`[PB-NUMERIC-SNAPSHOT]`) — they
  cannot be a live Fusion expression. So editing a parameter in Fusion updates the *dimensions* but does
  **not** re-cut the lobe; **regenerate** to apply a change (the normal workflow here). The dimension
  references are for parametric clarity + consistency on regen, not in-place editing of the profile.

## Method contract — call graph

```
generate(inputs)
  → processInputs(inputs)            # read selections; getOccurrence(); register params; precompute dims
  → generateName()                   # 'Cycloidal Drive (N=…):L'  (L = N−1, the reduction ratio)
  → normalize self.plane to a ConstructionPlane
  → buildLobeSketch()                # 'Rotor Lobe': pin + output-pin circles on O; ecc disk-centre Od=O+E;
                                     #   root circle + locked open lobe + 2 spokes + angle dim on Od.
                                     #   Stashes the Od sketch point on self.lobeDiskCentre.
  → buildDisk()                      # extrude the lobe pie-sector (Disc Thickness) → New Body 'Cycloidal
                                     #   Disk' → buildDiskAxis(capFace): axis ⟂ sketch at Od FROM THE BODY
                                     #   FACE (setByPerpendicularAtPoint; setByLine is unsupported) →
                                     #   circular-pattern ×L (= N−1) about self.diskAxis → Join ×L into one
                                     #   'Cycloidal Disk' body (self.diskBody)
  → buildOutputHoleSketch()          # 'Output Hole' (separate sketch): output-hole circle + 1 hole on Od
  → buildOutputHoles()               # extrude-CUT the hole through self.diskBody → circular-pattern the cut
                                     #   FEATURE ×M (= Output Pin Count) about self.diskAxis → M output holes
  → buildRingPins()                  # 'Ring Housing Plane' 1mm below disk (opposite side); one 'Housing Ring'
                                     #   sketch (annulus + projected pin circle + ring pin) → solid ring
                                     #   extruded AWAY from disk by Base Thickness → drive axis at O (cap) →
                                     #   'Ring Pin' two-sided extrude (housing bottom ↔ disk top) → combine-
                                     #   CUT the pin from the housing (keepTool) → CHAMFER pin ends → pattern
                                     #   pin + hole + chamfer ×N (= Pin Count) about the drive axis
  → buildCam()                       # 'Disk Bore' sketch: CUT disk center bore (CenterBearingDiameter +
                                     #   BearingClearance) through self.diskBody. 'Eccentric Cam' sketch: cam
                                     #   outer (CenterBearingDiameter) on Od + input bore (InputShaftDiameter)
                                     #   on O → extrude cam annulus by Disc Thickness → 'Eccentric Cam'
  → buildOutputPins()                # construction plane 1mm ABOVE disk (opposite housing); 'Output Plate'
                                     #   sketch on O (solid disc + output-pin circle Rop + 1 pin) → plate
                                     #   extruded AWAY from disk by Output Plate Thickness → 'Output Pin'
                                     #   two-sided (plate top ↔ disk bottom, through output holes) → combine-
                                     #   CUT the pin from the plate (keepTool) → CHAMFER pin ends → pattern
                                     #   pin + socket + chamfer ×M (= Output Pin Count) about self.driveAxis
  → buildChamfers()                  # if Chamfer Size > 0: chamfer the outer rim (both faces) of the rotor
                                     #   disc, Housing Ring, Output Plate (_chamferCapRims). Pin-end chamfers
                                     #   are done in buildRingPins/buildOutputPins (inside their patterns).
```

As with spur, creating the occurrence (first `addParameter` / `getOccurrence`) shifts Fusion's active
component, so **pull both selection inputs (Plane, Anchor) and Parent, and stash them on `self`, before
any parameter registration.** `generateName()` returns `'Cycloidal Drive (N={}):{}'.format(N, L)` where
`N = PinCount` (rounded to int) and `L = N − 1` (the reduction ratio) — e.g. `'Cycloidal Drive (N=16):15'`.

## Instructions

### 1: Normalize the Target Plane
If the selected plane is not a `ConstructionPlane`, make a coplanar one via
`ConstructionPlaneInput.setByOffset(selectedPlane, 0)` and use it (`[SPUR-F` parity).

### 2: Fully-constrained lobe, on the eccentric disk centre — `buildLobeSketch()`
Create a sketch named `Rotor Lobe` on the (normalized) plane. Anchor a fresh local origin `O` (the drive
axis) to the Anchor (`[CYCLOIDAL-F-ANCHOR-CHAIN]`), then make the **eccentric disk centre** `Od = O + E·X̂`
(`[CYCLOIDAL-F-DISK-CENTER]` — a point on a horizontal construction line from `O`, with a driving distance
dimension = `Eccentricity`). The lobe is built about `Od`: `disk_point(t, cx = E, cy = 0, phi = 0)`.
Per `[CYCLOIDAL-F-DISK-LOBE]`, build in order:
1. **pin circle** (radius `R`) construction, centred on **`O`** (the fixed ring), diameter dim, with an
   **along-path text label `'Pin Circle'`** wrapped on it (`spurgear`'s `sketchTexts` idiom,
   `[CYCLOIDAL-F-DISK-LOBE]`); **stash this circle on `self.lobePinCircle`** — `buildRingPins` projects it
   onto the housing plane to place the ring pins (`[CYCLOIDAL-F-RING-PINS]`);
2. **output-pin circle** (radius `Rop`) construction, centred on **`Od`** (the disk centre — concentric
   with the root circle, where the disk's output holes sit; the innermost reference circle), diameter dim,
   with an **along-path text label `'Output Pin Circle'`**;
3. **root/valley circle** (radius `Rv = R − Rr_eff − E`) construction, centred on **`Od`**, diameter dim,
   with an **along-path text label `'Root Circle'`**;
4. the **lobe** — the open lobe spline about `Od`, **adaptively sampled** by bounded turn angle (≤ ~5°,
   `epitrochoid-trace.md` "Sampling") so the fitted spline doesn't overshoot near the undercut limit
   (**not** uniform `t`); **not** `isClosed`, no arc;
5. **lock the spline**: fix every **interior** fit point (`isFixed = True`) and coincide each **endpoint
   onto the root circle** (pins the valley radii) — do **not** fix the whole spline;
6. **spoke line 1** from `Od` to the lobe's **first** point, `coincident(line1.end, spline first point)` +
   **`horizontal(line1)`** (first valley on the `+X̂` ray from `Od`);
7. **spoke line 2** from `Od` to the lobe's **last** point, `coincident(line2.end, spline last point)`;
8. a **driving angular dimension** between the spokes = one **lobe pitch** `360°/(N − 1)` via
   `360 deg / Lobes`, text point in the minor wedge.
After this the sketch is **fully constrained**. Leave it **visible**; build no bodies. **Stash the
Root-circle-centre sketch point (`diskCentre`, `Od`) on `self.lobeDiskCentre`** for the next step.

### 2·A: Extrude + axis + pattern → the rotor disk — `buildDisk()` (and `buildDiskAxis(capFace)`)
Build the full rotor from the Rotor-Lobe sketch's **one closed profile** — the **lobe pie-sector** bounded
by spoke 1 + the lobe spline + spoke 2 (`[CYCLOIDAL-F-DISK-BODY]`):
1. **Extrude** that sector profile by **`Disc Thickness`** as a **New Body** `'Cycloidal Disk'`
   (`PositiveExtentDirection`). ⚠️ Select the sector profile by the one whose loop contains the **lobe
   spline** — **not** `profiles.item(0)`, since the along-path text labels add their own letter profiles.
2. **Disk axis — `buildDiskAxis(capFace)`** (`[CYCLOIDAL-F-DISK-AXIS]`): now that a body exists, create the
   construction axis **perpendicular to the sketch at the Root-circle centre `Od`** **from the extrude's
   planar cap face** via `setByPerpendicularAtPoint(capFace, self.lobeDiskCentre)`. Name it `'Disk Axis'`,
   stash on `self.diskAxis`. ⚠️ **Pick the cap by NORMAL (∥ sketch normal) — use `extrude.startFaces.item(0)`,
   NOT a nearest-/contains-`Od` planar-face search**: the pie-sector's two **spoke faces are also planar and
   also contain the apex `Od`**, so a proximity search grabs a spoke → an **in-plane (horizontal) axis** →
   the pattern (and the whole disk) comes out garbage (`[CYCLOIDAL-F-DISK-AXIS]`). ⚠️ **Do NOT use
   `setByLine`** (face-less) and do **NOT** `activate()` — both fail/are unnecessary; a face-anchored axis
   works on the non-active component, but **needs the body**, so this comes AFTER the extrude.
3. **Circular-pattern the extrude FEATURE ×`L` (= `N − 1`)** about **`self.diskAxis`** over `360°` — pass
   the **`ExtrudeFeature`** to `circularPatternFeatures.createInput`, **not** its body. The `L` sectors tile
   the full rotor disk.
4. **Join ×`L` → one disk body** (`[CYCLOIDAL-F-DISK-BODY]` step 4): combine the `L` lobe-sector bodies into
   one with a `combineFeatures` Join (target = `component.bRepBodies.item(0)`, tools = the rest). Name the
   result `'Cycloidal Disk'` and stash **`self.diskBody`** (the output-hole cut needs it).

### 3: Output hole, separate sketch — `buildOutputHoleSketch()`
Create a **new** sketch named `Output Hole` (so the lobe and hole profiles never share a sketch). Anchor a
local origin to `O` and rebuild the eccentric disk centre `Od` (`[CYCLOIDAL-F-DISK-CENTER]`). Then, on `Od`
(`[CYCLOIDAL-F-OUTPUT-HOLE]`): an **output-hole circle** (radius `Rop`, construction, driving diameter dim)
and **one solid hole** of derived diameter `OutputHoleDiameter = D_pin + 2E`, seated on the output-hole
circle, with a **driving diameter dimension** and its centre fully pinned (on the `Rop` circle + a
horizontal spoke from `Od`). Leave the sketch **visible**; stash the solid hole circle on **`self.outputHole`**
for the cut step. No bodies here.

### 4: Cut + pattern the output holes — `buildOutputHoles()`
Using the `Output Hole` sketch's solid hole (`[CYCLOIDAL-F-OUTPUT-HOLES]`): select the profile whose loop
contains **`self.outputHole`** (not `profiles.item(0)` — the construction circle + text label add other
profiles), **extrude-cut** it through **`self.diskBody`** by `Disc Thickness` (`CutFeatureOperation`,
`participantBodies = [self.diskBody]`), then **circular-pattern that cut `ExtrudeFeature` ×`M`** (=
`Output Pin Count`) about **`self.diskAxis`** over `360°` — pass the **feature**, not a body. ⚠️ Set
**`patternComputeOption = AdjustPatternCompute`** on the pattern input — a lone patterned CUT fails with
`NO_TARGET_BODY / NO_PASTE_INT_EDGES` under the default paste compute (`[CYCLOIDAL-F-OUTPUT-HOLES]`). → `M`
output holes orbiting `Od`. Done.

### 5: Ring pins + base — `buildRingPins()`
The fixed ring: the **Housing Ring** base (an annulus on `O`) on its own construction plane `1 mm` below the
disk (opposite side), and `N` **full-length ring-pin posts** standing from the housing bottom up to the disk
top. Built from **one** sketch, all on `O` (**not** `Od`) (`[CYCLOIDAL-F-RING-PINS]`):
1. **Housing plane.** A construction plane offset `−1 mm` from the target plane — `setByOffset(self.plane,
   '-1 mm')` — i.e. `1 mm` away on the side **opposite** the disk's `+normal` extrude. Name `'Ring Housing
   Plane'`.
2. **One sketch `Housing Ring`** on that plane, anchored to `O`: the **annulus** (outer `R + Rr + Wall`,
   inner `R − Rr − Wall`, solid, diameter dims → `HousingOuterDiameter` / `HousingInnerDiameter`); the
   **projected pin circle** — `sketch.project(self.lobePinCircle)`, set **construction** — to locate the pins
   parametrically; and **one solid ring-pin circle** (radius `Rr`) seated on the projected pin circle at
   `(R, 0)` (diameter dim → `2 * PinRadius`, centre coincident to the projected circle + a horizontal spoke
   from `O`).
3. **Housing extrude — the SOLID ring including the pin footprint, AWAY from the disk.** The pin circle
   splits the annulus, so select **every profile EXCEPT the central hole** (the 1-loop profile bounded by the
   inner circle): that is the 3-loop ring-with-pin-bite **plus** the pin disc → a complete solid ring with no
   pin-shaped hole. Extrude by `Base Thickness` in **`NegativeExtentDirection`** as a New Body `'Housing
   Ring'`. ⚠️ The disk extrudes `+normal` (`PositiveExtentDirection`); this offset plane shares that normal,
   so **away from the disk = `NegativeExtentDirection`** (Positive would drive the housing into the disk)
   (`[CYCLOIDAL-F-RING-PINS]`). Stash `self.housingRing`. ⚠️ Not `profileLoops.count == 2` /
   `find_profile_by_curve_counts` — once the pin is added the ring is no longer a clean 2-loop profile.
4. **Drive axis at `O`.** Construction axis `'Drive Axis'` ⟂ the sketch at `O`, from the Housing-Ring
   extrude's **cap face selected by NORMAL** (`startFaces.item(0)`, never a proximity search) via
   `setByPerpendicularAtPoint(capFace, originPoint)`. Stash `self.driveAxis`.
5. **Ring pin extrude — two-sided full-length post.** ⚠️ **Select the pin DISC, not the ring around it:**
   the ring-pin circle bounds the small pin disc (1 loop) **and** the surrounding 3-loop ring (where the pin
   is an inner hole-loop); a generic "any-loop-contains-pin" match grabs the ring (the area *outside* the
   pin) — wrong. Pick the profile with **`profileLoops.count == 1` whose loop curve is the ring-pin circle**.
   Then extrude **both sides** with `setTwoSidesExtent(sideOne, sideTwo)` (this plane shares `self.plane`'s
   normal, so `+normal` is toward the disk): `sideOne` (toward the disk) = `DiscThickness + 1 mm` (reaching
   the disk top); `sideTwo` (away, toward/through the housing) = `Base Thickness` (reaching the housing's far
   face). New Body `'Ring Pin'`, a **separate dowel**. Keep the `ExtrudeFeature` **and** its body `pinBody`.
6. **Combine-cut a socket hole in the housing (keep the pin).** `combineFeatures` Cut: target =
   `self.housingRing`, tools = `[pinBody]`, `operation = CutFeatureOperation`, **`isKeepToolBodies = True`**
   (the pin survives). → a pin-shaped through-hole in the housing, the dowel seated in it
   (`[CYCLOIDAL-F-RING-PINS]`). Keep the `CombineFeature`.
7. **Chamfer the pin ends** (if `Chamfer Size > 0`). `chamferFeature = self._chamferCapRims(pinBody)`
   (`[CYCLOIDAL-F-CHAMFERS]`) — a 45° equal chamfer on the pin's two circular ends. Keep the `ChamferFeature`
   (may be `None` when `Chamfer Size == 0`).
8. **Pattern the pin, hole AND chamfer ×`N` (= `Pin Count`) about `self.driveAxis`** over `360°`:
   circular-pattern an `ObjectCollection` holding the pin `ExtrudeFeature`, the socket `CombineFeature`, and
   the pin-end `ChamferFeature` (when present) — same input, so each patterned pin gets its matching hole and
   its chamfer. → `N` posts, each through a matching hole in the single Housing Ring. Done.

The ring pins span `[−1mm − BaseThickness, +DiscThickness]`: flush with the housing bottom, through the
`1 mm` gap, up to the disk top — so the disk lobes roll against them over the full disk height while the
Housing Ring sits clear of the disk `1 mm` below it.

### 6: Eccentric cam + disk center bore — `buildCam()`
The **input**: an eccentric cam the disk rides on, on a plain (journal) bearing. The cam **outer** is on the
disk centre `Od` (diameter `Center Bearing Diameter`); its **input-shaft bore** is on the drive axis `O`
(diameter `Input Shaft Diameter`) — the `E` offset between `Od` and `O` makes it eccentric. The disk's
**center bore** is enlarged over the cam by `Bearing Clearance` (a concentric running gap, both on `Od`) so
the cam spins freely. **Two sketches** keep the bore-cut and cam profiles clean (`[CYCLOIDAL-F-CAM]`):
1. **Disk center bore — sketch `Disk Bore`** on `self.plane`, anchored to `O`, rebuild `Od`
   (`[CYCLOIDAL-F-DISK-CENTER]`). One **solid** circle on `Od`, diameter dim
   `.parameter.expression = 'CenterBearingDiameter + BearingClearance'`. Extrude-**cut** it (all the sketch's
   profiles = the disc) through **`self.diskBody`** by `Disc Thickness` (`CutFeatureOperation`,
   `participantBodies = [self.diskBody]`, `PositiveExtentDirection`). → the disk bore, `Bearing Clearance`
   larger than the cam (≈ 0.1 mm radial gap at the 0.2 mm default).
2. **Eccentric Cam — sketch `Eccentric Cam`** on `self.plane`, anchored to `O`, rebuild `Od`. Draw the
   **cam-outer circle** (radius `CenterBearingDiameter/2`, **solid**, on `Od`, diameter dim →
   `CenterBearingDiameter`). If `Input Shaft Diameter > 0`, draw the **input-bore circle**
   (radius `InputShaftDiameter/2`, **solid**, on `O`, diameter dim → `InputShaftDiameter`) — it sits inside
   the cam outer (offset `E` from `Od`), splitting it into a small bore disc + the cam annulus. Extrude the
   **cam cross-section** by `Disc Thickness` as a New Body `'Eccentric Cam'` (`PositiveExtentDirection`,
   `[0, DiscThickness]`, co-level with the disk): the **2-loop annulus** when `Input Shaft Diameter > 0`,
   else the **1-loop cam disc**. Stash `self.cam`. Done.

### 7: Output pins + plate — `buildOutputPins()`
The **output member**: a solid plate above the disk carrying `M` output pins that hang **down** through the
disk's `M` output holes. Built like the ring housing, mirrored to the `+normal` side (`[CYCLOIDAL-F-OUTPUT-PINS]`):
1. **Output plate plane.** A construction plane `setByOffset(self.plane, 'DiscThickness + 1 mm')` — `1 mm`
   above the disk top, **opposite** the housing. Name `'Output Plate Plane'`.
2. **One sketch `Output Plate`** on that plane, anchored to `O`. Add: a **solid plate-outer circle** on `O`
   (diameter dim → `OutputPlateDiameter`); a **construction output-pin circle** on `O` (radius `Rop`, dim →
   `OutputPinCircleDiameter`); and **one solid output pin** on `O` at `(Rop, 0)` (diameter dim →
   `OutputHoleDiameter - 2 * Eccentricity` = `D_pin`; centre coincident to the output-pin circle + a
   horizontal spoke from `O`). The pin sits inside the plate disc, splitting it (pin disc + plate-with-pin-bite).
3. **Output plate body.** Extrude the **full plate disc** — **all** the sketch's profiles (plate-with-bite +
   pin disc, so the pin footprint is solid) — **away from the disk** (`PositiveExtentDirection`, the plate is
   above the disk so `+normal` points away) by `Output Plate Thickness` as a New Body `'Output Plate'`. Stash
   `self.outputPlate`.
4. **Output pin (two-sided).** From the **pin disc** (the `profileLoops.count == 1` profile whose loop is the
   output pin), extrude **both sides** with `setTwoSidesExtent(sideOne, sideTwo)`: `sideOne`
   (`PositiveExtentDirection`, away from the disk, into the plate) = `Output Plate Thickness`; `sideTwo`
   (toward the disk) = `DiscThickness + 1 mm` (reaching the disk bottom, so the pin runs the full output-hole
   height). New Body `'Output Pin'`; keep the `ExtrudeFeature` and its body.
5. **Socket (combine-cut, keep the pin).** `combineFeatures` Cut: target `self.outputPlate`, tool the output
   pin body, `isKeepToolBodies = True` → a matching hole in the plate, the pin seated in it.
6. **Chamfer the pin ends** (if `Chamfer Size > 0`): `chamferFeature = self._chamferCapRims(pinBody)`
   (`[CYCLOIDAL-F-CHAMFERS]`). Keep the `ChamferFeature` (may be `None`).
7. **Pattern ×`M` (= `Output Pin Count`) about `self.driveAxis`** (reuse the drive axis at `O` from
   `buildRingPins`): an `ObjectCollection` with the pin `ExtrudeFeature`, the socket `CombineFeature`, and the
   pin-end `ChamferFeature` (when present). → `M` output pins, each through a plate socket and orbiting
   through the disk's output holes. Done.

### 8: Edge chamfers — `buildChamfers()`
If `Chamfer Size > 0`, chamfer the **outer rim** (both flat faces) of each disc-like body via
`self._chamferCapRims(body)` (`[CYCLOIDAL-F-CHAMFERS]`): `self.diskBody` (the rotor disc — its rim is the
**lobe profile**), `self.housingRing`, and `self.outputPlate`. Inner edges (bores, output holes, sockets)
are left sharp — outer rim only. (The ring pins and output pins are already chamfered in their own build
steps, inside their patterns, so every instance carries the chamfer.) If `Chamfer Size == 0`, do nothing.
Done.
