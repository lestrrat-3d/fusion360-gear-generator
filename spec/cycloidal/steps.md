The proof for this step list is `proof/cycloidal/geometry_test.go`, `proof/cycloidal/sketches_test.go`, `proof/cycloidal/solids_test.go` and the generated `proof/cycloidal/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/cycloidal/instructions.md` | `aeadefabc96be386cc9610f01d38a3a1d096f444` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/cycloidal/epitrochoid-trace.md` | `2dd150ac312ca9c673812661e4fa229df433dade` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## S01 `[PROSE]` Dialog inputs and per-field message slots

`CycloidalDriveCommandInputsConfigurator.configure(cls, command)` adds every input to
`command.commandInputs` in exactly the order below, and adds a hidden message slot immediately
after each value and dropdown input. Selections get no slot.

The ids, labels and registered parameter names are reproduced here in full because nothing
downstream can look them up:

| # | Dialog label | input id | add call | unit string | default | registered user parameter |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `plane` | `addSelectionInput` | — | — | — |
| 2 | Anchor Point | `anchorPoint` | `addSelectionInput` | — | — | — |
| 3 | Disc Count | `discCount` | `addDropDownCommandInput` | — | `'1'` | — |
| 4 | Pin Count | `pinCount` | `addValueInput` | `''` | 16 | `PinCount` |
| 5 | Pin Circle Diameter | `pinCircleDiameter` | `addValueInput` | `'mm'` | 90 mm | `PinCircleDiameter` |
| 6 | Pin Diameter | `pinDiameter` | `addValueInput` | `'mm'` | 0 mm | `PinDiameter` |
| 7 | Eccentricity | `eccentricity` | `addValueInput` | `'mm'` | 1.5 mm | `Eccentricity` |
| 8 | Disk Clearance | `diskClearance` | `addValueInput` | `'mm'` | 0.3 mm | `DiskClearance` |
| 9 | Disc Thickness | `discThickness` | `addValueInput` | `'mm'` | 8 mm | `DiscThickness` |
| 10 | Disc Gap | `discGap` | `addValueInput` | `'mm'` | 0.5 mm | `DiscGap` |
| 11 | Center Bearing Diameter | `centerBearingDiameter` | `addValueInput` | `'mm'` | 30 mm | `CenterBearingDiameter` |
| 12 | Input Shaft Diameter | `inputShaftDiameter` | `addValueInput` | `'mm'` | 8 mm | `InputShaftDiameter` |
| 13 | Bearing Clearance | `bearingClearance` | `addValueInput` | `'mm'` | 0.2 mm | `BearingClearance` |
| 14 | Output Pin Circle Diameter | `outputPinCircleDiameter` | `addValueInput` | `'mm'` | 50 mm | `OutputPinCircleDiameter` |
| 15 | Output Pin Count | `outputPinCount` | `addValueInput` | `''` | 6 | `OutputPinCount` |
| 16 | Output Pin Diameter | `outputPinDiameter` | `addValueInput` | `'mm'` | 0 mm | `OutputPinDiameter` |
| 17 | Housing Wall | `wall` | `addValueInput` | `'mm'` | 3 mm | `Wall` |
| 18 | Base Thickness | `baseThickness` | `addValueInput` | `'mm'` | 5 mm | `BaseThickness` |
| 19 | Output Plate Thickness | `outputPlateThickness` | `addValueInput` | `'mm'` | 5 mm | `OutputPlateThickness` |
| 20 | Chamfer Size | `chamferSize` | `addValueInput` | `'mm'` | 0.5 mm | `ChamferSize` |
| 21 | Parent Component | `parentComponent` | `addSelectionInput` | — | root component | — |

Each row's add call takes its own shape: a selection is
`inputs.addSelectionInput(<id>, <label>, <tooltip>)`, the dropdown is
`inputs.addDropDownCommandInput(<id>, <label>, <style>)`, and every numeric is
`inputs.addValueInput(<id>, <label>, <unit>, <ValueInput>)`.

Every numeric default above is written in display units and passed to `addValueInput` as
`adsk.core.ValueInput.createByReal(to_cm(<display value>))`, because a `createByReal` default is
read in Fusion internal units no matter what the unit string says (`[PB-DIALOG-DEFAULT-UNITS]`).
The two counts carry the unitless unit string `''` and are passed as
`adsk.core.ValueInput.createByReal(16)` and `adsk.core.ValueInput.createByReal(6)` with no
conversion.

Selection filters and limits, per input (`[PB-SELECTION-DECL]`, `[PB-SELECTION-FILTER-ENUM]`):

- `plane` — `addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` and
  `addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)`; `setSelectionLimits(1, 1)`.
- `anchorPoint` — `addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)` and
  `addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)`; `setSelectionLimits(1, 1)`.
- `parentComponent` — `addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)` and
  `addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)`;
  `setSelectionLimits(0, 1)`; pre-select the root with
  `addSelection(get_design().rootComponent)`.

`plane` is added first so Fusion's auto-focus lands on it (`[PB-AUTOFOCUS-FIRST]`).

Disc Count is a dropdown: `addDropDownCommandInput(INPUT_ID_DISC_COUNT, 'Disc Count',
adsk.core.DropDownStyles.TextListDropDownStyle)`, then two items through the returned input's
`listItems` collection — `add('1', True)` and `add('2', False)` — so `'1'` is the default.

**Per-field message slot.** Immediately after each of inputs 3 through 20, add
`inputs.addTextBoxCommandInput(<that input's id> + '__status', '', '', 2, True)` — the id, an
empty label, empty initial formatted text, 2 rows, read-only — and set the returned input's
`isVisible = False`. The suffix is exactly `'__status'`, so the slot for input 5 is
`pinCircleDiameter__status`. These slots are never read by any `get_*` helper and are not
registered as parameters; the shared `GearCommand.command_validate_input` handler writes into them
(`[PB-VALIDATE-INPUTS]`).

There is no `handle_input_changed`: this dialog has no conditional visibility.

<!-- check-step-calls: ignore configure command_validate_input -->

**From:** `spec/cycloidal/instructions.md` L15–19, L58–123, L154–216

## S02 `[PROSE]` Live input validation — one routine, two callers

`CycloidalDriveGenerator` declares the two members the shared handler consults
(`[PB-VALIDATE-INPUTS]`):

- the class attribute `DEFAULT_STATUS_INPUT_ID = INPUT_ID_PIN_CIRCLE_DIAMETER + '__status'`, the
  fallback slot;
- `@staticmethod validate_inputs(inputs) -> list[str]`, a pure check that reads raw values off
  `inputs.itemById(<id>).value` (internal cm for lengths, the rounded int for the two counts) and
  `inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem.name` for the dropdown, resolves the derived
  dimensions, and returns the problem list. It writes nothing to the document and registers no
  parameter. A value that cannot be read yet is left to raise; the shared handler catches it and
  treats the inputs as provisionally valid.

Both `validate_inputs` and `_resolveDimensions` build the same value set and call the single
module-level helper `evaluate_problems(vals) -> list[str]`. The formulas are written once, there.

Resolution, in this order, on internal-cm values (`R = pinCircleDiameter / 2`,
`Rop = outputPinCircleDiameter / 2`, `E`, `c = diskClearance`, `N` and `M` the rounded counts,
`CBD = centerBearingDiameter`, `clr = bearingClearance`, `ISD = inputShaftDiameter`):

- `Rr = pinDiameter / 2` when `pinDiameter > 0`, else `0.5 * (E + R * math.sin(math.pi / N))`;
- `Rr_eff = Rr + c`;
- `Rv = R - Rr_eff - E`;
- `D_pin = outputPinDiameter` when `outputPinDiameter > 0`, else `Rop * math.sin(math.pi / M) - E`;
- `D_hole = D_pin + 2 * E`.

`evaluate_problems` returns every failing message, in this order, formatted in mm through `to_mm`
and rounded to about two decimals, with counts as integers:

| # | Must hold | Message when it fails |
|---|---|---|
| 1 | two discs ⇒ `N` even and `M` even | `Two discs require an even Pin Count and an even Output Pin Count (currently N=…, M=…).` |
| 2 | `E < Rr < R * sin(pi / N)` | auto: `Pin geometry out of range — increase Pin Circle Diameter above {2E/sin(pi/N)} mm or reduce Eccentricity below {R*sin(pi/N)} mm.` override: `Pin Diameter must be between {2E} mm and {2*R*sin(pi/N)} mm (currently {2*Rr}).` |
| 3 | `D_pin > 0` | auto: `Output pins vanish (resolved diameter ≤ 0) — increase Output Pin Circle Diameter above {2E/sin(pi/M)} mm, increase Output Pin Count, or reduce Eccentricity.` override: `Output Pin Diameter must be greater than 0.` |
| 4 | `D_hole < 2 * Rop * sin(pi / M)` | `Output holes overlap — increase Output Pin Circle Diameter above {(D_pin+2E)/sin(pi/M)} mm, increase Output Pin Count, or reduce Output Pin Diameter / Eccentricity.` |
| 5 | `E < R / N` | `Eccentricity too large — reduce it below {R/N} mm (or increase Pin Circle Diameter / reduce Pin Count).` |
| 6 | `Rop < Rv` | `Output Pin Circle too large — set Output Pin Circle Diameter below {2*Rv} mm (currently {2*Rop}).` |
| 7 | `Rr_eff < rho_min_O` | `Eccentricity too large — the rotor profile undercuts/self-intersects. Reduce Eccentricity below {E*} mm.` |
| 8 | `ISD < CBD` | `Input Shaft Diameter must be less than Center Bearing Diameter ({CBD} mm).` |
| 9 | `E + ISD / 2 < CBD / 2` | `Input bore doesn't fit inside the cam — set Input Shaft Diameter below {CBD − 2E} mm, or reduce Eccentricity / increase Center Bearing Diameter.` |
| 10 | `(CBD + clr) / 2 < Rop - D_hole / 2` | `Disk center bore overlaps the output holes — increase Output Pin Circle Diameter above {CBD + clr + D_hole} mm, or reduce Center Bearing Diameter / Bearing Clearance / output pin size.` |

Check 7 is the binding eccentricity limit and it is numeric. `rho_min_O` is the smallest radius of
curvature of the base trochoid at the points whose centre of curvature lies toward `O`, scanned at
exactly 2000 uniform values of `t` over `[0, 2*pi)`:

```
bx  =  R*cos t − E*cos(N t)        by  = −R*sin t + E*sin(N t)
xp  = −R*sin t + E*N*sin(N t)      yp  = −R*cos t + E*N*cos(N t)
xpp = −R*cos t + E*N²*cos(N t)     ypp =  R*sin t − E*N²*sin(N t)
k   = xp*ypp − yp*xpp                        # skip the sample when |k| < 1e-12
rho = (xp² + yp²)**1.5 / k
s   = sqrt(xp² + yp²);  nx, ny = −yp/s, xp/s
Cx, Cy = bx + rho*nx, by + rho*ny
rho_min_O = min |rho| over the samples with Cx² + Cy² < bx² + by²
```

`E*` in message 7 is found by exactly 40 bisection rounds on `E'` in `(0, E]`, every other input
held, re-resolving `Rr` and `rho_min_O` at each round because both move with `E'` when
`pinDiameter` is 0; report `to_mm(E*)`. When no positive `E'` satisfies the guard, drop the number
and give the plain "reduce Eccentricity" wording. The cost of this — a 2000-point scan per
keystroke, and 40 more inside the bisection when check 7 fails — is accepted; do not cache or
downsample.

`proof/cycloidal/sketches_test.go` reproduces checks 7, 9 and 10 and the `rho_min_O` scan, and
pins the bound the spec states for the dialog defaults: `E*` is 2.50 mm against the loose
`R / N` of 2.8125 mm. The eight arithmetic checks that only compare two resolved numbers have no
geometry for either harness to build, which is why this step is `[PROSE]`;
`proof/cycloidal/sketches_test.go` says so beside the guard it does build.

`validate_inputs` and `evaluate_problems` are this module's own Python, and the word "vanish" in
message 3 is prose inside a string literal, so none of the three is a name to look for in the
Fusion API database.

<!-- check-step-calls: ignore validate_inputs vanish -->
<!-- check-compile: ignore validate_inputs evaluate_problems vanish -->

**From:** `spec/cycloidal/instructions.md` L142–152, L218–273; `spec/cycloidal/epitrochoid-trace.md` L40–84

## S03 `[PROSE]` Read the inputs, register the parameters, resolve the dimensions

`processInputs(self, inputs)` runs in a fixed order, because creating the occurrence shifts
Fusion's active component and drops selections (`[PB-SELECTION-STASH]`):

1. Pull all three selections first, with `get_selection(inputs, <id>)`, and stash them on `self`:
   `self.parentComponent` from `parentComponent` (an `Occurrence` resolves to its `.component`; an
   empty selection falls back to `get_design().rootComponent`), `self.plane` from `plane`, and
   `self.anchorPoint` from `anchorPoint`.
2. Read Disc Count from the dropdown, never with a `get_*` helper (`[PB-INPUT-READ]`):
   `int(inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem.name)` gives `D`.
3. Set `self.chamfersSkipped = 0` before any build step runs.
4. Read every value input with `get_value(inputs, <id>, <unit>)` — unit `'mm'` for the lengths and
   `''` for `pinCount` and `outputPinCount` — and register each with
   `self.addParameter(<name>, <ValueInput>, <unit>, <comment>)` under the names in S01's table.
   Read `N` and `M` for the Python formulas as the dialog value rounded to `int`, since a Fusion
   user parameter is a float.
5. Call `self._resolveDimensions()`. It builds the same value set S02 describes, and raises
   `Exception('\n'.join(problems))` when `evaluate_problems(vals)` is non-empty. On success it
   stashes the resolved `self.Rr` and `self.D_hole`.
6. Register the derived parameters, after the resolve, because two of them read what it stashed:
   - `PinRadius` and `OutputHoleDiameter` are numeric snapshots in internal cm —
     `self.addParameter(PARAM_PIN_RADIUS, adsk.core.ValueInput.createByReal(self.Rr), 'mm', …)` and
     the same shape with `createByReal(self.D_hole)`. They resolve through an auto-versus-override
     branch in Python that a live Fusion expression cannot reproduce, so they are snapshots and not
     `createByString` (`[PB-NUMERIC-SNAPSHOT]`).
   - `Lobes`, `PinCircleRadius`, `OutputPinCircleRadius`, `HousingInnerDiameter`,
     `HousingOuterDiameter` and `OutputPlateDiameter` stay live, each registered with
     `adsk.core.ValueInput.createByString(<expression>)`, and each expression naming its operands
     through `self.parameterName(PARAM_…)`. Their values are `Lobes = N − 1`,
     `PinCircleRadius = PinCircleDiameter / 2`,
     `OutputPinCircleRadius = OutputPinCircleDiameter / 2`,
     `HousingInnerDiameter = 2 * (PinCircleRadius − PinRadius − Wall)`,
     `HousingOuterDiameter = 2 * (PinCircleRadius − PinRadius + 2 * Eccentricity + Wall)`,
     `OutputPlateDiameter = OutputPinCircleDiameter + (OutputHoleDiameter − 2 * Eccentricity) + 2 * Wall`.

Per-disc handles are lists indexed by `d`: `self.diskBodies`, `self.diskAxes`, `self.lobeSplines`,
`self.outputHoles`, `self.lobeDiskCentres`, `self.discPlanes`. The rest are scalars:
`self.driveAxis`, `self.housingRing`, `self.ringCasing`, `self.cam`, `self.outputPlate`, and
`self.lobePinCircle`, which is stashed and never read.

`self.prefixBase()` returns `'CycloidalDrive'`, so every registered name is
`CycloidalDrive<N>_<name>` and every expression must substitute through
`self.parameterName(PARAM_…)`. A bare name raises `RuntimeError: 3 : Expression is invalid`.

`evaluate_problems` is the module-level helper S02 describes, not a Fusion API call.

<!-- check-step-calls: ignore prefixBase -->
<!-- check-compile: ignore evaluate_problems -->

**From:** `spec/cycloidal/instructions.md` L20–37, L126–152, L184–216, L331–347

## S04 `[PROSE]` Build the component, name it, normalise the Target Plane

`generate(self, inputs)` calls `processInputs(inputs)`, then
`component = self.getComponent()` and `component.name = self.generateName()`.
`generateName()` returns `'Cycloidal Drive (N={}):{}'.format(N, L)` with `N` the rounded Pin Count
and `L = N − 1` — for the defaults, `Cycloidal Drive (N=16):15`.

If `self.plane` is not an `adsk.fusion.ConstructionPlane`, replace it with a coplanar one
(`[PB-CONSTRUCTION-PLANES]`):
`planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('0 mm'))`,
`self.plane = component.constructionPlanes.add(planeInput)`.

Then the build steps run in the order this list gives them: the per-disc loop `for d in range(D)`
over `buildLobeSketch(d)`, `buildDisk(d)`, `buildOutputHoleSketch(d)`, `buildOutputHoles(d)` and
`buildDiskBore(d)`; then `buildCam()`, `buildRingPins()`, `buildOutputPins()`, `buildChamfers()`
and `buildSubComponents()`. Use `futil.log` for step progress and let the entry point's own
try/except and `deleteComponent` handle rollback; add no silent failure path of your own
(`[PB-LOGGING]`).

At the very end of `generate`, after `buildSubComponents()`, if `self.chamfersSkipped > 0` show a
non-fatal message and continue. The message text names a count of skipped chamfers, so the literal
below contains a parenthesis that is not a call.

<!-- check-step-calls: ignore chamfer -->

`adsk.core.Application.get().userInterface.messageBox('Cycloidal drive generated, but {n} chamfer(s) could not be created at Chamfer Size {sz} mm and were skipped. Reduce Chamfer Size (or set it to 0) for this geometry.'.format(n=self.chamfersSkipped, sz=to_mm(self.chamferSize)))`.

The ten `build…` names above are this generator's own methods, and the word "chamfer" in the
message literal is prose, so none of them is a Fusion API call to look up.

<!-- check-step-calls: ignore generate -->
<!-- check-compile: ignore buildLobeSketch buildDisk buildOutputHoleSketch buildOutputHoles buildDiskBore buildCam buildRingPins buildOutputPins buildChamfers buildSubComponents chamfer -->

**From:** `spec/cycloidal/instructions.md` L349–385, L421–424, L631–645

## S05 `[PROSE]` Construction plane `Disc Plane {d+1}` for each disc above the first

For `d = 0` the disc is built on `self.plane` and no plane is created. For every `d > 0`:
`planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('{} * ({} + {})'.format(d, self.parameterName(PARAM_DISC_THICKNESS), self.parameterName(PARAM_DISC_GAP))))`,
`plane = component.constructionPlanes.add(planeInput)`, `plane.name = 'Disc Plane {}'.format(d + 1)`,
`self.discPlanes[d] = plane`.

The offset is `z_d = d * (T + g)` with `T = DiscThickness` and `g = DiscGap`, so disc `d` spans
`[z_d, z_d + T]` and every disc extrude runs `PositiveExtentDirection` from its own plane. Both
parameter names in that string are prefixed; a bare `'DiscThickness'` raises
`RuntimeError: invalid expression` (`[CYCLOIDAL-F-TWO-DISC]`).

`DiscCount` is a dropdown, not a parameter, so it never appears in an expression — only as the
literal integer `d` or `D`. The stack top expression used later is `stackTopExpr = nT` for `D = 1`
and `'2 * {} + {}'.format(nT, nG)` for `D = 2`, with `nT = self.parameterName(PARAM_DISC_THICKNESS)`
and `nG = self.parameterName(PARAM_DISC_GAP)`.

A construction plane's offset is a number Fusion resolves; `proof/cycloidal/solids_test.go` places
every disc-`d` body at `z_d` directly and reads the z faces back, which is the same fact, so this
step carries no proof function of its own.

**From:** `spec/cycloidal/instructions.md` L333–347, L389–419; `spec/cycloidal/fusion.md` L462–498

## S06 `[GO]` Sketch `Rotor Lobe {d+1}` — the fully constrained lobe on the eccentric disc centre

`buildLobeSketch(d)` creates the sketch on `plane(d)` — `self.plane` for `d = 0`, else
`self.discPlanes[d]` — names it `'Rotor Lobe {}'.format(d + 1)` and leaves it visible.

**Anchor chain** (`[CYCLOIDAL-F-ANCHOR-CHAIN]`). `projected = sketch.project(self.anchorPoint).item(0)`;
`localOrigin = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`;
`sketch.geometricConstraints.addCoincident(localOrigin, projected)`. The local origin is a fresh
point, not `sketch.originPoint`, and everything below is drawn relative to it.

**Eccentric disc centre** (`[CYCLOIDAL-F-DISK-CENTER]`). With `s_d = +1` for `d = 0` and `−1` for
`d = 1`:
`diskCentre = sketch.sketchPoints.add(adsk.core.Point3D.create(s_d * E, 0, 0))`;
`eccLine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, diskCentre)`;
`eccLine.isConstruction = True`; `sketch.geometricConstraints.addHorizontal(eccLine)`; then the
driving distance dimension
`sketch.sketchDimensions.addDistanceDimension(localOrigin, diskCentre, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, adsk.core.Point3D.create(s_d * E / 2, -Rv / 10, 0))`
and `dim.parameter.expression = self.parameterName(PARAM_ECCENTRICITY)`. The dimension's value is
a magnitude; disc 1's sign lives in the seeded point at `(−E, 0)` and in the line's direction, never
in a negative value (`[PB-DIM-VALUE-SEMANTICS]`). Stash `diskCentre` on `self.lobeDiskCentres[d]`.

**The three reference circles**, in this order, each created with
`sketch.sketchCurves.sketchCircles.addByCenterRadius`, each `isConstruction = True`, each centre
constrained with one `sketch.geometricConstraints.addCoincident` and no `isFixed`
(`[PB-SHARE-XOR-COINCIDENT]`, `[PB-CIRCLE-CENTER]`), each given a driving
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` whose text point is off the centre
(`[PB-RADIAL-DIM]`) and whose `.parameter.expression` is set as follows, and each labelled along
its own path (`[PB-SKETCH-TEXT]`):

1. **Pin circle**, radius `R`, centre `adsk.core.Point3D.create(0, 0, 0)`, coincident to
   `localOrigin` — on the drive axis, not the disc centre. Expression
   `self.parameterName(PARAM_PIN_CIRCLE_DIAMETER)`. Label `'Pin Circle'`. For `d = 0` only, stash
   it on `self.lobePinCircle`; nothing reads that stash.
2. **Output-pin circle**, radius `Rop`, centre `adsk.core.Point3D.create(s_d * E, 0, 0)`,
   coincident to `diskCentre`. Expression `self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)`.
   Label `'Output Pin Circle'`.
3. **Root circle**, radius `Rv`, centre `adsk.core.Point3D.create(s_d * E, 0, 0)`, coincident to
   `diskCentre`. Expression
   `'2 * ({} - {} - {} - {})'.format(self.parameterName(PARAM_PIN_CIRCLE_RADIUS), self.parameterName(PARAM_PIN_RADIUS), self.parameterName(PARAM_DISK_CLEARANCE), self.parameterName(PARAM_ECCENTRICITY))`.
   Label `'Root Circle'`.

Each label is the three-call shape `textInput = sketch.sketchTexts.createInput2(<name>, self.Rr)`,
`textInput.setAsAlongPath(<that circle>, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
`sketch.sketchTexts.add(textInput)`. The height is the resolved pin radius `self.Rr`, in cm.

**The lobe** (`[CYCLOIDAL-F-DISK-LOBE]`). One open fitted spline through the adaptively sampled
points of `disk_point(t, cx = s_d * E, cy = 0, phi = d * pi)`. The point function, in internal cm,
with `Rr_eff = Rr + c`:

```
num = sin((1 − N) * t)
den = R / (E * N) − cos((1 − N) * t)
psi = atan2(num, den)                      # uses R, E and N only, never Rr
x0  =  R*cos(t) − Rr_eff*cos(t + psi) − E*cos(N * t)
y0  = −R*sin(t) + Rr_eff*sin(t + psi) + E*sin(N * t)
x   = cx + x0*cos(phi) − y0*sin(phi)
y   = cy + x0*sin(phi) + y0*cos(phi)
```

Sample it over `t` in `[0, 2*pi/L]`, `L = N − 1`, by bounded turn angle and never uniformly: trace
exactly 2000 uniform steps (2001 points); keep the first; accumulate the direction change between
consecutive fine points and keep a point and reset whenever the accumulator reaches exactly 5.0
degrees; always keep the last. A uniform sample overshoots into rabbit-ear loops near the undercut
limit. Add each kept point as `adsk.core.Point3D.create(x, y, 0)` — already cm, never re-wrapped in
`to_cm` — into `coll = adsk.core.ObjectCollection.create()`, then
`spline = sketch.sketchCurves.sketchFittedSplines.add(coll)` (`[PB-SKETCHCURVES]`). Never set
`isClosed` and add no closing arc. Stash it on `self.lobeSplines[d]`.

**Lock the spline.** `for i in range(1, spline.fitPoints.count - 1): spline.fitPoints.item(i).isFixed = True`
— the interior points only; fixing the whole spline makes the angle dimension redundant. Then put
each end on the root circle: `sketch.geometricConstraints.addCoincident(spline.fitPoints.item(0), rootCircle)`
and `sketch.geometricConstraints.addCoincident(spline.fitPoints.item(spline.fitPoints.count - 1), rootCircle)`.
Those are point-on-curve constraints and pin each valley's radius to `Rv`; the angles come from the
spokes.

**Spoke 1.** `line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(diskCentre, adsk.core.Point3D.create(s_d * E + Rv * cos(d * pi), Rv * sin(d * pi), 0))`
— the start shares `diskCentre`, so no coincident is added to it. Then
`sketch.geometricConstraints.addCoincident(line1.endSketchPoint, spline.fitPoints.item(0))` and
`sketch.geometricConstraints.addHorizontal(line1)`.

**Spoke 2.** `line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(diskCentre, adsk.core.Point3D.create(s_d * E + Rv * cos(d * pi - 2 * pi / L), Rv * sin(d * pi - 2 * pi / L), 0))`
and `sketch.geometricConstraints.addCoincident(line2.endSketchPoint, spline.fitPoints.item(spline.fitPoints.count - 1))`.
No horizontal on this one.

**Lobe pitch.** `angDim = sketch.sketchDimensions.addAngularDimension(line1, line2, adsk.core.Point3D.create(s_d * E + 0.4 * Rv * cos(d * pi - pi / L), 0.4 * Rv * sin(d * pi - pi / L), 0))`,
the text point in the minor wedge so Fusion measures the minor angle and not the reflex one
(`[PB-ANGULAR-DIM]`), then
`angDim.parameter.expression = '360 deg / {}'.format(self.parameterName(PARAM_LOBES))`. The
dimension is driving; never pass `isDriven` (`[PB-DRIVING-DIM]`).

After this the sketch is fully constrained. Build no bodies here; the output hole is a separate
sketch so the two profiles never share one.

**What the proof holds.** `stepRotorLobeSketch` builds this scheme in the sketch engine and gates it
on the engine's whole verdict — DOF 0, no redundancy, one valid region, and no second discrete
configuration. Two constraints are stated differently there and the proof file says why at each:
the eccentric offset is a signed horizontal distance rather than an unsigned magnitude plus a seed,
and spoke 1's direction is a zero angle to the eccentric line rather than a horizontal. Both
unsigned forms reach DOF 0 and still admit the mirrored answer, which the gate refuses. The proof
also runs the `rho_min_O` scan of S02 and the 40-round bisection for every case, so the profile it
draws is known to be clean, and it pins the spec's worked bound — `E*` of 2.50 mm at the defaults.

`buildLobeSketch` is this generator's own method and `disk_point` is the module's own Python
transcription of the point function, so neither is a Fusion API call.

<!-- check-compile: ignore buildLobeSketch disk_point -->

The spec records `[PB-SKETCH-FIRST]` as waived for this gear, on the grounds that no bench proof of
the lobe exists. `proof/cycloidal/sketches_test.go` is that proof, so the waiver's stated reason no
longer holds and the paragraph that records it is stale.

`<!-- proof-run: proofkit.RunParallel(sketchCases, stepRotorLobeSketch) -->`

**From:** `spec/cycloidal/instructions.md` L275–329, L389–419, L426–456; `spec/cycloidal/fusion.md` L8–98; `spec/cycloidal/epitrochoid-trace.md` L86–132

## S07 `[GO]` Extrude the lobe sector by Disc Thickness — `Cycloidal Disk {d+1}`

Record `base = component.bRepBodies.count` **before** this extrude; S10 needs it.

Select the sector profile by identity, never by index: iterate `sketch.profiles`, and for each scan
`profileLoops` and their `profileCurves` for `sketchEntity` equal to `self.lobeSplines[d]`. The
along-path text labels add their own letter outlines, so an index would take one of those
(`[CYCLOIDAL-F-DISK-BODY]`). Do not reach for `find_profile_by_curve_counts` here or anywhere else
in this gear: it counts NURBS, arcs and lines per loop and treats a full circle as neither.

`ext = component.features.extrudeFeatures.createInput(sectorProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
`ext.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(self.parameterName(PARAM_DISC_THICKNESS))), adsk.fusion.ExtentDirections.PositiveExtentDirection)`;
`extrude = component.features.extrudeFeatures.add(ext)`; name the body
`'Cycloidal Disk {}'.format(d + 1)`.

`setDistanceExtent` belongs to `HoleFeatureInput` and is not the call here.

**What the proof holds.** `stepExtrudeLobeSector` extrudes the same sector from `z_d` by `T` and
reads its volume against the sector's own area and its bounding box against the sector's extent.
The boundary is the chord polyline through the sampled points rather than a fitted spline: decad
refuses to extrude a free-form span whose curvature sign it cannot certify, and this lobe turns
from concave to convex inside one span. The proof file states that substitution and its cost.

<!-- check-step-calls: ignore find_profile_by_curve_counts setDistanceExtent -->

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepExtrudeLobeSector, assertExtrudeLobeSector) -->`

**From:** `spec/cycloidal/instructions.md` L458–464; `spec/cycloidal/fusion.md` L137–165

## S08 `[PROSE]` Construction axis `Disk Axis {d+1}` at the disc centre

`buildDiskAxis(capFace, d)` runs after S07, because a face-less axis is unsupported in the
parametric environment (`[CYCLOIDAL-F-DISK-AXIS]`).

`capFace = extrude.startFaces.item(0)` — the cap in the sketch plane, picked by its normal being
parallel to the sketch normal. Take it that way unconditionally and never by nearest planar face to
`Od_d` or by which planar face contains it: the pie sector's two spoke faces are also planar and
also contain the apex `Od_d`, and picking one gives an in-plane axis that turns the circular pattern
into garbage.

`axInput = component.constructionAxes.createInput()`;
`axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])`;
`axis = component.constructionAxes.add(axInput)`; `axis.name = 'Disk Axis {}'.format(d + 1)`;
`self.diskAxes[d] = axis`. Do not call `setByLine`, which raises
`RuntimeError: 3 : Environment is not supported`, and do not call `activate` — face-anchored axis
methods work on a component that is not the active one.

decad has no construction axis, and the axis's only observable effect is the direction the next two
steps pattern about, which S10 and S13 read as the tiling they produce. That is why this step
carries no proof function; `proof/cycloidal/solids_test.go` says so beside the tiling it does build.

`buildDiskAxis` is this generator's own method.

<!-- check-step-calls: ignore setByLine activate -->
<!-- check-compile: ignore buildDiskAxis -->

**From:** `spec/cycloidal/instructions.md` L465–472; `spec/cycloidal/fusion.md` L100–135

## S09 `[PROSE]` Circular-pattern the lobe-sector extrude ×L about the Disk Axis

`coll = adsk.core.ObjectCollection.create()`; `coll.add(extrude)` — the `ExtrudeFeature` from S07,
never its body (`[PB-PATTERN-BODIES]`);
`pat = component.features.circularPatternFeatures.createInput(coll, self.diskAxes[d])`;
`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`;
`pat.quantity = adsk.core.ValueInput.createByReal(L)` with `L = N − 1`;
`pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
`component.features.circularPatternFeatures.add(pat)` (`[PB-CIRCULAR-PATTERN]`).

`AdjustPatternCompute` is mandatory on every circular pattern in this gear
(`[CYCLOIDAL-F-OUTPUT-HOLES]`).

decad has no pattern feature, and placing the L copies by hand leaves L bodies that meet face to
face: decad refuses the boolean that would join them, and a touching pair leaves its report unable
to say whether the two cross. The tiling those L sectors have to satisfy is read in S10 instead,
which is why this step is `[PROSE]`; `proof/cycloidal/solids_test.go` records it at
`stepJoinDiscSectors`.

**From:** `spec/cycloidal/instructions.md` L473–475; `spec/cycloidal/fusion.md` L166–173, L230–236

## S10 `[GO]` Join disc `d`'s own L sectors into one `Cycloidal Disk {d+1}` body

Join only this disc's sectors. With two discs, disc 0's body already exists when disc 1 builds, so
`component.bRepBodies.item(0)` is the wrong target. Using the `base` recorded in S07, disc `d`'s
sectors are `component.bRepBodies.item(base)` through `component.bRepBodies.item(base + L - 1)`.

`target = component.bRepBodies.item(base)`; `tools = adsk.core.ObjectCollection.create()` holding
`component.bRepBodies.item(i)` for `i` in `base + 1 … base + L - 1`;
`ci = component.features.combineFeatures.createInput(target, tools)`;
`ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`;
`component.features.combineFeatures.add(ci)`. Name the resulting body
`'Cycloidal Disk {}'.format(d + 1)` and stash `self.diskBodies[d]`. The tools argument is an
`ObjectCollection`, never the pattern's own `bodies` collection (`[PB-PATTERN-BODIES]`,
`[CYCLOIDAL-F-DISK-BODY]`).

**What the proof holds.** `stepJoinDiscSectors` builds the tiled rotor boundary — the one lobe
turned through all L positions about `Od_d` — in a single extrude, reads it as one lump, and reads
its volume against both the tiled polygon's area and, through a seed sector built in a scratch
document, exactly one L-th of it. A pattern that left a gap between sectors is what that pair of
readings refuses, and a gap is how the Join fails.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepJoinDiscSectors, assertJoinDiscSectors) -->`

**From:** `spec/cycloidal/instructions.md` L476–481; `spec/cycloidal/fusion.md` L174–182, L490–493

## S11 `[GO]` Sketch `Output Hole {d+1}` — the one solid hole on the disc centre

`buildOutputHoleSketch(d)` creates a **new** sketch on `plane(d)`, named
`'Output Hole {}'.format(d + 1)` and leaves it visible, so the lobe and hole profiles never share a sketch (`[CYCLOIDAL-F-OUTPUT-HOLE]`).

Anchor it exactly as S06 does (`[CYCLOIDAL-F-ANCHOR-CHAIN]`): project the Anchor with
`sketch.project`, add a fresh `localOrigin` with `sketch.sketchPoints.add` and constrain it with
`sketch.geometricConstraints.addCoincident`. Rebuild `Od_d` exactly as S06 does
(`[CYCLOIDAL-F-DISK-CENTER]`): a `diskCentre` point at `adsk.core.Point3D.create(s_d * E, 0, 0)`, a
construction `eccLine` through `sketch.sketchCurves.sketchLines.addByTwoPoints`, a
`sketch.geometricConstraints.addHorizontal` on it, and a driving
`sketch.sketchDimensions.addDistanceDimension` whose `.parameter.expression` is
`self.parameterName(PARAM_ECCENTRICITY)`.

**Output-hole circle**, construction, on `Od_d`:
`addByCenterRadius(adsk.core.Point3D.create(s_d * E, 0, 0), Rop)`, `isConstruction = True`, centre
coincident to `diskCentre`, driving diameter dimension with
`.parameter.expression = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)`, and the along-path
label `'Output Hole Circle'` at height `self.Rr`.

**One solid hole**, on the `+X` ray from `Od_d` for both discs, since `M` is even whenever two
discs are asked for and disc 1's half-turn maps the hole set onto itself:
`hole = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(s_d * E + Rop, 0, 0), D_hole / 2)`
with no `isConstruction`. Pin its size with a driving
`sketch.sketchDimensions.addDiameterDimension` whose
`.parameter.expression = self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)`. Pin its position with
`sketch.geometricConstraints.addCoincident(hole.centerSketchPoint, outputHoleCircle)` and a
horizontal construction line from `diskCentre` to `hole.centerSketchPoint`
(`addByTwoPoints`, `isConstruction = True`, `sketch.geometricConstraints.addHorizontal`).

Stash the solid circle on `self.outputHoles[d]`; the cut step selects its profile by identity.
Build no bodies here.

**What the proof holds.** `stepOutputHoleSketch` builds the same sketch and gates it on the
engine's whole verdict, then measures the hole's centre against `Od_d + (Rop, 0)`, its seating
radius against the construction circle's own radius, and its radius against `D_hole / 2`. The
seating is stated as a signed horizontal distance rather than the point-on-circle-plus-horizontal
pair: that pair reaches DOF 0 and still admits the hole at `−Rop`, which the gate refuses. The
proof file records the substitution and measures the coincidence the spec's constraint asserts.

`buildOutputHoleSketch` is this generator's own method.

<!-- check-compile: ignore buildOutputHoleSketch -->

`<!-- proof-run: proofkit.RunParallel(sketchCases, stepOutputHoleSketch) -->`

**From:** `spec/cycloidal/instructions.md` L483–492; `spec/cycloidal/fusion.md` L184–204

## S12 `[GO]` Extrude-cut one output hole through the disc

Select the hole profile by identity: the profile whose loop contains `self.outputHoles[d]`. The
construction circle and the text label add other profiles, so an index would take one of those
(`[CYCLOIDAL-F-OUTPUT-HOLES]`).

`ci = component.features.extrudeFeatures.createInput(holeProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)`;
`ci.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(self.parameterName(PARAM_DISC_THICKNESS))), adsk.fusion.ExtentDirections.PositiveExtentDirection)`;
`ci.participantBodies = [self.diskBodies[d]]`;
`cut = component.features.extrudeFeatures.add(ci)`.

The sketch sits on `plane(d)` and the disc spans `[z_d, z_d + T]`, so a cut of `DiscThickness` in
the positive direction passes through it. `participantBodies` restricts the cut to this disc.

**What the proof holds.** `stepCutOutputHole` cuts one hole of diameter `D_hole` through the rotor
and reads the volume it removed against `pi * (D_hole/2)^2 * T`. The tool is run past both faces
rather than made flush: a tool cap in the plane of the body's is a coplanar pair decad refuses to
classify. The removed solid is the same either way, which is what the reading holds.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCutOutputHole, assertCutOutputHole) -->`

**From:** `spec/cycloidal/instructions.md` L494–500; `spec/cycloidal/fusion.md` L206–229

## S13 `[GO]` Circular-pattern the output-hole cut ×M about the Disk Axis

`coll = adsk.core.ObjectCollection.create()`; `coll.add(cut)` — the `ExtrudeFeature` from S12, not
a body;
`pat = component.features.circularPatternFeatures.createInput(coll, self.diskAxes[d])`;
`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`;
`pat.quantity = adsk.core.ValueInput.createByReal(M)` with `M = Output Pin Count`;
`pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
`component.features.circularPatternFeatures.add(pat)`. Quantity, total angle and symmetry are all
pinned explicitly (`[PB-CIRCULAR-PATTERN]`, `[PB-PATTERN-BODIES]`).

`AdjustPatternCompute` is not optional here (`[CYCLOIDAL-F-OUTPUT-HOLES]`). This is a lone patterned cut with no body-creating
feature to anchor it, and under the default paste compute Fusion copies the cut's edges instead of
recomputing each instance against the body; the pattern then fails with
`RuntimeError: 3 … NO_TARGET_BODY … PATTERN_FEATURES_NO_PASTE_INT_EDGES`.

The Disk Axis stands at `Od_d`, so the M holes orbit the disc centre.

**What the proof holds.** `stepPatternOutputHoles` reads the rotor carrying all M holes: one lump,
and a volume exactly `M` hole cylinders under the plain disc. It also refuses two dialogs the
spec's own table does not: holes that run into each other on the output-pin circle, and holes whose
outer edge passes the valley circle `Rv` and so breaks out through the lobe profile. The M openings
are stated as holes in the extruded profile rather than cut one after another, because decad
refuses a boolean whose tool is finer than the mesh the previous boolean left; the proof file says
so, and S12 is where the cut itself is proven as a boolean.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepPatternOutputHoles, assertPatternOutputHoles) -->`

**From:** `spec/cycloidal/instructions.md` L494–502; `spec/cycloidal/fusion.md` L223–236

## S14 `[GO]` Sketch `Disc Bore {d+1}` — the enlarged centre bore

`buildDiskBore(d)` creates a sketch on `plane(d)` named `'Disc Bore {}'.format(d + 1)`, anchored to
`O` and with `Od_d` rebuilt, both exactly as S11 does.

One **solid** circle on the disc centre:
`addByCenterRadius(adsk.core.Point3D.create(s_d * E, 0, 0), (CBD + clr) / 2)`, centre coincident to
`diskCentre`, driving diameter dimension with
`.parameter.expression = '{} + {}'.format(self.parameterName(PARAM_CENTER_BEARING_DIAMETER), self.parameterName(PARAM_BEARING_CLEARANCE))`.

The bore is the cam outer enlarged by the whole Bearing Clearance, so the running gap is half of it
all the way round and the cam turns freely in it (`[CYCLOIDAL-F-CAM]`).

**What the proof holds.** `stepDiscBoreSketch` builds the sketch, gates it on the engine's verdict,
reads the bore radius back and reads its one solid region's area against `pi * ((CBD + clr)/2)^2`.
It also refuses a case whose bore would reach the output holes, which is check 10 of S02.

`buildDiskBore` is this generator's own method.

<!-- check-compile: ignore buildDiskBore -->

`<!-- proof-run: proofkit.RunParallel(sketchCases, stepDiscBoreSketch) -->`

**From:** `spec/cycloidal/instructions.md` L557–567; `spec/cycloidal/fusion.md` L320–339

## S15 `[GO]` Extrude-cut the disc centre bore

Cut every profile of the `Disc Bore {d+1}` sketch (`[CYCLOIDAL-F-CAM]`) — there is only the one disc — through this
disc:
`coll = adsk.core.ObjectCollection.create()` holding each `sketch.profiles.item(i)`;
`ci = component.features.extrudeFeatures.createInput(coll, adsk.fusion.FeatureOperations.CutFeatureOperation)`;
`ci.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(self.parameterName(PARAM_DISC_THICKNESS))), adsk.fusion.ExtentDirections.PositiveExtentDirection)`;
`ci.participantBodies = [self.diskBodies[d]]`;
`component.features.extrudeFeatures.add(ci)`.

**What the proof holds.** `stepCutDiscBore` cuts the same bore through the rotor and reads the
removed volume against `pi * ((CBD + clr)/2)^2 * T`, then checks that the bore stands off the cam
outer by exactly half the Bearing Clearance. The bore is cut through the plain rotor rather than
the holed one: the bore and the holes never meet, which S14 refuses the case for when they would.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCutDiscBore, assertCutDiscBore) -->`

**From:** `spec/cycloidal/instructions.md` L563–567; `spec/cycloidal/fusion.md` L329–339

## S16 `[GO]` Sketch `Eccentric Cam {d+1}` — the cam section

`buildCam()` runs once, after the per-disc loop and before `buildRingPins()`, and loops over `d`
itself. For each `d` it creates a sketch on `plane(d)` named `'Eccentric Cam {}'.format(d + 1)`,
anchored to `O` (`[CYCLOIDAL-F-ANCHOR-CHAIN]`) and with `Od_d` rebuilt
(`[CYCLOIDAL-F-DISK-CENTER]`), both exactly as S11 does (`[CYCLOIDAL-F-CAM]`).

**Cam outer**, solid, on the disc centre:
`addByCenterRadius(adsk.core.Point3D.create(s_d * E, 0, 0), CBD / 2)`, centre coincident to
`diskCentre`, driving diameter dimension with
`.parameter.expression = self.parameterName(PARAM_CENTER_BEARING_DIAMETER)`.

**Input-shaft bore**, only when `Input Shaft Diameter > 0`, solid, on the drive axis:
`addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), ISD / 2)`, centre coincident to
`localOrigin`, driving diameter dimension with
`.parameter.expression = self.parameterName(PARAM_INPUT_SHAFT_DIAMETER)`.

The bore sits inside the cam outer, offset by `E` from it, so it splits the cam disc into a small
bore disc and the cam annulus.

**What the proof holds.** `stepEccentricCamSketch` builds both branches. With a bore it reads two
regions and takes the one carrying a hole, whose area it measures against
`pi * (CBD^2 − ISD^2) / 4`; with `Input Shaft Diameter` at 0 it reads the single disc region
against `pi * CBD^2 / 4`. That two-region reading is the engine's form of the rule S17 selects by.

`buildCam` and `buildRingPins` are this generator's own methods.

<!-- check-compile: ignore buildCam buildRingPins -->

`<!-- proof-run: proofkit.RunParallel(sketchCases, stepEccentricCamSketch) -->`

**From:** `spec/cycloidal/instructions.md` L569–575; `spec/cycloidal/fusion.md` L340–361

## S17 `[GO]` Extrude cam section `d` as a New Body

Select the cross-section by loop count, never by curve counts: when `Input Shaft Diameter > 0` take
the profile whose `profileLoops.count` is 2 — the outer loop is the cam outer and the inner loop is
the bore — and when it is 0 take the sketch's only profile. A full circle is a
`Circle3DCurveType` curve and an annulus keeps its two circles in separate loops, so the
curve-count helper raises `Could not find profile` here (`[CYCLOIDAL-F-CAM]`,
`[CYCLOIDAL-F-DISK-BODY]`).

`ext = component.features.extrudeFeatures.createInput(camProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
`ext.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(<extent>)), adsk.fusion.ExtentDirections.PositiveExtentDirection)`;
`extrude = component.features.extrudeFeatures.add(ext)`; name the body
`'Eccentric Cam {}'.format(d + 1)`.

The extent is `'{} + {}'.format(nT, nG)` for every section but the last, which fills the inter-disc
gap so adjacent sections abut, and `nT` for the last — both prefixed, with
`nT = self.parameterName(PARAM_DISC_THICKNESS)` and `nG = self.parameterName(PARAM_DISC_GAP)`.

**What the proof holds.** `stepExtrudeCamSection` extrudes the section — the eccentric annulus, or
the plain disc when the bore is off — from `z_d` by that extent, and reads its volume and its
bounding box, the box centred on `Od_d` rather than on `O`, which is where the eccentricity shows.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepExtrudeCamSection, assertExtrudeCamSection) -->`

**From:** `spec/cycloidal/instructions.md` L569–579; `spec/cycloidal/fusion.md` L349–361, L500–506

## S18 `[GO]` Join the cam sections into one `Eccentric Cam`

For `D = 1` there is one section and nothing to join; name it `'Eccentric Cam'` and stash
`self.cam`. For `D = 2` (`[CYCLOIDAL-F-CAM]`, `[CYCLOIDAL-F-TWO-DISC]`):
`target` is section 0's body; `tools = adsk.core.ObjectCollection.create()` holding the rest;
`ci = component.features.combineFeatures.createInput(target, tools)`;
`ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`;
`component.features.combineFeatures.add(ci)`. Name the result `'Eccentric Cam'` and stash
`self.cam`.

The two sections sit at `+E` and `−E`, so their centres are `2E` apart while each has radius
`CenterBearingDiameter / 2`; they overlap through the whole central region and the join is one
continuous solid, with the input bore running the full height on `O`.

**What the proof holds.** `stepJoinCamSections` unions the two sections and reads one lump, the
joined volume against the two sections less the lens they share, and the bounding box spanning
`−E − CBD/2` to `+E + CBD/2`. Three things are stated differently and the proof file says so at the
call: the axial abutment is an overlap of a hundredth of a millimetre, because decad refuses a
boolean whose operands meet face to face; the input bore is left out of the joined pair, because
both sections put the same cylinder on the drive axis and decad refuses a tangent contact it cannot
classify; and the bore's fit inside both sections, which is what makes it one unbroken hole, is
checked arithmetically instead. Its case table is the two-disc one, since a single-disc build has
nothing to join.

`<!-- proof-run: proofkit3d.RunSolidParallel(twoDiscCases, stepJoinCamSections, assertJoinCamSections) -->`

**From:** `spec/cycloidal/instructions.md` L574–579; `spec/cycloidal/fusion.md` L356–361, L500–506

## S19 `[PROSE]` Construction plane `Ring Housing Plane`, 1 mm below the disc

`buildRingPins()` starts here, after the discs and the cam (`[CYCLOIDAL-F-RING-PINS]`,
`[PB-CONSTRUCTION-PLANES]`).

`planeInput = component.constructionPlanes.createInput()`;
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('-1 mm'))`;
`housingPlane = component.constructionPlanes.add(planeInput)`;
`housingPlane.name = 'Ring Housing Plane'`.

The offset is the literal `'-1 mm'`, away from the disc, and it is load-bearing: S24's downward
side is the matching `'1 mm'`, which is what puts the casing's bottom face on the base's top face so
S27 leaves one connected solid.

`buildRingPins` is this generator's own method.

<!-- check-compile: ignore buildRingPins -->

The plane's only observable effect is where the base sits, which
`proof/cycloidal/solids_test.go` reads directly as the base's z faces at `−1 − BaseThickness` and
`−1`, so this step carries no proof function of its own.

**From:** `spec/cycloidal/instructions.md` L504–512; `spec/cycloidal/fusion.md` L238–249

## S20 `[GO]` Sketch `Housing Ring` — the base annulus on the drive axis

On `housingPlane`, named `'Housing Ring'`, anchored to `O` exactly as S06 does — project the
Anchor with `sketch.project`, add a fresh `localOrigin` with `sketch.sketchPoints.add`, constrain
it with `sketch.geometricConstraints.addCoincident`.

Two solid circles, both centred on the drive axis:

- outer at `R − Rr + 2 * E + Wall`:
  `addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), R - Rr + 2 * E + Wall)`, centre coincident
  to `localOrigin`, driving diameter dimension with
  `.parameter.expression = self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)`;
- inner at `R − Rr − Wall`:
  `addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), R - Rr - Wall)`, centre coincident to
  `localOrigin`, driving diameter dimension with
  `.parameter.expression = self.parameterName(PARAM_HOUSING_INNER_DIAMETER)`.

Constrain each centre coincident to the anchored local origin and to nothing else. Do not also set
the centre `isFixed`: the coincident already pins it and the pair over-constrains
(`[PB-SHARE-XOR-COINCIDENT]`).

The outer wall is the rolling contour's peak at `R − PinRadius + 2 * E` cleared by `Wall`, so
`Wall` is the minimum wall thickness, reached at the peaks and exceeded at the valleys. The inner
lip sits `Wall` inside the contour valley at `R − PinRadius`. There are no pins and no projected
circle here — this is the base alone.

**What the proof holds.** `stepHousingRingSketch` builds the annulus, gates it on the engine's
verdict, reads both radii back, reads the region carrying a hole and measures its area against
`pi * (ro^2 − ri^2)`, and measures the wall at the contour peak against `Wall` itself.

`<!-- proof-run: proofkit.RunParallel(sketchCases, stepHousingRingSketch) -->`

**From:** `spec/cycloidal/instructions.md` L513–516; `spec/cycloidal/fusion.md` L249–259

## S21 `[GO]` Extrude the housing base annulus by Base Thickness, away from the disc

Select the annulus by `profileLoops.count == 2`, not by curve counts — the two circles sit in
separate loops and a full circle is not an arc (`[CYCLOIDAL-F-DISK-BODY]`).

`ext = component.features.extrudeFeatures.createInput(annulusProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
`ext.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(self.parameterName(PARAM_BASE_THICKNESS))), adsk.fusion.ExtentDirections.NegativeExtentDirection)`;
`housingExtrude = component.features.extrudeFeatures.add(ext)`; name the body `'Housing Ring'` and
stash `self.housingRing`.

The offset plane shares `self.plane`'s normal, so away from the disc is the negative direction.

**What the proof holds.** `stepExtrudeHousingBase` extrudes the annulus and reads its volume
against `pi * (ro^2 − ri^2) * BaseThickness` and its box spanning `−1 − BaseThickness` to `−1`,
which is the top face S24's downward side has to reach.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepExtrudeHousingBase, assertExtrudeHousingBase) -->`

**From:** `spec/cycloidal/instructions.md` L513–516; `spec/cycloidal/fusion.md` L255–259

## S22 `[PROSE]` Construction axis `Drive Axis` at the drive axis `O`

`capFace = housingExtrude.startFaces.item(0)` — the cap picked by normal, exactly as S08 picks the
disc's (`[CYCLOIDAL-F-DISK-AXIS]`, `[CYCLOIDAL-F-RING-PINS]`, `[PB-CONSTRUCTION-AXES]`);
`axInput = component.constructionAxes.createInput()`;
`axInput.setByPerpendicularAtPoint(capFace, originPoint)` with `originPoint` the Housing Ring
sketch's own anchored local origin;
`axis = component.constructionAxes.add(axInput)`; `axis.name = 'Drive Axis'`;
`self.driveAxis = axis`.

Both later patterns about `O` — the casing sectors in S25 and the output pins in S34 — reuse this
axis.

decad has no construction axis, and what this one decides is the direction those two patterns turn
about, which S26 and S34 read as the tiling and the pin placement they produce. That is why this
step carries no proof function; `proof/cycloidal/solids_test.go` says so beside them.

**From:** `spec/cycloidal/instructions.md` L517–519; `spec/cycloidal/fusion.md` L260–261

## S23 `[GO]` Sketch `Ring Casing` — one pin-pitch section of the pinless contour

On `self.plane`, named `'Ring Casing'`, anchored to `O` exactly as S20 does.

**Compute one pin pitch of the contour first, in Python.** The inner wall follows the disc's swept
envelope offset outward by the clearance: `contour(phi) = env(phi) + c`, a formula rather than a
call.

<!-- check-step-calls: ignore contour env -->
<!-- check-compile: ignore contour disk_point -->
 Sweep the world disc
`disk_point(t, E*cos(theta), E*sin(theta), -theta/L)` over `theta` and `t`, each at exactly 240
uniform steps over `[0, 2*pi)`. For each sampled point take `a = atan2(y, x)`, keep only the points
with `a` in `[-pi/N, +pi/N]`, bin them by angle into exactly 80 bins and keep the maximum
`hypot(x, y)` per bin, tracking which bins were hit.

Then emit the contour at bin **edges**, not bin centres: for `i` in `0 … 80`,
`phi_i = -pi/N + (2*pi/N) * i / 80` and `r_i = c + max(binMax[i-1], binMax[i])`, using the single
existing neighbour at each end and taking the maximum as 0 when both neighbours are unhit, which
leaves that edge at radius `c`. The point is `(r_i * cos(phi_i), r_i * sin(phi_i))`, ordered by
angle, already in cm.

Bin centres are the reported bug: they inset the first and last points by half a bin, which leaves
an angular gap between every pair of patterned sectors, so the N sectors never touch and the Join
leaves N unnamed bodies instead of one casing. Edges put the first point exactly on `-pi/N` and the
last exactly on `+pi/N`, where the contour is a mid-gap peak and tangential by symmetry, so the
sectors tile seamlessly.

**The sketch** then holds:

- an **outer circle**, `addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), R - Rr + 2 * E + Wall)`,
  left **solid** — do not set `isConstruction`, because it forms the sector's outer arc and a
  construction circle would leave the wedge open — centre coincident to the anchored
  `localOrigin`, driving diameter dimension with
  `.parameter.expression = self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)`;
- an open **fitted spline** through the contour points: each as
  `adsk.core.Point3D.create(x, y, 0)` into `adsk.core.ObjectCollection.create()`, then
  `sketch.sketchCurves.sketchFittedSplines.add(coll)`, with no `isClosed`. Keep the handle as
  `contour`;
- **two radial spokes**, each `sketch.sketchCurves.sketchLines.addByTwoPoints` from a spline end
  out to a point on the outer circle at that same angle — from the first contour point at `-pi/N`
  to `(ro*cos(-pi/N), ro*sin(-pi/N))`, and from the last at `+pi/N` to
  `(ro*cos(pi/N), ro*sin(pi/N))`.

This sketch is a deliberate `[PB-FULL-CONSTRAINT]` exemption. The contour's fit points are numeric
snapshots and are not set `isFixed`; the two spokes' outer endpoints are only seeded on the outer
circle with no coincident to it. What is constrained is the outer circle's centre and its diameter.
The sketch is consumed immediately by S24 and never re-solved, so the free geometry is accepted.

**What the proof holds.** `stepRingCasingSketch` builds the same section, measures the first and
last contour points' angles against exactly `-pi/N` and `+pi/N`, checks that no contour point
reaches the outer wall, and reads the two regions the solid outer circle and the open contour
bound. It measures the smaller one against the wedge's own area and refuses a run where the
complement is not the larger, which is the ambiguity S24 selects through. Two things are stated
differently and the proof file says so: the contour is the chord polyline through those points
rather than a fitted spline, because decad will not extrude a free-form span whose curvature sign
it cannot certify; and the snapshot points are grounded, because proofkit's gate is DOF 0 with
nothing waived — the spec's own exemption is exactly that they are snapshots.

`<!-- proof-run: proofkit.RunParallel(sketchCases, stepRingCasingSketch) -->`

**From:** `spec/cycloidal/instructions.md` L316–322, L520–534; `spec/cycloidal/fusion.md` L262–283; `spec/cycloidal/epitrochoid-trace.md` L149–188

## S24 `[GO]` Extrude the casing sector two-sided

Select the wedge by **minimum area** among the profiles whose loop contains `contour`, and by
nothing weaker. The solid outer circle makes the open contour a shared edge of two closed profiles:
the thin annular wedge, and the whole complement inside the circle. Both contain the contour, so a
first-match containing-curve search can return the complement, and extruding that gives a near-full
disc which patterns and joins into a solid cylinder with every scallop erased. Compare candidates
by `profile.areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area` and take
the smallest (`[CYCLOIDAL-F-RING-PINS]`).

`ext = component.features.extrudeFeatures.createInput(sectorProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
`ext.setTwoSidesExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(stackTopExpr)), adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString('1 mm')))`;
`sectorFeature = component.features.extrudeFeatures.add(ext)`.

Side one runs up to the stack top and side two runs the literal `'1 mm'` down to the housing base's
top face. That `'1 mm'` has to match S19's `'-1 mm'` exactly, or the casing floats above the base
and S27's Join leaves two lumps. `stackTopExpr` is the prefixed string of S05:
`self.parameterName(PARAM_DISC_THICKNESS)` for one disc, and
`'2 * {} + {}'.format(nT, nG)` for two.

Record `base = component.bRepBodies.count` before this extrude; S26 needs it.

**What the proof holds.** `stepExtrudeCasingSector` extrudes the same wedge — contour, a spoke at
each end, and the outer arc between them — two-sided from the target plane, and reads its volume
against one pin pitch of the outer disc less the pie the contour encloses, and its box spanning
`−1` to the stack top.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepExtrudeCasingSector, assertExtrudeCasingSector) -->`

**From:** `spec/cycloidal/instructions.md` L535–547; `spec/cycloidal/fusion.md` L284–303

## S25 `[PROSE]` Circular-pattern the casing sector ×N about the Drive Axis

`coll = adsk.core.ObjectCollection.create()`; `coll.add(sectorFeature)`;
`pat = component.features.circularPatternFeatures.createInput(coll, self.driveAxis)`;
`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`;
`pat.quantity = adsk.core.ValueInput.createByReal(N)` with `N = Pin Count`;
`pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
`component.features.circularPatternFeatures.add(pat)` (`[PB-CIRCULAR-PATTERN]`,
`[CYCLOIDAL-F-RING-PINS]`).

The pattern steps by exactly `2*pi/N`, which carries one sector's spoke onto its neighbour's only
because S23 put the contour ends exactly on `±pi/N`.

decad has no pattern feature and refuses the boolean that would join face-to-face copies, for the
reason S09 gives, so the tiling those N sectors have to satisfy is read in S26 instead;
`proof/cycloidal/solids_test.go` records it at `stepJoinCasingSectors`.

**From:** `spec/cycloidal/instructions.md` L541–547; `spec/cycloidal/fusion.md` L303–310

## S26 `[GO]` Join the N casing sectors into one casing body

Collect the sectors from the `base` recorded in S24 — `component.bRepBodies.item(base)` through
`component.bRepBodies.item(base + N - 1)` — then
`target = component.bRepBodies.item(base)`; `tools = adsk.core.ObjectCollection.create()` holding
the rest; `ci = component.features.combineFeatures.createInput(target, tools)`;
`ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`;
`component.features.combineFeatures.add(ci)` (`[PB-PATTERN-BODIES]`, `[CYCLOIDAL-F-RING-PINS]`).
The section ends fall on valley midpoints, tangential by symmetry, so the joined inner wall is
smooth.

**What the proof holds.** `stepJoinCasingSectors` builds the whole casing ring — the outer circle
with the contour turned through all N pitches as its one hole — in a single extrude, reads it as
one lump with no voids, and reads its volume both against the ring's own area and, against a seed
sector built in a scratch document, as exactly `N` of them. A contour emitted at bin centres leaves
a seam gap, and a seam gap is exactly what that pair of readings refuses.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepJoinCasingSectors, assertJoinCasingSectors) -->`

**From:** `spec/cycloidal/instructions.md` L541–547; `spec/cycloidal/fusion.md` L303–310

## S27 `[GO]` Combine the casing into the housing base — one `Housing` body

With the casing body as the tool and the base as the target (`[CYCLOIDAL-F-RING-PINS]`):
`tools = adsk.core.ObjectCollection.create()` holding the casing body;
`ci = component.features.combineFeatures.createInput(self.housingRing, tools)`;
`ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`;
`component.features.combineFeatures.add(ci)`. Then `self.housingRing.name = 'Housing'`; keep
`self.housingRing` as the combined body and set `self.ringCasing = None`, since the casing is
consumed by the Join and S35 skips a `None` casing so the housing is chamfered once.

The casing's bottom face is coincident with the base's top face, so the result is one connected
solid spanning `[−1 mm − BaseThickness, stackTop]`: a floor to mount, the scalloped reaction wall
around the disc stack, one printed part.

**What the proof holds.** `stepCombineHousing` unions the base and the ring and reads one lump, the
joined volume, and a box spanning `−1 − BaseThickness` to the stack top. It also checks the fact
the volume arithmetic rests on — that the casing's footprint lies wholly inside the base annulus,
so no contour point falls inside the base's inner lip. Two substitutions are stated at the call:
the two bodies overlap axially by a hundredth of a millimetre rather than abutting, and the base's
outer wall is grown by the same amount so the two outer cylinders cross instead of coinciding.
decad refuses both an abutting pair and a tangent one.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCombineHousing, assertCombineHousing) -->`

**From:** `spec/cycloidal/instructions.md` L548–555; `spec/cycloidal/fusion.md` L311–318

## S28 `[PROSE]` Construction plane `Output Plate Plane`, 1 mm above the disc stack

`buildOutputPins()` starts here, after the cam and the housing (`[CYCLOIDAL-F-OUTPUT-PINS]`,
`[PB-CONSTRUCTION-PLANES]`).

`planeInput = component.constructionPlanes.createInput()`;
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm'))`;
`platePlane = component.constructionPlanes.add(planeInput)`;
`platePlane.name = 'Output Plate Plane'`.

`buildOutputPins` is this generator's own method.

<!-- check-compile: ignore buildOutputPins -->

`stackTopExpr` is S05's prefixed string, so this is the top disc's top face plus 1 mm, and it is the
mirror of the housing plane: the positive offset is toward the disc side, so on this plane
`PositiveExtentDirection` points **away** from the disc and `Negative` points toward it.

The plane's only observable effect is where the plate and the pin sit, which
`proof/cycloidal/solids_test.go` reads directly as their z faces, so this step carries no proof
function of its own.

**From:** `spec/cycloidal/instructions.md` L581–589; `spec/cycloidal/fusion.md` L363–376

## S29 `[GO]` Sketch `Output Plate` — plate outer, pin circle and one output pin

On `platePlane`, named `'Output Plate'`, anchored to `O` exactly as S20 does
(`[CYCLOIDAL-F-OUTPUT-PINS]`, `[CYCLOIDAL-F-ANCHOR-CHAIN]`). Every dimension below is driving;
never pass `isDriven` (`[PB-DRIVING-DIM]`), and every diameter dimension's text point sits off the
circle's centre (`[PB-RADIAL-DIM]`).

- **Plate outer**, solid, on `O`:
  `addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), OutputPlateDiameter / 2)`, centre coincident
  to `localOrigin`, driving diameter dimension with
  `.parameter.expression = self.parameterName(PARAM_OUTPUT_PLATE_DIAMETER)`.
- **Output-pin circle**, construction, on `O`:
  `addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), Rop)`, `isConstruction = True`, centre
  coincident to `localOrigin`, driving diameter dimension with
  `.parameter.expression = self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)`. Keep the handle.
- **One output pin**, solid, at `(Rop, 0)`:
  `addByCenterRadius(adsk.core.Point3D.create(Rop, 0, 0), D_pin / 2)`, driving diameter dimension
  with
  `.parameter.expression = '{} - 2 * {}'.format(self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER), self.parameterName(PARAM_ECCENTRICITY))`,
  which is `D_pin`. Pin its centre with
  `sketch.geometricConstraints.addCoincident(pin.centerSketchPoint, outPinCircle)` and a horizontal
  construction line from `localOrigin` to `pin.centerSketchPoint`
  (`addByTwoPoints`, `isConstruction = True`, `sketch.geometricConstraints.addHorizontal`).

The pin sits inside the plate disc and splits it, which is what S30 and S31 select on. The plate
covers the outermost pin by `Wall`, since `OutputPlateDiameter = 2 * Rop + D_pin + 2 * Wall`.

**What the proof holds.** `stepOutputPlateSketch` builds the sketch, gates it on the engine's
verdict, measures the pin centre, its seating radius against the construction circle's own radius,
its radius against `D_pin / 2` and the plate's cover past the outermost pin against `Wall`. It then
reads the two regions the pin splits the plate into — the pin disc with no hole and the plate with
one — and measures both areas. The seating is a signed horizontal distance for the reason S11
gives, and the proof file records it.

`<!-- proof-run: proofkit.RunParallel(sketchCases, stepOutputPlateSketch) -->`

**From:** `spec/cycloidal/instructions.md` L590–594; `spec/cycloidal/fusion.md` L377–386

## S30 `[GO]` Extrude the output plate away from the disc — `Output Plate`

Take **every** profile of the sketch, so the pin's footprint is solid
(`[CYCLOIDAL-F-OUTPUT-PINS]`):
`coll = adsk.core.ObjectCollection.create()` holding each `sketch.profiles.item(i)`;
`ext = component.features.extrudeFeatures.createInput(coll, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
`ext.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))), adsk.fusion.ExtentDirections.PositiveExtentDirection)`;
`component.features.extrudeFeatures.add(ext)`; name the body `'Output Plate'` and stash
`self.outputPlate`.

The plate is above the disc, so the positive direction is away from it.

Record `pinBase = component.bRepBodies.count` **before** S31's extrude; S34 needs it.

**What the proof holds.** `stepExtrudeOutputPlate` extrudes the plate disc — which is what the
plate-with-bite and the pin disc come to together — and reads its volume against
`pi * (OutputPlateDiameter/2)^2 * OutputPlateThickness` and its box spanning the stack top plus
1 mm to that plus the plate thickness.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepExtrudeOutputPlate, assertExtrudeOutputPlate) -->`

**From:** `spec/cycloidal/instructions.md` L595–598; `spec/cycloidal/fusion.md` L387–392

## S31 `[GO]` Extrude the output pin two-sided — `Output Pin`

Select the pin disc by `profileLoops.count == 1` and by that loop's curve being the pin circle
itself (`[PB-PROFILE-MATCH]`, `[CYCLOIDAL-F-OUTPUT-PINS]`). An any-loop-contains search returns the
surrounding plate ring instead.

`pinExt = component.features.extrudeFeatures.createInput(pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
`pinExt.setTwoSidesExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))), adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm')))`;
`pinFeature = component.features.extrudeFeatures.add(pinExt)`; name the body `'Output Pin'` and keep
`pinBody = pinFeature.bodies.item(0)`.

Side one is away from the disc, into the plate, and is the plate thickness. Side two is toward the
disc and is `stackTopExpr + ' + 1 mm'`, which lands the pin's lower end on disc 0's bottom face at
`z = 0`, so one pin threads every disc's output holes.

**What the proof holds.** `stepExtrudeOutputPin` extrudes the same two-sided cylinder and reads its
volume against `pi * (D_pin/2)^2 * (OutputPlateThickness + stackTop + 1)` and its box, whose lower
face is `z = 0` — a pin that stopped short would miss the lower disc of a two-disc stack. It also
checks the relation the orbit rests on: the output hole is wider than its pin by exactly `2E`.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepExtrudeOutputPin, assertExtrudeOutputPin) -->`

**From:** `spec/cycloidal/instructions.md` L599–603; `spec/cycloidal/fusion.md` L393–402

## S32 `[GO]` Combine-Cut the pin's socket in the plate, keeping the pin

Keeping the tool is what leaves the pin in place (`[CYCLOIDAL-F-OUTPUT-PINS]`):
`tools = adsk.core.ObjectCollection.create()`; `tools.add(pinBody)`;
`ci = component.features.combineFeatures.createInput(self.outputPlate, tools)`;
`ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation`;
`ci.isKeepToolBodies = True`;
`combineFeature = component.features.combineFeatures.add(ci)`. Keep the `CombineFeature`; S34
patterns it.

`isKeepToolBodies` is what leaves the pin seated in the matching hole instead of consuming it.

**What the proof holds.** `stepCutPinSocket` cuts the socket and reads the plate's volume against
`pi * (plateRadius^2 − (D_pin/2)^2) * OutputPlateThickness`, which is the plate less exactly one
pin footprint. Two substitutions are stated at the call: the tool is a cylinder spanning the plate
alone, run past both of its faces, rather than the pin body itself, whose cap sits inside decad's
chord tolerance of the plate's and whose extra length below the plate removes nothing; and the pin
is not left live beside the plate, because decad judges every pair of live bodies and cannot
classify a touching one. The pin's own geometry is proven by S31.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepCutPinSocket, assertCutPinSocket) -->`

**From:** `spec/cycloidal/instructions.md` L604–605; `spec/cycloidal/fusion.md` L403–405

## S33 `[GO]` Chamfer the output pin's two ends

`chamferFeature = self._chamferCapRims(pinBody)`, before the pattern, so S34 carries the chamfer
onto every copy. It returns `None` when `Chamfer Size` is 0, and may return `None` when the chamfer
will not compute; keep whatever it returns.

`_chamferCapRims(self, body)` is the one helper both this step and S35 use
(`[CYCLOIDAL-F-CHAMFERS]`):

- return `None` when `self.chamferSize <= 0`;
- `axis = self.plane.geometry.normal` and `ref = self.plane.geometry.origin`;
- first pass, collect the cap faces with their axial heights: for each `face` in `body.faces`, skip
  unless `face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType`; take
  `n = face.geometry.normal` and skip unless `abs(n.dotProduct(axis)) > 0.999`; record the axial
  height `h`, the dot of `face.geometry.origin - ref` with `axis`;
- return `None` when nothing was collected; otherwise take `hmin` and `hmax` over what was;
- second pass, chamfer **only** the two axially extreme caps, `h` within about `1e-4` cm of `hmin`
  or of `hmax`, and within those only each loop with `loop.isOuter`, adding every edge of it into
  `edges = adsk.core.ObjectCollection.create()`;
- return `None` when `edges.count == 0`;
- `chamferFeatures = self.getComponent().features.chamferFeatures` — hold the collection under its
  own name, so the call below is made against a `ChamferFeatures`, which is the class that declares
  `createInput2`; `ci = chamferFeatures.createInput2()`;
  `ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByString(self.parameterName(PARAM_CHAMFER_SIZE)), False)`;
  `return chamferFeatures.add(ci)` (`[PB-FILLET-CHAMFER]`).

Wrap the `chamferFeatures.add(ci)` call in `try/except` with `futil.log(<reason>)` on failure: on failure log the reason with `futil.log`,
increment `self.chamfersSkipped` and return `None`, never re-raise. A chamfer too large for the
geometry raises `RuntimeError` from Fusion, and the part is already built.

The extreme-height filter is what keeps the combined `Housing` buildable in S35 — it has an
internal ledge at the base-to-casing junction whose outer loop is the scalloped contour, and
chamfering that throws `ASM_BL_CAP_COMPLEX`. A uniform pin, plate or disc has exactly two cap
faces, both extreme, so the filter changes nothing for them.

**What the proof holds.** `stepChamferPinEnds` chamfers the pin's two circular rims and reads the
volume against the pin less two chamfer rings, each the solid of revolution of a right triangle
with legs `Chamfer Size` — by Pappus, `(s^2/2) * 2*pi*(r − s/3)` at the pin's radius `r`. With
`Chamfer Size` at 0 it reads the pin through unchanged, which is the branch this step returns
`None` on. It also refuses a chamfer at or past the pin's own radius, which is the pin-end case of
the resilient-chamfer rule.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepChamferPinEnds, assertChamferPinEnds) -->`

**From:** `spec/cycloidal/instructions.md` L606–607, L618–645; `spec/cycloidal/fusion.md` L417–460

## S34 `[GO]` Circular-pattern the pin, its socket and its chamfer ×M about the Drive Axis

`coll = adsk.core.ObjectCollection.create()`; `coll.add(pinFeature)`; `coll.add(combineFeature)`;
and `coll.add(chamferFeature)` only when S33 returned one;
`pat = component.features.circularPatternFeatures.createInput(coll, self.driveAxis)`;
`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`;
`pat.quantity = adsk.core.ValueInput.createByReal(M)`;
`pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')`; `pat.isSymmetric = False`;
`component.features.circularPatternFeatures.add(pat)` (`[PB-CIRCULAR-PATTERN]`,
`[PB-PATTERN-BODIES]`, `[CYCLOIDAL-F-OUTPUT-PINS]`).

Then name all `M` pin bodies so S36 can group them. The socket Cut adds no body, so the pins are
the contiguous block `component.bRepBodies.item(pinBase)` through
`component.bRepBodies.item(pinBase + M - 1)`, with `pinBase` recorded in S30. Rename each:
`body.name = 'Output Pin {}'.format(k + 1)` for `k` in `0 … M - 1`, which overwrites the seed pin's
`'Output Pin'` with `'Output Pin 1'`.

The pins sit on `O` and the holes on `Od`, both starting at the `+X` point, so each pin sits in its
hole offset by `E`.

**What the proof holds.** `stepPatternOutputPins` builds the M pins on the output-pin circle about
`O` and reads each one's volume and box under its own `Output Pin k` name, so a failure says which
pin is wrong. It then checks that neighbouring pins keep a real gap — the chord between them
exceeds `D_pin` — which is what the output holes' non-overlap bound buys. The M pins are built as
M separate bodies rather than patterned from one, since decad has no pattern feature; they are
disjoint, so the document's own pairwise verdict covers them.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepPatternOutputPins, assertPatternOutputPins) -->`

**From:** `spec/cycloidal/instructions.md` L608–616; `spec/cycloidal/fusion.md` L406–415

## S35 `[GO]` Chamfer the outer rim of every disc, the Housing and the Output Plate

`buildChamfers` is this generator's own method.

<!-- check-compile: ignore buildChamfers -->

`buildChamfers()` does nothing when `Chamfer Size` is 0 (`[CYCLOIDAL-F-CHAMFERS]`,
`[PB-FILLET-CHAMFER]`). Otherwise it calls
`self._chamferCapRims(body)` — the helper S33 describes in full — once for each of: every body in
`self.diskBodies`, then `self.housingRing`, then `self.outputPlate`. Guard each against `None`
before calling: `self.ringCasing` is `None` after S27 consumed it into the housing, so the housing
is chamfered exactly once through `self.housingRing`.

Only the outer loop of each extreme cap is chamfered, so bores, output holes, sockets and the
casing's inner contour stay sharp. The rotor disc's outer loop is its lobe profile, which a chamfer
follows; keep `Chamfer Size` well under the lobe size or it self-intersects at the tight valleys.
The output pins were already chamfered in S33 and the casing's bumps are integral, so neither needs
anything here.

**What the proof holds.** `stepChamferRims` chamfers the Output Plate's two circular rims and reads
the volume against the plate less two Pappus rings, and reads the plate through unchanged when
`Chamfer Size` is 0. The rotor disc's rim and the Housing's are not reachable: decad refuses a
cap-loop chamfer whose corner offset it cannot enclose, which a lobe valley and a contour seam both
are. `proof/cycloidal/solids_test.go` records that beside the chamfer it does build, and it is the
same geometry the spec's resilient-chamfer rule exists for — only a Fusion session decides where a
lobe valley stops accepting one.

`<!-- proof-run: proofkit3d.RunSolidParallel(solidCases, stepChamferRims, assertChamferRims) -->`

**From:** `spec/cycloidal/instructions.md` L618–645; `spec/cycloidal/fusion.md` L417–460

## S36 `[PROSE]` Organize the bodies into four sub-components and hide the construction geometry

`buildSubComponents` is this generator's own method.

<!-- check-compile: ignore buildSubComponents -->

`buildSubComponents()` runs last, after every body exists and every chamfer is done, because
`moveToComponent` invalidates the moved body's reference and the earlier steps still need the
bodies in the Cycloidal Drive component (`[CYCLOIDAL-F-SUBCOMPONENTS]`).

Snapshot first, move second. Walk `component.bRepBodies` **once** and bucket each body by name into
four Python lists, because moving mutates that collection and a loop over it would skip bodies:

- name starts with `'Cycloidal Disk'` → **`Rotor Discs`**;
- name equals `'Housing'` → **`Housing`**;
- name equals `'Eccentric Cam'` → **`Eccentric Cam`**;
- name equals `'Output Plate'` or starts with `'Output Pin'` → **`Output`**.

Then, for each non-empty group in that order:
`occ = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())` — an identity transform,
so each body keeps its world position — `occ.component.name = <group name>`, and
`body.moveToComponent(occ)` for each body in the list. Ignore the return; the body has already
moved, and the pre-move reference must not be reused.

Finally call `solids.hide_construction_geometry(component)`, the shared helper, which walks the
component and its new sub-occurrences and turns off the light bulb on every sketch, construction
plane and construction axis, leaving only the solid bodies visible (`[PB-TREE-CLEANUP]`,
`[PB-HIDE-AFTER-USE]`). Do not re-implement it.

Sub-components, occurrences and visibility are browser-tree state with no geometry of their own,
and `moveToComponent` preserves each body's world position, which is the one thing that could
change and the thing neither harness has an occurrence tree to represent. That is why this step is
`[PROSE]`; `proof/cycloidal/solids_test.go` reads each finished body's world box in the step that
builds it, which is the position the move has to preserve.

**From:** `spec/cycloidal/instructions.md` L647–667; `spec/cycloidal/fusion.md` L508–544
