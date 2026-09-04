# Cycloidal drive — compiled step list

The proof for this step list is `proof/cycloidal/geometry_test.go`, `proof/cycloidal/sketches_test.go`,
`proof/cycloidal/solids_test.go` and the generated `proof/cycloidal/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/cycloidal/instructions.md` | `64188398aca7e6c9244b1d23d654ad85cdc82c42` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/cycloidal/epitrochoid-trace.md` | `2dd150ac312ca9c673812661e4fa229df433dade` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `1b3078d6767d6a3f56c228e1e934c82ccfbf53fe` |

## S01 `[PROSE]` Add the dialog inputs

`CycloidalDriveCommandInputsConfigurator.configure(cls, command)` adds every input below to
`command.commandInputs`, in exactly this order. There is no `handle_input_changed` — nothing in this
dialog is conditionally visible.

Two selections come first (`[PB-AUTOFOCUS-FIRST]`: Fusion focuses the first `SelectionCommandInput`
and ignores a later focus flag, so Target Plane owns the initial focus by being added first), then
every value and dropdown input, then Parent Component last.

The whole table, verbatim. Column 3 is the `unitType` string passed to `addValueInput`; column 4 is
the default in **display** units, which is registered as
`adsk.core.ValueInput.createByReal(to_cm(<default>))` for a length and
`adsk.core.ValueInput.createByReal(<default>)` for a count, since a `createByReal` default is read in
Fusion's internal units whatever the unit string says (`[PB-DIALOG-DEFAULT-UNITS]`).

| # | label | input id | unit | default | kind | registered parameter |
|---|---|---|---|---|---|---|
| 1 | `Target Plane` | `plane` | — | — | selection | — |
| 2 | `Anchor Point` | `anchorPoint` | — | — | selection | — |
| 3 | `Disc Count` | `discCount` | — | `'1'` | dropdown | — |
| 4 | `Pin Count` | `pinCount` | `''` | `16` | value | `PinCount` |
| 5 | `Pin Circle Diameter` | `pinCircleDiameter` | `'mm'` | `90` | value | `PinCircleDiameter` |
| 6 | `Pin Diameter` | `pinDiameter` | `'mm'` | `0` | value | `PinDiameter` |
| 7 | `Eccentricity` | `eccentricity` | `'mm'` | `1.5` | value | `Eccentricity` |
| 8 | `Disk Clearance` | `diskClearance` | `'mm'` | `0.3` | value | `DiskClearance` |
| 9 | `Disc Thickness` | `discThickness` | `'mm'` | `8` | value | `DiscThickness` |
| 10 | `Disc Gap` | `discGap` | `'mm'` | `0.5` | value | `DiscGap` |
| 11 | `Center Bearing Diameter` | `centerBearingDiameter` | `'mm'` | `30` | value | `CenterBearingDiameter` |
| 12 | `Input Shaft Diameter` | `inputShaftDiameter` | `'mm'` | `8` | value | `InputShaftDiameter` |
| 13 | `Bearing Clearance` | `bearingClearance` | `'mm'` | `0.2` | value | `BearingClearance` |
| 14 | `Output Pin Circle Diameter` | `outputPinCircleDiameter` | `'mm'` | `50` | value | `OutputPinCircleDiameter` |
| 15 | `Output Pin Count` | `outputPinCount` | `''` | `6` | value | `OutputPinCount` |
| 16 | `Output Pin Diameter` | `outputPinDiameter` | `'mm'` | `0` | value | `OutputPinDiameter` |
| 17 | `Housing Wall` | `wall` | `'mm'` | `3` | value | `Wall` |
| 18 | `Base Thickness` | `baseThickness` | `'mm'` | `5` | value | `BaseThickness` |
| 19 | `Output Plate Thickness` | `outputPlateThickness` | `'mm'` | `5` | value | `OutputPlateThickness` |
| 20 | `Chamfer Size` | `chamferSize` | `'mm'` | `0.5` | value | `ChamferSize` |
| 21 | `Parent Component` | `parentComponent` | — | root component | selection | — |

Selections are `inputs.addSelectionInput(<id>, <label>, <label>)`, then their filters as the named
enum constants, never quoted literals (`[PB-SELECTION-FILTER-ENUM]`), then their limits. The spec
states no tooltip text for any input, so the label is passed as the command prompt too (SPEC GAP:
the tooltip strings are not written down anywhere). Filters and limits per input
(`[PB-SELECTION-DECL]`):

- `plane`: `input.addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)` and
  `input.addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)`;
  `input.setSelectionLimits(1, 1)`.
- `anchorPoint`: `adsk.core.SelectionCommandInput.ConstructionPoints` and
  `adsk.core.SelectionCommandInput.SketchPoints`; `input.setSelectionLimits(1, 1)`.
- `parentComponent`: `adsk.core.SelectionCommandInput.Occurrences` and
  `adsk.core.SelectionCommandInput.RootComponents`; `input.setSelectionLimits(0, 1)` — empty is
  allowed — with the root pre-selected by `input.addSelection(get_design().rootComponent)`.

Disc Count is a dropdown, not a value input:
`inputs.addDropDownCommandInput('discCount', 'Disc Count', adsk.core.DropDownStyles.TextListDropDownStyle)`,
then `dropdown.listItems.add('1', True)` and `dropdown.listItems.add('2', False)`. `'1'` is a single
disc; `'2'` is two discs 180 degrees opposed, which requires an even `Pin Count` and an even
`Output Pin Count` (S03).

Immediately **after each value and dropdown input** — rows 3 to 20, never after a selection — add its
hidden per-field message slot with the exact signature
`inputs.addTextBoxCommandInput(<id> + '__status', '', '', 2, True)` (id, empty label, empty initial
formatted text, 2 rows, read-only), then set `slot.isVisible = False`. Their ids are the input's id
plus the suffix `'__status'`, for example `pinCircleDiameter__status`. They are not registered
parameters and no `get_*` helper ever reads them; the shared handler writes into them
(`[PB-VALIDATE-INPUTS]`).

`configure` is the method the framework calls on this class rather than a call this module makes,
and `handle_input_changed` is named only to say the class does not define one.

<!-- check-step-calls: ignore configure handle_input_changed -->

**From:** `spec/cycloidal/instructions.md` L12-19 L60-124 L154-216, `.claude/skills/generate-gear/PLAYBOOK.md` L53-60 L128-143 L317-348 L524-535

## S02 `[PROSE]` Read the inputs and register the parameters

`CycloidalDriveGenerator(base.Generator)` overrides `prefixBase` to return `'CycloidalDrive'`, so
every parameter registers under `CycloidalDrive<N>_`. It declares no generation-context class; the
handles live on `self`, and the per-disc ones are **lists** indexed by the disc index `d`:
`self.diskBodies`, `self.diskAxes`, `self.lobeSplines`, `self.outputHoles`, `self.lobeDiskCentres`,
`self.discPlanes`. The rest are scalars: `self.driveAxis`, `self.housingRing`, `self.ringCasing`,
`self.cam`, `self.outputPlate`, and `self.lobePinCircle` (disc 0's pin circle, stashed and never
read — legacy of the pinned-ring design).

`processInputs(inputs)` runs in this fixed order, because creating the occurrence shifts Fusion's
active component and drops selections (`[PB-SELECTION-STASH]`):

1. Pull **all three selections first**, before anything registers a parameter:
   `get_selection(inputs, 'parentComponent')` — empty falls back to
   `get_design().rootComponent`, and an `adsk.fusion.Occurrence` selection resolves through its
   `.component`; then `get_selection(inputs, 'plane')` into `self.plane` and
   `get_selection(inputs, 'anchorPoint')` into `self.anchorPoint`.
2. Read Disc Count from the dropdown, never with a `get_*` helper (`[PB-INPUT-READ]`):
   `int(inputs.itemById('discCount').selectedItem.name)` into `D`.
3. Read every value input with `get_value(inputs, <id>, <unit>)` and register it with
   `self.addParameter(<name>, <value>, <unit>, <comment>)`. `get_value` already returns a
   `ValueInput` ready to hand straight to `addParameter` (`[PB-GET-VALUE-CONTRACT]`). The two counts
   use the unitless string: `get_value(inputs, 'pinCount', '')` and
   `get_value(inputs, 'outputPinCount', '')`, registered with unit `''`; the Python formulas
   read the counts from the dialog value rounded to int, because a Fusion user parameter is a float.
4. Call `self._resolveDimensions()`, which computes the derived scalars and raises
   `Exception('\n'.join(problems))` when `evaluate_problems` returns any (S03).
5. Register the derived parameters, after the resolve, because two of them are snapshots of what it
   stashed.

The derived scalars, in internal cm, are `Lobes = N - 1`; `PinCircleRadius`;
`OutputPinCircleRadius`; `PinRadius` = `Pin Diameter / 2` when Pin Diameter is greater than zero and
otherwise `0.5 * (E + R * sin(pi / N))`; `Rr_eff = PinRadius + DiskClearance`;
`Rv = PinCircleRadius - Rr_eff - Eccentricity`; `D_pin` = `Output Pin Diameter` when that is greater
than zero and otherwise `OutputPinCircleRadius * sin(pi / M) - E`; `OutputHoleDiameter = D_pin + 2E`;
`HousingInnerDiameter = 2 * (R - PinRadius - Wall)`;
`HousingOuterDiameter = 2 * (R - PinRadius + 2 * E + Wall)`;
`OutputPlateDiameter = OutputPinCircleDiameter + D_pin + 2 * Wall`.

Registration mode is fixed per parameter and is not a free choice:

- `PinRadius` and `OutputHoleDiameter` are **numeric snapshots** of the resolved Python values:
  `self.addParameter(PARAM_PIN_RADIUS, adsk.core.ValueInput.createByReal(Rr), 'mm', ...)` and the
  same with `createByReal(D_hole)`. Both resolve through an auto-versus-override branch in Python
  whose auto arm uses `sin(pi / N)`, which a live Fusion expression cannot reproduce
  (`[PB-NUMERIC-SNAPSHOT]`).
- `Lobes`, `PinCircleRadius`, `OutputPinCircleRadius`, `HousingInnerDiameter`,
  `HousingOuterDiameter` and `OutputPlateDiameter` stay live expressions registered with
  `adsk.core.ValueInput.createByString(...)`; they compose cleanly from the registered inputs,
  including the snapshot `PinRadius`.

Also initialise `self.chamfersSkipped = 0` here, before any build step, since S34 and S36 both count
into it.

Every parameter name written into any expression string — a dimension's expression, a construction
plane's offset, an extrude's extent — must be the **prefixed** name from
`self.parameterName(PARAM_...)`. A bare `'DiscThickness'` in a `createByString` raises
`RuntimeError: invalid expression`, because the registered name is `CycloidalDrive<N>_DiscThickness`.

`processInputs` and `_resolveDimensions` are methods this class defines for the framework and for
itself; they are named here, not required as API calls.

<!-- check-step-calls: ignore processInputs -->
<!-- check-compile: ignore processInputs -->

**From:** `spec/cycloidal/instructions.md` L20-38 L125-152 L183-216 L331-348 L382-385 L636-639, `spec/cycloidal/epitrochoid-trace.md` L40-54, `.claude/skills/generate-gear/PLAYBOOK.md` L75-126 L196-228 L624-625

## S03 `[PROSE]` Validate the geometry live and at execute time

All the validity math lives in one module-level helper, `evaluate_problems`, taking the resolved
scalars in cm and ints and returning a list of problem strings. Two callers share it and neither
duplicates a formula (`[PB-VALIDATE-INPUTS]`):

- `@staticmethod validate_inputs(inputs) -> list[str]` on `CycloidalDriveGenerator`, a pure check
  that writes nothing to the document. It reads the raw values straight off the inputs —
  `inputs.itemById(<id>).value` for a length, the same rounded to int for a count,
  `inputs.itemById('discCount').selectedItem.name` for the dropdown — resolves `Rr`, `Rr_eff`, `Rv`,
  `D_pin` and `D_hole` by the same auto-versus-override rules as S02, and returns the problems. It
  runs on every keystroke, and its cost — a 2000-point curvature scan, plus a 40-iteration bisection
  that re-runs that scan each time when the undercut guard fails — is accepted as it stands: do not
  cache it and do not downsample. When a value cannot be read yet, let the read raise; the shared
  handler catches it and treats the inputs as provisionally valid.
- `_resolveDimensions`, at execute time, builds the same values from the registered parameters and
  raises `Exception('\n'.join(problems))` when the list is not empty.

The class also declares
`DEFAULT_STATUS_INPUT_ID = INPUT_ID_PIN_CIRCLE_DIAMETER + '__status'`, the fallback slot the shared
handler writes into when the last-edited input has none of its own.

The checks, in this order, with the message each failure returns. Return **every** failing message,
not just the first. Symbols are the resolved internal-cm values: `R = PinCircleDiameter / 2`,
`Rop = OutputPinCircleDiameter / 2`, `E`, `c`, `N`, `M`, `CBD = CenterBearingDiameter`,
`clr = BearingClearance`, `ISD = InputShaftDiameter`, `Rr`, `Rr_eff = Rr + c`, `Rv = R - Rr_eff - E`,
`D_pin`, `D_hole = D_pin + 2E`. Every number in a message is formatted in **mm** through
`to_mm` and rounded to about two decimals; counts print as integers.

| # | must hold | message when it fails |
|---|---|---|
| 1 | 2 discs implies `N` even **and** `M` even | `Two discs require an even Pin Count and an even Output Pin Count (currently N=…, M=…).` |
| 2 | `E < Rr < R*sin(pi/N)` | auto (`Pin Diameter == 0`): `Pin geometry out of range — increase Pin Circle Diameter above {2E/sin(pi/N)} mm or reduce Eccentricity below {R*sin(pi/N)} mm.` override: `Pin Diameter must be between {2E} mm and {2*R*sin(pi/N)} mm (currently {2*Rr}).` |
| 3 | `D_pin > 0` | auto: `Output pins vanish (resolved diameter ≤ 0) — increase Output Pin Circle Diameter above {2E/sin(pi/M)} mm, increase Output Pin Count, or reduce Eccentricity.` override: `Output Pin Diameter must be greater than 0.` |
| 4 | `D_hole < 2*Rop*sin(pi/M)` | `Output holes overlap — increase Output Pin Circle Diameter above {(D_pin+2E)/sin(pi/M)} mm, increase Output Pin Count, or reduce Output Pin Diameter / Eccentricity.` |
| 5 | `E < R/N` | `Eccentricity too large — reduce it below {R/N} mm (or increase Pin Circle Diameter / reduce Pin Count).` |
| 6 | `Rop < Rv` | `Output Pin Circle too large — set Output Pin Circle Diameter below {2*Rv} mm (currently {2*Rop}).` |
| 7 | `Rr_eff < rho_min_O` | `Eccentricity too large — the rotor profile undercuts/self-intersects. Reduce Eccentricity below {E*} mm.` |
| 8 | `ISD < CBD` | `Input Shaft Diameter must be less than Center Bearing Diameter ({CBD} mm).` |
| 9 | `E + ISD/2 < CBD/2` | `Input bore doesn't fit inside the cam — set Input Shaft Diameter below {CBD − 2E} mm, or reduce Eccentricity / increase Center Bearing Diameter.` |
| 10 | `(CBD + clr)/2 < Rop - D_hole/2` | `Disk center bore overlaps the output holes — increase Output Pin Circle Diameter above {CBD + clr + D_hole} mm, or reduce Center Bearing Diameter / Bearing Clearance / output pin size.` |

Check 7's `rho_min_O` is the smallest radius of curvature of the base trochoid at the points whose
centre of curvature lies toward `O`, sampled at exactly **2000** points of `t` over `[0, 2*pi)`:

```
bx  =  R*cos t - E*cos(N t) ;            by  = -R*sin t + E*sin(N t)
xp  = -R*sin t + E*N*sin(N t) ;          yp  = -R*cos t + E*N*cos(N t)
xpp = -R*cos t + E*N^2*cos(N t) ;        ypp =  R*sin t - E*N^2*sin(N t)
k   = xp*ypp - yp*xpp                                  # skip the sample if |k| ~ 0
rho = (xp^2 + yp^2)^1.5 / k
s   = sqrt(xp^2 + yp^2) ;  nx, ny = -yp/s, xp/s
Cx, Cy = bx + rho*nx, by + rho*ny
rho_min_O = min |rho| over the samples where Cx^2 + Cy^2 < bx^2 + by^2
```

`E*` in check 7's message has no closed form. Hold every other input fixed and find the largest `E'`
in `(0, E]` for which `Rr_eff(E') < rho_min_O(E')` still holds, by exactly **40** bisection rounds;
report `to_mm(E*)`. Both sides depend on `E'` when Pin Diameter is 0. If no positive `E'`
satisfies it, fall back to the plain "reduce Eccentricity" wording with no number.

This is the binding eccentricity limit, far tighter than check 5: for the dialog defaults it is about
**2.50 mm** against `R/N = 2.81 mm`. The proof measures that number on the same scan
(`stepRotorLobeSketch`), and every proof case is held under it.

The bullet ranges in the spec's Variables section — "integer >= 4", "integer >= 3", "mm > 0",
"mm >= 0" — are authoring documentation, not checks: `evaluate_problems` does not enforce them, and
this table is the complete list of what it does.

`validate_inputs` and `evaluate_problems` are what this module declares for the shared command
handler to call; `_resolveDimensions` calls `evaluate_problems` itself. `rho_min_O` is the scan
above, written as a function of `E'` in the bisection, and `vanish` is a word inside a message
string, neither of them a call.

<!-- check-step-calls: ignore validate_inputs evaluate_problems rho_min_O vanish -->
<!-- check-compile: ignore validate_inputs evaluate_problems rho_min_O vanish -->

**From:** `spec/cycloidal/instructions.md` L63-67 L142-152 L218-273, `spec/cycloidal/epitrochoid-trace.md` L56-84, `.claude/skills/generate-gear/PLAYBOOK.md` L317-344

## S04 `[PROSE]` Create the component and name it

`generate(inputs)` calls `processInputs(inputs)` first, then takes the component with
`self.getComponent()` and sets `component.name = self.generateName()`. `generateName` returns
`'Cycloidal Drive (N={}):{}'.format(N, L)` with `N = PinCount` rounded to int and `L = N - 1`, the
reduction ratio — for the defaults, `'Cycloidal Drive (N=16):15'`.

Never call `occurrence.activate()` anywhere in this build (`[PB-NEVER-ACTIVATE]`): activating a
sub-occurrence makes Fusion resolve the user's externally selected plane in the local frame and the
whole drive lands flat on world XY whatever plane was picked. Every sketch and feature is built by
calling the non-activated component's own collections.

`generate` and `generateName` are this class's own members, named here rather than required as API
calls; `activate` is named only to forbid it.

<!-- check-step-calls: ignore generate generateName activate -->
<!-- check-compile: ignore generateName -->

**From:** `spec/cycloidal/instructions.md` L1-10 L39-58 L349-363 L380 L382-385, `spec/cycloidal/fusion.md` L1-6, `spec/cycloidal/epitrochoid-trace.md` L1-16, `.claude/skills/generate-gear/PLAYBOOK.md` L244-254 L788-795

## S05 `[PROSE]` Normalize the Target Plane

If `self.plane` is not already an `adsk.fusion.ConstructionPlane`, build a coplanar one and use that
instead: `planeInput = component.constructionPlanes.createInput()`, then
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))`, then
`component.constructionPlanes.add(planeInput)`. This is a single-component generator, so normalizing
is safe here (`[PB-CONSTRUCTION-PLANES]`).

**From:** `spec/cycloidal/instructions.md` L421-424 L356, `.claude/skills/generate-gear/PLAYBOOK.md` L244-251 L742-746

## S06 `[PROSE]` Construction plane for disc `d`

The build loops `for d in range(D)` over the disc count. Disc `d` spans `[z_d, z_d + T]` with
`z_d = d * (T + g)`, `T = DiscThickness`, `g = DiscGap`. For `d == 0` the plane is `self.plane`
itself and no construction plane is created. For every later disc, create one:

```
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString(
    '{} * ({} + {})'.format(d, self.parameterName(PARAM_DISC_THICKNESS),
                            self.parameterName(PARAM_DISC_GAP))))
plane = component.constructionPlanes.add(planeInput)
plane.name = 'Disc Plane {}'.format(d + 1)
```

Stash it on `self.discPlanes[d]`. The offset string uses the **prefixed** names; `DiscCount` is a
dropdown, not a parameter, so it never appears in an expression — only as the literal integer `d`.

Every disc's sketches are anchored to `O` on its own plane, and every disc extrude runs in
`adsk.fusion.ExtentDirections.PositiveExtentDirection` from it.

**From:** `spec/cycloidal/instructions.md` L333-347 L387-397 L417-419, `spec/cycloidal/fusion.md` L462-479

## S07 `[GO]` Draw the Rotor Lobe sketch for disc `d`

`buildLobeSketch(d)` creates a sketch named `'Rotor Lobe {}'.format(d + 1)` on `plane(d)` and leaves
it visible, with no bodies built. Proof function: `stepRotorLobeSketch`.

<!-- proof-run: proofkit.Run(discSketchCases, stepRotorLobeSketch) -->

**Anchor chain** (`[CYCLOIDAL-F-ANCHOR-CHAIN]`). Project the user's Anchor with
`sketch.project(self.anchorPoint)` and take `.item(0)`; add a fresh local origin with
`sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))` — not `sketch.originPoint` — and tie the
two with `sketch.geometricConstraints.addCoincident(localOrigin, projected)`. All the geometry below
is drawn relative to that local origin, so anchoring it drags the drawing onto the user's Anchor.

**The eccentric disc centre** (`[CYCLOIDAL-F-DISK-CENTER]`). `Od_d = O + s_d * E * Xhat` with
`s_0 = +1` and `s_1 = -1`:

1. `diskCentre = sketch.sketchPoints.add(adsk.core.Point3D.create(s_d * E, 0, 0))` — the signed
   coordinate, seeded on the side the disc belongs on.
2. `eccLine = sketch.sketchCurves.sketchLines.addByTwoPoints(localOrigin, diskCentre)`
   (`[PB-SKETCHCURVES]`: the curve collections hang off `sketch.sketchCurves`, never off the sketch),
   `eccLine.isConstruction = True`, and
   `sketch.geometricConstraints.addHorizontal(eccLine)`.
3. A driving distance dimension between the two points,
   `sketch.sketchDimensions.addDistanceDimension(localOrigin, diskCentre, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`,
   with its `.parameter.expression` set to `self.parameterName(PARAM_ECCENTRICITY)`. Never pass
   `isDriven=True` (`[PB-DRIVING-DIM]`). The dimension is a **magnitude** and its side comes from
   where `diskCentre` was seeded, so disc 1 is the point at `(-E, 0)` and still the expression
   `Eccentricity`, never a negative value (`[PB-DIM-VALUE-SEMANTICS]`).

**Three reference circles**, in this order (`[CYCLOIDAL-F-DISK-LOBE]`). Each is created with
`sketch.sketchCurves.sketchCircles.addByCenterRadius(<Point3D>, <radius>)` at a fresh centre point,
then `circle.isConstruction = True`, then
`sketch.geometricConstraints.addCoincident(circle.centerSketchPoint, <the centre it belongs to>)` —
share xor coincident, never both (`[PB-SHARE-XOR-COINCIDENT]`) — then a driving
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` whose `.parameter.expression` is
set as below. The text point must sit on or near the circle, never at its centre
(`[PB-RADIAL-DIM]`).

| circle | centre | seed | radius | `.parameter.expression` | label |
|---|---|---|---|---|---|
| pin circle, **disc 0 only** | `localOrigin` (`O`, the fixed ring) | `Point3D.create(0, 0, 0)` | `R` | `PinCircleDiameter` | `'Pin Circle'` |
| output-pin circle | `diskCentre` (`Od`) | `Point3D.create(s_d * E, 0, 0)` | `Rop` | `OutputPinCircleDiameter` | `'Output Pin Circle'` |
| root circle | `diskCentre` (`Od`) | `Point3D.create(s_d * E, 0, 0)` | `Rv` | `2 * (PinCircleRadius - PinRadius - DiskClearance - Eccentricity)` | `'Root Circle'` |

Every logical name in those expressions is substituted through `self.parameterName(PARAM_...)`,
including inside the multi-term one: a bare `'2 * (PinCircleRadius - ...)'` raises
`RuntimeError: 3 : Expression is invalid` because no unprefixed parameter of that name exists. The
output-pin circle is centred on `Od`, concentric with the root circle and **not** with the pin
circle, and it is the innermost of the three (`Rop < Rv`). Stash the pin circle on
`self.lobePinCircle` for `d == 0`; nothing reads it.

Each circle carries an along-path text label, the three-call shape exactly (`[PB-SKETCH-TEXT]`):
`textInput = sketch.sketchTexts.createInput2(<label>, Rr)`, then
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
then `sketch.sketchTexts.add(textInput)`. The height is the resolved `Rr`. Because those labels carry
their own position, this sketch never reports `sketch.isFullyConstrained` as true even though its
geometry is completely determined — log that result, never raise on it
(`[PB-TEXT-HOLDS-DOF]`).

**The lobe.** One open fitted spline through the adaptively sampled points of
`disk_point(t, cx = s_d * E, cy = 0, phi = d * pi)` over `t` in `[0, 2*pi/L]`. The point function,
reproduced exactly, in internal cm — the values come back in cm already and are added as they are,
never re-wrapped in `to_cm`:

```
Rr_eff = Rr + c
num = sin((1 - N) * t)
den = (R / (E * N)) - cos((1 - N) * t)
psi = atan2(num, den)                       # uses R, E, N only — not Rr
x0 =  R*cos(t) - Rr_eff*cos(t + psi) - E*cos(N * t)
y0 = -R*sin(t) + Rr_eff*sin(t + psi) + E*sin(N * t)
x = cx + (x0*cos(phi) - y0*sin(phi))
y = cy + (x0*sin(phi) + y0*cos(phi))
```

Sampling is adaptive and must not be uniform: a uniformly sampled fitted spline overshoots into
rabbit-ear loops where the lobe turns sharply, by about 25 degrees per step near the undercut limit.
Evaluate `disk_point` at exactly **2000** uniform steps (2001 points) over `[0, 2*pi/L]`; keep the
first; accumulate the turn angle between consecutive fine points and keep a point and reset the
accumulator each time the total reaches exactly **5.0 degrees**; always keep the last. That gives
about 30 to 55 points at any valid eccentricity. Add each kept point as
`adsk.core.Point3D.create(x, y, 0)` to an `adsk.core.ObjectCollection.create()` and build the curve
with `sketch.sketchCurves.sketchFittedSplines.add(coll)`. Leave it **open**: never set `isClosed`,
and add no closing arc — the disc is closed later by the pattern.

**Lock the spline.** Fix every **interior** fit point,
`for i in range(1, spline.fitPoints.count - 1): spline.fitPoints.item(i).isFixed = True`, which locks
the lobe's shape, and coincide each **end onto the root circle** —
`sketch.geometricConstraints.addCoincident(spline.fitPoints.item(0), rootCircle)` and
`sketch.geometricConstraints.addCoincident(spline.fitPoints.item(spline.fitPoints.count - 1), rootCircle)`
— which pins each valley's radius to `Rv`. Do **not** fix the whole spline: that makes the angle
dimension below redundant and the solver fails (`[PB-NO-OVERCONSTRAIN]`).

**Two spokes and the lobe pitch.** Spoke 1 runs from the disc centre to the lobe's first point:
`line1 = sketch.sketchCurves.sketchLines.addByTwoPoints(diskCentre, adsk.core.Point3D.create(s_d * E + Rv, 0, 0))`,
sharing `diskCentre` and taking a raw seed at the far end, then
`sketch.geometricConstraints.addCoincident(line1.endSketchPoint, spline.fitPoints.item(0))` and
`sketch.geometricConstraints.addHorizontal(line1)`. Spoke 2 runs to the last point:
`line2 = sketch.sketchCurves.sketchLines.addByTwoPoints(diskCentre, adsk.core.Point3D.create(s_d * E + Rv * cos(2*pi/L), -Rv * sin(2*pi/L), 0))`
then `sketch.geometricConstraints.addCoincident(line2.endSketchPoint, spline.fitPoints.item(spline.fitPoints.count - 1))`,
and no horizontal. Both spoke seeds stay **unrotated** for disc 1 — only the signed `E` is
substituted — and the coincident constraints drag the spokes onto the rotated valleys, because disc
1's first valley at `Od_1 + (-Rv, 0)` still lies on spoke 1's horizontal.

Then one driving angular dimension between the spokes:
`angDim = sketch.sketchDimensions.addAngularDimension(line1, line2, textPoint)` with
`angDim.parameter.expression` set to `'360 deg / {}'.format(self.parameterName(PARAM_LOBES))`. The
text point must lie in the **minor** wedge, below the spokes at the bisector `-pi/L` — for example
`adsk.core.Point3D.create(s_d * E + 0.4 * Rv * cos(pi/L), -0.4 * Rv * sin(pi/L), 0)` — or Fusion
dimensions the reflex angle instead (`[PB-ANGULAR-DIM]`).

Stash `self.lobeDiskCentres[d] = diskCentre` and `self.lobeSplines[d] = spline`. Leave the sketch
visible; consume it in S08 before hiding anything (`[PB-HIDE-AFTER-USE]`).

There is **no** `spec/cycloidal/sketch/` bench for this gear, so `[PB-SKETCH-FIRST]` is waived rather
than satisfied; this proof step is what stands in its place, and it holds the scheme to DOF 0 with no
redundant or conflicting constraint and no discrete ambiguity.

`disk_point` is this module's own point function, and `buildLobeSketch` is the method the call graph
gives this step; both are named here rather than required as API calls.

<!-- check-step-calls: ignore disk_point -->
<!-- check-compile: ignore disk_point buildLobeSketch -->

**From:** `spec/cycloidal/instructions.md` L275-330 L389-416 L426-456, `spec/cycloidal/fusion.md` L8-98 L462-488, `spec/cycloidal/epitrochoid-trace.md` L86-132, `.claude/skills/generate-gear/PLAYBOOK.md` L282-283 L350-359 L438-443 L470-475 L506-522 L581-601 L602-603 L615-623 L626-638 L649-659, and the call graph at `spec/cycloidal/instructions.md` L357-358

## S08 `[GO]` Extrude the lobe sector for disc `d`

`buildDisk(d)` starts by recording `base = component.bRepBodies.count` **before** this extrude, which
S11 needs, and then extrudes the Rotor Lobe sketch's one closed profile: the lobe pie-sector bounded
by spoke 1, the lobe spline and spoke 2 (`[CYCLOIDAL-F-DISK-BODY]`). Proof function:
`stepExtrudeLobeSector`.

<!-- proof-run: proofkit3d.RunSolid(discSolidCases, stepExtrudeLobeSector, assertExtrudeLobeSector) -->

**Select the profile by identity, never by index.** The three along-path text labels add their own
letter outline profiles, so `sketch.profiles.item(0)` is not the sector (`[PB-PROFILE-MATCH]`,
and `[PB-SINGLE-PROFILE]` does not apply because this sketch holds more than one closed region).
Iterate `sketch.profiles` and, for each, scan
`profile.profileLoops` and each loop's `profileCurves` for a `curve.sketchEntity` that **is** the
saved `self.lobeSplines[d]` object, by identity.

Then:

```
ext = component.features.extrudeFeatures.createInput(
    sectorProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
ext.setOneSideExtent(
    adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
        self.parameterName(PARAM_DISC_THICKNESS))),
    adsk.fusion.ExtentDirections.PositiveExtentDirection)
extrude = component.features.extrudeFeatures.add(ext)
```

Name the resulting body `'Cycloidal Disk {}'.format(d + 1)`. The extent is the prefixed
`DiscThickness` parameter by name; the disc therefore spans `[z_d, z_d + T]` from its own plane.

The proof asserts that span exactly, and that the body's volume is the sector profile's area times
the thickness. The sector's apex is the disc centre `Od_d`, which is where both spoke side-faces
meet — the reason S09 may not pick its cap face by proximity to that point.

`buildDisk` is the method the call graph gives this step, named rather than required.

<!-- check-step-calls: ignore buildDisk -->
<!-- check-compile: ignore buildDisk -->

**From:** `spec/cycloidal/instructions.md` L359-360 L458-465 L476-481, `spec/cycloidal/fusion.md` L137-162, `.claude/skills/generate-gear/PLAYBOOK.md` L491-497 L639-648

## S09 `[PROSE]` Construction axis through the disc centre

`buildDiskAxis(capFace, d)`, called from `buildDisk(d)` **after** the extrude, because a face-less
axis is not supported here: `setByLine` with an `InfiniteLine3D` raises
`RuntimeError: 3 : Environment is not supported` in the parametric environment and `activate()` does
not fix it (`[CYCLOIDAL-F-DISK-AXIS]`, `[PB-CONSTRUCTION-AXES]`,
`[PB-CONSTRUCTION-NEEDS-ACTIVE]`).

Take the cap **by normal**, unconditionally `extrude.startFaces.item(0)` — the cap in the sketch
plane. Do **not** search for the planar face nearest to `Od_d`, or the one whose plane contains it,
and do not score faces with `getParameterAtPoint`: the pie-sector's two spoke faces are also planar
and also contain `Od_d`, since it is the sector's apex, so such a search can pick a spoke, whose
normal lies **in** the sketch plane, and the pattern then spins the lobes about a sideways axis.
Either cap gives the same vertical direction; the axis's location comes from the point argument.

```
axInput = component.constructionAxes.createInput()
axInput.setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])
axis = component.constructionAxes.add(axInput)
axis.name = 'Disk Axis {}'.format(d + 1)
```

Stash it on `self.diskAxes[d]`. The cap need not contain `Od_d`; the face supplies only the
direction. `setByPerpendicularAtPoint` works on a non-active component.

`setByLine` and `activate` are named here only to forbid them, and `buildDiskAxis` is this class's
own method.

<!-- check-step-calls: ignore setByLine activate buildDiskAxis getParameterAtPoint -->
<!-- check-compile: ignore buildDiskAxis -->

**From:** `spec/cycloidal/instructions.md` L465-472, `spec/cycloidal/fusion.md` L100-135 L163-165, `.claude/skills/generate-gear/PLAYBOOK.md` L754-766

## S10 `[GO]` Circular-pattern the lobe sector `L` times

Pattern the **extrude feature** of S08 — not its body — `L = N - 1` times about `self.diskAxes[d]`
over a full turn (`[CYCLOIDAL-F-DISK-BODY]`, `[PB-CIRCULAR-PATTERN]`, `[PB-PATTERN-BODIES]`).
Proof function: `stepPatternLobeSectors`.

<!-- proof-run: proofkit3d.RunSolid(discSolidCases, stepPatternLobeSectors, assertPatternLobeSectors) -->

```
coll = adsk.core.ObjectCollection.create()
coll.add(extrude)
pat = component.features.circularPatternFeatures.createInput(coll, self.diskAxes[d])
pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
pat.quantity = adsk.core.ValueInput.createByReal(L)
pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
pat.isSymmetric = False
component.features.circularPatternFeatures.add(pat)
```

All three of quantity, total angle and symmetry are set explicitly rather than left to Fusion's
defaults. `AdjustPatternCompute` is mandatory on every circular pattern in this build
(`[CYCLOIDAL-F-OUTPUT-HOLES]`); the default paste compute copies edges instead of recomputing each
instance and a lone patterned cut fails with
`RuntimeError: 3 … NO_TARGET_BODY … PATTERN_FEATURES_NO_PASTE_INT_EDGES`.

The `L` sectors tile disc `d`. The proof places each instance and checks it lands where a rotation by
`k * 360/L` about `Od_d` puts it, with the seed's own volume, and that `L` steps of that pitch close
the turn exactly — which is what leaves no gap and no overlap between neighbours.

**From:** `spec/cycloidal/instructions.md` L473-475, `spec/cycloidal/fusion.md` L166-173 L230-236, `.claude/skills/generate-gear/PLAYBOOK.md` L660-670

## S11 `[GO]` Join disc `d`'s own `L` sectors into one body

Join **only this disc's** sectors. With two discs, disc 0's body already exists when disc 1 builds,
so `component.bRepBodies.item(0)` is the wrong target. Disc `d`'s sectors are
`component.bRepBodies.item(base)` through `item(base + L - 1)`, where `base` was recorded before S08's
extrude (`[CYCLOIDAL-F-TWO-DISC]`). Proof function: `stepJoinDiskSectors`.

<!-- proof-run: proofkit3d.RunSolid(discSolidCases, stepJoinDiskSectors, assertJoinDiskSectors) -->

```
target = component.bRepBodies.item(base)
tools = adsk.core.ObjectCollection.create()      # item(base + 1) … item(base + L - 1)
ci = component.features.combineFeatures.createInput(target, tools)
ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
component.features.combineFeatures.add(ci)
```

Name the result `'Cycloidal Disk {}'.format(d + 1)` and stash it on `self.diskBodies[d]`; S13, S14 and
S16 all cut into it.

The proof builds the joined disc and checks the two things the Join has to deliver: one connected
lump spanning `[z_d, z_d + T]`, and a volume of exactly `L` times one sector's, so no material is
lost at a seam or counted twice at an overlap. It also counts faces — one per outline chord plus the
two caps — which is how it sees that the `L` pairs of spoke faces have disappeared into the interior
rather than survived as `L` separate bodies.

**From:** `spec/cycloidal/instructions.md` L476-481, `spec/cycloidal/fusion.md` L174-182 L490-493

## S12 `[GO]` Draw the Output Hole sketch for disc `d`

`buildOutputHoleSketch(d)` creates a **new** sketch named `'Output Hole {}'.format(d + 1)` on
`plane(d)`, so the lobe and hole profiles never share a sketch and never interfere
(`[CYCLOIDAL-F-OUTPUT-HOLE]`). Proof function: `stepOutputHoleSketch`.

<!-- proof-run: proofkit.Run(discSketchCases, stepOutputHoleSketch) -->

Anchor a local origin to `O` and rebuild `Od_d` exactly as S07 does — project, coincident, signed
seed point, construction line, `addHorizontal`, and the driving distance dimension whose expression
is `Eccentricity` (`[CYCLOIDAL-F-ANCHOR-CHAIN]`, `[CYCLOIDAL-F-DISK-CENTER]`). Then, on `Od_d`:

1. The **output-hole circle**, construction, radius `Rop`:
   `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(s_d * E, 0, 0), Rop)`,
   `isConstruction = True`, `sketch.geometricConstraints.addCoincident(circle.centerSketchPoint, diskCentre)`,
   a driving `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` whose
   `.parameter.expression` is `self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)`, and an
   along-path label `'Output Hole Circle'` at height `Rr` (`[PB-SKETCH-TEXT]`).
2. **One solid hole**, radius `D_hole / 2` with `D_hole = D_pin + 2E`, seeded on the `+X` ray from
   the disc centre at `adsk.core.Point3D.create(s_d * E + Rop, 0, 0)` — **solid**, so no
   `isConstruction`. Pin its diameter with a driving
   `sketch.sketchDimensions.addDiameterDimension(hole, textPoint)` whose `.parameter.expression` is
   `self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER)`. Pin its position two ways: on the circle with
   `sketch.geometricConstraints.addCoincident(hole.centerSketchPoint, outputHoleCircle)`, and on the
   ray with a construction line
   `sketch.sketchCurves.sketchLines.addByTwoPoints(diskCentre, hole.centerSketchPoint)` plus
   `sketch.geometricConstraints.addHorizontal(spokeLine)`.

The hole spoke is horizontal in the sketch's own `+X` sense and is **not** rotated with the disc's
clocking: only the signed `E` is substituted for disc 1, and the `M`-fold pattern of S14 maps a
half-turned hole set onto itself whenever `M` is even, which two discs require anyway. (SPEC GAP: the
spec says only "just centre on `Od_d`" for disc 1's holes and never states whether the seed follows
the clocking; both readings give the same hole set under the even-`M` gate, and this list takes the
unrotated one.)

Stash the solid hole circle on `self.outputHoles[d]`; S13 selects its profile by identity. Leave the
sketch visible and build no bodies.

<!-- check-step-calls: ignore buildOutputHoleSketch -->
<!-- check-compile: ignore buildOutputHoleSketch -->

**From:** `spec/cycloidal/instructions.md` L361 L483-492, `spec/cycloidal/fusion.md` L184-204, `spec/cycloidal/epitrochoid-trace.md` L25-38 L49-54

## S13 `[GO]` Cut one output hole through disc `d`

`buildOutputHoles(d)` cuts the `Output Hole {d+1}` sketch's solid hole through
`self.diskBodies[d]` (`[CYCLOIDAL-F-OUTPUT-HOLES]`). Proof function: `stepCutOutputHole`.

<!-- proof-run: proofkit3d.RunSolid(discSolidCases, stepCutOutputHole, assertCutOutputHole) -->

Select the hole profile by identity — the profile whose loop contains `self.outputHoles[d]` — not
`sketch.profiles.item(0)`, since the construction circle and the text label contribute other
profiles (`[PB-PROFILE-MATCH]`). Then:

```
ci = component.features.extrudeFeatures.createInput(
    holeProfile, adsk.fusion.FeatureOperations.CutFeatureOperation)
ci.setOneSideExtent(
    adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
        self.parameterName(PARAM_DISC_THICKNESS))),
    adsk.fusion.ExtentDirections.PositiveExtentDirection)
ci.participantBodies = [self.diskBodies[d]]
cut = component.features.extrudeFeatures.add(ci)
```

The sketch sits on `plane(d)` and the disc spans `[z_d, z_d + T]`, so a cut of `DiscThickness` in the
positive direction passes right through it. `participantBodies` restricts the cut to this disc.
`setDistanceExtent` belongs to `HoleFeatureInput` and must not be used on an extrude.

The proof measures the disc before and after and checks the hole removed exactly its own cylinder,
leaving one lump. It also checks that the hole lies wholly inside the root circle — see the spec gap
recorded at `requireHolesInsideRim` in `proof/cycloidal/solids_test.go`, since the validity table
checks only `Rop < Rv` and lets a hole that opens the disc's rim through.

<!-- check-step-calls: ignore setDistanceExtent buildOutputHoles -->
<!-- check-compile: ignore buildOutputHoles -->

**From:** `spec/cycloidal/instructions.md` L362 L494-499, `spec/cycloidal/fusion.md` L206-222

## S14 `[GO]` Circular-pattern the output-hole cut `M` times

Pattern the cut **feature** `M = Output Pin Count` times about `self.diskAxes[d]` over a full turn,
with the same input shape as S10 and `pat.quantity = adsk.core.ValueInput.createByReal(M)`. Proof
function: `stepPatternOutputHoles`.

<!-- proof-run: proofkit3d.RunSolid(discSolidCases, stepPatternOutputHoles, assertPatternOutputHoles) -->

`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute` is not optional
here: this is a lone patterned **cut**, with no body-creating feature to anchor it, and under the
default compute Fusion fails with
`RuntimeError: 3 … NO_TARGET_BODY … PATTERN_FEATURES_NO_PASTE_INT_EDGES` (`[CYCLOIDAL-F-OUTPUT-HOLES]`).

The `M` holes orbit `Od_d`, because the disk axis is at `Od_d`. The proof builds the disc carrying all
`M` holes and checks the volume identity and that every hole centre sits at radius `Rop` from `Od_d`,
one per `M`-th of a turn.

**From:** `spec/cycloidal/instructions.md` L494-502, `spec/cycloidal/fusion.md` L223-236

## S15 `[GO]` Draw the Disc Bore sketch for disc `d`

`buildDiskBore(d)` creates a sketch named `'Disc Bore {}'.format(d + 1)` on `plane(d)`, anchored to
`O`, with `Od_d` rebuilt as in S07 (`[CYCLOIDAL-F-CAM]`, `[CYCLOIDAL-F-ANCHOR-CHAIN]`,
`[CYCLOIDAL-F-DISK-CENTER]`). Proof function: `stepDiscBoreSketch`.

<!-- proof-run: proofkit.Run(discSketchCases, stepDiscBoreSketch) -->

One **solid** circle on `Od_d` of radius `(CenterBearingDiameter + BearingClearance) / 2`:
`sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(s_d * E, 0, 0), (CBD + clr) / 2)`,
its centre coincident to `diskCentre`, and a driving diameter dimension whose
`.parameter.expression` is
`'{} + {}'.format(self.parameterName(PARAM_CENTER_BEARING_DIAMETER), self.parameterName(PARAM_BEARING_CLEARANCE))`.

The bore is concentric with the cam and wider than it by the clearance, so the running gap is the same
all the way round — a bore on `O` instead of `Od_d` would foul the cam on one side. The proof checks
both: the diameter is `CenterBearingDiameter + BearingClearance`, and the bore's centre is the disc
centre.

<!-- check-step-calls: ignore buildDiskBore -->
<!-- check-compile: ignore buildDiskBore -->

**From:** `spec/cycloidal/instructions.md` L363 L557-567, `spec/cycloidal/fusion.md` L320-339, `spec/cycloidal/epitrochoid-trace.md` L17-23

## S16 `[GO]` Cut the centre bore through disc `d`

Cut the Disc Bore sketch through `self.diskBodies[d]`. Proof function: `stepCutDiscBore`.

<!-- proof-run: proofkit3d.RunSolid(discSolidCases, stepCutDiscBore, assertCutDiscBore) -->

Collect **every** profile in that sketch — there is only the one disc — into an
`adsk.core.ObjectCollection.create()` and pass the collection as the profile:

```
ci = component.features.extrudeFeatures.createInput(
    coll, adsk.fusion.FeatureOperations.CutFeatureOperation)
ci.setOneSideExtent(
    adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
        self.parameterName(PARAM_DISC_THICKNESS))),
    adsk.fusion.ExtentDirections.PositiveExtentDirection)
ci.participantBodies = [self.diskBodies[d]]
component.features.extrudeFeatures.add(ci)
```

Each disc ends the per-disc loop with a clean round centre bore, `Bearing Clearance` wider than the
cam. The proof checks the volume removed is that bore's own cylinder and that the radial running gap
is half the Bearing Clearance.

**From:** `spec/cycloidal/instructions.md` L557-567, `spec/cycloidal/fusion.md` L329-339

## S17 `[GO]` Draw the Eccentric Cam section sketch for section `d`

`buildCam()` runs once, after the per-disc loop and before `buildRingPins`, and builds `D` sections.
Section `d` starts with a sketch named `'Eccentric Cam {}'.format(d + 1)` on `plane(d)`, anchored to
`O`, with `Od_d` rebuilt as in S07 (`[CYCLOIDAL-F-CAM]`). Proof function: `stepEccentricCamSketch`.

<!-- proof-run: proofkit.Run(discSketchCases, stepEccentricCamSketch) -->

- **Cam outer circle**, on the disc centre:
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(s_d * E, 0, 0), CBD / 2)`,
  **solid**, centre coincident to `diskCentre`, driving diameter dimension whose
  `.parameter.expression` is `self.parameterName(PARAM_CENTER_BEARING_DIAMETER)`.
- **Input-bore circle, only when `Input Shaft Diameter > 0`**, on the drive axis:
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), ISD / 2)`,
  **solid**, centre coincident to the local origin, driving diameter dimension whose
  `.parameter.expression` is `self.parameterName(PARAM_INPUT_SHAFT_DIAMETER)`. It lies inside the cam
  outer but offset `E` from it, so it splits the cam disc into a small bore disc and the cam annulus.

The `E` offset between the two centres is the eccentricity: the cam outer rides on the disc centre and
the bore runs on the drive axis. The proof builds both branches — bore and no bore — and checks the
cam outer's diameter, that its centre sits `E` from `O`, and that the cross-section closes as two
regions with the bore and one without, with exactly one of them a two-loop annulus.

<!-- check-step-calls: ignore buildCam -->
<!-- check-compile: ignore buildCam -->

**From:** `spec/cycloidal/instructions.md` L569-573, `spec/cycloidal/fusion.md` L340-348, `spec/cycloidal/epitrochoid-trace.md` L17-21

## S18 `[GO]` Extrude cam section `d`

Extrude the cam cross-section as a new body named `'Eccentric Cam {}'.format(d + 1)`, in the positive
direction from `plane(d)` toward the disk (`[CYCLOIDAL-F-CAM]`). Proof function:
`stepExtrudeCamSection`.

<!-- proof-run: proofkit3d.RunSolid(camSolidCases, stepExtrudeCamSection, assertExtrudeCamSection) -->

**Select the cross-section by loop count.** With an input shaft it is the **two-loop** annulus —
outer loop the cam outer, inner loop the bore — found by `profile.profileLoops.count == 2`. Without
one it is the single-loop cam disc, and `sketch.profiles.item(0)` is the only profile
(`[PB-SINGLE-PROFILE]`). Do **not** use `find_profile_by_curve_counts` for either: it counts
`NurbsCurve3D`, `Arc3D` and `Line3D` per loop and treats everything else as other, a full circle is a
`Circle3DCurveType` rather than an arc, and an annulus's two circles sit in separate loops, so it
raises `Could not find profile` (`[PB-PROFILE-MATCH]`).

The extent, with `nT = self.parameterName(PARAM_DISC_THICKNESS)` and
`nG = self.parameterName(PARAM_DISC_GAP)`, is
`adsk.core.ValueInput.createByString('{} + {}'.format(nT, nG))` for every section but the last, so it
fills the inter-disc gap and abuts the next one, and `adsk.core.ValueInput.createByString(nT)` for the
last. Both names are prefixed. The extrude is the same
`setOneSideExtent` and `PositiveExtentDirection` shape as S08.

The proof checks the section spans `[z_d, z_d + T + g]` when it is not the last and `[z_d, z_d + T]`
when it is, and that its volume is the annulus area — or the plain disc area with no bore — times
that depth.

<!-- check-step-calls: ignore find_profile_by_curve_counts -->

**From:** `spec/cycloidal/instructions.md` L569-579, `spec/cycloidal/fusion.md` L149-155 L349-361 L500-506

## S19 `[GO]` Join the cam sections into the Eccentric Cam

With two discs, join the `D` section bodies into one: target the first section's body, tools the
rest, `JoinFeatureOperation`, exactly the `combineFeatures` shape of S11. Name the result
`'Eccentric Cam'` and stash it on `self.cam`. Proof function: `stepJoinCamSections`.

<!-- proof-run: proofkit3d.RunSolid(camSolidCases, stepJoinCamSections, assertJoinCamSections) -->

The `+E` and `-E` sections have centres only `2E` apart against a radius of
`CenterBearingDiameter / 2`, so they overlap across most of their area and the join is one continuous
solid with the input bore running through it.

**SPEC DEFECT.** For `D == 1` the spec still says to Join, target section 0 with "the rest" as tools,
and the rest is empty. `combineFeatures.createInput(targetBody, toolBodies)` takes an
`ObjectCollection` of **one or more** tool bodies, so a single-disc build has no Join to make: skip
the combine entirely and rename the single section body to `'Eccentric Cam'` instead. The proof takes
that branch for `D == 1` and the real join for `D == 2`.

The proof checks the joined cam is one lump spanning `[0, stackTop]`, with the volume of the two
sections less the lens they share, and that the input bore's footprint sits inside both sections —
which is the condition, checked at S03 as `E + ISD/2 < CBD/2`, that makes the bore continuous through
the joined cam.

**From:** `spec/cycloidal/instructions.md` L364-366 L569-579, `spec/cycloidal/fusion.md` L356-361 L500-506, `spec/cycloidal/epitrochoid-trace.md` L134-147

## S20 `[PROSE]` Construction plane for the housing base

`buildRingPins()` starts here. Create the housing plane `1 mm` below the disc, on the side away from
it (`[CYCLOIDAL-F-RING-PINS]`, `[PB-CONSTRUCTION-PLANES]`):

```
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString('-1 mm'))
housingPlane = component.constructionPlanes.add(planeInput)
housingPlane.name = 'Ring Housing Plane'
```

The `'-1 mm'` here and the `'1 mm'` negative side of S25's two-sided extrude are the same number and
must stay so, or the casing's bottom face will not land on the base's top face and S28 will leave two
lumps instead of one.

<!-- check-step-calls: ignore buildRingPins -->
<!-- check-compile: ignore buildRingPins -->

**From:** `spec/cycloidal/instructions.md` L367-371 L504-511, `spec/cycloidal/fusion.md` L238-248

## S21 `[GO]` Draw the Housing Ring sketch

A sketch named `'Housing Ring'` on the housing plane, anchored to `O`
(`[CYCLOIDAL-F-ANCHOR-CHAIN]`), holding a plain annulus (`[CYCLOIDAL-F-RING-PINS]`). Proof function:
`stepHousingRingSketch`.

<!-- proof-run: proofkit.Run(casingSketchCases, stepHousingRingSketch) -->

- Outer circle:
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), R - Rr + 2 * E + Wall)`,
  driving diameter dimension with `.parameter.expression` set to
  `self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)`.
- Inner circle:
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), R - Rr - Wall)`,
  driving diameter dimension with `.parameter.expression` set to
  `self.parameterName(PARAM_HOUSING_INNER_DIAMETER)`.

Constrain each circle's centre **coincident to the local origin only**, with
`sketch.geometricConstraints.addCoincident(circle.centerSketchPoint, localOrigin)`. Do not also set
the centre's `isFixed`: coincident to the anchored origin already pins it, and adding the fix is
redundant and risks an over-constrained solve (`[PB-SHARE-XOR-COINCIDENT]`,
`[PB-NO-OVERCONSTRAIN]`). `[PB-CIRCLE-CENTER]` prefers `isFixed` for a circle drawn at the origin of
an unanchored sketch; this gear anchors instead, and the coincident to the anchored local origin is
what this spec pins the centre with.

This is a **pinless** wall. The outer radius clears the disc's furthest reach — the rolling contour's
peak at `R - PinRadius + 2 * E` — by exactly `Wall`, which is what makes `Wall` the minimum wall
thickness, a little more at the contour valleys. The inner floor lip sits `Wall` inside the contour
valley at `R - PinRadius`. The proof measures both clearances against the contour it computes rather
than against the formula.

**From:** `spec/cycloidal/instructions.md` L504-516 L130-138, `spec/cycloidal/fusion.md` L249-255, `.claude/skills/generate-gear/PLAYBOOK.md` L448-454

## S22 `[GO]` Extrude the housing base annulus

Extrude the **annulus** profile — the one with `profile.profileLoops.count == 2`, never
`find_profile_by_curve_counts`, for the reason S18 gives — by `Base Thickness` in the **negative**
direction, away from the disc, as a new body named `'Housing Ring'`. Proof function:
`stepExtrudeHousingBase`.

<!-- proof-run: proofkit3d.RunSolid(casingSolidCases, stepExtrudeHousingBase, assertExtrudeHousingBase) -->

```
ext = component.features.extrudeFeatures.createInput(
    annulusProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
ext.setOneSideExtent(
    adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
        self.parameterName(PARAM_BASE_THICKNESS))),
    adsk.fusion.ExtentDirections.NegativeExtentDirection)
housingExtrude = component.features.extrudeFeatures.add(ext)
```

Away from the disc is the **negative** direction here, because the offset plane keeps `self.plane`'s
normal. Stash the body on `self.housingRing`. The base spans `[-1 - BaseThickness, -1]`, which the
proof checks along with the annulus volume and the single lump.

**From:** `spec/cycloidal/instructions.md` L512-516, `spec/cycloidal/fusion.md` L255-259

## S23 `[PROSE]` Construction axis on the drive axis

Take the housing extrude's cap **by normal**, `housingExtrude.startFaces.item(0)`, and build the
drive axis through the local origin (`[CYCLOIDAL-F-RING-PINS]`, `[CYCLOIDAL-F-DISK-AXIS]`):

```
axInput = component.constructionAxes.createInput()
axInput.setByPerpendicularAtPoint(capFace, originPoint)
axis = component.constructionAxes.add(axInput)
axis.name = 'Drive Axis'
```

Stash it on `self.driveAxis`. Both the casing pattern in S26 and the output-pin pattern in S35 turn
about it, so it is created once here and reused.

**From:** `spec/cycloidal/instructions.md` L517-519, `spec/cycloidal/fusion.md` L260-261

## S24 `[GO]` Draw the Ring Casing section sketch

A sketch named `'Ring Casing'` on `self.plane`, anchored to `O`, holding one pin-pitch of the casing
(`[CYCLOIDAL-F-RING-PINS]`). Proof function: `stepRingCasingSketch`.

<!-- proof-run: proofkit.Run(casingSketchCases, stepRingCasingSketch) -->

**Compute the contour in Python first.** The inner wall follows the disc's swept envelope offset
outward by the clearance, `contour(phi) = env(phi) + c` — a smooth conjugate curve, not a
constant-radius circle. Sweep the disc over a full cam cycle and bin what it reaches:

```
for theta in 240 uniform steps over [0, 2*pi):
    cx, cy = E*cos(theta), E*sin(theta)
    phi = -theta / L
    for t in 240 uniform steps over [0, 2*pi):
        x, y = disk_point(t, cx, cy, phi)
        a = atan2(y, x)
        if -pi/N <= a <= pi/N:
            bin a into nbins = 80 bins over [-pi/N, +pi/N]
            binMax[bin] = max(binMax[bin], hypot(x, y)); hit[bin] = True
```

Then emit `nbins + 1` points at the bin **edges**, `phi_i = -pi/N + (2*pi/N) * i / nbins` for
`i = 0 … nbins`, each at radius `c + max(binMax[i-1], binMax[i])` using only its **hit** neighbours,
and `c` alone when both neighbours are unhit. The point is `(r_i * cos(phi_i), r_i * sin(phi_i))`,
ordered by angle, already in cm.

**Do not use bin centres.** Edges are what put the first point exactly on `-pi/N` and the last
exactly on `+pi/N`. Centres inset both ends by half a bin, which leaves an angular gap of
`2*pi/(N*nbins)` between every pair of patterned sectors, so the `N` sectors never touch, the Join
cannot merge them, and the build ends with `N` unnamed bodies instead of one casing. The proof
asserts the two end angles to twelve decimal places for exactly this reason.

The sketch then holds:

- an **outer circle** of radius `R - Rr + 2 * E + Wall`,
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), R - Rr + 2 * E + Wall)`,
  centre coincident to the local origin, driving diameter dimension with `.parameter.expression`
  `self.parameterName(PARAM_HOUSING_OUTER_DIAMETER)` — left **solid**, never construction, because it
  forms the sector's outer arc and a construction circle would leave the wedge open;
- an open **fitted spline** through the contour points, built the same way as the lobe:
  `adsk.core.Point3D.create(x, y, 0)` into an `adsk.core.ObjectCollection.create()`, then
  `sketch.sketchCurves.sketchFittedSplines.add(coll)`, `isClosed` never set. Keep the handle; S25
  selects on it;
- **two radial spokes**, `sketch.sketchCurves.sketchLines.addByTwoPoints(<spline end>, <point on the
  outer circle at the same angle>)` from each spline end at `phi = ±pi/N` out to the outer circle.

This sketch is a deliberate `[PB-FULL-CONSTRAINT]` exemption. The contour's fit points are numeric
snapshots and are **not** fixed, and the spokes' outer ends are only seeded on the outer circle with
no coincident constraint to it. What is constrained is the outer circle's centre and diameter, and
each spoke's inner end sharing the spline's end fit point. The sketch is consumed immediately by S25
and never re-solved, so the free geometry is accepted as it stands.

The proof cannot accept free geometry — its gate has no way to pass an under-constrained sketch — so
it pins what Fusion leaves loose and says so at `stepRingCasingSketch`. What it still proves is
everything the exemption does not touch: the ends land exactly on `±pi/N`, the contour's valley is at
`R - Rr` and its ends are the mid-gap peaks at `R - Rr + 2E` where the curve is tangential by
symmetry, and the contour bounds exactly two closed regions of which the wedge is much the smaller.

`contour` and `env` name the swept-envelope curve as functions of the polar angle, in the formula
above; neither is a call the module makes.

<!-- check-step-calls: ignore contour -->
<!-- check-compile: ignore contour -->

**From:** `spec/cycloidal/instructions.md` L316-322 L520-534, `spec/cycloidal/fusion.md` L262-283, `spec/cycloidal/epitrochoid-trace.md` L149-188, `.claude/skills/generate-gear/PLAYBOOK.md` L438-443 L506-513

## S25 `[GO]` Extrude the ring casing sector, two-sided

Extrude the thin annular pie wedge bounded by the outer arc, the contour spline and the two spokes,
as a new body (`[CYCLOIDAL-F-RING-PINS]`). Proof function: `stepExtrudeCasingSector`.

<!-- proof-run: proofkit3d.RunSolid(casingSolidCases, stepExtrudeCasingSector, assertExtrudeCasingSector) -->

**Select it by MINIMUM AREA among the profiles whose loop contains the contour spline** — not by
"contains the spline" alone, and not `sketch.profiles.item(0)`. The outer circle is solid, so the open
contour spline is a shared edge of **two** closed profiles: the thin wedge, and the large complement,
which is everything else inside the outer circle. Both contain the spline, so a first-match search can
return the complement, and extruding that gives a near-full disc which patterns `N` times into a solid
cylinder with the scallops erased. Compare with
`profile.areaProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy).area` and take the
smallest.

```
ext = component.features.extrudeFeatures.createInput(
    sectorProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
ext.setTwoSidesExtent(
    adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByString(stackTopExpr)),
    adsk.fusion.DistanceExtentDefinition.create(
        adsk.core.ValueInput.createByString('1 mm')))
sectorFeature = component.features.extrudeFeatures.add(ext)
```

`stackTopExpr` is built for the current disc count from **prefixed** names, with
`nT = self.parameterName(PARAM_DISC_THICKNESS)` and `nG = self.parameterName(PARAM_DISC_GAP)`: it is
`nT` when `D == 1` and `'2 * {} + {}'.format(nT, nG)` when `D == 2`. `DiscCount` is a dropdown, not a
parameter, so it never appears in an expression.

The negative `'1 mm'` side matches the housing plane's `'-1 mm'` offset exactly, so the casing's
bottom face is coincident with the base's top face and S28 yields one connected solid rather than two
lumps. The proof checks that span, `[-1, stackTop]`, and the wedge's volume against the ring area
divided by `N`.

**From:** `spec/cycloidal/instructions.md` L535-543 L340-347, `spec/cycloidal/fusion.md` L284-303 L495-498

## S26 `[GO]` Circular-pattern the casing sector `N` times

Pattern the sector **feature** `N = Pin Count` times about `self.driveAxis` over a full turn, with the
same input shape as S10 and `pat.quantity = adsk.core.ValueInput.createByReal(N)` — and
`pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute`, as on every
pattern here. Proof function: `stepPatternCasingSectors`.

<!-- proof-run: proofkit3d.RunSolid(casingSolidCases, stepPatternCasingSectors, assertPatternCasingSectors) -->

The pattern steps by exactly `2*pi/N`, which is the angle the contour's ends at `±pi/N` were chosen
for: adjacent sectors then share a spoke face rather than leaving a gap between them. The proof
checks the ring's volume is exactly `N` times the sector's, which is what a tiling with neither gap
nor overlap means, and that the ring spans the same `[-1, stackTop]` as the sector.

**From:** `spec/cycloidal/instructions.md` L543-547, `spec/cycloidal/fusion.md` L303-310, `spec/cycloidal/epitrochoid-trace.md` L164-186

## S27 `[GO]` Join the `N` casing sectors into one casing

Collect the `N` sector bodies by a pre-extrude `base = component.bRepBodies.count` baseline, exactly
as S11 does for the disc, and Join them with `combineFeatures` and `JoinFeatureOperation` into one
casing body. Proof function: `stepJoinCasingSectors`.

<!-- proof-run: proofkit3d.RunSolid(casingSolidCases, stepJoinCasingSectors, assertJoinCasingSectors) -->

The section ends fall on the contour's mid-gap peaks, where it is tangential by symmetry, so the
joined inner wall is smooth: the disc's valleys roll on the contour near each pin with clearance `c`,
and the lobe tips clear it between pins.

The proof checks the joined casing is one lump — `N` sectors that do not touch never become one — and
counts its faces: one per contour chord all the way round, the outer wall, and two caps. A seam left
at any pitch boundary shows up there as extra faces.

**From:** `spec/cycloidal/instructions.md` L543-547, `spec/cycloidal/fusion.md` L307-310, `spec/cycloidal/epitrochoid-trace.md` L176-186

## S28 `[GO]` Combine the casing into the Housing

Join the casing body into the base with `combineFeatures`: target `self.housingRing`, tools an
`adsk.core.ObjectCollection.create()` holding the casing body, `JoinFeatureOperation`. Then rename
`self.housingRing.name = 'Housing'`, keep `self.housingRing` as the combined body, and set
`self.ringCasing = None`, since the casing was consumed by the Join
(`[CYCLOIDAL-F-RING-PINS]`). Proof function: `stepJoinHousing`.

<!-- proof-run: proofkit3d.RunSolid(housingJoinCases, stepJoinHousing, assertJoinHousing) -->

Do not leave the base and the casing as two bodies: the housing is one printable part. Because the
casing's bottom face is coincident with the base's top face, the result is one connected solid
spanning `[-1 - BaseThickness, stackTop]` — the base floor below, the scalloped reaction wall around
the disc stack. There are no separate pins and no sockets. S36 then chamfers `self.housingRing` once
and skips the `None` casing.

The proof checks the join gives one lump spanning the whole housing, with the two bodies' volumes
less the overlap it had to introduce to get the engine to perform the union at all. This is the one
step whose case table is trimmed: it sweeps the pin count, which sets the ring's facet count and the
contact the join has to resolve, but leaves out the two-disc entry, whose only delta here is the
casing's extent — which S25 and S26 both build and measure at two discs. The reason is cost: this
union is minutes of work at the default pin count, and it is the proof's slowest step by a wide
margin.

**From:** `spec/cycloidal/instructions.md` L548-555 L618-629, `spec/cycloidal/fusion.md` L311-318

## S29 `[PROSE]` Construction plane for the output plate

`buildOutputPins()` starts here. Create the plate plane `1 mm` above the **top** disc, on the side
opposite the housing (`[CYCLOIDAL-F-OUTPUT-PINS]`, `[PB-CONSTRUCTION-PLANES]`):

```
planeInput = component.constructionPlanes.createInput()
planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByString(stackTopExpr + ' + 1 mm'))
platePlane = component.constructionPlanes.add(planeInput)
platePlane.name = 'Output Plate Plane'
```

`stackTopExpr` is the prefixed stack-top string of S25 — `nT` for one disc, `'2 * {} + {}'.format(nT,
nG)` for two — never a bare `'DiscThickness'`, which raises `RuntimeError: invalid expression`.

This plane is **above** the disk, so on its sketch the positive direction points **away** from the
disk and the negative one toward it: the mirror of the housing plane, which is below, where away is
negative.

<!-- check-step-calls: ignore buildOutputPins -->
<!-- check-compile: ignore buildOutputPins -->

**From:** `spec/cycloidal/instructions.md` L340-347 L372-373 L581-589, `spec/cycloidal/fusion.md` L363-376 L495-498

## S30 `[GO]` Draw the Output Plate sketch

One sketch named `'Output Plate'` on the plate plane, anchored to `O`. Everything in it is on the
drive axis, not the disc centre: the plate and its pins are the fixed output member, and it is the
disc's holes that orbit around them (`[CYCLOIDAL-F-OUTPUT-PINS]`). Proof function:
`stepOutputPlateSketch`.

<!-- proof-run: proofkit.Run(outputSketchCases, stepOutputPlateSketch) -->

- **Plate outer circle**,
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), OutputPlateDiameter / 2)`,
  **solid**, centre coincident to the local origin, driving diameter dimension with
  `.parameter.expression` set to `self.parameterName(PARAM_OUTPUT_PLATE_DIAMETER)`.
- **Output-pin circle**,
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(0, 0, 0), Rop)`,
  **construction**, centre coincident to the local origin, driving diameter dimension with
  `.parameter.expression` set to `self.parameterName(PARAM_OUTPUT_PIN_CIRCLE_DIAMETER)`.
- **One output pin**,
  `sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(Rop, 0, 0), D_pin / 2)`,
  **solid**, with a driving diameter dimension whose `.parameter.expression` is
  `'{} - 2 * {}'.format(self.parameterName(PARAM_OUTPUT_HOLE_DIAMETER), self.parameterName(PARAM_ECCENTRICITY))`
  — the hole less its orbit clearance. Pin its centre with
  `sketch.geometricConstraints.addCoincident(pin.centerSketchPoint, outPinCircle)` and a horizontal
  construction line from the local origin to that centre plus
  `sketch.geometricConstraints.addHorizontal(spokeLine)`.

The pin sits inside the plate disc and splits it into two regions: the pin disc, and the plate with a
bite taken out of it. The proof checks the plate overhangs the outermost pin by exactly `Wall`, that
the pin centre sits at `Rop` from `O`, and that those two regions' areas add back up to the whole
plate disc — which is what S31 relies on when it extrudes both together.

**From:** `spec/cycloidal/instructions.md` L590-594 L138-141, `spec/cycloidal/fusion.md` L377-386

## S31 `[GO]` Extrude the output plate

Extrude the **full** plate disc — **every** profile in the sketch, the plate-with-bite and the pin
disc together, so the footprint under the pin is solid plate — away from the disk by
`Output Plate Thickness`, as a new body named `'Output Plate'`. Proof function:
`stepExtrudeOutputPlate`.

<!-- proof-run: proofkit3d.RunSolid(outputSolidCases, stepExtrudeOutputPlate, assertExtrudeOutputPlate) -->

```
coll = adsk.core.ObjectCollection.create()        # every profile in the Output Plate sketch
ext = component.features.extrudeFeatures.createInput(
    coll, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
ext.setOneSideExtent(
    adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
        self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
    adsk.fusion.ExtentDirections.PositiveExtentDirection)
plateExtrude = component.features.extrudeFeatures.add(ext)
```

Positive is away from the disk here, because the plate plane is above it. Stash the body on
`self.outputPlate`. The plate spans `[stackTop + 1 mm, stackTop + 1 mm + OutputPlateThickness]`,
which the proof checks along with its volume.

**From:** `spec/cycloidal/instructions.md` L595-598, `spec/cycloidal/fusion.md` L387-392

## S32 `[GO]` Extrude the output pin, two-sided

Select the **pin disc** — the profile with `profile.profileLoops.count == 1` whose loop curve is the
pin circle, not an any-loop-contains match, which returns the surrounding plate ring — and extrude it
both ways as a new body named `'Output Pin'` (`[CYCLOIDAL-F-OUTPUT-PINS]`). Proof function:
`stepExtrudeOutputPin`.

<!-- proof-run: proofkit3d.RunSolid(outputSolidCases, stepExtrudeOutputPin, assertExtrudeOutputPin) -->

```
pinExt = component.features.extrudeFeatures.createInput(
    pinProfile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
pinExt.setTwoSidesExtent(
    adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
        self.parameterName(PARAM_OUTPUT_PLATE_THICKNESS))),
    adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByString(
        stackTopExpr + ' + 1 mm')))
pinFeature = component.features.extrudeFeatures.add(pinExt)
```

Side one, positive, is away from the disk and into the plate, `Output Plate Thickness` of it. Side
two is toward the disk, `stackTopExpr + ' + 1 mm'`, which reaches disc 0's bottom at `z = 0` so the
pin runs through **every** disc's output holes. Keep the `ExtrudeFeature` and its body,
`pinFeature.bodies.item(0)`; S33, S34 and S35 all need them.

The proof checks the far end lands exactly on `z = 0` — short of that and the pin misses the lowest
disc's holes — and that the pin is the hole less `2E`, the clearance that lets a hole orbiting by `E`
about it stay clear.

**From:** `spec/cycloidal/instructions.md` L599-603, `spec/cycloidal/fusion.md` L393-402

## S33 `[GO]` Cut the plate socket, keeping the pin

A combine-cut whose tool survives: target `self.outputPlate`, tool an
`adsk.core.ObjectCollection.create()` holding the pin body, `CutFeatureOperation`, and
`ci.isKeepToolBodies = True` — which leaves a matching hole in the plate with the pin seated in it
(`[CYCLOIDAL-F-OUTPUT-PINS]`). Proof function: `stepCutOutputSocket`.

<!-- proof-run: proofkit3d.RunSolid(outputSolidCases, stepCutOutputSocket, assertCutOutputSocket) -->

```
tools = adsk.core.ObjectCollection.create()
tools.add(pinBody)
ci = component.features.combineFeatures.createInput(self.outputPlate, tools)
ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
ci.isKeepToolBodies = True
combineFeature = component.features.combineFeatures.add(ci)
```

Keep the `CombineFeature`; S35 patterns it alongside the pin. The proof checks the socket removed
exactly the pin's own footprint through the plate's thickness, leaving one lump, and that the pin is
that same diameter, so it seats rather than rattling or refusing to enter.

**From:** `spec/cycloidal/instructions.md` L604-605, `spec/cycloidal/fusion.md` L403-405

## S34 `[GO]` Chamfer the output pin's ends

If `Chamfer Size > 0`, chamfer the pin body's two ends with `self._chamferCapRims(pinBody)`, the same
helper the rim chamfers use (`[CYCLOIDAL-F-CHAMFERS]`). Keep the returned `ChamferFeature`, which may
be `None`. Proof function: `stepChamferOutputPinEnds`.

<!-- proof-run: proofkit3d.RunSolid(outputSolidCases, stepChamferOutputPinEnds, assertChamferOutputPinEnds) -->

The helper's contract, in full, since S36 uses it too:

- Return `None` immediately when `self.chamferSize <= 0`.
- `axis = self.plane.geometry.normal` and `ref = self.plane.geometry.origin`.
- First pass, collecting cap faces with their heights: for each `face` in `body.faces`, skip it unless
  `face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType`; take `n = face.geometry.normal`
  and skip unless `abs(n.dotProduct(axis)) > 0.999`, which keeps the flat faces perpendicular to the
  axis and drops the side walls; record the axial height `h`, the dot of `face.geometry.origin - ref`
  with `axis`. `SurfaceTypes` is in `adsk.core`, not `adsk.fusion` (`[PB-ADSK-MODULES]`).
- Return `None` when nothing was collected. Otherwise take `hmin` and `hmax` over what was.
- Second pass: chamfer **only the two axially extreme caps**, those with `h` within about `1e-4` cm of
  `hmin` or `hmax`, and within each, only the loops with `loop.isOuter` — the rim — adding every edge
  of those loops into an `adsk.core.ObjectCollection.create()`. The extreme filter is what keeps the
  combined `Housing`'s internal ledge at the base-casing junction out: that ledge is a cap-normal face
  too, its outer loop is the scalloped contour, and chamfering it throws
  `RuntimeError ... ASM_BL_CAP_COMPLEX`. For a uniform disc, plate or pin the filter changes nothing,
  since both caps are extreme.
- Return `None` when no edge was collected. Otherwise the modern chamfer input shape
  (`[PB-FILLET-CHAMFER]`: a chamfer's edge set goes on the input's `chamferEdgeSets` collection, unlike
  a fillet's, which goes on the input itself):

```
chamfers = self.getComponent().features.chamferFeatures
ci = chamfers.createInput2()
ci.chamferEdgeSets.addEqualDistanceChamferEdgeSet(
    edges, adsk.core.ValueInput.createByString(self.parameterName(PARAM_CHAMFER_SIZE)), False)
return chamfers.add(ci)
```

⚠️ Wrap the `chamfers.add(ci)` call in `try`/`except`: when the requested size is too large for the
geometry Fusion raises `RuntimeError` from it, and that must not abort the build. On failure,
`futil.log` the reason, increment `self.chamfersSkipped` and return `None` — never re-raise.

Only the **outer** loop is chamfered, so bores, output holes and sockets stay sharp.

The proof builds the pin, chamfers both end rims, and checks the volume dropped by exactly the two
rings a 45-degree equal-distance chamfer of that size takes, `pi * c^2 * (r - c/3)` each, with the
pin's length unchanged. It runs the `Chamfer Size == 0` case too, where the helper returns before it
selects an edge and the pin is left exactly as extruded.

<!-- check-step-calls: ignore _chamferCapRims -->

**From:** `spec/cycloidal/instructions.md` L606-607 L631-645, `spec/cycloidal/fusion.md` L417-460, `.claude/skills/generate-gear/PLAYBOOK.md` L536-542

## S35 `[GO]` Pattern the pin, socket and chamfer `M` times

One pattern carries all three features round the drive axis (`[CYCLOIDAL-F-OUTPUT-PINS]`). Proof
function: `stepPatternOutputPins`.

<!-- proof-run: proofkit3d.RunSolid(outputSolidCases, stepPatternOutputPins, assertPatternOutputPins) -->

```
coll = adsk.core.ObjectCollection.create()
coll.add(pinFeature)          # the ExtrudeFeature from S32
coll.add(combineFeature)      # the socket CombineFeature from S33
                              # and the ChamferFeature from S34, when it is not None
pat = component.features.circularPatternFeatures.createInput(coll, self.driveAxis)
pat.patternComputeOption = adsk.fusion.PatternComputeOptions.AdjustPatternCompute
pat.quantity = adsk.core.ValueInput.createByReal(M)
pat.totalAngle = adsk.core.ValueInput.createByString('360 deg')
pat.isSymmetric = False
component.features.circularPatternFeatures.add(pat)
```

The axis is `self.driveAxis` from S23, at `O` — the pins are on the drive axis while the disc's holes
are on `Od`, both starting from their own `+X` point, so each pin sits in its hole offset by `E`.

**Then name all `M` pin bodies**, which is what S37 groups them by. Capture
`pinBase = component.bRepBodies.count` **before** S32's pin extrude; the `M` pin bodies are then the
contiguous block `component.bRepBodies.item(pinBase)` through `item(pinBase + M - 1)`, because the
socket cut adds no body and only the pin's new-body extrude and its `M - 1` pattern instances do.
Rename each: `body.name = 'Output Pin {}'.format(k + 1)` for `k` in `0 … M - 1`, which overwrites the
seed pin's `'Output Pin'`.

The proof builds the plate carrying all `M` sockets and checks the volume identity, the single lump,
that every pin centre sits at `Rop` from `O`, and that the plate still covers each of them by `Wall`.

**From:** `spec/cycloidal/instructions.md` L608-616, `spec/cycloidal/fusion.md` L406-415

## S36 `[GO]` Chamfer the outer rims

`buildChamfers()` runs once over the finished bodies. If `Chamfer Size == 0`, do nothing at all.
Otherwise call `self._chamferCapRims(body)` — the helper whose contract S34 sets out — on each of:
**every** disc in `self.diskBodies`, whose rim is the lobe profile; `self.housingRing`, which is the
combined `Housing`, base and casing in one body; and `self.outputPlate`. Proof function:
`stepChamferOuterRims`.

<!-- proof-run: proofkit3d.RunSolid(outputSolidCases, stepChamferOuterRims, assertChamferOuterRims) -->

**Guard each against `None`.** `self.ringCasing` is `None` after S28's Join consumed it, so it is
skipped, and chamfering only `self.housingRing` covers the whole housing exactly once. Inner edges —
bores, output holes, the casing's inner contour — are left sharp; outer rim only. The output pins were
already chamfered inside their own pattern in S34, and the casing's pins are integral bumps that need
no chamfer of their own.

At the **very end** of `generate()`, after S37, if `self.chamfersSkipped > 0`, show a **non-fatal**
message and carry on, because the part is already built:

```
adsk.core.Application.get().userInterface.messageBox(
    'Cycloidal drive generated, but {n} chamfer(s) could not be created at Chamfer Size {sz} mm '
    'and were skipped. Reduce Chamfer Size (or set it to 0) for this geometry.'.format(
        n=self.chamfersSkipped, sz=to_mm(self.chamferSize)))
```

The proof chamfers the output plate's two rim loops and checks the volume dropped by exactly the two
rings, with the plate's own span unchanged, and runs the `Chamfer Size == 0` case where nothing is
chamfered at all. What it does not reach is recorded at `stepChamferOuterRims` in
`proof/cycloidal/solids_test.go`: the rotor disc's lobe rim, whose self-intersection at the tight
valleys is the Fusion failure the resilient wrapper exists for, and the combined `Housing`'s rim,
which the solid engine will not chamfer because a boolean result is not a straight prism.

<!-- check-step-calls: ignore buildChamfers -->
<!-- check-compile: ignore buildChamfers -->

**From:** `spec/cycloidal/instructions.md` L618-645, `spec/cycloidal/fusion.md` L451-460

## S37 `[PROSE]` Group the bodies into sub-components and hide the construction geometry

`buildSubComponents()` is the **last** step, after every body is built and chamfered, because
`moveToComponent` invalidates the moved body's reference and every earlier step needs its bodies in
the Cycloidal Drive component (`[CYCLOIDAL-F-SUBCOMPONENTS]`, `[PB-OCCURRENCE-TREE]`).

**Snapshot first, move second.** Iterate `component.bRepBodies` **once** and bucket every body by
name into four Python lists — moving mutates that collection, so a loop that moved as it went would
skip bodies:

- name starts with `'Cycloidal Disk'` → `Rotor Discs`
- name equals `'Housing'` → `Housing`
- name equals `'Eccentric Cam'` → `Eccentric Cam`
- name equals `'Output Plate'`, or starts with `'Output Pin'` → `Output`

Keying by name, not by a stashed handle or a body index, is deliberate: chamfers and combines may have
refreshed the stashed proxies, and each move shifts every later index. S35 guarantees every pin is
named `'Output Pin k'`, and S19 and S28 leave the final bodies named `'Eccentric Cam'` and
`'Housing'`, so every body falls into exactly one bucket.

Then, for each non-empty group in the fixed order `Rotor Discs`, `Housing`, `Eccentric Cam`, `Output`:

```
occ = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())
occ.component.name = <group name>
for body in group:
    body.moveToComponent(occ)
```

The transform must be the **identity** `Matrix3D.create()`, so each body keeps its world position; a
non-identity transform would shift it. `moveToComponent` returns the relocated body or `None` on
failure — ignore the return and never reuse the pre-move reference. No new parameters, sketches or
dimensions are created here, and the construction planes and axes stay in the root Cycloidal Drive
component.

**Finally, hide the construction geometry** with the shared helper, which recursively walks the
component and its new sub-occurrences and turns off every sketch, construction plane and construction
axis: `solids.hide_construction_geometry(component)`. Do not re-implement it — a private equivalent is
a helper-shadow rejection (`[PB-TREE-CLEANUP]`). One call supersedes any per-axis
`isLightBulbOn = False`; note that a construction plane or axis is hidden by `isLightBulbOn`, never by
`isVisible`, which is what hides a sketch (`[PB-HIDE-AFTER-USE]`). Only the solid bodies stay visible.

<!-- check-step-calls: ignore buildSubComponents -->
<!-- check-compile: ignore buildSubComponents -->

**From:** `spec/cycloidal/instructions.md` L374-379 L647-667, `spec/cycloidal/fusion.md` L508-544, `.claude/skills/generate-gear/PLAYBOOK.md` L626-638 L784-787 L802-804
