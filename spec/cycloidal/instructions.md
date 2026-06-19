# Cycloidal Rotor Creation Instructions

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
  visibility). After **every value/dropdown input** it adds a hidden, read-only **per-field message
  slot** (a `TextBoxCommandInput` whose id is the input's id + `'__status'`); the shared handler shows
  the slot next to the field the user last edited (see "Live input validation").
- **`CycloidalDriveGenerator(base.Generator)`** — the generator. 1-arg constructor `(design)`
  (inherited); implements `generate(self, inputs)` and the call-graph methods below; relies on inherited
  `deleteComponent()` for error cleanup. Overrides `prefixBase()` to return `'CycloidalDrive'`. It also
  declares the **live-validation contract** the shared `GearCommand` consults (`[PB-VALIDATE-INPUTS]`):
  the class attribute **`DEFAULT_STATUS_INPUT_ID`** (the fallback slot id) and the **`@staticmethod
  validate_inputs(inputs) -> list[str]`** described under "Live input validation" below.

**Generation Context: none.** No `CycloidalDriveGenerationContext` class; carry handles on `self` —
per-disc handles are **lists** indexed by the disc index `d` (`self.diskBodies`, `self.diskAxes`,
`self.lobeSplines`, `self.outputHoles`, `self.lobeDiskCentres`, `self.discPlanes`), the rest scalars
(`self.driveAxis`, `self.housingRing`, `self.ringCasing`, `self.cam`, `self.outputPlate`).

**Dependencies: none.** Imports only the framework (`base`, `misc`, `utilities`, `solids`,
`fusion360utils`). ⚠️ Use **`solids.hide_construction_geometry(component)`** for the final cleanup — do NOT
re-implement it (helper-shadowing is rejected).

## Component Setup

One command invocation creates a new child component (named per `generateName`) of the user-selected
Parent Component. It draws six sketches — `Rotor Lobe` (one fully-constrained open lobe section + its
reference frame), `Output Hole` (one output hole), `Housing Ring` (the base annulus, on a construction plane
`1 mm` below the disk) + `Ring Casing` (one swept-contour section), `Disk Bore` (the enlarged center bore),
`Eccentric Cam` (cam outer + input bore), and `Output Plate` (the output disc + one output pin, on a plane
`1 mm` above the disk) — and builds the **rotor disk** (with its **center bore**), **`M` output holes**, the
**`Housing` body** (base annulus + pinless swept-contour ring casing, Combined into one part), the
**eccentric input cam**, and the **output plate + `M` output pins**. The **Anchor is the drive axis** `O`; the
**cycloidal disk is eccentric**, its centre
`Od = O + E·X̂` offset by the Eccentricity. So the parts on the fixed axes (the `Housing` base + casing
contour, the cam's input bore, **the output plate + output pins**) are built on `O`, while the **disk**
geometry (root circle, lobe, output holes, center bore) and the **cam outer** are built on `Od` — the disk
sits eccentrically inside the casing contour (`[CYCLOIDAL-F-DISK-CENTER]`). The **output pins are on `O`** but the
disk's **output holes are on `Od`** and oversized by `2E`, so the pins pass through as the disk orbits. Each
profile lives in its own sketch so they never interfere.

## Variables

User inputs in dialog order; all linear inputs are mm. Derived parameters are registered as Fusion user
parameters under the `CycloidalDrive<N>_` prefix.

- **Target Plane** — user-specified plane; the lobe sketch sits here. Any non-`ConstructionPlane`
  selection (e.g. a planar face) is normalized to a coplanar construction plane at generation time.
- **Anchor Point** — user-specified `ConstructionPoint` or `SketchPoint`; the lobe is centred on its
  in-plane projection `O`.
- **Disc Count** — a **dropdown** (`DropDownCommandInput`, `TextListDropDownStyle`) with items `'1'` and
  `'2'`, default **`'1'`**. `1` = single disc (today's build); `2` = two discs 180° opposed for balance.
  Read as int from the selected item's name. ⚠️ **`2` requires even `Pin Count` and even `Output Pin
  Count`** (see Validity) — the 180° disc-2 construction only maps pins→pins / holes→holes when both are
  even.
- **Pin Count** — integer ≥ 4 (`N`). Sets `Lobes = N − 1`. Default **16**.
- **Pin Circle Diameter** — mm > 0. `R = Pin Circle Radius` (pitch radius). Default 90 mm.
- **Pin Diameter** — mm ≥ 0. The ring-pin diameter, **`0` = auto** (derive `Rr` from the bounds-mean).
  Default **0**. If > 0, `Rr = Pin Diameter / 2` (an explicit override).
- **Eccentricity** — mm > 0 (`E`). Default 1.5 mm. (Validity: `E < R/N`; reject otherwise.)
- **Disk Clearance** — mm ≥ 0 (`c`). Backlash cut into the profile (`Rr_eff = Rr + c`). Default 0.3 mm.
- **Disc Thickness** — mm > 0. Axial thickness of **each** rotor disk (the lobe-sector extrude depth). Default 8 mm.
- **Disc Gap** — mm ≥ 0. Axial clearance between disc 1 and disc 2 (two-disc only). Default **0.5 mm**.
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
- **Housing Wall** — mm > 0 (`wall`). Radial wall thickness of the pinless casing: the outer wall clears the
  rolling contour's furthest reach (the peak at `R − PinRadius + 2·E`) by `wall`, so this is the **minimum**
  wall thickness (a little more at the contour valleys). Default **3 mm** (thin; no pins to enclose).
- **Base Thickness** — mm > 0. Axial thickness of the ring-pin base (Housing Ring); the ring pins run from
  its far face up to the disk top. Default **5 mm**.
- **Output Plate Thickness** — mm > 0. Axial thickness of the **output plate** (the disc carrying the `M`
  output pins, on a plane `1 mm` above the disk, opposite the housing). Default **5 mm**.
- **Chamfer Size** — mm ≥ 0. A 45° equal-distance chamfer on the **outer rim** (both flat faces) of the
  rotor disc, Housing Ring and Output Plate, and on the two **ends** of every ring pin and output pin.
  **`0` = no chamfer**. Default **0.5 mm**.
- **Parent Component** — component, default root (pre-selected); listed last among the editable inputs.
- **Per-field message slots** — **not** user inputs: immediately **after each value/dropdown input**
  above, `configure()` adds a hidden, read-only `TextBoxCommandInput` whose id is **that input's id +
  `'__status'`** (e.g. `pinCircleDiameter__status`). Create each with the exact signature
  `inputs.addTextBoxCommandInput(<id> + '__status', '', '', 2, True)` (id, empty label, empty initial
  formattedText, 2 rows, `isReadOnly=True`), then set its **`isVisible = False`**. These slots are **not**
  registered parameters and are **never read** by any `get_*` helper — the shared
  `GearCommand.command_validate_input` handler writes the live problem list into the slot next to the
  field the user last edited and toggles its visibility (`[PB-VALIDATE-INPUTS]`). Selections (Target
  Plane, Anchor Point, Parent Component) get **no** slot. See "Live input validation".

**`Rr` and the output pin/hole auto-derive (mean of bounds) when their `…Diameter` input is `0`, else use
the override** — see `epitrochoid-trace.md` "Pin / hole sizes". Resolve: `Rr = (Pin Diameter/2)` if
`Pin Diameter > 0` else `½·(E + R·sin(π/N))`; `D_pin = Output Pin Diameter` if `Output Pin Diameter > 0`
else `Rop·sin(π/M) − E`; `D_hole = D_pin + 2E`.

Derived (Python-precomputed, internal cm): `Lobes = N − 1`; `PinCircleRadius`, `OutputPinCircleRadius`;
**`PinRadius`** (resolved per above); `Rr_eff = PinRadius + c`; `Rv = R − Rr_eff − E`;
**`OutputHoleDiameter = D_hole`** (resolved per above); and the **ring-pin base annulus** diameters
**`HousingInnerDiameter = 2·(R − PinRadius − Wall)`** and **`HousingOuterDiameter = 2·(R − PinRadius + 2·E +
Wall)`** (**pinless** — the casing is a thin wall around the rolling contour, NOT a thick ring around the old
pin circle. The disc's furthest reach is the **contour peak at radius `R − PinRadius + 2·E`** [exact: the
lobe-tip envelope `env_max + c`, the `c` cancels — verified]; the outer wall clears it by `Wall`, so the
**minimum wall thickness is exactly `Wall`** at the peaks, a bit more at the valleys. The inner floor lip sits
`Wall` inside the contour valley `R − PinRadius`); and the **output-plate**
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
Also (two-disc): when **`Disc Count == 2`**, require **`N` even AND `M` even** — the 180° disc-2
construction maps pins→pins only for even `N` and output-holes→output-pins only for even `M`. Reject with a
clear message (e.g. "Two discs require an even Pin Count and even Output Pin Count").

⚠️ **All of the above validity checks are implemented exactly once**, in the shared
`evaluate_problems(vals)` helper, and surfaced **both** live (OK greys out while editing, via
`validate_inputs`) **and** at execute time (`_resolveDimensions` raises the joined messages) — see
"Live input validation" for the routine, the actionable message wording, and the numeric `E*` bound.

### Exact input ids and parameter-name strings (verbatim)

| Dialog input | input id | registered user-parameter |
|---|---|---|
| Parent Component (selection) | `parentComponent` | — |
| Target Plane (selection) | `plane` | — |
| Anchor Point (selection) | `anchorPoint` | — |
| Disc Count (dropdown 1/2) | `discCount` | — |
| Pin Count | `pinCount` | `PinCount` |
| Pin Circle Diameter | `pinCircleDiameter` | `PinCircleDiameter` |
| Pin Diameter | `pinDiameter` | `PinDiameter` |
| Eccentricity | `eccentricity` | `Eccentricity` |
| Disk Clearance | `diskClearance` | `DiskClearance` |
| Disc Thickness | `discThickness` | `DiscThickness` |
| Disc Gap | `discGap` | `DiscGap` |
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
| Per-field message slot after each value/dropdown input | `<inputId>__status` (e.g. `pinCircleDiameter__status`) | — (not read, not a parameter) |

`Pin Diameter` and `Output Pin Diameter` are `addValueInput` defaulting to **`0`** (`0` = auto-derive).
Derived parameter names (kept verbatim): `Lobes`, `PinCircleRadius`, `OutputPinCircleRadius`,
`PinRadius` (resolved), `OutputHoleDiameter` (resolved), `HousingInnerDiameter`, `HousingOuterDiameter`,
`OutputPlateDiameter`. (The ring casing's inner wall is a Python-computed swept-envelope spline — a numeric
snapshot, not a parameter-referenced dimension.)

⚠️ **Registration mode — `PinRadius` and `OutputHoleDiameter` MUST be `createByReal` numeric snapshots**
(the Python-**resolved** value, in internal cm), **NOT** live `createByString` expressions. They resolve by an
**auto-vs-override branch in Python** (`if Pin Diameter > 0 …`, and the auto formula uses `sin(π/N)`), which a
live Fusion expression cannot reproduce cleanly — so register the already-resolved float: `self.addParameter(
PARAM_PIN_RADIUS, adsk.core.ValueInput.createByReal(Rr), 'mm', …)` and likewise `createByReal(D_hole)` for
`OutputHoleDiameter`. Other dims then reference these two **by name** in their expressions (the names exist as
real parameters; their *value* is the snapshot). `PinCircleRadius`, `OutputPinCircleRadius`,
`HousingInnerDiameter`, `HousingOuterDiameter`, `OutputPlateDiameter`, `Lobes` stay **live** `createByString`
expressions (they compose cleanly from the registered inputs, including the snapshot `PinRadius`).

**Dialog display order (`configure()` adds inputs in exactly this sequence):** Target Plane, Anchor
Point, Disc Count, Pin Count, Pin Circle Diameter, Pin Diameter, Eccentricity, Disk Clearance, Disc
Thickness, Disc Gap, Center Bearing Diameter, Input Shaft Diameter, Bearing Clearance, Output Pin Circle
Diameter, Output Pin Count, Output Pin Diameter, Housing Wall, Base Thickness, Output Plate Thickness,
Chamfer Size, Parent Component. Selections first two; Parent last. **Immediately after each
value/dropdown input** in that sequence (i.e. after Disc Count, Pin Count, …, Chamfer Size — but NOT
after the selections), add that input's hidden `<id>__status` message slot, so each editable field is
directly followed by its own slot. All numerics are `addValueInput` (read with `get_value`); selections
via `get_selection`; the `__status` slots are `addTextBoxCommandInput` and are never read.
`Pin Count` (`N`) and `Output Pin Count` (`M`) are integer counts —
register the parameters but **read the counts from the dialog value rounded to int** for the Python
formulas (`sin(π/N)`, `sin(π/M)`; Fusion user params are floats).

## Live input validation

The dialog validates geometry **live** so the user is never silently kicked out at OK time. The shared
`GearCommand.command_validate_input` (`[PB-VALIDATE-INPUTS]`) consults two members this generator
declares, and on any problem **disables OK** (the dialog stays open and editable) while showing the
problem list in the **per-field message slot next to the field the user last edited** (the handler
tracks the last-changed input and writes into `<that-input-id>__status`, hiding the previously shown
slot; for selection changes or the initial open it uses `DEFAULT_STATUS_INPUT_ID`):

- **`DEFAULT_STATUS_INPUT_ID = INPUT_ID_PIN_CIRCLE_DIAMETER + '__status'`** (class attribute on
  `CycloidalDriveGenerator`) — the fallback slot, used when the last-edited input has no slot of its own.
- **`@staticmethod validate_inputs(inputs) -> list[str]`** — a **pure** check (no document writes, no
  parameter registration): it reads the **raw** values straight off the command inputs
  (`inputs.itemById(<id>).value` → internal cm for lengths, the rounded int for counts,
  `inputs.itemById(INPUT_ID_DISC_COUNT).selectedItem.name` for the dropdown), resolves the same derived
  dimensions as `_resolveDimensions` (`Rr`, `Rr_eff`, `Rv`, `D_pin`, `D_hole` per the auto-vs-override
  rules above), runs the validity checks, and returns a list of **actionable** problem strings (empty =
  valid). It runs on every keystroke, so keep it cheap. If a value can't be read yet (a half-typed
  expression), let the read raise — the shared handler catches it and treats the inputs as provisionally
  valid (the execute-time guard is the backstop).

**Single source of validity math.** Implement the resolve-and-check once: a module-level helper
`evaluate_problems(vals) -> list[str]` taking the resolved scalar values (cm/ints) and returning the
problem strings. `validate_inputs` builds `vals` by reading the raw inputs; **`_resolveDimensions` builds
the same `vals` from the registered parameters and, if `evaluate_problems(vals)` is non-empty, raises
`Exception('\n'.join(problems))`** — so OK-time execution still fails cleanly with the same messages if
anything slips through. The two paths MUST share `evaluate_problems`; never duplicate the formulas.

**Each problem names the offending input and a concrete numeric bound** (formatted in **mm** via
`to_mm`, rounded to ~2 decimals; counts as integers). Use these messages and their inverted bounds
(internal cm: `R = PinCircleDiameter/2`, `Rop = OutputPinCircleDiameter/2`, `E`, `c`, `N`, `M`,
`CBD = CenterBearingDiameter`, `clr = BearingClearance`, `ISD = InputShaftDiameter`; resolved `Rr`,
`Rr_eff = Rr + c`, `Rv = R − Rr_eff − E`, `D_pin`, `D_hole = D_pin + 2E`):

| Validity check (must hold) | Problem string when it fails (suggestion) |
|---|---|
| **2 discs ⇒ `N` even AND `M` even** | "Two discs require an even Pin Count and an even Output Pin Count (currently N=…, M=…)." |
| `E < Rr < R·sin(π/N)` | auto (`Pin Diameter = 0`): "Pin geometry out of range — increase Pin Circle Diameter above {2E/sin(π/N)} mm or reduce Eccentricity below {R·sin(π/N)} mm." override (`Pin Diameter > 0`): "Pin Diameter must be between {2E} mm and {2·R·sin(π/N)} mm (currently {2·Rr})." |
| `D_pin > 0` | auto: "Output pins vanish (resolved diameter ≤ 0) — increase Output Pin Circle Diameter above {2E/sin(π/M)} mm, increase Output Pin Count, or reduce Eccentricity." override: "Output Pin Diameter must be greater than 0." |
| `D_hole < 2·Rop·sin(π/M)` | "Output holes overlap — increase Output Pin Circle Diameter above {(D_pin+2E)/sin(π/M)} mm, increase Output Pin Count, or reduce Output Pin Diameter / Eccentricity." |
| `E < R/N` | "Eccentricity too large — reduce it below {R/N} mm (or increase Pin Circle Diameter / reduce Pin Count)." |
| `Rop < Rv` | "Output Pin Circle too large — set Output Pin Circle Diameter below {2·Rv} mm (currently {2·Rop})." |
| **no-undercut `Rr_eff < ρ_min^O`** | "Eccentricity too large — the rotor profile undercuts/self-intersects. Reduce Eccentricity below {E*} mm." where **`E*` is found by bisection** (below). |
| `ISD < CBD` | "Input Shaft Diameter must be less than Center Bearing Diameter ({CBD} mm)." |
| `E + ISD/2 < CBD/2` | "Input bore doesn't fit inside the cam — set Input Shaft Diameter below {CBD − 2E} mm, or reduce Eccentricity / increase Center Bearing Diameter." |
| `(CBD + clr)/2 < Rop − D_hole/2` | "Disk center bore overlaps the output holes — increase Output Pin Circle Diameter above {CBD + clr + D_hole} mm, or reduce Center Bearing Diameter / Bearing Clearance / output pin size." |

Return **every** failing message (not just the first), so the user sees all problems at once. Order them
to match the table. Two-disc evenness is checked first (it needs only `N`, `M`, the dropdown).

**Undercut bound `E*` (numeric).** The undercut limit is the *binding* eccentricity constraint and has no
closed form, so solve it for the message: holding every other input fixed, find the largest `E*` in
`(0, E]` for which `Rr_eff(E') < ρ_min^O(E')` still holds, where `ρ_min^O` is the existing base-trochoid
curvature scan (`epitrochoid-trace.md` "No-undercut guard"; note both `Rr_eff` and `ρ_min^O` depend on
`E'` when `Pin Diameter = 0`). A ~40-iteration **bisection** on `E'` is ample; report `to_mm(E*)`. If the
solve degenerates (no positive `E'` satisfies it), fall back to the plain "reduce Eccentricity" message
without a number.

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
  references are deterministic every regen.
  ⚠️ **The names in the table below are the LOGICAL parameter names — the actual registered parameters are
  PREFIXED (`CycloidalDrive<N>_…`). EVERY name in an expression must be substituted with its registered
  name via `self.parameterName(PARAM_…)`, including inside multi-term expressions** (e.g. the root-circle
  diameter is `'2 * ({} - {} - {} - {})'.format(parameterName(PinCircleRadius), parameterName(PinRadius),
  parameterName(DiskClearance), parameterName(Eccentricity))`). A bare-name expression like
  `'2 * (PinCircleRadius - …)'` raises **`RuntimeError: 3 : Expression is invalid`** because no parameter
  with that unprefixed name exists.
  | dimension (param names are LOGICAL — prefix each via `parameterName`) | `.parameter.expression` |
  |---|---|
  | eccentricity offset (`O→Od`) | `Eccentricity` |
  | lobe-pitch angle | `360 deg / Lobes` |
  | pin-circle diameter | `PinCircleDiameter` |
  | root-circle diameter | `2 * (PinCircleRadius - PinRadius - DiskClearance - Eccentricity)` (= `2·Rv`) |
  | output-pin-circle diameter | `OutputPinCircleDiameter` |
  | output-hole diameter | `OutputHoleDiameter` |
  | housing inner diameter | `HousingInnerDiameter` |
  | housing / casing outer diameter | `HousingOuterDiameter` |
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

The build loops over **`D = Disc Count`** discs (`D ∈ {1,2}`). Per-disc quantities (`d = 0..D−1`, `T =
DiscThickness`, `g = DiscGap`): centre **`Od_d = O + s_d·E·X̂`** (`s_0 = +1`, `s_1 = −1`); clocking
**`phi_d = d·π`** (disc 1 is disc 0 rotated 180° about `O`); z-base **`z_d = d·(T + g)`** (disc `d` spans
`[z_d, z_d + T]`); plane = `self.plane` for `d=0`, else a construction plane offset `z_d`. Stack top
**`stackTop = (D−1)·(T+g) + T`**. The **per-disc stashes are LISTS** indexed by `d`: `self.diskBodies[d]`,
`self.diskAxes[d]`, `self.lobeSplines[d]`, `self.outputHoles[d]`, `self.lobeDiskCentres[d]`,
`self.discPlanes[d]`. (`self.lobePinCircle` — disc 0's pin circle only — is still scalar, for the ring-pin
projection.) ⚠️ **EVERY Fusion expression string — construction-plane offsets AND extrude extents, not just
`.parameter.expression` dims — must use the PREFIXED parameter name via `self.parameterName(PARAM_…)`.** The
params register under `CycloidalDrive<N>_`, so a bare `'DiscThickness'`/`'DiscGap'` in any
`ValueInput.createByString(...)` raises **`RuntimeError: invalid expression`**. Build the literal string for
the current `D` substituting prefixed names. Let `nT = self.parameterName(PARAM_DISC_THICKNESS)`,
`nG = self.parameterName(PARAM_DISC_GAP)`. Then **`stackTopExpr`** = `nT` for `D=1`, `'2 * {} + {}'.format(nT,
nG)` for `D=2`; the **disc-`d` plane offset** = `'{} * ({} + {})'.format(d, nT, nG)`. (`DiscCount` is a
dropdown, not a param, so it never appears in an expression — only as the literal integer multiplier `d`/`D`.)

```
generate(inputs)
  → processInputs(inputs)            # read selections incl Disc Count (dropdown→int); register params;
                                     #   precompute dims; _resolveDimensions raises evaluate_problems(vals)
                                     #   joined (the same checks shown live by validate_inputs — Validity)
  → generateName()                   # 'Cycloidal Drive (N=…):L'
  → normalize self.plane to a ConstructionPlane
  → for d in range(D):               # build each rotor disc on its own plane/centre/clocking
       buildLobeSketch(d)            # plane(d); 'Rotor Lobe {d+1}': lobe about Od_d clocked phi_d; pin circle
                                     #   on O (d=0 only, stash self.lobePinCircle). Stash Od_d, spline.
       buildDisk(d)                  # extrude sector (T, from plane(d)) → axis ⟂ at Od_d → pattern ×L about
                                     #   self.diskAxes[d] → Join → self.diskBodies[d] 'Cycloidal Disk {d+1}'
       buildOutputHoleSketch(d)      # 'Output Hole {d+1}': output-hole circle + 1 hole on Od_d
       buildOutputHoles(d)           # cut through diskBodies[d] → pattern ×M about diskAxes[d] (M holes)
       buildDiskBore(d)              # 'Disc Bore {d+1}': cut center bore (CenterBearing+BearingClr) on Od_d
  → buildCam()                       # D eccentric sections (section d: outer on Od_d, input bore on O,
                                     #   over [z_d, z_d+T] (+ the gap below the next)) JOINED → 'Eccentric
                                     #   Cam'; the ±E sections overlap centrally so the join is one solid
  → buildRingPins()                  # base annulus 1mm below disc 0 + ring casing around the stack:
                                     #   ONE pin-pitch section (outer arc + swept-envelope spline env+c + 2
                                     #   spokes) → extrude TWO-SIDED (down to base top) → pattern ×N about O →
                                     #   join sectors → JOIN casing+base into ONE 'Housing'. Disc rolls on the
                                     #   smooth conjugate contour; no loose pins.
  → buildOutputPins()                # plate 1mm ABOVE stackTop; pins span [plate top, disc-0 bottom] through
                                     #   ALL discs' output holes → sockets + chamfer → pattern ×M about O
  → buildChamfers()                  # chamfer outer rim of EVERY disc body + 'Housing' + Output Plate
  → buildSubComponents()             # LAST. Organize the finished bodies into 4 sub-components inside the
                                     #   Cycloidal Drive component, by part type: 'Rotor Discs' (Disc 1[,2]),
                                     #   'Housing', 'Eccentric Cam', 'Output' (plate + M pins). moveToComponent.
                                     #   THEN solids.hide_construction_geometry(component): hide all sketches/
                                     #   construction planes/axes, leaving only the solid bodies visible.
```

As with spur, creating the occurrence (first `addParameter` / `getOccurrence`) shifts Fusion's active
component, so **pull both selection inputs (Plane, Anchor) and Parent, and stash them on `self`, before
any parameter registration.** `generateName()` returns `'Cycloidal Drive (N={}):{}'.format(N, L)` where
`N = PinCount` (rounded to int) and `L = N − 1` (the reduction ratio) — e.g. `'Cycloidal Drive (N=16):15'`.

## Instructions

### 0: Two-disc parameterization (read first — generalizes §2–§8)
`D = Disc Count` (`1` or `2`). The per-disc steps `buildLobeSketch`, `buildDisk`, `buildOutputHoleSketch`,
`buildOutputHoles`, `buildDiskBore` each take a **disc index `d`** and run once per disc in a
`for d in range(D)` loop (`[CYCLOIDAL-F-TWO-DISC]`). §2–§4 below describe **`d = 0`** (centre `Od_0 = O +
E·X̂`, clocking `phi = 0`, on `self.plane`); for general `d` substitute:
- **Plane** `plane(d)`: `self.plane` for `d = 0`; else a construction plane `setByOffset(self.plane,
  ValueInput.createByString('{} * ({} + {})'.format(d, parameterName(PARAM_DISC_THICKNESS),
  parameterName(PARAM_DISC_GAP))))` at `z_d = d·(T+g)` — **prefixed param names** (⚠️ above). Stash
  `self.discPlanes[d]`; anchor every disc-`d` sketch to **`O`** on `plane(d)`.
- **Centre** `Od_d = O + s_d·E·X̂`, `s_0 = +1`, `s_1 = −1`. So disc 1's eccentric-offset construction
  (`[CYCLOIDAL-F-DISK-CENTER]`) points to **`−E·X̂`** — use the distance dim expression `Eccentricity` but
  place `Od` at `(−E, 0)` and the spokes/lobe on it. (Generator: a signed `E`.)
- **Clocking** `phi_d = d·π`. Disc 1's lobe is `disk_point(t, cx = −E, cy = 0, phi = π)` — exactly the **180°
  rotation of disc 0 about `O`** (`disk_point(cx=−E, phi=π) ≡ −disk_point(cx=+E, phi=0)`), which is what
  meshes the second disc with the **same** ring pins (even `N`) at the opposite eccentric. The two spokes
  and the angle dim rotate with `phi_d` too (spoke 1 along `−X̂` for disc 1, i.e. the lobe's first valley at
  `Od_1 + Rv·(cos π, −sin π) = Od_1 + (−Rv, 0)`).
- **Names** carry `{d+1}`: `'Rotor Lobe {d+1}'`, `'Cycloidal Disk {d+1}'`, `'Output Hole {d+1}'`,
  `'Disc Bore {d+1}'`. **Stash into the `[d]` lists** (`self.diskBodies[d]`, `self.diskAxes[d]`,
  `self.lobeSplines[d]`, `self.outputHoles[d]`, `self.lobeDiskCentres[d]`). The **pin circle** (on `O`) is
  drawn and stashed (`self.lobePinCircle`) **only for `d = 0`** — the ring pins reuse it.
- The disc-`d` extrude, axis (`buildDiskAxis` from `plane(d)`-extrude cap face at `Od_d`), lobe pattern, and
  output-hole pattern are all about **`self.diskAxes[d]`** (at `Od_d`). `buildDiskBore(d)` is the center-bore
  cut (formerly inside `buildCam` step 1) on `Od_d` through `self.diskBodies[d]`.

`buildCam`, `buildRingPins`, `buildOutputPins`, `buildChamfers` then run **once** over the whole stack (see
their sections + `[CYCLOIDAL-F-TWO-DISC]` for the `stackTop`/two-section deltas). For `D = 1` everything
reduces to today's single-disc build.

### 1: Normalize the Target Plane
If the selected plane is not a `ConstructionPlane`, make a coplanar one via
`ConstructionPlaneInput.setByOffset(selectedPlane, 0)` and use it (`[SPUR-F` parity).

### 2: Fully-constrained lobe, on the eccentric disk centre — `buildLobeSketch(d)`
(Takes the disc index `d` per **§0**; the text below is the `d=0` case — for general `d` substitute
`plane(d)`, centre `Od_d = O + s_d·E·X̂`, clocking `phi_d = d·π`, and the `'… {d+1}'` names / `[d]` stashes.)
Create a sketch named `Rotor Lobe {d+1}` on `plane(d)`. Anchor a fresh local origin `O` (the drive axis)
to the Anchor (`[CYCLOIDAL-F-ANCHOR-CHAIN]`), then make the **eccentric disk centre** `Od_d = O + s_d·E·X̂`
(`[CYCLOIDAL-F-DISK-CENTER]` — a point on a horizontal construction line from `O`, with a driving distance
dimension = `Eccentricity`; `s_d` is the sign). The lobe is `disk_point(t, cx = s_d·E, cy = 0, phi = d·π)`
(disc 1 = disc 0 rotated 180° about `O`). Per `[CYCLOIDAL-F-DISK-LOBE]`, build in order:
1. **pin circle** (radius `R`) construction, centred on **`O`** (the fixed ring), diameter dim, with an
   **along-path text label `'Pin Circle'`** — **for `d = 0` only**: **stash it on `self.lobePinCircle`** so
   `buildRingPins` projects it (`[CYCLOIDAL-F-RING-PINS]`). (Disc 1 doesn't need its own pin circle; draw it
   for the lobe reference if convenient but the ring pins use disc 0's.)
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
Root-circle-centre sketch point (`diskCentre`, `Od_d`) on `self.lobeDiskCentres[d]`** and the lobe spline on
**`self.lobeSplines[d]`** for the next steps.

### 2·A: Extrude + axis + pattern → the rotor disk — `buildDisk(d)` (and `buildDiskAxis(capFace, d)`)
(Takes the disc index `d` per §0.) Build disc `d` from its Rotor-Lobe sketch's **one closed profile** — the
**lobe pie-sector** bounded by spoke 1 + the lobe spline + spoke 2 (`[CYCLOIDAL-F-DISK-BODY]`):
1. **Extrude** that sector profile by **`Disc Thickness`** as a **New Body** `'Cycloidal Disk {d+1}'`
   (`PositiveExtentDirection`, from `plane(d)`). ⚠️ Select the sector profile by the one whose loop contains
   **`self.lobeSplines[d]`** — **not** `profiles.item(0)`, since the along-path text labels add their own
   letter profiles.
2. **Disk axis — `buildDiskAxis(capFace, d)`** (`[CYCLOIDAL-F-DISK-AXIS]`): now that a body exists, create the
   construction axis **perpendicular to the sketch at the disc centre `Od_d`** **from the extrude's planar cap
   face** via `setByPerpendicularAtPoint(capFace, self.lobeDiskCentres[d])`. Name it `'Disk Axis {d+1}'`,
   stash on **`self.diskAxes[d]`**. ⚠️ **Pick the cap by NORMAL (∥ sketch normal) — use
   `extrude.startFaces.item(0)`, NOT a nearest-/contains-`Od` planar-face search**: the pie-sector's two
   **spoke faces are also planar and also contain the apex `Od_d`**, so a proximity search grabs a spoke → an
   **in-plane (horizontal) axis** → the pattern comes out garbage (`[CYCLOIDAL-F-DISK-AXIS]`). ⚠️ **Do NOT
   use `setByLine`** and do **NOT** `activate()`.
3. **Circular-pattern the extrude FEATURE ×`L` (= `N − 1`)** about **`self.diskAxes[d]`** over `360°` — pass
   the **`ExtrudeFeature`** to `circularPatternFeatures.createInput`, **not** its body; set
   `patternComputeOption = AdjustPatternCompute` (`[CYCLOIDAL-F-OUTPUT-HOLES]`). The `L` sectors tile disc `d`.
4. **Join ×`L` → one disc body.** ⚠️ **Join ONLY disc `d`'s OWN `L` sectors — not the whole component.** With
   two discs, disc 0's body already exists when disc 1 builds, so `bRepBodies.item(0)` is the WRONG target.
   Record **`base = component.bRepBodies.count` BEFORE step 1's extrude**; disc `d`'s sectors are then
   `bRepBodies.item(base) … item(base + L − 1)`. `combineFeatures` Join: target = `bRepBodies.item(base)`,
   tools = the other `L − 1` of disc `d`'s sectors. Name the result `'Cycloidal Disk {d+1}'` and stash
   **`self.diskBodies[d]`**.

### 3: Output hole, separate sketch — `buildOutputHoleSketch(d)`
(Takes the disc index `d` per §0; uses `plane(d)` and `Od_d`.) Create a **new** sketch named `Output Hole
{d+1}` on `plane(d)` (so the lobe and hole profiles never share a sketch). Anchor a local origin to `O` and
rebuild the eccentric disk centre `Od_d` (`[CYCLOIDAL-F-DISK-CENTER]`). Then, on `Od_d`
(`[CYCLOIDAL-F-OUTPUT-HOLE]`): an **output-hole circle** (radius `Rop`, construction, driving diameter dim)
and **one solid hole** of derived diameter `OutputHoleDiameter = D_pin + 2E`, seated on the output-hole
circle, with a **driving diameter dimension** and its centre fully pinned (on the `Rop` circle + a
horizontal spoke from `Od_d`). Leave the sketch **visible**; stash the solid hole circle on
**`self.outputHoles[d]`** for the cut step. No bodies here. (Hole clocking: the `M`-fold pattern in §4 means
the half-turn clocking of disc 1 maps the hole set onto itself for even `M`, so just centre on `Od_d`.)

### 4: Cut + pattern the output holes — `buildOutputHoles(d)`
Using the `Output Hole {d+1}` sketch's solid hole (`[CYCLOIDAL-F-OUTPUT-HOLES]`): select the profile whose
loop contains **`self.outputHoles[d]`** (not `profiles.item(0)` — the construction circle + text label add
other profiles), **extrude-cut** it through **`self.diskBodies[d]`** by `Disc Thickness`
(`CutFeatureOperation`, `participantBodies = [self.diskBodies[d]]`), then **circular-pattern that cut
`ExtrudeFeature` ×`M`** (= `Output Pin Count`) about **`self.diskAxes[d]`** over `360°` — pass the
**feature**, not a body. ⚠️ Set **`patternComputeOption = AdjustPatternCompute`** on the pattern input — a
lone patterned CUT fails with `NO_TARGET_BODY / NO_PASTE_INT_EDGES` under the default paste compute
(`[CYCLOIDAL-F-OUTPUT-HOLES]`). → `M` output holes orbiting `Od_d`. Done.

### 5: Housing base + ring casing (no discrete pins) — `buildRingPins()`
The fixed reaction member is **ONE `Housing` body on `O`**, built as two extrusions then **Combined**: a base
annulus `1 mm` below the disc (mounting base), and a ring casing surrounding the disc stack whose inner wall
**contours the cycloidal path** — the rolling surface is integral, not loose pins (`[CYCLOIDAL-F-RING-PINS]`,
`epitrochoid-trace.md` "Pinless ring casing"). The casing reaches **down to the base top** so the two meet and
Join into one connected, printable part. `stackTopExpr` per §0.
1. **Housing plane.** Construction plane `setByOffset(self.plane, '-1 mm')`, `1 mm` below the disc (opposite
   the disk's `+normal` extrude). Name `'Ring Housing Plane'`.
2. **Housing base — sketch `Housing Ring`** on that plane, anchored to `O`: a plain **annulus** (outer
   `R − Rr + 2·E + Wall` [the thin pinless wall — contour peak + `Wall`], inner `R − Rr − Wall`, solid, diameter dims → `HousingOuterDiameter` /
   `HousingInnerDiameter`). Extrude the **annulus profile** (`profileLoops.count == 2`) by `Base Thickness`
   in **`NegativeExtentDirection`** (away from the disc) as a New Body `'Housing Ring'`. Stash
   `self.housingRing`. (No pins, no projected circle — just the base.)
3. **Drive axis at `O`.** Construction axis `'Drive Axis'` ⟂ the sketch at `O`, from the Housing-Ring
   extrude's **cap face by NORMAL** (`startFaces.item(0)`) via `setByPerpendicularAtPoint(capFace, origin)`.
   Stash `self.driveAxis`. (Reused by the casing pin pattern and the output-pin pattern.)
4. **Ring casing — ONE section, patterned ×`N`** (like the disc's lobe-sector → pattern → join;
   `[CYCLOIDAL-F-RING-PINS]`, `epitrochoid-trace.md` "Pinless ring casing"). The inner wall follows the disc's
   **swept envelope** `contour(φ) = env(φ) + c` (smooth — replaces the old constant-radius bridge circle).
   - **Compute one pin-pitch of the contour.** Sweep the disc `disk_point(t, E·cosθ, E·sinθ, −θ/L)` over
     `θ, t ∈ [0, 2π)` (≈ 240 each); bin points whose angle is in `[−π/N, π/N]` into `nbins` by angle, keep max
     radius per bin → `env`. ⚠️ **Emit the points at bin EDGES so the first/last land EXACTLY on `∓π/N`**:
     `nbins+1` points at `φ_i = −π/N + (2π/N)·i/nbins` (`i = 0…nbins`), radius `c + max(binMax[i−1], binMax[i])`
     — **NOT bin centres**, which inset the ends by half a bin and leave a gap between every patterned sector so
     the ×`N` sectors don't touch and won't Join into one casing (the "several unnamed bodies" bug). Point =
     `(r_i·cosφ_i, r_i·sinφ_i)`, ordered by angle, in cm. (`N`-fold symmetric, pin centred at `φ = 0`; the ends
     at `±π/N` are mid-gap peaks, tangential by symmetry → seamless tiling. See `[CYCLOIDAL-F-RING-PINS]`.)
   - **Section sketch `Ring Casing`** on `self.plane`, anchored to `O`: an **outer circle** (`R − Rr + 2·E + Wall`,
     **SOLID — NOT construction**; it forms the sector's outer arc, so the wedge profile closes; dim →
     `HousingOuterDiameter`); a **fitted spline** through the contour points (open); and **two radial
     spokes** from the spline's two ends (at `φ = ±π/N`, on the contour) out to the outer circle.
   - **Extrude the SECTOR profile** (the thin annular pie wedge bounded by the outer arc + contour spline + the
     two spokes). ⚠️ **Select it by MINIMUM AREA among profiles whose loop contains the contour spline — NOT
     just "contains the spline", and NOT `profiles.item(0)`.** The SOLID outer circle makes the open contour
     spline a shared edge of **two** closed profiles — the thin wedge *and* the large complement (everything
     else inside the outer circle) — both of which contain the spline; picking the complement extrudes a
     near-full disc that patterns ×`N` into a **solid cylinder** (scallops erased). Take the smallest-area
     profile (`areaProperties(...).area`) to get the wedge. See `[CYCLOIDAL-F-RING-PINS]`. Extrude it
     **TWO-SIDED**: `stackTopExpr` in the `+normal` (up to `stackTop`) **and `'1 mm'` in the `−normal`** (down
     to the base top, closing the gap so the casing can Join the base). **Circular-pattern ×`N`** (= `Pin
     Count`) about `self.driveAxis` (`patternComputeOption = AdjustPatternCompute`), then **Join** the `N`
     sector bodies into one casing body. The section ends fall on valley midpoints (tangential by symmetry) so
     the joined wall is smooth. The disc valleys roll on the contour near each pin (clearance `c`); the lobe
     tips clear it between pins.
5. **Combine into one housing.** **Join** the casing body (tool) into `self.housingRing` (target, the base) —
     `combineFeatures`, `JoinFeatureOperation`. The casing bottom is coincident with the base top (step 4), so
     the result is one connected solid. Rename `self.housingRing.name = 'Housing'`; keep `self.housingRing` =
     the combined body and set `self.ringCasing = None` (consumed by the Join; `buildChamfers` skips a `None`
     casing, so the housing is chamfered once). Done.

The combined `Housing` spans `[−1 mm−BaseThickness, stackTop]` — the base floor below, the scalloped reaction
wall around the disc stack, one printed part to mount. No separate pins, no sockets.

### 6: Disc center bore (per disc) + eccentric cam — `buildDiskBore(d)` and `buildCam()`
The disk rides on the eccentric cam on a plain (journal) bearing: cam **outer** on the disc centre `Od_d`
(diameter `Center Bearing Diameter`), **input-shaft bore** on the drive axis `O` (diameter `Input Shaft
Diameter`); the `E` offset makes it eccentric. Each disc's **center bore** is enlarged over the cam by
`Bearing Clearance` (concentric running gap) so the cam turns freely (`[CYCLOIDAL-F-CAM]`).

**`buildDiskBore(d)`** (runs per disc, in the §0 loop): sketch `'Disc Bore {d+1}'` on `plane(d)`, anchored to
`O`, rebuild `Od_d`. One **solid** circle on `Od_d`, diameter dim `.parameter.expression =
'CenterBearingDiameter + BearingClearance'`. Extrude-**cut** it (all the sketch's profiles) through
**`self.diskBodies[d]`** by `Disc Thickness` (`CutFeatureOperation`, `participantBodies =
[self.diskBodies[d]]`, `PositiveExtentDirection`). → each disc gets a clean center bore.

**`buildCam()`** (once, over the stack): build **`D` eccentric sections** and join them into one
`'Eccentric Cam'`. Section `d`: sketch `'Eccentric Cam {d+1}'` on `plane(d)`, anchored to `O`, rebuild
`Od_d`; **cam-outer circle** on `Od_d` (dim → `CenterBearingDiameter`); if `Input Shaft Diameter > 0`,
**input-bore circle** on `O` (dim → `InputShaftDiameter`). Extrude the **cam cross-section** (2-loop annulus
selected by `profileLoops.count == 2` — ⚠️ **NOT** `find_profile_by_curve_counts`, see `[CYCLOIDAL-F-CAM]`;
or 1-loop disc if no bore) as a New Body — section `d` extends from `plane(d)` toward the disk for **`T + g`**
when `d < D−1` (filling the inter-disc gap so adjacent sections meet) and **`T`** for the last section.
After all sections exist, **Join** them (`combineFeatures` Join, target = section 0, tools = the rest) into
one `'Eccentric Cam'`; the `±E` sections overlap in the central region (centres only `2E` apart, radius
`CenterBearingDiameter/2`), so the join is one continuous solid with the input bore running through. Stash
`self.cam`. (For `D = 1` this is exactly the old single cam.)

### 7: Output pins + plate — `buildOutputPins()`
The **output member**: a solid plate above the **whole disc stack** carrying `M` output pins that hang
**down** through **every** disc's `M` output holes (for even `M`, all discs' holes share the same `M` angular
positions, so the same pins thread all discs — `[CYCLOIDAL-F-TWO-DISC]`). Built like the ring housing,
mirrored to the `+normal` side (`[CYCLOIDAL-F-OUTPUT-PINS]`). Use the **PREFIXED `stackTopExpr`** (§ call
graph ⚠️: `nT` for `D=1`, `'2 * {} + {}'.format(nT, nG)` for `D=2` — never bare `'DiscThickness'`):
1. **Output plate plane.** A construction plane `setByOffset(self.plane, ValueInput.createByString(
   stackTopExpr + ' + 1 mm'))` — `1 mm` above the **top** disc, **opposite** the housing. Name `'Output Plate
   Plane'`.
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
   (toward the disk) = `stackTopExpr + ' + 1 mm'` (reaching disc-0's bottom `z=0`, so the pin runs through
   **all** discs' output holes). New Body `'Output Pin'`; keep the `ExtrudeFeature` and its body.
5. **Socket (combine-cut, keep the pin).** `combineFeatures` Cut: target `self.outputPlate`, tool the output
   pin body, `isKeepToolBodies = True` → a matching hole in the plate, the pin seated in it.
6. **Chamfer the pin ends** (if `Chamfer Size > 0`): `chamferFeature = self._chamferCapRims(pinBody)`
   (`[CYCLOIDAL-F-CHAMFERS]`). Keep the `ChamferFeature` (may be `None`).
7. **Pattern ×`M` (= `Output Pin Count`) about `self.driveAxis`** (reuse the drive axis at `O` from
   `buildRingPins`): an `ObjectCollection` with the pin `ExtrudeFeature`, the socket `CombineFeature`, and the
   pin-end `ChamferFeature` (when present). → `M` output pins, each through a plate socket and orbiting
   through the disk's output holes.
8. **Name all `M` pin bodies** so `buildSubComponents` can group them by name. Capture `pinBase =
   component.bRepBodies.count` **before** the step-4 pin extrude; the `M` pin bodies are then the contiguous
   block `bRepBodies.item(pinBase … pinBase+M−1)` (the socket Cut adds no body; only the pin `NewBody` extrude
   and its `M−1` pattern instances do). Rename each: `body.name = 'Output Pin {}'.format(k+1)` for `k =
   0…M−1`. (The seed pin's step-4 `'Output Pin'` name is overwritten to `'Output Pin 1'`.) Done.

### 8: Edge chamfers — `buildChamfers()`
If `Chamfer Size > 0`, chamfer the **outer rim** (both flat faces) of each disc-like body via
`self._chamferCapRims(body)` (`[CYCLOIDAL-F-CHAMFERS]`): **every disc in `self.diskBodies`** (each rotor
disc — its rim is the **lobe profile**), `self.housingRing` (the combined `Housing` — base + casing in one
body), and `self.outputPlate`. **Guard each against `None`** — `self.ringCasing` is `None` after §5's Join
(consumed into `self.housingRing`), so it is skipped; chamfering only `self.housingRing` covers the whole
housing once. ⚠️ `_chamferCapRims` chamfers **only the two axially-extreme caps** (topmost + bottommost), NOT
every cap-normal face — the combined `Housing` has an internal ledge at the base/casing junction whose outer
loop is the scalloped contour, and chamfering it throws `ASM_BL_CAP_COMPLEX` (`[CYCLOIDAL-F-CHAMFERS]`). Inner
edges (bores, output holes, the casing's inner pin contour) are left sharp — outer rim only. (The
output pins are already chamfered in their own build step, inside the pattern; the casing's pins are integral
so they need no separate chamfer.) If `Chamfer Size == 0`, do nothing. Done.

⚠️ **Resilient chamfers — a chamfer that won't compute must NOT abort the build.** Fusion raises
`RuntimeError` from `chamfers.add(ci)` ("could not create fillet/chamfer at the requested size" /
`ASM_BL_…`) when the requested `Chamfer Size` is too large for the geometry — e.g. an equal-distance
chamfer on a rotor disc's **lobe contour** self-intersects at the tight valleys once the lobes shrink at a
small `Pin Circle Diameter`. So **wrap the `chamfers.add(ci)` call in `_chamferCapRims` in
`try/except`**: on failure, `futil.log` the reason, increment a counter **`self.chamfersSkipped`** (a
plain int), and `return None` — do **not** re-raise. Initialize `self.chamfersSkipped = 0` in
`processInputs` (before any build step), since `_chamferCapRims` is also used for the **pin-end** chamfers
(§7 step 6), so the counter accumulates across all chamfer attempts. At the **very end of `generate()`**
(after `buildSubComponents`), if `self.chamfersSkipped > 0`, show a **non-fatal** message and continue
(the part is already built): `adsk.core.Application.get().userInterface.messageBox('Cycloidal drive
generated, but {n} chamfer(s) could not be created at Chamfer Size {sz} mm and were skipped. Reduce
Chamfer Size (or set it to 0) for this geometry.'.format(n=self.chamfersSkipped,
sz=to_mm(self.chamferSize)))`. This keeps the "make it smaller" path producing a part with a clear note,
instead of aborting with a raw Fusion traceback.

### 9: Organize bodies into sub-components — `buildSubComponents()`
**LAST step, after everything is built and chamfered.** Group the finished bodies into **four sub-components
inside the Cycloidal Drive component**, one per part type, so the browser tree is a tidy assembly rather than a
flat body list (`[CYCLOIDAL-F-SUBCOMPONENTS]`). Run it **last** because `moveToComponent` invalidates the
moved body's reference and `buildChamfers` (and the earlier build steps) still need the bodies in the root
Cycloidal Drive component. The four groups, matched **by body name** (robust against stale refs):
- **`Rotor Discs`** — every body whose name starts with `'Cycloidal Disk'` (Disc 1, and Disc 2 when `D=2`).
- **`Housing`** — the body named `'Housing'`.
- **`Eccentric Cam`** — the body named `'Eccentric Cam'`.
- **`Output`** — the bodies named `'Output Plate'` and `'Output Pin …'` (plate + all `M` pins).

For each non-empty group: `occ = component.occurrences.addNewComponent(adsk.core.Matrix3D.create())` (identity
transform → bodies keep their world position), `occ.component.name = <group name>`, then `body.moveToComponent(
occ)` for each body in the group. ⚠️ **Snapshot all bodies into the four name-keyed lists FIRST** (one pass over
`component.bRepBodies`), THEN create occurrences and move — moving mutates `component.bRepBodies`, so iterating
it while moving would skip bodies. `moveToComponent` returns the moved body or `None` on failure; ignore the
return (the body is already relocated).

**Finally, hide all construction geometry.** Call **`solids.hide_construction_geometry(component)`** (recursive
— hides every sketch, construction plane, and construction axis under the Cycloidal Drive component and its new
sub-components, leaving only the solid bodies visible). Use the shared helper; do NOT re-implement it. Done.
