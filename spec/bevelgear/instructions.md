# Bevel Gear Creation Instructions

This file is the **design & geometry intent**. The bevel-specific **Fusion-API realization** (the §2
lattice constraint style, sketch-local apex positioning, the full-constraint gate and its
exemptions, the cleanup recipe) lives in the sidecar **`fusion.md`** next to this file and is cited
by anchor (`[BEVEL-F…]`). Cross-gear Fusion conventions are cited as `[PB-…]` (shared `PLAYBOOK.md`),
and the spiral-tooth geometry derivation is in `spiral-tooth-trace.md`. Read them together; the
cited rules are as binding as this body.

**Sketch-first status (`[PB-SKETCH-FIRST]`) — a waiver, not a proof.** Bevel has **no**
`spec/bevelgear/sketch/` bench proof: the constrained Gear Profiles (§2) sketch predates the
sketch-first gate, and its constraint scheme has only been exercised inside Fusion, where the
runtime `[BEVEL-F-FULL-CONSTRAINT]` gate passes on the working builds. The `[PB-SKETCH-FIRST]`
gate is therefore **waived** for bevel as it stands: a regen keeps the §2 constraint scheme exactly
as specified here (it may not invent a new scheme, and may not claim bench-proven status).
Reproducing §2 in the standalone sketch engine is future work; when that proof lands under
`spec/bevelgear/sketch/`, it replaces this waiver.

## Component Setup

Bevel gears are inherently created in pairs. As such, we shall have a single component containing three components

The first two components are for the driving gear and pinion gear. Because the designs are shared between the two components, the third one will be the design component.

## Variables

User inputs are listed below in the order they appear in the command dialog. Target Plane comes first so it receives keyboard/pick focus when the dialog opens (Fusion auto-focuses the first selection input — `[PB-AUTOFOCUS-FIRST]`); Center Point follows so the user flows naturally from plane to point; Parent Component comes third since it is already pre-selected to the root component. Calculated values are listed after the inputs they depend on.

Target Plane: user-specified plane. This is where the bottom of the driving gear will sit flush against.

Center Point: user-specified point. This is where the driving bevel gear will be centered on. Does not need to be co-planar with target plane.

Parent Component: user-specified component. Defaults to the root component (pre-selected).

Module: user-supplied number. Specifies the module of gears.

Shaft Angle: User-supplied angle in degrees between 30° and 150°. Default 90° (perpendicular shafts — the classic bevel pair). The input is a `deg` Fusion expression (e.g. `60 deg`); radians read-back (`[PB-EVAL-EXPRESSION]`) — convert to degrees before the 30–150° range check.

Driving Gear Teeth: user-specified number of teeth on the driving gear. Default is 31. (The dialog label is `Driving Gear Teeth`, per the input table; formulas below refer to this value as Driving Gear Teeth Number.)

Pinion Gear Teeth: user-specified number of teeth on the pinion gear. Default is 31. (Formulas below refer to this value as Pinion Gear Teeth Number.)

Driving Gear Pitch Diameter: calculated number. Module * Driving Gear Teeth Number.

Pinion Gear Pitch Diameter: calculated number. Module * Pinion Gear Teeth Number.

Driving Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Pinion Gear Base Height: user-specified positive number, default 0mm (i.e. unspecified).

Enable Bore: user-specified boolean, default `true`. Applies to both gears. When unchecked, no bore is cut on either gear and the per-gear bore diameter inputs below are ignored.

Driving Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Driving Gear Pitch Diameter / 4` as the bore diameter.

Pinion Gear Bore Diameter: user-specified positive number, default 0mm. Only consulted when Enable Bore is checked. A value of 0 means "auto-calculate" — use `Pinion Gear Pitch Diameter / 4` as the bore diameter.

Face Width: User-specified positive number. If unspecified, default to (Cone Distance / 6). In **every** case (default or user-specified) the Face Width is bounded by the Maximum Face Width (defined below):
- If unspecified, use `min(Cone Distance / 6, Maximum Face Width)`.
- If the user specifies a value greater than the Maximum Face Width, this is an error: reject it with a message stating the maximum, rather than proceeding (the gear-body revolve in "Create the Gear Bodies" would otherwise fail — see the Maximum Face Width rationale).

Cone Distance: calculated number. `sqrt((Module * Driving Gear Teeth Number)**2 + (Module * Pinion Gear Teeth Number)**2)`.

Maximum Face Width: a geometric upper bound that cannot be evaluated until the Gear Profiles sketch points A, B, C, D, H, J exist (see §2 — apply the bound there). It is `0.95 *` the smaller of:
- the perpendicular distance from point A to the line through C and H (the Pinion Gear Dedendum line, i.e. Apex2->C extended), and
- the perpendicular distance from point B to the line through D and J (the Driving Gear Dedendum line, i.e. Apex2->D extended).

**Compute both distances from the points' SOLVED sketch geometry — `pointA.geometry`, `pointB.geometry`, `pointC.geometry`, `pointD.geometry`, `pointH.geometry`, `pointJ.geometry` — NOT from the pre-solve seed coordinates (`[PB-SOLVED-GEOMETRY]`).** By the time §2 reaches this step the constraint network has located all six, so `.geometry` is exact; seeds diverge substantially for asymmetric tooth counts (e.g. Driving 17 / Pinion 31) or non-90° shaft angles, making a seed-based bound too loose on the binding side — the toe still crosses the axis and the cap is defeated.

Rationale (do not drop this when regenerating): the toe line M->N is C->H offset *toward the Apex* by Face Width, with N pinned to line A->Apex2; its mirror O->P is D->J offset toward the Apex by Face Width, with P pinned to line B->Apex2. When the offset reaches the perpendicular distance from A to line C->H, point N lands exactly on A; any larger value drives N **past** A, across the gear's own shaft axis (Apex->A). The frustum profile (hexagon A, G, H, C, M, N, built here in §2 and revolved later in "Create the Gear Bodies") is revolved about that shaft axis, so a profile that has crossed the axis self-intersects the axis of revolution and Fusion aborts the revolve with `ASM_WIRE_X_AXIS` (`[PB-REVOLVE]`). The pinion side is normally the binding one (its smaller pitch radius gives the smaller distance), but compute both and take the minimum so the bound holds for any Shaft Angle. The `0.95` factor keeps N clearly off A, since a near-coincident N≈A degenerates the toe edge even before it strictly crosses. At Shaft Angle 90° this limit equals `Pinion Gear Pitch Diameter**2 / (2 * Cone Distance)`, so the naive `Cone Distance / 6` default exceeds it — and the gear fails to generate — for any gear ratio above roughly √2 (e.g. Driving 31 / Pinion 17).

Tooth Spacing: user-specified non-negative number in mm, default 0mm. A single value applied to **both** gears. It is a clearance offset that shifts each virtual spur tooth profile's **center** radially outward along the dedendum line — *away from the lower corner* (the rim corner opposite the Apex: point C for the pinion, point D for the driving gear), i.e. in the C->K direction beyond K (and D->L beyond L) — by this distance, **while the tooth itself is still drawn at the original virtual pitch radius** (virtual tooth number × Module / 2; the virtual tooth number is unchanged). At 0 (the default) the tooth center sits exactly at K / L; a positive value moves the center farther from the rim, loosening the mesh so 3D-printed teeth have more clearance. Applied in §3; see "Gear Tooth Profiles".

Mean Spiral Angle (ψ): user-specified angle in degrees, default 35°, valid range **[0, 60)**. The angle between the tooth trace and the cone element, measured at the mean cone distance (see `spiral-tooth-trace.md`). **ψ = 0 means a STRAIGHT bevel gear** — the tooth-body build takes the original straight path unchanged (apex-point loft + the two conical trims) and every spiral input below is ignored; any value **> 0 builds a curved (spiral) tooth**. The driving gear uses this hand; the meshing pinion is built with the **opposite** hand (mirror) so the pair meshes. Input is a `deg` Fusion expression; radians read-back (`[PB-EVAL-EXPRESSION]`) — convert to degrees before the [0, 60)° range check.

Hand of Spiral: user-specified dropdown — `Right` (default) or `Left`. The **driving** gear's hand of spiral; the pinion is built with the opposite hand. Only consulted when ψ > 0. **Shown in the dialog only when ψ > 0** (hidden for straight bevels — see "Conditional visibility" under Exact input ids). The two list-item strings `Right`/`Left` are reproduced surface (module constants `_HAND_RIGHT = 'Right'`, `_HAND_LEFT = 'Left'`).

Cutter Radius: user-specified non-negative number in mm, default 0mm. The face-mill cutter radius `r_c` that sets the radius of the tooth-trace arc (see `spiral-tooth-trace.md`). **0 means auto** — use the mean cone distance `R_mean` as `r_c`. Only consulted when ψ > 0. **Shown in the dialog only when ψ > 0** (hidden for straight bevels — see "Conditional visibility" under Exact input ids). Reject negative values.
### Exact input ids and parameter-name strings

These literal strings are part of the reproduced surface. Use them verbatim. The dialog **display
order** (the order `configure()` adds inputs) is fixed as the rows below — Target Plane first so it
wins Fusion's auto-focus, then Center Point, then the pre-selected Parent Component, then the
numeric/bool fields. Module-level constants name the input ids (`INPUT_ID_PLANE = 'targetPlane'`, …).

| # | Dialog input | input id | input type | unit | default | selection filters / tooltip |
|---|---|---|---|---|---|---|
| 1 | Target Plane | `targetPlane` | `addSelectionInput` | — | — | `ConstructionPlanes`, `PlanarFaces`; limit 1; tooltip `Plane the bottom of the driving gear sits flush against` |
| 2 | Center Point | `centerPoint` | `addSelectionInput` | — | — | `ConstructionPoints`, `SketchPoints`; limit 1; tooltip `Point the driving bevel gear is centered on` |
| 3 | Parent Component | `parentComponent` | `addSelectionInput` | — | root component pre-selected | `Occurrences`, `RootComponents`; limit 1; tooltip `Component the gear pair is created under` |
| 4 | Module | `module` | `addValueInput` | `''` | `createByReal(1)` | — |
| 5 | Shaft Angle | `shaftAngle` | `addValueInput` | `deg` | `createByString('90 deg')` | — |
| 6 | Driving Gear Teeth | `drivingTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 7 | Pinion Gear Teeth | `pinionTeeth` | `addValueInput` | `''` | `createByReal(31)` | — |
| 8 | Driving Gear Base Height | `drivingBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 9 | Pinion Gear Base Height | `pinionBaseHeight` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 10 | Enable Bore | `boreEnable` | `addBoolValueInput` (checkbox) | — | `True` | — |
| 11 | Driving Gear Bore Diameter | `drivingBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 12 | Pinion Gear Bore Diameter | `pinionBore` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 13 | Face Width | `faceWidth` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 14 | Tooth Spacing | `toothSpacing` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
| 15 | Mean Spiral Angle | `spiralAngle` | `addValueInput` | `deg` | `createByString('35 deg')` | — |
| 16 | Hand of Spiral | `spiralHand` | `addDropDownCommandInput` (text-list) | — | items `Right` (selected), `Left` | — |
| 17 | Cutter Radius | `cutterRadius` | `addValueInput` | `mm` | `createByReal(to_cm(0))` | — |
There are now **17** dialog inputs and **17 `INPUT_ID_*`** module constants, named exactly:
`INPUT_ID_PLANE`, `INPUT_ID_CENTER_POINT`, `INPUT_ID_PARENT`, `INPUT_ID_MODULE`,
`INPUT_ID_SHAFT_ANGLE`, `INPUT_ID_DRIVING_TEETH`, `INPUT_ID_PINION_TEETH`,
`INPUT_ID_DRIVING_BASE_HEIGHT`, `INPUT_ID_PINION_BASE_HEIGHT`, `INPUT_ID_BORE_ENABLE`,
`INPUT_ID_DRIVING_BORE`, `INPUT_ID_PINION_BORE`, `INPUT_ID_FACE_WIDTH`, `INPUT_ID_TOOTH_SPACING`,
`INPUT_ID_SPIRAL_ANGLE`, `INPUT_ID_HAND`, `INPUT_ID_CUTTER_RADIUS` — holding the table's id
strings in row order. Inputs 15–17 are appended **after** Tooth Spacing in display order. The Hand dropdown is a
`DropDownStyles.TextListDropDownStyle` with `Right` added selected and `Left` added unselected; read
its `selectedItem.name` (default `Right` if none). `spiralAngle` reads back in radians (range-check
[0, 60)° after converting to degrees); `cutterRadius` reads back in internal cm via `'mm'`. Still **no live
Fusion user parameters** — the spiral values are precomputed in Python like everything else.

**Conditional visibility — the spiral-only inputs show only when ψ > 0.** Hand of Spiral
(`spiralHand`) and Cutter Radius (`cutterRadius`) are relevant **only** for curved bevels, so they
are **hidden whenever Mean Spiral Angle ψ = 0 and shown when ψ > 0**. Mean Spiral Angle (`spiralAngle`)
itself is the controller and is **always visible** — it is how the user reaches ψ > 0. Realize this
with the `commandInput.isVisible` property (there is no declarative "show-if" in the Fusion API):
- Add a `@classmethod _updateSpiralInputVisibility(cls, inputs)` helper. It evaluates the
  `spiralAngle` input's **`.expression`** via
  `unitsManager.evaluateExpression(spiral.expression, 'rad')` (internal **radians** — it does NOT
  read the input's `.value`) and sets `inputs.itemById(INPUT_ID_HAND).isVisible` and
  `inputs.itemById(INPUT_ID_CUTTER_RADIUS).isVisible` to `(value > 0)`. **Guard it:** if any of the
  three inputs is `None` return early, and wrap the expression evaluation in `try/except` (a
  half-typed expression can raise mid-edit) — on failure leave both inputs **shown**. `isVisible` only hides the
  dialog row; the input still exists and `_readInputs` reads it normally (and the ψ = 0 build ignores
  Hand/Cutter anyway), so hiding is purely cosmetic and cannot affect generation.
- `configure()` calls `cls._updateSpiralInputVisibility(inputs)` **as its last step**, so the initial
  state is correct (default ψ = 35° → both shown).
- The dialog's `inputChanged` event drives the reactive update through a second classmethod
  `@classmethod def handle_input_changed(cls, args)`, which simply calls
  `cls._updateSpiralInputVisibility(args.inputs)` — recompute on **every** input change (cheap and
  robust; no need to branch on which input changed). `handle_input_changed` is bound by name from
  `commands/bevelgear/entry.py` (see Method contract — external bindings).

Selection filters (`[PB-SELECTION-FILTER-ENUM]`) and limit-1 are set per the table. Each selection
input's tooltip string (the third argument to `addSelectionInput`, shown in the table) is part of
the reproduced surface — use it verbatim. The Parent
selection pre-selects `get_design().rootComponent`. The numeric `mm`/`deg` defaults are passed in
internal units (`to_cm(...)` for lengths; `createByString('90 deg')` for the angle so the
expression engine parses it).

**No live Fusion user parameters.** Unlike the spur family, bevel registers **no** user parameters
under a prefix. Every value (pitch diameters, cone distance, base heights, bore diameters, virtual
tooth counts, face width) is **precomputed in Python in internal cm** and written into geometry
numerically — sketch dimensions via `dimension.parameter.value = <number>` and feature inputs via
`ValueInput.createByReal(<number>)`. There are therefore no `PARAM_*` name strings to reproduce;
the only module-level constants are the 17 `INPUT_ID_*` strings plus `_HAND_RIGHT`/`_HAND_LEFT`.
(`[PB-PRECOMPUTED-MODE]`.)

**Reading the raw numbers.** Read each numeric/angle input by evaluating its expression with units
`''`/`'mm'`/`'deg'` as appropriate; the values come back in Fusion internal units (cm / radians)
regardless of the unit string (`[PB-EVAL-EXPRESSION]`).

**Units — critical (Module is unitless mm; everything else is already internal):**
- The `'mm'` inputs (Driving/Pinion Base Height, Driving/Pinion Bore Diameter, Face Width, Tooth
  Spacing) and the
  `'deg'` input (Shaft Angle) come back from `evaluateExpression(expr, 'mm'|'deg')` **already in
  internal units** (cm / radians). Use them as-is; do **not** `to_cm` them again.
- **`Module` is read with unit `''`, so it comes back as a raw number that means *millimetres*** (a
  module of `1` is 1 mm). Therefore **every length derived from Module must be `to_cm`-converted
  before it touches geometry**: Pitch Diameter = `to_cm(Module * teeth)`, Cone Distance = `to_cm(...)`,
  dedendum = `to_cm(1.25 * Module)`, the module-length construction extensions (E, F, G, H, I, J),
  and the default Face Width (`Cone Distance / 6`). The `VirtualSpurProxy` likewise receives Module
  in mm and applies the standard spur formulas, then `to_cm`'s the resulting circle radii/diameters
  it serves (they must be in cm, matching what the spur tooth generator expects). Mixing a raw-mm
  Module-derived length with an already-cm `'mm'` input (e.g. comparing Face Width against the
  Module-derived Maximum Face Width) without this conversion makes the gear come out ~10× off and
  the Face-Width bound meaningless. The boolean Enable-Bore input is read with
`get_boolean` (or `input.value`). Both teeth inputs are coerced to whole numbers with
`int(round(…))` before validation. Read all inputs up front in a single `_readInputs` pass that
validates ranges (module > 0, teeth ≥ 3, shaft angle 30–150°, non-negative
heights/bores/width/tooth-spacing) and returns the primary values, stashing the rest on `self`.

## Architecture

Bevel uses a **standalone generator** — it does **not** subclass `base.Generator` and uses **no
`GenerationContext`**. **One** generator handles **both** straight and spiral bevels: the same class
builds a straight bevel when Mean Spiral Angle ψ = 0 and a curved (spiral) bevel when ψ > 0. There is
**no separate spiral subclass or command** — the spiral is a branch inside the tooth-body step
(`_transformToothBody`, see Method contract), gated on ψ. Two plain classes plus one framework
import (names are the reproduced surface; the entry point binds the two classes by name):

1. **`BevelGearCommandInputsConfigurator`** — `@classmethod def configure(cls, cmd)` that adds the
   **17** dialog inputs above in display order, plus `@classmethod def handle_input_changed(cls, args)`
   (with a private `_updateSpiralInputVisibility(cls, inputs)` helper) that drives the spiral-only
   inputs' conditional visibility (see "Conditional visibility" under Exact input ids). Both
   `configure` and `handle_input_changed` are bound **by name** from `commands/bevelgear/entry.py`.
2. **`BevelGearGenerator`** — `__init__(self, design)` (stores `self.design`, `self.bevelOccurrence
   = None`); `generate(inputs)`; `deleteComponent()`. Creates the occurrence tree directly with
   `parent.occurrences.addNewComponent(...)` — it does **not** use `getOccurrence` / `addParameter`
   / `parameterName` / `createSketchObject` from `base.Generator`, and registers no user parameters.
3. The virtual-spur proxy is **imported from the framework** — `from .spurproxy import
   VirtualSpurProxy` — a fake spur `parent` so the borrowed spur tooth generator can run without
   registering Fusion user parameters. Bevel defines **no local proxy or value-wrapper class**.
   See Dependencies.

From `base.py` import only the input readers (`get_selection`, `get_boolean`) — its
`Generator`/`ParamNamePrefix`/`ComponentCleaner` machinery is unused. Imports are explicit per the
PLAYBOOK's Module-layout rule (no `import *`).

## Generation Context — none

Bevel carries **no `GenerationContext` object** — no `GenerationContext` class and none of the
`base.Generator` context machinery. State is threaded three ways: (a) `_readInputs`
returns a 7-tuple `(parentComponent, targetPlane, centerPoint, module, drivingTeeth, pinionTeeth,
shaftAngle_deg)` and stashes the rest as instance attributes (`self._drivingBaseHeight_cm`,
`self._pinionBaseHeight_cm`, `self._boreEnable`, `self._drivingBore_cm`, `self._pinionBore_cm`,
`self._faceWidth_cm`, `self._toothSpacing_cm`, `self._spiralAngle_rad`, `self._hand`,
`self._cutterRadius_cm`); `generate()` later stashes the derived `self._coneDistance_cm`,
`self._gamma_p`, `self._gamma_g` (and `_buildGearProfiles` stashes the resolved
`self._faceWidthResolved_cm`); (b) the per-gear geometric anchors are carried in **plain per-gear
dicts** (`pinionCtx` / `drivingCtx` — holding this gear's label, teeth, pitch diameter, γ,
tooth-center point and reference line, hexagon vertices, shaft-edge point pair, toe/heel edges,
toe/heel cone points, root axis, bore diameter, and mesh angle), built in `_buildGearProfiles` and
passed to `_buildVirtualSpurProfile` / `_createGearBody`, which also write the tooth sketch/plane,
`embedded` flag, and virtual tooth count back into the dict; shared anchors are self-stashed
(`self._gearProfilesPlane`, `self._apexSketchPoint`, `self._gpSketch`, `self._apex2d`, the §1
`self._anchorCenterPoint`); (c) `self.bevelOccurrence` holds the top occurrence for cleanup
(`self.designOccurrence` / `self.designComponent` / `self.bevelComponent` hold the inner tree).
The "no ctx" rule means exactly that class-level shape: per-gear plain-dict carriers and self
attributes ARE the intended structure — do not introduce a `GenerationContext`-style class or the
`base.Generator` context.

## Method contract — call graph

No gear subclasses bevel, so there are **no override boundaries to preserve** (unlike spur). The
only hard external bindings are `commands/bevelgear/entry.py` → `BevelGearCommandInputsConfigurator.
configure(args.command)`, `BevelGearCommandInputsConfigurator.handle_input_changed(args)` (the
dialog's `inputChanged` handler delegates one line to it — drives the spiral inputs' conditional
visibility, see Exact input ids), and `BevelGearGenerator(design).generate(inputs)` +
`deleteComponent()` on failure. The internal decomposition below is the intended structure; private helper names may vary,
but keep the step boundaries (they map to the Instructions sections):

```
generate(inputs)
  → _readInputs(inputs)                      # read+validate all inputs; returns 7-tuple, stashes the rest on self
  → resolve pitch diameters & bores (Python, cm)
  → build component tree                     # Bevel Gear → Design (Pinion/Driving components made in _createGearBody)
  → _buildAnchorSketch(design, plane, center)        # §1 → anchorLine
  → _buildGearProfiles(...)                  # §2 + §3 + per-gear body creation; internally PER GEAR,
        # pinion fully first then driving — profile→body INTERLEAVED per gear
        # (pinion profile → pinion body → driving profile → driving body),
        # NOT both profiles then both bodies:
        → _buildVirtualSpurProfile(...)      # §3 this gear: tooth plane + spur tooth + axis
        → _createGearBody(...)               # revolve → loft (uncut tooth) → _transformToothBody → pattern → combine → bore → mesh-rotate → moveToComponent
              → _transformToothBody(...)     # the tooth-body step. ψ=0 → solids.cut_conical_ends (2 conical trims, straight bevel). ψ>0 → spiral build (see §3 "Spiral tooth body")
              # mesh-rotate: driving by 180°/drivingTeeth; pinion by _pinionMeshPhase(pinionTeeth) (0 unless a spiral pair needs a nudge)
  → _hideConstructionGeometry(bevelComponent)        # Cleanup
deleteComponent()                            # error rollback (entry point calls on exception)
```

`_transformToothBody(designComponent, toothBody, gearBody, shaftAxisEdge, apexWorld, apexSketchPoint,
toeMid, heelMid, toeConeWorld, heelConeWorld, parentToothPlane, gearLabel, teethNumber, gamma)` —
`gamma` is this gear's pitch-cone half-angle (γ_p pinion / γ_g driving, from §2), forwarded to the
spiral build's twist law (§3a step G) — is the single
tooth-body hook `_createGearBody` calls after lofting the uncut apex→heel tooth. **`_createGearBody`
builds the four `toeMid / heelMid / toeConeWorld / heelConeWorld` arguments exactly per the §3a
"Caller hand-off" table — toe edge = M→N (pinion) / O→P (driving), heel edge = C→H / D→J; `toeMid`/
`heelMid` are the toe/heel edge MIDpoints and `toeConeWorld`/`heelConeWorld` are M/O and C/D. Getting
this wrong silently inverts the spiral (see §3a).** Its first line is the
gate `if self._spiralAngle_rad <= 0: return cut_conical_ends(...)` (the framework helper from
`.solids` — see "Conical cuts") — straight bevels are byte-for-byte the prior behavior.
`_pinionMeshPhase(pinionTeeth)` returns the pinion's extra mesh rotation about its own
shaft axis (0 for straight; for spiral it is `_PINION_MESH_PHASE_TEETH` tooth-fractions, default 0).

Helpers used by the above come in two kinds.

**Framework helpers (import; do NOT re-implement — behavior pinned in the PLAYBOOK "Shared geargen
helper library"):** from `.solids`: `cut_conical_ends` / `apply_conical_cut` / `select_keeper` /
`find_cone_faces_by_midpoint` / `surface_distance` (the conical-cut machinery — see "Conical
cuts"), `slice_body_by_offset_planes` (§3a step E), `rotate_body_about_edge` (the meshing
rotations), and the spiral-only `plane_by_angle`, `combine_point`, `circle_intersect_nearest`
(§3a steps B–C), plus `hide_construction_geometry` (Cleanup); from `.utilities`:
`find_profile_by_curve_counts` (tooth-profile selection, below); from `.spurproxy`:
`VirtualSpurProxy` (see Dependencies).

**Own helpers (names may vary; behaviour is pinned by the Instructions):** `_readInputs`,
`_cutBore`, and `_pinionMeshPhase(pinionTeeth)` (returns the
pinion's extra mesh rotation in **radians**: `_PINION_MESH_PHASE_TEETH · 2π / pinionTeeth`).

**Tooth-profile selection.** Select the tooth cross-section loop with
`find_profile_by_curve_counts(toothSketch, nurbs=2, arcs=2, lines=wantLines)` — but the line count
is **DETERMINED BY the `embedded` flag, NOT guessed/accepted-either**: `wantLines = 0 if embedded
else 2`. ⚠️ Do **NOT** accept "0 **or** 2 lines" — for a given gear only ONE of those is the real
tooth; an **unrelated** loop (e.g. an inter-tooth or annular region between the drawCircles
circles) can also have 2 NURBS + 2 arcs but the *other* line count, and selecting it makes the
apex→profile loft in "Create the Gear Bodies" fail with `RuntimeError ... ASM_RBI_INTERNAL /
LOFT_NO_TOOLBODY` (the impostor loop can't form a loft tool body). The `embedded` flag is read from
the borrowed spur generator — see Dependencies (`_lastToothEmbedded`); `embedded` ⇒ tip/root/flanks
meet with no connecting lines (4 curves), non-embedded ⇒ 2 connecting lines (6 curves), mirroring
spur's own selection.

Pinion is built first; driving second (mesh offset `180° / drivingTeeth`); the pinion additionally
gets `_pinionMeshPhase(pinionTeeth)` (0 unless a spiral pair needs it).

## Sketch Discipline (bevel-specific)

Bevel's sketch work differs from the spur family. The bevel-specific Fusion mechanics live in
`fusion.md` (cited below); the general Fusion gotchas in `PLAYBOOK.md` still apply. In brief:

- **Every *permanent* sketch must end fully constrained** (Anchor, Gear Profiles §2, the two
  per-gear Profile sketches, Bore) — a free DOF is a generation defect; gate and raise. The two
  tooth-profile sketches (drawn by the borrowed spur generator) and the spiral build's transient
  auxiliary sketches are **exempt**. Gate, exemptions, and rationale: `[BEVEL-F-FULL-CONSTRAINT]`.
- **§2 lattice lines use the COINCIDENT style, not sharing** — build every §2 line (lattice *and*
  short reference/connector lines) from raw `Point3D` coords and `addCoincident` each endpoint to
  its existing point: `[BEVEL-F-COINCIDENT-STYLE]`. Each named §2 line is created **once** and
  reused, never redrawn: `[BEVEL-F-LINE-ONCE]`. The driven §2 lengths are **not** dimensioned:
  `[BEVEL-F-DRIVEN-DIMS]`.
- **The §2 figure is positioned in sketch-local 2-D coordinates** (apex = `c + perp·DPD`), never via
  a world position round-trip — this is what keeps the gear off world XY: `[BEVEL-F-APEX-LOCAL]`.
  The grow side is chosen by the target-plane **normal**, not the sketch's local +Y:
  `[BEVEL-F-GROW-SIDE]`.
- **Never activate any occurrence** (`[PB-NEVER-ACTIVATE]`; bevel reason and the sole spiral-crown
  exception in `[BEVEL-F-NEVER-ACTIVATE]`). **Cleanup hides by entity kind** (`[BEVEL-F-CLEANUP]`).

## Generation Order

`generate()` reads **all** inputs first (`_readInputs`), then creates occurrences. Because bevel
registers no user parameters, nothing creates an occurrence until after every selection is already
read, so the selection-context-shift hazard the spur spec warns about does not bite here — but keep
the order (read inputs → build tree → build geometry) so it stays that way. The geometry steps run
in the order of the Instructions below (§1 → §2 → §3 → Pinion → Driving → Cleanup), pinion before
driving. **When ψ > 0**, the per-gear tooth-body step (inside `_createGearBody`, via
`_transformToothBody`) builds the spiral tooth — trace → slice → rotate → loft → crown → flush trim —
*in place of* the straight tooth's two conical trims, before pattern/combine/bore (see §3 "Spiral
tooth body"). The straight (ψ = 0) order and behavior are unchanged.

## Dependencies

Bevel **borrows the spur tooth generator** — `from .spurgear import
SpurGearInvoluteToothDesignGenerator`. It is used only inside `_buildVirtualSpurProfile`, once per
gear:

```python
proxy  = VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)
drawer = SpurGearInvoluteToothDesignGenerator(sketch, proxy)
drawer.draw(anchorPoint, angle=math.radians(180))   # the 180° tooth rotation IS the draw() angle
```

The borrowed generator's surface (from `spec/spurgear/instructions.md`): constructor `(sketch, parent, angle=0)`;
`draw(anchorPoint, angle=0)` runs `drawCircles()` → `drawTooth(angle)` → anchor-projection; it reads
parameters via `parent.getParameter(name).value`. So bevel must supply a `parent` exposing
`getParameter(name)` → an object with a `.value`. That is the framework's **`VirtualSpurProxy`**
(`lib/geargen/spurproxy.py` — import it; do NOT define a local copy): it precomputes, in internal
cm, exactly the keys the spur drawer reads (its defaults match bevel: pressure angle 20° — not a
bevel dialog input — and `InvoluteSteps` 15) and returns each wrapped in a `.value` carrier.
Construct it as `VirtualSpurProxy(module_mm=module, virtualTeeth=virtualTeeth)` with the raw-mm
module and the §3 virtual tooth number.

**The proxy carries `_lastToothEmbedded` — an OUTPUT the spur generator writes, and bevel MUST read it back.** During `draw()` the spur generator decides whether the tooth is *embedded* (tip/root/flanks meet with no connecting lines) and records it with `self.parent._lastToothEmbedded = <bool>` (it has no ctx of its own); the framework proxy pre-initialises the slot to absorb that write. **After `drawer.draw(...)` returns, read `proxy._lastToothEmbedded` and thread it to the tooth-profile selection (see Method contract)** — e.g. stash it alongside the tooth sketch/plane returned by `_buildVirtualSpurProfile`. This flag is **not optional bookkeeping** — it is the deterministic selector for the tooth loop's line count (`0 if embedded else 2`); skipping it and accepting either count grabs an unrelated loop and the apex→tooth loft dies with `LOFT_NO_TOOLBODY` (see Method contract).

The **180° rotation is delivered through the `draw()` angle argument** (the spur generator rotates
the whole tooth by `angle`, per `spec/spurgear/instructions.md`), *not* a post-hoc Move/sketch rotation — this relies
on spur's radial flank-to-root pinning so the connecting lines rotate with the tooth. (The
construction axis through point K / L, built normal to the tooth plane via `setByTwoPlanes`, is
still created as described in §3.)

## Instructions

### Create the Parent Component

Create the Bevel Gear component as a child of the Parent Component. Name it `Bevel Gear`.

## Creating the Design Component

The Design Component shall contain the necessary sketches and construction planes / axis / etc that will be used by the components creating the actual gears later.

Create the Design component as a child of the Bevel Gear component. Name it `Design`.

### 1: Anchor Sketch

Start the Anchor Sketch (name the sketch `Anchor`) **directly on the user-selected target plane**, whether the selection is a `ConstructionPlane` or a `PlanarFace`; don't re-derive or offset it (`[PB-USE-SELECTED-PLANE]` — re-deriving collapses the gear onto XY). Mark the center point by projecting the user-specified center point onto the sketch.

Create a line through the projected center point. **Seed its two endpoints at exactly ±0.5 cm from the projected center** along the sketch-local X (so the seeded length is 10 mm). Apply **BOTH** `addCoincident(projectedCenter, anchorLine)` (the "intersection" — pins the center onto the line) **and** `addMidPoint(projectedCenter, anchorLine)` (center bisects the line). Use both, not midpoint alone. Add an **aligned distance dimension WITHOUT assigning `.parameter.value`** — the dimension simply locks the length at the seeded 10 mm (the value is arbitrary; it's only a reference line). **Then pin its direction so the sketch ends FULLY CONSTRAINED:** add `addHorizontal(anchorLine)` (sketch-local, per `[PB-REFLINE-DIRECTION]` — works on any tilted target plane; a world-axis lock would mis-orient). The anchor line's absolute direction is arbitrary (nothing downstream depends on it — §2 derives all directions *relative* to the projected anchor line), but it must not be a free degree of freedom: with midpoint + length + Horizontal the line has zero DOF. **Stash the projected-center SketchPoint** (e.g. on `self`) so §2 re-projects *this* anchor-sketch point — not the raw user-selected center — into the Gear Profiles sketch. This line is the Anchor Line. After all constraints, the Anchor Sketch must report `isFullyConstrained` (see the full-constraint gate in Sketch Discipline).

### 2: Gear Profiles

Using setByAngle, create a plane that includes the Anchor Line, set at 90° (by default it would lie flush to the anchor line's plane, but we want it perpendicular). **Build it off the original `targetPlane`** as the reference — don't re-derive/offset it (`[PB-USE-SELECTED-PLANE]`) — this is the other place the target-plane orientation reaches the bodies; substituting a different plane here also collapses the gear onto XY. Name the plane `Gear Profiles Plane`. Create a sketch on this plane, named `Gear Profiles`.

**Every line drawn in this §2 sketch is a construction line (`isConstruction = True`)** — the lattice lines, the toe lines M->N / O->P, and the short reference/connector lines (M->C, N->A, O->D, P->B, A->G, B->I, C->K/K′, D->L/L′) alike. The solid features later consume only the per-gear Profile sketches (see Create the Gear Bodies), never a §2 curve directly.

In the sketch, project **the Anchor Sketch's center SketchPoint** (the one you stashed in §1) — NOT the raw user-selected center point. Both happen to be coincident, but projecting the anchor-sketch point keeps the chain within the Design component and faithful to the anchor geometry; projecting the raw external point is a cross-component reference and can resolve inconsistently.

From the projected center point, draw a construction line **perpendicular to the (projected) anchor line, in the gear-profiles sketch's own 2-D frame** (apply a Perpendicular constraint to the anchor line). Its far end is the **Apex**, placed in sketch-local coordinates at `c + perp·(Driving Gear Pitch Diameter)` where `c` is the projected center and `perp` is the in-plane unit vector perpendicular to the projected anchor line (`(-d.y, d.x)` for anchor-line direction `d`). The apex **position** is sketch-local — do **NOT** compute it from a world-coordinate round-trip; that is what caused the XY-collapse. The **sign of `perp`** (which side the gear grows) is chosen by the target-plane normal as a one-bit direction (see Sketch Discipline "Grow side" — toward the normal), NOT by the sketch's local +Y. Read that normal as **`targetPlane.geometry.normal`** for BOTH selection kinds — a `BRepFace`'s `geometry` and a `ConstructionPlane`'s `geometry` are each a `core.Plane` carrying `.normal`. Do NOT add a length constraint on this line.

Create a construction line from the apex representing the Driving Gear Shaft Axis, pointing from the apex back toward the anchor line (seed its far end at `c - perp·(some length)`, i.e. the `-perp` direction). It must run **parallel to the center→apex construction line** — apply `addParallel(drivingShaftAxis, centerToApex)`. **Do NOT use `addVertical`**: `addVertical` forces the line to the sketch's world-vertical, which is wrong on a tilted target plane (the gear-profiles sketch is not world-aligned) and over-constrains/mis-orients the figure. The shaft axis must be parallel to the *in-plane* apex direction (`perp`), expressed via `addParallel` to the center→apex line — never an absolute Horizontal/Vertical constraint. Beginning of this line uses a coincidence constraint with the apex. The end of this line shall be called point B. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

Create a construction line from the apex representing the Pinion Gear Shaft Axis. The pinion shaft is the Driving Gear Shaft Axis direction rotated about the apex by the Shaft Angle. Rotating by the Shaft Angle has two senses (one to each side of the driving shaft), and they place point A on opposite sides — choosing wrong mirrors the whole gear onto the wrong side of the target plane. **Select the sense (seed) this way: form both candidate point-A positions (the driving-shaft direction rotated about the apex by +Shaft Angle and by −Shaft Angle) and keep the candidate whose endpoint has the greater X coordinate in the Gear Profiles sketch.** Compare the two candidates' X and take the larger — do **not** rotate one fixed sense and only flip it when its X comes out negative; when *both* candidates have a positive X that shortcut keeps the wrong one (this is exactly the side-flip to avoid). The +X-most endpoint is the side away from the anchor sketch's leading direction and is consistent across all target planes. Call the chosen seed direction `pinionDir` (the unit Apex→A direction).

Apply an angular dimension between this line and the Driving Gear Shaft Axis equal to Shaft Angle (the traditional "angle between the two shaft axes" — 90° gives the classic perpendicular bevel pair). **Place its text point inside the Σ wedge so it measures Σ and not its supplement 180−Σ** (`[PB-ANGULAR-DIM]`) — e.g. on the interior bisector of the two shaft directions, `apex + normalize(pinionDir + drivingDir) · (PPD/4)`, where `drivingDir` is the unit Apex→B direction. (The angular dimension only fixes the angle *magnitude*; it does not by itself pin which side the pinion lies on, and the text point does not prevent a frame flip — the pinion side is held by the seed above together with the Apex 2 closure below. The real side-flip hazard lives in the perpendicular drops to Apex 2 — see the ⚠️ on the B→Apex 2 drop.) Beginning of this line should use coincidence constraint with apex. The end of this line shall be called point A. Do **not** dimension the line's length — it is determined by the closing constraint at Apex 2 below.

From A, create a construction line perpendicular to the Pinion Gear Shaft Axis, drawn toward the side where Apex 2 will lie. ⚠️ **Apex 2 sits in the interior wedge *between* the two shaft axes — so this drop must point toward the OTHER (Driving) shaft axis / point B, NOT "toward the anchor line".** Pick the perpendicular sense by the sign of its dot product with the A→B direction (toward the driving shaft), not against a generic "toward the anchor" reference. Apply a perpendicular constraint against the Pinion Gear Shaft Axis. Apply a dimensional constraint with length = Pinion Gear Pitch Diameter / 2 (this equals the pinion's pitch radius at the heel, which is the perpendicular distance from Apex 2 to the Pinion Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with A. **Naming convention used throughout this spec: this perpendicular drop line (A → its far end, which becomes Apex 2) is what "A->Apex2" / "line A->Apex2" always refers to — it is NOT the Apex->A shaft axis. The two share point A but are different lines (one is the PPD/2 perpendicular drop, the other the shaft axis). Whenever a later step says to pin something to or dimension against "A->Apex2", it means this drop line. The same holds for "B->Apex2" (the DPD/2 drop) vs the Apex->B shaft axis.**

From B, create a construction line perpendicular to the Driving Gear Shaft Axis, drawn toward the side where Apex 2 will lie. ⚠️ **This drop must point toward the OTHER (Pinion) shaft axis / point A** — pick the perpendicular sense by the sign of its dot product with the B→A direction. **Do NOT choose this sense by a "toward the anchor line" reference (i.e. the −perp / center→apex grow direction): the Driving Gear Shaft Axis is itself parallel to that grow direction, so the perpendicular's dot with it is ≈ 0 — a degenerate test that silently selects an arbitrary (usually wrong) side.** This is the critical failure: if this B→Apex 2 drop seeds Apex 2 on the wrong side of the driving shaft while the Pinion's A→Apex 2 drop seeds it on the correct side, the coincidence that closes the two drops at Apex 2 (below) makes the solver **flip the entire frame to the mirror solution** — point A jumps to the opposite side, and the pinion dedendum C collapses onto the driving dedendum D, inverting the pinion (the toe ends up *outside* the heel, the revolved frustum is degenerate, and the conical end-cut finds no cone face at the toe midpoint → `face dist = inf`). Both Apex 2 drops must aim at the *same* interior-wedge point. Apply a perpendicular constraint against the Driving Gear Shaft Axis. Apply a dimensional constraint with length = Driving Gear Pitch Diameter / 2 (the driving pitch radius at the heel, perpendicular distance from Apex 2 to the Driving Gear Shaft Axis for any Shaft Angle). Beginning of this line should use coincidence constraint with B.

Constrain the end points of the two perpendicular lines from the previous two paragraphs with a coincident constraint. Let this point be called Apex 2. (At Shaft Angle = 90° the four points Apex, A, Apex 2, B form a rectangle. For other shaft angles the figure is a non-rectangular parallelogram-like quadrilateral; the lengths of Apex→A and Apex→B adjust so the perpendicular drops of length PPD/2 and DPD/2 coincide at Apex 2.)

Note that this quadrilateral deliberately lies well above the anchor line. The Apex's upward offset from the anchor line (= Driving Gear Pitch Diameter) is chosen to keep the whole figure above the anchor line for Shaft Angle in the supported range 30°–150°.

**Seed the along-shaft lengths (Apex→A, Apex→B) with the closed-form cone geometry** so the solver converges on the right branch for any Shaft Angle Σ (these are seed coordinates only — the lengths stay undimensioned, fixed by the Apex2 closing constraint): `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`, `γ_g = Σ − γ_p`, cone distance `R = (PPD/2) / sin γ_p`; seed `|Apex→A| = R · cos γ_p` and `|Apex→B| = R · cos γ_g`. (γ_p, γ_g are also reused in §3 for the virtual tooth radii.) Seeding A/B merely by a pitch diameter is wrong for Σ≠90° and can send the solver to the wrong branch.

Draw a construction line from Apex to Apex 2. This line shall be called the Pitch Line. Each end of the Pitch Line should be constrained to the respective points using coincidence constraint.

From the Apex 2, create a construction line in either side whose length is constrained Module * 1.25, and are perpendicular to the Pitch Line. Constrain them against the Pitch Line with the Perpendicular constraint. Let the line drawn towards the anchor line be Driving Gear Dedendum, whose end point shall be point D. The one drawn away from the anchor line be Pinion Gear Dedendum; whose end point shall be point C.

Draw two construction lines, from the Apex to the point D and point C, respectively. Apply coincidence constraints on beginning and end of these lines. These lines shall be the Root Axis for driving and pinion gear, respectively.

From point A, create construction line collinear with the line from Apex to point A, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a collinear constraint, and the end of Apex->A and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point E.

Draw a construction line from point C to point E. constrain each end to respective points from pre-existing lines. The lines A->E and C->E should be constrained with perpendicular constraint.

From point B, create a construction line collinear with Apex->B, extending for length equals to module (but do NOT add dimensional constraint). The line should receive a collinear constraint, and the end of Apex->B and the beginning of the new line should be constrained via coincidence constraint. The end of this new line shall be point F.

Draw a construction line from point D to point F. constrain each end to respective points from pre-existing lines. The lines B->F and D->F should be constrained with perpendicular constraint.

Draw a construction line from point E collinear to line A->E, with length equal to module (but do NOT add dimensional constraint). Constrain point E and the beginning of this line. Let the end be known as point G.

From point C, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point C and the beginning of this line. Let the end of the new line be point H. Line C->H should be collinear with line Apex2->C.

Connect point G and H with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line E->G and H->G with a perpendicular constraint.


Draw a construction line from point F collinear to line B->F, with length equal to module (but do NOT add dimensional constraint). Constrain point F and the beginning of this line. Let the end be known as point I.

From point D, draw a line with length equal to module (but do NOT add dimensional constraint). Constrain point D and the beginning of this line. Let the end of the new line be point J. Line D->J should be collinear with line Apex2->D.

Connect point I and J with a line. Constrain end points of line accordingly with coincidence constraints. Constrain line F->I and J->I with a perpendicular constraint.

Create an **offset dimension between the B->Apex2 perpendicular drop line (the DPD/2 drop per the naming convention above — NOT the Apex->B shaft axis) and J->I**. J->I is **already parallel** to the drop by construction (J->I ⊥ F->I, which runs along the driving shaft), so add **no** extra parallel constraint (`[PB-OFFSET-DIM]`). Set the value equal to Driving Gear Base Height _if_ specified (non-0); otherwise `module * Driving Gear Teeth Number / 8`.

Create an **offset dimension between the A->Apex2 perpendicular drop line (the PPD/2 drop per the naming convention — not the Apex->A shaft axis) and G->H** — already parallel by construction (G->H ⊥ E->G), so as with the driving side add no parallel constraint (`[PB-OFFSET-DIM]`). The value should be equal to Pinion Gear Base Height _if_ specified (non-0); otherwise the **RESOLVED** Driving Gear Base Height `* (Pinion Gear Teeth Number / Driving Gear Teeth Number)`. "Resolved" means the value the driving offset above actually used — i.e. after the driving side's own fallback (`module * Driving Gear Teeth Number / 8` when the driving input was 0) — NOT the raw driving input.

Draw a line from A to G. Constrain endpoints appropriately.

Constrain Point I with center point.


Draw a construction line away from Apex, starting from point G, extending along Apex->A, and call its end point K. Then **pin K with two point-on-line coincident constraints** — `addCoincident(K, line Apex->A)` and `addCoincident(K, the Pinion Dedendum line Apex2->C extended)` — rather than `addCollinear` on the connecting lines. By the time K is added, G and C are already fixed, so an `addCollinear` here over-constrains the sketch and Fusion errors; the two point-on-line coincidents locate K exactly (intersection of the two lines) without over-constraining. Draw a construction line from point C to K for reference.

**Tooth-center point K′ (Tooth Spacing offset).** The §3 spur tooth is centered not at K but at a tooth-center point **K′**, obtained by shifting K outward along the dedendum line by **Tooth Spacing**, *away from the lower corner C*. **When Tooth Spacing is 0 (the default), do NOT build anything here — set K′ ≡ K and reuse the C->K reference line** (a zero-length dimensioned line would be degenerate, and one segment gets ONE line — `[BEVEL-F-LINE-ONCE]`). When Tooth Spacing > 0: draw a construction line starting at K with its far end seeded on the *far side of K from C* along the dedendum direction; pin its far end **the same way K is pinned to its line** — `addCoincident(start, K)` and `addCoincident(K′, the Pinion Dedendum line Apex2->C extended)` to keep K′ on the dedendum line — then add a **length dimension on this line = Tooth Spacing** (do **not** use `addCollinear`, for the same over-constraint reason as K). The far end is K′. Build it **here, inside the Gear Profiles sketch, before that sketch's end-of-step full-constraint gate**, so the gate covers it. Finally draw the **tooth-center reference line C->K′** (from the lower corner C to K′) for §3 to use in place of C->K. Only the tooth's center moves; the virtual tooth number and drawn tooth size are unchanged (see §3).

At this point all of A, B, C, D, H, J exist **and are solved**, so resolve the **Maximum Face Width** (see the Parameters section) from their solved `.geometry` (NOT the seed coordinates — see that section) and apply it before using Face Width below: cap the auto default to it, and reject a user value that exceeds it. Skipping this — or computing it from seeds — makes the M->N / O->P line push N/P across the shaft axis for asymmetric tooth counts (either gear can be the smaller, binding side), which fails the gear-body revolve with `ASM_WIRE_X_AXIS`.

Create line M->N. **Seed it near its solved position** (`[PB-SEED-NEAR]`): seed M at roughly the **midpoint of Apex->C**, and seed N by sliding from that M-seed **along the C->H direction far enough to roughly reach line A->Apex2** (e.g. by the distance from the M-seed to A). Do NOT seed M/N just `Face Width` away from C/H — that starts N near H, far from its constraint target (line A->Apex2), and the solve fails to converge. Then apply **exactly these constraints** — all four are required, and the two coincident pins are what make the Maximum-Face-Width guarantee real (N reaches A exactly at the cap):
- `addCoincident(M, Pinion Root Axis)` — M lies on the Apex->C root axis;
- `addCoincident(N, line A->Apex2)` — N lies on the A->Apex2 **perpendicular DROP** (per the naming convention — NOT the Apex->A shaft axis). This is the pin; do not merely place N numerically. Load-bearing: pinning N to the shaft axis puts N *on the axis of revolution*, and the later conical split fails with `ASM_API_FAILED` for asymmetric tooth counts even though the symmetric 45° case happens to survive;
- `addParallel(M->N, C->H)` — the toe line is parallel to C->H;
- `addOffsetDimension(C->H, M->N, textPoint).parameter.value = Face Width` — the parallel offset between C->H and M->N equals Face Width. Place the `textPoint` in the gap between C->H and M->N on the Apex side (e.g. the midpoint of the M-seed and point C, `(M_seed + C)/2`) so the dimension reads cleanly (`[PB-OFFSET-DIM]`). The toe's side relative to the heel (`toe→Apex < heel→Apex`) follows from the §2 frame being built correctly — in particular from the Apex 2 drops aiming at the interior wedge (see the ⚠️ above); it is **not** controlled by this text point.

(Because N is pinned to A->Apex2, the Maximum Face Width above is exactly the value at which N reaches A; the capped Face Width keeps N between Apex2 and A.)

Let the beginning of this new line be point M, the end be point N. Draw a line from M to C. Draw a line from N to A.

Draw a construction line away from Apex, starting from point I, extending along Apex->B, and call its end point L. **Pin L the same way as K** — `addCoincident(L, line Apex->B)` and `addCoincident(L, the Driving Dedendum line Apex2->D extended)`; do not use `addCollinear`. Draw a construction line from point D to L for reference.

**Tooth-center point L′ (Tooth Spacing offset).** Build the driving-side tooth center **L′** exactly as K′ on the pinion side, substituting L for K, D for C, and the Driving Dedendum line Apex2->D for the pinion's; the reference line for §3 is **D->L′**. Same single Tooth Spacing value, same full-constraint gate, same reuse-the-existing-line rule at 0.

Create line O->P, the mirror of M->N on the driving side. Seed it the same way (O near the midpoint of Apex->D, P slid along D->J toward line B->Apex2), then apply the same four constraints:
- `addCoincident(O, Driving Root Axis)` — O on the Apex->D root axis;
- `addCoincident(P, line B->Apex2)` — P on the B->Apex2 perpendicular DROP, **NOT the Apex->B shaft axis** (same trap as N above);
- `addParallel(O->P, D->J)`;
- `addOffsetDimension(D->J, O->P, textPoint).parameter.value = Face Width` — as for the pinion, place the `textPoint` in the gap on the Apex side of D->J (e.g. `(O_seed + D)/2`) so it reads cleanly (`[PB-OFFSET-DIM]`).

Let the beginning of this new line be point O, the end be point P. Draw a line from O to D. Draw a line from P to B. Draw line from B to I.

### 3: Gear Tooth Profiles

**API note for this whole section:** pass the relevant **sketch line directly** to `setByAngle` and
`setByDistanceOnPath`; never wrap it in `Path.create` first (`[PB-CONSTRUCTION-PLANES]`).

**Throughout this section the tooth center is the §2 tooth-center point K′ (pinion) / L′ (driving) and the center reference line is C->K′ / D->L′ — which equal K / L and C->K / D->L exactly when Tooth Spacing is 0.** The virtual tooth number below is computed from the pitch diameter and is **independent of Tooth Spacing**; the spacing offset moves only the center, not the tooth size.

Do all four steps below **once per gear** — pinion first, then driving — with this gear's parameters: pinion uses tooth-center **K′**, reference line **C->K′**, and pitch-cone half-angle **γ_p** (from §2: `tan γ_p = sin Σ · PPD / (DPD + PPD · cos Σ)`); driving uses **L′**, **D->L′**, and **γ_g = Σ − γ_p**.

1. Compute this gear's virtual (back-cone / Tredgold) tooth number from the closed form, **not** by measuring Apex2->K′/L′: virtual pitch radius = `(this gear's Pitch Diameter / 2) / cos(γ)`. Virtual tooth number = `floor(2 · virtualPitchRadius / Module)`, as an int. **Units — pin the cm→mm conversion:** the stashed pitch diameters are internal **cm** while Module is the raw **mm** value, so compute `virtualPitchRadius_mm = (pitchDia_cm · 10 / 2) / cos(γ)` — the `· 10` converts cm to mm — and then `virtualTeeth = floor(2 · virtualPitchRadius_mm / Module)`. Skipping the ×10 makes the virtual tooth count ~10× off (see the Units note).

2. Create a new plane that includes the tooth-center reference line, named `{gearLabel} Plane`. Use setByAngle to make this plane perpendicular to the Gear Profiles sketch plane (`plane_by_angle` from `.solids`).

3. Using the new plane and the tooth-center point as the center point, create a spur gear tooth profile with module and the virtual tooth number from step 1, in a sketch named `{gearLabel} Tooth`. Draw it **already rotated 180°** by passing `angle=math.radians(180)` to the spur tooth generator's `draw(anchorPoint, angle=…)` (see Dependencies) — the generator rotates the whole tooth by that angle; do not draw it flat and rotate the sketch afterward. **After `draw()` returns do NOT hard-gate this sketch: log if `not toothSketch.isFullyConstrained`, never raise** — the tooth-profile sketches are exempt from the full-constraint gate (see Sketch Discipline: the embedded low-tooth-count tooth is legitimately under-constrained, and the profile is consumed immediately by the loft).

4. Create a construction axis (named `{gearLabel} Tooth Axis`) through the tooth-center point, normal to the plane the tooth profile was drawn on, via `setByTwoPlanes` (`[PB-CONSTRUCTION-AXES]`; `setByPerpendicularAtPoint` would need a `BRepFace`). The two planes are: the **Gear Profiles plane** and a **helper plane built `setByDistanceOnPath(<tooth-center reference line>, 1.0)`** (perpendicular to that line at its far end, the tooth-center point); their intersection is the line through the tooth center normal to the tooth plane. (Creating this axis in the never-activated Design component is proven to work — `constructionAxes.add` via `setByTwoPlanes` does not hit `[PB-CONSTRUCTION-NEEDS-ACTIVE]` here; keep the axis.)

### 3a: Spiral tooth body (ψ > 0)

This is the ψ > 0 branch of the tooth-body hook `_transformToothBody` (Method contract) — it **replaces** the straight tooth's two conical trims with a curved tooth. When **ψ = 0 the hook returns immediately** with the framework's `cut_conical_ends` (the straight tooth, trimmed to a flush band — byte-for-byte the prior behavior); everything below runs **only when ψ > 0**. It is invoked once per gear (pinion then driving), inside `_createGearBody`, on the freshly lofted uncut apex→heel `toothBody`, before pattern/combine/bore. The arc math it realizes is derived in `spiral-tooth-trace.md`; this section states **how** that construction is realized as Fusion sketches/features and the order it runs in. Use the existing `{gear}` sketch-naming (`gearLabel` is `Pinion` or `Driving`).

**Caller hand-off — the four toe/heel world points `_createGearBody` builds and passes to the hook (PIN EXACTLY; mislabeling them silently inverts the spiral — this is the single biggest spiral-regen hazard).** The §2 lattice gives each gear a **toe edge** (the inner face-width edge, nearer the apex) and a **heel edge** (the outer edge, at the back cone). Per gear they are exactly:

| gear | toe edge (inner) | heel edge (outer) | `toeConeWorld` | `heelConeWorld` |
|---|---|---|---|---|
| Pinion | **M→N** | **C→H** | **M** | **C** |
| Driving | **O→P** | **D→J** | **O** | **D** |

From those §2 sketch points' **world** geometry, `_createGearBody` computes (and passes positionally into `_transformToothBody` in the order `toeMid, heelMid, toeConeWorld, heelConeWorld`):
- `toeMid` = world **midpoint of the TOE edge** — ½(M+N) pinion / ½(O+P) driving.
- `heelMid` = world **midpoint of the HEEL edge** — ½(C+H) pinion / ½(D+J) driving.
- `toeConeWorld` = the **toe edge's inner endpoint** — **M** (pinion) / **O** (driving). This point lies on the **root cone element** (the Pinion Root Axis Apex→C / Driving Root Axis Apex→D) at the inner/toe end — M is pinned onto Apex→C in §2 (`addCoincident(M, Pinion Root Axis)`), O onto Apex→D.
- `heelConeWorld` = the **dedendum corner** (the heel edge's first endpoint) — **C** (pinion) / **D** (driving). C/D is the **outer** end of that **same root cone element**, so `coneVec = normalize(heelConeWorld − apex)` runs along Apex→C / Apex→D pointing outward and `distAlong(heelConeWorld) > distAlong(toeConeWorld)`.

⚠️ **Two scrambles to avoid** (a fresh regen has made both):
- Do **NOT** pass the two endpoints of a *single* edge as `toeMid`/`heelMid` (e.g. M as `toeMid` and N as `heelMid`). M and N both sit at the **toe**, so `span = distAlong(heelMid) − distAlong(toeMid)` collapses to ≈0 or negative and the spiral inverts. `toeMid` is the midpoint of the **toe** edge; `heelMid` is the midpoint of the **heel** edge — two different edges.
- `heelConeWorld` is the **dedendum corner C/D** (on the root axis Apex→C / Apex→D), **never H/J**. H/J lie on the `Apex2→C` / `Apex2→D` dedendum line (one Module beyond C/D), **off** the root cone element — using them skews `coneVec` away from Apex→C / Apex→D.

**A. Gate & frame.** Build a world frame from the geometry already constructed for this gear:

- `axisDir` = the **shaft axis** direction, from the two **world** endpoints of `shaftAxisEdge` (the in-sketch profile edge A→G / B→I), normalized.
- `coneVec` = the **dedendum (root) cone element** Apex→D (driving) / Apex→C (pinion), realized as `normalize(heelConeWorld − apex)` where `apex` = `apexWorld`. (`heelConeWorld` is the heel end of that dedendum element; `toeConeWorld` is its toe end.)
- `v` = `axisDir × coneVec`, normalized — the **circumferential** direction (the sideways sense the tooth is displaced from the radial element).
- `tpNormal` = `coneVec × v`, normalized — the **tangent-plane normal** (the direction the flat trace is projected onto the cone, step D).
- `distAlong(p)` = `(p − apex) · coneVec` — the **cone distance** of a point (its distance from the apex measured along the cone element).

⚠️ **The heel MUST be the OUTER end (farther from the apex) so `coneVec` points outward and `span > 0`.** Before building `coneVec`, check the passed midpoints and **fix swapped toe/heel**: if `apex.distanceTo(heelMid) < apex.distanceTo(toeMid)`, swap `toeMid ↔ heelMid` **and** `toeConeWorld ↔ heelConeWorld`, then build `coneVec = apex → heelConeWorld`. A negative `span` (toe farther than heel) **silently inverts the entire spiral frame** — it flips the cutter-arc direction, the slice direction (the first cut misses; see step E), and the per-segment twist — and the gear comes out completely wrong with no error. (The inversion can also originate upstream in §2/§3 mislabeling the toe vs heel edges; this guard catches it at the frame.)

From `toeMid`/`heelMid` (the toe/heel root-edge midpoints, **after** the swap guard above) get `R_toe = distAlong(toeMid)`, `R_heel = distAlong(heelMid)`, `R_mean = ½(R_toe + R_heel)`, and `span = R_heel − R_toe` (the face width, now **positive**). These are the only quantities the rest of the build needs.

**B. Cutter-arc geometry.** Work in the tangent-plane 2-D frame with origin at the apex, **x = coneVec** (so a point's x is its cone distance) and **y = v** (circumferential). The cutter radius is `r_c = Cutter Radius` if non-zero, **else `R_mean`** (the auto default). The hand sign is `handSign = +1` for `Right` else `−1`, then **negated for the pinion** (the pair meshes with opposite hands). The cutter-circle centre is

```
Cx = R_mean − r_c · sin ψ
Cy = handSign · r_c · cos ψ
```

⚠️ **Gotcha — the hand sign goes on the `cos`/`Cy` term, NOT the `sin`/`Cx` term** (this was a real bug). Opposite hand mirrors the cutter centre **across the cone element (y = 0)**, which flips `Cy`. Putting `handSign` on `Cx` mirrors about `x = R_mean` instead — a *different* curve that gives the two gears **unequal twist**; for equal teeth the driving and pinion traces must come out as exact mirror images.

The trace's toe/heel arc endpoints are circle∩circle intersections taken a hair **past** the face so the kept arc reaches cleanly past the end-trims: `toe2d = circle_intersect_nearest(R_lo, …)` and `heel2d = circle_intersect_nearest(R_hi, …)` (the framework helper from `.solids`) with `R_lo = R_toe − 0.06·span` and `R_hi = R_heel + 0.06·span`. `circle_intersect_nearest` intersects the apex circle of radius R with the cutter circle (centre `(Cx,Cy)`, radius `r_c`) and keeps the solution nearest `(R_mean, 0)` — the branch the mean point sits on. (See `spiral-tooth-trace.md` §6 for why the near branch, and §5 for the centre derivation.)

**C. 2-D trace sketch (the genuine cutter arc).** Build the tangent plane with the framework's `plane_by_angle`: first draw a **cone-element construction line** Apex→(Apex + R_heel·coneVec) in a sketch on the **axial / Gear Profiles plane** (name it `{gear} Cone Element`), then make the tangent plane = that axial plane rotated **90°** about the cone-element line (`plane_by_angle(comp, coneElementLine, axialPlane, 90)`; name it `{gear} Trace Plane`). Add a sketch on it named **`{gear} 2D Tooth Trace`**. In it draw, with `tanW(px,py) = combine_point(apex, px, coneVec, py, v)` (framework) mapping 2-D coords to world:

- the **cutter circle** — centre at `tanW(Cx, Cy)`, radius `r_c` — `isConstruction`, with its centre pinned via `centerSketchPoint.isFixed = True` (`[PB-CIRCLE-CENTER]`) and a diameter dimension = `2·r_c`;
- the **trace arc** — a 3-point arc through `tanW(toe2d)`, `tanW(R_mean, 0)` (the mean point on the cone element), `tanW(heel2d)`, with its **centre coincident to the cutter circle's centre** and a **radius dimension = `r_c`**, so it is the genuine cutter circle and not a look-alike spline. ⚠️ Text points per `[PB-RADIAL-DIM]` (off-centre, on/near the curve): use the mean point `tanW(R_mean, 0)` for the trace arc's radius dimension, and a point on the cutter circle such as `tanW(Cx + r_c, Cy)` for its diameter dimension.

**Coordinates:** the world `Point3D`s from `tanW(...)` (and the raw `apex`/cone-end points of the `{gear} Cone Element` line) are passed **directly** into the sketch calls (`addByTwoPoints`, `addByCenterRadius`, `addByThreePoints`), where they are consumed as **sketch-space** input — **no `modelToSketchSpace` conversion is applied**. This is deliberate and harmless because the trace sketch is **construction/reference only: no downstream feature ever consumes it** — the twist is computed analytically from the 2-D endpoints in step G, and the sketch exists only so the genuine cutter arc is inspectable before cleanup hides it.

This sketch is **deliberately left with free DOF** — the arc's endpoints are pinned by the 3-point construction, not by endpoint dimensions (dimensioning them over-constrains the solve against the cone-element plane). It is therefore **exempt from the full-constraint gate**, as already declared in Sketch Discipline ("The spiral build's auxiliary sketches are EXEMPT") — do not gate it.

**D. (No 3-D projection.)** The 2-D cutter-arc sketch from step C is the only trace geometry needed — the spiral twist is computed **analytically** from it in step G, so there is **no `projectToSurface`, no root-cone-face search, and no 3-D trace sketch.** (Earlier versions projected the 2-D arc onto the root cone along `tpNormal` and measured the trace azimuth there. That projection is *fragile*: for unequal-ratio pairs the arc wraps around the cone and `projectToSurface` returns it as **multiple disjoint fragments**, so the measured azimuth collapses to a fraction of the true sweep — the pinion comes out grossly under-twisted and the pair interferes. The analytic crown-gear law in step G is exact, deterministic, and cannot wrap.)

**E. Slice the straight tooth.** Split the uncut apex→heel `toothBody` into cross-section slabs by planes **perpendicular to the cone element**, spanning a touch past toe and heel, via a **fixed** slice scheme (≈8 planes — the count is not user-configurable). The first cut plane is the **parent transverse tooth plane** (`parentToothPlane`, the virtual-spur tooth-profile plane `{label} Plane` from §3, passed into the hook) offset toward the apex by `span/6`; the offset **sign is chosen per gear** so it moves toward the apex (the parent plane's normal points opposite ways for the two gears — pick `sign` so `sign·normal` points apex-ward, i.e. test `(apex − planeOrigin)·normal`). Then a sequence of ~8 planes stepped further toward the apex in `span/6` increments (`sign·(k+1)·span/6` for k = 0…7, k = 0 being the first cut plane). Split the body with the framework's `slice_body_by_offset_planes(designComponent, toothBody, parentToothPlane, offsets)` where `offsets = [sign·(k+1)·span/6 for k in 0…7]` — it splits piece-by-piece and keeps a piece whole when a plane misses it. ⚠️ **The slice MUST actually split the tooth.** After the cut loop, if the body is still in **one piece** (no plane cut it), the offset sign was wrong or `parentToothPlane` sits outside the tooth's span — **retry the whole cut once with the opposite sign**. If it is *still* one piece, **`raise` a clear self-diagnosing error** naming the gear, the final piece count, `span`, and the sign tried. Do **NOT** return an unsliced (single-piece) result: step F then drops that one piece as the apex scrap, leaving `segments` **empty**, and the crown later crashes with `ValueError: max() iterable argument is empty` far from the cause. The result is the set of cross-section segments.

**F. Order & drop scrap.** Sort the segments by `distAlong` of their centroid (`physicalProperties.centerOfMass`). The first (apex-most) is the long **apex-side scrap** below the toe — **remove it**; keep the rest as the working `segments`. (Drop the scrap by re-slicing the list, *then* delete it — `segments = segments[1:]` before `removeFeatures.add(scrap)`.) After dropping the scrap, **`segments` must be non-empty** (≥1 cross-section); if it is empty the slice failed in step E — `raise` a clear error rather than proceeding into the twist (G) and crown (H), which assume ≥1 segment.

**G. Twist (the spiral).** Rotate each segment about the **shaft axis** (`axisDir` through `apex`) so the tooth follows the trace, **centred on R_mean so the mid-face section stays unrotated** — that section then meshes exactly like the straight tooth (critical; the pinion's zero mesh nudge depends on it). The total toe→heel shaft-axis twist comes from the **conjugate crown-gear generation law** (the standard Gleason/Litvin model — see `spiral-tooth-trace.md` and the NASA references): a spiral bevel is generated by an imaginary flat *crown gear*, and the work gear’s shaft rotation relates to the developed crown-plane azimuth by the **roll ratio `1/sin γ`** (the generating crown gear has `N/sin γ` teeth; γ = this gear’s **pitch cone angle**). Compute it **analytically — no projection, no curve sampling:**

```
phi_crown = atan2(heel2d[1], heel2d[0]) - atan2(toe2d[1], toe2d[0])   # developed azimuth of the cutter arc at the apex
total     = abs(phi_crown) / math.sin(gamma)                          # shaft-axis twist magnitude
```

`phi_crown` is the angle the cutter arc’s **toe and heel endpoints subtend at the apex** in the flat 2-D crown frame (apex at the origin, x = cone distance along `coneVec`, y = circumferential along `v` — exactly the `toe2d`/`heel2d` pairs from step B). `gamma` is this gear’s **pitch cone angle**: `self._gamma_p` (Pinion) / `self._gamma_g` (Driving), already computed in §2. `handSign` sets the direction; `total` is the magnitude. ⚠️ **Use the PITCH cone angle γ from §2 — NOT `acos(coneVec·axisDir)`** (that is the *root/dedendum* cone angle, e.g. ~14° vs the pitch ~29° for a 17-tooth pinion, and yields a twist ~1.6× too large). ⚠️ The two members of a meshing pair **legitimately get different twists**: same cutter, same spiral angle ψ, but γ differs, so `1/sin γ` differs (≈2.08× for a 17-tooth pinion vs ≈1.14× for a 31-tooth gear — a ratio ~1.83). This is *why* equal-teeth pairs (31/31, equal γ) always meshed while ratio pairs failed under any method that gets `1/sin γ` wrong. **Do NOT** measure the twist off a projected 3-D cone trace (the old approach): `projectToSurface` wraps the arc around the cone for ratio pairs and the measurement collapses. The analytic law here is exact and deterministic. Each segment's rotation angle is a **linear share** keyed to the **cone distance of its HEEL FACE** (the segment's farthest-along-the-element face — the exact section the later loft samples). **Define a slab's heel face precisely: the face whose centroid has the GREATEST `distAlong(face.centroid)`, searched across ALL of the slab's faces with NO surface-type filter** (its toe/apex-side face is the LEAST-centroid one). ⚠️ Do **NOT** restrict this search to `PlaneSurfaceType` (or any surface type) — a sliced slab is bounded by a mix of the two planar cut faces and ruled side faces, and a type filter can pick the wrong face or miss the cut face, which makes the step-I loft fail with `ASM_NOT_ALL_SECTIONS_MEET / LOFT_NO_TOOLBODY`. Use this **same all-faces-by-centroid** rule (max → heel, min → toe) everywhere a slab end face is needed: the twist key here (G), the crown base (H), and the loft sections (I). The rotation:

```
ang = −handSign · total · (R_mean − R_heelFace(seg)) / span
```

⚠️ **Gotcha — key the twist on the segment's HEEL-FACE cone distance, NOT its centroid.** The loft (step I) samples each segment's heel face, so that face is what must land at the right azimuth. Centroid-keying leaves the loft's mid-face section rotated by half a segment → mid-face overlap. Apply the rotation with a free-move by a `Matrix3D.setToRotation(ang, axisDir, apex)`.

**H. Lengthwise crown (relief).** Crown the tooth by scaling each segment **except the outermost (heel) one** down by a **monotonic** factor — full at the heel, growing smoothly toward the toe — **about a sketch point on the ROOT edge of its heel face** (gotcha 3 — NOT the heel-face centroid). For each segment compute its **heel-distance fraction** `u = (R_heel − R_heelFace) / span`, where `R_heelFace` = `distAlong` of that segment's heel face — **found by the step-G all-faces-by-centroid rule but RECOMPUTED here, AFTER the step-G twist has moved the slabs** (do not reuse pre-twist values) — and `R_heel`/`span` are from step A; `u` runs `0` at the held-full heel to `1` at the toe. **"Outermost (heel) segment" = the one with the GREATEST post-twist heel-face `distAlong`** — sort the segments by their recomputed heel-face `distAlong` and skip the last. Then:

```
factor = 1 − _CROWN_PER_RAD · (|total| / 2) · u
```

`total` is the full toe→heel twist from step G; `|total|/2` = the per-end peak twist magnitude (`max|ang|`), so the maximum relief — now at the **toe** — keeps the same magnitude the old per-end peak had, just relocated. This makes relief **grow monotonically from the (full) heel to the toe**, so slab heights stay **strictly ordered heel→toe** and the natural cone taper is never reversed. If a computed `factor` comes out ≤ 0 (extreme twist), `raise` a self-diagnosing error naming the gear, the segment's `u`, and the factor — never scale by a non-positive factor. **`_CROWN_PER_RAD` is a tunable class constant, default `0.5`** (0 disables the crown) — set it to 0.5, do not leave it unset/0.

⚠️ **Do NOT key the relief on `|ang|` (the twist magnitude, i.e. |distance from mid-face|).** That is **symmetric** about mid-face — maximal at BOTH ends — so, because the heel slab is held full (gotcha 2), the slab *just inside* the heel becomes the **most**-relieved one and dips below both its neighbours (a notch), reversing the heel→toe taper. This was the observed bug: the heel-adjacent slab came out at factor `0.932` while the next slab inward was `0.972` (taller). Key the relief on the monotonic heel-distance `u`, never on `|ang|`. Three gotchas:

1. **The scale base must be a sketch point** (a point added in a sketch on the heel face, or a BRep vertex), per `[PB-CONSTRUCTION-NEEDS-ACTIVE]`. (`scaleFeatures` is the ONE exception to never-activate: it needs the Design occurrence as the **active** edit target — call **`designOccurrence.activate()`** (a method on the `Occurrence`) before the crown scales, and restore afterward — in a `finally` — with **`design.activateRootComponent()`** (a method on `Design`). ⚠️ Do **NOT** write `design.rootComponent.activate()` or `someComponent.activate()` — a `Component` has **no** `.activate()` method and raises `AttributeError`. Only `Occurrence` has `.activate()`; the root is re-activated via `Design.activateRootComponent()`.)
2. **Skip the outermost (heel) segment.** Its heel face is the loft's heel end and must stay full so the heel cone (step J) trims it flush with the gear base.
3. **Anchor the scale on the heel face's ROOT edge, NOT its centroid — otherwise the crowned tooth lifts off the gear base.** `scaleFeatures` shrinks **uniformly** toward the base point, so a base point at the heel-face **centroid** (mid tooth-height) pulls the tooth's **root** edge *upward* by `(1−factor)·(½ tooth height)` → the tooth no longer seats on the gear body's root cone, floats above the base, and the Combine-Join leaves a gap (clearly visible for ratio pairs — e.g. module 2 / driving 19 / pinion 13 — the original symptom that exposed this). Put the base point on the **root** instead: of the heel face's vertices (`heelFace.vertices`, each `.geometry` a world `Point3D`), take the **two with the smallest perpendicular distance to the shaft axis** (the line through `apex` along `axisDir`; perpendicular distance = `|(p−apex) − ((p−apex)·axisDir)·axisDir|`) — those are the two **root corners** (the tip corners are the farthest from the axis) — and place the base sketch point at their **midpoint** (mapped into the heel-face sketch via `modelToSketchSpace`). The heel face is a planar cut, so that midpoint lies on it. Rationale: a uniform scale about a point keeps every line/plane through that point invariant, so anchoring on the root keeps the root edge on the seating cone (the tooth stays flush) while the tip is relieved progressively toward the toe — which is exactly the lengthwise crown intended. (Finding the heel face itself is unchanged — still the max-`distAlong`-centroid face per step G; only the point *on* it changes from centroid to root-edge midpoint.)

**I. Loft → curved tooth.** ⚠️ **Re-sort the segments by their heel-face cone distance HERE, AFTER the twist (G) and crown (H) — do NOT reuse the pre-twist slice/centroid order from step F.** The twist rotates each slab about the shaft axis, and for high-twist *unequal-ratio* pairs that rotation changes the slabs' along-cone (`distAlong`) order enough to **reorder adjacent slabs**; lofting in the stale pre-twist order then assembles the cross-sections out of sequence and the crowned tooth comes out distorted → the two gears interfere. (For equal/low-twist pairs the two orders coincide, which is why equal-teeth gears mesh even with the stale order but unequal ratios distort — this is the single thing that makes a ratio pair like 31/17 fail while 31/31 looks fine.) So: compute `order = sorted(segment indices, key = distAlong(slabHeelFace(seg).centroid))` **now**, and loft a NewBody through, in that order: first the **toe-most segment's apex-side (toe-facing) face** — the toe segment is `order[0]`, its toe face added first to push the loft past the toe cone so the toe trim bites — then the **heel-facing face of every segment, iterated in `order`** (each segment's farthest-along-the-element face by post-twist centroid; the last reaches past the heel cone). Name the resulting body **`{gear} Spiral Tooth`**. Then remove the segment scaffolding (the loft has captured their faces).

**J. Flush trim + mesh phase.** Return `cut_conical_ends(designComponent, curvedTooth, gearBody, toeMid, heelMid, apexWorld, gearLabel)` (framework) — the same toe-then-heel two-cone trim used for the straight tooth — so the curved tooth's ends sit **flush** on the gear base. The toe/heel **mesh phasing** is handled outside this hook by `_createGearBody`'s mesh-rotate step (driving by `180°/drivingTeeth`; the **pinion additionally** by `_pinionMeshPhase(pinionTeeth)`, which is 0 by default because the mid-face section is unrotated and already meshes — see Method contract).

## Create the Gear Bodies (once per gear — pinion first, then driving)

Run this whole section **once per gear** — pinion first, then driving — with these substitutions:

| | Pinion | Driving |
|---|---|---|
| hexagon vertices, in draw order | A -> G -> H -> C -> M -> N -> A | B -> I -> J -> D -> O -> P -> B |
| profile sketch name | `Pinion Profile` | `Driving Profile` |
| shaft-axis edge (the hexagon's FIRST edge) | A->G | B->I |
| toe cut edge | M->N | O->P |
| heel cut edge | C->H | D->J |
| teeth / bore / pitch-diameter inputs | Pinion Gear … | Driving Gear … |
| §2 shaft construction line (NOT usable as the axis) | Apex->A | Apex->B |

Create a new component as a child of the **Bevel Gear** component (the same component that owns Design — *not* the user's Parent Component; this intentionally overrides the looser "child of Parent Component" phrasing so the pair nests cleanly inside Bevel Gear), named `{gearLabel} Gear` (`Pinion Gear` / `Driving Gear`). The resulting bodies for this gear end up in this component. (Implementation note: Fusion rejects cross-sibling sketch and project calls even when the target is activated or the entities are wrapped in `createForAssemblyContext` proxies — `[PB-NO-CROSS-SIBLING]` — so the actual feature operations run in the Design component and the finished bodies are `moveToComponent`'d here at the end. The visible end state is identical.)

**Profile sketch.** Open a **fresh sketch on the axial (Gear Profiles) plane**, named per the table — **one profile sketch per gear**, so `sketch.profiles` holds exactly this one hexagon loop (do not draw both gears' hexagons in the shared Gear Profiles sketch — that would leave two identically-shaped loops to disambiguate). Build the hexagon on fixed vertices per the `[PB-PROJECT-NOT-FIXED]` recreate-share-fix recipe: recreate the six §2 vertices as new points at their exact (world-mapped) positions — valid because §2 is fully constrained by now — then draw the closed hexagon (in the table's draw order) as six `SketchLine`s **sharing** those points, then fix the lines and their endpoints **after** the lines exist (not before). The hexagon's **first edge is the gear's shaft axis** for the revolve, pattern, bore plane AND the meshing-rotation axis, so it must be fixed well enough to carry a trustworthy world position: fixed endpoints give that edge a well-defined `worldGeometry` (`[PB-WORLDGEO-CONSTRAINED]`); a free edge resolves against a default/world-XY frame and silently moves the body onto world XY (observed on the driving gear — the pinion looked fine only because it never read the edge's `worldGeometry`).

**The shaft axis used by every body operation below is this profile sketch's first edge, NOT the §2 `Apex->A`/`Apex->B` construction line.** The edge is collinear with the shaft axis but lives in the *same* sketch as the profile, which is what Fusion's revolve/pattern/path accept; reusing the §2 construction line (a different sketch) fails or misbuilds.

**Revolve.** This sketch holds exactly one hexagon loop, so take its single profile (`[PB-SINGLE-PROFILE]`) and revolve it around the shaft-axis edge; let the result be the Gear Body (the frustum). Because the toe edge is one edge of the revolved profile, the body already carries the conical face produced by sweeping it around the axis — that face is reused as the cutting tool below (likewise the heel edge's cone).

**Tooth loft.** Loft the **§2 Apex sketch point** (the `centerToApex.endSketchPoint` from the Gear Profiles sketch — the degenerate point-section) to this gear's §3 Tooth profile; let the result be the Tooth Body. Use the §2 Apex SKETCH point directly — do NOT create a construction point for it (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`; the Design component is never active).

**Conical cuts.** Trim the Tooth Body to a flush band with the framework helper — return
`cut_conical_ends(designComponent, toothBody, gearBody, toeMid, heelMid, apexWorld, gearLabel)`
(from `.solids`; do **NOT** re-implement the cut machinery). **Two distinct bodies are involved —
do not conflate them:** the cutting TOOLS are `ConeSurfaceType` faces of the **Gear Body (the
revolved-hexagon frustum)** — the lofted Tooth Body has no cone faces, so searching *it* for the
cone face finds none — and the TARGET being split is the **Tooth Body (the loft)**.

The helper implements the pinned cut behavior (see the PLAYBOOK "Shared geargen helper library"):
the **toe cut first**, its cone face identified by the toe edge's world **MIDPOINT** best-first
across the frustum's cone faces (`[PB-FACE-BY-MIDPOINT]` — endpoints sit near the apex
singularity), each candidate tried as the actual split tool and the first that splits (>1 piece)
kept; **keeper selection after each cut** (remove apex-containing pieces, keep the largest —
`[PB-REMOVE-PIECES]`); then the **heel cut on the keeper alone** (removing the apex tip first is
what makes it deterministically two split features for every gear ratio). A heel cone that does
not intersect the keeper at all (common on ratio pairs, e.g. module 1 / driving 31 / pinion 43,
where the heel cone never overshoots the tooth) is raised by the helper as the **typed
`solids.NonIntersectError`** and caught — the keeper is returned whole. Every failure is
self-diagnosing with the per-face distance/error history (`[PB-SELF-DIAGNOSING]`), and each cut's
outcome is logged with `force_console=True`.

**Caller obligations (these stay in the generator):** pass `toeMid` = the toe edge's world
midpoint (`(M_world + N_world)/2` pinion / `(O_world + P_world)/2` driving) and `heelMid` = the
heel edge's world midpoint (`(C_world + H_world)/2` / `(D_world + J_world)/2`) — the same
edge-midpoint pairs as the §3a hand-off; `apexWorld` = the §2 Apex sketch point's world geometry;
`gearBody` = the revolved frustum (the cone-face source). The toe cut must split (its failure
propagates and crashes the build — correct, since an uncut tooth is unusable); only the heel cut
is lenient, and only via the typed `NonIntersectError`.

**Pattern.** Circular-pattern the remaining tooth piece around the **shaft-axis edge** (the same in-sketch profile edge used for the revolve, not the §2 construction line). The number of copies equals this gear's Teeth Number. Pin the pattern inputs: `quantity = <Teeth Number>`, `totalAngle = '360 deg'` (full circle), `isSymmetric = False`. (Although the pitch diameter shrinks from heel toward apex, the *angular* spacing around the shaft axis stays constant at `360° / N` for the entire face width — the radial taper is already produced by the loft from Apex to the heel-end tooth profile, so the pattern just rotates that single tapered tooth into N evenly spaced copies.)

**Combine.** Join all patterned tooth pieces with the Gear Body in a single Combine-Join (the Gear Body as the target, the patterned tooth bodies as the tools).

**Bore.** If Enable Bore is checked, cut a cylindrical through bore through the Gear Body along the shaft axis. The bore diameter is this gear's Bore Diameter if specified (non-zero); otherwise `this gear's Pitch Diameter / 4`. Build the bore plane normal to the shaft at its start (`setByDistanceOnPath(<shaft-axis edge>, 0.0)`; pass the in-sketch edge, not the §2 construction line). In a sketch named `{gearLabel} Bore`, sketch the bore circle centered at the sketch origin (the plane is rooted at the shaft start, so the origin is on the axis): **fix the bore circle's center and add a diameter dimension** set to the bore diameter (`[PB-CIRCLE-CENTER]`). Extrude-cut it as a symmetric through-cut restricted to `[this Gear Body]` (`[PB-THROUGH-CUT]`; use `2 × Cone Distance` as the per-side half-length — generously past any face width). Skip this step entirely if Enable Bore is unchecked.

**Meshing rotation (driving gear only) — do this here, in the Design component, before the body is moved out.** Rotate the driving body by `180° / Driving Gear Teeth Number` (half a tooth pitch) about its shaft axis — `rotate_body_about_edge(designComponent, gearBody, shaftAxisEdge, angle)` (framework, from `.solids`), which takes the rotation axis/origin from the B->I profile edge's **world** endpoints (`[PB-MOVE-ROTATE]`) — so a driving valley sits where the pinion tooth crosses the axial plane, giving the interlocked meshing look. Rationale: both gears are patterned from a starting tooth in the axial plane, so without the offset a driving tooth and a pinion tooth would both sit at the axial-plane crossing and visually collide. This runs in Design before `moveToComponent` because a construction axis can't be added in the moved-out gear component (`[PB-CONSTRUCTION-NEEDS-ACTIVE]`), so the rotation must use the edge's world geometry while still in Design. (The pinion additionally gets `_pinionMeshPhase(pinionTeeth)` — 0 unless a spiral pair needs it; see Method contract.)



## Cleanup

Call the framework's `hide_construction_geometry(bevelComponent)` (from `.solids`) — it recursively walks the Bevel Gear component tree (dedupe by `entityToken`) and hides every sketch, construction plane, and construction axis with `isLightBulbOn = False` (construction planes/axes are **not** hidden by `isVisible` — `[PB-HIDE-AFTER-USE]`). Leave only the two finished gear bodies visible.

(The driving gear's half-tooth-pitch **meshing rotation** is performed earlier, at the end of "Create the Gear Bodies", in the Design component before the body is moved out — not a cleanup step; see that section for the rationale.)