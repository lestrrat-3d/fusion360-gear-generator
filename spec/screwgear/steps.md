The proof of this step list is `proof/screwgear/compiled_model_test.go`, `proof/screwgear/compiled_sketches_test.go`, `proof/screwgear/compiled_solids_test.go` and `proof/screwgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `4e888f38cbd9c5d5ddbe2e5a10942421aab4d6a6` |
| `spec/screwgear/fusion.md` | `67ed5840cd39ad1bdd72144fbf82b449007ec325` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `485a08412a102a277ec5491d5feb2fca2d7ae20c` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/screwgear/mesh-search.md` | `245bc5c833387a83598ee6c8a7971e8efd5825be` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `cdd32545b0f8c651752827c6697601f1e32b4d39` |

## S01 `[PROSE]` Module, classes, constants and entry wiring

Write `lib/geargen/screwgear.py`. It subclasses `base.Generator` directly and shares no involute
math with any other gear. Imports, explicitly and nothing more (no `import *`):

```python
import math
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import get_design
from .base import Generator
from .utilities import find_profile_by_curve_counts
from . import solids
```

**Public classes, exactly these two** (the command wiring binds to them by name, and
`lib/geargen/__init__.py` exports them):

- `ScrewGearCommandInputsConfigurator`, with the classmethod `configure` of S02, taking `(cls, command)`.
- `ScrewGearGenerator(Generator)`, with the one-argument constructor `(design)` it inherits,
  `generate` (S04), taking `(self, inputs)`, and `prefixBase`, taking `(self)` and returning `'ScrewGear'`, overriding the
  inherited `self.prefixBase()`. It relies on the inherited `self.deleteComponent()` for cleanup on
  failure and the inherited `self.getOccurrence()` for its top occurrence. No generation context
  class: every handle is carried on `self` (S04).

**Entry wiring.** `commands/screwgear/entry.py` constructs
`GearCommand(gear_type='ScrewGear', name='Screw Gear Generator', description='Generates a screw/screw gear pair and its cage', icon_folder=..., configurator=geargen.ScrewGearCommandInputsConfigurator, generator_class=geargen.ScrewGearGenerator)`
and re-exports `start = command.start` and `stop = command.stop`, as every other gear's entry does.

**Module constants.** One per dialog input id, named for it:

| Constant | Value |
|---|---|
| `INPUT_ID_PLANE` | `'plane'` |
| `INPUT_ID_POINT` | `'point'` |
| `INPUT_ID_PARENT` | `'parent'` |
| `INPUT_ID_RIBBON_WIDTH` | `'ribbonWidth'` |
| `INPUT_ID_TOOTH_COUNT` | `'toothCount'` |
| `INPUT_ID_TWIST_LEAD` | `'twistLead'` |
| `INPUT_ID_RIBBON_THICKNESS` | `'ribbonThickness'` |
| `INPUT_ID_TOOTH_PITCH` | `'toothPitch'` |
| `INPUT_ID_TOOTH_HEIGHT` | `'toothHeight'` |
| `INPUT_ID_TOOTH_SLANT` | `'toothSlant'` |
| `INPUT_ID_TOOTH_BOW` | `'toothBow'` |
| `INPUT_ID_CAGE_RADIUS` | `'cageRadius'` |
| `INPUT_ID_CAGE_RISE` | `'cageRise'` |
| `INPUT_ID_CLEARANCE` | `'clearance'` |
| `INPUT_ID_ROOF_ALLOWANCE` | `'roofAllowance'` |
| `INPUT_ID_COLLAR_HALF` | `'collarHalf'` |
| `INPUT_ID_COLLAR_WALL` | `'collarWall'` |
| `INPUT_ID_CROSS_ANGLE` | `'crossAngle'` |
| `INPUT_ID_ENGAGEMENT` | `'engagement'` |
| `INPUT_ID_MOUNT_ANGLE_A` | `'mountAngleA'` |
| `INPUT_ID_MOUNT_ANGLE_B` | `'mountAngleB'` |
| `INPUT_ID_ASSEMBLY_PHASE` | `'assemblyPhase'` |

Two more module constants are not dialog inputs: `TOOTH_SPLINE_POINTS = 11`, the number of points
each section's toothed side is fitted through (S10), and `CELL_TEETH = 4`, the number of teeth the
lofted cell holds, written `cellTeeth` below. Every count that depends on `CELL_TEETH` is derived
from it in `processInputs` (S03); nothing hard-codes 4.

**Parameter mode: all-Python-precomputed** `[PB-PRECOMPUTED-MODE]`. Every value is computed in
Python and written numerically: sketch dimensions through `dimension.parameter.value`, feature
inputs through `ValueInput.createByReal`. The generator registers no user parameter and never
calls `addParameter`. Lengths are held in centimetres, Fusion's internal unit, and angles in
radians; the two searches of S03 alone work on millimetre figures (S03 says how).

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "prefixBase",
      "owner": null,
      "reason": "base.Generator.prefixBase is the hook this gear overrides",
      "receiver": "self",
      "role": "inherited",
      "span": "self.prefixBase()"
    },
    {
      "condition": null,
      "name": "deleteComponent",
      "owner": null,
      "reason": "base.Generator.deleteComponent removes the top occurrence on failure",
      "receiver": "self",
      "role": "inherited",
      "span": "self.deleteComponent()"
    },
    {
      "condition": null,
      "name": "getOccurrence",
      "owner": null,
      "reason": "base.Generator.getOccurrence creates the top occurrence",
      "receiver": "self",
      "role": "inherited",
      "span": "self.getOccurrence()"
    }
  ],
  "citations": [
    {
      "first": 8,
      "last": 9,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 566,
      "last": 597,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 673,
      "last": 686,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 17,
      "last": 41,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 291,
      "last": 324,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 962,
      "last": 979,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L8–9; `spec/screwgear/instructions.md` L566–597; `spec/screwgear/instructions.md` L673–686; `.claude/skills/generate-gear/PLAYBOOK.md` L17–41; `.claude/skills/generate-gear/PLAYBOOK.md` L291–324; `.claude/skills/generate-gear/PLAYBOOK.md` L962–979.

## S02 `[PROSE]` The dialog: `configure`

The classmethod `configure` of `ScrewGearCommandInputsConfigurator`, taking `(cls, command)`, adds the inputs below to
`command.commandInputs`, in exactly this order. No input has conditional visibility.

The three selections come first and at the top level, because nothing can be built without them
and Fusion focuses the first selection input it is given `[PB-AUTOFOCUS-FIRST]`. Each is
`addSelectionInput(id, label, tooltip)` with its filters added as the named enum constants
`[PB-SELECTION-FILTER-ENUM]` through `addSelectionFilter(filter)`, then
`setSelectionLimits(1, 1)` `[PB-SELECTION-DECL]`:

| Order | id | Label | Tooltip | Filters |
|---|---|---|---|---|
| 1 | `plane` | `Target Plane` | `Plane the cage's axis is normal to` | `adsk.core.SelectionCommandInput.ConstructionPlanes`, `adsk.core.SelectionCommandInput.PlanarFaces` |
| 2 | `point` | `Centre Point` | `Centre of the mechanism` | `adsk.core.SelectionCommandInput.ConstructionPoints`, `adsk.core.SelectionCommandInput.SketchPoints` |
| 3 | `parent` | `Parent Component` | `Component the mechanism is created under` | `adsk.core.SelectionCommandInput.Occurrences`, `adsk.core.SelectionCommandInput.RootComponents` |

The Parent Component input pre-selects the design's root component:
`parentInput.addSelection(get_design().rootComponent)`.

Then three groups, each `command.commandInputs.addGroupCommandInput(groupId, groupLabel)`, whose
inputs are added to the group's `children` collection, not to the top-level `commandInputs`. The
`ribbonGroup` and `frameGroup` groups start expanded; the `meshGroup` group starts collapsed,
`group.isExpanded = False`, because its values come from the meshing search and jam when changed
carelessly. The groups are added in the order Ribbon, Frame, Mesh, each right before its own
inputs.

Every value input is `group.children.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))`
with the default in Fusion's internal units `[PB-DIALOG-DEFAULT-UNITS]`: a length in mm is written
as the mm figure divided by 10 (cm), an angle in degrees as its radians
(`math.radians` of the degrees), and the two unitless inputs as the bare number. The unit strings are
`'mm'`, `'deg'` and `''`.

| Order | Group id | Group label | id | Label | Unit | Default (display) | `createByReal` argument |
|---|---|---|---|---|---|---|---|
| 4 | `ribbonGroup` | `Ribbon` | `ribbonWidth` | `Ribbon Width` | `'mm'` | 15 mm | `1.5` |
| 5 | `ribbonGroup` | `Ribbon` | `toothCount` | `Tooth Count` | `''` | 68 | `68` |
| 6 | `ribbonGroup` | `Ribbon` | `twistLead` | `Twist Lead` | `'mm'` | 49.5 mm | `4.95` |
| 7 | `ribbonGroup` | `Ribbon` | `ribbonThickness` | `Ribbon Thickness` | `'mm'` | 3.75 mm | `0.375` |
| 8 | `ribbonGroup` | `Ribbon` | `toothPitch` | `Tooth Pitch` | `'mm'` | 2.625 mm | `0.2625` |
| 9 | `ribbonGroup` | `Ribbon` | `toothHeight` | `Tooth Height` | `'mm'` | 2.625 mm | `0.2625` |
| 10 | `ribbonGroup` | `Ribbon` | `toothSlant` | `Tooth Slant` | `'deg'` | 25.8° | radians of 25.8 |
| 11 | `ribbonGroup` | `Ribbon` | `toothBow` | `Tooth Bow` | `''` | 0.048 (mm⁻¹) | `0.048` |
| 12 | `frameGroup` | `Frame` | `cageRadius` | `Cage Radius` | `'mm'` | 15 mm | `1.5` |
| 13 | `frameGroup` | `Frame` | `cageRise` | `Cage Rise` | `'mm'` | 18.75 mm | `1.875` |
| 14 | `frameGroup` | `Frame` | `clearance` | `Clearance` | `'mm'` | 0.20 mm | `0.02` |
| 15 | `frameGroup` | `Frame` | `roofAllowance` | `Roof Allowance` | `'mm'` | 0.30 mm | `0.03` |
| 16 | `frameGroup` | `Frame` | `collarHalf` | `Collar Half Length` | `'mm'` | 3 mm | `0.3` |
| 17 | `frameGroup` | `Frame` | `collarWall` | `Collar Wall` | `'mm'` | 3 mm | `0.3` |
| 18 | `meshGroup` | `Mesh (from the mesh search)` | `crossAngle` | `Crossing Angle` | `'deg'` | 80° | radians of 80 |
| 19 | `meshGroup` | `Mesh (from the mesh search)` | `engagement` | `Engagement` | `'mm'` | 1.05 mm | `0.105` |
| 20 | `meshGroup` | `Mesh (from the mesh search)` | `mountAngleA` | `Mounting Angle A` | `'deg'` | 0° | `0` |
| 21 | `meshGroup` | `Mesh (from the mesh search)` | `mountAngleB` | `Mounting Angle B` | `'deg'` | 0° | `0` |
| 22 | `meshGroup` | `Mesh (from the mesh search)` | `assemblyPhase` | `Assembly Phase` | `'mm'` | −1.31 mm | `-0.131` |

The tooth bow is a bare number in mm⁻¹; the build, which works in cm, lowers the toothed edge by
`10*toothBow*v^2` cm for `v` in cm (S10).

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "addSelectionInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addSelectionInput(id, label, tooltip)"
    },
    {
      "condition": null,
      "name": "addSelectionFilter",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addSelectionFilter(filter)"
    },
    {
      "condition": null,
      "name": "setSelectionLimits",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "setSelectionLimits(1, 1)"
    },
    {
      "condition": null,
      "name": "addSelection",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "parentInput",
      "role": "required",
      "span": "parentInput.addSelection(get_design().rootComponent)"
    },
    {
      "condition": null,
      "name": "get_design",
      "owner": null,
      "reason": "lib/geargen/misc.py defines get_design for every gear",
      "receiver": null,
      "role": "inherited",
      "span": "parentInput.addSelection(get_design().rootComponent)"
    },
    {
      "condition": null,
      "name": "addGroupCommandInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "command.commandInputs",
      "role": "required",
      "span": "command.commandInputs.addGroupCommandInput(groupId, groupLabel)"
    },
    {
      "condition": null,
      "name": "addValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "group.children",
      "role": "required",
      "span": "group.children.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "group.children.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))"
    }
  ],
  "citations": [
    {
      "first": 617,
      "last": 698,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 128,
      "last": 143,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 355,
      "last": 357,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 557,
      "last": 568,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L617–698; `.claude/skills/generate-gear/PLAYBOOK.md` L128–143; `.claude/skills/generate-gear/PLAYBOOK.md` L355–357; `.claude/skills/generate-gear/PLAYBOOK.md` L557–568.

## S03 `[PROSE]` `processInputs`: read, check, precompute

`processInputs(inputs)` runs first in `generate` and creates nothing in the design.

**Read the selections before anything else** `[PB-SELECTION-STASH]`: for each of `plane`, `point`
and `parent`, `inputs.itemById(id)`, raising naming the id if the lookup returns `None`, and take
its `selectionInput.selection(0).entity`, `selectionInput` being the input the lookup returned. The parent is an `Occurrence` (use its `component`) or a `Component`;
store it as `self.parentComponent`. Store the plane as `self.targetPlane` and the point as
`self.centrePoint`.

**Read every value input** by id with `inputs.itemById(id)` on the command's top-level
`commandInputs`, grouped or not (ids are unique across the command, which is what lets the lookup
reach into a group); raise naming the id if any lookup returns `None`. Read each raw value with
`design.unitsManager.evaluateExpression(input.expression, units)` `[PB-EVAL-EXPRESSION]`, where
`design` is `get_design()`, with `units` `'mm'` for a length, `'deg'` for an angle and `''` for
`toothCount` and `toothBow`. It returns internal units: cm for a length and radians for an angle;
compare an angle in degrees with `math.degrees`. Call the values `W` (ribbonWidth), `N`
(toothCount), `TwistLead`, `T` (ribbonThickness), `P` (toothPitch), `H` (toothHeight), `Slant`
(toothSlant, radians), `Bow` (toothBow, mm⁻¹), `cageRadius`, `cageRise`, `clearance`,
`roofAllowance`, `collarHalf`, `collarWall`, `Sigma` (crossAngle, radians), `Engagement`, `PhiA`,
`PhiB` (the mounting angles, radians) and `assemblyPhase`.

**Range checks**, in this order, each raising a clear error that names the input id and states the
bound (lengths in the message in mm):

1. `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `collarHalf`, `collarWall` and
   `clearance` must each be `> 0`.
2. `roofAllowance` must be `>= 0` (zero cuts every bore to the clearance alone).
3. `toothCount` must be a whole number `>= 4`; then `N = int(round(value))`.
4. `toothHeight` must be `> 0` and `< ribbonWidth/2`.
5. `toothSlant` must lie strictly between −90° and 90°.
6. `toothBow` must be `>= 0`, and `H + Bow_cm*(T/2)^2 < W/2` with `Bow_cm = 10*Bow`; the message
   names `toothBow`.
7. `engagement` must be `> 0` and `<= toothHeight`.
8. `crossAngle` must lie strictly between 0° and 180°.
9. `assemblyPhase` must lie strictly within `±toothPitch`.
10. `cageRadius + collarHalf + 0.1 cm < N*P/2` (a millimetre of cut margin); the message names
    `cageRadius`.

No range is enforced on either mounting angle, and none on `twistLead` above zero.

**Derived values**, in the build's cm and radians:

```
Lambda   = TwistLead / (2*pi)              mm (cm) of advance per radian
tanSlant = tan(Slant)
Bow_cm   = 10 * Bow                        the edge drops Bow_cm * v^2 cm at v cm
A        = W - Engagement                  distance between the two axes
Ri       = cageRadius - collarHalf         sleeve inner radius        (12 mm at the defaults)
Ro       = cageRadius + collarHalf         sleeve outer radius        (18 mm)
hw       = W/2 + clearance                 the bore's half-width      (7.70 mm)
ht       = T/2 + clearance                 the bore's half-thickness  (2.075 mm)
a        = roofAllowance
c        = hypot(hw, ht + a)               furthest bore corner from its axis (8.058 mm)
sIn      = sqrt(Ri^2 - c^2) - 0.1 cm       where a +R bore's cut starts (7.892 mm)
sOut     = Ro + 0.1 cm                     where it ends               (19 mm)
axialWindow = 1.5 * sqrt(W^2 - A^2) / sin(Sigma)   (8.40 mm)
cell     = min(CELL_TEETH, N)              teeth in the lofted cell
n        = max(ceil((P/Lambda) / radians(2)), 8)   steps per tooth (10 at the defaults)
q        = N // cell                       whole cells (17 at the defaults)
r        = N % cell                        remainder teeth (0 at the defaults)
Z0       = [0, assemblyPhase]              tooth phase of gear A and gear B
Phi      = [PhiA, PhiB]
```

`ceil` of a quotient that is a whole number up to rounding must not step past it: compute the
twist step count as the ceiling of `x - 1e-12`.

**The sleeve's four checks**, next and in this order, each naming the input given and the bound,
with the measured value in the message:

1. *The channel starts in the hollow*, naming `cageRadius`: `hypot(c, 0.1 cm) < Ri`, which is
   `sIn > 0`.
2. *The mesh stays visible along the axis*, naming `cageRadius`:
   `hypot(axialWindow, hypot(W/2, T/2)) + clearance <= Ri` (11.61 mm against 12 mm at the defaults).
3. *The end faces keep collarWall*, naming `cageRise`: `cageRise >= A/2 + c + collarWall`
   (18.03 mm at the defaults).
4. *The wall between neighbouring bores keeps collarWall*, naming `collarWall`: the separation of
   "The wall between the bores" below must be `>= collarWall`; the message names the two bores
   and the separation. It is 5.067 mm at the defaults, across the `-ê` gap between the two `-R`
   bores.

**The frame of the two searches.** The searches run before any feature, so they cannot read `C`,
`ê` or `n̂` from the design; they work in an abstract frame with `C` at the origin, `ê`, `k̂` and
`n̂` the X, Y and Z axes, and every length in **millimetres** (multiply each cm value by 10 on
entry; the step sizes below are millimetres). Every point they hand on is expressed in that frame
as `(t, z)` plane coordinates or as `along ê, along k̂, along n̂` components, which S05 onwards maps
onto the real `C`, `ê`, `k̂` and `n̂`; every length they hand on is divided by 10 back to cm. In it:

```
dirA = ( cos(Sigma/2),  sin(Sigma/2), 0)     originA = (0, 0, -A/2)    uA = (0, 0,  1)
dirB = ( cos(Sigma/2), -sin(Sigma/2), 0)     originB = (0, 0, +A/2)    uB = (0, 0, -1)
v_g  = dir_g x u_g
Theta_g(s) = s/Lambda + Phi_g
turn(u, v, th) = (u*cos(th) - v*sin(th), u*sin(th) + v*cos(th))      -> (x along u_g, y along v_g)
Point_g(s, u, v) = origin_g + s*dir_g + x*u_g + y*v_g,  (x, y) = turn(u, v, Theta_g(s))
```

**The four bores**, in this fixed order, which is also the order of S21's cuts: gear A `-R`,
gear A `+R`, gear B `-R`, gear B `+R`. A bore has its gear `g`, its sign `sigma` (−1 for `-R`, +1
for `+R`), its cut span (`[-sOut, -sIn]` for `-R`, `[sIn, sOut]` for `+R`), its crossing station
`sc = sigma*cageRadius`, and its rectangle `u` from `-hw` to `hw`, `v` from `vLo` to `vHi`.

**The level bore and the roof allowance.** For each gear, compute each bore's tilt: take
`theta` at the two stations `sigma*(cageRadius - collarHalf)` and `sigma*(cageRadius + collarHalf)`;
when some `pi/2 + k*pi` (any integer `k`) lies between the two, inclusive, the tilt is 0, and
otherwise it is the smaller `|cos(theta)|` of the two. The bore with the smaller tilt is the gear's
**level bore**; on a tie, the `-R` bore. At the defaults both tilts are 0 and each gear's level
bore is its `-R` bore. The level bore's **roof** is its `+v` face when
`-sin(Theta_g(sc)) * (u_g . n)` is positive at its crossing station (`u_g . n` is +1 for gear A and
−1 for gear B), and its `-v` face otherwise: gear A's `+v` face and gear B's `-v` face at the
defaults. Then:

```
level bore, roof +v :  vLo = -ht       vHi = ht + a
level bore, roof -v :  vLo = -ht - a   vHi = ht
any other bore      :  vLo = -ht       vHi = ht
```

Store the four bores, each with these values, on `self` for S20 to S22.

**A bore's section in the wall** (used by both searches). For bore `b` at station `s` with
`|s| < Ro`, in the section plane's `(x, y)`: turn the corners `(-hw, vLo)`, `(hw, vLo)`,
`(hw, vHi)`, `(-hw, vHi)`, in that order, by `Theta_g(s)`; that polygon runs counter-clockwise.
Clip it to the heights inside the end faces, keeping the `x` for which `zg + x*un` lies within
`±cageRise`, with `zg = (origin_g).z` (−A/2 or +A/2) and `un = u_g . n` (±1). Then clip what is left
twice more, separately: to `near <= y <= far` and to `-far <= y <= -near`, with
`near = sqrt(max(0, Ri^2 - s^2))` and `far = sqrt(Ro^2 - s^2)`. Those two are the section's two
**pieces** (either may be empty). At `|s| >= Ro` there is no piece. Every clip keeps the part of
a convex polygon on one side of a line by walking its edges in order (Sutherland–Hodgman): keep a
corner on the kept side (the boundary included), and add the point where an edge crosses the line.

```
clip(poly, a, b, c):          # keep a*x + b*y <= c
    out = []
    for each edge p -> q (q the next corner, the last edge back to the first):
        fp = a*p.x + b*p.y - c ; fq = a*q.x + b*q.y - c
        if fp <= 0: out.append(p)
        if (fp < 0 and fq > 0) or (fp > 0 and fq < 0):
            out.append(p + (q - p) * fp/(fp - fq))
    return out
heights:  clip(poly, un, 0, cageRise - zg) then clip(that, -un, 0, cageRise + zg)
piece 1:  clip(clip(poly, 0, -1, -near), 0, 1, far)
piece 2:  clip(clip(poly, 0, 1, -near), 0, -1, far)
```

**The wall between the bores** (the fourth sleeve check):

1. *Outline.* For each bore, at stations `s = sigma*(sIn + 0.1*k)` for `k = 0, 1, …` while
   `|s| <= sOut`, take seventeen points on each side of its rectangle — `(hw, v)` and `(-hw, v)`
   for `v = vLo + (vHi - vLo)*i/16`, and `(u, vHi)` and `(u, vLo)` for `u = -hw + 2*hw*i/16`,
   `i = 0 … 16` — turned to the station's angle and placed as `Point_g(s, u, v)`. Keep a point when
   `hypot(P.x, P.y)` lies within `Ri - 0.5` to `Ro + 0.5`.
2. *Gaps.* Four gaps, each between two bores and named by the direction `d` it faces: `+k`
   (`d = (0, 1, 0)`; gear A `+R` and gear B `-R`), `-e` (`d = (-1, 0, 0)`; gear A `-R` and gear B
   `-R`), `-k` (`d = (0, -1, 0)`; gear A `-R` and gear B `+R`) and `+e` (`d = (1, 0, 0)`; gear A `+R`
   and gear B `+R`). With `across = n x d` (`n = (0, 0, 1)`), project each of the gap's two bores'
   kept points to `(P . across, P.z)` and take each bore's convex hull by Andrew's monotone chain.
3. *Separation.* For every edge of either hull, with `m` the unit vector square to it, the
   separation along `m` is the larger of `min(m . q) - max(m . p)` and `min(m . p) - max(m . q)`,
   `p` over the first hull's corners and `q` over the second's. The gap's separation is the largest
   over all edges of both hulls (negative when the hulls overlap).
4. The least of the four gaps' separations must be `>= collarWall` (check 4 above).

**The window search.** It refuses nothing: a gap with no room gets no window, and the build logs
which and why. The two windows face `+k` then `-k` when `Sigma <= 90°`, and `+e` then `-e` past it
(`d` as in the gaps above). For each facing `d`, with `across = n x d`, a point `P` has plane
coordinates `t = P . across`, `z = P.z` and depth `a = P . d`; at `t` the wall runs from
`a0(t) = sqrt(max(0, Ri^2 - t^2))` to `a1(t) = sqrt(max(0, Ro^2 - t^2))`.

1. *Flanking and far bores.* A bore's crossing is `Point_g(sc, 0, 0)`, that is
   `origin_g + sc*dir_g`. The two bores whose crossing has `crossing . d > 0` flank the window; the
   other two are its far bores. The low flanking bore is the one whose crossing has the smaller
   `z`; `lean = +1` when the high one's crossing has the larger `t`, else `-1`.
2. *The long sides* (`wallCorners`). Walk each flanking bore's cut span at stations
   `spanLo + 0.001*k`, `k = 0, 1, …` while `<= spanHi`; at each, take every corner of both pieces as
   the point `origin_g + x*u_g + y*v_g + s*dir_g` and its `m = z + lean*t`. `lowReach` is the largest
   `m` over the low bore, `highReach` the least over the high bore. Then
   `lo = lowReach + sqrt(2)*collarWall` and `hi = highReach - sqrt(2)*collarWall`. No window when
   `hi <= lo`, the reason being that the flanking bores leave no band.
3. *zLimit* (`channelTop`), computed once for both windows: over all four bores, at stations
   `s = sigma*(sIn + 0.01*k)` while `|s| <= sOut`, at the stations with `|s| <= Ro` where
   `hypot(s, ymax) >= Ri` (`ymax` the largest `|u*sin(th) + v*cos(th)|` over the bore's four corners
   `(u, v)`, `th = Theta_g(s)`), the largest `|zg + (u*cos(th) - v*sin(th))*un|` over those corners
   (13.81 mm at the defaults).
4. *The trims.* `top = min(2*zLimit - hi, hi + sqrt(2)*Ri)` and
   `bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)`.
5. *The ends* (`windowEnd`). For `delta = +1` and `delta = -1`: bisect `te` 24 times between
   `lower = 0` and `upper = min(Ri, Ro/sqrt(2))*(1 - 1e-9)`: take `mid = (lower + upper)/2`, set
   `lower = mid` when `clear(delta*mid)` holds and `upper = mid` otherwise; the end is the final
   `lower`. `right = end(+1)`, `left = -end(-1)`. No window when `right <= left`, the reason being
   that the far bores leave the band no length. `clear(t)`:
   - `zLow = max(lo - lean*t, bottom + lean*t)`, `zHigh = min(hi - lean*t, top + lean*t)`; not
     clear when `zLow > zHigh`.
   - `need = collarWall + 0.1/sqrt(2) + 0.005` (3.076 mm at the defaults).
   - The points, each `t*across + z*n + a*d`: for each of `z = zLow` and `z = zHigh`, at
     `a = a0(t), a0(t) + 0.1, …` while below `a1(t)`, and then `a1(t)` itself; and, at both
     `a = a0(t)` and `a = a1(t)`, at `z = zLow + (zHigh - zLow)*k/nz` for `k = 1 … nz - 1`,
     `nz = ceil((zHigh - zLow)/0.1)`, and at `z = -zLimit + 0.1*j` for every `j >= 0` with
     `z <= zLimit` and `zLow < z < zHigh`.
   - Not clear as soon as any point's `wallGap` to either far bore, with `reach = need`, is under
     `need`; clear otherwise.
6. *The hexagon.* Clip the square `(-2*Ro, -2*Ro)`, `(2*Ro, -2*Ro)`, `(2*Ro, 2*Ro)`,
   `(-2*Ro, 2*Ro)`, in `(t, z)`, in this order to `lean*t + z <= hi`, `-lean*t - z <= -lo`,
   `-lean*t + z <= top`, `lean*t - z <= -bottom`, `t <= right` and `-t <= -left`. Drop a corner that
   lies within 0.001 mm of the one kept before it, and the last while it lies within 0.001 mm of the
   first. No window when fewer than three corners remain or their signed area is not positive.

At the defaults the `+k` window has `lo = -5.180`, `hi = 5.180`, `bottom = -22.151`,
`top = 22.151`, `left = -11.570`, `right = 11.570` and corners `(11.57, -6.39)`,
`(-8.49, 13.67)`, `(-11.57, 10.58)`, `(-11.57, 6.39)`, `(8.49, -13.67)`, `(11.57, -10.58)`, area
220.7 mm²; the `-k` window has `lo = -4.886`, `hi = 5.180`, `bottom = -21.857`, `top = 22.151`,
`left = -11.536`, `right = 11.570` and corners `(11.57, -6.39)`, `(-8.49, 13.67)`,
`(-11.54, 10.61)`, `(-11.54, 6.65)`, `(8.49, -13.37)`, `(11.57, -10.29)`, area 213.8 mm².

**wallGap(P, bore, reach)**, the distance from a point to a bore's channel in the wall:

```
x  = (P - origin_g) . u_g ;  y = (P - origin_g) . v_g ;  sq = (P - origin_g) . dir_g
if hypot(hypot(x, y), sq - min(max(sq, spanLo), spanHi)) - c >= reach: return reach
table = the bore's station table (built once per bore, before its first walk):
    stations spanLo + 0.002*k, k = 0, 1, … while <= spanHi, each with its non-empty pieces;
    wherever the set of non-empty pieces differs between two neighbouring stations, add
    stations every 0.0001 strictly between them; all in order of s.
    For each piece keep its circle: centre the average of its corners, radius the largest
    distance from that centre to a corner.
best = reach^2
walk up from the first station with s >= sq, then down from the one before it;
stop each way at the first station with (sq - s)^2 >= best.
at each station, for each non-empty piece:
    o = hypot(x - cx, y - cy) - radius
    if o > 0 and (sq - s)^2 + o^2 >= best: skip the piece
    d2 = 0 when (x, y) is inside the piece (on the inner side of every edge of the
         counter-clockwise polygon; a piece of fewer than three corners is never inside),
         else the least squared distance from (x, y) to the piece's edges
    best = min(best, (sq - s)^2 + d2)
return sqrt(best)
```

Store each window found on `self.windows` (zero to two, in facing order): its facing name (`+k`,
`-k`, `+e` or `-e`), its `d` and `across` as frame components, and its corners as `(t, z)` in cm.
For a facing with no room, log `No window facing {d}: {reason}` with `futil.log(...)`
`[PB-LOGGING]`, `{d}` the facing name, and keep no window for it.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "processInputs",
      "owner": null,
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "processInputs(inputs)"
    },
    {
      "condition": null,
      "name": "itemById",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.itemById(id)"
    },
    {
      "condition": null,
      "name": "selection",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.selection(0).entity"
    },
    {
      "condition": null,
      "name": "evaluateExpression",
      "owner": "adsk.core.UnitsManager",
      "reason": null,
      "receiver": "design.unitsManager",
      "role": "required",
      "span": "design.unitsManager.evaluateExpression(input.expression, units)"
    },
    {
      "condition": null,
      "name": "get_design",
      "owner": null,
      "reason": "lib/geargen/misc.py defines get_design for every gear",
      "receiver": null,
      "role": "inherited",
      "span": "get_design()"
    },
    {
      "condition": null,
      "name": "log",
      "owner": null,
      "reason": "lib/fusion360utils defines log as futil.log",
      "receiver": "futil",
      "role": "inherited",
      "span": "futil.log(...)"
    }
  ],
  "citations": [
    {
      "first": 591,
      "last": 597,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 668,
      "last": 782,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1208,
      "last": 1233,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1265,
      "last": 1294,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1403,
      "last": 1434,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1435,
      "last": 1587,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L591–597; `spec/screwgear/instructions.md` L668–782; `spec/screwgear/instructions.md` L1208–1233; `spec/screwgear/instructions.md` L1265–1294; `spec/screwgear/instructions.md` L1403–1434; `spec/screwgear/instructions.md` L1435–1587.

## S04 `[PROSE]` `generate` and the component tree

`generate`, taking `(self, inputs)`, calls, in order: `processInputs(inputs)` (S03), the method `buildComponentTree`
(this step, no argument), the method `buildAnchor` (S05 to S07, no argument), the method
`buildGear` with index 0 and then with index 1 (S08 to S16; each runs the methods
`buildSweepPaths`, `buildToothCell` and `repeatCellByDoubling`, each with that index), the method
`buildCage` (S17 to S26, no argument), the method `relocateBodies` (no argument) and the cleanup
(S27). The gear labels are
`'Gear A'` for index 0 and `'Gear B'` for index 1. Every failure raises; the command layer catches
it and calls the inherited `self.deleteComponent()` `[PB-LOGGING]`.

Handles carried on `self` (no generation context): `self.designOcc`, `self.gearOccs` (list of
two), `self.cageOcc`, `self.gearBodies` (list of two), `self.cageBody`, `self.pathLines` (list of
two dicts keyed `'bore-'` and `'bore+'`, each the Paths sketch line of S08 that bore's sweep runs
along), and `self.windows` (S03).

**The component tree** `[PB-OCCURRENCE-TREE]`. The top occurrence is the inherited
`self.getOccurrence()`, which creates it under `self.parentComponent` and is what
`self.deleteComponent()` deletes on failure; name its component `Screw Gearing`
(`component.name = 'Screw Gearing'`). Never call `addNewComponent` for it. Under that component
create four children, each `topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())`,
in this order, naming each `occurrence.component.name`: `Design`, `Gear A`, `Gear B`, `Cage`.
Keep the occurrences as `self.designOcc`, `self.gearOccs = [gearA, gearB]` and `self.cageOcc`.
Every sketch, construction plane and feature below is made in `self.designOcc.component`, called
`design` from here on; the three other children stay empty until S27 `[PB-NO-CROSS-SIBLING]`.

**Never call `occurrence.activate()`** `[PB-NEVER-ACTIVATE]`, on any occurrence. Sketches go
directly on the user-selected plane `[PB-USE-SELECTED-PLANE]`: no coplanar construction plane is
made from it, and nothing is normalised.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "processInputs",
      "owner": null,
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "processInputs(inputs)"
    },
    {
      "condition": null,
      "name": "deleteComponent",
      "owner": null,
      "reason": "base.Generator.deleteComponent removes the top occurrence on failure",
      "receiver": "self",
      "role": "inherited",
      "span": "self.deleteComponent()"
    },
    {
      "condition": null,
      "name": "getOccurrence",
      "owner": null,
      "reason": "base.Generator.getOccurrence creates the top occurrence",
      "receiver": "self",
      "role": "inherited",
      "span": "self.getOccurrence()"
    },
    {
      "condition": null,
      "name": "addNewComponent",
      "owner": "adsk.fusion.Occurrences",
      "reason": null,
      "receiver": "topComponent.occurrences",
      "role": "required",
      "span": "topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())"
    },
    {
      "condition": null,
      "name": "activate",
      "owner": "adsk.fusion.Occurrence",
      "reason": "PB-NEVER-ACTIVATE: activating mis-resolves the selected plane",
      "receiver": "occurrence",
      "role": "forbidden",
      "span": "occurrence.activate()"
    }
  ],
  "citations": [
    {
      "first": 577,
      "last": 582,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 598,
      "last": 616,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 855,
      "last": 869,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 929,
      "last": 960,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L577–582; `spec/screwgear/instructions.md` L598–616; `spec/screwgear/instructions.md` L855–869; `.claude/skills/generate-gear/PLAYBOOK.md` L929–960.

## S05 `[GO]` The Anchor sketch, and the frame read from it

The method `buildAnchor` begins here. Create the sketch with `design.sketches.add(self.targetPlane)` on the
user-selected plane itself `[PB-USE-SELECTED-PLANE]` and name it `Anchor`. Do not defer its
computing (it projects, and deferral under a projection has not been measured `[SCREW-F-DEFER]`).

1. Project the selected point: `projected = anchorSketch.project(self.centrePoint).item(0)`. This
   is the one projection in the build `[SCREW-F-REFERENCES]`. The API reference declares only
   `project2`; this gear keeps `project`, which every add-in that has loaded in Fusion calls.
2. Draw the **Anchor Line** from two raw `Point3D` seeds 0.5 cm either side of the projected point
   along the sketch's own x axis, the start to the left: with `p = projected.geometry`,
   `start = adsk.core.Point3D.create(p.x - 0.5, p.y, 0)` and
   `end = adsk.core.Point3D.create(p.x + 0.5, p.y, 0)`, then
   `anchorLine = anchorSketch.sketchCurves.sketchLines.addByTwoPoints(start, end)`. Every seed is
   the solved position `[PB-SEED-NEAR]`.
3. Constrain it with these four and nothing else, which is the bevel gear's Anchor sketch and reads
   fully constrained in Fusion:
   - `anchorSketch.geometricConstraints.addCoincident(projected, anchorLine)` and
     `anchorSketch.geometricConstraints.addMidPoint(projected, anchorLine)`: both, not the midpoint
     alone.
   - `anchorSketch.geometricConstraints.addHorizontal(anchorLine)`, sketch-local, so it survives a
     tilted plane `[PB-REFLINE-DIRECTION]`.
   - A **horizontal** distance dimension from the line's start to its end:
     `anchorSketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
     with `textPoint = adsk.core.Point3D.create(p.x, p.y + 0.3, 0)`, and its `parameter.value`
     set to `1.0` (cm, 10 mm). Not an aligned dimension: an aligned length with the midpoint and
     the horizontal is satisfied by the line either way round, and the horizontal distance from
     start to end holds the seeded orientation only `[PB-DIM-VALUE-SEMANTICS]`.
4. Raise naming `Anchor` unless `anchorSketch.isFullyConstrained` `[PB-FULL-CONSTRAINT]`.
5. Only then read the frame from world geometry `[PB-WORLDGEO-CONSTRAINED]`, `[PB-WORLD-FRAME]`:
   `C = projected.worldGeometry`, and `ê` the unit vector from
   `anchorLine.startSketchPoint.worldGeometry` to `anchorLine.endSketchPoint.worldGeometry`.
   Keep `anchorLine` as `self.anchorLine`: S23 builds the Window Plane on it. No later sketch
   projects the point or the line.

The proof draws the projected point as a reference point, which the engine never moves, and
writes the midpoint alone, because the engine's midpoint carries the coincident row itself. It
holds that the line solves where it was seeded and runs along `+x` from start to end, with the
centre at the origin, off it, and far off it.

Proof: `stepAnchorSketch`.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "design.sketches.add(self.targetPlane)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "anchorSketch",
      "role": "required",
      "span": "projected = anchorSketch.project(self.centrePoint).item(0)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "start = adsk.core.Point3D.create(p.x - 0.5, p.y, 0)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "end = adsk.core.Point3D.create(p.x + 0.5, p.y, 0)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "anchorSketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "anchorLine = anchorSketch.sketchCurves.sketchLines.addByTwoPoints(start, end)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "anchorSketch.geometricConstraints",
      "role": "required",
      "span": "anchorSketch.geometricConstraints.addCoincident(projected, anchorLine)"
    },
    {
      "condition": null,
      "name": "addMidPoint",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "anchorSketch.geometricConstraints",
      "role": "required",
      "span": "anchorSketch.geometricConstraints.addMidPoint(projected, anchorLine)"
    },
    {
      "condition": null,
      "name": "addHorizontal",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "anchorSketch.geometricConstraints",
      "role": "required",
      "span": "anchorSketch.geometricConstraints.addHorizontal(anchorLine)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "anchorSketch.sketchDimensions",
      "role": "required",
      "span": "anchorSketch.sketchDimensions.addDistanceDimension(anchorLine.startSketchPoint, anchorLine.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "textPoint = adsk.core.Point3D.create(p.x, p.y + 0.3, 0)"
    }
  ],
  "citations": [
    {
      "first": 914,
      "last": 945,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 563,
      "last": 581,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 441,
      "last": 450,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L914–945; `spec/screwgear/fusion.md` L563–581; `.claude/skills/generate-gear/PLAYBOOK.md` L441–450.

## S06 `[PROSE]` The Gear A Axis Plane, and the sign of n̂

`planeInput = design.constructionPlanes.createInput()`, then
`planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-A/2))` on the selected
plane itself `[PB-USE-SELECTED-PLANE]`, `[PB-CONSTRUCTION-PLANES]`, then
`design.constructionPlanes.add(planeInput)`, named `Gear A Axis Plane`.

Fusion offsets along the selected entity's own normal, whose sign the build does not assume
`[SCREW-F-NORMAL-SIGN]`. Read the plane's `geometry`, a `Plane` with `origin` and `normal`: let
`d = (C - origin) . normal`; set `n̂ = normal` when `d > 0` and `n̂ = -normal` otherwise, so that `C`
lies `+A/2` along `n̂` from gear A's plane. Raise naming the plane unless `|d|` is `A/2` within
1e-6 cm.

Then complete the frame of §1 from `ê` and `n̂`, in world space and cm:

```
k̂       = n̂ × ê
dirA     = cos(Sigma/2)*ê + sin(Sigma/2)*k̂        (ê turned +Sigma/2 about n̂)
dirB     = cos(Sigma/2)*ê - sin(Sigma/2)*k̂        (ê turned -Sigma/2 about n̂)
originA  = C - (A/2)*n̂                            originB = C + (A/2)*n̂
ûA       = +n̂                                     ûB      = -n̂
v̂_g      = dir_g × û_g
Theta_g(s) = s/Lambda + Phi_g
World_g(s, u, v) = origin_g + s*dir_g + (u*cos(theta) - v*sin(theta))*û_g
                                      + (u*sin(theta) + v*cos(theta))*v̂_g,   theta = Theta_g(s)
```

`û` points at the other gear: that is what makes `Phi` mean the same for both parts. Every seed
and every reference point below is a `World_g` point. A search result of S03 given as frame
components `(eX, kY, nZ)` is the world vector `eX*ê + kY*k̂ + nZ*n̂` (scaled to cm), and a plane
point `(t, z)` of a window is `C + t*across + z*n̂`, with `across = n̂ × d` in world terms.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "planeInput = design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-A/2))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-A/2))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 946,
      "last": 969,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 626,
      "last": 635,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L946–969; `spec/screwgear/fusion.md` L626–635.

## S07 `[PROSE]` The Gear B Axis Plane

As S06, offset `+A/2`: `planeInput = design.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(A/2))`,
`design.constructionPlanes.add(planeInput)`, named `Gear B Axis Plane`. Raise naming the plane
unless `C` is `A/2` from it within 1e-6 cm (`|(C - origin) . normal|` of its own `geometry`). Its
normal is not read for anything else; `n̂` comes from the Gear A Axis Plane. No later plane is
offset from the selected plane. Keep both planes, `self.axisPlanes = [gearA, gearB]`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "planeInput = design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(A/2))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(A/2))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 962,
      "last": 969,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 626,
      "last": 635,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L962–969; `spec/screwgear/fusion.md` L626–635.

## S08 `[GO]` A gear's Paths sketch

The method `buildGear`, given `index`, runs S08 to S16 for gear `g = index`, gear A then gear B;
`{gearLabel}` is `Gear A` or `Gear B`. The method `buildSweepPaths`, given `index`, is this step.

Create `paths = design.sketches.add(self.axisPlanes[g])`, named `{gearLabel} Paths`. It holds
the two lines the gear's bores are swept along in S21, both on the gear's axis, which lies in this
plane. It is not deferred `[SCREW-F-DEFER]`.

1. Four reference points `[SCREW-F-REFERENCES]`, `[PB-PROJECT-NOT-FIXED]`, at
   `origin_g + s*dir_g` for `s` = `-sOut`, `-sIn`, `sIn`, `sOut` (−19, −7.892, 7.892 and 19 mm at
   the defaults): for each, `local = paths.modelToSketchSpace(world)`, then `local.z = 0`
   `[PB-SKETCH-ZERO-Z]`, then `paths.sketchPoints.add(local)`.
2. Two solid lines sharing those points `[PB-SHARE-XOR-COINCIDENT]`, each from its negative
   station to its positive one, so its start is its negative end and it runs along `+dir_g`:
   `boreMinus = paths.sketchCurves.sketchLines.addByTwoPoints(p0, p1)` and
   `borePlus = paths.sketchCurves.sketchLines.addByTwoPoints(p2, p3)`.
3. After the second line, set all four points `isFixed = True`.

The lines carry no dimension and no constraint, and nothing else is in the sketch. Raise naming the
sketch unless `paths.isFullyConstrained`. Keep `self.pathLines[g] = {'bore-': boreMinus, 'bore+': borePlus}`.

The proof draws the four points and the two lines on the Axis Plane, holds each line's length at
`sOut - sIn` and its direction along `+dir_g`, and the gap between them at `2*sIn`, for both gears,
at the 110° crossing, a thick wall and no roof allowance.

Proof: `stepPathsSketch`.

<!-- proof-run: proofkit.Run(pathsCases, stepPathsSketch) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "paths = design.sketches.add(self.axisPlanes[g])"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "paths",
      "role": "required",
      "span": "local = paths.modelToSketchSpace(world)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "paths.sketchPoints",
      "role": "required",
      "span": "paths.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "paths.sketchCurves.sketchLines",
      "role": "required",
      "span": "boreMinus = paths.sketchCurves.sketchLines.addByTwoPoints(p0, p1)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "paths.sketchCurves.sketchLines",
      "role": "required",
      "span": "borePlus = paths.sketchCurves.sketchLines.addByTwoPoints(p2, p3)"
    }
  ],
  "citations": [
    {
      "first": 970,
      "last": 989,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 563,
      "last": 624,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 458,
      "last": 472,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 591,
      "last": 608,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L970–989; `spec/screwgear/fusion.md` L563–624; `.claude/skills/generate-gear/PLAYBOOK.md` L458–472; `.claude/skills/generate-gear/PLAYBOOK.md` L591–608.

## S09 `[GO]` A gear's Cell Sections sketch

The method `buildToothCell`, given `index`, is S09 and S10. The cell is `cell` tooth pitches of the finished ribbon at
its negative end: stations from `s0 = Z0[g] - N*P/2` to `s0 + cell*P`, `Z0` being 0 for gear A and
`assemblyPhase` for gear B, so gear B's whole ribbon is gear A's advanced `assemblyPhase` along its
own axis under its own screw motion. It is lofted through `cell*n + 1` sections, 41 at the
defaults (`n` of S03: the larger of the 2° twist count and 8).

Create `cellSketch = design.sketches.add(self.axisPlanes[g])`, named `{gearLabel} Cell Sections`
`[SCREW-F-CELL-LOFT]`, `[PB-3D-SKETCH-SECTIONS]`. Set `cellSketch.isComputeDeferred = True` right
after naming it and before its first point `[PB-SKETCH-DEFER]`, `[SCREW-F-DEFER]`. No section plane
is made.

Section `k`, for `k = 0 … cell*n`, is the cross-section at station `s_k = s0 + k*P/n`, turned by
`Theta_g(s_k)`. With `M = TOOTH_SPLINE_POINTS`, `hv = T/2` and `uB = -W/2`:

```
v_j             = -T/2 + j*T/(M - 1),     j = 0 … M - 1
Utooth(v, s)    = W/2 - H/2 + (H/2)*cos(2*pi*(s + tanSlant*v - Z0[g])/P) - Bow_cm*v^2
B0  = World_g(s_k, uB, -hv)            the back corner on the v = -T/2 face
F_j = World_g(s_k, Utooth(v_j, s_k), v_j)   the toothed points; F_0 and F_(M-1) are the toothed corners
B1  = World_g(s_k, uB, +hv)            the back corner on the v = +T/2 face
```

Each of the section's `M + 2` points is `cellSketch.sketchPoints.add(cellSketch.modelToSketchSpace(world))`
**with its `z` kept**: these points lie off the sketch's plane on purpose, at every height above and
below it, so `[PB-SKETCH-ZERO-Z]` is not applied here and only here. Four curves share them, in
this order `[PB-SHARE-XOR-COINCIDENT]`, `[PB-SKETCHCURVES]`:

- `L1 = cellSketch.sketchCurves.sketchLines.addByTwoPoints(B0, F_0)`, the face at `v = -T/2`.
- `S = cellSketch.sketchCurves.sketchFittedSplines.add(fitPoints)`, the toothed side, where
  `fitPoints = adsk.core.ObjectCollection.create()` holds the sketch points `F_0` to `F_(M-1)`, each
  put in with `fitPoints.add(F_j)` in order of `j`. Raise naming the section unless `S` is not
  `None` and `S.fitPoints.count` is `M`. Activate no tangent or curvature handle.
- `L3 = cellSketch.sketchCurves.sketchLines.addByTwoPoints(F_(M-1), B1)`, the face at `v = +T/2`.
- `L4 = cellSketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)`, the back side.

Keep each section's four curves together, in the order `L1`, `S`, `L3`, `L4`; they are what S10's
loft is fed. After the last curve of the last section, set every point in the sketch
`isFixed = True` `[PB-PROJECT-NOT-FIXED]`: every point the build added, and every item of every
spline's `fitPoints`, by `S.fitPoints.item(i)` for `i` below `S.fitPoints.count` (the reference does
not say whether the spline keeps the points it is given, so both are fixed). Nothing else goes in
the sketch: no construction line, constraint or dimension. Then set
`cellSketch.isComputeDeferred = False`, raise naming the sketch unless `cellSketch.isFullyConstrained`
`[PB-FULL-CONSTRAINT]`, and raise unless `cellSketch.profiles.count` is `cell*n + 1`. The profiles
are not what the loft is fed: nothing says which profile is which station.

Before S10, raise unless every section holds exactly the four curves above, a line, a fitted
spline, a line and a line, so the loft meets like curves with like.

The proof draws one section per case on that section's own plane, in the plane's coordinates
turned to the station's angle: the `M + 2` points fixed after the last curve, the three lines, and
the toothed side as the engine's fit spline through `F_0 … F_(M-1)`, a natural cubic through every
point. It holds the profile valid with the area of "The part"'s section, and the section a pitch
on equal to this one carried by the screw step, which is what lets S11 to S13 repeat the cell.
That Fusion reads the one sketch of all the sections fully constrained is Fusion's: it did for 41
rectangle sections on 2026-09-28, and the sketch of spline sections has not been loaded.

Proof: `stepCellSectionsSketch`.

<!-- proof-run: proofkit.Run(cellSectionCases, stepCellSectionsSketch) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "cellSketch = design.sketches.add(self.axisPlanes[g])"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "cellSketch.sketchPoints",
      "role": "required",
      "span": "cellSketch.sketchPoints.add(cellSketch.modelToSketchSpace(world))"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "cellSketch",
      "role": "required",
      "span": "cellSketch.sketchPoints.add(cellSketch.modelToSketchSpace(world))"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "cellSketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "L1 = cellSketch.sketchCurves.sketchLines.addByTwoPoints(B0, F_0)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchFittedSplines",
      "reason": null,
      "receiver": "cellSketch.sketchCurves.sketchFittedSplines",
      "role": "required",
      "span": "S = cellSketch.sketchCurves.sketchFittedSplines.add(fitPoints)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "fitPoints = adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "fitPoints",
      "role": "required",
      "span": "fitPoints.add(F_j)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "cellSketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "L3 = cellSketch.sketchCurves.sketchLines.addByTwoPoints(F_(M-1), B1)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "cellSketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "L4 = cellSketch.sketchCurves.sketchLines.addByTwoPoints(B1, B0)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.SketchPointList",
      "reason": null,
      "receiver": "S.fitPoints",
      "role": "required",
      "span": "S.fitPoints.item(i)"
    }
  ],
  "citations": [
    {
      "first": 991,
      "last": 1021,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1044,
      "last": 1118,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 347,
      "last": 386,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 878,
      "last": 902,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L991–1021; `spec/screwgear/instructions.md` L1044–1118; `spec/screwgear/fusion.md` L347–386; `.claude/skills/generate-gear/PLAYBOOK.md` L878–902.

## S10 `[GO]` A gear's cell loft, and the slant's sign check

Loft the sections in station order `[PB-LOFT]`, `[SCREW-F-CELL-LOFT]`:

1. `loftInput = design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
2. For each section in order of `k`: `collection = adsk.core.ObjectCollection.create()`, its four
   curves put in with `collection.add(curve)` in the order `L1`, `S`, `L3`, `L4`, then
   `path = design.features.createPath(collection, False)` `[PB-PATH-FROM-SKETCH]` and
   `loftInput.loftSections.add(path)`. Never `adsk.fusion.Path.create(collection, False)`, which
   raises in this component.
3. `loftFeature = design.features.loftFeatures.add(loftInput)`.

Raise naming the gear unless `loftFeature.bodies.count` is 1 and `loftFeature.bodies.item(0)` is
`isSolid` `[PB-EMPTY-RESULT]`. That body is the cell, `cellBody`. Fusion's loft through more than two
sections is smooth between them; it passes through every section.

**The slant's sign check** `[PB-SELF-DIAGNOSING]`, after the loft and before S11. With
`sc = Z0[g] + P*ceil((s0 + P/2 - Z0[g])/P)`, the first crest station at least half a pitch into
the cell, for each face sign `sf` of −1 and +1:

```
v_p = sf*(T/2 - 0.025 cm)
u_p = W/2 - Bow_cm*v_p^2 - 0.025 cm          a quarter millimetre inside the face and under the crest
on-ridge probe : World_g(sc - tanSlant*v_p, u_p, v_p)
off-ridge probe: World_g(sc + tanSlant*v_p, u_p, v_p)
for each probe at its station s:
    m      = Utooth(v_p, s) - u_p                       under the input slant
    mWrong = the same with tanSlant negated
    used   = |m| >= 0.01 cm and |mWrong| >= 0.01 cm and the two differ in sign
```

Read `cellBody.pointContainment(probe)` for each used probe: it must be
`adsk.fusion.PointContainment.PointInsidePointContainment` when `m > 0` and
`adsk.fusion.PointContainment.PointOutsidePointContainment` when `m < 0`; otherwise raise naming
the gear, the probe and what it read. When no probe is used, as at a zero slant, log with
`futil.log(...)` `[PB-LOGGING]` that `{gearLabel}: the tooth slant's sign was not checked`. At the
defaults all four are used: on the ridge each stands 0.25 mm inside the tooth under the right sign
and 2.13 mm outside under the wrong one, and off the ridge the reverse; gear A's stand 1.84 and
3.41 mm from the cell's start, inside its 10.5 mm.

The proof lofts the same `cell*n + 1` sections two at a time: decad lofts only between two
sections, ruled, so each pair is lofted as a sheet, the end sections are capped, and the sheets
are stitched into one solid. It holds the cell's volume within 4% under the exact helicoid's
(2.1% at the defaults), every vertex within the crest rectangle's reach of the axis and between the
cell's end stations, and the four probes as above, reading containment off the stand-in's mesh: all
four used at the defaults with those margins, none at the straight ridge, and every used probe on
the right side at a negative slant, a 400 mm lead (eight steps a tooth), a 20 mm lead and 30°
mounts.

Proof: `stepCellLoft`.

<!-- proof-run: proofkit3d.RunSolid(cellLoftCases, stepCellLoft, assertCellLoft) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "loftInput = design.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "collection = adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "collection",
      "role": "required",
      "span": "collection.add(curve)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "path = design.features.createPath(collection, False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.LoftSections",
      "reason": null,
      "receiver": "loftInput.loftSections",
      "role": "required",
      "span": "loftInput.loftSections.add(path)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.Path",
      "reason": "PB-PATH-FROM-SKETCH: Path.create raises in a sub-component",
      "receiver": "adsk.fusion.Path",
      "role": "forbidden",
      "span": "adsk.fusion.Path.create(collection, False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "design.features.loftFeatures",
      "role": "required",
      "span": "loftFeature = design.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loftFeature.bodies",
      "role": "required",
      "span": "loftFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "cellBody",
      "role": "required",
      "span": "cellBody.pointContainment(probe)"
    },
    {
      "condition": null,
      "name": "log",
      "owner": null,
      "reason": "lib/fusion360utils defines log as futil.log",
      "receiver": "futil",
      "role": "inherited",
      "span": "futil.log(...)"
    }
  ],
  "citations": [
    {
      "first": 1113,
      "last": 1156,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 376,
      "last": 403,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 742,
      "last": 746,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 840,
      "last": 850,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1113–1156; `spec/screwgear/fusion.md` L376–403; `.claude/skills/generate-gear/PLAYBOOK.md` L742–746; `.claude/skills/generate-gear/PLAYBOOK.md` L840–850.

## S11 `[GO]` A doubling round: copy the body

The method `repeatCellByDoubling`, given `index`, is S11 to S16. The finished ribbon is the cell repeated under the
**screw step** `Step(k)`: a turn of `k*P/Lambda` about the gear's axis composed with an advance of
`k*P` along it, `k` a number of teeth. With `q` whole cells and `r` remainder teeth (S03), the
`q` cells are built by doubling. A **round** is copy (this step), move (S12), join (S13): copy the
current body, move the copy by `Step(m*cell)` where `m` is the number of cells the body holds, join
the two, and the body holds `2m` cells. The schedule:

```
body = cellBody; m = 1; asides = []
for each bit of q below its top bit, lowest first:
    if that bit is set: asides.append((copy of body, m))      # an aside: a copy kept unmoved
    copy the body; move the copy by Step(m*cell); join it to the body; m = 2*m
for (aside, cells) in asides, largest cells first:
    move the aside by Step(m*cell); join it to the body; m = m + cells
# now m == q.  When q == 1 there is no round.
```

That is the floor of log2 q, plus the number of set bits in q, less one, rounds: 5 at the defaults (`q = 17`: one aside of the
single cell taken before the first doubling, four doublings to 16 cells, and the aside moved by
`Step(64)`). An aside is a copy too, made by this step, and is moved by S12 and joined by S13.

Each copy is `copyFeature = design.features.copyPasteBodies.add(body)`, and the copy is
`copyFeature.bodies.item(0)`: the feature is a `CopyPasteBody`, whose own `sourceBody` is the
original, so never read the copy from `sourceBody` `[SCREW-F-COPY-BODY]`.

The proof copies the cell with decad's Duplicate and holds the copy vertex for vertex and volume
for volume against its source, in a document of its own: a body beside its exact copy is a pair
decad's verification cannot classify, so the harness gates the copy's geometry, the cell, alone.

Proof: `stepCopyBody`.

<!-- proof-run: proofkit3d.RunSolid(copyCases, stepCopyBody, assertCopyBody) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "design.features.copyPasteBodies",
      "role": "required",
      "span": "copyFeature = design.features.copyPasteBodies.add(body)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "copyFeature.bodies",
      "role": "required",
      "span": "copyFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1158,
      "last": 1189,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1198,
      "last": 1199,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 338,
      "last": 345,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1158–1189; `spec/screwgear/instructions.md` L1198–1199; `spec/screwgear/fusion.md` L338–345.

## S12 `[GO]` A doubling round: screw-move the copy

Build `Step(k)` for gear `g` from two matrices `[SCREW-F-SCREW-STEP]`, never by assigning a
rotation matrix's translation:

```python
axisVector = adsk.core.Vector3D.create(dir_g.x, dir_g.y, dir_g.z)        # unit
axisPoint  = adsk.core.Point3D.create(origin_g.x, origin_g.y, origin_g.z) # on the axis
rot = adsk.core.Matrix3D.create()
rot.setToRotation(k * P / Lambda, axisVector, axisPoint)
shift = axisVector.copy()
shift.scaleBy(k * P)
mov = adsk.core.Matrix3D.create()
mov.translation = shift
rot.transformBy(mov)
```

The calls are `rot.setToRotation(angle, axisVector, axisPoint)`, `axisVector.copy()`,
`shift.scaleBy(k * P)` and `rot.transformBy(mov)`. Build the shift vector and assign it whole: the
getter of `translation` may hand back a copy. Move with
`bodies = adsk.core.ObjectCollection.create()`, `bodies.add(copyBody)`,
`moveInput = design.features.moveFeatures.createInput2(bodies)`,
`moveInput.defineAsFreeMove(rot)`, `design.features.moveFeatures.add(moveInput)`
`[PB-MOVE-ROTATE]`. A screw step is never zero for `k >= 1`, so no zero guard is needed; do not
fold a `k = 0` case into the loop, because Fusion refuses an identity move.

The proof moves a copy of the cell by `Step(m*cell)` for one, two and sixteen cells, the last the
aside's move at the defaults, on both gears, and holds every vertex of the moved copy within
1e-6 mm of the analytic sections of the cell it lands on: the ribbon is invariant under its screw
step, so every placement is exact.

Proof: `stepScrewMove`.

<!-- proof-run: proofkit3d.RunSolid(moveCases, stepScrewMove, assertScrewMove) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "setToRotation",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(angle, axisVector, axisPoint)"
    },
    {
      "condition": null,
      "name": "copy",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "axisVector",
      "role": "required",
      "span": "axisVector.copy()"
    },
    {
      "condition": null,
      "name": "scaleBy",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "shift",
      "role": "required",
      "span": "shift.scaleBy(k * P)"
    },
    {
      "condition": null,
      "name": "transformBy",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "rot",
      "role": "required",
      "span": "rot.transformBy(mov)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "bodies = adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "bodies",
      "role": "required",
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "moveInput = design.features.moveFeatures.createInput2(bodies)"
    },
    {
      "condition": null,
      "name": "defineAsFreeMove",
      "owner": "adsk.fusion.MoveFeatureInput",
      "reason": null,
      "receiver": "moveInput",
      "role": "required",
      "span": "moveInput.defineAsFreeMove(rot)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "design.features.moveFeatures",
      "role": "required",
      "span": "design.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1158,
      "last": 1205,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 301,
      "last": 336,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 822,
      "last": 831,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1158–1205; `spec/screwgear/fusion.md` L301–336; `.claude/skills/generate-gear/PLAYBOOK.md` L822–831.

## S13 `[GO]` A doubling round: join the copy to the body

`tools = adsk.core.ObjectCollection.create()`, `tools.add(movedCopy)`,
`combineInput = design.features.combineFeatures.createInput(body, tools)`, then set
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation` and
`combineInput.isKeepToolBodies = False`, then
`combineFeature = design.features.combineFeatures.add(combineInput)` `[SCREW-F-JOIN]`. Raise naming
the gear, the round and the count unless `combineFeature.bodies.count` is 1 `[PB-EMPTY-RESULT]`,
`[PB-SELF-DIAGNOSING]`; `combineFeature.bodies.item(0)` is the body from then on. Each move is by a
whole number of teeth already built, so each join meets its neighbour at one shared planar
cross-section and leaves no sliver and no overlap.

The proof cannot run the join: the two pieces are stitched solids, which no decad boolean takes,
and they meet face to face on the shared section, which decad's union refuses even for plain
prisms. It builds the body and its moved copy, holds the copy's first section on the body's last
and the two on either side of it, and builds what the join leaves, the ribbon over both spans,
whose volume it holds to the two pieces' sum to 1e-6 mm³.

Proof: `stepJoinBodies`.

<!-- proof-run: proofkit3d.RunSolid(joinCases, stepJoinBodies, assertJoinBodies) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "tools = adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "tools",
      "role": "required",
      "span": "tools.add(movedCopy)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "combineInput = design.features.combineFeatures.createInput(body, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "design.features.combineFeatures",
      "role": "required",
      "span": "combineFeature = design.features.combineFeatures.add(combineInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "combineFeature.bodies",
      "role": "required",
      "span": "combineFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1166,
      "last": 1203,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 551,
      "last": 561,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1166–1203; `spec/screwgear/fusion.md` L551–561.

## S14 `[GO]` The remainder's sections sketch, when `r > 0`

When `r > 0`, the last `r` teeth are a second, shorter cell built where they belong rather than
moved there, after the last round. Its sketch is `{gearLabel} Cell Remainder`, on
`self.axisPlanes[g]`, drawn exactly as S09 with `cell` replaced by `r`: `r*n + 1` sections at
stations `s0 + q*cell*P + k*P/n` for `k = 0 … r*n`, deferred, every point fixed after the last
curve, and checked fully constrained with `r*n + 1` profiles. The defaults have no remainder; 69
teeth in four-tooth cells would have one of one tooth.

The proof draws sections of the remainder at one, two and three teeth over, as S09's proof does.

Proof: `stepRemainderSectionsSketch`.

<!-- proof-run: proofkit.Run(remainderSectionCases, stepRemainderSectionsSketch) -->

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 1191,
      "last": 1197,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1191–1197.

## S15 `[GO]` The remainder's loft, when `r > 0`

The loft of S10 over the remainder's sections, in station order, raising unless it leaves one
solid body. No slant check is run on it. Its first section is the body's last, so S16's join meets
at a shared cross-section like every other.

The proof lofts the remainder as S10's proof does, holds its volume within 4% under the helicoid's,
its vertices on the analytic sections, and its first section on the cell's last carried by the
screw step of the whole cells before it.

Proof: `stepRemainderLoft`.

<!-- proof-run: proofkit3d.RunSolid(remainderCases, stepRemainderLoft, assertRemainderLoft) -->

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 1191,
      "last": 1197,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1191–1197.

## S16 `[GO]` The remainder's join, and the gear body's name

When `r > 0`, join the remainder to the body after the last round, exactly as S13, the body the
target and the remainder the one tool, raising unless the combine feature's `bodies.count` is 1.
After the last join (or the last round when `r = 0`, or the cell itself when `q = 1` and `r = 0`),
name the body `Gear A` or `Gear B` (`body.name`) and keep it as `self.gearBodies[g]`.

The proof stands in for the join as S13's does, with the last whole cell as the body and the
remainder as the piece the screw step carries after it.

Proof: `stepRemainderJoin`.

<!-- proof-run: proofkit3d.RunSolid(remainderCases, stepRemainderJoin, assertRemainderJoin) -->

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 1191,
      "last": 1203,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1191–1203.

## S17 `[GO]` The Sleeve sketch

The method `buildCage` is S17 to S26. The cage is the **sleeve**: one tube about the frame's axis, the four
bores cut through its wall by twisted sweeps, and up to two windows cut through the wall by
extrusions `[SCREW-F-SLEEVE]`. Heights are along `n̂` from `C`; gear B's axis is on the `+n̂` side.

Create `sleeve = design.sketches.add(self.targetPlane)` on the selected plane itself
`[PB-USE-SELECTED-PLANE]`, named `Sleeve`; not deferred. With
`centre = sleeve.modelToSketchSpace(C)` and `centre.z = 0` `[PB-SKETCH-ZERO-Z]`, draw two circles,
`circle = sleeve.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)` with radius `Ri` and
then `Ro`. For each: set `circle.centerSketchPoint.isFixed = True` `[PB-CIRCLE-CENTER]` (the two
centres are separate points at one place, both fixed, with no coincident between them), and add
`sleeve.sketchDimensions.addDiameterDimension(circle, textPoint)` with its `parameter.value` set to
`2*radius`, the text point on the circle, off the centre `[PB-RADIAL-DIM]`:
`textPoint = adsk.core.Point3D.create(centre.x + radius, centre.y, 0)`. Nothing else is in the
sketch. Raise naming the sketch unless `sleeve.isFullyConstrained`.

The sketch has two profiles, the inner disc and the ring. The ring is the profile whose
`profileLoops.count` is 2: iterate `sleeve.profiles`, take the one profile with two loops, and
raise with the counts unless there is exactly one `[PB-EMPTY-RESULT]`. `find_profile_by_curve_counts`
cannot pick it, because it treats a circle as a curve type that disqualifies a loop.

The proof draws the two circles with their centres fixed and their diameters, picks the one
profile with a hole and holds its area at `pi*(Ro^2 - Ri^2)`. It also runs S03's range checks on
every case, the case being one the build accepts, and holds the wall between the bores at
5.067 mm across the `-e` gap at the defaults, by S03's algorithm.

Proof: `stepSleeveSketch`.

<!-- proof-run: proofkit.Run(sleeveSketchCases, stepSleeveSketch) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "sleeve = design.sketches.add(self.targetPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sleeve",
      "role": "required",
      "span": "centre = sleeve.modelToSketchSpace(C)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sleeve.sketchCurves.sketchCircles",
      "role": "required",
      "span": "circle = sleeve.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sleeve.sketchDimensions",
      "role": "required",
      "span": "sleeve.sketchDimensions.addDiameterDimension(circle, textPoint)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "textPoint = adsk.core.Point3D.create(centre.x + radius, centre.y, 0)"
    }
  ],
  "citations": [
    {
      "first": 1208,
      "last": 1246,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 503,
      "last": 525,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 451,
      "last": 457,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 670,
      "last": 674,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1208–1246; `spec/screwgear/fusion.md` L503–525; `.claude/skills/generate-gear/PLAYBOOK.md` L451–457; `.claude/skills/generate-gear/PLAYBOOK.md` L670–674.

## S18 `[GO]` Extrude the tube

`tubeInput = design.features.extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `tubeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise), False)`: `False`
makes the value each side's length `[PB-THROUGH-CUT]`, so the tube runs from `-cageRise` to
`+cageRise` along `n̂`, 37.5 mm tall at the defaults with a flat 565.5 mm² ring at each end. Then
`tubeFeature = design.features.extrudeFeatures.add(tubeInput)`. Raise naming the sleeve unless
`tubeFeature.bodies.count` is 1; `tubeFeature.bodies.item(0)` is `self.cageBody` from here on.

The sleeve prints standing on its `−n̂` end, the end below the selected plane, with no support.

The proof extrudes the ring symmetric and holds the volume at `pi*(Ro^2 - Ri^2)*2*cageRise` and the
bounding box at `C ± (Ro, Ro, cageRise)`.

Proof: `stepExtrudeTube`.

<!-- proof-run: proofkit3d.RunSolid(tubeCases, stepExtrudeTube, assertExtrudeTube) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "tubeInput = design.features.extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "setSymmetricExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "tubeInput",
      "role": "required",
      "span": "tubeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise), False)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "tubeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise), False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "tubeFeature = design.features.extrudeFeatures.add(tubeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "tubeFeature.bodies",
      "role": "required",
      "span": "tubeFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1227,
      "last": 1246,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 511,
      "last": 525,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1227–1246; `spec/screwgear/fusion.md` L511–525.

## S19 `[PROSE]` A bore's plane

S19 to S22 run once per bore, in S03's order: gear A `-R`, gear A `+R`, gear B `-R`, gear B `+R`.
For bore `(g, sigma)` the path line is `line = self.pathLines[g]['bore-']` for `-R` and
`self.pathLines[g]['bore+']` for `+R`. Its section plane is
`planeInput = design.constructionPlanes.createInput()`,
`planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))`, with the line passed
directly `[PB-CONSTRUCTION-PLANES]`, then `design.constructionPlanes.add(planeInput)`, named
`{gearLabel} Bore -R Plane` or `{gearLabel} Bore +R Plane`. It stands at the span's negative end,
station `s0 = -sOut` for `-R` and `sIn` for `+R`, square to the axis; Fusion put such a plane's
origin on the station to four decimals of a millimetre.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "planeInput = design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "design.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1304,
      "last": 1310,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 429,
      "last": 437,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1304–1310; `spec/screwgear/fusion.md` L429–437.

## S20 `[GO]` A bore's section sketch: the rectangle scheme

Create `boreSketch = design.sketches.add(borePlane)`, named `{gearLabel} Bore -R` or
`{gearLabel} Bore +R`, and set `boreSketch.isComputeDeferred = True` right away
`[PB-SKETCH-DEFER]`, `[SCREW-F-DEFER]`. Everything below is at the plane's station `s0`, with
`theta = Theta_g(s0)`, `uB = -hw`, `uF = hw`, and the bore's own `vLo` and `vHi` (S03). Every point
is a `World_g` point mapped with `boreSketch.modelToSketchSpace(world)` and given `z = 0`
`[PB-SKETCH-ZERO-Z]`. Nothing is projected into it `[PB-PROJECT-NOT-FIXED]`.

1. **References.** Two reference points: `O` at `origin_g + s0*dir_g`, where the path pierces the
   plane, and `Cp` at `origin_g + s0*dir_g + (A/2)*û_g`, both added with
   `boreSketch.sketchPoints.add(local)`. The construction line
   `Ru = boreSketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)`, `Ru.isConstruction = True`.
2. **The spine.** The construction line `K` from `O` to `E`, `E` seeded at `World_g(s0, uF, 0)`:
   `K = boreSketch.sketchCurves.sketchLines.addByTwoPoints(O, eSeed)` with `eSeed` the raw mapped
   `Point3D`, `K.isConstruction = True`, and `E = K.endSketchPoint`. Then set `O.isFixed = True` and
   `Cp.isFixed = True`, after `Ru` and `K` and before any dimension.
3. **The rectangle.** Four solid lines sharing their corners, every seed the solved point
   `[PB-SEED-NEAR]`, `[PB-SHARE-XOR-COINCIDENT]` (no coincident on a corner):
   `L1 = addByTwoPoints(c0, c1)` with raw seeds `c0 = World_g(s0, uB, vLo)` and
   `c1 = World_g(s0, uF, vLo)`; `L2 = addByTwoPoints(L1.endSketchPoint, c2)` with
   `c2 = World_g(s0, uF, vHi)`; `L3 = addByTwoPoints(L2.endSketchPoint, c3)` with
   `c3 = World_g(s0, uB, vHi)`; `L4 = addByTwoPoints(L3.endSketchPoint, L1.startSketchPoint)`, each
   on `boreSketch.sketchCurves.sketchLines`.
4. **The length.** `boreSketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
   with `parameter.value = uF`, the text point at `World_g(s0, uF/2, 0.1 cm)` mapped, `z = 0`.
5. **The angle** `[PB-ANGULAR-DIM]`. When `|sin(theta)| >= sqrt(1/2)` the two rays are `r1 = +û`
   from `O` along `Ru` (toward `Cp`) and `r2 = O→E` along `K`, meeting at `O`, and the dimension is
   `boreSketch.sketchDimensions.addAngularDimension(Ru, K, textPoint)`. Otherwise the rays are
   `r1 = +û` along `Ru`'s line and `r2` along `L2` from its start to its end, meeting where `L2`'s line
   crosses `Ru`'s, at `X = origin_g + s0*dir_g + (uF/cos(theta))*û_g`, and the dimension is
   `boreSketch.sketchDimensions.addAngularDimension(Ru, L2, textPoint)`. In world terms
   `r1 = û_g`, and `r2` is `cos(theta)*û_g + sin(theta)*v̂_g` for `K` or
   `-sin(theta)*û_g + cos(theta)*v̂_g` for `L2`. The value is the angle between the rays,
   `acos(r1 . r2)`, which always lies in 45°–135°; set `parameter.value` to it in radians. The
   text point is inside that wedge: the vertex (`O` or `X`) plus `(uF/2)` times the unit bisector of
   `r1` and `r2`, mapped, `z = 0`.
6. **The rectangle's constraints** `[PB-OFFSET-DIM]`, `[PB-NO-OVERCONSTRAIN]`:
   `boreSketch.geometricConstraints.addParallel(L1, K)` and
   `boreSketch.sketchDimensions.addOffsetDimension(K, L1, textPoint)` with `parameter.value = -vLo`;
   `boreSketch.geometricConstraints.addParallel(L3, K)` and
   `boreSketch.sketchDimensions.addOffsetDimension(K, L3, textPoint)` with `parameter.value = vHi`;
   `boreSketch.geometricConstraints.addCoincident(E, L2)`;
   `boreSketch.geometricConstraints.addPerpendicular(L2, K)`;
   `boreSketch.geometricConstraints.addParallel(L4, L2)` and
   `boreSketch.sketchDimensions.addOffsetDimension(L2, L4, textPoint)` with
   `parameter.value = uF - uB`. Each offset's text point is the midpoint of its second line's two
   seeds, mapped, `z = 0`. All values are magnitudes; the side is the seed's
   `[PB-DIM-VALUE-SEMANTICS]`.

Ten degrees of freedom, `E` and the four corners, against ten rows: five dimensions and five
constraints. Set `boreSketch.isComputeDeferred = False`, then raise naming the sketch unless
`boreSketch.isFullyConstrained`. The profile is the rectangle, the one loop of four lines:
`profile = find_profile_by_curve_counts(boreSketch, lines=4)` `[PB-PROFILE-MATCH]`; the
construction lines bound nothing. At the defaults gear A's `-R` profile stands at −138.2° and takes
the `L2` reference, and its `+R` profile at 57.4° and takes `K`.

The proof draws the scheme on the bore's plane in the plane's coordinates. The engine's signed
offset carries Fusion's parallel and offset dimension as one constraint, with the side as its sign,
and its signed angle runs from `Ru`'s start→end to the other line's. It holds every corner where it
was seeded, the unsigned angle in 45°–135° and equal to the dimension's value, the profile one valid
loop of four lines with the rectangle's area, and the sketch fully constrained and unambiguous, at
all four bores, at the 14° third print, with no roof allowance, level at 30°, at unequal angles,
at 110° and at a 60 mm lead.

Proof: `stepBoreSectionSketch`.

<!-- proof-run: proofkit.Run(boreSectionCases, stepBoreSectionSketch) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "boreSketch = design.sketches.add(borePlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "boreSketch",
      "role": "required",
      "span": "boreSketch.modelToSketchSpace(world)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "boreSketch.sketchPoints",
      "role": "required",
      "span": "boreSketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "boreSketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "Ru = boreSketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "boreSketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "K = boreSketch.sketchCurves.sketchLines.addByTwoPoints(O, eSeed)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "L1 = addByTwoPoints(c0, c1)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "L2 = addByTwoPoints(L1.endSketchPoint, c2)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "L3 = addByTwoPoints(L2.endSketchPoint, c3)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "L4 = addByTwoPoints(L3.endSketchPoint, L1.startSketchPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "boreSketch.sketchDimensions",
      "role": "required",
      "span": "boreSketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "boreSketch.sketchDimensions",
      "role": "required",
      "span": "boreSketch.sketchDimensions.addAngularDimension(Ru, K, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "boreSketch.sketchDimensions",
      "role": "required",
      "span": "boreSketch.sketchDimensions.addAngularDimension(Ru, L2, textPoint)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "boreSketch.geometricConstraints",
      "role": "required",
      "span": "boreSketch.geometricConstraints.addParallel(L1, K)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "boreSketch.sketchDimensions",
      "role": "required",
      "span": "boreSketch.sketchDimensions.addOffsetDimension(K, L1, textPoint)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "boreSketch.geometricConstraints",
      "role": "required",
      "span": "boreSketch.geometricConstraints.addParallel(L3, K)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "boreSketch.sketchDimensions",
      "role": "required",
      "span": "boreSketch.sketchDimensions.addOffsetDimension(K, L3, textPoint)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "boreSketch.geometricConstraints",
      "role": "required",
      "span": "boreSketch.geometricConstraints.addCoincident(E, L2)"
    },
    {
      "condition": null,
      "name": "addPerpendicular",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "boreSketch.geometricConstraints",
      "role": "required",
      "span": "boreSketch.geometricConstraints.addPerpendicular(L2, K)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "boreSketch.geometricConstraints",
      "role": "required",
      "span": "boreSketch.geometricConstraints.addParallel(L4, L2)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "boreSketch.sketchDimensions",
      "role": "required",
      "span": "boreSketch.sketchDimensions.addOffsetDimension(L2, L4, textPoint)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": "lib/geargen/utilities.py defines find_profile_by_curve_counts",
      "receiver": null,
      "role": "inherited",
      "span": "profile = find_profile_by_curve_counts(boreSketch, lines=4)"
    }
  ],
  "citations": [
    {
      "first": 1326,
      "last": 1364,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 429,
      "last": 437,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 655,
      "last": 669,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1326–1364; `spec/screwgear/fusion.md` L429–437; `.claude/skills/generate-gear/PLAYBOOK.md` L655–669.

## S21 `[GO]` A bore's twisted sweep cut, and its check

`path = design.features.createPath(line, False)` on the bore's path line `[PB-PATH-FROM-SKETCH]`,
`[SCREW-F-TWISTED-SLOT]`, then
`sweepInput = design.features.sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`sweepInput.twistAngle = adsk.core.ValueInput.createByReal((sOut - sIn)/Lambda)`, positive (80.78°
at the defaults) `[PB-SWEEP-TWIST]`, and `sweepInput.participantBodies = [self.cageBody]`; set
nothing else (not `orientation`, not `solidTwistAxis`, no rail). Then
`sweepFeature = design.features.sweepFeatures.add(sweepInput)`. The ribbons pass through the
channel and are not participants, so the cut leaves them whole.

The sign is positive because the path runs along `+dir_g` with the profile at its start, the
section's angle `s/Lambda + Phi_g` grows with `s`, and a positive `twistAngle` turns the profile
that way, as measured on 2026-09-28. Each profile starts in air, in the hollow for a `+R` bore and
outside the tube for a `-R` bore.

**The sweep's sense is checked** `[SCREW-F-SWEEP-CHECK]`, `[PB-SELF-DIAGNOSING]`: raise naming the
bore and the count unless `sweepFeature.bodies.count` is 1; `sweepFeature.bodies.item(0)` is the
cage from then on. Then, at the crossing `sc = sigma*cageRadius`, with
`û(sc) = cos(Theta_g(sc))*û_g + sin(Theta_g(sc))*v̂_g`, both probes
`origin_g + sc*dir_g + (W/2 + clearance/2)*û(sc)` and `origin_g + sc*dir_g - (W/2 + clearance/2)*û(sc)`
must read `adsk.fusion.PointContainment.PointOutsidePointContainment` from
`self.cageBody.pointContainment(probe)`; otherwise raise naming the bore and the containment read.
Under the wrong sense the channel at the crossing would stand turned `2*(sc - s0)/Lambda` from the
right one (103° for `+R`, 58° for `-R`), which puts both probes in the wall.

The proof cannot run this cut. decad has no twisted sweep, so the channel stands in as a chain of
two-section lofts through the sweep's own section turned by `s/Lambda + Phi_g` at a derived count
of sections, no two more than 5° of twist apart nor more than `2*acos(1 - 0.04*clearance/c)`, 18 at
the defaults; and decad refuses the cut of that chain from the curved tube. It holds that the
profile starts and ends in air, the channel's volume within 8% under the rectangle carried along
the span, and both probes in the tube's wall and in the exact channel under the right sense and
outside it under the wrong one, reading the stand-in's own mesh too wherever the probe stands
0.1 mm or more inside the channel's wall.

Proof: `stepCutBore`.

<!-- proof-run: proofkit3d.RunSolid(boreCutCases, stepCutBore, assertCutBore) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "design.features",
      "role": "required",
      "span": "path = design.features.createPath(line, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "sweepInput = design.features.sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "sweepInput.twistAngle = adsk.core.ValueInput.createByReal((sOut - sIn)/Lambda)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "design.features.sweepFeatures",
      "role": "required",
      "span": "sweepFeature = design.features.sweepFeatures.add(sweepInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "sweepFeature.bodies",
      "role": "required",
      "span": "sweepFeature.bodies.item(0)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "self.cageBody",
      "role": "required",
      "span": "self.cageBody.pointContainment(probe)"
    }
  ],
  "citations": [
    {
      "first": 1248,
      "last": 1264,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1304,
      "last": 1325,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1365,
      "last": 1402,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 412,
      "last": 487,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 851,
      "last": 869,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1248–1264; `spec/screwgear/instructions.md` L1304–1325; `spec/screwgear/instructions.md` L1365–1402; `spec/screwgear/fusion.md` L412–487; `.claude/skills/generate-gear/PLAYBOOK.md` L851–869.

## S22 `[PROSE]` The roof allowance's print note

After the four bores, log with `futil.log(...)` `[PB-LOGGING]` exactly
`Print the cage standing on its end below the selected plane: the roof allowance is on the bridged roofs that way up.`,
so the print orientation is explicit even when the bore marks are hard to see.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "log",
      "owner": null,
      "reason": "lib/fusion360utils defines log as futil.log",
      "receiver": "futil",
      "role": "inherited",
      "span": "futil.log(...)"
    }
  ],
  "citations": [
    {
      "first": 1295,
      "last": 1302,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1295–1302.

## S23 `[PROSE]` The Window Plane

When `self.windows` is empty, make no Window Plane and skip S24 and S25. Otherwise make one plane,
named `Window Plane`, through the frame's axis and square to the windows' facing `d`
`[PB-CONSTRUCTION-PLANES]`, `[SCREW-F-SLEEVE]`:

- For windows facing `±k̂` (`Sigma <= 90°`): `planeInput = design.constructionPlanes.createInput()`,
  `planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.targetPlane)`,
  the plane through the Anchor Line square to the selected plane, which holds `C`, `ê` and `n̂`.
- For windows facing `±ê` (past 90°):
  `planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))`, the plane
  square to the Anchor Line through its midpoint, `C`.

Then `windowPlane = design.constructionPlanes.add(planeInput)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "planeInput = design.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "setByAngle",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.targetPlane)"
    },
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.targetPlane)"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "design.constructionPlanes",
      "role": "required",
      "span": "windowPlane = design.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1588,
      "last": 1593,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 526,
      "last": 533,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1588–1593; `spec/screwgear/fusion.md` L526–533.

## S24 `[GO]` A Window sketch

For each window of `self.windows`, the one facing `d` before the one facing `-d`, create
`windowSketch = design.sketches.add(windowPlane)`, named `Window {d}` with `{d}` its facing name
(`Window +k`, `Window -k`, `Window +e` or `Window -e`); not deferred. For each corner `(t, z)` of
its hexagon, in order, add one reference point at the world point `C + t*across + z*n̂`:
`local = windowSketch.modelToSketchSpace(world)`, `local.z = 0` `[PB-SKETCH-ZERO-Z]`,
`windowSketch.sketchPoints.add(local)`. Draw one solid line from each corner to the next, the last
back to the first, sharing the points: `windowSketch.sketchCurves.sketchLines.addByTwoPoints(p_i, p_next)`
`[PB-SHARE-XOR-COINCIDENT]`. Then set every point `isFixed = True`. Nothing else is in the sketch.
Raise naming the sketch unless `windowSketch.isFullyConstrained`, raise unless
`windowSketch.profiles.count` is 1, and take `profile = windowSketch.profiles.item(0)`
`[PB-SINGLE-PROFILE]`. The two windows are two sketches because their hexagons cross on the shared
plane.

The proof draws the hexagon the window search finds, by S03's algorithm, and holds the search's
`lo`, `hi`, `bottom`, `top`, `left`, `right`, corners and area at the defaults to the figures S03
quotes; it holds the profile valid with the hexagon's area and every edge upright or at 45°, for
both windows at the defaults, the third print, 110° (where the windows face `±ê`), a 5 mm
`collarWall` and a sleeve scaled by 0.75.

Proof: `stepWindowSketch`.

<!-- proof-run: proofkit.Run(windowSketchCases, stepWindowSketch) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "design.sketches",
      "role": "required",
      "span": "windowSketch = design.sketches.add(windowPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "windowSketch",
      "role": "required",
      "span": "local = windowSketch.modelToSketchSpace(world)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "windowSketch.sketchPoints",
      "role": "required",
      "span": "windowSketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "windowSketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "windowSketch.sketchCurves.sketchLines.addByTwoPoints(p_i, p_next)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.Profiles",
      "reason": null,
      "receiver": "windowSketch.profiles",
      "role": "required",
      "span": "profile = windowSketch.profiles.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1554,
      "last": 1566,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1588,
      "last": 1600,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 534,
      "last": 537,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1554–1566; `spec/screwgear/instructions.md` L1588–1600; `spec/screwgear/fusion.md` L534–537.

## S25 `[GO]` A window's extrude cut, and its check

Before the cut, take the probe `C + tc*across + zc*n̂ + ((a0(tc) + a1(tc))/2)*d`, with `(tc, zc)` the
average of the hexagon's corners and `a0`, `a1` of S03 (in cm): the middle of the wall where the
window goes. Raise naming the window unless `self.cageBody.pointContainment(probe)` reads
`adsk.fusion.PointContainment.PointInsidePointContainment` `[PB-SELF-DIAGNOSING]`.

`cutInput = design.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
then
`cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro + 0.1)), direction)`,
`cutInput.participantBodies = [self.cageBody]` so the ribbons in the hollow are left whole
`[PB-THROUGH-CUT]`, then `cutFeature = design.features.extrudeFeatures.add(cutInput)`. `direction`
is `adsk.fusion.ExtentDirections.PositiveExtentDirection` when
`windowSketch.modelToSketchSpace(C + d)` has a positive `z`, that is when `d` points to the
sketch's positive side, and `adsk.fusion.ExtentDirections.NegativeExtentDirection` otherwise. The
cut runs one way only: the same hexagon on the far side of the axis is the other window's side of
the wall, where the bores stand.

After the cut, raise naming the window and the count unless `cutFeature.bodies.count` is 1
`[PB-EMPTY-RESULT]`, take `cutFeature.bodies.item(0)` as the cage, and raise naming the window and
what it read unless the same probe now reads
`adsk.fusion.PointContainment.PointOutsidePointContainment`. A cut extruded the wrong way leaves
the probe inside.

The proof cuts each window from the plain tube: decad refuses a second cut from the curved tube.
The window search keeps every face of the cut `collarWall` from every bore's channel, and the
proof holds that at the probe, so the cut is the same with the bores. It picks the direction by the
build's rule on a Window Plane facing either way, and holds the probe in the wall before the cut
and outside the cage after it, and the cage's volume at the tube's less the wall over the hexagon.

Proof: `stepCutWindow`.

<!-- proof-run: proofkit3d.RunSolid(windowCutCases, stepCutWindow, assertCutWindow) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "self.cageBody",
      "role": "required",
      "span": "self.cageBody.pointContainment(probe)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "cutInput = design.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "cutInput",
      "role": "required",
      "span": "cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro + 0.1)), direction)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro + 0.1)), direction)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro + 0.1)), direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "design.features.extrudeFeatures",
      "role": "required",
      "span": "cutFeature = design.features.extrudeFeatures.add(cutInput)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "windowSketch",
      "role": "required",
      "span": "windowSketch.modelToSketchSpace(C + d)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "cutFeature.bodies",
      "role": "required",
      "span": "cutFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1602,
      "last": 1624,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 538,
      "last": 549,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1602–1624; `spec/screwgear/fusion.md` L538–549.

## S26 `[PROSE]` The cage's order and body count

The tube is the first body. The four bores are cut from it in S03's order, each with the cage as
its only participant, then the windows, `d` before `-d`. Every cut leaves exactly one body,
counted as the feature's `bodies.count`, and that body is `self.cageBody` from then on. Four
marks are then extruded on the top end and joined to the cage: circles beside `+R` bores and
squares beside `-R` bores. Each join leaves one cage body; the build raises with the piece's name
otherwise. The proof's unmarked sleeve is 16,602 mm³ at the defaults; the marks add about 5.7 mm³.

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 1619,
      "last": 1624,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 547,
      "last": 549,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 2014,
      "last": 2024,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 657,
      "last": 664,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1619–1624; `spec/screwgear/fusion.md` L547–549; `spec/screwgear/instructions.md` L2014–2024; `spec/screwgear/fusion.md` L657–664.

## S27 `[PROSE]` Relocate the bodies and hide the construction geometry

The method `relocateBodies`: name the cage body `Cage` (`self.cageBody.name = 'Cage'`), then move each
finished body into its sub-component with `body.moveToComponent(occurrence)`, which keeps its world
position and needs no activation `[PB-NO-CROSS-SIBLING]`: `self.gearBodies[0]` into
`self.gearOccs[0]`, `self.gearBodies[1]` into `self.gearOccs[1]`, and `self.cageBody` into
`self.cageOcc`. Then call `solids.hide_construction_geometry(self.designOcc.component)`
`[PB-TREE-CLEANUP]`: the `Design` component is where every sketch and construction plane was made,
and the helper walks it and anything under it. Do not re-implement it, and add no display settle of
your own `[PB-SETTLE-DISPLAY]`.

At the defaults the build makes 16 sketches, 8 construction planes and 50 features (70 timeline
entries, 75 with the five component creations).

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "moveToComponent",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "body",
      "role": "required",
      "span": "body.moveToComponent(occurrence)"
    },
    {
      "condition": null,
      "name": "hide_construction_geometry",
      "owner": null,
      "reason": "lib/geargen/solids.py defines hide_construction_geometry",
      "receiver": "solids",
      "role": "inherited",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    }
  ],
  "citations": [
    {
      "first": 871,
      "last": 910,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1626,
      "last": 1632,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 534,
      "last": 555,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L871–910; `spec/screwgear/instructions.md` L1626–1632; `.claude/skills/generate-gear/PLAYBOOK.md` L534–555.
