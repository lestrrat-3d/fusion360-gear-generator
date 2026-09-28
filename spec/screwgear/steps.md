The proof of this step list is `proof/screwgear/compiled_frame_test.go`, `proof/screwgear/compiled_sketches_test.go`, `proof/screwgear/compiled_ribbon_test.go`, `proof/screwgear/compiled_cage_test.go` and the generated `proof/screwgear/zz_registrations_test.go`, beside the hand-written mechanism proof in the same package.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `8cb5bc1bc9fa9a94cec7ca767e654078215dfd47` |
| `spec/screwgear/fusion.md` | `16118dec06dda937a5f1f4cdfddffee6c0182094` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `78c49a33be8a4a227a97a41bddb76aac0c79ebf6` |
| `spec/screwgear/mesh-search.md` | `6bce4ed34ac716cbe6bdee0cb37202c25b66f476` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9c55b15a3e9a5af76d52635d6b8b6049699454e6` |

## S01 `[PROSE]` Module, classes and call graph

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 8,
      "last": 9,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 368,
      "last": 395,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 607,
      "last": 621,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 397,
      "last": 414,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L8–9; `spec/screwgear/instructions.md` L368–395; `spec/screwgear/instructions.md` L607–621; `spec/screwgear/instructions.md` L397–414.

The module is `lib/geargen/screwgear.py`. It subclasses `base.Generator` directly, shares no
involute math, and defines exactly two public classes, exported through `lib/geargen/__init__.py`
(the entry `commands/screwgear/entry.py` binds them by name with `gear_type='ScrewGear'` and
`name='Screw Gear Generator'`):

- `ScrewGearCommandInputsConfigurator`, with the classmethod `configure` (arguments `cls, command`) of S02.
  No conditional visibility and no `input_changed` hook.
- `ScrewGearGenerator`, subclassing `base.Generator`: the inherited one-argument constructor
  taking `design`; `generate`, taking `inputs`; and `prefixBase`, returning `'ScrewGear'`. It relies on the
  inherited `deleteComponent` for rollback on failure ([PB-LOGGING]); nothing here catches and
  swallows an error.

Imports are explicit, never `import *` (playbook "Module layout & imports"): `math`,
`adsk.core`, `adsk.fusion`, `fusion360utils as futil` from `...lib`, `get_design` from `.misc`,
`Generator` and `get_selection` from `.base`, `find_profile_by_curve_counts` from `.utilities`,
and `from . import solids`. It registers **no** user parameters ([PB-PRECOMPUTED-MODE]): every
value is computed in Python in Fusion's internal units — centimetres and radians — and written
numerically. Lengths below are quoted in millimetres; divide by 10 for centimetres.

There is no generation context. The generator carries its handles on `self`:
`self.designOcc`, `self.gearOccs` (two occurrences, gear A then gear B), `self.cageOcc`,
`self.gearBodies` (two bodies), `self.cageBody`, and `self.pathLines`, a list of two dicts, one
per gear, keyed `'collar-'`, `'bore-'`, `'collar+'` and `'bore+'`, each the Paths line of S07
that one sweep runs along.

The call graph is fixed; the methods keep these names and this order:

```
generate(inputs)
  processInputs(inputs)          # S03: read, check, precompute; the rod search runs here
  buildComponentTree()           # S04
  buildAnchor()                  # S05, S06: Anchor sketch, the two axis planes, n̂, the gear frames
  buildGear(index)  for index 0, then 1
    buildSweepPaths(index)       # S07
    buildToothCell(index)        # S08, S09
    repeatCellByDoubling(index)  # S10-S12 per round, then S13-S15 when r > 0
  buildCage()                    # S16-S34
  relocateBodies()               # S35
  solids.hide_construction_geometry(self.designOcc.component)
```

**Never activate an occurrence** ([PB-NEVER-ACTIVATE]): every sketch, construction plane and
feature is made through the `Design` component's own collections, `self.designOcc.component`,
which is never activated. Sketches and planes are placed on the user's selected plane or on
planes built from it, never on a re-derived copy of it ([PB-USE-SELECTED-PLANE]). Every
collection a step could leave empty is checked where it is produced and raises a message that
names the operation and carries the measured count ([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]).

Every sketch named below is gated: after its last entity, constraint, dimension and
`isFixed`, and after computing is turned back on where it was deferred, the build reads
`sketch.isFullyConstrained` and raises naming the sketch when it is `False`
([PB-SKETCH-FIRST], [PB-FULL-CONSTRAINT]). No sketch carries text, so none is exempt
([PB-TEXT-HOLDS-DOF]). Sketch curves come from `sketch.sketchCurves`
([PB-SKETCHCURVES]); constraint names are spelled as the API spells them ([PB-API-SPELLING]).

## S02 `[PROSE]` Dialog inputs

<!-- step-meta
{
  "calls": [
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
      "name": "addSelectionInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.addSelectionInput(inputId, label, tooltip)"
    },
    {
      "condition": null,
      "name": "setSelectionLimits",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.setSelectionLimits(1, 1)"
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
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "parentInput.addSelection(get_design().rootComponent)"
    }
  ],
  "citations": [
    {
      "first": 416,
      "last": 479,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L416–479.

`ScrewGearCommandInputsConfigurator.configure` adds, to
`command.commandInputs`, first the three selections at top level and then three groups. Each
group is `command.commandInputs.addGroupCommandInput(groupId, groupLabel)`, its inputs added to
`group.children` instead of the top level; `ribbonGroup` and `frameGroup` are left expanded and
`meshGroup` gets `group.isExpanded = False`. The selections come first because Fusion focuses the
first selection input ([PB-AUTOFOCUS-FIRST]).

Each selection is `inputs.addSelectionInput(inputId, label, tooltip)`, then its filters as the
named `adsk.core.SelectionCommandInput` constants ([PB-SELECTION-FILTER-ENUM]), then
`selectionInput.setSelectionLimits(1, 1)` ([PB-SELECTION-DECL]):

| order | input id | label | tooltip | filters |
|---|---|---|---|---|
| 1 | `plane` | Target Plane | `Plane the cage's axis is normal to` | `ConstructionPlanes`, `PlanarFaces` |
| 2 | `point` | Centre Point | `Centre of the mechanism` | `ConstructionPoints`, `SketchPoints` |
| 3 | `parent` | Parent Component | `Component the mechanism is created under` | `Occurrences`, `RootComponents` |

The parent input pre-selects the design's root component with
`parentInput.addSelection(get_design().rootComponent)`.

Each value input is `group.children.addValueInput(inputId, label, unit,
adsk.core.ValueInput.createByReal(default))` with the default in internal units
([PB-DIALOG-DEFAULT-UNITS]): a length default is millimetres divided by 10, an angle default is
radians (`math.radians` of the degrees), and `toothCount`'s default is the bare count with unit
`''`. The unit string is `'mm'` for a length and `'deg'` for an angle.

| group id | group label | order | input id | label | unit | default (display) | createByReal value |
|---|---|---|---|---|---|---|---|
| `ribbonGroup` | Ribbon | 1 | `ribbonWidth` | Ribbon Width | mm | 15 | 1.5 |
| `ribbonGroup` | Ribbon | 2 | `toothCount` | Tooth Count | `''` | 68 | 68 |
| `ribbonGroup` | Ribbon | 3 | `twistLead` | Twist Lead | mm | 49.5 | 4.95 |
| `ribbonGroup` | Ribbon | 4 | `ribbonThickness` | Ribbon Thickness | mm | 3.75 | 0.375 |
| `ribbonGroup` | Ribbon | 5 | `toothPitch` | Tooth Pitch | mm | 2.625 | 0.2625 |
| `ribbonGroup` | Ribbon | 6 | `toothHeight` | Tooth Height | mm | 2.625 | 0.2625 |
| `frameGroup` | Frame | 1 | `ringRadius` | Ring Radius | mm | 16.875 | 1.6875 |
| `frameGroup` | Frame | 2 | `cageRadius` | Cage Radius | mm | 15 | 1.5 |
| `frameGroup` | Frame | 3 | `cageRise` | Cage Rise | mm | 20.25 | 2.025 |
| `frameGroup` | Frame | 4 | `clearance` | Clearance | mm | 0.45 | 0.045 |
| `frameGroup` | Frame | 5 | `collarHalf` | Collar Half Length | mm | 3 | 0.3 |
| `frameGroup` | Frame | 6 | `collarWall` | Collar Wall | mm | 3 | 0.3 |
| `frameGroup` | Frame | 7 | `rodDiameter` | Rod Diameter | mm | 3 | 0.3 |
| `frameGroup` | Frame | 8 | `ringWire` | Ring Wire | mm | 3.75 | 0.375 |
| `meshGroup` | Mesh (from the mesh search) | 1 | `crossAngle` | Crossing Angle | deg | 80 | radians(80) |
| `meshGroup` | Mesh (from the mesh search) | 2 | `engagement` | Engagement | mm | 0.75 | 0.075 |
| `meshGroup` | Mesh (from the mesh search) | 3 | `mountAngleA` | Mounting Angle A | deg | 15 | radians(15) |
| `meshGroup` | Mesh (from the mesh search) | 4 | `mountAngleB` | Mounting Angle B | deg | 15 | radians(15) |
| `meshGroup` | Mesh (from the mesh search) | 5 | `assemblyPhase` | Assembly Phase | mm | −1.31 | −0.131 |

Module-level constants name every input id, spelled `INPUT_ID_` plus the id in upper snake
case: `INPUT_ID_PLANE = 'plane'`, `INPUT_ID_POINT = 'point'`, `INPUT_ID_PARENT = 'parent'`,
`INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'`, `INPUT_ID_TOOTH_COUNT = 'toothCount'`,
`INPUT_ID_TWIST_LEAD = 'twistLead'`, `INPUT_ID_RIBBON_THICKNESS = 'ribbonThickness'`,
`INPUT_ID_TOOTH_PITCH = 'toothPitch'`, `INPUT_ID_TOOTH_HEIGHT = 'toothHeight'`,
`INPUT_ID_RING_RADIUS = 'ringRadius'`, `INPUT_ID_CAGE_RADIUS = 'cageRadius'`,
`INPUT_ID_CAGE_RISE = 'cageRise'`, `INPUT_ID_CLEARANCE = 'clearance'`,
`INPUT_ID_COLLAR_HALF = 'collarHalf'`, `INPUT_ID_COLLAR_WALL = 'collarWall'`,
`INPUT_ID_ROD_DIAMETER = 'rodDiameter'`, `INPUT_ID_RING_WIRE = 'ringWire'`,
`INPUT_ID_CROSS_ANGLE = 'crossAngle'`, `INPUT_ID_ENGAGEMENT = 'engagement'`,
`INPUT_ID_MOUNT_ANGLE_A = 'mountAngleA'`, `INPUT_ID_MOUNT_ANGLE_B = 'mountAngleB'`,
`INPUT_ID_ASSEMBLY_PHASE = 'assemblyPhase'`; and the group ids `GROUP_ID_RIBBON =
'ribbonGroup'`, `GROUP_ID_FRAME = 'frameGroup'`, `GROUP_ID_MESH = 'meshGroup'`. One more
module constant is not an input: `CELL_TEETH = 4`, the teeth the lofted cell holds (S08).

## S03 `[PROSE]` Read, check and precompute the inputs

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "get_selection",
      "owner": null,
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "get_selection(inputs, INPUT_ID_PARENT)"
    },
    {
      "condition": null,
      "name": "get_selection",
      "owner": null,
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "get_selection(inputs, INPUT_ID_PLANE)"
    },
    {
      "condition": null,
      "name": "itemById",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.itemById(inputId)"
    },
    {
      "condition": null,
      "name": "evaluateExpression",
      "owner": "adsk.core.UnitsManager",
      "reason": null,
      "receiver": "design.unitsManager",
      "role": "required",
      "span": "design.unitsManager.evaluateExpression(valueInput.expression, units)"
    }
  ],
  "citations": [
    {
      "first": 456,
      "last": 534,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 153,
      "last": 184,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 222,
      "last": 235,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 692,
      "last": 704,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 878,
      "last": 911,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 912,
      "last": 919,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1001,
      "last": 1033,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 745,
      "last": 762,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 819,
      "last": 845,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L456–534; `spec/screwgear/instructions.md` L153–184; `spec/screwgear/instructions.md` L222–235; `spec/screwgear/instructions.md` L692–704; `spec/screwgear/instructions.md` L878–911; `spec/screwgear/instructions.md` L912–919; `spec/screwgear/instructions.md` L1001–1033; `spec/screwgear/instructions.md` L745–762; `spec/screwgear/instructions.md` L819–845.

`processInputs` pulls the selections out first, before anything creates an occurrence
([PB-SELECTION-STASH]): `get_selection(inputs, INPUT_ID_PARENT)` resolved to a component
(an `Occurrence`'s `component`, or the component itself) into `self.parentComponent`,
`get_selection(inputs, INPUT_ID_PLANE)` into `self.plane` and `get_selection(inputs,
INPUT_ID_POINT)` into `self.point`, each raising unless exactly one entity was selected.

Every input is read by id with `inputs.itemById(inputId)` on the command's top-level
`commandInputs` — ids are unique across the command, so the lookup reaches into the groups —
and `processInputs` raises naming the id when the lookup returns `None`. Values are read with
`design.unitsManager.evaluateExpression(valueInput.expression, units)`, `units` being `'cm'` for a
length, `'rad'` for an angle and `''` for `toothCount` ([PB-EVAL-EXPRESSION], [PB-INPUT-READ]);
the result is in internal units, and an angle is converted with `math.degrees` only for a
message.

**Range checks, in this order, each raising a message that names the field and the bound**
(numbers in millimetres; W is `ribbonWidth`, T `ribbonThickness`, H `toothHeight`, P
`toothPitch`, N `toothCount`):

1. `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `ringWire`, `rodDiameter`,
   `collarHalf`, `collarWall`, `clearance` are each `> 0`; `toothCount` is a whole number `>= 4`.
2. `toothHeight` is `> 0` and `< ribbonWidth/2`.
3. `engagement` is `> 0` and `<= toothHeight`.
4. `crossAngle` lies strictly between 0° and 180°. No clamp to the searched range.
5. `assemblyPhase` lies strictly within `±toothPitch`.
6. `cageRadius - collarHalf` exceeds the engaged zone's half-length
   `1.5*sqrt(W^2 - A^2)/sin(Sigma)` (7.13 mm at the defaults), and
   `cageRadius + collarHalf + 1 mm < N*P/2`, the millimetre being the bore's margin.
7. The rod search below, once per collar in the order gear A `-R`, gear A `+R`, gear B `-R`,
   gear B `+R`, fails `ringRadius` naming the collar when no azimuth clears, when
   `0 < depth <= collarWall` fails, or when `|sp - sc| + rodDiameter/2 <= collarHalf` fails;
   then, after all four, it fails `ringRadius` naming the two rods when two feet stand nearer
   than `rodDiameter + clearance`.
8. `cageRise - ringWire/2 - (A/2 + hypot(W/2, T/2)) >= clearance` (3.52 mm to spare at the
   defaults).
9. `collarWall >= rodDiameter`.

No range is enforced on either mounting angle. The build accepts inputs the mesh search never
measured and clamps none of them.

**Derived values**, computed once here and kept on `self`:

```
Lambda = twistLead / (2*pi)                 # screw parameter, advance per radian
Sigma  = crossAngle                         # radians
A      = W - engagement                     # distance between the axes
L      = N * P                              # ribbon length
Z0     = 0 for gear A, assemblyPhase for gear B
s0     = Z0 - L/2                           # each gear's ribbon starts here, on its own axis
Phi    = mountAngleA for gear A, mountAngleB for gear B
theta(g, s) = s/Lambda + Phi_g               # a section's turn at station s
Utooth(s)  = W/2 - H/2 + (H/2)*cos(2*pi*(s - Z0)/P)
n      = max(ceil((P/Lambda) / radians(2)), 8)   # steps per tooth: 10 at the defaults
c      = min(CELL_TEETH, N)                 # teeth per cell: 4 at the defaults
q, r   = N // c, N % c                      # 17 and 0 at the defaults
```

The frame of spec §1 is read in S05 and S06: `C`, `ê` and `n̂`, with `k̂ = n̂ × ê`. From them,
per gear `g`:

```
dirA = rotate(ê, +Sigma/2, about n̂)   originA = C - (A/2)*n̂   ûA = +n̂   v̂A = dirA × ûA
dirB = rotate(ê, -Sigma/2, about n̂)   originB = C + (A/2)*n̂   ûB = -n̂   v̂B = dirB × ûB
point(g, s, u, v) = origin_g + s*dir_g
                    + (u*cos(theta(g, s)) - v*sin(theta(g, s)))*û_g
                    + (u*sin(theta(g, s)) + v*cos(theta(g, s)))*v̂_g
```

`û` points at the other gear on both, which is what makes the two gears the same part. Every
seed, reference point and probe below is `point(g, s, u, v)` for the named `(s, u, v)`.

The rod search runs here, in `processInputs`, before any geometry exists. Azimuths about `n̂`,
stations and offsets do not depend on where the frame stands, so it runs in the frame's own
coordinates — `C` at the origin, `n̂ = (0, 0, 1)`, `ê = (1, 0, 0)`, `k̂ = (0, 1, 0)`, and the two
gears placed by the formulas above in those coordinates — and S19, S23 and S25 later place the
feet in the world frame that S05 and S06 read.

**The rod search** is the one the proof runs, step for step (`rodShift` and `rodClears` in
`proof/screwgear/cage_test.go`). A collar's **crossing** is `origin_g + sc*dir_g`, with
`sc = -cageRadius` or `+cageRadius`; its azimuth `psi_c` is measured about `+n̂` from `ê` in the
selected plane. A rod at azimuth `psi` stands on the foot `F = C + ringRadius*(cos(psi)*ê +
sin(psi)*k̂)`; on gear `g` its station is `sp = (F - origin_g)·dir_g` and its offset
`wp = (F - origin_g)·v̂_g`. A rod **clears** when, for both gears, at every station `s` that is a
whole multiple of 0.01 mm with `|s - sp| <= rodDiameter/2 + clearance`,

```
h(s) = (W/2)*|sin(theta(g, s))| + (T/2)*|cos(theta(g, s))|
hypot(sp - s, max(0, |wp| - h(s))) >= rodDiameter/2 + clearance
```

For each collar, walk `shift` from 0 upward in steps of 0.25° while `shift <= 360°`; at the first
`shift` whose azimuth `psi_c + shift` clears, return 0 if `shift` is 0, otherwise bisect between
`shift - 0.25°` (does not clear) and `shift` (clears), moving the upper end down when the middle
clears and the lower end up when it does not, until the two are within 0.001°, and return the
upper end. No clearing azimuth in a full turn fails the input. The rod's azimuth is
`psi = psi_c + shift`; all four turn the same way, counter-clockwise about `+n̂`. At the defaults
the turns are 34.61° for a collar at `-cageRadius` and 34.43° at `+cageRadius`, and the azimuths,
taken into `[0°, 360°)`, are 254.61° (gear A `-R`), 74.43° (gear A `+R`), 174.61° (gear B `-R`)
and 354.43° (gear B `+R`).

Each rod then passes two checks on its own gear, at its own `sp` and `wp`:

```
hb    = (W/2 + clearance)*|sin(theta(g, sp))| + (T/2 + clearance)*|cos(theta(g, sp))|
depth = |wp| - hb                        # must satisfy 0 < depth <= collarWall
|sp - sc| + rodDiameter/2 <= collarHalf  # the whole rod within the collar's length
```

At the defaults `depth` is 1.49 mm and 1.38 mm, and `|sp - sc|` 1.11 mm and 1.08 mm.

**Foot order** for the loop: foot 0 is the rod whose azimuth in `[0°, 360°)` is smallest, feet
1, 2 and 3 follow in increasing azimuth; bar `i` runs from foot `i` to foot `(i + 1) mod 4`. At
the defaults foot 0 is gear A's `+R` rod.

**The doubling schedule** of §3, which S10–S12 execute: write `q` in binary; start with the cell,
`m = 1`; for each bit of `q` below its top bit, lowest first, if that bit is set take an unmoved
copy of the body as an **aside** of `m` cells, then double (a round: copy, move by
`Step(m*c)`, join; `m` becomes `2m`); after the last doubling, move each aside into place,
largest first, by `Step(m*c)`, join it, and add its cells to `m`. There are
floor(log2 q) + popcount(q) − 1 rounds: 5 at the defaults (one aside of one cell taken before
the first doubling, doublings to 16 cells, the aside moved by `Step(64)`), 7 in one-tooth cells,
none when `q = 1`.

## S04 `[PROSE]` Component tree

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "getOccurrence",
      "owner": null,
      "reason": null,
      "receiver": "self",
      "role": "required",
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
    }
  ],
  "citations": [
    {
      "first": 397,
      "last": 414,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L397–414.

`buildComponentTree` takes the top occurrence from the inherited `self.getOccurrence()`, which
creates it under `self.parentComponent` and is what the inherited rollback deletes, and names its
component `Screw Gearing` through `occurrence.component.name`; it never calls `addNewComponent`
for the top itself. Under it, four children, each
`topComponent.occurrences.addNewComponent(adsk.core.Matrix3D.create())` with its component named
([PB-OCCURRENCE-TREE]): `Design` (kept as `self.designOcc`; every sketch, plane and feature is
made in its component), `Gear A` and `Gear B` (`self.gearOccs`) and `Cage` (`self.cageOcc`).
The last three stay empty until S35 moves the finished bodies in ([PB-NO-CROSS-SIBLING]).

## S05 `[GO]` Anchor sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "self.designOcc.component.sketches",
      "role": "required",
      "span": "sketch = self.designOcc.component.sketches.add(self.plane)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "projected = sketch.project(self.point).item(0)"
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
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "line = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(projected, line)"
    },
    {
      "condition": null,
      "name": "addMidPoint",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addMidPoint(projected, line)"
    },
    {
      "condition": null,
      "name": "addHorizontal",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addHorizontal(line)"
    }
  ],
  "citations": [
    {
      "first": 660,
      "last": 691,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 397,
      "last": 415,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 449,
      "last": 458,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L660–691; `spec/screwgear/fusion.md` L397–415; `spec/screwgear/fusion.md` L449–458.

Proof: `stepAnchorSketch`.

<!-- proof-run: proofkit.Run(csAnchorCases, stepAnchorSketch) -->

On the selected plane, `sketch = self.designOcc.component.sketches.add(self.plane)` in the
`Design` component, named `Anchor` ([PB-USE-SELECTED-PLANE]).
Computing is **not** deferred in this sketch ([SCREW-F-DEFER]).

1. Project the selected point: `projected = sketch.project(self.point).item(0)`. This is the one
   projection in the build ([SCREW-F-REFERENCES]).
2. Draw the Anchor Line from two raw seeds, `start = adsk.core.Point3D.create(p.x - 0.5, p.y, 0)`
   and `end = adsk.core.Point3D.create(p.x + 0.5, p.y, 0)` with `p = projected.geometry`:
   `line = sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)`, 10 mm long with its end
   to the right of its start ([PB-SEED-NEAR]).
3. Four things and nothing else:
   `sketch.geometricConstraints.addCoincident(projected, line)`,
   `sketch.geometricConstraints.addMidPoint(projected, line)` — both, the coincident is not
   redundant here — `sketch.geometricConstraints.addHorizontal(line)`, which is sketch-local and
   survives a tilted plane ([PB-REFLINE-DIRECTION]), and a **horizontal** distance
   `dim = sketch.sketchDimensions.addDistanceDimension(line.startSketchPoint,
   line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
   textPoint)` with `textPoint = adsk.core.Point3D.create(p.x, p.y + 0.3, 0)` and
   `dim.parameter.value = 1.0` (10 mm, a magnitude — [PB-DIM-VALUE-SEMANTICS]). Not an aligned
   length: that one is satisfied by the line in either end-for-end orientation.
4. Raise unless `sketch.isFullyConstrained`. Only then read the frame from world geometry
   ([PB-WORLDGEO-CONSTRAINED], [PB-WORLD-FRAME]): `C = projected.worldGeometry`, and `ê` the unit
   vector from `line.startSketchPoint.worldGeometry` to `line.endSketchPoint.worldGeometry`.
   Keep `line` as `self.anchorLine`: it is passed once more, to the Ring Plane of S16. No later
   sketch projects or reads this sketch again ([PB-SOLVED-GEOMETRY]).

The proof models the projection as a sketch-engine reference point and writes the midpoint
alone, because the engine's midpoint already carries the coincident row; it proves the line's
four degrees of freedom are taken, that the only configuration is the seeded one (the end right
of the start), and that `ê` runs along the sketch's own `+x`.

## S06 `[PROSE]` Gear axis planes and the direction n̂

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "planeInput = constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offset))"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offset))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 706,
      "last": 713,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 460,
      "last": 469,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 692,
      "last": 704,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L706–713; `spec/screwgear/fusion.md` L460–469; `spec/screwgear/instructions.md` L692–704.

Two construction planes offset from the selected plane itself ([PB-USE-SELECTED-PLANE],
[PB-CONSTRUCTION-PLANES]): `planeInput = constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offset))`,
`constructionPlanes.add(planeInput)`, with `constructionPlanes =
self.designOcc.component.constructionPlanes`: `Gear A Axis Plane` at `offset = -A/2` first, then
`Gear B Axis Plane` at `offset = +A/2`.

Fusion offsets along the selected entity's own normal and the build does not assume its sign
([SCREW-F-NORMAL-SIGN]): read `plane.geometry` of the Gear A Axis Plane, its `origin` and
`normal`, and set `n̂ = normal` when `(C - origin)·normal > 0`, else `-normal`, so that `C` lies
`+A/2` along `n̂` from gear A's plane. Then check, for both axis planes, that `C` stands `A/2`
from the plane, `|(C - origin)·normal| = A/2` to within 1e-6 cm, and raise naming the plane and
the two distances otherwise. Every later offset from the selected plane is signed the same way:
negative on gear A's side, positive on gear B's.

With `n̂` known, compute `k̂ = n̂ × ê` and both gears' world frames of S03. This is a construction step the sketch and solid engines do not build: the proof
takes the same frame as numbers (`csGears` and `csAxes` in `proof/screwgear/compiled_frame_test.go`)
and checks, in S07, that each gear's axis lies in its axis plane and that `C` stands `A/2` from it.

## S07 `[GO]` Paths sketch, per gear

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "pt = sketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(fromPoint, toPoint)"
    }
  ],
  "citations": [
    {
      "first": 715,
      "last": 734,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 536,
      "last": 560,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 208,
      "last": 223,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 417,
      "last": 447,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L715–734; `spec/screwgear/instructions.md` L536–560; `spec/screwgear/fusion.md` L208–223; `spec/screwgear/fusion.md` L417–447.

Proof: `stepPathsSketch`.

<!-- proof-run: proofkit.Run(csPathsCases, stepPathsSketch) -->

`buildSweepPaths` draws one sketch per gear, `sketch =
self.designOcc.component.sketches.add(axisPlane)` on that gear's Axis Plane of S06, named
`Gear A Paths` or `Gear B Paths`. Not deferred.

The eight stations, each `origin_g + s*dir_g` on the gear's axis (which lies in its axis
plane), for `sc = -cageRadius` and then `sc = +cageRadius`:

| line | from station | to station |
|---|---|---|
| `collar-` | `-cageRadius - collarHalf` | `-cageRadius + collarHalf` |
| `bore-` | `-cageRadius - collarHalf - 1 mm` | `-cageRadius + collarHalf + 1 mm` |
| `collar+` | `+cageRadius - collarHalf` | `+cageRadius + collarHalf` |
| `bore+` | `+cageRadius - collarHalf - 1 mm` | `+cageRadius + collarHalf + 1 mm` |

Each station is a **reference point** ([SCREW-F-REFERENCES]): `local =
sketch.modelToSketchSpace(worldPoint)`, then `local.z = 0` ([PB-SKETCH-ZERO-Z],
[PB-SPACE-METHODS]), then `pt = sketch.sketchPoints.add(local)`. The four lines are solid
`sketch.sketchCurves.sketchLines.addByTwoPoints(fromPoint, toPoint)`, each **sharing** its two
points ([PB-SHARE-XOR-COINCIDENT]), so each runs from its negative station to its positive one,
along `+dir_g`. After the fourth line, set every one of the eight points `pt.isFixed = True`
([PB-PROJECT-NOT-FIXED]). No constraint and no dimension; nothing else in the sketch. Raise unless
fully constrained. Store the four lines in `self.pathLines[index]` under their keys. A collar's
line and its bore's overlap, and all four lie on one carrier; Fusion reads that fully
constrained, and a path made from one of them with chaining off holds that one line
([PB-PATH-FROM-SKETCH]).

At the defaults the stations are ±12, ±18 (collars) and ±11, ±19 mm (bores) on each gear's axis.

## S08 `[GO]` Cell Sections sketch, per gear

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(corner))"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(corner))"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(cornerA, cornerB)"
    }
  ],
  "citations": [
    {
      "first": 736,
      "last": 743,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 745,
      "last": 762,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 784,
      "last": 808,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 561,
      "last": 568,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 590,
      "last": 600,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 153,
      "last": 172,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L736–743; `spec/screwgear/instructions.md` L745–762; `spec/screwgear/instructions.md` L784–808; `spec/screwgear/instructions.md` L561–568; `spec/screwgear/instructions.md` L590–600; `spec/screwgear/fusion.md` L153–172.

Proof: `stepCellSectionsSketch`.

<!-- proof-run: proofkit.Run(csSectionCases, stepCellSectionsSketch) -->

`buildToothCell` draws **one** sketch, `sketch =
self.designOcc.component.sketches.add(axisPlane)` on the gear's Axis Plane, named `Gear A Cell
Sections` or `Gear B Cell Sections` ([SCREW-F-CELL-LOFT], [PB-3D-SKETCH-SECTIONS]). Set
`sketch.isComputeDeferred = True` right after naming it, before its first point
([PB-SKETCH-DEFER], [SCREW-F-DEFER]).

The cell is `c` teeth at the ribbon's negative end: `c*n + 1` sections at stations
`s_k = s0 + k*P/n` for `k = 0 .. c*n` — 41 sections, 0.2625 mm apart and each turned 1.91°
further than the last, at the defaults, and 11 at `CELL_TEETH = 1`. The count is derived in S03,
never an input. Section `k` is the rectangle `u` from `uB = -W/2` to `uF = Utooth(s_k)`, `v` from
`-T/2` to `+T/2`, turned by `theta(g, s_k)`. Its four corners are `point(g, s_k, uB, -T/2)`,
`point(g, s_k, uF, -T/2)`, `point(g, s_k, uF, T/2)` and `point(g, s_k, uB, T/2)`, each added
with `sketch.sketchPoints.add(sketch.modelToSketchSpace(corner))` **keeping its z**: these
points are meant to lie off the plane, which is the one exception to [PB-SKETCH-ZERO-Z]. Four
solid lines share them in order — `L1` corner 1 to 2, `L2` 2 to 3, `L3` 3 to 4, `L4` 4 to 1 —
with `sketch.sketchCurves.sketchLines.addByTwoPoints(cornerA, cornerB)`, and the build keeps each
section's four lines together, in station order, for S09.

After the last line of the last section, set every point `isFixed = True`; nothing else goes in
the sketch — no construction line, constraint or dimension. Then `sketch.isComputeDeferred =
False`, raise unless `sketch.isFullyConstrained`, and raise unless `sketch.profiles.count` is
`c*n + 1`, with both numbers in the message. The profiles are not what the loft is fed.

## S09 `[GO]` Cell loft, per gear

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "loftFeatures",
      "role": "required",
      "span": "loftInput = loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
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
      "span": "collection.add(line)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "self.designOcc.component.features",
      "role": "required",
      "span": "path = self.designOcc.component.features.createPath(collection, False)"
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
      "name": "add",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "loftFeatures",
      "role": "required",
      "span": "loft = loftFeatures.add(loftInput)"
    }
  ],
  "citations": [
    {
      "first": 764,
      "last": 782,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 810,
      "last": 817,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 174,
      "last": 199,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L764–782; `spec/screwgear/instructions.md` L810–817; `spec/screwgear/fusion.md` L174–199.

Proof: `stepCellLoft`, checked by `checkCellLoft`.

<!-- proof-run: proofkit3d.RunSolid(csCellLoftCases, stepCellLoft, checkCellLoft) -->

One loft through all `c*n + 1` sections, in station order ([PB-LOFT], [SCREW-F-CELL-LOFT]):
`loftInput = loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
with `loftFeatures = self.designOcc.component.features.loftFeatures`; for each section in order
of `k`, `collection = adsk.core.ObjectCollection.create()`, `collection.add(line)` for its four
lines, `path = self.designOcc.component.features.createPath(collection, False)`
([PB-PATH-FROM-SKETCH]; `Path.create` raises in this component, so it is not used), and
`loftInput.loftSections.add(path)`; then `loft = loftFeatures.add(loftInput)`. Raise unless
`loft.bodies.count` is 1 and that body `isSolid`, with the count in the message
([PB-EMPTY-RESULT]). That body is the gear's ribbon from here on (`self.gearBodies[index]`).

Nothing else is set on the loft input: it has no ruled option, and a loft through more than two
sections is smooth between them ([PB-LOFT-TWO-SECTIONS-STRAIGHT]). The proof's stand-in is a
loft through the same sections that decad builds two at a time with flat walls, stitched into one
solid; its volume sits 2.2% under the exact helicoid cell at the defaults. Fusion's smooth loft
was measured on 2026-09-28 within 0.04 mm of the helicoid between sections, at the 10 mm ribbon.

## S10 `[GO]` Doubling round: copy the body

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "self.designOcc.component.features.copyPasteBodies",
      "role": "required",
      "span": "copyFeature = self.designOcc.component.features.copyPasteBodies.add(body)"
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
      "first": 819,
      "last": 845,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 859,
      "last": 860,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 144,
      "last": 151,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L819–845; `spec/screwgear/instructions.md` L859–860; `spec/screwgear/fusion.md` L144–151.

Proof: `stepCopyRibbon`, checked by `checkCopyRibbon`.

<!-- proof-run: proofkit3d.RunSolid(csCopyCases, stepCopyRibbon, checkCopyRibbon) -->

`repeatCellByDoubling` runs the schedule of S03. Every round, and every aside taken on the
way up, starts with a copy of the current body:
`copyFeature = self.designOcc.component.features.copyPasteBodies.add(body)`, and the copy is
`copyFeature.bodies.item(0)` ([SCREW-F-COPY-BODY]). Never read the feature's `sourceBody`, which
is the original: moving it walks the ribbon away one step at a time. Raise unless
`copyFeature.bodies.count` is 1 ([PB-EMPTY-RESULT]). An aside is such a copy, kept unmoved until its turn in S11.

## S11 `[GO]` Doubling round: screw-move the copy

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "rot = adsk.core.Matrix3D.create()"
    },
    {
      "condition": null,
      "name": "copy",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "axisVector",
      "role": "required",
      "span": "shift = axisVector.copy()"
    },
    {
      "condition": null,
      "name": "scaleBy",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "shift",
      "role": "required",
      "span": "shift.scaleBy(k*P)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "mov = adsk.core.Matrix3D.create()"
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
      "span": "bodies.add(copy)"
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
      "receiver": "moveFeatures",
      "role": "required",
      "span": "moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 821,
      "last": 825,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 859,
      "last": 867,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 107,
      "last": 142,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L821–825; `spec/screwgear/instructions.md` L859–867; `spec/screwgear/fusion.md` L107–142.

Proof: `stepMoveRibbon`, checked by `checkMoveRibbon`.

<!-- proof-run: proofkit3d.RunSolid(csMoveCasesTable, stepMoveRibbon, checkMoveRibbon) -->

A round's copy is moved by `Step(m*c)` teeth, `m` the cells the body holds; an aside is moved,
after the last doubling, by `Step(m*c)` for the `m` reached at that point, largest aside first.
`Step(k)` is a turn of `k*P/Lambda` about the gear's axis composed with `k*P` along it
([SCREW-F-SCREW-STEP]), built from two matrices:

1. `rot = adsk.core.Matrix3D.create()`, then `rot.setToRotation(k*P/Lambda, axisVector,
   axisPoint)` with `axisVector = adsk.core.Vector3D.create(dir.x, dir.y, dir.z)` for the
   gear's unit `dir_g` and `axisPoint` the gear's `origin_g` as a `Point3D`.
2. `shift = axisVector.copy()`, then `shift.scaleBy(k*P)`; `mov = adsk.core.Matrix3D.create()`;
   `mov.translation = shift` — the whole vector assigned.
3. `rot.transformBy(mov)`. Never assign `rot.translation` after `setToRotation`: that overwrites
   the offset that carries the rotation onto the gear's axis, and the body lands elsewhere with
   nothing raised.

Then `bodies = adsk.core.ObjectCollection.create()`, `bodies.add(copy)`, `moveInput =
self.designOcc.component.features.moveFeatures.createInput2(bodies)`,
`moveInput.defineAsFreeMove(rot)`, `moveFeatures.add(moveInput)` ([PB-MOVE-ROTATE]). `k` is
never 0 here, so the zero-angle refusal of [PB-MOVE-ROTATE] cannot arise; do not add a `k = 0`
case to the loop.

The proof moves a cell by each `Step(k)` the schedule makes and holds it, vertex for vertex, on
the cell lofted `k` teeth further along: the move lands exactly because the ribbon is invariant
under its own screw step.

## S12 `[GO]` Doubling round: join

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
      "receiver": "combineFeatures",
      "role": "required",
      "span": "combineInput = combineFeatures.createInput(body, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combineFeatures",
      "role": "required",
      "span": "combine = combineFeatures.add(combineInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "combine.bodies",
      "role": "required",
      "span": "combine.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 845,
      "last": 850,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 859,
      "last": 864,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 381,
      "last": 392,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L845–850; `spec/screwgear/instructions.md` L859–864; `spec/screwgear/fusion.md` L381–392.

Proof: `stepJoinRibbon`, checked by `checkJoinRibbon`.

<!-- proof-run: proofkit3d.RunSolid(csJoinCasesTable, stepJoinRibbon, checkJoinRibbon) -->

Join the moved copy onto the body: `tools = adsk.core.ObjectCollection.create()`,
`tools.add(movedCopy)`, `combineInput = combineFeatures.createInput(body, tools)` with
`combineFeatures = self.designOcc.component.features.combineFeatures`, `combineInput.operation =
adsk.fusion.FeatureOperations.JoinFeatureOperation`, `combineInput.isKeepToolBodies = False`,
`combine = combineFeatures.add(combineInput)`. Raise unless `combine.bodies.count` is 1, naming
the gear, the round and the count ([PB-EMPTY-RESULT], [SCREW-F-ROUND-FRAME]); `combine.bodies.item(0)`
is the ribbon from then on. Every move is by a whole number of teeth already built, so each join
meets its neighbour on one shared cross-section: no gap, no sliver, no overlap.

After the gear's last join — or after S15 when there is a remainder, or straight after S09 when
`q = 1` and `r = 0` — name the body `Gear A` or `Gear B` with `body.name`.

The proof checks every round whose joined ribbon it can stitch, up to about 320 sections; the
defaults' last two rounds (32 + 32 and 64 + 4 teeth) exceed decad's loft audit and are reported
unmodelled, the smaller rounds proving the same move and the same shared section.

## S13 `[GO]` Cell Remainder sketch, when N mod c is not zero

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 852,
      "last": 857,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 784,
      "last": 808,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L852–857; `spec/screwgear/instructions.md` L784–808.

Proof: `stepRemainderSketch`.

<!-- proof-run: proofkit.Run(csRemainderCases, stepRemainderSketch) -->

Only when `r > 0`, after the last round of S10–S12: a second Cell Sections sketch on the same
Axis Plane, named `Gear A Cell Remainder` or `Gear B Cell Remainder`, drawn exactly as S08
([PB-3D-SKETCH-SECTIONS], [PB-SKETCH-DEFER], [PB-SKETCH-FIRST]) with
`c` replaced by `r`: `r*n + 1` sections at stations from `s0 + q*c*P` to `s0 + N*P`, deferred,
points added with their z kept, fixed after the last line, gated on `isFullyConstrained` and on
`profiles.count == r*n + 1`. The defaults have none; 69 teeth in four-tooth cells have a
one-tooth remainder of 11 sections.

## S14 `[GO]` Cell Remainder loft

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "createPath(collection, False)"
    }
  ],
  "citations": [
    {
      "first": 852,
      "last": 857,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 810,
      "last": 817,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L852–857; `spec/screwgear/instructions.md` L810–817.

Proof: `stepRemainderLoft`, checked by `checkRemainderLoft`.

<!-- proof-run: proofkit3d.RunSolid(csRemainderLoftCases, stepRemainderLoft, checkRemainderLoft) -->

Only when `r > 0`: the loft of S09 through the remainder's sections, in station order ([PB-LOFT]),
one `createPath(collection, False)` per section ([PB-PATH-FROM-SKETCH]), `NewBodyFeatureOperation`; raise unless the
feature's `bodies.count` is 1 and its body `isSolid` ([PB-EMPTY-RESULT]). It is built where the teeth belong, never
moved. Its first section is the ribbon's last.

## S15 `[GO]` Cell Remainder join

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 852,
      "last": 857,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 859,
      "last": 864,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L852–857; `spec/screwgear/instructions.md` L859–864.

Proof: `stepRemainderJoin`, checked by `checkRemainderJoin`.

<!-- proof-run: proofkit3d.RunSolid(csRemainderJoinCases, stepRemainderJoin, checkRemainderJoin) -->

Only when `r > 0`: the join of S12 with the remainder body as the one tool, `isKeepToolBodies =
False`, raising unless the combine's `bodies.count` is 1 ([PB-EMPTY-RESULT]); then name the ribbon `Gear A` or
`Gear B`. The two meet on the shared section at `s0 + q*c*P`.

## S16 `[PROSE]` Ring Plane

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "planeInput = constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)"
    },
    {
      "condition": null,
      "name": "setByAngle",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 323,
      "last": 327,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 875,
      "last": 877,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L323–327; `spec/screwgear/instructions.md` L875–877.

`buildCage` starts with the frame's round pieces ([SCREW-F-ROUND-FRAME]). No construction axis
is made anywhere; every revolve axis is a sketch line ([PB-CONSTRUCTION-NEEDS-ACTIVE]).

The Ring Plane is the plane through the Anchor Line square to the selected plane:
`planeInput = constructionPlanes.createInput()`,
`planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)`
with the line passed directly ([PB-CONSTRUCTION-PLANES]), `constructionPlanes.add(planeInput)`,
named `Ring Plane`. It holds `C`, `ê` and `n̂`. The engines build no construction plane; the proof
draws the Ring sketch in this plane's `(ê, n̂)` coordinates.

## S17 `[GO]` Ring sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchCircles",
      "role": "required",
      "span": "circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, ringWire/2)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDiameterDimension(circle, textPoint)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.Profiles",
      "reason": null,
      "receiver": "sketch.profiles",
      "role": "required",
      "span": "sketch.profiles.item(0)"
    }
  ],
  "citations": [
    {
      "first": 323,
      "last": 338,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 875,
      "last": 877,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 549,
      "last": 560,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L323–338; `spec/screwgear/instructions.md` L875–877; `spec/screwgear/instructions.md` L549–560.

Proof: `stepRingSketch`.

<!-- proof-run: proofkit.Run(csRingCases, stepRingSketch) -->

On the Ring Plane, a sketch named `Ring`, not deferred:

1. The revolve axis `An`: reference points at `C` and at `C + cageRise*n̂` (modelToSketchSpace,
   z set to 0, `sketchPoints.add`), the line between them sharing both, set
   `line.isConstruction = True`; then both points `isFixed = True`. The revolve uses the whole
   line the segment lies on.
2. The wire: `circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, ringWire/2)`
   at `centre = C + cageRise*n̂ + ringRadius*ê` mapped in with z set to 0; then
   `circle.centerSketchPoint.isFixed = True` ([PB-CIRCLE-CENTER]: fixed, never coincident to a
   point) and `sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` with the text
   point on the circle, `centre + (ringWire/2)*ê` mapped in with z set to 0 ([PB-RADIAL-DIM]),
   its `parameter.value = ringWire`.
3. Raise unless fully constrained, and unless `sketch.profiles.count == 1`; the profile is
   `sketch.profiles.item(0)` ([PB-SINGLE-PROFILE]). `find_profile_by_curve_counts` cannot pick a
   circle, so it is not used here. The circle never reaches the axis, since the rod checks imply
   `ringRadius > ringWire/2`.

## S18 `[GO]` Ring revolve

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))"
    },
    {
      "condition": null,
      "name": "setAngleExtent",
      "owner": "adsk.fusion.RevolveFeatureInput",
      "reason": null,
      "receiver": "revolveInput",
      "role": "required",
      "span": "revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.RevolveFeatures",
      "reason": null,
      "receiver": "revolveFeatures",
      "role": "required",
      "span": "ring = revolveFeatures.add(revolveInput)"
    }
  ],
  "citations": [
    {
      "first": 323,
      "last": 334,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 1096,
      "last": 1104,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L323–334; `spec/screwgear/instructions.md` L1096–1104.

Proof: `stepRingRevolve`, checked by `checkRingRevolve`.

<!-- proof-run: proofkit3d.RunSolid(csRingRevolveCases, stepRingRevolve, checkRingRevolve) -->

`revolveInput = revolveFeatures.createInput(profile, axisLine,
adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` with `revolveFeatures =
self.designOcc.component.features.revolveFeatures` and `axisLine` the line `An` of S17;
`revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))`;
`ring = revolveFeatures.add(revolveInput)` ([PB-REVOLVE]). Raise unless `ring.bodies.count`
is 1. That body is the cage (`self.cageBody`), the first body of the frame. At the defaults it is
a torus of 1171.05 mm³.

## S19 `[GO]` Rods sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "self.designOcc.component.sketches",
      "role": "required",
      "span": "self.designOcc.component.sketches.add(self.plane)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchCircles",
      "role": "required",
      "span": "sketch.sketchCurves.sketchCircles.addByCenterRadius(foot, rodDiameter/2)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDiameterDimension(circle, textPoint)"
    }
  ],
  "citations": [
    {
      "first": 878,
      "last": 893,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 339,
      "last": 350,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L878–893; `spec/screwgear/fusion.md` L339–350.

Proof: `stepRodsSketch`.

<!-- proof-run: proofkit.Run(csRodsCases, stepRodsSketch) -->

On the selected plane itself, `self.designOcc.component.sketches.add(self.plane)`, a sketch
named `Rods`, not deferred ([PB-USE-SELECTED-PLANE]). Four circles, in the rod search's collar order (gear A `-R`, gear A
`+R`, gear B `-R`, gear B `+R`): each
`sketch.sketchCurves.sketchCircles.addByCenterRadius(foot, rodDiameter/2)` at
`foot = C + ringRadius*(cos(psi)*ê + sin(psi)*k̂)` for that rod's azimuth `psi` from S03, mapped
in with z set to 0; `circle.centerSketchPoint.isFixed = True` ([PB-CIRCLE-CENTER]); and
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` with the text point on the
circle, `foot + (rodDiameter/2)*ê` mapped in with z set to 0 ([PB-RADIAL-DIM], [PB-SKETCH-ZERO-Z]), `parameter.value = rodDiameter`.
Nothing else in the sketch.

Raise unless fully constrained and unless `sketch.profiles.count == 4`: the rods stand at least
`rodDiameter + clearance` apart (S03), so each circle is its own profile.

## S20 `[GO]` Rods extrude

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
      "span": "profiles = adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "profiles",
      "role": "required",
      "span": "profiles.add(sketch.profiles.item(i))"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.Profiles",
      "reason": null,
      "receiver": "sketch.profiles",
      "role": "required",
      "span": "profiles.add(sketch.profiles.item(i))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise), False)"
    },
    {
      "condition": null,
      "name": "setSymmetricExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise), False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudeFeatures",
      "role": "required",
      "span": "rods = extrudeFeatures.add(extrudeInput)"
    }
  ],
  "citations": [
    {
      "first": 339,
      "last": 350,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 878,
      "last": 893,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L339–350; `spec/screwgear/instructions.md` L878–893.

Proof: `stepRodsExtrude`, checked by `checkRodsExtrude`.

<!-- proof-run: proofkit3d.RunSolid(csRodsExtrudeCases, stepRodsExtrude, checkRodsExtrude) -->

All four in one feature: `profiles = adsk.core.ObjectCollection.create()`,
`profiles.add(sketch.profiles.item(i))` for `i` in 0..3,
`extrudeInput = extrudeFeatures.createInput(profiles,
adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` with `extrudeFeatures =
self.designOcc.component.features.extrudeFeatures`,
`extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise), False)` — `False`
makes `cageRise` each side's length ([PB-THROUGH-CUT]), no taper argument — then
`rods = extrudeFeatures.add(extrudeInput)`. Raise unless `rods.bodies.count` is 4. Each rod runs
from `-cageRise` to `+cageRise` along `n̂`. They are new bodies, never a join operation on the
extrude, which would join into whatever it touches.

## S21 `[GO]` Rods join

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
      "span": "tools.add(rod)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combineFeatures",
      "role": "required",
      "span": "combineInput = combineFeatures.createInput(self.cageBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combineFeatures",
      "role": "required",
      "span": "combine = combineFeatures.add(combineInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "combine.bodies",
      "role": "required",
      "span": "self.cageBody = combine.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 381,
      "last": 392,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 1096,
      "last": 1104,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L381–392; `spec/screwgear/instructions.md` L1096–1104.

Proof: `stepRodsJoin`, checked by `checkRodsJoin`.

<!-- proof-run: proofkit3d.RunSolid(csRodsJoinCases, stepRodsJoin, checkRodsJoin) -->

One join, the ring as target and the four rods as tools:
`tools = adsk.core.ObjectCollection.create()`, `tools.add(rod)` for each of `rods.bodies`,
`combineInput = combineFeatures.createInput(self.cageBody, tools)`, operation
`JoinFeatureOperation`, `isKeepToolBodies = False`, `combine = combineFeatures.add(combineInput)`.
Raise naming the group `rods` and the count unless `combine.bodies.count` is 1
([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]); then
`self.cageBody = combine.bodies.item(0)`. Each rod's top end, at `+cageRise`, stands inside the
ring's wire, which is what joins them.

## S22 `[PROSE]` Loop Plane

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-cageRise))"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-cageRise))"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 351,
      "last": 355,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 460,
      "last": 469,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L351–355; `spec/screwgear/fusion.md` L460–469.

`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-cageRise))` on a fresh
`constructionPlanes.createInput()`, then `constructionPlanes.add(planeInput)`, named `Loop
Plane` ([PB-CONSTRUCTION-PLANES], [PB-USE-SELECTED-PLANE]). It is signed like the axis planes, so it lands on `-n̂`, gear A's side (S06). It holds
every foot `C - cageRise*n̂ + ringRadius*(cos(psi)*ê + sin(psi)*k̂)`. No plane is made per bar or
per ball. The proof draws the loop's sketches in this plane's `(ê, k̂)` coordinates.

## S23 `[GO]` Loop Bar sketch, per bar

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.Profiles",
      "reason": null,
      "receiver": "profiles",
      "role": "required",
      "span": "profiles.item(0)"
    }
  ],
  "citations": [
    {
      "first": 912,
      "last": 933,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 356,
      "last": 366,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L912–933; `spec/screwgear/fusion.md` L356–366.

Proof: `stepLoopBarSketch`.

<!-- proof-run: proofkit.Run(csLoopCases, stepLoopBarSketch) -->

For bar `i` in 0..3, in foot order (S03), `j = (i + 1) mod 4`: a sketch named `Loop Bar i` on
the Loop Plane, not deferred. Four reference points, each mapped in with z set to 0:
`foot_i`, `foot_j`, `foot_j + (ringWire/2)*m̂` and `foot_i + (ringWire/2)*m̂`, with `m̂` the unit
vector in the Loop Plane square to the bar and pointing away from the loop's centre
`C - cageRise*n̂` — the component of `(foot_i + foot_j)/2 - (C - cageRise*n̂)` square to
`foot_j - foot_i`, normalised. Four solid lines share them in that order, `B0` from `foot_i` to
`foot_j` first, each `addByTwoPoints` sharing its points ([PB-SHARE-XOR-COINCIDENT]); then all
four points `isFixed = True` ([PB-PROJECT-NOT-FIXED], [PB-SKETCH-ZERO-Z]). No dimension, no
constraint. Raise unless fully constrained and unless `profiles.count == 1`; the profile is
`profiles.item(0)` ([PB-SINGLE-PROFILE]).

## S24 `[GO]` Loop Bar revolve, per bar

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))"
    },
    {
      "condition": null,
      "name": "setAngleExtent",
      "owner": "adsk.fusion.RevolveFeatureInput",
      "reason": null,
      "receiver": "revolveInput",
      "role": "required",
      "span": "revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))"
    }
  ],
  "citations": [
    {
      "first": 356,
      "last": 366,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L356–366.

Proof: `stepLoopBarRevolve`, checked by `checkLoopBarRevolve`.

<!-- proof-run: proofkit3d.RunSolid(csLoopPieceCases, stepLoopBarRevolve, checkLoopBarRevolve) -->

Right after each bar's sketch: `revolveFeatures.createInput(profile, b0,
adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` about the bar's own side `B0`, which the
profile lies against, as a revolve profile may ([PB-REVOLVE]); `revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))` as in S18; add.
Raise unless one body. It is a cylinder of diameter `ringWire` from foot to foot, kept as a new
body for S27.

## S25 `[GO]` Loop Ball sketch, per ball

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, bl)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "find_profile_by_curve_counts(sketch, lines=1, arcs=1)"
    }
  ],
  "citations": [
    {
      "first": 367,
      "last": 377,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 912,
      "last": 933,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L367–377; `spec/screwgear/instructions.md` L912–933.

Proof: `stepLoopBallSketch`.

<!-- proof-run: proofkit.Run(csLoopCases, stepLoopBallSketch) -->

After the four bars, for ball `i` in foot order: a sketch named `Loop Ball i` on the Loop Plane,
not deferred.

1. `Bl`, a solid line between two reference points, `foot_i - (ringWire/2)*ê` (start) and
   `foot_i + (ringWire/2)*ê` (end), sharing both.
2. `arc = sketch.sketchCurves.sketchArcs.addByThreePoints(bl.startSketchPoint, through,
   bl.endSketchPoint)`, sharing Bl's two points, through `through = foot_i + (ringWire/2)*k̂`
   mapped in with z set to 0.
3. Both of Bl's points `isFixed = True`, then
   `sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, bl)`: with its ends fixed the
   arc keeps one freedom, its bulge, and the coincident puts its centre on the chord, where the
   half-disc's centre is; the bulge's side is the seed's.
4. Raise unless fully constrained. The profile is the half-disc,
   `find_profile_by_curve_counts(sketch, lines=1, arcs=1)` ([PB-PROFILE-MATCH]).

## S26 `[GO]` Loop Ball revolve, per ball

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.RevolveFeatures",
      "reason": null,
      "receiver": "revolveFeatures",
      "role": "required",
      "span": "revolveFeatures.createInput(profile, bl, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))"
    },
    {
      "condition": null,
      "name": "setAngleExtent",
      "owner": "adsk.fusion.RevolveFeatureInput",
      "reason": null,
      "receiver": "revolveInput",
      "role": "required",
      "span": "revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))"
    }
  ],
  "citations": [
    {
      "first": 367,
      "last": 377,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L367–377.

Proof: `stepLoopBallRevolve`, checked by `checkLoopBallRevolve`.

<!-- proof-run: proofkit3d.RunSolid(csLoopPieceCases, stepLoopBallRevolve, checkLoopBallRevolve) -->

Right after each ball's sketch: the half-disc revolved a full turn about `Bl`,
`revolveFeatures.createInput(profile, bl, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`revolveInput.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))`, add; raise
unless one body. It is a sphere of diameter
`ringWire` centred on the foot, kept as a new body. A pipe along the bars is not used: its
mitred corners reach past a ball ([PB-PIPE-CORNER]).

## S27 `[GO]` Loop join

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "combine.bodies",
      "role": "required",
      "span": "combine.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 381,
      "last": 392,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 1096,
      "last": 1104,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/fusion.md` L381–392; `spec/screwgear/instructions.md` L1096–1104.

Proof: `stepLoopJoin`, checked by `checkLoopJoin`.

<!-- proof-run: proofkit3d.RunSolid(csLoopJoinCases, stepLoopJoin, checkLoopJoin) -->

One join into `self.cageBody` with eight tools, the four bars then the four balls, in an
`ObjectCollection`, `JoinFeatureOperation`, `isKeepToolBodies = False`. Raise naming the group
`loop` and the count unless `combine.bodies.count` is 1 ([PB-EMPTY-RESULT]); the cage is `combine.bodies.item(0)`.
Each bar and ball meets a rod at its foot, where the rod's bottom end stands.

## S28 `[PROSE]` Collar planes

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 934,
      "last": 946,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 224,
      "last": 227,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 728,
      "last": 734,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L934–946; `spec/screwgear/fusion.md` L224–227; `spec/screwgear/instructions.md` L728–734.

For each collar, in the rod search's order (gear A `-R`, gear A `+R`, gear B `-R`, gear B
`+R`), before its sketch and sweep: `planeInput.setByDistanceOnPath(line,
adsk.core.ValueInput.createByReal(0))` on a fresh `constructionPlanes.createInput()`, `line` being
`self.pathLines[g]['collar-']` for a collar at `-cageRadius` and `['collar+']` at `+cageRadius`,
passed directly ([PB-CONSTRUCTION-PLANES]); `constructionPlanes.add(planeInput)`, named
`Gear A Collar -R Plane`, `Gear A Collar +R Plane`, `Gear B Collar -R Plane` or `Gear B Collar +R
Plane`. It stands square to the axis at the line's start, station `sc - collarHalf`, where the
line pierces it at `origin_g + (sc - collarHalf)*dir_g`. The engines build no plane; the proof
draws each section on the plane square to the axis at that station.

## S29 `[GO]` Collar sketch, per collar

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(l1, k)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(k, l1, textPoint)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(e, l2)"
    },
    {
      "condition": null,
      "name": "addPerpendicular",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addPerpendicular(l2, k)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addParallel(l4, l2)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addOffsetDimension(l2, l4, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addAngularDimension(ru, l2, textPoint)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addParallel(oi, li)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addOffsetDimension(li, oi, textPoint)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addCoincident(oi.startSketchPoint, l_prev)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "addCoincident(oi.endSketchPoint, l_next)"
    },
    {
      "condition": null,
      "name": "addTangent",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addTangent(oi, arc_i)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "find_profile_by_curve_counts(sketch, lines=4, arcs=4)"
    }
  ],
  "citations": [
    {
      "first": 946,
      "last": 951,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 962,
      "last": 999,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 569,
      "last": 589,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 224,
      "last": 239,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L946–951; `spec/screwgear/instructions.md` L962–999; `spec/screwgear/instructions.md` L569–589; `spec/screwgear/fusion.md` L224–239.

Proof: `stepCollarSketch`.

<!-- proof-run: proofkit.Run(csCollarCases, stepCollarSketch) -->

On the collar's plane, a sketch named `Gear A Collar -R` (and so on), drawn with
`sketch.isComputeDeferred = True` from just after naming to just before the gate ([SCREW-F-DEFER]).
With `st = sc - collarHalf`, `theta = theta(g, st)`, `uB = -(W/2 + clearance)`,
`uF = W/2 + clearance`, `hv = T/2 + clearance`, `w = collarWall`, every seed is
`point(g, st, u, v)` mapped in with z set to 0 ([PB-SEED-NEAR], [PB-SKETCH-ZERO-Z]). The
**rectangle scheme**:

1. **References.** Reference points `O = origin_g + st*dir_g` and `Cp = O + (A/2)*û_g`. The
   construction line `Ru` from `O` to `Cp`, sharing both (`isConstruction = True`).
2. **Spine.** The construction line `K` from `O` to a new point `E` seeded at `point(g, st, uF,
   0)`, sharing `O`. Then `O` and `Cp` `isFixed = True`, before any dimension.
3. **Rectangle.** `L1` from `(uB,-hv)` to `(uF,-hv)`, `L2` on to `(uF,hv)`, `L3` on to `(uB,hv)`,
   `L4` back to the start, sharing corners (no coincident on a corner), all four
   `isConstruction = True` in a collar sketch.
4. **Rows.** `sketch.sketchDimensions.addDistanceDimension(o, e,
   adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)` from `O` to `E`,
   value `uF`; `sketch.geometricConstraints.addParallel(l1, k)` and
   `sketch.sketchDimensions.addOffsetDimension(k, l1, textPoint)` value `hv`; the same for `L3`,
   value `hv`, on the other side; `sketch.geometricConstraints.addCoincident(e, l2)`;
   `sketch.geometricConstraints.addPerpendicular(l2, k)`; `addParallel(l4, l2)` and
   `addOffsetDimension(l2, l4, textPoint)` value `uF - uB` ([PB-OFFSET-DIM],
   [PB-NO-OVERCONSTRAIN]). Every dimension's value is a magnitude and its side is the seed's
   ([PB-DIM-VALUE-SEMANTICS]); each offset's text point is the midpoint of the two lines' seeds.
5. **Angle** ([PB-ANGULAR-DIM]). When `|sin(theta)| >= sqrt(1/2)`: `addAngularDimension(ru, k,
   textPoint)` with value `acos(cos(theta))`, the angle between the rays `O→Cp` and `O→E`, its
   text point `O + (uF/2)*b` with `b` the unit bisector of the two rays. Otherwise
   `addAngularDimension(ru, l2, textPoint)` with value `acos(-sin(theta))`, the angle between
   `Ru`'s direction `O→Cp` and `L2`'s start→end direction, measured from the point
   `X = O + (uF/cos(theta))*û_g` where their lines meet, its text point `X + (uF/2)*b` with `b` the
   unit bisector of those two directions. Either way the value lies in 45°–135°.
6. **Outline.** Four solid sides `O1..O4`, `Oi` seeded `w` outside `Li` and running the way `Li`
   does, from the neighbouring side's line to the next one's: `O1` from `(uB,-hv-w)` to
   `(uF,-hv-w)`, `O2` from `(uF+w,-hv)` to `(uF+w,hv)`, `O3` from `(uF,hv+w)` to `(uB,hv+w)`, `O4`
   from `(uB-w,hv)` to `(uB-w,-hv)`. For each: `addParallel(oi, li)` and
   `addOffsetDimension(li, oi, textPoint)` value `w`; `addCoincident(oi.startSketchPoint, l_prev)`
   and `addCoincident(oi.endSketchPoint, l_next)` on the neighbouring construction sides' infinite
   lines. Four arcs, `arc_i = sketch.sketchCurves.sketchArcs.addByThreePoints(oi.endSketchPoint,
   through, o_next.startSketchPoint)` through the construction corner pushed out by `w` along its
   diagonal, and one `sketch.geometricConstraints.addTangent(oi, arc_i)` each. The arc's centre
   then falls on the corner with radius `w`; neither is dimensioned, and a second tangent would
   over-constrain.
7. `isComputeDeferred = False`; raise unless fully constrained; the profile is
   `find_profile_by_curve_counts(sketch, lines=4, arcs=4)` — the rounded rectangle; the
   construction lines bound nothing ([SCREW-F-TWISTED-SLOT]).

Nothing is projected, and `intersectWithSketchPlane` is not called: `O` is the path's own
point at `st` ([SCREW-F-REFERENCES]).

## S30 `[GO]` Collar sweep, per collar

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "sweepFeatures",
      "role": "required",
      "span": "sweep = sweepFeatures.add(sweepInput)"
    }
  ],
  "citations": [
    {
      "first": 940,
      "last": 960,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1078,
      "last": 1086,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 241,
      "last": 258,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 277,
      "last": 284,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L940–960; `spec/screwgear/instructions.md` L1078–1086; `spec/screwgear/fusion.md` L241–258; `spec/screwgear/fusion.md` L277–284.

Proof: `stepCollarSweep`, checked by `checkCollarSweep`.

<!-- proof-run: proofkit3d.RunSolid(csCollarBodyCases, stepCollarSweep, checkCollarSweep) -->

Right after each collar's sketch: `path =
self.designOcc.component.features.createPath(line, False)` on the collar's Paths line,
`sweepInput = sweepFeatures.createInput(profile, path,
adsk.fusion.FeatureOperations.NewBodyFeatureOperation)` with `sweepFeatures =
self.designOcc.component.features.sweepFeatures`, `sweepInput.twistAngle =
adsk.core.ValueInput.createByReal(2*collarHalf/Lambda)` — positive, 43.64° at the defaults — and
nothing else set: not `orientation`, not `solidTwistAxis`, no rail ([PB-SWEEP-TWIST],
[SCREW-F-NO-SOLID-TWIST]); `sweep = sweepFeatures.add(sweepInput)`. Raise unless one body. It is
a solid of ten faces and sixteen vertices.

**The sense is checked, not assumed** ([SCREW-F-SWEEP-CHECK], [PB-SELF-DIAGNOSING]). Read the
body's `vertices`, each `vertex.geometry` a world point. At the near station `sc - collarHalf`
and the far station `sc + collarHalf`, the eight outline points `(uB, -hv-w)`, `(uF, -hv-w)`,
`(uF+w, -hv)`, `(uF+w, hv)`, `(uF, hv+w)`, `(uB, hv+w)`, `(uB-w, hv)`, `(uB-w, -hv)`, each as
`point(g, station, u, v)`, must each lie within 0.005 cm (0.05 mm) of some vertex; raise naming
the collar, the end and the worst distance otherwise. Measured with the right sense 0.0047 mm
at the far end, 4.9 mm with the wrong one.

The proof has no twisted sweep; it stands in a loft through the section, chorded, at
`ceil(turn/step) + 1` stations — 10 for the collar's 43.64°, the step being the smaller of 5° and
`2*acos(1 - 0.04*clearance/R)` — and holds the same sixteen outline points as vertices of it.

## S31 `[GO]` Collars join

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 1001,
      "last": 1004,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1096,
      "last": 1104,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 381,
      "last": 392,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1001–1004; `spec/screwgear/instructions.md` L1096–1104; `spec/screwgear/fusion.md` L381–392.

Proof: `stepCollarsJoin`, checked by `checkCollarsJoin`.

<!-- proof-run: proofkit3d.RunSolid(csCollarBodyCases, stepCollarsJoin, checkCollarsJoin) -->

After the four collars: one join into `self.cageBody` with the four collars as tools,
`JoinFeatureOperation`, `isKeepToolBodies = False`; raise naming the group `collars` and the
count unless `combine.bodies.count` is 1 ([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]). What joins each collar is its rod, running through the
collar's wall at station `sp` (S03); nothing else is added.

## S32 `[PROSE]` Bore planes

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))"
    }
  ],
  "citations": [
    {
      "first": 1035,
      "last": 1041,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 224,
      "last": 227,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1035–1041; `spec/screwgear/fusion.md` L224–227.

For each bore, in the collars' order, before its sketch and cut: a plane
`setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))` on the bore's Paths line
(`'bore-'` or `'bore+'`), passed directly ([PB-CONSTRUCTION-PLANES]), added and named
`Gear A Bore -R Plane` and so on. It stands at station
`sc - collarHalf - 1 mm`.

## S33 `[GO]` Bore sketch, per bore

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "find_profile_by_curve_counts(sketch, lines=4)"
    }
  ],
  "citations": [
    {
      "first": 962,
      "last": 999,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1035,
      "last": 1041,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L962–999; `spec/screwgear/instructions.md` L1035–1041.

Proof: `stepBoreSketch`.

<!-- proof-run: proofkit.Run(csCollarCases, stepBoreSketch) -->

On the bore's plane, a sketch named `Gear A Bore -R` (and so on), deferred as in S29
([PB-SKETCH-DEFER]): the
rectangle scheme of S29 steps 1–5 at `st = sc - collarHalf - 1 mm`, with the same `uB`, `uF` and
`hv`, the rectangle's four lines **solid** and no outline. Raise unless fully constrained; the
profile is `find_profile_by_curve_counts(sketch, lines=4)` ([PB-PROFILE-MATCH]), the one loop
of four lines, 15.9 by
4.65 mm at the defaults.

## S34 `[GO]` Bore cut, per bore

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "path = createPath(boreLine, False)"
    }
  ],
  "citations": [
    {
      "first": 1041,
      "last": 1053,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1086,
      "last": 1094,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 286,
      "last": 296,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1041–1053; `spec/screwgear/instructions.md` L1086–1094; `spec/screwgear/fusion.md` L286–296.

Proof: `stepBoreCut`, checked by `checkBoreCut`.

<!-- proof-run: proofkit3d.RunSolid(csCollarBodyCases, stepBoreCut, checkBoreCut) -->

Right after each bore's sketch: `path = createPath(boreLine, False)`,
`sweepInput = sweepFeatures.createInput(profile, path,
adsk.fusion.FeatureOperations.CutFeatureOperation)`, `sweepInput.twistAngle =
adsk.core.ValueInput.createByReal(2*(collarHalf + 0.1)/Lambda)` (the millimetre is 0.1 cm; 58.18°
at the defaults), `sweepInput.participantBodies = [self.cageBody]` before `add` so that the
ribbons running through the channel are left whole, and nothing else set ([PB-SWEEP-TWIST]); `sweep =
sweepFeatures.add(sweepInput)`. Raise unless `sweep.bodies.count` is 1; `sweep.bodies.item(0)` is
the cage from then on. One sweep per bore, spanning the collar plus a millimetre each end, so the
cut runs clean through the collar's ends and through whatever of its rod reaches the channel.

**The channel is checked open**: at `s = sc + collarHalf/2`, with `û(s) = cos(theta(g, s))*û_g +
sin(theta(g, s))*v̂_g`, the two probes `origin_g + s*dir_g ± (W/2 + clearance/2)*û(s)` as
`Point3D`s must each give `self.cageBody.pointContainment(probe) ==
adsk.fusion.PointContainment.PointOutsidePointContainment`; raise naming the bore and which
probe otherwise ([PB-SELF-DIAGNOSING]).

## S35 `[PROSE]` Relocate the bodies and hide construction geometry

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "moveToComponent",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "self.gearBodies[0].moveToComponent(self.gearOccs[0])"
    },
    {
      "condition": null,
      "name": "moveToComponent",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": null,
      "role": "required",
      "span": "self.gearBodies[1].moveToComponent(self.gearOccs[1])"
    },
    {
      "condition": null,
      "name": "moveToComponent",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "self.cageBody",
      "role": "required",
      "span": "self.cageBody.moveToComponent(self.cageOcc)"
    },
    {
      "condition": null,
      "name": "hide_construction_geometry",
      "owner": null,
      "reason": null,
      "receiver": "solids",
      "role": "required",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    }
  ],
  "citations": [
    {
      "first": 1106,
      "last": 1112,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1106–1112.

`relocateBodies`: name the cage body `Cage` (`self.cageBody.name = 'Cage'`), then
`self.gearBodies[0].moveToComponent(self.gearOccs[0])`,
`self.gearBodies[1].moveToComponent(self.gearOccs[1])` and
`self.cageBody.moveToComponent(self.cageOcc)`, which keep world position and need no activation
([PB-NO-CROSS-SIBLING]). Then `solids.hide_construction_geometry(self.designOcc.component)`
([PB-TREE-CLEANUP]), which walks `Design` and everything under it. Do not re-implement it, and add
no display-settling call of its own ([PB-SETTLE-DISPLAY]). At the defaults the timeline holds 96
items: 23 sketches, 12 construction planes, 56 features and 5 component creations.
