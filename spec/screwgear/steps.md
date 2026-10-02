The proof for these steps is `proof/screwgear/compiled_model_test.go`,
`proof/screwgear/compiled_sketches_test.go`, `proof/screwgear/compiled_solids_test.go` and the
generated `proof/screwgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `2086fb6863cae62a88990017c3ded021c576e110` |
| `spec/screwgear/fusion.md` | `b303ec403139aeb27b7d5b0631040bf62d0c21c9` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `74d0a88753d50d35e118b61947bdf24349b00a1a` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/screwgear/mesh-search.md` | `394a2e0b52f4f4c40528646bf825f3db577f27ca` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `cdd32545b0f8c651752827c6697601f1e32b4d39` |

## S01 `[PROSE]` Module, classes, constants and the dialog

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "configure",
      "owner": null,
      "reason": "GearCommand in commands/_gear_command.py calls the configurator classmethod configure; no shared framework module defines it",
      "receiver": null,
      "role": "example",
      "span": "configure(cls, command)"
    },
    {
      "condition": null,
      "name": "generate",
      "owner": null,
      "reason": "lib/geargen/base.py declares the abstract generate that the generator implements",
      "receiver": null,
      "role": "inherited",
      "span": "generate(self, inputs)"
    },
    {
      "condition": null,
      "name": "deleteComponent",
      "owner": null,
      "reason": "base.Generator provides deleteComponent for error cleanup",
      "receiver": null,
      "role": "inherited",
      "span": "deleteComponent()"
    },
    {
      "condition": null,
      "name": "prefixBase",
      "owner": null,
      "reason": "lib/geargen/base.py defines prefixBase, which the generator overrides",
      "receiver": null,
      "role": "inherited",
      "span": "prefixBase()"
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
      "span": "group.children.addValueInput(inputId, label, unit, adsk.core.ValueInput.createByReal(value))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "group.children.addValueInput(inputId, label, unit, adsk.core.ValueInput.createByReal(value))"
    },
    {
      "condition": null,
      "name": "addSelectionInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "command.commandInputs",
      "role": "required",
      "span": "command.commandInputs.addSelectionInput(inputId, label, tooltip)"
    },
    {
      "condition": null,
      "name": "setSelectionLimits",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selection",
      "role": "required",
      "span": "selection.setSelectionLimits(1, 1)"
    },
    {
      "condition": null,
      "name": "addSelectionFilter",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selection",
      "role": "required",
      "span": "selection.addSelectionFilter(...)"
    },
    {
      "condition": null,
      "name": "addSelection",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selection",
      "role": "required",
      "span": "selection.addSelection(get_design().rootComponent)"
    },
    {
      "condition": null,
      "name": "get_design",
      "owner": null,
      "reason": "lib/geargen/misc.py provides get_design",
      "receiver": null,
      "role": "inherited",
      "span": "selection.addSelection(get_design().rootComponent)"
    }
  ],
  "citations": [
    {
      "first": 475,
      "last": 505,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 526,
      "last": 597,
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
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L475–505; `spec/screwgear/instructions.md` L526–597; `.claude/skills/generate-gear/PLAYBOOK.md` L128–143; `.claude/skills/generate-gear/PLAYBOOK.md` L355–357.

The module `lib/geargen/screwgear.py` defines exactly two public classes, exported through
`lib/geargen/__init__.py`, and subclasses no gear class: it shares no involute math.

- `ScrewGearCommandInputsConfigurator`, with a classmethod `configure(cls, command)` that adds the
  dialog inputs below, in the order below. No input has conditional visibility.
- `ScrewGearGenerator(base.Generator)`, constructed with the one argument `design` (inherited),
  implementing `generate(self, inputs)` and the call graph of S04, relying on the inherited
  `deleteComponent()` for error cleanup, and overriding `prefixBase()` to return `'ScrewGear'`.

Imports: `math`, `adsk.core`, `adsk.fusion`, `futil` (`from ...lib import fusion360utils as futil`),
`get_design` from `.misc`, `Generator` and `get_selection` from `.base`,
`find_profile_by_curve_counts` from `.utilities`, and `solids` from `.` — nothing else, and no
`import *` ([PB-PRECOMPUTED-MODE]: no user parameters are registered, so `get_value` and
`addParameter` are not used). The command entry `commands/screwgear/entry.py` constructs
`GearCommand(gear_type='ScrewGear', name='Screw Gear Generator', …)` binding these two classes by
name (PLAYBOOK "Command-entry wiring").

Module-level constants, one per input id, and one more:

```
INPUT_ID_PLANE = 'plane'                 INPUT_ID_POINT = 'point'
INPUT_ID_PARENT = 'parent'               INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'
INPUT_ID_TOOTH_COUNT = 'toothCount'      INPUT_ID_TWIST_LEAD = 'twistLead'
INPUT_ID_RIBBON_THICKNESS = 'ribbonThickness'
INPUT_ID_TOOTH_PITCH = 'toothPitch'      INPUT_ID_TOOTH_HEIGHT = 'toothHeight'
INPUT_ID_CAGE_RADIUS = 'cageRadius'      INPUT_ID_CAGE_RISE = 'cageRise'
INPUT_ID_CLEARANCE = 'clearance'         INPUT_ID_ROOF_ALLOWANCE = 'roofAllowance'
INPUT_ID_COLLAR_HALF = 'collarHalf'      INPUT_ID_COLLAR_WALL = 'collarWall'
INPUT_ID_CROSS_ANGLE = 'crossAngle'      INPUT_ID_ENGAGEMENT = 'engagement'
INPUT_ID_MOUNT_ANGLE_A = 'mountAngleA'   INPUT_ID_MOUNT_ANGLE_B = 'mountAngleB'
INPUT_ID_ASSEMBLY_PHASE = 'assemblyPhase'
CELL_TEETH = 4
```

**The dialog, in this exact order** ([PB-AUTOFOCUS-FIRST]: the plane selection is added first so
the dialog opens on it). The three selections are top-level inputs of `command.commandInputs`.
Then three groups, each made with `command.commandInputs.addGroupCommandInput(groupId, groupLabel)`;
a group's inputs are added to that group's `group.children`, not to the top level.
`ribbonGroup` and `frameGroup` keep `isExpanded = True`; `meshGroup` is set `isExpanded = False`
(its values come from the meshing search and jam when changed carelessly).

| # | Group id, group label | Dialog label | Input id | Unit string | Default (display) | `createByReal` value (internal) |
|---|---|---|---|---|---|---|
| 1 | top level | Target Plane | `plane` | selection | — | — |
| 2 | top level | Centre Point | `point` | selection | — | — |
| 3 | top level | Parent Component | `parent` | selection | — | — |
| 4 | `ribbonGroup`, `Ribbon` | Ribbon Width | `ribbonWidth` | `mm` | 15 mm | 1.5 |
| 5 | `ribbonGroup`, `Ribbon` | Tooth Count | `toothCount` | `''` | 68 | 68 |
| 6 | `ribbonGroup`, `Ribbon` | Twist Lead | `twistLead` | `mm` | 49.5 mm | 4.95 |
| 7 | `ribbonGroup`, `Ribbon` | Ribbon Thickness | `ribbonThickness` | `mm` | 3.75 mm | 0.375 |
| 8 | `ribbonGroup`, `Ribbon` | Tooth Pitch | `toothPitch` | `mm` | 2.625 mm | 0.2625 |
| 9 | `ribbonGroup`, `Ribbon` | Tooth Height | `toothHeight` | `mm` | 2.625 mm | 0.2625 |
| 10 | `frameGroup`, `Frame` | Cage Radius | `cageRadius` | `mm` | 15 mm | 1.5 |
| 11 | `frameGroup`, `Frame` | Cage Rise | `cageRise` | `mm` | 18.75 mm | 1.875 |
| 12 | `frameGroup`, `Frame` | Clearance | `clearance` | `mm` | 0.20 mm | 0.02 |
| 13 | `frameGroup`, `Frame` | Roof Allowance | `roofAllowance` | `mm` | 0.30 mm | 0.03 |
| 14 | `frameGroup`, `Frame` | Collar Half Length | `collarHalf` | `mm` | 3 mm | 0.3 |
| 15 | `frameGroup`, `Frame` | Collar Wall | `collarWall` | `mm` | 3 mm | 0.3 |
| 16 | `meshGroup`, `Mesh (from the mesh search)` | Crossing Angle | `crossAngle` | `deg` | 80° | radians(80) |
| 17 | `meshGroup`, `Mesh (from the mesh search)` | Engagement | `engagement` | `mm` | 0.90 mm | 0.09 |
| 18 | `meshGroup`, `Mesh (from the mesh search)` | Mounting Angle A | `mountAngleA` | `deg` | 14° | radians(14) |
| 19 | `meshGroup`, `Mesh (from the mesh search)` | Mounting Angle B | `mountAngleB` | `deg` | 14° | radians(14) |
| 20 | `meshGroup`, `Mesh (from the mesh search)` | Assembly Phase | `assemblyPhase` | `mm` | −1.30 mm | −0.13 |

Each value input is `group.children.addValueInput(inputId, label, unit, adsk.core.ValueInput.createByReal(value))`
with the value of the last column ([PB-DIALOG-DEFAULT-UNITS]: cm for a length, radians for an
angle, the bare count for `toothCount`).

Each selection input is `command.commandInputs.addSelectionInput(inputId, label, tooltip)`, then
its filters as named constants ([PB-SELECTION-FILTER-ENUM]), then
`selection.setSelectionLimits(1, 1)` ([PB-SELECTION-DECL]):

| Input id | Filters (`selection.addSelectionFilter(...)` each) | Tooltip, verbatim |
|---|---|---|
| `plane` | `adsk.core.SelectionCommandInput.ConstructionPlanes`, `adsk.core.SelectionCommandInput.PlanarFaces` | `Plane the cage's axis is normal to` |
| `point` | `adsk.core.SelectionCommandInput.ConstructionPoints`, `adsk.core.SelectionCommandInput.SketchPoints` | `Centre of the mechanism` |
| `parent` | `adsk.core.SelectionCommandInput.Occurrences`, `adsk.core.SelectionCommandInput.RootComponents` | `Component the mechanism is created under` |

The `parent` input pre-selects the root component with `selection.addSelection(get_design().rootComponent)`.

## S02 `[PROSE]` processInputs: read, check and derive

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
      "name": "get_selection",
      "owner": null,
      "reason": "lib/geargen/base.py provides get_selection",
      "receiver": null,
      "role": "inherited",
      "span": "get_selection(inputs, INPUT_ID_PARENT)"
    },
    {
      "condition": null,
      "name": "get_selection",
      "owner": null,
      "reason": "lib/geargen/base.py provides get_selection",
      "receiver": null,
      "role": "inherited",
      "span": "get_selection(inputs, INPUT_ID_PLANE)"
    },
    {
      "condition": null,
      "name": "get_selection",
      "owner": null,
      "reason": "lib/geargen/base.py provides get_selection",
      "receiver": null,
      "role": "inherited",
      "span": "get_selection(inputs, INPUT_ID_POINT)"
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
      "span": "design.unitsManager.evaluateExpression(valueInput.expression, unit)"
    }
  ],
  "citations": [
    {
      "first": 526,
      "last": 670,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1023,
      "last": 1031,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1070,
      "last": 1093,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L526–670; `spec/screwgear/instructions.md` L1023–1031; `spec/screwgear/instructions.md` L1070–1093.

`processInputs(inputs)` runs first in `generate` and before anything creates the occurrence
([PB-SELECTION-STASH]).

**Selections first.** Read the three selections with `get_selection(inputs, INPUT_ID_PARENT)`,
`get_selection(inputs, INPUT_ID_PLANE)` and `get_selection(inputs, INPUT_ID_POINT)`; raise naming
the id when a selection does not hold exactly one entity. The parent is an `Occurrence` (take its
`component`) or a `Component`; store it as `self.parentComponent`. Store the plane as
`self.plane` and the point as `self.anchorPoint`.

**Values.** Every input is looked up by id on the command's top-level inputs with
`inputs.itemById(inputId)` — grouped inputs included, since ids are unique across the command —
and `processInputs` raises naming the id if a lookup returns `None`. Each value is read with
`design.unitsManager.evaluateExpression(valueInput.expression, unit)` ([PB-EVAL-EXPRESSION]),
`unit` being `'mm'` for a length, `'deg'` for an angle and `''` for `toothCount`; the result is
internal units, cm and **radians**. Convert to millimetres (×10) and keep angles in radians: the
two searches of S03 and every check below work in millimetres, and every length handed to Fusion
afterwards is divided by 10 ([PB-PRECOMPUTED-MODE]). For the messages, degrees are
`math.degrees` of the radians.

**Range checks, in this order, each raising a clear error naming the field and the bound:**

1. `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `collarHalf`, `collarWall` and
   `clearance` must each be `> 0`.
2. `roofAllowance` must be `>= 0` (zero cuts every bore to the clearance alone).
3. `toothCount` must be a whole number `>= 4`: refuse a value whose distance from its nearest
   integer exceeds 1e-9, then use that integer as `N`.
4. `toothHeight` must be `> 0` and `< ribbonWidth/2`.
5. `engagement` must be `> 0` and `<= toothHeight`.
6. `crossAngle` must lie strictly between 0° and 180°.
7. `assemblyPhase` must lie strictly within `±toothPitch`.
8. Both bores, each cut a millimetre past the wall, lie within the ribbon:
   `cageRadius + collarHalf + 1 < toothCount*toothPitch/2` (mm); the message names `cageRadius`.

`twistLead` has no upper bound and neither mounting angle has any range; do not add one.

**Derived values** (millimetres and radians; `W` ribbonWidth, `T` ribbonThickness, `P`
toothPitch, `H` toothHeight, `N` toothCount, `Sigma` crossAngle):

```
Lambda = twistLead / (2*pi)                       7.8782 mm per radian at the defaults
A      = W - engagement                           14.10, the distance between the axes
n      = max(ceil((P/Lambda) / radians(2)), 8)    steps to the tooth, 10 at the defaults
c      = min(CELL_TEETH, N)                       teeth in the cell, 4
q      = N // c,  r = N % c                       17 whole cells and no remainder
L      = N*P                                      178.5, the ribbon's length
Ri     = cageRadius - collarHalf                  12
Ro     = cageRadius + collarHalf                  18
hw     = W/2 + clearance                          7.70, the bore's half-width
ht     = T/2 + clearance                          2.075, the bore's half-thickness
a      = roofAllowance                            0.30
cc     = hypot(hw, ht + a)                        8.058, the furthest any bore corner stands from its axis
sIn    = sqrt(Ri^2 - cc^2) - 1                    7.892, where a +R bore's cut starts
sOut   = Ro + 1                                   19, where it ends
axialWindow = 1.5*sqrt(W^2 - A^2)/sin(Sigma)      7.79
```

(`cc` is the spec's `c` of §4; it is renamed here only so that it is not mistaken for the cell's
`c`.) `n` is 10 at the defaults, so the cell has `c*n + 1 = 41` sections.

**The sleeve's four checks, in this order** (each naming the field and the bound; they are
`sleeveRefusal` of the hand-written proof):

1. The channel starts in the hollow, naming `cageRadius`: `sIn > 0`, i.e. `hypot(cc, 1) < Ri`.
2. The mesh stays visible along the axis, naming `cageRadius`:
   `hypot(axialWindow, hypot(W/2, T/2)) + clearance <= Ri` (11.18 against 12 at the defaults).
3. The end faces keep `collarWall`, naming `cageRise`: `cageRise >= A/2 + cc + collarWall`
   (18.11 at the defaults).
4. The wall between neighbouring bores keeps `collarWall`, naming `collarWall`: the separation of
   S03 must be at least `collarWall`; the message names the two bores and the separation
   (5.078 mm across the `-ê` gap at the defaults).

**Each gear's level bore and roof face** (§4 "The roof allowance"). The gears are indexed `g = A`
(`Phi_A` = mountAngleA, `u_dot_n = +1`) and `g = B` (`Phi_B` = mountAngleB, `u_dot_n = -1`), and
`Theta_g(s) = s/Lambda + Phi_g`. For each gear and each bore sign `sigma` in `{-1, +1}` (`-R`,
`+R`):

- `tilt(sigma)`: take `t1 = Theta_g(sigma*(cageRadius - collarHalf))` and
  `t2 = Theta_g(sigma*(cageRadius + collarHalf))`; the tilt is 0 when some `pi/2 + k*pi` (any
  integer `k`) lies between them (inclusive), otherwise the smaller of `|cos t1|` and `|cos t2|`.
- The level bore is the sign with the smaller tilt; on a tie, the `-R` bore. At the defaults both
  gears' level bores are `-R` (tilt 0 against 0.195).
- Its roof is the `+v` face when `-sin(Theta_g(sigma*cageRadius)) * u_dot_n > 0`, else the `-v`
  face. At the defaults gear A's roof is `+v` and gear B's is `-v`.
- Every bore spans `u` from `-hw` to `hw`. A non-level bore spans `v` from `vLo = -ht` to
  `vHi = ht`. The level bore spans `vLo = -ht`, `vHi = ht + a` when its roof is `+v`, and
  `vLo = -ht - a`, `vHi = ht` otherwise (4.45 mm through at the defaults).

The bores are, in the order every later step uses them: gear A `-R`, gear A `+R`, gear B `-R`,
gear B `+R`. A `-R` bore's cut spans stations `[-sOut, -sIn]` of its gear's axis, its profile at
`-sOut`; a `+R` bore's spans `[sIn, sOut]`, its profile at `sIn`.

## S03 `[PROSE]` processInputs: the wall between the bores and the window search

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "log",
      "owner": null,
      "reason": "fusion360utils provides futil.log",
      "receiver": "futil",
      "role": "inherited",
      "span": "futil.log(f'No window facing {d}: {reason}')"
    }
  ],
  "citations": [
    {
      "first": 1203,
      "last": 1232,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1234,
      "last": 1372,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1203–1232; `spec/screwgear/instructions.md` L1234–1372.

Both searches run in `processInputs`, after the four checks' first three and before any feature,
in millimetres, at the step sizes stated below. The Anchor sketch does not exist yet, so they run
in an abstract frame, and every result they hand on is frame-free (a separation, or a window's
corners in its own plane coordinates `(t, z)`): put `C` at the origin, `ê = (1, 0, 0)`,
`k̂ = (0, 1, 0)`, `n̂ = (0, 0, 1)`, and build both gears' axes as S06 does from these. In that frame:

```
dir_A = (cos(Sigma/2),  sin(Sigma/2), 0)    origin_A = (0, 0, -A/2)    u_A = n̂    v_A = dir_A × u_A
dir_B = (cos(Sigma/2), -sin(Sigma/2), 0)    origin_B = (0, 0,  A/2)    u_B = -n̂   v_B = dir_B × u_B
World_g(s, x, y) = origin_g + s*dir_g + x*u_g + y*v_g
turn(u, v, th)   = (u*cos th - v*sin th,  u*sin th + v*cos th)
```

A bore's **crossing** is `origin_g + sigma*cageRadius*dir_g`. Its rectangle's corners, in this
order (counter-clockwise), are `(-hw, vLo)`, `(hw, vLo)`, `(hw, vHi)`, `(-hw, vHi)` with the
bore's own `vLo`, `vHi` (S02).

**The wall between the bores** (`channelSeparation`). Round the tube the bores alternate between
the gears, so the four gaps between neighbours face `+k̂` (gear A `+R` and gear B `-R`), `-ê`
(gear A `-R` and gear B `-R`), `-k̂` (gear A `-R` and gear B `+R`) and `+ê` (gear A `+R` and gear
B `+R`).

1. Sample each bore's outline: at stations `s = sigma*sIn + sigma*0.1*j` for `j = 0, 1, …` while
   `|s| <= sOut`, take 17 points on each side of the rectangle — `(hw, v)` and `(-hw, v)` for
   `v = vLo + (vHi - vLo)*i/16`, and `(u, vHi)` and `(u, vLo)` for `u = -hw + 2*hw*i/16`,
   `i = 0 … 16` — each turned by `Theta_g(s)` and placed at `World_g(s, x, y)`. Keep a point when
   `hypot(P·ê, P·k̂)` lies within `Ri - 0.5` to `Ro + 0.5`.
2. For each gap facing `d`, with `across = n̂ × d`, project the kept points of its two bores to
   `(P·across, P·n̂)` and take each bore's convex hull by Andrew's monotone chain.
3. For every edge of either hull, with `m` the unit vector square to that edge, the separation
   along `m` is the larger of `min(m·q) - max(m·p)` and `min(m·p) - max(m·q)`, `p` over the first
   hull's corners and `q` over the second's. The gap's separation is the largest over all those
   edges (negative when the hulls overlap).
4. The sleeve's fourth check compares the least of the four gaps' separations with `collarWall`.

**The windows** (`newWindow`, the hand-written proof's search, step for step). The windows face
`d = +k̂` and `d = -k̂` when `crossAngle <= 90°`, and `d = +ê` and `d = -ê` past it; the `+` one
first. Each window is found the same way from its own `d`, with `across = n̂ × d`; a point `P` has
plane coordinates `t = P·across`, `z = P·n̂` and depth `a = P·d`. At `t` the wall runs from
`a0(t) = sqrt(max(0, Ri^2 - t^2))` to `a1(t) = sqrt(max(0, Ro^2 - t^2))`.

*A bore's section in the wall* (`sectionInWall`) at station `s` with `|s| < Ro` (none otherwise):
turn the bore's four corners by `Theta_g(s)` to `(x, y)` (x along `u_g`, y along `v_g`). Clip that
polygon to the heights inside the end faces: keep the `x` for which
`(origin_g + x*u_g)·n̂` lies within `±cageRise`. Then clip what is left twice more, once to
`near <= y <= far` and once to `-far <= y <= -near`, with `near = sqrt(max(0, Ri^2 - s^2))` and
`far = sqrt(Ro^2 - s^2)`: those are the section's two **pieces** (either may be empty). Every
clip keeps the part of a convex polygon on one side of a line, walking its edges in order,
keeping each corner on the kept side and adding the point where an edge crosses the line.

*The long sides.* The two bores whose crossings have `crossing·d > 0` **flank** the window; the
other two are its **far** bores. The low flanking bore is the one whose crossing has the smaller
`·n̂`; `lean = +1` when the high one's crossing has the larger `t`, else `-1`. Walk each flanking
bore's cut span at stations every 0.001 from its lower end (`sigma*sIn` … for `+R`, `-sOut` … for
`-R`, i.e. from the span's smaller station up to its larger), and take every corner of every piece
as the point `origin_g + x*u_g + y*v_g + s*dir_g`, with `m = z + lean*t` (`wallCorners`).
`lowReach` is the low bore's largest `m`, `highReach` the high bore's least.
`lo = lowReach + sqrt(2)*collarWall`, `hi = highReach - sqrt(2)*collarWall`.

*The trims.* `zLimit` (`channelTop`): over all four bores, at stations `s = sigma*sIn +
sigma*0.01*j` while `|s| <= sOut`, wherever `|s| <= Ro` and `hypot(s, ymax) >= Ri` with `ymax` the
largest `|u*sin th + v*cos th|` over the bore's four corners `(u, v)` (`th = Theta_g(s)`), take the
largest `|(origin_g)·n̂ + (u*cos th - v*sin th)*(u_g·n̂)|` over those corners; `zLimit` is the
largest over everything (14.54 at the defaults). Then

```
top    = min(2*zLimit - hi, hi + sqrt(2)*Ri)
bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)
```

*The ends* (`windowEnd`). For `delta = +1` (right) and `delta = -1` (left): bisect `te` 24 times
between 0 and `min(Ri, Ro/sqrt(2))*(1 - 1e-9)`, taking the middle and making it the new lower end
when `clear(te)` holds and the new upper end when it does not; the end is the final lower end.
`right = end(+1)`, `left = -end(-1)`. `clear(te)`, at `t = delta*te`:

1. `zLow = max(lo - lean*t, bottom + lean*t)`, `zHigh = min(hi - lean*t, top + lean*t)`; not
   clear when `zLow > zHigh`.
2. `need = collarWall + 0.1/sqrt(2) + 0.005` (3.076 at the defaults).
3. The points, each `t*across + z*n̂ + a*d`: the two corner lines through the wall, at
   `z = zLow` and `z = zHigh`, each at `a = a0(t), a0(t) + 0.1, …` and last `a1(t)` (a step that
   would pass `a1` lands on it); and, on the inner face `a = a0(t)` and on the outer face
   `a = a1(t)`, the upright edge at `z = zLow + (zHigh - zLow)*k/nz` for `k = 1 … nz - 1` with
   `nz = ceil((zHigh - zLow)/0.1)`, and at `z = -zLimit + 0.1*j` for every `j >= 0` with
   `z <= zLimit` and `zLow < z < zHigh`.
4. Not clear as soon as any point's distance to either far bore's channel, by `wallGap` below
   with `reach = need`, is under `need`; clear otherwise.

*The distance to a channel* (`wallGap`) for a point `Q` and one bore of gear `g` with cut span
`[s1, s2]`: `x = (Q - origin_g)·u_g`, `y = (Q - origin_g)·v_g`, `sq = (Q - origin_g)·dir_g`.
When `hypot(hypot(x, y), sq - min(max(sq, s1), s2)) - cc >= reach`, the distance is `reach`.
Otherwise walk the bore's **station table**, built once per bore: stations `s1 + 0.002*k` for
`k = 0, 1, …` while `<= s2`, each with its two pieces; wherever the set of non-empty pieces
differs between two neighbouring stations, add stations every 0.0001 strictly between them; all
in order of `s`. For each piece keep its circle: centre the average of its corners, radius the
largest distance from that to a corner. Start with `best = reach^2`; walk up from the first
station at or above `sq`, then down from the one before it, stopping each way at the first station
with `(sq - s)^2 >= best`. At each station, for each non-empty piece: pass over it when
`o = hypot(x - cx, y - cy) - radius` is positive and `(sq - s)^2 + o^2 >= best`; otherwise set
`best = min(best, (sq - s)^2 + d2)`, `d2` being 0 when `(x, y)` is inside the piece (on the inner
side of every edge of the counter-clockwise polygon; a piece of fewer than three corners never
is) and otherwise the least squared distance from `(x, y)` to its edges. The distance is
`sqrt(best)`.

*The hexagon* (`clipCorners`). Start from the square `-2*Ro <= t, z <= 2*Ro`, corners in the
order `(-2Ro, -2Ro)`, `(2Ro, -2Ro)`, `(2Ro, 2Ro)`, `(-2Ro, 2Ro)`, and clip it, in this order, to
`lean*t + z <= hi`, `-lean*t - z <= -lo`, `-lean*t + z <= top`, `lean*t - z <= -bottom`,
`t <= right` and `-t <= -left`; drop a corner within 0.001 of the one before it (and the last
when it is within 0.001 of the first). At the defaults:

| Window | lo | hi | bottom | top | left | right | Corners `(t, z)` | Area mm² |
|---|---|---|---|---|---|---|---|---|
| `+k` | −3.780 | 6.730 | −20.751 | 22.352 | −11.523 | 11.703 | (11.70, −4.97), (−7.81, 14.54), (−11.52, 10.83), (−11.52, 7.74), (8.49, −12.27), (11.70, −9.05) | 220.0 |
| `-k` | −6.436 | 3.780 | −22.645 | 20.751 | −11.670 | 11.523 | six, by the same clip | 215.1 |

*When a gap has no room* (`room`). A window is not cut when `hi <= lo`, or when `right <= left`,
or when the hexagon has fewer than three corners or no area. The build then logs
`futil.log(f'No window facing {d}: {reason}')` ([PB-LOGGING]), `{d}` being `+k`, `-k`, `+e` or
`-e` and `{reason}` naming which of those held, and builds the sleeve without that window. This
refuses nothing.

Store `self.windows` as a list of zero to two entries, each the facing direction's name and unit
vector in the abstract frame, its `across`, and its corner list `(t, z)` in mm. The frame-free
results carry over to the real frame of S06 unchanged: a corner `(t, z)` is the world point
`C + t*across + z*n̂` with `across` and `d` rebuilt from the real `ê`, `k̂`, `n̂`.

## S04 `[PROSE]` generate and the component tree

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "generate",
      "owner": null,
      "reason": "lib/geargen/base.py declares the abstract generate that the generator implements",
      "receiver": null,
      "role": "inherited",
      "span": "generate(self, inputs)"
    },
    {
      "condition": null,
      "name": "processInputs",
      "owner": null,
      "reason": null,
      "receiver": "self",
      "role": "required",
      "span": "self.processInputs(inputs)"
    },
    {
      "condition": null,
      "name": "hide_construction_geometry",
      "owner": null,
      "reason": "lib/geargen/solids.py provides hide_construction_geometry",
      "receiver": "solids",
      "role": "inherited",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    },
    {
      "condition": null,
      "name": "getOccurrence",
      "owner": null,
      "reason": "base.Generator provides getOccurrence, which creates the top occurrence",
      "receiver": "self",
      "role": "inherited",
      "span": "self.getOccurrence()"
    },
    {
      "condition": null,
      "name": "deleteComponent",
      "owner": null,
      "reason": "base.Generator provides deleteComponent for error cleanup",
      "receiver": null,
      "role": "inherited",
      "span": "deleteComponent()"
    },
    {
      "condition": null,
      "name": "addNewComponent",
      "owner": "adsk.fusion.Occurrences",
      "reason": null,
      "receiver": "component.occurrences",
      "role": "required",
      "span": "component.occurrences.addNewComponent(adsk.core.Matrix3D.create())"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "component.occurrences.addNewComponent(adsk.core.Matrix3D.create())"
    }
  ],
  "citations": [
    {
      "first": 475,
      "last": 524,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 743,
      "last": 757,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L475–524; `spec/screwgear/instructions.md` L743–757.

`generate(self, inputs)` runs, in this order: `self.processInputs(inputs)`,
the method `buildComponentTree`, the method `buildAnchor`, the method `buildGear` for gear index 0
and then for gear index 1, the method `buildCage`, the method `relocateBodies`, then
`solids.hide_construction_geometry(self.designOcc.component)`. The method `buildGear`, given a
gear index, runs the methods `buildSweepPaths`, `buildToothCell` and `repeatCellByDoubling` for
that index, in that order.

**No generation context** — handles live on `self`: `self.designOcc`, `self.gearOccs` (two),
`self.cageOcc`, `self.gearBodies` (two), `self.cageBody`, `self.pathLines` (two dicts, one per gear,
keyed `'bore-'` and `'bore+'`, each the sketch line of S07 its bore's sweep runs along) and
`self.windows` (S03).

**The tree** ([PB-OCCURRENCE-TREE]). The top occurrence is the inherited `self.getOccurrence()`,
created under `self.parentComponent`; name its component `Screw Gearing` (it is what the inherited
`deleteComponent()` deletes on failure; never `addNewComponent` it yourself). Under it, four
children, each `component.occurrences.addNewComponent(adsk.core.Matrix3D.create())` with its
component named, in this order: `Design` (`self.designOcc`; every sketch, plane and feature of
this build is made in its component), `Gear A`, `Gear B` (`self.gearOccs`) and `Cage`
(`self.cageOcc`). The last three stay empty until S23.

**Never** activate an occurrence ([PB-NEVER-ACTIVATE]); every collection is called on the
non-activated `Design` component. Sketches go directly on the user-selected plane
([PB-USE-SELECTED-PLANE]); the selected plane is never normalised into a coplanar construction
plane. No construction axis or point is made anywhere ([PB-CONSTRUCTION-NEEDS-ACTIVE]).

## S05 `[GO]` Anchor sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "sketch = component.sketches.add(self.plane)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "projected = sketch.project(self.anchorPoint).item(0)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "line = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(px - 0.5, py, 0), adsk.core.Point3D.create(px + 0.5, py, 0))"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "line = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(px - 0.5, py, 0), adsk.core.Point3D.create(px + 0.5, py, 0))"
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
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDistanceDimension(line.startSketchPoint, line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "textPoint = adsk.core.Point3D.create(px, py + 0.3, 0)"
    }
  ],
  "citations": [
    {
      "first": 802,
      "last": 832,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 498,
      "last": 516,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L802–832; `spec/screwgear/fusion.md` L498–516.

Proof: `stepAnchorSketch`.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

`sketch = component.sketches.add(self.plane)` on the `Design` component, `sketch.name = 'Anchor'`.
This is the one projection in the build ([SCREW-F-REFERENCES]): `projected = sketch.project(self.anchorPoint).item(0)`.
The sketch does not defer computing.

Let `(px, py)` be the projected point's sketch-local position (`projected.geometry`). Draw the
Anchor Line from two raw seeds, 0.5 cm either side along the sketch's own x axis, `z = 0`:
`line = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(px - 0.5, py, 0), adsk.core.Point3D.create(px + 0.5, py, 0))`,
so its end is to the right of its start and it is 10 mm long. Constrain it with exactly these four
things and nothing else:

1. `sketch.geometricConstraints.addCoincident(projected, line)` — the centre lies on the line.
2. `sketch.geometricConstraints.addMidPoint(projected, line)` — and bisects it. Both, not the
   midpoint alone, as the bevel gear's Anchor sketch does; that sketch reads fully constrained in
   Fusion.
3. `sketch.geometricConstraints.addHorizontal(line)` — sketch-local ([PB-REFLINE-DIRECTION]).
4. A horizontal distance from the line's start to its end, value 1.0 cm:
   `sketch.sketchDimensions.addDistanceDimension(line.startSketchPoint, line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
   with `textPoint = adsk.core.Point3D.create(px, py + 0.3, 0)`. Not an aligned dimension: a
   midpoint, a horizontal and an aligned length admit the line end-for-end reversed, and the
   proof's gate refuses that ambiguity; a horizontal start-to-end distance is satisfied only by
   the seeded orientation ([PB-DIM-VALUE-SEMANTICS]: the seed fixes the side, the value is the
   magnitude).

Raise naming `Anchor` unless `sketch.isFullyConstrained` ([PB-FULL-CONSTRAINT]). Only then read
the frame from world geometry ([PB-WORLDGEO-CONSTRAINED], [PB-WORLD-FRAME]): `C` is
`projected.worldGeometry`, and `ê` is the unit vector from `line.startSketchPoint.worldGeometry` to
`line.endSketchPoint.worldGeometry`. Keep `line` as `self.anchorLine`: it is passed once more, to
the Window Plane of S20. No later sketch projects anything.

Proof: the projected point is a reference point (`CreateReferencePoint`), the line is seeded as
above, and the engine's midpoint carries the point-on-line row, so the proof writes the midpoint
alone (as `proof/bevelgear` does) with the signed horizontal distance +10 mm; DOF 0, unambiguous,
at the centre on the origin and away from it.

## S06 `[PROSE]` Gear A Axis Plane and Gear B Axis Plane, and the frame

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "planeInput = component.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-A/2/10))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-A/2/10))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "plane = component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 834,
      "last": 856,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 561,
      "last": 570,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L834–856; `spec/screwgear/fusion.md` L561–570.

Two construction planes on the `Design` component, offset from the selected plane itself
([PB-USE-SELECTED-PLANE], [PB-CONSTRUCTION-PLANES]), in this order:

- `Gear A Axis Plane`: `planeInput = component.constructionPlanes.createInput()`,
  `planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-A/2/10))`,
  `plane = component.constructionPlanes.add(planeInput)`, `plane.name = 'Gear A Axis Plane'`.
- `Gear B Axis Plane`: the same with offset `+A/2/10` cm and the name `Gear B Axis Plane`.

**The sign of `n̂`** ([SCREW-F-NORMAL-SIGN]). Fusion offsets along the selected entity's own
normal, which may point either way. After the Gear A plane is made, read `gA = plane.geometry`
(an `adsk.core.Plane` with `origin` and `normal`); `nrm` is its unit normal. Set `n̂ = nrm` when
`(C - gA.origin)·nrm > 0`, else `n̂ = -nrm`. Then check, for both planes, that
`|(C - origin)·nrm|` is `A/2` (cm) to within 1e-6 cm, and raise naming the plane and the reading
otherwise. No later plane is offset from the selected plane.

**The frame** (all world vectors, cm): `k̂ = n̂ × ê`;

```
dir_A = rotate(ê, +Sigma/2 about n̂) = cos(Sigma/2)*ê + sin(Sigma/2)*k̂     origin_A = C - (A/2)*n̂
dir_B = rotate(ê, -Sigma/2 about n̂) = cos(Sigma/2)*ê - sin(Sigma/2)*k̂     origin_B = C + (A/2)*n̂
u_A = +n̂,  v_A = dir_A × u_A          u_B = -n̂,  v_B = dir_B × u_B
Theta_g(s) = s/Lambda + Phi_g
World_g(s, u, v) = origin_g + s*dir_g + (u*cos Theta_g(s) - v*sin Theta_g(s))*u_g
                                       + (u*sin Theta_g(s) + v*cos Theta_g(s))*v_g
```

`û` points at the other gear, which is what makes the two gears the same part. Every seed and
every reference point below is a `world_g` point or a point of the window planes, computed in
Python and mapped into its sketch ([PB-SEED-NEAR], [PB-SOLVED-GEOMETRY]). Gear A's tooth phase
`Z0_A = 0`; gear B's `Z0_B = assemblyPhase`.

## S07 `[GO]` Gear Paths sketch (per gear)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "sketch = component.sketches.add(axisPlane_g)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "local = sketch.modelToSketchSpace(worldPoint)"
    },
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
      "span": "boreMinus = sketch.sketchCurves.sketchLines.addByTwoPoints(pts[0], pts[1])"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "borePlus = sketch.sketchCurves.sketchLines.addByTwoPoints(pts[2], pts[3])"
    }
  ],
  "citations": [
    {
      "first": 858,
      "last": 877,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 518,
      "last": 548,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L858–877; `spec/screwgear/fusion.md` L518–548.

Proof: `stepPathsSketch`.

<!-- proof-run: proofkit.Run(pathsCases, stepPathsSketch) -->

The method `buildSweepPaths`, given the gear index. For gear `g` (label `Gear A` or `Gear B`):
`sketch = component.sketches.add(axisPlane_g)` on that gear's Axis Plane of S06,
`sketch.name = f'{gearLabel} Paths'`. No deferral.

Four reference points ([SCREW-F-REFERENCES], [PB-PROJECT-NOT-FIXED] (b)), at the world points
`origin_g + s*dir_g` for `s = -sOut, -sIn, sIn, sOut` (cm), each mapped
`local = sketch.modelToSketchSpace(worldPoint)`, then `local.z = 0` ([PB-SKETCH-ZERO-Z]), then
`pt = sketch.sketchPoints.add(local)`. Two solid lines sharing them
([PB-SHARE-XOR-COINCIDENT]): `boreMinus = sketch.sketchCurves.sketchLines.addByTwoPoints(pts[0], pts[1])`
from `-sOut` to `-sIn`, and `borePlus = sketch.sketchCurves.sketchLines.addByTwoPoints(pts[2], pts[3])`
from `sIn` to `sOut`: each drawn from its negative station to its positive one, so its start is
its negative end and it runs along `+dir_g`. Then set `isFixed = True` on all four points (after
the lines, never before). No constraint, no dimension, nothing else.

Raise naming the sketch unless `sketch.isFullyConstrained`. Store
`self.pathLines[index] = {'bore-': boreMinus, 'bore+': borePlus}`.

## S08 `[GO]` Cell Sections sketch (per gear)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "sketch = component.sketches.add(axisPlane_g)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(worldPoint))"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(worldPoint))"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(corner, nextCorner)"
    }
  ],
  "citations": [
    {
      "first": 879,
      "last": 907,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 929,
      "last": 953,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 292,
      "last": 312,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 424,
      "last": 436,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 1459,
      "last": 1463,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L879–907; `spec/screwgear/instructions.md` L929–953; `spec/screwgear/fusion.md` L292–312; `spec/screwgear/fusion.md` L424–436; `spec/screwgear/instructions.md` L1459–1463.

Proof: `stepCellSectionSketch`.

<!-- proof-run: proofkit.Run(cellSectionCases, stepCellSectionSketch) -->

The method `buildToothCell`, given the gear index: its sketch ([SCREW-F-CELL-LOFT], [PB-3D-SKETCH-SECTIONS]).
`sketch = component.sketches.add(axisPlane_g)` on the gear's Axis Plane, `sketch.name =
f'{gearLabel} Cell Sections'`, then `sketch.isComputeDeferred = True` before the first point
([SCREW-F-DEFER], [PB-SKETCH-DEFER]). No section plane is made.

The cell is `c` teeth at the ribbon's negative end: `s0 = Z0_g - L/2` (mm). Section `k`, for
`k = 0 … c*n` (41 sections at the defaults), stands at `s_k = s0 + k*P/n` and is the rectangle
`u` from `uB = -W/2` to `uF = Utooth(s_k)`, `v` from `-T/2` to `+T/2`, where

```
Utooth(s) = W/2 - H/2 + (H/2)*cos(2*pi*(s - Z0_g)/P)
```

Its four corners, in this order, are `World_g(s_k, uB, -T/2)`, `World_g(s_k, uF, -T/2)`,
`World_g(s_k, uF, T/2)`, `World_g(s_k, uB, T/2)` (divided by 10 for cm), each added with
`sketch.sketchPoints.add(sketch.modelToSketchSpace(worldPoint))` **with its z kept**: these points
lie off the sketch's plane on purpose, and [PB-SKETCH-ZERO-Z] does not apply to them. Four solid
lines share them: `L1` corner 1→2, `L2` 2→3, `L3` 3→4, `L4` 4→1, each
`sketch.sketchCurves.sketchLines.addByTwoPoints(corner, nextCorner)`. Keep each section's four
lines together, in order of `k`. After the last line of the last section exists, set every
point's `isFixed = True`. Nothing else: no construction line, constraint or dimension.

Then `sketch.isComputeDeferred = False`; raise naming the sketch unless `sketch.isFullyConstrained`,
and raise with the count unless `sketch.profiles.count` is `c*n + 1`. The profiles are not used.

Proof stand-in: the sketch engine is planar, so each section is its own planar sketch on its
station's plane, four fixed corners and four lines, one valid profile of area
`(Utooth(s_k) + W/2)*T`. Fusion's verdict on the one 3D sketch (fully constrained, one profile
per section, at 11, 41 and 81 sections on 2026-09-28) is the part the proof cannot reach.

## S09 `[GO]` Cell loft (per gear)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "component.features.loftFeatures",
      "role": "required",
      "span": "loftInput = component.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "coll = adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "coll",
      "role": "required",
      "span": "coll.add(line)"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "component.features",
      "role": "required",
      "span": "path = component.features.createPath(coll, False)"
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
      "receiver": "component.features.loftFeatures",
      "role": "required",
      "span": "loft = component.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loft.bodies",
      "role": "required",
      "span": "loft.bodies.item(0).isSolid"
    }
  ],
  "citations": [
    {
      "first": 955,
      "last": 962,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 313,
      "last": 332,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 1464,
      "last": 1467,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L955–962; `spec/screwgear/fusion.md` L313–332; `spec/screwgear/instructions.md` L1464–1467.

Proof: `stepCellLoft`.

<!-- proof-run: proofkit3d.RunSolid(cellLoftCases, stepCellLoft, assertCellLoft) -->

([PB-LOFT], [SCREW-F-CELL-LOFT], [PB-PATH-FROM-SKETCH]) On the `Design` component:
`loftInput = component.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
For each section in order of `k` (station order — the order of the calls is the loft order):
`coll = adsk.core.ObjectCollection.create()`, `coll.add(line)` for its `L1`, `L2`, `L3`, `L4`,
then `path = component.features.createPath(coll, False)` and `loftInput.loftSections.add(path)`.
Never `adsk.fusion.Path.create`, which raises in this component. Set nothing else on the input;
then `loft = component.features.loftFeatures.add(loftInput)`.

Raise with the piece's name and the count unless `loft.bodies.count` is 1 and
`loft.bodies.item(0).isSolid` ([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]). That body is the cell.

Proof stand-in: decad lofts between two sections only, so the cell is one two-section sheet loft
per neighbouring pair, two end patches and a stitch into one solid, held to the helicoid's volume
`T*(W - H/2)*c*P` within the slack of its flat-triangle walls, and to its station span.

## S10 `[PROSE]` The doubling schedule (per gear)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "translate",
      "owner": null,
      "reason": "formula notation for the screw step, not a call",
      "receiver": null,
      "role": "prose",
      "span": "Step(k) = translate(k*P along dir_g) ∘ rotate(k*P/Lambda about the gear's axis)"
    },
    {
      "condition": null,
      "name": "rotate",
      "owner": null,
      "reason": "formula notation for the screw step, not a call",
      "receiver": null,
      "role": "prose",
      "span": "Step(k) = translate(k*P along dir_g) ∘ rotate(k*P/Lambda about the gear's axis)"
    }
  ],
  "citations": [
    {
      "first": 964,
      "last": 995,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1009,
      "last": 1012,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L964–995; `spec/screwgear/instructions.md` L1009–1012.

The method `repeatCellByDoubling`, given the gear index, repeats the cell under the **screw step**
`Step(k) = translate(k*P along dir_g) ∘ rotate(k*P/Lambda about the gear's axis)`, `k` a number
of teeth. It is control flow; the three features of each round are S11, S12 and S13.

`N = q*c + r`. Build the `q` cells by doubling:

- Start with the cell as the body, `m = 1` (cells it holds).
- For each bit of `q` below its top bit, lowest first: if that bit is set, take a copy of the
  body (S11) and keep it unmoved — an **aside** of `m` cells; then do a **round**: copy the body
  (S11), move the copy by `Step(m*c)` (S12), join it to the body (S13); `m` doubles.
- After the last doubling, move each aside, **largest first**, by `Step(m*c)` (S12) and join it
  (S13); add its cells to `m`. The last join brings `m` to `q`.

That is `floor(log2 q)` rounds of doubling plus one join per set bit of `q` below its top bit, 5 at the defaults: `q = 17`, one aside of the
single cell taken before the first doubling, doublings moving the copy by `Step(4)`, `Step(8)`,
`Step(16)`, `Step(32)` to 16 cells, then the aside moved by `Step(64)`. When `q = 1` there is no
round. A screw step is never zero for `k >= 1`; do not add a `k = 0` case
([PB-MOVE-ROTATE]: a zero-angle matrix is rejected).

When `r > 0` the remainder cell (S14, S15) is built after the last round and joined (S13). After
the last join name the body `Gear A` or `Gear B` (`body.name`) and store it in
`self.gearBodies[index]`.

## S11 `[GO]` Copy the body

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "component.features.copyPasteBodies",
      "role": "required",
      "span": "copyFeature = component.features.copyPasteBodies.add(body)"
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
      "first": 1004,
      "last": 1009,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 283,
      "last": 290,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1004–1009; `spec/screwgear/fusion.md` L283–290.

Proof: `stepCopyBody`.

<!-- proof-run: proofkit3d.RunSolid(copyCases, stepCopyBody, assertCopyBody) -->

([SCREW-F-COPY-BODY]) `copyFeature = component.features.copyPasteBodies.add(body)`. It returns a
`CopyPasteBody` feature, not a body: the copy is `copyFeature.bodies.item(0)`. Never read the
feature's `sourceBody`, which is the original. Raise with the count unless
`copyFeature.bodies.count` is 1 ([PB-EMPTY-RESULT]).

Proof stand-in: a copy coincides with its source, which decad's pairwise verification cannot
classify, so the copy is made with `Duplicate` in a document of its own and held to its source's
volume and centroid there; the gated document holds the cell.

## S12 `[GO]` Move the copy by a screw step

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
      "name": "setToRotation",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "rot",
      "role": "required",
      "span": "rot.setToRotation(k*P/Lambda, axisVector, axisPoint)"
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
      "name": "scaleBy",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "mov.translation",
      "role": "required",
      "span": "mov.translation.scaleBy(...)"
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
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "component.features.moveFeatures",
      "role": "required",
      "span": "moveInput = component.features.moveFeatures.createInput2(bodies)"
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
      "receiver": "component.features.moveFeatures",
      "role": "required",
      "span": "component.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 1004,
      "last": 1006,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 246,
      "last": 281,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1004–1006; `spec/screwgear/fusion.md` L246–281.

Proof: `stepScrewMove`.

<!-- proof-run: proofkit3d.RunSolid(moveCases, stepScrewMove, assertScrewMove) -->

([SCREW-F-SCREW-STEP], [PB-MOVE-ROTATE]) For a move by `k` teeth, all lengths in cm, the axis
`axisVector = dir_g` (an `adsk.core.Vector3D`, unit) through `axisPoint = origin_g` (an
`adsk.core.Point3D`):

1. `rot = adsk.core.Matrix3D.create()`, `rot.setToRotation(k*P/Lambda, axisVector, axisPoint)`.
2. `shift = axisVector.copy()`, `shift.scaleBy(k*P)` — build the translation vector first.
3. `mov = adsk.core.Matrix3D.create()`, `mov.translation = shift` — assigned whole, never
   `mov.translation.scaleBy(...)`.
4. `rot.transformBy(mov)`.

Never assign `rot.translation` on the rotation matrix: `setToRotation` already wrote the
translation that carries the rotation onto `axisPoint`, and overwriting it turns the rotation
about the gear's axis into one about a parallel line through the world origin.

Then `bodies = adsk.core.ObjectCollection.create()`, `bodies.add(copy)`,
`moveInput = component.features.moveFeatures.createInput2(bodies)`,
`moveInput.defineAsFreeMove(rot)`, `component.features.moveFeatures.add(moveInput)`.

Proof stand-in: the copy is made and moved with `Duplicate` and `Placed` in a document of its own,
by every `k` the defaults' schedule uses; every vertex of the moved copy lands on a vertex of the
cell built in place `k` teeth along, which the gated document holds.

## S13 `[GO]` Join

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
      "span": "tools.add(tool)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "joinInput = component.features.combineFeatures.createInput(body, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "join = component.features.combineFeatures.add(joinInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "join.bodies",
      "role": "required",
      "span": "join.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1006,
      "last": 1009,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 486,
      "last": 496,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1006–1009; `spec/screwgear/fusion.md` L486–496.

Proof: `stepJoinBodies`.

<!-- proof-run: proofkit3d.RunSolid(joinCases, stepJoinBodies, assertJoinBodies) -->

([SCREW-F-JOIN]) The body is the target and the moved copy (or the aside, or the remainder cell)
the one tool: `tools = adsk.core.ObjectCollection.create()`, `tools.add(tool)`,
`joinInput = component.features.combineFeatures.createInput(body, tools)`,
`joinInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`,
`joinInput.isKeepToolBodies = False`, `join = component.features.combineFeatures.add(joinInput)`.
Raise with the piece's name and the count unless `join.bodies.count` is exactly 1
([PB-EMPTY-RESULT], [PB-SELF-DIAGNOSING]); `join.bodies.item(0)` is the body from then on. Every
move is by whole teeth already built, so each join meets its neighbour at one shared planar
cross-section, with no sliver and no overlap.

Proof stand-in: decad's Union refuses two bodies that meet face to face, and its stitch audit
refuses a whole 68-tooth ribbon, so the join is proved at its seam: the cell before the seam and
the piece after it, each lofted from sketches of its own, weld into one solid with no face left at
the seam — at every seam of the defaults' schedule and at the remainder's seam.

## S14 `[GO]` Remainder Sections sketch (per gear, only when r > 0)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "sketch = component.sketches.add(axisPlane_g)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(worldPoint))"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(worldPoint))"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(corner, nextCorner)"
    }
  ],
  "citations": [
    {
      "first": 997,
      "last": 1002,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 929,
      "last": 953,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L997–1002; `spec/screwgear/instructions.md` L929–953.

Proof: `stepRemainderSectionSketch`.

<!-- proof-run: proofkit.Run(remainderSectionCases, stepRemainderSectionSketch) -->

Only when `r = N mod c > 0` (none at the defaults; 69 teeth in four-tooth cells leave one), after
the last round. `sketch = component.sketches.add(axisPlane_g)`, `sketch.name =
f'{gearLabel} Cell Remainder'`, `sketch.isComputeDeferred = True` before the first point.

Section `k`, for `k = 0 … r*n`, stands at `s_k = s0 + q*c*P + k*P/n` (from `s0 + q*c*P` to
`s0 + N*P`), with `s0 = Z0_g - L/2`, and is the same rectangle as the Cell Sections sketch's:
`u` from `-W/2` to `Utooth(s_k)`, `v` from `-T/2` to `T/2`, its four corners
`World_g(s_k, -W/2, -T/2)`, `World_g(s_k, Utooth(s_k), -T/2)`, `World_g(s_k, Utooth(s_k), T/2)`,
`World_g(s_k, -W/2, T/2)`, each `sketch.sketchPoints.add(sketch.modelToSketchSpace(worldPoint))`
with its z kept; four solid lines `L1`…`L4` sharing them by
`sketch.sketchCurves.sketchLines.addByTwoPoints(corner, nextCorner)`; every point `isFixed = True`
after the last line. Then `sketch.isComputeDeferred = False`, raise unless
`sketch.isFullyConstrained`, and raise with the count unless `sketch.profiles.count` is `r*n + 1`.
Its first section is the body's last, so the join of S13 meets a shared cross-section.

Proof stand-in: one planar sketch per section, as for the Cell Sections sketch.

## S15 `[GO]` Remainder loft (per gear, only when r > 0)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "component.features.loftFeatures",
      "role": "required",
      "span": "loftInput = component.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "coll = adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "coll",
      "role": "required",
      "span": "coll.add(line)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.LoftSections",
      "reason": null,
      "receiver": "loftInput.loftSections",
      "role": "required",
      "span": "loftInput.loftSections.add(component.features.createPath(coll, False))"
    },
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "component.features",
      "role": "required",
      "span": "loftInput.loftSections.add(component.features.createPath(coll, False))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "component.features.loftFeatures",
      "role": "required",
      "span": "loft = component.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loft.bodies",
      "role": "required",
      "span": "loft.bodies.item(0).isSolid"
    }
  ],
  "citations": [
    {
      "first": 997,
      "last": 1002,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 955,
      "last": 962,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L997–1002; `spec/screwgear/instructions.md` L955–962.

Proof: `stepRemainderLoft`.

<!-- proof-run: proofkit3d.RunSolid(remainderLoftCases, stepRemainderLoft, assertRemainderLoft) -->

`loftInput = component.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
for each remainder section in order of `k`: `coll = adsk.core.ObjectCollection.create()`,
`coll.add(line)` for its four lines, `loftInput.loftSections.add(component.features.createPath(coll, False))`;
then `loft = component.features.loftFeatures.add(loftInput)`. Raise unless `loft.bodies.count` is
1 and `loft.bodies.item(0).isSolid` ([PB-LOFT], [PB-EMPTY-RESULT]). That body is joined into the
ribbon by S13 after the last round.

Proof stand-in: the chain of two-section sheet lofts and a stitch, held to `T*(W - H/2)*r*P` and
ending at the ribbon's positive end `s0 + N*P`.

## S16 `[GO]` Sleeve sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "sketch = component.sketches.add(self.plane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "centre = sketch.modelToSketchSpace(C)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchCircles",
      "role": "required",
      "span": "inner = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, Ri/10)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDiameterDimension(inner, innerText)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "innerText = adsk.core.Point3D.create(centre.x + Ri/10, centre.y, 0)"
    }
  ],
  "citations": [
    {
      "first": 1040,
      "last": 1047,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 438,
      "last": 457,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1040–1047; `spec/screwgear/fusion.md` L438–457.

Proof: `stepSleeveSketch`.

<!-- proof-run: proofkit.Run(sleeveSketchCases, stepSleeveSketch) -->

The method `buildCage` begins here ([SCREW-F-SLEEVE]). `sketch = component.sketches.add(self.plane)` on the
selected plane ([PB-USE-SELECTED-PLANE]), `sketch.name = 'Sleeve'`. No deferral.
`centre = sketch.modelToSketchSpace(C)`, `centre.z = 0` ([PB-SKETCH-ZERO-Z]). Two circles, each
created at that position, never on a shared point:

- `inner = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, Ri/10)`, then
  `inner.centerSketchPoint.isFixed = True` ([PB-CIRCLE-CENTER]; never a coincident to another
  point), then a diameter dimension
  `sketch.sketchDimensions.addDiameterDimension(inner, innerText)` of `2*Ri/10` cm, its text
  point `innerText = adsk.core.Point3D.create(centre.x + Ri/10, centre.y, 0)` on the circle, off
  the centre ([PB-RADIAL-DIM]).
- `outer` the same with radius `Ro/10`, its own centre point fixed, a diameter of `2*Ro/10` cm and
  its text point at `centre.x + Ro/10`.

Nothing else is in the sketch. Raise naming `Sleeve` unless `sketch.isFullyConstrained`. It has two
profiles, the inner disc and the ring; the ring is the profile whose `profile.profileLoops.count`
is 2. Iterate `sketch.profiles`, take the one profile with two loops, and raise with the counts
when there is not exactly one ([PB-EMPTY-RESULT]). `find_profile_by_curve_counts` cannot pick it:
it treats a circle as a curve type that disqualifies a loop.

## S17 `[GO]` Sleeve tube extrude

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "extrudeInput = component.features.extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "setSymmetricExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise/10), False)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise/10), False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "tube = component.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "tube.bodies",
      "role": "required",
      "span": "tube.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1047,
      "last": 1052,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 456,
      "last": 459,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1047–1052; `spec/screwgear/fusion.md` L456–459.

Proof: `stepSleeveTube`.

<!-- proof-run: proofkit3d.RunSolid(sleeveTubeCases, stepSleeveTube, assertSleeveTube) -->

`extrudeInput = component.features.extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise/10), False)` — `False`
makes the value each side's length ([PB-THROUGH-CUT] for the argument), so the tube runs from
`-cageRise` to `+cageRise` along `n̂`: 37.5 mm tall at the defaults, with a flat 565.5 mm² ring at
each end. `tube = component.features.extrudeFeatures.add(extrudeInput)`. Raise with the count
unless `tube.bodies.count` is 1. `tube.bodies.item(0)` is `self.cageBody` from here on, the first
body of the cage.

## S18 `[GO]` Bore section sketch (per bore)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "planeInput = component.constructionPlanes.createInput()"
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
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "plane = component.constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "sketch = component.sketches.add(plane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(worldPoint)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "O = sketch.sketchPoints.add(local of origin_g + s0*dir_g)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "Cp = sketch.sketchPoints.add(local of origin_g + s0*dir_g + (A/2)*u_g)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "K = sketch.sketchCurves.sketchLines.addByTwoPoints(O, seedE)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textK)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addAngularDimension(Ru, second, textA)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(L1, K)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L1, textL1)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(L3, K)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L3, textL3)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(E, L2)"
    },
    {
      "condition": null,
      "name": "addPerpendicular",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addPerpendicular(L2, K)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(L4, L2)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(L2, L4, textL4)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": "lib/geargen/utilities.py provides find_profile_by_curve_counts",
      "receiver": null,
      "role": "inherited",
      "span": "profile = find_profile_by_curve_counts(sketch, lines=4)"
    }
  ],
  "citations": [
    {
      "first": 1104,
      "last": 1110,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1126,
      "last": 1163,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 364,
      "last": 372,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1104–1110; `spec/screwgear/instructions.md` L1126–1163; `spec/screwgear/fusion.md` L364–372.

Proof: `stepBoreSectionSketch`.

<!-- proof-run: proofkit.Run(boreSketchCases, stepBoreSectionSketch) -->

For each bore of S02, in the order gear A `-R`, gear A `+R`, gear B `-R`, gear B `+R`, first its
plane, then this sketch, then its cut (S19), before the next bore.

**The plane** ([PB-CONSTRUCTION-PLANES], [SCREW-F-TWISTED-SLOT]): on the bore's own line of S07,
`line = self.pathLines[gear]['bore-']` for a `-R` bore and `['bore+']` for a `+R` bore, passed
directly: `planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))`,
`plane = component.constructionPlanes.add(planeInput)`,
`plane.name = f'{gearLabel} Bore {"-R" if sigma < 0 else "+R"} Plane'`. It stands square to the
axis at the line's start, station `s0 = -sOut` for a `-R` bore and `s0 = sIn` for a `+R` bore.

**The sketch** on it: `sketch = component.sketches.add(plane)`,
`sketch.name = f'{gearLabel} Bore {"-R" if sigma < 0 else "+R"}'`, then
`sketch.isComputeDeferred = True` ([SCREW-F-DEFER]). `th = Theta_g(s0)`. Every point below is the
world point named, divided by 10, mapped with `sketch.modelToSketchSpace(worldPoint)` and given
`z = 0` before use ([PB-SKETCH-ZERO-Z]) — reference points, seeds and text points alike. The
rectangle spans `u` from `uB = -hw` to `uF = hw` and `v` from the bore's `vLo` to `vHi` (S02).

1. **References.** `O = sketch.sketchPoints.add(local of origin_g + s0*dir_g)` and
   `Cp = sketch.sketchPoints.add(local of origin_g + s0*dir_g + (A/2)*u_g)` (on the plane, `A/2`
   along the unrotated `u_g`). The construction line
   `Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(O, Cp)`, `Ru.isConstruction = True`.
2. **The spine.** `K = sketch.sketchCurves.sketchLines.addByTwoPoints(O, seedE)` with
   `seedE` the local of `World_g(s0, uF, 0)`, `K.isConstruction = True`; `E = K.endSketchPoint`.
   Now set `O.isFixed = True` and `Cp.isFixed = True` (after `Ru` and `K`, before any constraint
   or dimension).
3. **The rectangle.** Four solid lines sharing their corners ([PB-SHARE-XOR-COINCIDENT]: no
   coincident on a corner), seeds the solved points: `L1` from `World_g(s0, uB, vLo)` to
   `World_g(s0, uF, vLo)`, `L2` from `L1.endSketchPoint` to `World_g(s0, uF, vHi)`, `L3` from
   `L2.endSketchPoint` to `World_g(s0, uB, vHi)`, `L4` from `L3.endSketchPoint` to
   `L1.startSketchPoint`, each `sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)`.
4. **Constraints and dimensions**, all driving ([PB-DRIVING-DIM]), in this order:
   - the spine's length: `sketch.sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textK)`,
     value `uF/10` cm, `textK` the local of `World_g(s0, uF/2, 0.5)`;
   - the angle ([PB-ANGULAR-DIM]): when `|sin th| >= sqrt(1/2)`, between `Ru` and `K`;
     otherwise between `Ru` and `L2`. Call `sketch.sketchDimensions.addAngularDimension(Ru, second, textA)`.
     The value is the unsigned angle between two rays, `phi = |fold(psi)|` with `fold` taking an
     angle into `(-pi, pi]` and `psi = th` for `K`, `psi = th + pi/2` for `L2`; it lies in
     45°–135°. The rays: for `K`, from `O` along `u_g` (toward `Cp`) and from `O` toward `E`; for
     `L2`, from `X` along `u_g` and from `X` along `L2`'s start→end direction, `X` the point where
     the line through `O` and `Cp` meets the line of `L2`. The text point `textA` is the local of
     `X + 0.3*(e1 + e2)/|e1 + e2|` (cm), `e1` the unit vector of the first ray and `e2` the
     second's (`X = O` for `K`), so it sits inside the wedge the value measures;
   - `sketch.geometricConstraints.addParallel(L1, K)`, then
     `sketch.sketchDimensions.addOffsetDimension(K, L1, textL1)` of `-vLo/10` cm, `textL1` the
     local of `World_g(s0, uF/2, vLo/2)`;
   - `sketch.geometricConstraints.addParallel(L3, K)`, then
     `sketch.sketchDimensions.addOffsetDimension(K, L3, textL3)` of `vHi/10` cm, `textL3` the
     local of `World_g(s0, uF/2, vHi/2)`;
   - `sketch.geometricConstraints.addCoincident(E, L2)` — `E` on the toothed side;
   - `sketch.geometricConstraints.addPerpendicular(L2, K)`;
   - `sketch.geometricConstraints.addParallel(L4, L2)`, then
     `sketch.sketchDimensions.addOffsetDimension(L2, L4, textL4)` of `(uF - uB)/10` cm, `textL4`
     the local of `World_g(s0, 0, (vLo + vHi)/2)` ([PB-OFFSET-DIM], [PB-NO-OVERCONSTRAIN]).
   Each dimension's value is set by `dimension.parameter.value = value` with the magnitude above
   ([PB-DIM-VALUE-SEMANTICS]); the seeds already sit on the right sides.

Ten degrees of freedom (`E` and the four corners) against ten rows. Then
`sketch.isComputeDeferred = False`; raise naming the sketch unless `sketch.isFullyConstrained`. The
profile is the one loop of four lines: `profile = find_profile_by_curve_counts(sketch, lines=4)`
([PB-PROFILE-MATCH]); the construction lines bound nothing. Nothing is projected into this sketch:
the Anchor Line's projection would run through `Cp` across the rectangle and split the profile.

Proof: the engine's `NewOffset` carries the parallel and the offset rows together, signed (the
seed's side); its `NewAngle` is signed, so the proof writes the signed angle the seeds make and
checks its magnitude is the 45°–135° value the build writes. Cases reach both angle branches,
a `+R` level bore with its roof on `-v`, no roof allowance, a 0.05 mm clearance and a 100°
crossing.

## S19 `[GO]` Bore sweep cut (per bore)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "component.features",
      "role": "required",
      "span": "path = component.features.createPath(line, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "component.features.sweepFeatures",
      "role": "required",
      "span": "sweepInput = component.features.sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "sweepInput.twistAngle = adsk.core.ValueInput.createByReal(+(sOut - sIn)/Lambda)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "component.features.sweepFeatures",
      "role": "required",
      "span": "sweep = component.features.sweepFeatures.add(sweepInput)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "self.cageBody",
      "role": "required",
      "span": "self.cageBody.pointContainment(probe) == adsk.fusion.PointContainment.PointOutsidePointContainment"
    }
  ],
  "citations": [
    {
      "first": 1104,
      "last": 1124,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1165,
      "last": 1177,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1187,
      "last": 1201,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 374,
      "last": 422,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 1468,
      "last": 1475,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1104–1124; `spec/screwgear/instructions.md` L1165–1177; `spec/screwgear/instructions.md` L1187–1201; `spec/screwgear/fusion.md` L374–422; `spec/screwgear/instructions.md` L1468–1475.

Proof: `stepBoreCut`.

<!-- proof-run: proofkit3d.RunSolid(boreCutCases, stepBoreCut, assertBoreCut) -->

([SCREW-F-TWISTED-SLOT], [PB-SWEEP-TWIST], [PB-PATH-FROM-SKETCH]) For the bore just sketched:
`path = component.features.createPath(line, False)` on the same line as its plane (S18),
`sweepInput = component.features.sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`sweepInput.twistAngle = adsk.core.ValueInput.createByReal(+(sOut - sIn)/Lambda)` (radians,
positive; 80.78° at the defaults), `sweepInput.participantBodies = [self.cageBody]`, and nothing
else — not `orientation`, not `solidTwistAxis`, no guide rail or surface. Then
`sweep = component.features.sweepFeatures.add(sweepInput)`. The sign is positive: the path runs
along `+dir_g`, `Theta_g` grows with `s`, and a positive twist turns the profile that way
(measured 2026-09-28). The participant list keeps the ribbons whole.

**The check** ([SCREW-F-SWEEP-CHECK], [PB-SELF-DIAGNOSING]). Raise naming the bore and the count
unless `sweep.bodies.count` is 1; that body is `self.cageBody` from then on. Then, with
`sc = sigma*cageRadius`, `th = Theta_g(sc)` and `uhat = cos(th)*u_g + sin(th)*v_g`, both probes
`origin_g + sc*dir_g ± (W/2 + clearance/2)*uhat` (cm, as `adsk.core.Point3D`) must read
`self.cageBody.pointContainment(probe) == adsk.fusion.PointContainment.PointOutsidePointContainment`;
raise naming the bore, the probe and the containment read otherwise. They stand inside the wall
(16.80 mm from the frame's axis for a `-R` bore and 16.30 mm for a `+R` bore at the defaults),
clear of the ribbon and of the channel's wall by `clearance/2`; under the wrong twist sense they
sit in the wall.

Proof stand-in: decad has no twisted sweep, so each channel is the chain of two-section lofts
through the bore's rectangle turned by `Theta_g` at `boreSections` stations over the span (18 at
the defaults: no two more than 5° apart nor more than `2*acos(1 - 0.04*clearance/cc)`); chained
cuts are refused on a faceted cage, so the cage after bore `i` is the tube cut once by the union of
the channels so far. The probes are held inside the channel.

## S20 `[PROSE]` Window Plane

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "planeInput = component.constructionPlanes.createInput()"
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
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)"
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
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "windowPlane = component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1386,
      "last": 1391,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 461,
      "last": 468,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1386–1391; `spec/screwgear/fusion.md` L461–468.

Made only when at least one window of S03 has room; one plane shared by both windows,
([SCREW-F-SLEEVE], [PB-CONSTRUCTION-PLANES]) through the frame's axis and square to the facing
direction. `planeInput = component.constructionPlanes.createInput()`, then:

- windows facing `±k̂` (`crossAngle <= 90°`):
  `planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)`
  — the plane through the Anchor Line square to the selected plane, holding `C`, `ê` and `n̂`;
- windows facing `±ê`: `planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))`
  — square to the Anchor Line at its midpoint, `C` (never run in Fusion at 0.5).

`windowPlane = component.constructionPlanes.add(planeInput)`, `windowPlane.name = 'Window Plane'`.

## S21 `[GO]` Window sketch (per window)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "component.sketches",
      "role": "required",
      "span": "sketch = component.sketches.add(windowPlane)"
    },
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
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i + 1) % len(pts)])"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.Profiles",
      "reason": null,
      "receiver": "sketch.profiles",
      "role": "required",
      "span": "profile = sketch.profiles.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1391,
      "last": 1398,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 469,
      "last": 472,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1391–1398; `spec/screwgear/fusion.md` L469–472.

Proof: `stepWindowSketch`.

<!-- proof-run: proofkit.Run(windowSketchCases, stepWindowSketch) -->

For each window with room, the one facing `d` before the one facing `-d`; first this sketch, then
its cut (S22). `sketch = component.sketches.add(windowPlane)`, `sketch.name = f'Window {name}'`
with `name` one of `+k`, `-k`, `+e`, `-e`. No deferral. In the real frame `across = n̂ × d`. For
each corner `(t, z)` of the window's hexagon (S03), in order: `local =
sketch.modelToSketchSpace(C + (t*across + z*n̂)/10)`, `local.z = 0` ([PB-SKETCH-ZERO-Z]),
`pt = sketch.sketchPoints.add(local)`. One solid line from each corner to the next, the last back
to the first, sharing the points: `sketch.sketchCurves.sketchLines.addByTwoPoints(pts[i], pts[(i + 1) % len(pts)])`
([PB-SHARE-XOR-COINCIDENT]). Then every point `isFixed = True`. Nothing else.

Raise naming the sketch unless `sketch.isFullyConstrained`; raise with the count unless
`sketch.profiles.count` is 1, then take `profile = sketch.profiles.item(0)` ([PB-SINGLE-PROFILE]).
The two windows are two sketches because their hexagons cross on the shared plane.

## S22 `[GO]` Window extrude cut (per window)

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
      "span": "self.cageBody.pointContainment(probe) == adsk.fusion.PointContainment.PointInsidePointContainment"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "cutInput = component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "cutInput",
      "role": "required",
      "span": "cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro/10 + 0.1)), direction)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro/10 + 0.1)), direction)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro/10 + 0.1)), direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "cut = component.features.extrudeFeatures.add(cutInput)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(C + d)"
    },
    {
      "condition": null,
      "name": "pointContainment",
      "owner": "adsk.fusion.BRepBody",
      "reason": null,
      "receiver": "self.cageBody",
      "role": "required",
      "span": "self.cageBody.pointContainment(probe) == adsk.fusion.PointContainment.PointOutsidePointContainment"
    }
  ],
  "citations": [
    {
      "first": 1400,
      "last": 1422,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1099,
      "last": 1102,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 472,
      "last": 484,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1400–1422; `spec/screwgear/instructions.md` L1099–1102; `spec/screwgear/fusion.md` L472–484.

Proof: `stepWindowCut`.

<!-- proof-run: proofkit3d.RunSolid(windowCutCases, stepWindowCut, assertWindowCut) -->

**Before the cut** ([PB-SELF-DIAGNOSING]): with `(tc, zc)` the average of the hexagon's corners,
`probe = C + (tc*across + zc*n̂ + ((a0(tc) + a1(tc))/2)*d)/10` (cm; `a0`, `a1` of S03) must read
`self.cageBody.pointContainment(probe) == adsk.fusion.PointContainment.PointInsidePointContainment`;
raise naming the window and the reading otherwise.

**The cut** ([SCREW-F-SLEEVE], [PB-THROUGH-CUT]):
`cutInput = component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`cutInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal(Ro/10 + 0.1)), direction)`,
`cutInput.participantBodies = [self.cageBody]`, `cut = component.features.extrudeFeatures.add(cutInput)`.
`direction` is `adsk.fusion.ExtentDirections.PositiveExtentDirection` when
`sketch.modelToSketchSpace(C + d)` has a positive `z`, else
`adsk.fusion.ExtentDirections.NegativeExtentDirection`. The cut runs one way only.

**After the cut:** raise with the count unless `cut.bodies.count` is 1; that body is
`self.cageBody`. The same probe must now read
`self.cageBody.pointContainment(probe) == adsk.fusion.PointContainment.PointOutsidePointContainment`;
raise naming the window otherwise (a cut extruded the wrong way leaves it inside).

After the last window (or the last bore when no window has room), name the cage body `Cage`
(`self.cageBody.name = 'Cage'`) and log, with `futil.log` ([PB-LOGGING]), exactly:
`Print the cage standing on its end below the selected plane: the roof allowance is on the bridged roofs that way up.`

Proof stand-in: the window's prism starts 1 mm out along `d` rather than on the plane (the two
prisms would otherwise meet face to face on it, which decad refuses); the wall begins further out
at every `t` of the hexagon, so the same wall is removed. The cage after a window is the tube cut
once by the union of the four channels and the windows so far; the finished cage is held to the
spec's 16,561 mm³ within 1%.

## S23 `[PROSE]` Relocate the bodies and hide the construction geometry

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
      "reason": "lib/geargen/solids.py provides hide_construction_geometry",
      "receiver": "solids",
      "role": "inherited",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    }
  ],
  "citations": [
    {
      "first": 1424,
      "last": 1430,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 759,
      "last": 798,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1424–1430; `spec/screwgear/instructions.md` L759–798.

The method `relocateBodies` ([PB-NO-CROSS-SIBLING]): move each finished body into its sub-component with
`body.moveToComponent(occurrence)`, which keeps the world position and needs no activation:
`self.gearBodies[0]` into `self.gearOccs[0]` (`Gear A`), `self.gearBodies[1]` into
`self.gearOccs[1]` (`Gear B`), and `self.cageBody` into `self.cageOcc` (`Cage`), in that order.
Then `solids.hide_construction_geometry(self.designOcc.component)` ([PB-TREE-CLEANUP]) — never a
re-implementation of it; the `Design` component holds every sketch and plane of the build.

Counted at the defaults the build makes 12 sketches (Anchor, two Paths, two Cell Sections,
Sleeve, four bore sections, two windows), 7 construction planes (two axis planes, four bore
planes, the Window Plane) and 42 features (two lofts, 30 for the doubling — 5 rounds of copy,
move and join per gear — the tube, four sweeps, two window cuts, three relocations): 61 timeline
entries plus the five component creations, 66 in all. A remainder cell adds one sketch, one loft
and one join per gear; a window without room takes off one sketch and one feature, and the Window
Plane when neither has room.
