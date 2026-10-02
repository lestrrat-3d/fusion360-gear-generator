The step proof for this list is `proof/screwgear/compiled_model_test.go`, `proof/screwgear/compiled_sketches_test.go` and `proof/screwgear/compiled_solids_test.go`, registered by the generated `proof/screwgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `fd5b8f6adb1a9671976f03fdd3c3da925f5fe4af` |
| `spec/screwgear/fusion.md` | `a310d6db21f4e5c3bee41017bd20507ef6cd1a1d` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `87a9c5b7bdf6f3cc3ffbf55f9989e2ee48617576` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/screwgear/mesh-search.md` | `68c46e2aeaa5e9ce28f03748c72c70ba497d732f` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `cdd32545b0f8c651752827c6697601f1e32b4d39` |

## S01 `[PROSE]` Module, classes and dialog inputs

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "addSelectionInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "command.commandInputs",
      "role": "required",
      "span": "command.commandInputs.addSelectionInput(id, label, tooltip)"
    },
    {
      "condition": null,
      "name": "addSelectionFilter",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.addSelectionFilter(filter)"
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
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.addSelection(get_design().rootComponent)"
    },
    {
      "condition": null,
      "name": "get_design",
      "owner": null,
      "reason": "the framework helper in lib/geargen/misc.py returns the active design",
      "receiver": null,
      "role": "inherited",
      "span": "selectionInput.addSelection(get_design().rootComponent)"
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
      "first": 438,
      "last": 469,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 489,
      "last": 559,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L438–469; `spec/screwgear/instructions.md` L489–559.

Not a timeline entry. This step fixes the module surface and the dialog.

**Module.** `lib/geargen/screwgear.py` defines exactly two public classes, exported through
`lib/geargen/__init__.py`, and imports only the framework (`base`, `misc`, `utilities`, `solids`,
`fusion360utils`) and `math`, `adsk.core`, `adsk.fusion`, explicitly, no star import:

- `ScrewGearCommandInputsConfigurator` with the classmethod `configure`, taking `cls` and `command`, which adds
  the dialog inputs below. No conditional visibility, no `inputChanged` handling.
- `ScrewGearGenerator(base.Generator)`, constructed with the inherited one-argument constructor
  `(design)`. It implements `generate`, taking `self` and `inputs`, overrides `prefixBase` to return
  `'ScrewGear'`, and relies on the inherited `deleteComponent` for error cleanup. It carries no
  generation context; its handles live on `self`: `self.designOcc`, `self.gearOccs` (list of two),
  `self.cageOcc`, `self.gearBodies` (list of two), `self.cageBody`, `self.pathLines` (list of two
  dicts, one per gear, keyed `'bore-'` and `'bore+'`, each the Paths sketch line of S09 that
  bore's sweep of S20 runs along) and `self.windows` (zero to two windows from S04, each its facing
  direction and its corners).

The command entry `commands/screwgear/entry.py` constructs `GearCommand(gear_type='ScrewGear',
name='Screw Gear Generator', …)` binding the two classes by name; the playbook's command-entry
wiring applies. Parameter mode is all-Python-precomputed [PB-PRECOMPUTED-MODE]: no user
parameter is registered; every value is computed in Python and written numerically.

**Module-level constants**, every input id, and the cell size:

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
| `INPUT_ID_CAGE_RADIUS` | `'cageRadius'` |
| `INPUT_ID_CAGE_RISE` | `'cageRise'` |
| `INPUT_ID_CLEARANCE` | `'clearance'` |
| `INPUT_ID_COLLAR_HALF` | `'collarHalf'` |
| `INPUT_ID_COLLAR_WALL` | `'collarWall'` |
| `INPUT_ID_CROSS_ANGLE` | `'crossAngle'` |
| `INPUT_ID_ENGAGEMENT` | `'engagement'` |
| `INPUT_ID_MOUNT_ANGLE_A` | `'mountAngleA'` |
| `INPUT_ID_MOUNT_ANGLE_B` | `'mountAngleB'` |
| `INPUT_ID_ASSEMBLY_PHASE` | `'assemblyPhase'` |
| `CELL_TEETH` | `4` (the spec's `cellTeeth`; not a dialog input) |

**The dialog, in this order.** The three selection inputs come first and at the top level, the
plane first, since Fusion focuses the first selection input [PB-AUTOFOCUS-FIRST]. Each is
`command.commandInputs.addSelectionInput(id, label, tooltip)`, then one
`selectionInput.addSelectionFilter(filter)` per filter, each filter the named constant on
`adsk.core.SelectionCommandInput` and never a quoted string [PB-SELECTION-FILTER-ENUM], then
`selectionInput.setSelectionLimits(1, 1)` [PB-SELECTION-DECL]:

| id | Label | Tooltip | Filters |
|---|---|---|---|
| `plane` | Target Plane | Plane the cage's axis is normal to | `ConstructionPlanes`, `PlanarFaces` |
| `point` | Centre Point | Centre of the mechanism | `ConstructionPoints`, `SketchPoints` |
| `parent` | Parent Component | Component the mechanism is created under | `Occurrences`, `RootComponents` |

The Parent Component input pre-selects the root component with
`selectionInput.addSelection(get_design().rootComponent)`.

Then three groups, each `command.commandInputs.addGroupCommandInput(groupId, groupLabel)`, whose
inputs are added to the group's `children` collection, not to the top-level inputs:
`group.children.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))`. The
default is in internal units [PB-DIALOG-DEFAULT-UNITS]: millimetres divided by 10 for a length,
radians for an angle, the bare count for `toothCount`. The ribbon and frame groups start expanded
and the mesh group starts collapsed, `group.isExpanded = False` for `meshGroup` and `True` for the
other two. Rows within a group are added in the order listed:

| Group id | Group label | Label | id | unit string | default shown | `createByReal` value |
|---|---|---|---|---|---|---|
| `ribbonGroup` | Ribbon | Ribbon Width | `ribbonWidth` | `'mm'` | 15 mm | 1.5 |
| `ribbonGroup` | Ribbon | Tooth Count | `toothCount` | `''` | 68 | 68 |
| `ribbonGroup` | Ribbon | Twist Lead | `twistLead` | `'mm'` | 49.5 mm | 4.95 |
| `ribbonGroup` | Ribbon | Ribbon Thickness | `ribbonThickness` | `'mm'` | 3.75 mm | 0.375 |
| `ribbonGroup` | Ribbon | Tooth Pitch | `toothPitch` | `'mm'` | 2.625 mm | 0.2625 |
| `ribbonGroup` | Ribbon | Tooth Height | `toothHeight` | `'mm'` | 2.625 mm | 0.2625 |
| `frameGroup` | Frame | Cage Radius | `cageRadius` | `'mm'` | 15 mm | 1.5 |
| `frameGroup` | Frame | Cage Rise | `cageRise` | `'mm'` | 18.75 mm | 1.875 |
| `frameGroup` | Frame | Clearance | `clearance` | `'mm'` | 0.20 mm | 0.02 |
| `frameGroup` | Frame | Collar Half Length | `collarHalf` | `'mm'` | 3 mm | 0.3 |
| `frameGroup` | Frame | Collar Wall | `collarWall` | `'mm'` | 3 mm | 0.3 |
| `meshGroup` | Mesh (from the mesh search) | Crossing Angle | `crossAngle` | `'deg'` | 80° | 1.3962634 (80° in radians) |
| `meshGroup` | Mesh (from the mesh search) | Engagement | `engagement` | `'mm'` | 0.90 mm | 0.09 |
| `meshGroup` | Mesh (from the mesh search) | Mounting Angle A | `mountAngleA` | `'deg'` | 14° | 0.2443461 (14° in radians) |
| `meshGroup` | Mesh (from the mesh search) | Mounting Angle B | `mountAngleB` | `'deg'` | 14° | 0.2443461 (14° in radians) |
| `meshGroup` | Mesh (from the mesh search) | Assembly Phase | `assemblyPhase` | `'mm'` | −1.30 mm | −0.13 |

Input ids are unique across the whole command, which is what lets the top-level lookup of S02
reach into a group.

## S02 `[PROSE]` Read the inputs and check their ranges

<!-- step-meta
{
  "calls": [
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
      "span": "design.unitsManager.evaluateExpression(valueInput.expression, units)"
    }
  ],
  "citations": [
    {
      "first": 535,
      "last": 579,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 617,
      "last": 627,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 196,
      "last": 230,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 272,
      "last": 285,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L535–579; `spec/screwgear/instructions.md` L617–627; `spec/screwgear/instructions.md` L196–230; `spec/screwgear/instructions.md` L272–285.

Not a timeline entry; the first part of `processInputs`, which `generate` calls before
anything else.

**Look every input up first.** Read each input by id with `inputs.itemById(id)` on the command's
top-level inputs, grouped or not, and raise naming the id when the lookup returns `None`. Read the
three selections before anything creates an occurrence [PB-SELECTION-STASH]: for each,
`selectionInput.selectionCount` must be 1 and the entity is `selectionInput.selection(0).entity`.
The parent entity is an occurrence or a component; resolve an occurrence to its `component` and
store the result as `self.parentComponent`. Store the plane as `self.plane` and the point as
`self.point`.

**Values.** Read each value input as
`design.unitsManager.evaluateExpression(valueInput.expression, units)`, `units` being `'mm'` for a
length, `'deg'` for an angle and `''` for `toothCount` [PB-EVAL-EXPRESSION]. The call returns
internal units, centimetres and radians. This build works in millimetres in Python: multiply every
length by 10, keep angles in radians, and divide every length by 10 again where it is handed to
Fusion. Names used from here on, all in millimetres or radians:

```
W = ribbonWidth      T = ribbonThickness   P = toothPitch      H = toothHeight
N = toothCount       lead = twistLead      Sigma = crossAngle  E = engagement
PhiA = mountAngleA   PhiB = mountAngleB    Z0B = assemblyPhase
R = cageRadius       rise = cageRise       clr = clearance     ch = collarHalf   cw = collarWall
Lambda = lead / (2*pi)          A = W - E          L = N * P
```

**Range checks**, in this order, each raising a clear error that names the offending input id
and the bound:

1. `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `collarHalf`, `collarWall` and
   `clearance` must each be greater than 0.
2. `toothCount` must be a whole number (its value equal to its rounded value) and at least 4;
   from here `N` is that integer.
3. `toothHeight` must be greater than 0 and less than `ribbonWidth / 2`.
4. `engagement` must be greater than 0 and at most `toothHeight`.
5. `crossAngle` must lie strictly between 0 and 180 degrees (compare `math.degrees` of the read
   value).
6. `assemblyPhase` must lie strictly within plus or minus `toothPitch`.
7. Both bores, each cut a millimetre past the wall, must lie within the ribbon:
   `R + ch + 1 < N*P/2`, the message naming `cageRadius`.

`twistLead` has no upper bound and neither mounting angle has any range; do not clamp them. The
sleeve's four checks of S03 and the window search of S04 come next, still in `processInputs`.

**Derived counts** of the cell (S10, S12), computed here from `CELL_TEETH`:

```
c = min(CELL_TEETH, N)          q = N // c        r = N % c
n = max(ceil((P / Lambda) / radians(2)), 8)       # steps to the tooth
```

At the defaults `c = 4`, `q = 17`, `r = 0`, `n = 10`, so the cell has `c*n + 1 = 41` sections.

## S03 `[PROSE]` The sleeve's four checks

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 580,
      "last": 616,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 971,
      "last": 987,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1119,
      "last": 1147,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L580–616; `spec/screwgear/instructions.md` L971–987; `spec/screwgear/instructions.md` L1119–1147.

Not a timeline entry; `processInputs` continues. These lengths, in millimetres, with their values
at the defaults:

```
Ri   = R - ch                     inner radius, 12
Ro   = R + ch                     outer radius, 18
hw   = W/2 + clr                  the bore's half-width, 7.70
ht   = T/2 + clr                  the bore's half-thickness, 2.075
c    = hypot(hw, ht)              the bore's corner radius, 7.975
sIn  = sqrt(Ri^2 - c^2) - 1       where a +R bore's cut starts on its axis, 7.967
sOut = Ro + 1                     where it ends, 19
axialWindow = 1.5 * sqrt(W^2 - A^2) / sin(Sigma)          7.79
```

Run the four checks in this order; each raises naming the input given and the bound:

1. **The channel starts in the hollow**, naming `cageRadius`: raise unless
   `hypot(c, 1) < Ri`, which is `sIn > 0`.
2. **The mesh stays visible along the axis**, naming `cageRadius`: raise unless
   `hypot(axialWindow, hypot(W/2, T/2)) + clr <= Ri` (11.18 against 12 at the defaults).
3. **The end faces keep `collarWall`**, naming `cageRise`: raise unless `rise >= A/2 + c + cw`
   (18.02 at the defaults).
4. **The wall between neighbouring bores keeps `collarWall`**, naming `collarWall`: compute the
   separation below and raise unless it is at least `cw`; the message names the two bores and the
   separation (5.206 mm at the defaults, across the `-ê` gap between the two `-R` bores).

**The search frame.** Both searches of S03 and S04 run before any feature exists, so they work in
an abstract frame of their own, in millimetres: `C` at the origin, `ê = (1, 0, 0)`,
`k̂ = (0, 1, 0)`, `n̂ = (0, 0, 1)`. Results are mapped to the world in S22 by the same
coordinates against the world's `C`, `ê`, `k̂` and `n̂`. In it, for gear `g` in A, B:

```
dir_A    = (cos(Sigma/2),  sin(Sigma/2), 0)     origin_A = (0, 0, -A/2)     u_A = (0, 0,  1)
dir_B    = (cos(Sigma/2), -sin(Sigma/2), 0)     origin_B = (0, 0, +A/2)     u_B = (0, 0, -1)
v_g      = dir_g x u_g
ang_g(s) = s/Lambda + Phi_g
world(g, u, v, s) = origin_g + s*dir_g + (u*cos(th) - v*sin(th))*u_g + (u*sin(th) + v*cos(th))*v_g,
                    th = ang_g(s)
```

The four bores, in this order and by these names: `gear A -R`, `gear A +R`, `gear B -R`,
`gear B +R`. Bore `σ` of gear `g` (σ = -1 for `-R`, +1 for `+R`) has its cut span
`[sIn, sOut]` for `+R` and `[-sOut, -sIn]` for `-R`, and its crossing at
`origin_g + σ*R*dir_g`.

**The separation** (`channelSeparation` in `proof/screwgear/sleeve_test.go`, step for step):

1. Sample each bore's outline: at stations `s = σ*sIn + σ*0.1*i` for `i = 0, 1, …` while
   `|s| <= sOut`, take seventeen points on each side of the bore's rectangle, `(hw, v)` and
   `(-hw, v)` for `v = -ht + 2*ht*i/16`, and `(u, ht)` and `(u, -ht)` for `u = -hw + 2*hw*i/16`,
   `i = 0 … 16`, each placed as `world(g, u, v, s)`. Keep a point when its distance from the
   frame's axis, `hypot(x, y)`, lies within `Ri - 0.5` to `Ro + 0.5`.
2. The four gaps between neighbouring bores, each with the direction `d` it faces:

   | Facing `d` | First bore | Second bore |
   |---|---|---|
   | `+k̂ = (0, 1, 0)` | gear A +R | gear B -R |
   | `-ê = (-1, 0, 0)` | gear B -R | gear A -R |
   | `-k̂ = (0, -1, 0)` | gear A -R | gear B +R |
   | `+ê = (1, 0, 0)` | gear B +R | gear A +R |

   For each gap, with `across = n̂ × d`, project the kept points of each of its two bores to
   `(P·across, P·n̂)` and take the convex hull of each bore's projections by Andrew's monotone
   chain (counter-clockwise).
3. For every edge of either hull, with `m` the unit vector square to it, the separation along `m`
   is the larger of `min(m·q) - max(m·p)` and `min(m·p) - max(m·q)`, `p` over the first hull's
   corners and `q` over the second's. Skip an edge of zero length. The gap's separation is the
   largest over all edges, negative when the hulls overlap.
4. The separation is the least over the four gaps, and the two bores named are that gap's.

## S04 `[PROSE]` The window search

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "log",
      "owner": null,
      "reason": "fusion360utils.log is the framework logger [PB-LOGGING]",
      "receiver": "futil",
      "role": "inherited",
      "span": "futil.log(message)"
    }
  ],
  "citations": [
    {
      "first": 1149,
      "last": 1294,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 621,
      "last": 622,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1149–1294; `spec/screwgear/instructions.md` L621–622.

Not a timeline entry; the last part of `processInputs`, in the search frame of S03, in
millimetres. It refuses nothing: a window with no room is left out and logged. It is `newWindow`
in `proof/screwgear/sleeve_test.go` step for step. The result is `self.windows`: for each window
cut, its facing direction `d` and its hexagon corners `(t, z)`.

**Which windows.** The windows face `+k̂` then `-k̂` when `Sigma <= 90°`, and `+ê` then `-ê`
otherwise. Each window is found the same way from its own facing direction `d`; nothing of one is
derived from the other.

**The window's plane.** For a window facing `d`, `across = n̂ × d` (which is `-ê` for `d = +k̂`).
A point `P` has plane coordinates `t = P·across`, `z = P·n̂` and depth `a = P·d`. At `t` the wall
runs from `a0(t) = sqrt(max(0, Ri^2 - t^2))` to `a1(t) = sqrt(max(0, Ro^2 - t^2))`.

**A bore's section in the wall** (`sectionInWall`). For a bore of gear `g` at station `s` with
`|s| < Ro`, in the section plane's coordinates `x` along `u_g` and `y` along `v_g`: take the
rectangle's corners `(-hw, -ht)`, `(hw, -ht)`, `(hw, ht)`, `(-hw, ht)` in that order
(counter-clockwise), each turned by `th = ang_g(s)` to
`(u*cos(th) - v*sin(th), u*sin(th) + v*cos(th))`. Clip it to the heights inside the end faces:
keep the `x` for which `(origin_g + x*u_g)·n̂` lies within `±rise` (since `u_g` is `±n̂`, that is
`x` between `(-rise - origin_g·n̂)/(u_g·n̂)` and `(rise - origin_g·n̂)/(u_g·n̂)`, ordered). Then clip
what is left twice more, once to `near <= y <= far` and once to `-far <= y <= -near`, with
`near = sqrt(max(0, Ri^2 - s^2))` and `far = sqrt(Ro^2 - s^2)`: those are the section's two
**pieces**, either of which may be empty. Each clip keeps the part of a convex polygon on one side
of a line, walking its edges in order, keeping each corner on the kept side (a corner exactly on
the line is kept) and adding the point where an edge crosses the line. At `|s| >= Ro` the section
has no piece.

**The long sides.** The two bores whose crossings satisfy `crossing·d > 0` **flank** the window;
the other two are its **far** bores. The low flanking bore is the one whose crossing has the
smaller `z`; `lean = +1` when the high one's crossing has the larger `t`, else `-1`. Walk each
flanking bore's cut span at stations `lo + 0.001*i` for `i = 0, 1, …` while `<= hi` (`lo`, `hi`
its span's ends), take every corner of every piece as the point
`origin_g + x*u_g + y*v_g + s*dir_g`, and its `m = z + lean*t`. The low bore's largest `m` is
`lowReach`, the high bore's least is `highReach`, and

```
lo = lowReach  + sqrt(2)*cw
hi = highReach - sqrt(2)*cw
```

**The trims.** `zLimit` is the highest any channel reaches in the wall: over all four bores, at
stations `s = σ*sIn + σ*0.01*i` while `|s| <= sOut`, wherever the section reaches the wall, that
is `|s| <= Ro` and `hypot(s, hw*|sin(th)| + ht*|cos(th)|) >= Ri`, it is the largest
`A/2 + hw*|cos(th)| + ht*|sin(th)|` (14.54 at the defaults). Then

```
top    = min(2*zLimit - hi, hi + sqrt(2)*Ri)
bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)
```

**The ends.** `need = cw + 0.1/sqrt(2) + 0.005` (3.076 at the defaults). For each side
`δ = +1` (right) and `δ = -1` (left), bisect `te` 24 times between `0` and
`min(Ri, Ro/sqrt(2)) * (1 - 1e-9)`: take the middle, make it the new lower end when `clear(te)`
holds and the new upper end when it does not. The end is the final lower end:
`right = end(+1)`, `left = -end(-1)`. `clear(te)`, at `t = δ*te`:

1. `zLow = max(lo - lean*t, bottom + lean*t)` and `zHigh = min(hi - lean*t, top + lean*t)`; not
   clear when `zLow > zHigh`.
2. With `a0, a1` the wall's chord at `t`, the points `C + t*across + z*n̂ + a*d` are: at
   `z = zLow` and at `z = zHigh`, each at `a = a0, a0 + 0.1, …`, a step that would pass `a1`
   landing on `a1`, and `a1` last; then, on `a = a0` and on `a = a1`, at
   `z = zLow + (zHigh - zLow)*k/nz` for `k = 1 … nz - 1` with `nz = ceil((zHigh - zLow)/0.1)`, and
   at `z = -zLimit + 0.1*j` for every `j >= 0` with `z <= zLimit` and `zLow < z < zHigh`.
3. Not clear as soon as any point's distance to either far bore's channel, by the walk below with
   `reach = need`, is under `need`; clear otherwise.

**The distance to a channel** (`wallGap`). Before the first walk, build each bore's **station
table** once: stations `spanStart + 0.002*k` for `k = 0, 1, …` while `<= spanEnd`, each with its
two pieces; and wherever the set of non-empty pieces differs between two neighbouring stations,
stations every 0.0001 strictly between those two, all in order of `s`. For each non-empty piece
keep its **circle**: centre the average of its corners, radius the largest distance from that
centre to a corner. For a point `P` and one bore of gear `g`: `x = (P - origin_g)·u_g`,
`y = (P - origin_g)·v_g`, `sq = (P - origin_g)·dir_g`. When
`hypot(hypot(x, y), sq - clamp(sq, spanStart, spanEnd)) - c >= reach` the distance is `reach`
and nothing is walked. Otherwise start with `best = reach^2`; walk up from the first station with
`s >= sq`, then down from the one before it, stopping each way at the first station with
`(sq - s)^2 >= best`. At each station, for each non-empty piece: with
`o = hypot(x - cx, y - cy) - radius`, pass over the piece when `o > 0` and
`(sq - s)^2 + o^2 >= best`; otherwise set `best = min(best, (sq - s)^2 + d2)`, `d2` being 0 when
`(x, y)` is on the inner side of (or on) every edge of the counter-clockwise piece and the piece
has at least three corners, and otherwise the least squared distance from `(x, y)` to the piece's
edges. The distance is `sqrt(best)`.

**The hexagon.** Start from the square with corners `(-2*Ro, -2*Ro)`, `(2*Ro, -2*Ro)`,
`(2*Ro, 2*Ro)`, `(-2*Ro, 2*Ro)` and clip it, in this order, to each half-plane
`a*t + b*z <= k`:

| `a` | `b` | `k` |
|---|---|---|
| `lean` | 1 | `hi` |
| `-lean` | -1 | `-lo` |
| `-lean` | 1 | `top` |
| `lean` | -1 | `-bottom` |
| 1 | 0 | `right` |
| -1 | 0 | `-left` |

with the same keep-and-cross clip as `sectionInWall`. Drop a corner that lies within 0.001 of the
one before it, and the last one when it lies within 0.001 of the first. At the defaults the `+k̂`
window has `lo = -3.780`, `hi = 6.730`, `bottom = -20.751`, `top = 22.355`, `left = -11.523`,
`right = 11.703`, and six corners near `(11.70, -4.97)`, `(-7.81, 14.54)`, `(-11.52, 10.83)`,
`(-11.52, 7.74)`, `(8.49, -12.27)`, `(11.70, -9.05)`, 220.0 mm² by the shoelace formula.

**When a gap has no room.** A window is not cut when `hi <= lo` (reason "the flanking bores leave
no band between them"), or when `right <= left`, the hexagon has fewer than three corners, or its
shoelace area is not positive (reason "the far bores leave the band no length"). Then log
`futil.log(message)` with `message` the text `No window facing {d}: {reason}`, `{d}` being `+k`,
`-k`, `+e` or `-e` [PB-LOGGING], and go on without that window.

## S05 `[PROSE]` Component tree

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "getOccurrence",
      "owner": null,
      "reason": "base.Generator.getOccurrence creates the top occurrence under self.parentComponent",
      "receiver": "self",
      "role": "inherited",
      "span": "self.getOccurrence()"
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
      "first": 470,
      "last": 488,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 700,
      "last": 714,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L470–488; `spec/screwgear/instructions.md` L700–714.

Not a geometry step; five component creations in the timeline. `generate` runs, in order:
`processInputs` (S02–S04), `buildComponentTree`, `buildAnchor` (S06–S08), `buildGear` for gear A
then for gear B (each S09–S15), `buildCage` (S16–S23), `relocateBodies` and the cleanup (S24).

`buildComponentTree`: the top occurrence is the inherited `self.getOccurrence()`, which creates
it under `self.parentComponent`; name its component `Screw Gearing`. Never call
`addNewComponent` for it and never call `occurrence.activate` on anything [PB-NEVER-ACTIVATE]
[PB-OCCURRENCE-TREE]. Under it create four children, in this order, each with
`component.occurrences.addNewComponent(adsk.core.Matrix3D.create())`, and name each child's
component: `Design`, `Gear A`, `Gear B`, `Cage`. Store `self.designOcc`, `self.gearOccs`
(`[Gear A, Gear B]`) and `self.cageOcc`. Every sketch, construction plane and feature of this
build is made in the `Design` component; the finished bodies are moved into the other three in
S24 [PB-NO-CROSS-SIBLING]. Sketches go directly on the user's selected plane, never on a plane
normalised from it [PB-USE-SELECTED-PLANE].

## S06 `[GO]` Anchor sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "designComponent.sketches",
      "role": "required",
      "span": "designComponent.sketches.add(self.plane)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "centre = sketch.project(self.point)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "centre",
      "role": "required",
      "span": "centre.item(0)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(gx, gy, 0)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "line = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(centre, line)"
    },
    {
      "condition": null,
      "name": "addMidPoint",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addMidPoint(centre, line)"
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
      "first": 759,
      "last": 806,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 413,
      "last": 431,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L759–806; `spec/screwgear/fusion.md` L413–431.

One sketch, named `Anchor`, on the user's selected plane: `designComponent.sketches.add(self.plane)`
[PB-USE-SELECTED-PLANE]. Computing is not deferred here [SCREW-F-DEFER].

1. Project the selected point, the one projection in the build [SCREW-F-REFERENCES]:
   `centre = sketch.project(self.point)` and take its first item, `centre.item(0)`, as the
   projected `SketchPoint`.
2. Draw the **Anchor Line** from two raw seeds, 0.5 cm either side of the projected point along the
   sketch's own x axis: with `g` the projected point's `geometry`, `startSeed` is
   `adsk.core.Point3D.create(gx, gy, 0)` with `gx = g.x - 0.5`, `gy = g.y`, and `endSeed` the same
   with `gx = g.x + 0.5`. Then
   `line = sketch.sketchCurves.sketchLines.addByTwoPoints(startSeed, endSeed)`, so its end is to
   the right of its start.
3. Constrain it with these four and nothing else, the bevel gear's Anchor recipe:
   `sketch.geometricConstraints.addCoincident(centre, line)`,
   `sketch.geometricConstraints.addMidPoint(centre, line)` (both, not the midpoint alone),
   `sketch.geometricConstraints.addHorizontal(line)` (sketch-local, so it survives a tilted
   plane, [PB-REFLINE-DIRECTION]), and a **horizontal** distance dimension from the line's start
   to its end, `sketch.sketchDimensions.addDistanceDimension(line.startSketchPoint,
   line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
   textPoint)` with `textPoint` the point 0.2 cm above the projected point in sketch coordinates,
   then `dimension.parameter.value = 1.0` (10 mm, in cm) [PB-DIM-VALUE-SEMANTICS]. Not an aligned
   dimension: midpoint, horizontal and an aligned length admit the line end for end.
4. Raise naming `Anchor` unless `sketch.isFullyConstrained` [PB-FULL-CONSTRAINT]. Only then read
   the frame from world geometry [PB-WORLDGEO-CONSTRAINED] [PB-WORLD-FRAME]: `C` is the projected
   point's `worldGeometry`; `ê` is the unit vector from `line.startSketchPoint.worldGeometry` to
   `line.endSketchPoint.worldGeometry`. Keep the line as `self.anchorLine`; it is passed once more,
   to the Window Plane of S21. No later sketch projects anything.

The proof is `stepAnchorSketch`, run with `anchorCases`: the projection is a reference point, the
midpoint carries the coincident row (so the proof writes the midpoint alone), and the sketch
solves to one configuration with the line 10 mm long running along +x.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

## S07 `[PROSE]` Gear A Axis Plane, and the sign of n̂

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetA))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetA))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "designComponent.constructionPlanes",
      "role": "required",
      "span": "designComponent.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 791,
      "last": 813,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 476,
      "last": 485,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L791–813; `spec/screwgear/fusion.md` L476–485.

One construction plane, `Gear A Axis Plane`, offset from the selected plane itself
[PB-USE-SELECTED-PLANE] [PB-CONSTRUCTION-PLANES]: `planeInput =
designComponent.constructionPlanes.createInput()`, then
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetA))` with
`offsetA = -A/2/10` cm, then `designComponent.constructionPlanes.add(planeInput)`; name it.

Read `n̂` from it, never assuming the sign Fusion offset along [SCREW-F-NORMAL-SIGN]: take the
plane's `geometry`, a `Plane` with `origin` and `normal`; normalise `normal`; if
`(C - origin)·normal > 0` then `n̂ = normal`, else `n̂ = -normal`. So `C` lies `+A/2` along `n̂`
from gear A's plane. Then the rest of the world frame, in centimetres:

```
k̂ = n̂ × ê
dir_A = cos(Sigma/2)*ê + sin(Sigma/2)*k̂      origin_A = C - (A/2/10)*n̂      û_A = +n̂
dir_B = cos(Sigma/2)*ê - sin(Sigma/2)*k̂      origin_B = C + (A/2/10)*n̂      û_B = -n̂
v̂_g = dir_g × û_g
ang_g(s) = s/Lambda + Phi_g                 (s in mm; Phi_A = mountAngleA, Phi_B = mountAngleB)
wpt_g(u, v, s) = origin_g + (s/10)*dir_g + ((u*cos(th) - v*sin(th))/10)*û_g
                              + ((u*sin(th) + v*cos(th))/10)*v̂_g,       th = ang_g(s)
```

`û` points at the other gear on both, which is what makes the two gears the same part. Every
later sketch seed and reference point is a `world_g` point or a point built from `C`, `ê`, `k̂`
and `n̂`.

Not built by the proof: the sketch engine places sketches on planes but reads nothing back from
how Fusion makes one, and the sign is what this step is about. The comment at the head of
`proof/screwgear/compiled_model_test.go` says so.

## S08 `[PROSE]` Gear B Axis Plane, and the check on both

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "designComponent.constructionPlanes",
      "role": "required",
      "span": "planeInput = designComponent.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetB))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetB))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "designComponent.constructionPlanes",
      "role": "required",
      "span": "designComponent.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 807,
      "last": 813,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 476,
      "last": 485,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L807–813; `spec/screwgear/fusion.md` L476–485.

One construction plane, `Gear B Axis Plane`, offset from the selected plane by `+A/2`:
`planeInput = designComponent.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(offsetB))` with
`offsetB = +A/2/10` cm, `designComponent.constructionPlanes.add(planeInput)`; name it. Then check
both planes [SCREW-F-NORMAL-SIGN] [PB-SELF-DIAGNOSING]: for each, with its `geometry`'s `origin` and
unit `normal`, `|(C - origin)·normal|` must equal `A/2/10` cm within `1e-5` cm; raise naming the
plane, the distance read and `A/2` otherwise. No later plane is offset from the selected plane.

Not built by the proof, for the reason S07 gives.

## S09 `[GO]` Paths sketch, per gear

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "designComponent.sketches",
      "role": "required",
      "span": "designComponent.sketches.add(axisPlane)"
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
      "span": "sketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)"
    }
  ],
  "citations": [
    {
      "first": 815,
      "last": 834,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 635,
      "last": 653,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 433,
      "last": 463,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L815–834; `spec/screwgear/instructions.md` L635–653; `spec/screwgear/fusion.md` L433–463.

`buildSweepPaths`, once per gear, gear A first. One sketch named `Gear A Paths` (or
`Gear B Paths`) on that gear's Axis Plane: `designComponent.sketches.add(axisPlane)`. Not deferred.

Four reference points on the gear's axis [SCREW-F-REFERENCES] [PB-PROJECT-NOT-FIXED], at stations
`-sOut`, `-sIn`, `sIn`, `sOut` (S03; 19 and 7.967 mm at the defaults): for each,
`local = sketch.modelToSketchSpace(worldPoint)` with `worldPoint = origin_g + (s/10)*dir_g`, then
`local.z = 0` [PB-SKETCH-ZERO-Z] [PB-SPACE-METHODS], then `sketch.sketchPoints.add(local)`. Two solid
lines sharing those points [PB-SHARE-XOR-COINCIDENT], each from its negative station to its
positive one so it runs along `+dir_g`:
`sketch.sketchCurves.sketchLines.addByTwoPoints(p0, p1)` for `bore-` (from `-sOut` to `-sIn`) and
the same with `p2, p3` for `bore+` (from `sIn` to `sOut`). Then set `isFixed = True` on all four
points, after both lines exist [PB-PROJECT-NOT-FIXED]. Nothing else: no constraint, no dimension.
Raise naming the sketch unless `sketch.isFullyConstrained` [PB-FULL-CONSTRAINT]. Store the two
lines in `self.pathLines[index]` under `'bore-'` and `'bore+'`; S18 and S20 use them.

The proof is `stepGearPathsSketch`, run with `pathsCases`: it draws the four fixed points and two
lines in the axis plane's coordinates, holds each line `sOut - sIn` long running along the gear's
axis, holds `sIn > 0`, and at the defaults reads `sIn = 7.967` and `sOut = 19`.

<!-- proof-run: proofkit.Run(pathsCases, stepGearPathsSketch) -->

## S10 `[GO]` Cell Sections sketch, per gear

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "designComponent.sketches",
      "role": "required",
      "span": "designComponent.sketches.add(axisPlane)"
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
      "span": "sketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(a, b)"
    }
  ],
  "citations": [
    {
      "first": 836,
      "last": 864,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 886,
      "last": 910,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 654,
      "last": 661,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 683,
      "last": 693,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 207,
      "last": 236,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L836–864; `spec/screwgear/instructions.md` L886–910; `spec/screwgear/instructions.md` L654–661; `spec/screwgear/instructions.md` L683–693; `spec/screwgear/fusion.md` L207–236.

`buildToothCell`. One sketch named `Gear A Cell Sections` (or `Gear B Cell Sections`) on
the gear's Axis Plane, `designComponent.sketches.add(axisPlane)`, with
`sketch.isComputeDeferred = True` set right after it is created and named, before its first point
[PB-SKETCH-DEFER] [SCREW-F-DEFER]. No section plane is made [PB-3D-SKETCH-SECTIONS].

The cell is `c` teeth (S02) at the ribbon's negative end. Its sections `k = 0 … c*n` stand at

```
Z0  = 0 for gear A, Z0B (assemblyPhase) for gear B
s0  = Z0 - L/2                       (gear A: -89.25 mm at the defaults)
s_k = s0 + k*P/n
th_k = s_k/Lambda + Phi_g
Utooth(s) = W/2 - H/2 + (H/2)*cos(2*pi*(s - Z0)/P)
```

Section `k` is the rectangle with corners, in section coordinates `(u, v)`,
`(uB, -hv)`, `(uF, -hv)`, `(uF, hv)`, `(uB, hv)` in that order, `uB = -W/2`, `uF = Utooth(s_k)`,
`hv = T/2`, each placed at `wpt_g(u, v, s_k)`. For each corner
`local = sketch.modelToSketchSpace(worldPoint)` and `sketch.sketchPoints.add(local)` **with its z
kept**: these corners lie off the sketch's plane on purpose, and this is the one sketch where
[PB-SKETCH-ZERO-Z] does not apply [SCREW-F-CELL-LOFT]. Four solid lines share them in order,
`sketch.sketchCurves.sketchLines.addByTwoPoints(a, b)` for `L1` corner 0 to 1, `L2` 1 to 2, `L3`
2 to 3, `L4` 3 to 0 [PB-SHARE-XOR-COINCIDENT]; keep each section's four lines together, in order
of `k`, for S11. After the last line of the last section, set `isFixed = True` on every point of
the sketch. Nothing else is in the sketch: no construction line, no constraint, no dimension.

Then set `sketch.isComputeDeferred = False`, raise naming the sketch unless
`sketch.isFullyConstrained`, and raise unless `sketch.profiles.count` is `c*n + 1` (41 at the
defaults), with the count read in the message [PB-SELF-DIAGNOSING]. The profiles are not used.

The same recipe makes the **remainder cell** of S12 in a sketch named `Gear A Cell Remainder` (or
`Gear B Cell Remainder`), with `c` replaced by `r` and section `k` at `s0 + q*c*P + k*P/n` for
`k = 0 … r*n`.

The proof is `stepCellSectionsSketch`, run with `cellSketchCases`. Its stand-in for the one 3D
sketch is each section drawn in its own station's plane and laid side by side: it holds one
profile per section, `teeth*n + 1` in all, each the rectangle's area `T*(Utooth(s_k) + W/2)`, and
`n = 10` at the defaults and the floor of 8 at a 400 mm lead. Fusion's own verdict on the sketch
with off-plane points was measured on 2026-09-28 [SCREW-F-CELL-LOFT].

<!-- proof-run: proofkit.Run(cellSketchCases, stepCellSectionsSketch) -->

## S11 `[GO]` Cell loft, per gear

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.LoftFeatures",
      "reason": null,
      "receiver": "designComponent.features.loftFeatures",
      "role": "required",
      "span": "loftInput = designComponent.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
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
      "receiver": "designComponent.features",
      "role": "required",
      "span": "path = designComponent.features.createPath(collection, False)"
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
      "receiver": "designComponent.features.loftFeatures",
      "role": "required",
      "span": "loftFeature = designComponent.features.loftFeatures.add(loftInput)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.Path",
      "reason": "Path.create raises InternalValidationError on a sketch curve in a sub-component [PB-PATH-FROM-SKETCH]",
      "receiver": "adsk.fusion.Path",
      "role": "forbidden",
      "span": "adsk.fusion.Path.create(collection, False)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "loftFeature.bodies",
      "role": "required",
      "span": "loftFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 866,
      "last": 884,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 912,
      "last": 919,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 228,
      "last": 253,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L866–884; `spec/screwgear/instructions.md` L912–919; `spec/screwgear/fusion.md` L228–253.

One loft through every section of S10, in station order [PB-LOFT] [SCREW-F-CELL-LOFT]:
`loftInput = designComponent.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`;
then for each section `k = 0 … c*n` in order, `collection = adsk.core.ObjectCollection.create()`,
`collection.add(line)` for its four lines `L1`, `L2`, `L3`, `L4`,
`path = designComponent.features.createPath(collection, False)` [PB-PATH-FROM-SKETCH], and
`loftInput.loftSections.add(path)`; then
`loftFeature = designComponent.features.loftFeatures.add(loftInput)`. Never fall back to
`adsk.fusion.Path.create(collection, False)`, which raises in this component. Set nothing else
on the input: there is no ruled option and none is wanted.

Raise naming the gear and the count unless `loftFeature.bodies.count` is 1 and that body,
`loftFeature.bodies.item(0)`, `isSolid` [PB-EMPTY-RESULT] [PB-SELF-DIAGNOSING]. That body is the
gear's body from here on. The remainder cell of S12 is lofted the same way from its own sketch.

The proof is `stepCellLoft` with `assertCellLoft`, run with `cellLoftCases`. decad lofts between
two sections only, so the proof lofts each neighbouring pair as a sheet, patches the two ends and
stitches one solid, and holds it to the sections: its box is the box of the section corners, its
faces are two caps and two triangles per wall per step, and its volume is the ruled loft's up to
the fold of each wall into two triangles.

<!-- proof-run: proofkit3d.RunSolid(cellLoftCases, stepCellLoft, assertCellLoft) -->

## S12 `[PROSE]` The doubling schedule, the remainder and the gear's name

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 921,
      "last": 969,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L921–969.

Not a timeline entry; `repeatCellByDoubling`, which orders S13–S15. The ribbon is the cell,
`c` teeth, repeated under the **screw step** `Step(k)`: a translation of `k*P` along the gear's
axis composed with a rotation of `k*P/Lambda` about it (S14). With `q` and `r` from S02:

1. Write `q` in binary. Start with the cell, `m = 1` (cells the body holds).
2. For each bit of `q` below its top bit, lowest first: if that bit is set, take a copy of the
   body (S13) and keep it unmoved, an **aside** of `m` cells; then **double**: copy the body
   (S13), move the copy by `Step(m*c)` (S14), join it to the body (S15), `m = 2*m`.
3. Then move each aside, largest first, by `Step(m*c)` (S14), join it (S15), and add its cells to
   `m`. The last join brings `m` to `q`; raise unless it does.

That is floor(log2 q) + popcount(q) - 1 rounds. At the defaults, `q = 17`: one aside of the
single cell, taken before the first doubling, four doublings to 16 cells, and the aside moved by
`Step(64)` (64 teeth): 5 rounds, 15 features. When `q = 1` there is no round. Every move is by a
whole number of teeth, never zero, so no zero-angle guard is needed; never add a `k = 0` move
[PB-MOVE-ROTATE].

**The remainder.** When `r > 0`, after the last round, build a second, shorter cell where it
belongs: the Cell Remainder sketch of S10 and its loft by S11, with `r*n + 1` sections from station
`s0 + q*c*P` to `s0 + N*P`, then join it to the body by S15. The defaults have none.

**The name.** After the last join, name the body `Gear A` (or `Gear B`) and store it in
`self.gearBodies[index]`.

## S13 `[GO]` Copy the body

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CopyPasteBodies",
      "reason": null,
      "receiver": "designComponent.features.copyPasteBodies",
      "role": "required",
      "span": "copyFeature = designComponent.features.copyPasteBodies.add(body)"
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
      "first": 961,
      "last": 962,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 198,
      "last": 205,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L961–962; `spec/screwgear/fusion.md` L198–205.

One `CopyPasteBody` feature: `copyFeature = designComponent.features.copyPasteBodies.add(body)`.
The new body is the feature's own, `copyFeature.bodies.item(0)` [SCREW-F-COPY-BODY]; never read
the feature's `sourceBody`, which is the original. An aside of S12 is this step alone; a doubling
is this step then S14 and S15 on the copy.

The proof is `stepCopyBody` with `assertCopyBody`, run with `copyCases`: decad's `Duplicate`
leaves the source live and gives an identical body, held to the source's volume and corners. The
source is set aside down the axis in the proof's document, because two coincident solids are not a
document decad verifies as sound.

<!-- proof-run: proofkit3d.RunSolid(copyCases, stepCopyBody, assertCopyBody) -->

## S14 `[GO]` Screw-move the copy

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "adsk.core.Vector3D",
      "role": "required",
      "span": "axisVector = adsk.core.Vector3D.create(dx, dy, dz)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "axisPoint = adsk.core.Point3D.create(ox, oy, oz)"
    },
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
      "span": "rot.setToRotation(angle, axisVector, axisPoint)"
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
      "span": "shift.scaleBy(distance)"
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
      "span": "bodies.add(copyBody)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "designComponent.features.moveFeatures",
      "role": "required",
      "span": "moveInput = designComponent.features.moveFeatures.createInput2(bodies)"
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
      "receiver": "designComponent.features.moveFeatures",
      "role": "required",
      "span": "designComponent.features.moveFeatures.add(moveInput)"
    }
  ],
  "citations": [
    {
      "first": 923,
      "last": 927,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 961,
      "last": 969,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 161,
      "last": 196,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L923–927; `spec/screwgear/instructions.md` L961–969; `spec/screwgear/fusion.md` L161–196.

One move feature taking the copy by `Step(k)`, `k` the teeth S12 names (`m*c`). Build the matrix
from two matrices [SCREW-F-SCREW-STEP], all lengths in cm:

1. `axisVector = adsk.core.Vector3D.create(dx, dy, dz)` with `(dx, dy, dz)` the gear's unit
   `dir_g`, and `axisPoint = adsk.core.Point3D.create(ox, oy, oz)` with `(ox, oy, oz)` the gear's
   `origin_g`.
2. `rot = adsk.core.Matrix3D.create()`, then `rot.setToRotation(angle, axisVector, axisPoint)`
   with `angle = k*P/Lambda` radians.
3. `shift = axisVector.copy()`, `shift.scaleBy(distance)` with `distance = k*P/10` cm, then
   `mov = adsk.core.Matrix3D.create()` and `mov.translation = shift`, assigned whole.
4. `rot.transformBy(mov)`.

Never build the rotation and then assign `rot.translation`: that overwrites the translation the
rotation already carries and turns the rotation about the gear's axis into one about a parallel
line through the world origin.

Move it [PB-MOVE-ROTATE]: `bodies = adsk.core.ObjectCollection.create()`,
`bodies.add(copyBody)`, `moveInput = designComponent.features.moveFeatures.createInput2(bodies)`,
`moveInput.defineAsFreeMove(rot)`, `designComponent.features.moveFeatures.add(moveInput)`.

The proof is `stepScrewMove` with `assertScrewMove`, run with `moveCases`: the cell moved by
`Step(shift*c)` for every shift the default schedule makes is held to the cell built in place that
many cells along, in a document of its own: the same volume, every vertex on a corner of that cell,
the same centroid.

<!-- proof-run: proofkit3d.RunSolid(moveCases, stepScrewMove, assertScrewMove) -->

## S15 `[GO]` Join the copy

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
      "span": "tools.add(toolBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "designComponent.features.combineFeatures",
      "role": "required",
      "span": "combineInput = designComponent.features.combineFeatures.createInput(body, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "designComponent.features.combineFeatures",
      "role": "required",
      "span": "combineFeature = designComponent.features.combineFeatures.add(combineInput)"
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
      "first": 943,
      "last": 966,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 401,
      "last": 411,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L943–966; `spec/screwgear/fusion.md` L401–411.

One combine feature joining the moved copy, a placed aside, or the remainder cell into the body
[SCREW-F-JOIN]: `tools = adsk.core.ObjectCollection.create()`, `tools.add(toolBody)`,
`combineInput = designComponent.features.combineFeatures.createInput(body, tools)`, then
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`,
`combineInput.isKeepToolBodies = False`, then
`combineFeature = designComponent.features.combineFeatures.add(combineInput)`. Raise naming the
gear, the round and the count unless `combineFeature.bodies.count` is 1 [PB-EMPTY-RESULT]
[PB-SELF-DIAGNOSING]; `combineFeature.bodies.item(0)` is the body from then on. Each join meets
its neighbour at one shared planar cross-section, with no sliver and no overlap, because every move
is by whole teeth already built.

The proof is `stepJoinCopy` with `assertJoinCopy`, run with `joinCases`. It checks the schedule of
S12 (round count, each copy moved by the cells the body holds, the last join reaching `q`), and
proves every join of the case at its seam: the body's last cell and the joined piece's first cell,
lofted from one station list into one stitched solid, whose volume agrees with the two cells built
apart. decad will not join two solids that meet on a face, and will not stitch the whole 68-tooth
ribbon (its audit ceiling falls between 48 and 60 teeth), which the proof file records.

<!-- proof-run: proofkit3d.RunSolid(joinCases, stepJoinCopy, assertJoinCopy) -->

## S16 `[GO]` Sleeve sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "designComponent.sketches",
      "role": "required",
      "span": "designComponent.sketches.add(self.plane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "local = sketch.modelToSketchSpace(C)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchCircles",
      "role": "required",
      "span": "sketch.sketchCurves.sketchCircles.addByCenterRadius(local, radius)"
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
      "span": "sketch.profiles.item(i)"
    }
  ],
  "citations": [
    {
      "first": 994,
      "last": 1002,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 361,
      "last": 371,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L994–1002; `spec/screwgear/fusion.md` L361–371.

`buildCage` starts here. One sketch named `Sleeve` on the selected plane,
`designComponent.sketches.add(self.plane)` [PB-USE-SELECTED-PLANE] [SCREW-F-SLEEVE]. Not deferred.
Map `C` in: `local = sketch.modelToSketchSpace(C)`, `local.z = 0` [PB-SKETCH-ZERO-Z]. Two circles
at it, `sketch.sketchCurves.sketchCircles.addByCenterRadius(local, radius)`, radius `Ri/10` then
`Ro/10` cm. On each, set `circle.centerSketchPoint.isFixed = True` [PB-CIRCLE-CENTER] (two separate
centre points at the same place, no coincident between them) and add a diameter dimension
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` with `textPoint` on the circle,
at `local` plus `(radius, 0, 0)`, never at the centre [PB-RADIAL-DIM], then
`dimension.parameter.value = 2*radius`. Nothing else is in the sketch. Raise naming `Sleeve`
unless `sketch.isFullyConstrained`.

The sketch has two profiles, the disc inside `Ri` and the ring. Iterate the profiles,
`sketch.profiles.item(i)` for `i` below `sketch.profiles.count`, and keep those whose
`profileLoops.count` is 2; raise with the counts unless exactly one does [PB-EMPTY-RESULT]. That
one is the ring. The curve-count profile finder cannot pick it, since it treats a circle as a type
that disqualifies a loop [PB-PROFILE-MATCH].

The proof is `stepSleeveSketch`, run with `sleeveSketchCases`: two fixed-centre circles with their
diameters, two profiles, exactly one with a hole, the ring's area `pi*(Ro^2 - Ri^2)`.

<!-- proof-run: proofkit.Run(sleeveSketchCases, stepSleeveSketch) -->

## S17 `[GO]` Sleeve extrude

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "designComponent.features.extrudeFeatures",
      "role": "required",
      "span": "extrudeInput = designComponent.features.extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "setSymmetricExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(riseCm), False)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(riseCm), False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "designComponent.features.extrudeFeatures",
      "role": "required",
      "span": "extrudeFeature = designComponent.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1002,
      "last": 1006,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 371,
      "last": 375,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1002–1006; `spec/screwgear/fusion.md` L371–375.

Extrude the ring as a new body, both ways from the selected plane:
`extrudeInput = designComponent.features.extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(riseCm), False)` with
`riseCm = rise/10`, where `False` makes the value each side's length [PB-THROUGH-CUT], then
`extrudeFeature = designComponent.features.extrudeFeatures.add(extrudeInput)`. The tube runs from
`-rise` to `+rise` along `n̂`, 37.5 mm tall at the defaults. Raise unless
`extrudeFeature.bodies.count` is 1; `extrudeFeature.bodies.item(0)` is `self.cageBody` from here on.

The proof is `stepSleeveTube` with `assertSleeveTube`, run with `sleeveTubeCases`: one solid of
volume `pi*(Ro^2 - Ri^2)*2*rise` and box `±Ro` by `±rise`.

<!-- proof-run: proofkit3d.RunSolid(sleeveTubeCases, stepSleeveTube, assertSleeveTube) -->

## S18 `[PROSE]` Bore plane, per bore

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "designComponent.constructionPlanes",
      "role": "required",
      "span": "planeInput = designComponent.constructionPlanes.createInput()"
    },
    {
      "condition": null,
      "name": "setByDistanceOnPath",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(boreLine, adsk.core.ValueInput.createByReal(0))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(boreLine, adsk.core.ValueInput.createByReal(0))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "designComponent.constructionPlanes",
      "role": "required",
      "span": "designComponent.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1023,
      "last": 1027,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 829,
      "last": 834,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 279,
      "last": 282,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1023–1027; `spec/screwgear/instructions.md` L829–834; `spec/screwgear/fusion.md` L279–282.

The bores are built in this order, each S18, S19, S20 before the next: gear A `-R`, gear A `+R`,
gear B `-R`, gear B `+R`. A bore's line is `self.pathLines[g]['bore-']` for `-R` and `['bore+']`
for `+R`; its first station `s0b` is `-sOut` for `-R` and `sIn` for `+R`.

One construction plane named `Gear A Bore -R Plane` (and so on, `{gearLabel} Bore {-R|+R} Plane`)
[PB-CONSTRUCTION-PLANES]: `planeInput = designComponent.constructionPlanes.createInput()`,
`planeInput.setByDistanceOnPath(boreLine, adsk.core.ValueInput.createByReal(0))` with the sketch
line passed directly, never through a path object, then
`designComponent.constructionPlanes.add(planeInput)`. It stands square to the gear's axis at the
line's start, the station `s0b`, where the axis point is `origin_g + (s0b/10)*dir_g`.

Not built by the proof: S19 draws on this plane in its own coordinates, and where Fusion puts the
plane's origin is not something the sketch engine reads.

## S19 `[GO]` Bore section sketch, per bore

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "designComponent.sketches",
      "role": "required",
      "span": "designComponent.sketches.add(borePlane)"
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
      "span": "sketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(o, cp)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDistanceDimension(o, e, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addAngularDimension(ru, k, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addAngularDimension(ru, l2, textPoint)"
    },
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
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(l3, k)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(k, l3, textPoint)"
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
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(l4, l2)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(l2, l4, textPoint)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": "the framework helper in lib/geargen/utilities.py [PB-PROFILE-MATCH]",
      "receiver": null,
      "role": "inherited",
      "span": "find_profile_by_curve_counts(sketch, lines=4)"
    }
  ],
  "citations": [
    {
      "first": 1045,
      "last": 1080,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 662,
      "last": 682,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 279,
      "last": 287,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1045–1080; `spec/screwgear/instructions.md` L662–682; `spec/screwgear/fusion.md` L279–287.

One sketch named `Gear A Bore -R` (and so on, `{gearLabel} Bore {-R|+R}`) on the bore's plane,
`designComponent.sketches.add(borePlane)`, with `sketch.isComputeDeferred = True` from right after
it is named until after the last dimension [SCREW-F-DEFER] [PB-SKETCH-DEFER]. Nothing is projected.

Values, in mm: `s = s0b`, `th = ang_g(s)`, `uB = -hw`, `uF = hw`, `hv = ht`, and the section
directions `û(th) = cos(th)*û_g + sin(th)*v̂_g` and `v̂(th) = -sin(th)*û_g + cos(th)*v̂_g`. Every
point below is a world point `O + …` in cm, mapped with
`local = sketch.modelToSketchSpace(worldPoint)` and given `local.z = 0` before any use, seeds,
reference points and text points alike [PB-SKETCH-ZERO-Z]; every seed is the solved position
[PB-SEED-NEAR].

1. **References.** `O = origin_g + (s/10)*dir_g`, and `Cp = O + (A/2/10)*û_g` (along the gear's
   unrotated `û`). Add each with `sketch.sketchPoints.add(local)`.
2. **Ru and the spine K.** `Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(o, cp)` sharing
   `O` and `Cp`, and `K` the same from `O` to a raw seed `E = O + (uF/10)*û(th)`; set
   `isConstruction = True` on both. Then set `isFixed = True` on `O` and `Cp`, before any
   dimension.
3. **The rectangle.** Four solid lines from raw seeds at the solved corners, sharing corners
   [PB-SHARE-XOR-COINCIDENT]: `L1` from `O + (uB*û(th) - hv*v̂(th))/10` to
   `O + (uF*û(th) - hv*v̂(th))/10`, `L2` from that point to `O + (uF*û(th) + hv*v̂(th))/10`, `L3`
   on to `O + (uB*û(th) + hv*v̂(th))/10`, `L4` back to `L1`'s start; each later line starts on the
   previous line's `endSketchPoint` and `L4` ends on `L1`'s `startSketchPoint`. No coincident on a
   corner.
4. **Constraints and dimensions**, in this order [PB-OFFSET-DIM] [PB-NO-OVERCONSTRAIN]
   [PB-DIM-VALUE-SEMANTICS] (each value is written with `dimension.parameter.value`, in cm or
   radians):
   - `sketch.sketchDimensions.addDistanceDimension(o, e, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
     from `O` to `K`'s end `E`, value `uF/10`, text point `O + (uF/2*û(th) + 0.5*v̂(th))/10`.
   - The angle [PB-ANGULAR-DIM]. When `|sin(th)| >= sqrt(1/2)`:
     `sketch.sketchDimensions.addAngularDimension(ru, k, textPoint)` between `Ru` and `K`, value
     `acos(cos(th))` (the angle between the ray from `O` toward `Cp` and the ray from `O` toward
     `E`), text point `O + (hw/2/10)*unit(û_g + û(th))`. Otherwise
     `sketch.sketchDimensions.addAngularDimension(ru, l2, textPoint)` between `Ru` and `L2`, value
     `acos(-sin(th))` (the angle between the ray along `+û_g` and the ray along `+v̂(th)`, from
     the point `X = O + (uF/cos(th)/10)*û_g` where the two lines meet), text point
     `X + (hw/2/10)*unit(û_g + v̂(th))`. Either way the value lies between 45° and 135°.
   - `sketch.geometricConstraints.addParallel(l1, k)` and
     `sketch.sketchDimensions.addOffsetDimension(k, l1, textPoint)`, value `hv/10`, text point
     `O + (uF/2*û(th) - hv/2*v̂(th))/10`.
   - `sketch.geometricConstraints.addParallel(l3, k)` and
     `sketch.sketchDimensions.addOffsetDimension(k, l3, textPoint)`, value `hv/10`, text point
     `O + (uF/2*û(th) + hv/2*v̂(th))/10`.
   - `sketch.geometricConstraints.addCoincident(e, l2)`: `E` on `L2`.
   - `sketch.geometricConstraints.addPerpendicular(l2, k)`.
   - `sketch.geometricConstraints.addParallel(l4, l2)` and
     `sketch.sketchDimensions.addOffsetDimension(l2, l4, textPoint)`, value `(uF - uB)/10`, text
     point `O + ((uF + uB)/2*û(th))/10`.

   Ten degrees of freedom, `E` and four corners, against ten rows; no `addPerpendicular` is put on
   a line only one of whose ends is anchored.
5. Set `sketch.isComputeDeferred = False`, raise naming the sketch unless
   `sketch.isFullyConstrained`, and take the profile with
   `find_profile_by_curve_counts(sketch, lines=4)` [PB-PROFILE-MATCH]: the one loop of four lines.
   The construction lines bound nothing.

The proof is `stepBoreSectionSketch`, run with `boreSketchCases`: the scheme closes, unambiguous,
at all four default bores and at mounting angles that put the section within 45° of 0° or 180°,
where the angle is taken against `L2`; every corner and `E` solve to the seeds; the one profile is
the `2*hw` by `2*ht` rectangle; and the written angle lies between 45° and 135°. Fusion's
parallel-plus-offset is the engine's `NewOffset` there.

<!-- proof-run: proofkit.Run(boreSketchCases, stepBoreSectionSketch) -->

## S20 `[GO]` Bore sweep cut, per bore

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "designComponent.features",
      "role": "required",
      "span": "path = designComponent.features.createPath(boreLine, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "designComponent.features.sweepFeatures",
      "role": "required",
      "span": "sweepInput = designComponent.features.sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "sweepInput.twistAngle = adsk.core.ValueInput.createByReal(twist)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "designComponent.features.sweepFeatures",
      "role": "required",
      "span": "sweepFeature = designComponent.features.sweepFeatures.add(sweepInput)"
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
      "first": 1008,
      "last": 1043,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1082,
      "last": 1117,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 289,
      "last": 337,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1008–1043; `spec/screwgear/instructions.md` L1082–1117; `spec/screwgear/fusion.md` L289–337.

One twisted sweep cut of the bore's rectangle along its line, from the cage alone
[SCREW-F-TWISTED-SLOT] [PB-SWEEP-TWIST] [PB-PATH-FROM-SKETCH]:
`path = designComponent.features.createPath(boreLine, False)`, then
`sweepInput = designComponent.features.sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
then `sweepInput.twistAngle = adsk.core.ValueInput.createByReal(twist)` with
`twist = +(sOut - sIn)/Lambda` radians (80.24° at the defaults; positive, because the line runs
along `+dir_g` and the section's angle grows with `s`), then
`sweepInput.participantBodies = [self.cageBody]`, then
`sweepFeature = designComponent.features.sweepFeatures.add(sweepInput)`. Set nothing else: not
`orientation`, not `solidTwistAxis`, no rail [SCREW-F-NO-SOLID-TWIST].

Check it [SCREW-F-SWEEP-CHECK] [PB-SELF-DIAGNOSING]: raise naming the bore unless
`sweepFeature.bodies.count` is 1, and take `sweepFeature.bodies.item(0)` as `self.cageBody`. Then
probe the cage at the crossing `sc = σ*R` (σ = -1 for `-R`, +1 for `+R`) at the two points

```
probe± = origin_g + (sc/10)*dir_g ± ((W/2 + clr/2)/10) * (cos(th)*û_g + sin(th)*v̂_g),   th = ang_g(sc)
```

with `self.cageBody.pointContainment(probe)` for each; both must be
`adsk.fusion.PointContainment.PointOutsidePointContainment`, the channel being open there. Raise
naming the bore and the containment read otherwise. At the defaults the probes stand 16.80 mm
(`-R`) and 16.30 mm (`+R`) from the frame's axis, inside the wall; under the wrong twist sense they
would sit 7.43 and 6.46 mm across the channel, in the wall.

The proof is `stepBoreSweepCut` with `assertBoreSweepCut`, run with `boreCutCases`. decad has no
twisted sweep, so each bore is the stand-in the spec prescribes: the rectangle turned to the
ribbon's angle at `boreSections` stations (18 at the defaults), lofted and stitched. decad will not
cut twice from a faceted result, so the four cuts are one cut of the four tools' union, which
removes the same material since no two channels meet. The cage after it is one solid, its volume
the tube's less each channel's section in the wall integrated along its span, within the
stand-in's facet and triangle-fold bound.

<!-- proof-run: proofkit3d.RunSolid(boreCutCases, stepBoreSweepCut, assertBoreSweepCut) -->

## S21 `[PROSE]` Window Plane

<!-- step-meta
{
  "calls": [
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
      "receiver": "designComponent.constructionPlanes",
      "role": "required",
      "span": "designComponent.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1296,
      "last": 1301,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 376,
      "last": 383,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1296–1301; `spec/screwgear/fusion.md` L376–383.

After the four bores, and only when `self.windows` is not empty: one construction plane named
`Window Plane` through the frame's axis, square to the windows' facing direction
[PB-CONSTRUCTION-PLANES] [SCREW-F-SLEEVE]. `planeInput =
designComponent.constructionPlanes.createInput()`; then, for windows facing `±k̂`
(`Sigma <= 90°`), `planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)`,
the plane through the Anchor Line square to the selected plane; for windows facing `±ê`,
`planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))`, the
plane square to the Anchor Line at its midpoint, `C`. Then
`designComponent.constructionPlanes.add(planeInput)`.

Not built by the proof, for the reason S07 gives; S22 draws on it in its own coordinates.

## S22 `[GO]` Window sketch, per window

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "designComponent.sketches",
      "role": "required",
      "span": "designComponent.sketches.add(windowPlane)"
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
      "span": "sketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(a, b)"
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
      "first": 1301,
      "last": 1308,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 384,
      "last": 387,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1301–1308; `spec/screwgear/fusion.md` L384–387.

For each window in `self.windows`, in order (facing `d` before `-d`), one sketch named
`Window +k` (or `-k`, `+e`, `-e`) on the Window Plane, `designComponent.sketches.add(windowPlane)`.
Not deferred. With `d` and `across = n̂ × d` taken in the world (`d = dx*ê + dy*k̂` from the
search frame's components), each hexagon corner `(t, z)` of S04 is the world point
`C + (t/10)*across + (z/10)*n̂`, mapped with `local = sketch.modelToSketchSpace(worldPoint)`,
`local.z = 0` [PB-SKETCH-ZERO-Z], and added with `sketch.sketchPoints.add(local)`. One solid line
from each corner to the next and from the last back to the first,
`sketch.sketchCurves.sketchLines.addByTwoPoints(a, b)`, sharing the points
[PB-SHARE-XOR-COINCIDENT]; then `isFixed = True` on every point. Nothing else. Raise naming the
sketch unless `sketch.isFullyConstrained`, and unless `sketch.profiles.count` is 1; the profile is
`sketch.profiles.item(0)` [PB-SINGLE-PROFILE]. The two windows are two sketches because their
hexagons cross on the shared plane.

The proof is `stepWindowSketch`, run with `windowSketchCases`: the hexagon from the hand-written
`newWindow` makes one valid profile of its shoelace area, and at the defaults the `+k̂` window reads
the sides, trims, ends and corners S04 quotes.

<!-- proof-run: proofkit.Run(windowSketchCases, stepWindowSketch) -->

## S23 `[GO]` Window extrude cut, per window

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
      "receiver": "designComponent.features.extrudeFeatures",
      "role": "required",
      "span": "extrudeInput = designComponent.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "depth = adsk.core.ValueInput.createByReal(depthCm)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "extent = adsk.fusion.DistanceExtentDefinition.create(depth)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, direction)"
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
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "designComponent.features.extrudeFeatures",
      "role": "required",
      "span": "extrudeFeature = designComponent.features.extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrudeFeature.bodies",
      "role": "required",
      "span": "extrudeFeature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 1310,
      "last": 1332,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 388,
      "last": 399,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1310–1332; `spec/screwgear/fusion.md` L388–399.

For each window, right after its sketch. First the probe [PB-SELF-DIAGNOSING]: with `(tc, zc)`
the average of the hexagon's corners,

```
probe = C + (tc/10)*across + (zc/10)*n̂ + ((a0(tc) + a1(tc))/2/10)*d
```

the middle of the wall where the window goes. `self.cageBody.pointContainment(probe)` must be
`adsk.fusion.PointContainment.PointInsidePointContainment`; raise naming the window and the
containment read otherwise.

Then one extrude cut from the cage alone [PB-THROUGH-CUT] [SCREW-F-SLEEVE]:
`extrudeInput = designComponent.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`;
`depth = adsk.core.ValueInput.createByReal(depthCm)` with `depthCm = (Ro + 1)/10`;
`extent = adsk.fusion.DistanceExtentDefinition.create(depth)`;
`extrudeInput.setOneSideExtent(extent, direction)`, with `direction`
`adsk.fusion.ExtentDirections.PositiveExtentDirection` when `sketch.modelToSketchSpace(C + d)` has
a positive `z` (`d` scaled to 1 cm), else `adsk.fusion.ExtentDirections.NegativeExtentDirection`;
`extrudeInput.participantBodies = [self.cageBody]`; then
`extrudeFeature = designComponent.features.extrudeFeatures.add(extrudeInput)`. The cut runs one way
only.

After it, raise naming the window and the count unless `extrudeFeature.bodies.count` is 1, take
`extrudeFeature.bodies.item(0)` as `self.cageBody`, and raise unless the same probe now reads
`PointOutsidePointContainment`; a cut extruded the wrong way leaves it inside. At the defaults the
finished sleeve is one piece of about 16,590 mm³.

The proof is `stepWindowExtrudeCut` with `assertWindowExtrudeCut`, run with `windowCutCases`. It
cuts both windows' prisms from the uncut tube in one cut, each prism started in the hollow rather
than on the axis plane, where the two hexagons cross; the material removed is the same. The tube
after it is one solid, its volume the tube's less each window's height times the wall's chord
integrated across the window. decad will not cut again from the bored cage, and the bores and
windows keeping `collarWall` apart is the hand-written `TestSleeveWindowsKeepTheirWalls`.

<!-- proof-run: proofkit3d.RunSolid(windowCutCases, stepWindowExtrudeCut, assertWindowExtrudeCut) -->

## S24 `[PROSE]` Relocate the bodies and hide the construction geometry

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
      "reason": "the framework helper in lib/geargen/solids.py [PB-TREE-CLEANUP]",
      "receiver": "solids",
      "role": "inherited",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    }
  ],
  "citations": [
    {
      "first": 1334,
      "last": 1340,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 452,
      "last": 454,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1334–1340; `spec/screwgear/instructions.md` L452–454.

`relocateBodies`: name the cage body `Cage`, then move the three bodies into their components
with `body.moveToComponent(occurrence)`: `self.gearBodies[0]` to `self.gearOccs[0]`,
`self.gearBodies[1]` to `self.gearOccs[1]`, `self.cageBody` to `self.cageOcc`. It preserves world
position and needs no activation [PB-NO-CROSS-SIBLING]. Then call
`solids.hide_construction_geometry(self.designOcc.component)` and do not re-implement it
[PB-TREE-CLEANUP]: the `Design` component holds every sketch and construction plane of the build.
The command wrapper settles sketch display after `generate` returns; the module adds nothing for
it [PB-SETTLE-DISPLAY].

Not built by the proof: moving a body between components changes nothing either engine models.
