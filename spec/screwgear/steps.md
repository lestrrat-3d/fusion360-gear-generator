The proof of this step list is `proof/screwgear/compiled_model_test.go`, `proof/screwgear/compiled_sketches_test.go`, `proof/screwgear/compiled_solids_test.go` and the generated registration file `proof/screwgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `f23b4c50c2a41fa0673137ac893f41f1deaf5483` |
| `spec/screwgear/fusion.md` | `b3511aaf4ac2c1eb0207c26828c6ef38766f83b8` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `e93381b533b9ba6bf4df27f55affe71623df52a8` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/screwgear/mesh-search.md` | `32d393485d2edbfd173dfaa98a7cb76e4fa8471f` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `b7e7811c13fe64aef683d7d6f5d78317b5bb643c` |

## 1 `[PROSE]` Module constants and the dialog

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "prefixBase",
      "owner": null,
      "reason": "prefixBase is the base.Generator hook getOccurrence uses to build the parameter prefix",
      "receiver": null,
      "role": "inherited",
      "span": "prefixBase()"
    },
    {
      "condition": null,
      "name": "deleteComponent",
      "owner": null,
      "reason": "deleteComponent is inherited from base.Generator",
      "receiver": null,
      "role": "inherited",
      "span": "deleteComponent()"
    },
    {
      "condition": null,
      "name": "configure",
      "owner": null,
      "reason": "configure is the classmethod the module defines; GearCommand in commands/_gear_command.py calls it, the module never does, and base.py declares no such hook",
      "receiver": "ScrewGearCommandInputsConfigurator",
      "role": "example",
      "span": "ScrewGearCommandInputsConfigurator.configure(cls, command)"
    },
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
      "reason": "get_design is the framework helper in lib/geargen/misc.py",
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
      "span": "group = command.commandInputs.addGroupCommandInput(groupId, groupLabel)"
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
      "first": 390,
      "last": 420,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 441,
      "last": 508,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L390–420; `spec/screwgear/instructions.md` L441–508.

The module is `lib/geargen/screwgear.py`. It defines exactly two public classes,
`ScrewGearCommandInputsConfigurator` and `ScrewGearGenerator(base.Generator)`, exported through
`lib/geargen/__init__.py`, and imports only the framework (`base`, `misc`, `utilities`, `solids`,
`fusion360utils`). The generator takes the inherited one-argument constructor `(design)`, overrides
`prefixBase()` to return `'ScrewGear'`, and relies on the inherited `deleteComponent()` for error
cleanup. There is no generation context: handles live on `self` (step 5 and later name them).
The parameter mode is all-Python-precomputed (`[PB-PRECOMPUTED-MODE]`): every value is computed in
Python in internal cm and written numerically, and the generator registers no user parameter.

Module-level constants, one per input id and group id, plus the cell size:

```
INPUT_ID_PLANE = 'plane'                    INPUT_ID_POINT = 'point'
INPUT_ID_PARENT = 'parent'
INPUT_ID_RIBBON_WIDTH = 'ribbonWidth'       INPUT_ID_TOOTH_COUNT = 'toothCount'
INPUT_ID_TWIST_LEAD = 'twistLead'           INPUT_ID_RIBBON_THICKNESS = 'ribbonThickness'
INPUT_ID_TOOTH_PITCH = 'toothPitch'         INPUT_ID_TOOTH_HEIGHT = 'toothHeight'
INPUT_ID_CAGE_RADIUS = 'cageRadius'         INPUT_ID_CAGE_RISE = 'cageRise'
INPUT_ID_CLEARANCE = 'clearance'            INPUT_ID_COLLAR_HALF = 'collarHalf'
INPUT_ID_COLLAR_WALL = 'collarWall'
INPUT_ID_CROSS_ANGLE = 'crossAngle'         INPUT_ID_ENGAGEMENT = 'engagement'
INPUT_ID_MOUNT_ANGLE_A = 'mountAngleA'      INPUT_ID_MOUNT_ANGLE_B = 'mountAngleB'
INPUT_ID_ASSEMBLY_PHASE = 'assemblyPhase'
GROUP_ID_RIBBON = 'ribbonGroup'             GROUP_ID_FRAME = 'frameGroup'
GROUP_ID_MESH = 'meshGroup'
CELL_TEETH = 4
```

`ScrewGearCommandInputsConfigurator.configure(cls, command)` is a classmethod that adds the inputs
below in exactly this order, with no conditional visibility. The three selection inputs come first
and at the top level, the plane first, because Fusion focuses the first selection input
(`[PB-AUTOFOCUS-FIRST]`). Each is `command.commandInputs.addSelectionInput(id, label, tooltip)`,
its filters added with `addSelectionFilter` as the named constants
`adsk.core.SelectionCommandInput.<Name>` (`[PB-SELECTION-FILTER-ENUM]`), and `setSelectionLimits(1, 1)`
(`[PB-SELECTION-DECL]`):

| Order | input id | Label | Filters | Tooltip |
|---|---|---|---|---|
| 1 | `plane` | Target Plane | `ConstructionPlanes`, `PlanarFaces` | `Plane the cage's axis is normal to` |
| 2 | `point` | Centre Point | `ConstructionPoints`, `SketchPoints` | `Centre of the mechanism` |
| 3 | `parent` | Parent Component | `Occurrences`, `RootComponents` | `Component the mechanism is created under` |

The Parent Component input pre-selects the design's root component:
`parentInput.addSelection(get_design().rootComponent)`.

Then three groups, each `group = command.commandInputs.addGroupCommandInput(groupId, groupLabel)`,
whose value inputs are added to `group.children` rather than to the top-level inputs. The ribbon
and frame groups start expanded (`group.isExpanded = True`) and the mesh group starts collapsed
(`group.isExpanded = False`). Every value input is
`group.children.addValueInput(id, label, unit, adsk.core.ValueInput.createByReal(default))` with
the default in internal units (`[PB-DIALOG-DEFAULT-UNITS]`): mm/10 for a length, radians for an
angle, the bare number for the count. Rows in this order:

| Group id, label | input id | Label | unit string | default (display) | `createByReal` value |
|---|---|---|---|---|---|
| `ribbonGroup`, `Ribbon` | `ribbonWidth` | Ribbon Width | `mm` | 15 mm | 1.5 |
| `ribbonGroup`, `Ribbon` | `toothCount` | Tooth Count | `''` | 68 | 68 |
| `ribbonGroup`, `Ribbon` | `twistLead` | Twist Lead | `mm` | 49.5 mm | 4.95 |
| `ribbonGroup`, `Ribbon` | `ribbonThickness` | Ribbon Thickness | `mm` | 3.75 mm | 0.375 |
| `ribbonGroup`, `Ribbon` | `toothPitch` | Tooth Pitch | `mm` | 2.625 mm | 0.2625 |
| `ribbonGroup`, `Ribbon` | `toothHeight` | Tooth Height | `mm` | 2.625 mm | 0.2625 |
| `frameGroup`, `Frame` | `cageRadius` | Cage Radius | `mm` | 15 mm | 1.5 |
| `frameGroup`, `Frame` | `cageRise` | Cage Rise | `mm` | 18.75 mm | 1.875 |
| `frameGroup`, `Frame` | `clearance` | Clearance | `mm` | 0.45 mm | 0.045 |
| `frameGroup`, `Frame` | `collarHalf` | Collar Half Length | `mm` | 3 mm | 0.3 |
| `frameGroup`, `Frame` | `collarWall` | Collar Wall | `mm` | 3 mm | 0.3 |
| `meshGroup`, `Mesh (from the mesh search)` | `crossAngle` | Crossing Angle | `deg` | 80° | radians(80) |
| `meshGroup`, `Mesh (from the mesh search)` | `engagement` | Engagement | `mm` | 0.75 mm | 0.075 |
| `meshGroup`, `Mesh (from the mesh search)` | `mountAngleA` | Mounting Angle A | `deg` | 15° | radians(15) |
| `meshGroup`, `Mesh (from the mesh search)` | `mountAngleB` | Mounting Angle B | `deg` | 15° | radians(15) |
| `meshGroup`, `Mesh (from the mesh search)` | `assemblyPhase` | Assembly Phase | `mm` | −1.31 mm | −0.131 |

The entry module `commands/screwgear/entry.py` constructs `GearCommand(gear_type='ScrewGear',
name='Screw Gear Generator', …)` binding the configurator and generator classes by name, per the
playbook's "Command-entry wiring".

## 2 `[PROSE]` Read the inputs and run the range checks

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "generate",
      "owner": null,
      "reason": "generate is the abstract method base.Generator declares and GearCommand calls",
      "receiver": null,
      "role": "inherited",
      "span": "generate(self, inputs)"
    },
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
      "name": "hide_construction_geometry",
      "owner": null,
      "reason": "solids.hide_construction_geometry is the shared cleanup helper in lib/geargen/solids.py",
      "receiver": "solids",
      "role": "inherited",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    },
    {
      "condition": null,
      "name": "get_selection",
      "owner": null,
      "reason": "get_selection is the framework helper in lib/geargen/base.py",
      "receiver": null,
      "role": "inherited",
      "span": "get_selection(inputs, id)"
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
      "name": "evaluateExpression",
      "owner": "adsk.core.UnitsManager",
      "reason": null,
      "receiver": "design.unitsManager",
      "role": "required",
      "span": "design.unitsManager.evaluateExpression(input.expression, units)"
    }
  ],
  "citations": [
    {
      "first": 486,
      "last": 573,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 922,
      "last": 930,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L486–573; `spec/screwgear/instructions.md` L922–930.

`generate(self, inputs)` runs, in order: `processInputs(inputs)`, `buildComponentTree`,
`buildAnchor`, `buildGear` for index 0 then for index 1 (each of which runs `buildSweepPaths`,
`buildToothCell` and `repeatCellByDoubling` for its index), `buildCage`, `relocateBodies`,
and last `solids.hide_construction_geometry(self.designOcc.component)` (step 28). Gear index 0 is
`Gear A` and 1 is `Gear B`; `gearLabel` below is that name.

`processInputs` reads every selection before anything creates an occurrence
(`[PB-SELECTION-STASH]`): `get_selection(inputs, id)` for `parent` (an `Occurrence` gives its
`.component`, a `Component` is used as is, anything else raises) into `self.parentComponent`,
then `plane` into `self.targetPlane` and `point` into `self.centrePoint`; each must hold exactly
one entity or it raises naming the id. Every input, grouped or not, is found with
`inputs.itemById(id)` on the command's top-level inputs; input ids are unique across the command,
which is what lets that lookup reach into a group. It raises naming the id when a lookup returns
`None`. Value inputs are read with `design.unitsManager.evaluateExpression(input.expression, units)`
(`[PB-EVAL-EXPRESSION]`) with units `'mm'` for a length, `'deg'` for an angle and `''` for
`toothCount`; the result is internal units, cm for a length and radians for an angle.

Every computation in this step list is written in millimetres and radians: multiply each length
read by 10 to get mm, and divide every length by 10 when it is handed to the Fusion API. Names
used from here on:

```
W = ribbonWidth   T = ribbonThickness   P = toothPitch   H = toothHeight   N = toothCount
TwistLead = twistLead      Sigma = crossAngle      Eng = engagement
PhiA = mountAngleA         PhiB = mountAngleB      Phase = assemblyPhase
cageRadius, cageRise, clearance, collarHalf, collarWall as read
Lambda = TwistLead / (2*pi)          mm of advance per radian
A      = W - Eng                     distance between the two axes
L      = N*P                         ribbon length
Ri = cageRadius - collarHalf         Ro = cageRadius + collarHalf
hw = W/2 + clearance                 ht = T/2 + clearance
c  = hypot(hw, ht)                   the bore's corner radius
axialWindow = 1.5 * sqrt(W^2 - A^2) / sin(Sigma)
```

At the defaults: `Lambda` 7.87817 mm/rad, `A` 14.25 mm, `L` 178.5 mm, `Ri` 12, `Ro` 18,
`hw` 7.95, `ht` 2.325, `c` 8.283 mm, `axialWindow` 7.13 mm.

The range checks run in this order and each raises a clear error naming the offending field and
the bound (`[PB-SELF-DIAGNOSING]`):

1. `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `collarHalf`, `collarWall` and
   `clearance` each must be `> 0`.
2. `toothCount` must be a whole number `>= 4`.
3. `toothHeight` must be `> 0` and `< ribbonWidth/2`.
4. `engagement` must be `> 0` and `<= toothHeight`.
5. `crossAngle` must lie strictly between 0° and 180°. `twistLead` has no upper bound, and no
   range is enforced on either mounting angle.
6. `assemblyPhase` must lie strictly within `±toothPitch`.
7. Naming `cageRadius`: `cageRadius + collarHalf + 1 mm < toothCount*toothPitch/2`.

Then the sleeve's four checks, in this order, each naming the field given:

8. Naming `cageRadius` (the channel starts in the hollow): `c < Ri`.
9. Naming `cageRadius` (the mesh stays visible along the axis):
   `hypot(axialWindow, hypot(W/2, T/2)) + clearance <= Ri` (10.97 mm against 12 mm at the
   defaults).
10. Naming `cageRise` (the end faces keep `collarWall`): `cageRise >= A/2 + c + collarWall`
    (18.41 mm at the defaults).
11. Naming `collarWall` (the wall between two neighbouring bores): the separation of step 3 must
    be at least `collarWall`; the message names the two bores and the separation.

Then the derived build numbers:

```
sIn  = sqrt(Ri^2 - c^2) - 1 mm       where a +R bore's cut starts, 7.683 mm
sOut = Ro + 1 mm                     where it ends, 19 mm
cell = min(CELL_TEETH, N)            teeth in the lofted cell, 4
n    = max(ceil((P/Lambda) / radians(2)), 8)     sections per tooth, 10
q    = N // cell,  r = N % cell      17 whole cells and no remainder
```

The two section-count bounds are the twist between neighbouring sections under 2° and a floor of
eight steps to the tooth; at a 400 mm lead the twist alone asks for two and the floor gives eight.
After the four checks, the window search of step 4 runs; it refuses nothing.

## 3 `[PROSE]` The wall between the bores (processInputs)

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 1061,
      "last": 1089,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 736,
      "last": 750,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1061–1089; `spec/screwgear/instructions.md` L736–750.

This runs in `processInputs` in millimetres, before any feature, and makes no timeline entry. It
needs the frame of §1 in a fixed abstract form, the same one step 5 later builds in world space:
`C` at the origin, `ê` along X, `k̂ = n̂ × ê` along Y, `n̂` along Z. In that form:

```
dirA = ( cos(Sigma/2),  sin(Sigma/2), 0)      originA = (0, 0, -A/2)    uA = +n̂    vA = dirA × uA
dirB = ( cos(Sigma/2), -sin(Sigma/2), 0)      originB = (0, 0, +A/2)    uB = -n̂    vB = dirB × uB
theta_g(s) = s/Lambda + Phi_g
point(g, s, u, v) = origin_g + s*dir_g + (u*cos theta - v*sin theta)*u_g + (u*sin theta + v*cos theta)*v_g
```

Any rigid placement of this frame gives the same numbers, since the check measures only distances
between bores. A gear's `+R` bore sits at station `+cageRadius` of its axis and its `-R` bore at
`-cageRadius`. Round the tube the bores stand at azimuths gear A `+R` `Sigma/2`, gear B `-R`
`180° - Sigma/2`, gear A `-R` `180° + Sigma/2`, gear B `+R` `-Sigma/2` (40°, 140°, 220°, 320° at the
defaults), so the four gaps between neighbours are:

| gap facing `d` | its two bores |
|---|---|
| `+k̂` | gear A `+R`, gear B `-R` |
| `-ê` | gear A `-R`, gear B `-R` |
| `-k̂` | gear A `-R`, gear B `+R` |
| `+ê` | gear A `+R`, gear B `+R` |

1. Sample each bore's outline (`channelOutline`). With `sigma = +1` for a `+R` bore and `-1` for a
   `-R` bore, take stations `s = sigma*(sIn + 0.1 mm*k)` for `k = 0, 1, …` while `|s| <= sOut`. At
   each station take seventeen points on each side of the rectangle, for `i = 0 … 16`:
   `(hw, v)` and `(-hw, v)` with `v = -ht + 2*ht*i/16`, and `(u, ht)` and `(u, -ht)` with
   `u = -hw + 2*hw*i/16`. Place each as `point(g, s, u, v)` and keep it when its distance from the
   frame's axis, `hypot(P.x, P.y)`, lies within `Ri - 0.5 mm` to `Ro + 0.5 mm`.
2. For each gap, facing `d`: `across = n̂ × d`. Project each kept point of the gap's two bores to
   `(P·across, P·n̂)`, and take the convex hull of each bore's projections by Andrew's monotone
   chain.
3. For every edge of either hull, with `m` the unit vector square to the edge, the separation
   along `m` is the larger of `min(m·q) - max(m·p)` and `min(m·p) - max(m·q)`, with `p` over the
   first hull's corners and `q` over the second's. The gap's separation is the largest of these
   over all the edges of both hulls; it is negative when the hulls overlap.
4. The least of the four gaps' separations must be at least `collarWall` (range check 11 of step
   2). At the defaults it is 4.640 mm, across the `-ê` gap.

## 4 `[PROSE]` The window search (processInputs)

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "log",
      "owner": null,
      "reason": "futil.log is the framework logging helper in lib/fusion360utils",
      "receiver": "futil",
      "role": "inherited",
      "span": "futil.log(f'No window facing {d}: {reason}')"
    }
  ],
  "citations": [
    {
      "first": 1091,
      "last": 1223,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1091–1223.

This runs in `processInputs` after the four checks, in millimetres, in the abstract frame of step
3, and makes no timeline entry. It finds zero to two windows and stores them on `self.windows`,
each with its facing direction (one of `+k`, `-k`, `+e`, `-e`) and its corners; the build later
places them in world space.

**Facings.** When `crossAngle <= 90°` the windows face `d = +k̂` and then `-k̂`; past 90° they face
`+ê` and then `-ê`. Each window is found the same way from its own `d`. For a window facing the
level unit direction `d`: `across = n̂ × d` (which is `-ê` for `d = +k̂`). A point `P` has plane
coordinates `t = P·across` and `z = P·n̂`, and depth `a = P·d`. At `t` the wall runs from
`a0(t) = sqrt(max(0, Ri^2 - t^2))` to `a1(t) = sqrt(max(0, Ro^2 - t^2))`.

**A bore's section in the wall** (`sectionInWall`), for a bore of gear `g` at station `s` with
`|s| < Ro`, in coordinates `x` along `u_g` and `y` along `v_g`. Take the rectangle's corners
`(-hw, -ht)`, `(hw, -ht)`, `(hw, ht)`, `(-hw, ht)` in that order (counter-clockwise), each turned by
`theta = s/Lambda + Phi_g` to `(u*cos theta - v*sin theta, u*sin theta + v*cos theta)`. Clip it to
the heights inside the end faces: keep the `x` for which `(origin_g + x*u_g)·n̂` lies within
`±cageRise`. Then clip what is left twice more, once to `near <= y <= far` and once to
`-far <= y <= -near`, with `near = sqrt(max(0, Ri^2 - s^2))` and `far = sqrt(Ro^2 - s^2)`: those are
the section's two pieces in the wall, either of which may be empty. Each clip keeps the part of a
convex polygon on one side of a line, walking its edges in order, keeping each corner on the kept
side and adding the point where an edge crosses the line. At `|s| >= Ro` the section has no piece.

**Flanking and far bores, lean.** A bore's crossing is `origin_g + sc*dir_g` with `sc = +cageRadius`
for `+R` and `-cageRadius` for `-R`. The two bores whose crossings have `crossing·d > 0` flank the
window; the other two are its far bores. The low flanking bore is the one whose crossing has the
smaller `crossing·n̂`; `lean = +1` when the high one's crossing has the larger `t`, else `-1`.

**Long sides** (`wallCorners`). Walk each flanking bore's cut span (`[sIn, sOut]` for `+R`,
`[-sOut, -sIn]` for `-R`) at stations every 0.001 mm from its lower end, and take every corner of
every piece as the point `origin_g + x*u_g + y*v_g + s*dir_g` with its `m = z + lean*t`.
`lowReach` is the low bore's largest `m` and `highReach` the high bore's least. Then
`lo = lowReach + sqrt(2)*collarWall` and `hi = highReach - sqrt(2)*collarWall`.

**Trims** (`channelTop`). `zLimit` is, over all four bores, at stations from `sigma*sIn` outward
every 0.01 mm while `|s| <= sOut`, wherever `|s| <= Ro` and
`hypot(s, hw*|sin theta| + ht*|cos theta|) >= Ri`, the largest
`A/2 + hw*|cos theta| + ht*|sin theta|` (15.01 mm at the defaults). Then:

```
top    = min(2*zLimit - hi, hi + sqrt(2)*Ri)
bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)
```

**The distance to a channel** (`wallGap`), for a point `P`, one bore of gear `g` and a `reach`:
`x = (P - origin_g)·u_g`, `y = (P - origin_g)·v_g`, `sq = (P - origin_g)·dir_g`. If
`hypot(hypot(x, y), sq - min(max(sq, spanStart), spanEnd)) - c >= reach`, the distance is `reach`
and nothing is walked. Otherwise walk the bore's station table, built once per bore before its
first walk: stations `spanStart + k*0.002 mm` for `k = 0, 1, …` while inside the span, each with its
two pieces, plus, wherever the set of non-empty pieces differs between two neighbouring stations,
stations every 0.0001 mm strictly between those two, all in order of `s`. For each piece keep its
circle: centre the average of its corners, radius the largest distance from that centre to a
corner. Start with `best = reach^2`; walk up from the first station at or above `sq`, then down from
the one before it, and stop each way at the first station with `(sq - s)^2 >= best`. At each
station, for each non-empty piece: with `o = hypot(x - cx, y - cy) - radius`, pass over the piece
when `o > 0` and `(sq - s)^2 + o^2 >= best`; otherwise set `best = min(best, (sq - s)^2 + d2)`,
where `d2` is 0 when `(x, y)` is inside the piece (on the inner side of every edge of the
counter-clockwise polygon; a piece of fewer than three corners never contains it) and otherwise
the least squared distance from `(x, y)` to the piece's edges. The distance is `sqrt(best)`.

**The ends** (`windowEnd`). For `delta = +1` (right) and `delta = -1` (left), bisect `te` 24 times
between 0 and `min(Ri, Ro/sqrt(2))*(1 - 1e-9)`: take the middle, and make it the new lower end when
`clear(te)` holds and the new upper end when it does not. The end is the final lower end:
`right = end(+1)` and `left = -end(-1)`. `clear(te)`, at `t = delta*te`:

1. `zLow = max(lo - lean*t, bottom + lean*t)` and `zHigh = min(hi - lean*t, top + lean*t)`; not
   clear when `zLow > zHigh`.
2. `need = collarWall + 0.1 mm/sqrt(2) + 0.005 mm` (3.076 mm at the defaults).
3. The points, each `t*across + z*n̂ + a*d`, are: the end's two corner lines through the wall, at
   `z = zLow` and at `z = zHigh`, each at `a = a0, a0 + 0.1 mm, …` and last `a1`, a step that would
   pass `a1` landing on it; and, on the inner face `a = a0(t)` and on the outer face `a = a1(t)`,
   the end's upright edge, at `z = zLow + (zHigh - zLow)*k/m` for `k = 1 … m - 1` with
   `m = ceil((zHigh - zLow)/0.1 mm)`, and at `z = -zLimit + j*0.1 mm` for every `j >= 0` with
   `z <= zLimit` and `zLow < z < zHigh`.
4. Not clear as soon as any point's `wallGap` to either far bore, with `reach = need`, is under
   `need`; clear otherwise.

**The hexagon** (`clipCorners`). Start from the square with corners, in this order,
`(-2*Ro, -2*Ro)`, `(2*Ro, -2*Ro)`, `(2*Ro, 2*Ro)`, `(-2*Ro, 2*Ro)` in `(t, z)`, and clip it, in
this order, to `lean*t + z <= hi`, `-lean*t - z <= -lo`, `-lean*t + z <= top`,
`lean*t - z <= -bottom`, `t <= right` and `-t <= -left`, by the clip of `sectionInWall`. Drop a
corner within 0.001 mm of the one before it (the last also against the first). At the defaults the
`+k̂` window has `lo = -3.335`, `hi = 6.495`, `bottom = -20.306`, `top = 23.466`, `left = -11.404`,
`right = 11.663` and `lean = +1`, and its six corners in `(t, z)`, in order, are `(11.663, -5.168)`,
`(-8.4855, 14.9805)`, `(-11.404, 12.062)`, `(-11.404, 8.069)`, `(8.4855, -11.8205)`,
`(11.663, -8.643)`: 208.1 mm² on the plane. The `-k̂` window comes out as the `+k̂` one turned half a
turn about `ê`, `(t, z)` to `(-t, -z)`; that is a property of the result, not a shortcut the build
takes.

**When a gap has no room** (`room`). A window is not cut when `hi <= lo`, or when `right <= left`
or the hexagon has fewer than three distinct corners or no area. The build then logs
`futil.log(f'No window facing {d}: {reason}')` (`[PB-LOGGING]`), `{d}` being `+k`, `-k`, `+e` or
`-e`, and leaves that window out.

## 5 `[PROSE]` The component tree

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "getOccurrence",
      "owner": null,
      "reason": "getOccurrence is inherited from base.Generator",
      "receiver": "self",
      "role": "inherited",
      "span": "self.getOccurrence()"
    },
    {
      "condition": null,
      "name": "deleteComponent",
      "owner": null,
      "reason": "deleteComponent is inherited from base.Generator",
      "receiver": null,
      "role": "inherited",
      "span": "deleteComponent()"
    },
    {
      "condition": null,
      "name": "addNewComponent",
      "owner": "adsk.fusion.Occurrences",
      "reason": null,
      "receiver": "occurrences",
      "role": "required",
      "span": "occurrences.addNewComponent(adsk.core.Matrix3D.create())"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Matrix3D",
      "reason": null,
      "receiver": "adsk.core.Matrix3D",
      "role": "required",
      "span": "occurrences.addNewComponent(adsk.core.Matrix3D.create())"
    },
    {
      "condition": null,
      "name": "activate",
      "owner": "adsk.fusion.Occurrence",
      "reason": "activating a sub-occurrence collapses the selected plane onto world XY (PB-NEVER-ACTIVATE)",
      "receiver": "occurrence",
      "role": "forbidden",
      "span": "occurrence.activate()"
    }
  ],
  "citations": [
    {
      "first": 422,
      "last": 439,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 401,
      "last": 409,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L422–439; `spec/screwgear/instructions.md` L401–409.

`buildComponentTree` makes five component creations in the timeline (`[PB-OCCURRENCE-TREE]`).
The top occurrence is the inherited `self.getOccurrence()`, which creates it under
`self.parentComponent` and is what the inherited `deleteComponent()` deletes on failure; name its
component `Screw Gearing` and never call `addNewComponent` for it yourself. Under that component
make four children, each `occurrences.addNewComponent(adsk.core.Matrix3D.create())` on the
`Screw Gearing` component's occurrences, named, in this order, `Design`, `Gear A`, `Gear B` and
`Cage`. Keep `self.designOcc` (the `Design` occurrence), `self.gearOccs` (a list of the `Gear A`
and `Gear B` occurrences) and `self.cageOcc`. Every sketch, construction plane and feature from
here on is made in `self.designOcc.component`; the three other children stay empty until step 26
(`[PB-NO-CROSS-SIBLING]`). Never call `occurrence.activate()` (`[PB-NEVER-ACTIVATE]`), and place
sketches directly on the user-selected plane, never on a coplanar copy of it
(`[PB-USE-SELECTED-PLANE]`).

## 6 `[GO]` The Anchor sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.Sketches",
      "reason": null,
      "receiver": "sketches",
      "role": "required",
      "span": "sketches.add(self.targetPlane)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "projectedPoint = sketch.project(point).item(0)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(x, y, 0)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addCoincident(projectedPoint, line)"
    },
    {
      "condition": null,
      "name": "addMidPoint",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addMidPoint(projectedPoint, line)"
    },
    {
      "condition": null,
      "name": "addHorizontal",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addHorizontal(line)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addDistanceDimension(line.startSketchPoint, line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)"
    }
  ],
  "citations": [
    {
      "first": 704,
      "last": 734,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 366,
      "last": 384,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L704–734; `spec/screwgear/fusion.md` L366–384.

`buildAnchor` first creates the sketch `Anchor` with `sketches.add(self.targetPlane)` on the
`Design` component and sets its `name`. It does not defer computing (`[SCREW-F-DEFER]`). It projects
the selected point, the one projection in the build (`[SCREW-F-REFERENCES]`,
`[PB-PROJECT-NOT-FIXED]`): `projectedPoint = sketch.project(point).item(0)`, with `point` the
stashed `self.centrePoint`.

The Anchor Line is `sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)` from two raw seeds
`adsk.core.Point3D.create(x, y, 0)` 0.5 cm either side of the projected point along the sketch's own
x axis: `x = projected.x - 0.5` for the start and `projected.x + 0.5` for the end, `y = projected.y`,
both `z = 0` (`[PB-SKETCH-ZERO-Z]`, `[PB-SEED-NEAR]`). So it is 10 mm long with its end to the right
of its start. Constrain it with these four things and nothing else:

- `geometricConstraints.addCoincident(projectedPoint, line)` and
  `geometricConstraints.addMidPoint(projectedPoint, line)` — both, not the midpoint alone;
- `geometricConstraints.addHorizontal(line)`, sketch-local, so it survives a tilted plane
  (`[PB-REFLINE-DIRECTION]`);
- `sketchDimensions.addDistanceDimension(line.startSketchPoint, line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
  with `textPoint` 0.3 cm below the projected point (`z = 0`), and its `parameter.value` set to
  1.0 cm. It is a horizontal distance from start to end, not an aligned one: only the seeded
  orientation satisfies it.

Raise naming `Anchor` unless `sketch.isFullyConstrained` (`[PB-FULL-CONSTRAINT]`). Only then read the
frame from world geometry (`[PB-WORLDGEO-CONSTRAINED]`, `[PB-WORLD-FRAME]`): `C` is
`projectedPoint.worldGeometry`, and `ê` is the unit vector from `line.startSketchPoint.worldGeometry`
to `line.endSketchPoint.worldGeometry`. Keep the line as `self.anchorLine`; it is passed once more,
to the Window Plane (step 23). No later sketch projects anything.

Proof function: `stepAnchorSketch`. The engine's midpoint already carries the coincident row, so
the proof writes the midpoint alone and a signed horizontal distance of +10 mm.

<!-- proof-run: proofkit.Run(cpAnchorCases, stepAnchorSketch) -->

## 7 `[PROSE]` The Gear A Axis Plane, and `n̂`

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
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-A/2/10))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-A/2/10))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "constructionPlanes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "rotate",
      "owner": null,
      "reason": "a math notation for turning a vector about an axis, not a call",
      "receiver": null,
      "role": "prose",
      "span": "rotate(ê, a, about n̂)"
    }
  ],
  "citations": [
    {
      "first": 736,
      "last": 758,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 429,
      "last": 438,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L736–758; `spec/screwgear/fusion.md` L429–438.

On the `Design` component: `planeInput = constructionPlanes.createInput()`, then
`planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(-A/2/10))`, then
`constructionPlanes.add(planeInput)`; name it `Gear A Axis Plane` (`[PB-CONSTRUCTION-PLANES]`,
`[PB-USE-SELECTED-PLANE]`). The offset is along the selected entity's own normal, whose sign the
build does not assume (`[SCREW-F-NORMAL-SIGN]`): read `plane.geometry`, a `Plane` with `origin` and
`normal`; set `n̂ = normal` when `(C - origin)·normal > 0`, else `-normal`, so that `C` lies `+A/2`
along `n̂` from gear A's axis plane.

With `C`, `ê` and `n̂` the build computes, in world cm, the frame of §1 (this is step 3's abstract
frame placed in the world):

```
k̂       = n̂ × ê
dirA    = rotate(ê, +Sigma/2, about n̂)     originA = C - (A/2)*n̂     uA = +n̂    vA = dirA × uA
dirB    = rotate(ê, -Sigma/2, about n̂)     originB = C + (A/2)*n̂     uB = -n̂    vB = dirB × uB
theta_g(s) = s/Lambda + Phi_g
point(g, s, u, v) = origin_g + s*dir_g + (u*cos theta - v*sin theta)*u_g + (u*sin theta + v*cos theta)*v_g
```

`rotate(ê, a, about n̂)` is `cos(a)*ê + sin(a)*(n̂ × ê)`. `u_g` points at the other gear: that is
what makes `Phi` mean the same for both gears, which are the same part, same hand. Every world
point a later step names is `point(...)` or a combination of `C`, `ê`, `k̂`, `n̂` stated there.

## 8 `[PROSE]` The Gear B Axis Plane

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
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(A/2/10))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(A/2/10))"
    }
  ],
  "citations": [
    {
      "first": 752,
      "last": 758,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 429,
      "last": 438,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L752–758; `spec/screwgear/fusion.md` L429–438.

The same recipe as step 7 with `planeInput.setByOffset(self.targetPlane, adsk.core.ValueInput.createByReal(A/2/10))`,
named `Gear B Axis Plane`. Then check that `|(C - origin)·normal|` is `A/2` (to 1e-6 cm) for both
axis planes, reading each plane's `geometry`, and raise naming the plane and both numbers otherwise
(`[PB-SELF-DIAGNOSING]`). No later plane is offset from the selected plane. The two planes are kept
as the axis plane of gear 0 and gear 1.

## 9 `[GO]` A gear's Paths sketch

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "local = sketch.modelToSketchSpace(point)"
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
      "receiver": "sketchLines",
      "role": "required",
      "span": "sketchLines.addByTwoPoints(pt[-sOut], pt[-sIn])"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketchLines",
      "role": "required",
      "span": "sketchLines.addByTwoPoints(pt[sIn], pt[sOut])"
    }
  ],
  "citations": [
    {
      "first": 760,
      "last": 779,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 581,
      "last": 599,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L760–779; `spec/screwgear/instructions.md` L581–599.

`buildSweepPaths`, once per gear, gear A first. A sketch named `{gearLabel} Paths` on that
gear's axis plane; computing is not deferred. It holds four reference points and two lines and
nothing else (`[SCREW-F-REFERENCES]`, `[PB-PROJECT-NOT-FIXED]`):

- For each station `s` of `-sOut`, `-sIn`, `sIn`, `sOut` (−19, −7.683, 7.683, 19 mm at the
  defaults): `local = sketch.modelToSketchSpace(point)` with the world point
  `origin_g + s*dir_g` (in cm), then `local.z = 0` (`[PB-SKETCH-ZERO-Z]`), then
  `sketch.sketchPoints.add(local)`.
- Two solid lines sharing those points (`[PB-SHARE-XOR-COINCIDENT]`): `bore-` is
  `sketchLines.addByTwoPoints(pt[-sOut], pt[-sIn])` and `bore+` is
  `sketchLines.addByTwoPoints(pt[sIn], pt[sOut])`, each drawn from its negative station to its
  positive one, so its start is its negative end and it runs along `+dir_g`.
- After both lines exist, set all four points `isFixed = True`.

No dimension and no constraint. Raise naming the sketch unless it reports `isFullyConstrained`
(`[PB-FULL-CONSTRAINT]`). Store the lines in `self.pathLines[index]`, a dict keyed `'bore-'` and
`'bore+'` (`self.pathLines` is a list of two such dicts). Overlapping collinear lines on one axis
read fully constrained in Fusion and a path made from one of them with chaining off holds that line
alone (`[SCREW-F-DIAGNOSTIC]`, `[PB-PATH-FROM-SKETCH]`).

Proof function: `stepPathsSketch`.

<!-- proof-run: proofkit.Run(cpPathsCases, stepPathsSketch) -->

## 10 `[GO]` A gear's Cell Sections sketch

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
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(world))"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.sketchPoints.add(sketch.modelToSketchSpace(world))"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketchLines",
      "role": "required",
      "span": "sketchLines.addByTwoPoints(corner, nextCorner)"
    }
  ],
  "citations": [
    {
      "first": 781,
      "last": 853,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 161,
      "last": 180,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 292,
      "last": 304,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L781–853; `spec/screwgear/fusion.md` L161–180; `spec/screwgear/fusion.md` L292–304.

`buildToothCell`, once per gear. One cell is `cell` teeth of the finished ribbon at its
negative end. With `Z0` the gear's tooth phase, 0 for gear A and `Phase` for gear B (gear B is gear
A advanced by `Phase` under its own screw motion, ends and teeth alike), the ribbon starts at
`s0 = Z0 - L/2` (−89.25 mm for gear A and −90.56 mm for gear B at the defaults). Section `k`, for
`k = 0 … cell*n` (41 sections at the defaults), stands at station `s_k = s0 + k*P/n` and is the
rectangle spanning `u` from `uB` to `uF` and `v` from `-hv` to `hv`:

```
uB = -W/2       uF = Utooth(s_k) = W/2 - H/2 + (H/2)*cos(2*pi*(s_k - Z0)/P)       hv = T/2
theta_k = s_k/Lambda + Phi_g
```

Create the sketch `{gearLabel} Cell Sections` on the gear's axis plane and set its name, then
`sketch.isComputeDeferred = True` before the first point (`[PB-SKETCH-DEFER]`, `[SCREW-F-DEFER]`).
For each section, its four corners are the world points `point(g, s_k, u, v)` at `(uB, -hv)`,
`(uF, -hv)`, `(uF, hv)` and `(uB, hv)`, each added with
`sketch.sketchPoints.add(sketch.modelToSketchSpace(world))` **with its `z` kept**: these corners lie
off the sketch's plane on purpose, the one exception to `[PB-SKETCH-ZERO-Z]`
(`[PB-3D-SKETCH-SECTIONS]`, `[SCREW-F-CELL-LOFT]`). Four solid lines share them in order
(`[PB-SHARE-XOR-COINCIDENT]`): `L1` from the first corner to the second, `L2` to the third, `L3` to
the fourth, `L4` back to the first, each `sketchLines.addByTwoPoints(corner, nextCorner)`. Keep each
section's four lines together, in station order. After the last line of the last section, set
every point `isFixed = True`. Nothing else goes in the sketch. Then set `isComputeDeferred = False`,
raise naming the sketch unless it reports `isFullyConstrained`, and raise unless
`sketch.profiles.count` is `cell*n + 1`, quoting both counts (`[PB-SELF-DIAGNOSING]`). The profiles
are not used.

Proof function: `stepCellSectionsSketch`. The sketch engine is planar, so the proof draws each
section on its own station's plane and holds every corner and every section's area; Fusion's
verdict on the one 3D sketch is the measurement of 2026-09-28 (`[SCREW-F-DIAGNOSTIC]`).

<!-- proof-run: proofkit.Run(cpCellSectionCases, stepCellSectionsSketch) -->

## 11 `[GO]` A gear's cell loft

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
      "name": "create",
      "owner": "adsk.fusion.Path",
      "reason": "Path.create raises InternalValidationError on a sketch curve in this sub-component; use component.features.createPath or pass the line directly (PB-PATH-FROM-SKETCH, PB-CONSTRUCTION-PLANES)",
      "receiver": "adsk.fusion.Path",
      "role": "forbidden",
      "span": "adsk.fusion.Path.create(collection, False)"
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
      "span": "feature = loftFeatures.add(loftInput)"
    }
  ],
  "citations": [
    {
      "first": 855,
      "last": 862,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 182,
      "last": 207,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L855–862; `spec/screwgear/fusion.md` L182–207.

Still in `buildToothCell`. `loftInput = loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`
on the `Design` component (`[PB-LOFT]`). For each section in order of `k`: a fresh
`collection = adsk.core.ObjectCollection.create()`, its four lines added with
`collection.add(line)`, then `path = self.designOcc.component.features.createPath(collection, False)`
(`[PB-PATH-FROM-SKETCH]`; never `adsk.fusion.Path.create(collection, False)`), then
`loftInput.loftSections.add(path)`. The order of the calls is the loft order. Then
`feature = loftFeatures.add(loftInput)`. Raise naming the gear unless `feature.bodies.count` is 1
and that body's `isSolid` is true (`[PB-EMPTY-RESULT]`); that body is the gear's ribbon body from
here on. Fusion's loft through more than two sections is smooth between them; the cell is one
body with six faces.

Proof function: `stepCellLoft`. decad lofts two sections at a time and cannot union two lofts that
share a section, so the proof's stand-in is the ruled loft through the same sections, stitched into
one solid, held to the ruled volume and to the box of its corners.

<!-- proof-run: proofkit3d.RunSolid(cpCellLoftCases, stepCellLoft, assertCellLoft) -->

## 12 `[PROSE]` The doubling schedule

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "popcount",
      "owner": null,
      "reason": "a count formula, not a call",
      "receiver": null,
      "role": "prose",
      "span": "floor(log2 q) + popcount(q) - 1"
    }
  ],
  "citations": [
    {
      "first": 864,
      "last": 912,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L864–912.

`repeatCellByDoubling`, once per gear, after the cell loft. This step is control flow and
makes no timeline entry; its copies, moves and joins are steps 13, 14 and 15. The finished ribbon is
`q` cells repeated under the screw step `Step(k)`: a turn of `k*P/Lambda` about the gear's axis
composed with `k*P` along it, `k` a number of teeth.

- `m = 1` (the body holds one cell). Write `q` in binary.
- For each bit of `q` below its top bit, lowest first: if that bit is set, take a copy of the body
  (step 13) and keep it unmoved as an aside of `m` cells; then double: copy the body (step 13), move
  the copy by `Step(m*cell)` (step 14), join it to the body (step 15), and `m = 2*m`.
- After the last doubling, move each aside into place, largest first, by `Step(m*cell)` (step 14),
  join it (step 15), and add its cells to `m`. The last join brings `m` to `q`.

That is `floor(log2 q) + popcount(q) - 1` rounds and as many joins. At the defaults `q = 17`: one
aside of the single cell taken before the first doubling, four doublings to 16 cells moving by
`Step(4)`, `Step(8)`, `Step(16)`, `Step(32)`, and the aside moved by `Step(64)`. When `q = 1` there is
no round. Never turn a `k = 0` step into a move: a zero-angle matrix is a no-op Fusion rejects
(`[PB-MOVE-ROTATE]`), and every step here has `k >= 1`.

When `r > 0`, steps 16 and 17 build the remainder and step 15 joins it after the last round. Then
name the body `Gear A` or `Gear B` (its `name`) and keep it in `self.gearBodies[index]`. With no
round and no remainder (`N = 4`), the cell itself is named.

## 13 `[GO]` Copy the body

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
      "span": "feature = self.designOcc.component.features.copyPasteBodies.add(body)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "feature.bodies",
      "role": "required",
      "span": "feature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 904,
      "last": 906,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 152,
      "last": 159,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L904–906; `spec/screwgear/fusion.md` L152–159.

`feature = self.designOcc.component.features.copyPasteBodies.add(body)` returns a `CopyPasteBody`
feature, not a body. The copy is `feature.bodies.item(0)`; raise naming the gear and the count
unless `feature.bodies.count` is 1 (`[PB-EMPTY-RESULT]`). Never read the feature's `sourceBody`: it
returns the original, and moving that walks the ribbon away one cell at a time
(`[SCREW-F-COPY-BODY]`).

Proof function: `stepCopyBody`. decad cannot verify two coincident bodies, so the proof's copy
retires its source and is held against a cell built afresh.

<!-- proof-run: proofkit3d.RunSolid(cpCopyCases, stepCopyBody, assertCopyBody) -->

## 14 `[GO]` Move a copy by the screw step

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
      "span": "shift.scaleBy(k*P/10)"
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
      "name": "createInput2",
      "owner": "adsk.fusion.MoveFeatures",
      "reason": null,
      "receiver": "moveFeatures",
      "role": "required",
      "span": "moveInput = moveFeatures.createInput2(bodies)"
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
      "first": 866,
      "last": 870,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 115,
      "last": 150,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L866–870; `spec/screwgear/fusion.md` L115–150.

`Step(k)` for `k` teeth (`[SCREW-F-SCREW-STEP]`), built from two matrices, in cm, with
`axisVector` the unit `dir_g` as a `Vector3D` and `axisPoint` `origin_g` as a `Point3D`:

1. `rot = adsk.core.Matrix3D.create()`, then `rot.setToRotation(k*P/Lambda, axisVector, axisPoint)`
   (radians; positive, the sense in which `theta` grows along `+dir_g`).
2. `shift = axisVector.copy()`, then `shift.scaleBy(k*P/10)`.
3. `mov = adsk.core.Matrix3D.create()`, then `mov.translation = shift` — assign the scaled vector
   whole; never scale `mov.translation` in place.
4. `rot.transformBy(mov)`.

Never assign `rot.translation` after `setToRotation`: that overwrites the part that carries the
rotation onto the gear's axis, and the body lands elsewhere with nothing raised.

Apply it (`[PB-MOVE-ROTATE]`): `bodies = adsk.core.ObjectCollection.create()`, `bodies.add(copy)`,
`moveInput = moveFeatures.createInput2(bodies)`, `moveInput.defineAsFreeMove(rot)`,
`moveFeatures.add(moveInput)`, all on the `Design` component's features.

Proof function: `stepScrewMove`. The moved cell is held against the cell built in place `k` teeth
further on: same volume, same centroid, and the same span along the axis.

<!-- proof-run: proofkit3d.RunSolid(cpMoveCases, stepScrewMove, assertScrewMove) -->

## 15 `[GO]` Join a piece to the body

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
      "span": "tools.add(piece)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combineFeatures",
      "role": "required",
      "span": "joinInput = combineFeatures.createInput(body, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combineFeatures",
      "role": "required",
      "span": "feature = combineFeatures.add(joinInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "feature.bodies",
      "role": "required",
      "span": "feature.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 886,
      "last": 909,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 354,
      "last": 364,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L886–909; `spec/screwgear/fusion.md` L354–364.

`[SCREW-F-JOIN]`: `tools = adsk.core.ObjectCollection.create()`, `tools.add(piece)`,
`joinInput = combineFeatures.createInput(body, tools)`, then
`joinInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation` and
`joinInput.isKeepToolBodies = False`, then `feature = combineFeatures.add(joinInput)`. Raise naming
the gear, the join (body cells + piece cells) and the count unless `feature.bodies.count` is
exactly 1 (`[PB-EMPTY-RESULT]`, `[PB-SELF-DIAGNOSING]`); `feature.bodies.item(0)` is the body from
then on. A piece is always made as its own new body and combined here, because a join operation
on the loft or the move itself would join into whatever it touches. Every move is by a whole
number of teeth already built, so each join meets its neighbour at one shared planar
cross-section.

Proof function: `stepJoinBodies`. decad cannot join two stitched solids, so the proof builds the
joined body directly and holds it to the two pieces, each built in a document of its own. The
default ribbon's 8+8 and 16+1 joins are past what that stand-in can build and the proof reports
them unmodelled.

<!-- proof-run: proofkit3d.RunSolid(cpJoinCases, stepJoinBodies, assertJoinBodies) -->

## 16 `[GO]` A gear's Cell Remainder sketch

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 897,
      "last": 902,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L897–902.

Only when `r > 0`, after the last round of step 12. The last `r` teeth are a second, shorter cell
built where they belong, not moved there: a sketch named `{gearLabel} Cell Remainder` by exactly
the recipe of step 10 with `cell` replaced by `r`, so `r*n + 1` sections at stations
`s_k = s0 + q*cell*P + k*P/n` for `k = 0 … r*n`, running from `s0 + q*cell*P` to `s0 + N*P`.
Deferred computing, corners with their `z` kept, four shared lines per section, every point fixed
after the last line, then the `isFullyConstrained` and `profiles.count == r*n + 1` checks. The
defaults have no remainder; 69 teeth in four-tooth cells have one of one tooth.

Proof function: `stepCellRemainderSketch`.

<!-- proof-run: proofkit.Run(cpRemainderCases, stepCellRemainderSketch) -->

## 17 `[GO]` A gear's remainder loft

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 897,
      "last": 902,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 855,
      "last": 862,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L897–902; `spec/screwgear/instructions.md` L855–862.

Only when `r > 0`. The loft of step 11 through the Cell Remainder sketch's `r*n + 1` sections, a
new body checked the same way. Its first section is the body's last, so step 15 then joins it into
the body like every other piece; after that join the body is named (step 12).

Proof function: `stepCellRemainderLoft`.

<!-- proof-run: proofkit3d.RunSolid(cpRemainderLoftCases, stepCellRemainderLoft, assertCellRemainderLoft) -->

## 18 `[GO]` The Sleeve sketch

<!-- step-meta
{
  "calls": [
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
      "receiver": "sketchCircles",
      "role": "required",
      "span": "sketchCircles.addByCenterRadius(centre, radius)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addDiameterDimension(circle, textPoint)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": "find_profile_by_curve_counts treats a circle as a curve type that disqualifies the loop, so it cannot pick the ring",
      "receiver": null,
      "role": "forbidden",
      "span": "find_profile_by_curve_counts(sketch, lines=4)"
    }
  ],
  "citations": [
    {
      "first": 937,
      "last": 949,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 314,
      "last": 328,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L937–949; `spec/screwgear/fusion.md` L314–328.

`buildCage` begins here. A sketch named `Sleeve` on `self.targetPlane` (`[PB-USE-SELECTED-PLANE]`);
computing is not deferred. `centre = sketch.modelToSketchSpace(C)` with `centre.z = 0`
(`[PB-SKETCH-ZERO-Z]`). Two circles, each `sketchCircles.addByCenterRadius(centre, radius)` with
radius `Ri/10` and `Ro/10` cm, each followed by `circle.centerSketchPoint.isFixed = True`
(`[PB-CIRCLE-CENTER]`) and `sketchDimensions.addDiameterDimension(circle, textPoint)` whose
`parameter.value` is `2*Ri/10` or `2*Ro/10`, its `textPoint` on its own circle, at `centre` plus the
circle's radius along the sketch's x axis, `z = 0` (`[PB-RADIAL-DIM]`). The two centre points are
separate points at the same place, both fixed, with no coincident between them. Nothing else goes
in the sketch. Raise unless it reports `isFullyConstrained`.

It has two profiles, the disc inside `Ri` and the ring. Iterate `sketch.profiles` and take the one
profile whose `profileLoops.count` is 2; raise with the counts unless there is exactly one
(`[PB-EMPTY-RESULT]`). Do not use `find_profile_by_curve_counts(sketch, lines=4)` here.

Proof function: `stepSleeveSketch`.

<!-- proof-run: proofkit.Run(cpSleeveSketchCases, stepSleeveSketch) -->

## 19 `[GO]` The tube

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudeFeatures",
      "role": "required",
      "span": "extrudeInput = extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
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
      "receiver": "extrudeFeatures",
      "role": "required",
      "span": "feature = extrudeFeatures.add(extrudeInput)"
    }
  ],
  "citations": [
    {
      "first": 944,
      "last": 949,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 324,
      "last": 328,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L944–949; `spec/screwgear/fusion.md` L324–328.

`extrudeInput = extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
then `extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise/10), False)` —
`False` makes the value each side's length (`[PB-THROUGH-CUT]` for the argument), so the tube runs
from `-cageRise` to `+cageRise` along the plane's normal, 37.5 mm tall at the defaults — then
`feature = extrudeFeatures.add(extrudeInput)`. Raise unless `feature.bodies.count` is 1
(`[PB-EMPTY-RESULT]`). That body is `self.cageBody` from here on.

Proof function: `stepSleeveExtrude`.

<!-- proof-run: proofkit3d.RunSolid(cpSleeveExtrudeCases, stepSleeveExtrude, assertSleeveExtrude) -->

## 20 `[PROSE]` A bore's plane

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
      "span": "constructionPlanes.createInput()"
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
      "name": "create",
      "owner": "adsk.fusion.Path",
      "reason": "Path.create raises InternalValidationError on a sketch curve in this sub-component; use component.features.createPath or pass the line directly (PB-PATH-FROM-SKETCH, PB-CONSTRUCTION-PLANES)",
      "receiver": "adsk.fusion.Path",
      "role": "forbidden",
      "span": "adsk.fusion.Path.create(line, False)"
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
      "first": 951,
      "last": 985,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 233,
      "last": 241,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L951–985; `spec/screwgear/fusion.md` L233–241.

The four bores are cut in the order gear A `-R`, gear A `+R`, gear B `-R`, gear B `+R`, each by
steps 20, 21 and 22 in turn. A bore `{-R|+R}` of gear `g` uses `line = self.pathLines[g]['bore-']`
for `-R` and `['bore+']` for `+R`, and its section station `s0 = -sOut` for `-R` and `sIn` for `+R`.

On the `Design` component: `constructionPlanes.createInput()`, then
`planeInput.setByDistanceOnPath(line, adsk.core.ValueInput.createByReal(0))`, passing the line
itself, never `adsk.fusion.Path.create(line, False)` (`[PB-CONSTRUCTION-PLANES]`), then
`constructionPlanes.add(planeInput)`, named `{gearLabel} Bore {-R|+R} Plane`. It stands square to
the axis at the line's start, the span's negative end, where Fusion put the origin on the station
to four decimals of a millimetre.

## 21 `[GO]` A bore's section sketch: the rectangle scheme

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketchLines",
      "role": "required",
      "span": "sketchLines.addByTwoPoints(O, Cp)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketchLines",
      "role": "required",
      "span": "sketchLines.addByTwoPoints(O, seedE)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addAngularDimension(Ru, K, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addAngularDimension(Ru, L2, textPoint)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addParallel(L1, K)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addOffsetDimension(K, L1, textPoint)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addParallel(L3, K)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addOffsetDimension(K, L3, textPoint)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addCoincident(E, L2)"
    },
    {
      "condition": null,
      "name": "addPerpendicular",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addPerpendicular(L2, K)"
    },
    {
      "condition": null,
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "geometricConstraints",
      "role": "required",
      "span": "geometricConstraints.addParallel(L4, L2)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketchDimensions",
      "role": "required",
      "span": "sketchDimensions.addOffsetDimension(L2, L4, textPoint)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": "find_profile_by_curve_counts is the framework helper in lib/geargen/utilities.py",
      "receiver": null,
      "role": "inherited",
      "span": "find_profile_by_curve_counts(sketch, lines=4)"
    }
  ],
  "citations": [
    {
      "first": 987,
      "last": 1022,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 608,
      "last": 628,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L987–1022; `spec/screwgear/instructions.md` L608–628.

A sketch named `{gearLabel} Bore {-R|+R}` on the bore's plane, with
`sketch.isComputeDeferred = True` right after it is created and named (`[PB-SKETCH-DEFER]`,
`[SCREW-F-DEFER]`). Every point below is a world point mapped with `modelToSketchSpace` and given
`z = 0` before use (`[PB-SKETCH-ZERO-Z]`), every seed is the solved position (`[PB-SEED-NEAR]`), and
nothing is projected (`[SCREW-F-REFERENCES]`). Write `theta = s0/Lambda + Phi_g` (−123.18° for a
`-R` bore and 70.88° for a `+R` bore at the defaults, both gears), `uB = -hw`, `uF = hw`,
`hv = ht`, and `S(u, v) = point(g, s0, u, v)`, the section point turned by `theta`.

- **References.** Reference points `O` at `origin_g + s0*dir_g` and `Cp` at
  `origin_g + s0*dir_g + (A/2)*u_g` (unturned), each `sketchPoints.add`. The construction line `Ru`
  is `sketchLines.addByTwoPoints(O, Cp)` with `isConstruction = True`.
- **The spine.** The construction line `K` from `O` to a new point `E` seeded at `S(uF, 0)`:
  `sketchLines.addByTwoPoints(O, seedE)`, `isConstruction = True`; `E` is `K.endSketchPoint`. Then set
  `O.isFixed = True` and `Cp.isFixed = True` — after `Ru` and `K` exist and before any dimension.
- **The rectangle.** Four solid lines sharing their corners, seeded at the solved points
  (`[PB-SHARE-XOR-COINCIDENT]`): `L1` from `S(uB, -hv)` to `S(uF, -hv)`, `L2` from `L1`'s end to
  `S(uF, hv)`, `L3` from `L2`'s end to `S(uB, hv)`, `L4` from `L3`'s end back to `L1`'s start.
- **The length.** `sketchDimensions.addDistanceDimension(O, E, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, textPoint)`
  at `S(uF/2, hv/4)`, `parameter.value = uF/10`.
- **The angle** (`[PB-ANGULAR-DIM]`). Normalise `theta` into (−180°, 180°].
  When `|sin theta| >= sqrt(1/2)`, `sketchDimensions.addAngularDimension(Ru, K, textPoint)`: the
  angle at `O` between the ray from `O` towards `Cp` and the ray from `O` towards `E`, value
  `acos(cos theta)` radians (between 45° and 135°), text point at `O + (uF/2)` along the unit
  bisector of those two rays, that is the section point `S'` at angle `theta/2` and radius `uF/2`
  in the unturned section frame: `origin_g + s0*dir_g + (uF/2)*(cos(theta/2)*u_g + sin(theta/2)*v_g)`.
  Otherwise, `sketchDimensions.addAngularDimension(Ru, L2, textPoint)`: the lines meet at
  `X = origin_g + s0*dir_g + (uF/cos theta)*u_g`; the angle at `X` between the ray from `X` along
  `+u_g` and the ray from `X` along `L2`'s own direction (start to end, `-sin theta*u_g + cos theta*v_g`),
  value `acos(-sin theta)` radians; text point at `X + (uF/2)` along the unit bisector of those two
  ray directions. Set the dimension's `parameter.value` to that value. All four bores take the first
  form at the defaults.
- **The rest of the rectangle** (`[PB-OFFSET-DIM]`, `[PB-NO-OVERCONSTRAIN]`).
  `geometricConstraints.addParallel(L1, K)` and `sketchDimensions.addOffsetDimension(K, L1, textPoint)`
  at `S(uF/2, -hv/2)`, value `hv/10`; `geometricConstraints.addParallel(L3, K)` and
  `sketchDimensions.addOffsetDimension(K, L3, textPoint)` at `S(uF/2, hv/2)`, value `hv/10`;
  `geometricConstraints.addCoincident(E, L2)`; `geometricConstraints.addPerpendicular(L2, K)`;
  `geometricConstraints.addParallel(L4, L2)` and `sketchDimensions.addOffsetDimension(L2, L4, textPoint)`
  at `S((uB + uF)/2, -hv/2)`, value `(uF - uB)/10`. Each offset dimension's side is the seed's
  (`[PB-DIM-VALUE-SEMANTICS]`); its value is the magnitude.

Ten degrees of freedom (`E` and the four corners) against ten rows: five dimensions and five
constraints. Set `isComputeDeferred = False`, raise naming the sketch unless it reports
`isFullyConstrained`, and take the profile with `find_profile_by_curve_counts(sketch, lines=4)`
(`[PB-PROFILE-MATCH]`): the four solid lines are the one loop; the construction lines bound nothing.

Proof function: `stepBoreSectionSketch`. The engine's signed offset carries Fusion's parallel and
offset dimension and their seeded side; its signed angle carries the wedge the text point selects.

<!-- proof-run: proofkit.Run(cpBoreSectionCases, stepBoreSectionSketch) -->

## 22 `[GO]` A bore's twisted sweep cut, and its check

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createPath",
      "owner": "adsk.fusion.Features",
      "reason": null,
      "receiver": "self.designOcc.component.features",
      "role": "required",
      "span": "path = self.designOcc.component.features.createPath(line, False)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "sweepFeatures",
      "role": "required",
      "span": "sweepInput = sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)"
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
      "receiver": "sweepFeatures",
      "role": "required",
      "span": "feature = sweepFeatures.add(sweepInput)"
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
      "first": 966,
      "last": 985,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1046,
      "last": 1059,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 243,
      "last": 290,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L966–985; `spec/screwgear/instructions.md` L1046–1059; `spec/screwgear/fusion.md` L243–290.

`path = self.designOcc.component.features.createPath(line, False)` on the bore's own Paths line
(`[PB-PATH-FROM-SKETCH]`), then
`sweepInput = sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
then `sweepInput.twistAngle = adsk.core.ValueInput.createByReal(twist)` with
`twist = +(sOut - sIn)/Lambda` radians (82.31° at the defaults), then
`sweepInput.participantBodies = [self.cageBody]`, then `feature = sweepFeatures.add(sweepInput)`.
Set nothing else: no `orientation`, no `solidTwistAxis`, no guide rail
(`[SCREW-F-TWISTED-SLOT]`, `[PB-SWEEP-TWIST]`). The sign is positive: the path runs along
`+dir_g`, the section's angle grows with `s`, and a positive twist turns the profile that way. The
ribbons are not participants and are left whole.

**The check** (`[SCREW-F-SWEEP-CHECK]`, `[PB-SELF-DIAGNOSING]`). Raise naming the bore unless
`feature.bodies.count` is 1; that body is `self.cageBody` from then on. Then, with
`sc = -cageRadius` for a `-R` bore and `+cageRadius` for a `+R` bore,
`thc = sc/Lambda + Phi_g` and `uhat = cos(thc)*u_g + sin(thc)*v_g`, probe the two points
`origin_g + sc*dir_g ± (W/2 + clearance/2)*uhat` (in cm) with
`self.cageBody.pointContainment(probe)`; both must be
`adsk.fusion.PointContainment.PointOutsidePointContainment`, and the build raises naming the bore,
both probes and both readings otherwise. The probes stand 16.86 mm (`-R`) and 16.31 mm (`+R`) from
the frame's axis inside the wall; under the wrong twist sense both sit in the wall.

Proof function: `stepBoreSweepCut`. decad has no twisted sweep and refuses a second cut of a
faceted tube, so the proof's stand-in is the channel itself: the ruled loft through the derived
count of the bore's sections (18 at the defaults), held to both ends in the air and to holding both
probes; the cut is not made there, and the proof file says so.

<!-- proof-run: proofkit3d.RunSolid(cpBoreCutCases, stepBoreSweepCut, assertBoreSweepCut) -->

## 23 `[PROSE]` The Window Plane

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
      "span": "constructionPlanes.createInput()"
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
      "receiver": "constructionPlanes",
      "role": "required",
      "span": "constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 1237,
      "last": 1249,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 329,
      "last": 336,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1237–1249; `spec/screwgear/fusion.md` L329–336.

Only when step 4 found room for at least one window; otherwise skip steps 23 to 25. One plane,
`Window Plane`, holds both windows: it holds `C` and `n̂` and stands square to the direction the
windows face (`[PB-CONSTRUCTION-PLANES]`). On the `Design` component, `constructionPlanes.createInput()`, then:

- for windows facing `±k̂` (`crossAngle <= 90°`):
  `planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.targetPlane)`,
  the plane through the Anchor Line square to the selected plane;
- for windows facing `±ê`:
  `planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))`, the
  plane square to the Anchor Line through its midpoint `C`;

then `constructionPlanes.add(planeInput)`. The windows are cut in the order the search found them,
the one facing `d` before the one facing `-d`, each by steps 24 and 25.

## 24 `[GO]` A Window sketch

<!-- step-meta
{
  "calls": [
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
      "first": 1242,
      "last": 1249,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 337,
      "last": 340,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1242–1249; `spec/screwgear/fusion.md` L337–340.

A sketch named `Window {d}` (`{d}` one of `+k`, `-k`, `+e`, `-e`) on the Window Plane; computing is
not deferred. For each hexagon corner `(t, z)` of step 4, in order, a reference point at the world
point `C + t*across + z*n̂` (cm, `across = n̂ × d`), mapped with `modelToSketchSpace`, given
`z = 0` (`[PB-SKETCH-ZERO-Z]`) and added with `sketchPoints.add`. One solid line from each corner to
the next and from the last back to the first, `sketchLines.addByTwoPoints`, sharing the points
(`[PB-SHARE-XOR-COINCIDENT]`). Then every point `isFixed = True`. Nothing else goes in the sketch.
Raise unless it reports `isFullyConstrained`, raise unless `sketch.profiles.count` is 1, then take
`profile = sketch.profiles.item(0)` (`[PB-SINGLE-PROFILE]`). The two windows are two sketches
because on the shared plane their hexagons cross.

Proof function: `stepWindowSketch`. The proof takes the search's results at the defaults as numbers
and has no case for the `±ê` windows.

<!-- proof-run: proofkit.Run(cpWindowCases, stepWindowSketch) -->

## 25 `[GO]` A window's extrude cut, and its check

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
      "receiver": "extrudeFeatures",
      "role": "required",
      "span": "extrudeInput = extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal((Ro + 1)/10)), direction)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.DistanceExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.DistanceExtentDefinition",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal((Ro + 1)/10)), direction)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal((Ro + 1)/10)), direction)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudeFeatures",
      "role": "required",
      "span": "feature = extrudeFeatures.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(C + d)"
    }
  ],
  "citations": [
    {
      "first": 1251,
      "last": 1273,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 341,
      "last": 352,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1251–1273; `spec/screwgear/fusion.md` L341–352.

**The probe** (`[PB-SELF-DIAGNOSING]`): with `(tc, zc)` the average of the hexagon's corners,
`probe = C + tc*across + zc*n̂ + ((a0(tc) + a1(tc))/2)*d` (cm), the middle of the wall where the
window goes. Before the cut, `self.cageBody.pointContainment(probe)` must be
`PointInsidePointContainment`.

**The cut**: `extrudeInput = extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
then
`extrudeInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal((Ro + 1)/10)), direction)`,
then `extrudeInput.participantBodies = [self.cageBody]` so that the ribbons in the hollow are left
whole (`[PB-THROUGH-CUT]`), then `feature = extrudeFeatures.add(extrudeInput)`. `direction` is
`adsk.fusion.ExtentDirections.PositiveExtentDirection` when `sketch.modelToSketchSpace(C + d)` has a
positive `z`, else `adsk.fusion.ExtentDirections.NegativeExtentDirection`. The cut runs one way only.

**After the cut**: raise unless `feature.bodies.count` is 1 (`[PB-EMPTY-RESULT]`); that body is
`self.cageBody`. The same probe must now be `PointOutsidePointContainment`. The build raises naming
the window and what it read otherwise; a cut extruded the wrong way leaves the probe inside.

Proof function: `stepWindowCut`. decad refuses a second cut of the same faceted tube, so each
window is cut from a tube of its own in the proof.

<!-- proof-run: proofkit3d.RunSolid(cpWindowCutCases, stepWindowCut, assertWindowCut) -->

## 26 `[PROSE]` Relocate Gear A

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
    }
  ],
  "citations": [
    {
      "first": 1275,
      "last": 1281,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1275–1281.

`relocateBodies` starts by naming the cage body `Cage` (its `name`). Then
`self.gearBodies[0].moveToComponent(self.gearOccs[0])`, which preserves world position and needs no
activation (`[PB-NO-CROSS-SIBLING]`). Keep the returned body in `self.gearBodies[0]`.

## 27 `[PROSE]` Relocate Gear B

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
      "span": "self.gearBodies[1].moveToComponent(self.gearOccs[1])"
    }
  ],
  "citations": [
    {
      "first": 1275,
      "last": 1281,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1275–1281.

`self.gearBodies[1].moveToComponent(self.gearOccs[1])`; keep the returned body in
`self.gearBodies[1]` (`[PB-NO-CROSS-SIBLING]`).

## 28 `[PROSE]` Relocate the Cage, and hide construction geometry

<!-- step-meta
{
  "calls": [
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
      "reason": "solids.hide_construction_geometry is the shared cleanup helper in lib/geargen/solids.py",
      "receiver": "solids",
      "role": "inherited",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    }
  ],
  "citations": [
    {
      "first": 1275,
      "last": 1281,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 662,
      "last": 700,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1275–1281; `spec/screwgear/instructions.md` L662–700.

`self.cageBody.moveToComponent(self.cageOcc)`; keep the returned body in `self.cageBody`
(`[PB-NO-CROSS-SIBLING]`). Then `generate` ends with
`solids.hide_construction_geometry(self.designOcc.component)` (`[PB-TREE-CLEANUP]`), which walks the
`Design` component and anything under it; do not re-implement it.

At the defaults the timeline then holds 12 sketches, 7 construction planes and 42 features
(61 entries, 66 with the five component creations). A remainder adds one sketch, one loft and one
join per gear that needs one; a window the search left out takes one sketch and one feature off,
and with neither window the Window Plane is not made.
