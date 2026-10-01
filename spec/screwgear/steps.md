The proof for this step list is `proof/screwgear/compiled_model_test.go`, `proof/screwgear/compiled_sketches_test.go`, `proof/screwgear/compiled_solids_test.go` and the generated `proof/screwgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/screwgear/instructions.md` | `376dd0621ca149606a8b519134cd81dcc5822c7a` |
| `spec/screwgear/fusion.md` | `b3511aaf4ac2c1eb0207c26828c6ef38766f83b8` |
| `CLAUDE.md` | `916e8624ca88af226c264c21f295c14a9fb9e901` |
| `proof/screwgear/README.md` | `3a24a8884a64c1bbd06031a5257dcba5511d6f20` |
| `spec/cycloidal/fusion.md` | `afa5a99986f2e0d9f82fb5e21591553cdc54aac4` |
| `spec/screwgear/mesh-search.md` | `32d393485d2edbfd173dfaa98a7cb76e4fa8471f` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `cdd32545b0f8c651752827c6697601f1e32b4d39` |

## S01 `[PROSE]` Module, classes and constants

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
      "span": "adsk.core.ValueInput.createByReal(x)"
    }
  ],
  "citations": [
    {
      "first": 8,
      "last": 9,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 390,
      "last": 421,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 492,
      "last": 499,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 650,
      "last": 664,
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

**From:** `spec/screwgear/instructions.md` L8–9; `spec/screwgear/instructions.md` L390–421; `spec/screwgear/instructions.md` L492–499; `spec/screwgear/instructions.md` L650–664; `spec/screwgear/fusion.md` L366–384.

The module is `lib/geargen/screwgear.py`. It shares no involute math and subclasses
`base.Generator` directly. It defines exactly these two public classes, which
`lib/geargen/__init__.py` exports and the command wiring binds to by name:

- **`ScrewGearCommandInputsConfigurator`** with a classmethod `configure` taking `(cls, command)` (S02).
- **`ScrewGearGenerator(base.Generator)`** with the inherited one-argument constructor
  `(design)`. It implements `generate` taking `(self, inputs)` and overrides `prefixBase` to return
  the string `'ScrewGear'`. Error cleanup is the inherited `deleteComponent`, which the shared command
  calls on failure; the module adds none of its own.

**Entry wiring.** `commands/screwgear/entry.py` declares
`command = GearCommand(gear_type='ScrewGear', name='Screw Gear Generator', description='Generates a screw/screw gearing', icon_folder=..., configurator=geargen.ScrewGearCommandInputsConfigurator, generator_class=geargen.ScrewGearGenerator)`
and re-exports `start = command.start` and `stop = command.stop`, as every gear's entry module
does. No `input_changed` callback: the dialog has no conditional visibility.

**Imports.** Only the framework, explicitly, no `import *`: `math`, `adsk.core`, `adsk.fusion`,
`fusion360utils as futil` from `...lib`, `get_design` from `.misc`, `Generator` from `.base`,
`find_profile_by_curve_counts` from `.utilities`, and `solids` from `.`.

**Parameter mode: all-Python-precomputed** [PB-PRECOMPUTED-MODE]. Every value is computed in
Python in Fusion's internal units (cm, radians) and written numerically: a sketch dimension's
value through `dimension.parameter.value`, a feature input through
`adsk.core.ValueInput.createByReal(x)`. The generator registers **no** named user parameters and
never calls `addParameter`. The two searches of S04 and S05 work in millimetres and every length
they hand to Fusion is divided by 10.

**Generation context: none.** Handles are carried on `self`: `self.designOcc` (the `Design`
occurrence), `self.gearOccs` (list of the two gear occurrences), `self.cageOcc`,
`self.gearBodies` (list of two), `self.cageBody`, `self.pathLines` (list of two dicts, one per
gear, keyed `'bore-'` and `'bore+'`, each the sketch line of S10 that bore's sweep runs along), and
`self.windows` (the windows S05 found room for, zero to two, each its facing direction, its name
and its corners).

**Module-level constants.** One per dialog input id, each named `INPUT_ID_` plus the id in
upper snake case, holding the id string of S02 exactly:

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

and one more that is not a dialog input: `CELL_TEETH = 4`, the teeth the lofted cell of S12
holds. Every count that depends on it is derived from it in `processInputs`.

**The call graph.** `generate` runs these methods, in this order and with these names:

1. `processInputs`, taking `inputs` — S03, S04, S05.
2. `buildComponentTree` — S06.
3. `buildAnchor` — S07, S08, S09.
4. `buildGear` with index 0, then with index 1 — each runs `buildSweepPaths` (S10),
   `buildToothCell` (S11, S12) and `repeatCellByDoubling` (S13 to S18), each taking the index.
5. `buildCage` — S19 to S26.
6. `relocateBodies` — S27.
7. The final cleanup of S27.

Private helpers beyond these names are free.

## S02 `[PROSE]` The dialog (`configure`)

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
      "name": "addValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "group.children",
      "role": "required",
      "span": "group.children.addValueInput(inputId, label, unitString, adsk.core.ValueInput.createByReal(internalDefault))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "group.children.addValueInput(inputId, label, unitString, adsk.core.ValueInput.createByReal(internalDefault))"
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
      "name": "addSelectionFilter",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.addSelectionFilter(filterConstant)"
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
      "reason": "misc helper the framework provides; returns the active design",
      "receiver": null,
      "role": "inherited",
      "span": "parentInput.addSelection(get_design().rootComponent)"
    }
  ],
  "citations": [
    {
      "first": 441,
      "last": 511,
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

**From:** `spec/screwgear/instructions.md` L441–511; `.claude/skills/generate-gear/PLAYBOOK.md` L128–143; `.claude/skills/generate-gear/PLAYBOOK.md` L355–357; `.claude/skills/generate-gear/PLAYBOOK.md` L557–568.

`ScrewGearCommandInputsConfigurator.configure`, taking `(cls, command)`, adds the inputs in exactly this
order. The three selection inputs come first, at the top level of
`command.commandInputs`, because Fusion focuses the first selection input and ignores a later
focus flag [PB-AUTOFOCUS-FIRST]. Then three groups, each made with
`command.commandInputs.addGroupCommandInput(groupId, groupLabel)`, whose inputs are added to that
group's `children` collection, not to the top level. `ribbonGroup` and `frameGroup` are left
expanded; `meshGroup` is set `isExpanded = False`, since its values come from the meshing search
and jam when changed carelessly. No input is ever hidden.

| Group id, label | Dialog label | input id | unit string | default (display) | `createByReal` value |
|---|---|---|---|---|---|
| top level | Target Plane | `plane` | selection | — | — |
| top level | Centre Point | `point` | selection | — | — |
| top level | Parent Component | `parent` | selection | — | — |
| `ribbonGroup`, `Ribbon` | Ribbon Width | `ribbonWidth` | `mm` | 15 mm | 1.5 |
| `ribbonGroup`, `Ribbon` | Tooth Count | `toothCount` | `''` | 68 | 68 |
| `ribbonGroup`, `Ribbon` | Twist Lead | `twistLead` | `mm` | 49.5 mm | 4.95 |
| `ribbonGroup`, `Ribbon` | Ribbon Thickness | `ribbonThickness` | `mm` | 3.75 mm | 0.375 |
| `ribbonGroup`, `Ribbon` | Tooth Pitch | `toothPitch` | `mm` | 2.625 mm | 0.2625 |
| `ribbonGroup`, `Ribbon` | Tooth Height | `toothHeight` | `mm` | 2.625 mm | 0.2625 |
| `frameGroup`, `Frame` | Cage Radius | `cageRadius` | `mm` | 15 mm | 1.5 |
| `frameGroup`, `Frame` | Cage Rise | `cageRise` | `mm` | 18.75 mm | 1.875 |
| `frameGroup`, `Frame` | Clearance | `clearance` | `mm` | 0.45 mm | 0.045 |
| `frameGroup`, `Frame` | Collar Half Length | `collarHalf` | `mm` | 3 mm | 0.3 |
| `frameGroup`, `Frame` | Collar Wall | `collarWall` | `mm` | 3 mm | 0.3 |
| `meshGroup`, `Mesh (from the mesh search)` | Crossing Angle | `crossAngle` | `deg` | 80° | `80*pi/180` |
| `meshGroup`, `Mesh (from the mesh search)` | Engagement | `engagement` | `mm` | 0.75 mm | 0.075 |
| `meshGroup`, `Mesh (from the mesh search)` | Mounting Angle A | `mountAngleA` | `deg` | 15° | `15*pi/180` |
| `meshGroup`, `Mesh (from the mesh search)` | Mounting Angle B | `mountAngleB` | `deg` | 15° | `15*pi/180` |
| `meshGroup`, `Mesh (from the mesh search)` | Assembly Phase | `assemblyPhase` | `mm` | −1.31 mm | −0.131 |

Each value input is
`group.children.addValueInput(inputId, label, unitString, adsk.core.ValueInput.createByReal(internalDefault))`,
with the default in internal units as the last column gives it [PB-DIALOG-DEFAULT-UNITS]: a length
in cm, an angle in radians, the bare count for `toothCount`.

Each selection input is
`command.commandInputs.addSelectionInput(inputId, label, tooltip)`, then two filters, each
`selectionInput.addSelectionFilter(filterConstant)` with the named constant of
`adsk.core.SelectionCommandInput`, never a quoted string [PB-SELECTION-FILTER-ENUM], then
`selectionInput.setSelectionLimits(1, 1)` [PB-SELECTION-DECL]:

| input id | filters | tooltip |
|---|---|---|
| `plane` | `ConstructionPlanes`, `PlanarFaces` | `Plane the cage's axis is normal to` |
| `point` | `ConstructionPoints`, `SketchPoints` | `Centre of the mechanism` |
| `parent` | `Occurrences`, `RootComponents` | `Component the mechanism is created under` |

The parent input pre-selects the design's root component with
`parentInput.addSelection(get_design().rootComponent)`.

## S03 `[PROSE]` Read and check the inputs (`processInputs`)

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
      "span": "inputs.itemById(inputId)"
    },
    {
      "condition": null,
      "name": "selection",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selectionInput",
      "role": "required",
      "span": "selectionInput.selection(0)"
    },
    {
      "condition": null,
      "name": "evaluateExpression",
      "owner": "adsk.core.UnitsManager",
      "reason": null,
      "receiver": "self.design.unitsManager",
      "role": "required",
      "span": "self.design.unitsManager.evaluateExpression(valueInput.expression, unitString)"
    }
  ],
  "citations": [
    {
      "first": 487,
      "last": 531,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 568,
      "last": 577,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 205,
      "last": 214,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 975,
      "last": 979,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L487–531; `spec/screwgear/instructions.md` L568–577; `.claude/skills/generate-gear/PLAYBOOK.md` L205–214; `.claude/skills/generate-gear/PLAYBOOK.md` L975–979.

**Selections first** [PB-SELECTION-STASH]. Before anything creates an occurrence, read the three
selections and stash them on `self`: each selection input is `inputs.itemById(inputId)`, and its
entity is `selectionInput.selection(0)` followed by `.entity`; raise naming the id when the input's
`selectionCount` is not 1. `self.plane` is the plane's entity and `self.point` the point's. The
parent's entity is an `Occurrence` or a `Component`: `self.parentComponent` is the occurrence's
`.component` for the first and the entity itself for the second; raise naming `parent` for anything
else.

**Every value by id.** Every input is read back with `inputs.itemById(inputId)` on the command's
top-level inputs collection, grouped or not; ids are unique across the command, which is what lets
that lookup reach into a group. `processInputs` raises naming the id if any lookup returns `None`.
Each value is read raw with
`self.design.unitsManager.evaluateExpression(valueInput.expression, unitString)` [PB-EVAL-EXPRESSION],
with the unit string of S02 (`'mm'`, `'deg'` or `''`): it returns cm for a length and radians for an
angle whatever the unit string. Convert a length to mm by multiplying by 10 for the checks and the
searches. Never read `ValueInput.realValue`.

Write the inputs, in mm and radians, as: `W` = ribbonWidth, `T` = ribbonThickness, `H` = toothHeight,
`P` = toothPitch, `N` = toothCount, `lead` = twistLead, `Sigma` = crossAngle, `Eng` = engagement,
`PhiA` = mountAngleA, `PhiB` = mountAngleB, `Z0B` = assemblyPhase, and the frame inputs by their
ids.

**Range checks**, in this order, each raised as an error whose message names the field and the
bound it broke (values quoted in the dialog's display units):

1. `ribbonWidth`, `ribbonThickness`, `toothPitch`, `twistLead`, `collarHalf`, `collarWall` and
   `clearance` must each be `> 0`.
2. `toothCount` must be a whole number, `abs(N - round(N)) < 1e-9`, and `>= 4`; use the rounded
   integer from here on.
3. `toothHeight` must be `> 0` and `< ribbonWidth/2`.
4. `engagement` must be `> 0` and `<= toothHeight`.
5. `twistLead` has no upper bound.
6. `crossAngle` must lie strictly between 0° and 180°: `0 < Sigma < pi`.
7. `assemblyPhase` must lie strictly within plus or minus `toothPitch`: `abs(Z0B) < P`.
8. Both bores must lie within the ribbon's length: `cageRadius + collarHalf + 1 mm < N*P/2`; the
   message names `cageRadius`.

No range is enforced on either mounting angle.

**Derived values**, in mm and radians:

| Name | Formula | Defaults |
|---|---|---|
| `Lambda` | `lead / (2*pi)` | 7.878 mm/rad |
| `A` | `W - Eng` | 14.25 |
| `L` | `N*P` | 178.5 |
| `c` (cell teeth) | the smaller of `CELL_TEETH` and `N` | 4 |
| `n` (steps to the tooth) | the larger of `ceil((P/Lambda) / (2*pi/180))` and 8 | 10 |
| `q`, `r` | `q = N // c`, `r = N % c` | 17, 0 |
| `Ri`, `Ro` | `cageRadius - collarHalf`, `cageRadius + collarHalf` | 12, 18 |
| `hw`, `ht` | `W/2 + clearance`, `T/2 + clearance` | 7.95, 2.325 |
| `cc` (bore corner radius) | `sqrt(hw^2 + ht^2)` | 8.283 |
| `sIn` | `sqrt(Ri^2 - cc^2) - 1` | 7.683 |
| `sOut` | `Ro + 1` | 19 |
| `axialWindow` | `1.5 * sqrt(W^2 - A^2) / sin(Sigma)` | 7.13 |
| `twist` (each bore's sweep) | `(sOut - sIn) / Lambda` | 82.31° |

## S04 `[PROSE]` The sleeve's four checks, and the wall between the bores

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 532,
      "last": 567,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1065,
      "last": 1093,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1073,
      "last": 1086,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 739,
      "last": 755,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L532–567; `spec/screwgear/instructions.md` L1065–1093; `spec/screwgear/instructions.md` L1073–1086; `spec/screwgear/instructions.md` L739–755.

After the range checks of S03 come the sleeve's four checks, in this order, each raised as an error
naming the field and the bound. All are in mm.

1. **The channel starts in the hollow**, naming `cageRadius`: require `sqrt(cc^2 + 1) < Ri`, which
   is `sIn > 0`. Message: the bore's corner with the cut's 1 mm margin reaches the sleeve's inner
   radius; quote `sIn`.
2. **The mesh stays visible along the axis**, naming `cageRadius`: require
   `sqrt(axialWindow^2 + (W/2)^2 + (T/2)^2) + clearance <= Ri`. At the defaults the left side is
   10.97 against 12.
3. **The end faces keep `collarWall`**, naming `cageRise`: require `cageRise >= A/2 + cc + collarWall`,
   18.41 at the defaults.
4. **The wall between neighbouring bores keeps `collarWall`**, naming `collarWall`: the separation
   below must be at least `collarWall`; the message names the two bores and the separation. At the
   defaults it is 4.640 mm, across the gap between the two `-R` bores.

**The frame the two searches use.** They work before any feature exists, so they place the
mechanism in a frame of their own: `C` at the origin, `e` = (1, 0, 0), `n` = (0, 0, 1),
`k = n x e` = (0, 1, 0). The gears are placed as S09 places them in world space, with these
vectors standing in for the world ones (the searches' outputs are plane coordinates and do not
depend on where the frame sits): `dirA = cos(Sigma/2) e + sin(Sigma/2) k`,
`originA = C - (A/2) n`, `uA = n`; `dirB = cos(Sigma/2) e - sin(Sigma/2) k`,
`originB = C + (A/2) n`, `uB = -n`; `v_g = dir_g x u_g`. A gear's section angle at station `s` is
`theta_g = s/Lambda + Phi_g` at station `s`, and the section point `(x, y)` (its coordinates along the
unrotated `u_g` and `v_g`) of section coordinates `(u, v)` is
`x = u*cos(theta) - v*sin(theta)`, `y = u*sin(theta) + v*cos(theta)`; the world point is
`origin_g + s*dir_g + x*u_g + y*v_g`.

**The four bores**, indexed and named in this order, which is also the order S23 cuts them:
0 `gear A -R`, 1 `gear A +R`, 2 `gear B -R`, 3 `gear B +R`. A `-R` bore's cut spans stations
`[-sOut, -sIn]` of its gear's axis and a `+R` bore's `[sIn, sOut]`; `sigma` is -1 for a `-R` bore and
+1 for a `+R` bore. A bore's **crossing** is `origin_g + sigma*cageRadius*dir_g`.

**The bore's outline** (`channelOutline`). For one bore, at stations `s = sigma*sIn`,
`sigma*(sIn + 0.1)`, `sigma*(sIn + 0.2)`, ... while `abs(s) <= sOut`, take seventeen points on each
side of the bore's rectangle, for `i = 0 .. 16`: `(hw, -ht + 2*ht*i/16)`, `(-hw, -ht + 2*ht*i/16)`,
`(-hw + 2*hw*i/16, ht)` and `(-hw + 2*hw*i/16, -ht)`, each in section coordinates, turned to the
station's angle and placed as the world point above. Keep a point when its distance from the frame's
axis, `sqrt((P - C).e^2 + (P - C).k^2)`, lies within `Ri - 0.5` to `Ro + 0.5`.

**The separation** (`channelSeparation`). The four gaps between neighbouring bores, each a facing
direction `d` and the two bores either side of it, are: `+k` between bores 1 and 2; `-e` between
bores 2 and 0; `-k` between bores 0 and 3; `+e` between bores 3 and 1. For each gap:

1. With `across = n x d`, project the kept outline points of each of its two bores to
   `((P - C).across, (P - C).n)`, and take the convex hull of each bore's projections by Andrew's
   monotone chain (sort by the first coordinate then the second; build the lower and the upper chain,
   popping while the cross product of the last two kept points and the new one is `<= 0`;
   counter-clockwise).
2. For every edge of either hull, from corner `a` to the next corner `b`, let `m` be the unit vector
   `(b.y - a.y, a.x - b.x)` normalised (skip an edge of zero length). With `p` over the first hull's
   corners and `q` over the second's, the separation along `m` is the larger of
   `min(m.q) - max(m.p)` and `min(m.p) - max(m.q)`.
3. The gap's separation is the largest over all those edges; it is negative when the hulls overlap.

The separation the fourth check reads is the least over the four gaps, and the gap it is least
across names the two bores in the message.

## S05 `[PROSE]` The window search

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "log",
      "owner": null,
      "reason": "fusion360utils logging helper the framework provides",
      "receiver": "futil",
      "role": "inherited",
      "span": "futil.log(message)"
    }
  ],
  "citations": [
    {
      "first": 1095,
      "last": 1239,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1220,
      "last": 1227,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 572,
      "last": 574,
      "path": "spec/screwgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1095–1239; `spec/screwgear/instructions.md` L1220–1227; `spec/screwgear/instructions.md` L572–574.

After the four checks, still in `processInputs` and before any feature, find each window's hexagon.
The search refuses nothing. All in mm, in the frame of S04.

**The facings.** The windows face `+k` then `-k` when `Sigma <= pi/2`, and `+e` then `-e`
otherwise; their names, used in sketch names and messages, are `+k`, `-k`, `+e`, `-e`. Each window
is found the same way from its own facing direction `d`; S26 cuts the one facing the first direction
before the one facing the second.

**Plane coordinates.** For a window facing `d`, `across = n x d` (which is `-e` for `d = +k`). A
point `P` has plane coordinates `t = (P - C).across` and `z = (P - C).n`, and depth
`a = (P - C).d`. The window is the set of points with `a > 0` whose `(t, z)` lie in its hexagon. At
`t` the wall runs from `a0(t) = sqrt(max(0, Ri^2 - t^2))` to `a1(t) = sqrt(max(0, Ro^2 - t^2))`.

**A bore's section in the wall** (`sectionInWall`), for a bore of gear `g` at station `s` with
`abs(s) < Ro` (at `abs(s) >= Ro` it has no piece). In section-plane coordinates `x` along `u_g`
and `y` along `v_g`: take the rectangle's corners `(-hw, -ht)`, `(hw, -ht)`, `(hw, ht)`,
`(-hw, ht)`, in that order (counter-clockwise), each turned by `theta_g` at `s`. Clip it to the heights
inside the end faces: keep the `x` for which `(origin_g + x*u_g - C).n` lies within plus or minus
`cageRise` (with `u_g.n` = +1 or -1, this is `x` between the two values
`(-cageRise - (origin_g - C).n)/(u_g.n)` and `(cageRise - (origin_g - C).n)/(u_g.n)`, taken low to
high). Then clip what is left twice more, into two **pieces**: one to `near <= y <= far` and one to
`-far <= y <= -near`, with `near = sqrt(max(0, Ri^2 - s^2))` and `far = sqrt(Ro^2 - s^2)`. Either
piece may be empty. Each clip keeps the part of a convex polygon where `alpha*x + beta*y <= gamma`:
walk its edges in order from each corner `p` to the next `r`, with `fp = gamma - (alpha*p.x + beta*p.y)`
and `fr` likewise; keep `p` when `fp >= 0`, and when `fp >= 0` and `fr >= 0` differ add the point
`p + (fp/(fp - fr))*(r - p)`.

**The flanking and far bores.** The two bores whose crossings lie on the window's side,
`(crossing - C).d > 0`, **flank** the window; the other two are its **far** bores. The **low**
flanking bore is the one whose crossing has the smaller `(crossing - C).n`, the other the **high**
one. `lean = +1` when the high one's crossing has the larger `t`, else `-1`.

**The long sides** (`wallCorners`). Walk each flanking bore's cut span at stations every 0.001 mm
from the span's lower end (`-sOut` for a `-R` bore, `sIn` for a `+R` bore) while `s <= ` its upper
end, and take every corner of every non-empty piece as the world point
`origin_g + x*u_g + y*v_g + s*dir_g`, with its measure `m = z + lean*t`. `lowReach` is the largest
`m` over the low bore's corners and `highReach` the least over the high bore's. Then
`lo = lowReach + sqrt(2)*collarWall` and `hi = highReach - sqrt(2)*collarWall`.

**The trims.** `zLimit` (`channelTop`) is the highest any channel reaches in the wall: over all four
bores, at stations from `sigma*sIn` outward every 0.01 mm while `abs(s) <= sOut`, wherever the
section reaches the wall — `abs(s) <= Ro` and
`sqrt(s^2 + (hw*abs(sin(theta)) + ht*abs(cos(theta)))^2) >= Ri` — take
`A/2 + hw*abs(cos(theta)) + ht*abs(sin(theta))`; `zLimit` is the largest, 15.01 mm at the defaults.
Then `top = min(2*zLimit - hi, hi + sqrt(2)*Ri)` and `bottom = max(-2*zLimit - lo, lo - sqrt(2)*Ri)`.

**The distance to a channel** (`wallGap`), for a point `Q`, one bore of gear `g` and a cap `reach`.
In the gear's frame `x = (Q - origin_g).u_g`, `y = (Q - origin_g).v_g`,
`sq = (Q - origin_g).dir_g`. When
`sqrt(x^2 + y^2 + (sq - clamp(sq, spanStart, spanEnd))^2) - cc >= reach` the distance is `reach`
and nothing is walked. Otherwise walk the bore's **station table**, built once per bore before its
first walk: stations `spanStart + k*0.002` for `k = 0, 1, ...` while `<= spanEnd`, each with its two
pieces, and wherever the set of non-empty pieces differs between two neighbouring stations, the
stations every 0.0001 mm strictly between those two, all in order of `s`. For each non-empty piece
keep its **circle**: the centre `(cx, cy)` is the average of its corners and the radius the largest
distance from it to a corner. Start with `best = reach^2`. Find the first station at or above `sq`;
walk up from it, then down from the one before it, stopping each way at the first station with
`(sq - s)^2 >= best`. At each station, for each non-empty piece: with
`o = sqrt((x - cx)^2 + (y - cy)^2) - radius`, pass over the piece when `o > 0` and
`(sq - s)^2 + o^2 >= best`; otherwise set `best = min(best, (sq - s)^2 + d2)`, where `d2` is 0 when
`(x, y)` is inside the piece — on the inner side of every edge of the counter-clockwise polygon,
`(r.x - p.x)*(y - p.y) - (r.y - p.y)*(x - p.x) >= 0` for every edge `p` to `r`, which a piece of
fewer than three corners never is — and otherwise the least squared distance from `(x, y)` to the
piece's edges (each edge a segment, the foot clamped to it). The distance is `sqrt(best)`.

**The ends** (`windowEnd`). `need = collarWall + 0.1/sqrt(2) + 0.005`, 3.076 at the defaults. For
each side `delta = +1` (right) and `delta = -1` (left), bisect `te` 24 times between `lo_e = 0` and
`hi_e = min(Ri, Ro/sqrt(2))*(1 - 1e-9)`: take the middle; when `clear(middle)` holds it becomes the
new `lo_e`, otherwise the new `hi_e`. The end is the final `lo_e`: `right = end(+1)` and
`left = -end(-1)`. `clear(te)`, at `t = delta*te`:

1. `zLow = max(lo - lean*t, bottom + lean*t)` and `zHigh = min(hi - lean*t, top + lean*t)`; not clear
   when `zLow > zHigh`.
2. The points tested are each `C + t*across + z*n + a*d`: the two corner lines through the wall, at
   `z = zLow` and at `z = zHigh`, each at `a = a0(t), a0(t) + 0.1, ...`, a step that would pass
   `a1(t)` landing on it and the walk ending there; and, on the inner face `a = a0(t)` and on the
   outer face `a = a1(t)`, the end's upright edge at `z = zLow + (zHigh - zLow)*kk/nn` for
   `kk = 1 .. nn - 1` with `nn = ceil((zHigh - zLow)/0.1)`, and at `z = -zLimit + j*0.1` for every
   `j >= 0` with `z <= zLimit` and `zLow < z < zHigh`.
3. Not clear as soon as any point's distance to either far bore's channel, by `wallGap` with
   `reach = need`, is under `need`; clear otherwise.

**The hexagon** (`clipCorners`). Start from the square of corners `(-2*Ro, -2*Ro)`, `(2*Ro, -2*Ro)`,
`(2*Ro, 2*Ro)`, `(-2*Ro, 2*Ro)` in `(t, z)` and clip it, by the convex clip above with `(x, y)` read
as `(t, z)`, in this order, to `lean*t + z <= hi`, `-lean*t - z <= -lo`, `-lean*t + z <= top`,
`lean*t - z <= -bottom`, `t <= right` and `-t <= -left`. Then drop near-duplicates: walking the
corners in order, drop a corner that lies within 0.001 mm of the last corner kept, and after the walk
drop the last corner kept while it lies within 0.001 mm of the first, since a sketch line cannot have
zero length. At the defaults the `+k` window has `lo = -3.335`, `hi = 6.495`, `bottom = -20.306`,
`top = 23.466`, `left = -11.404` and `right = 11.663`, and its six corners in `(t, z)` are
`(11.66, -5.17)`, `(-8.49, 14.98)`, `(-11.40, 12.06)`, `(-11.40, 8.07)`, `(8.49, -11.82)` and
`(11.66, -8.64)`, 208.1 mm² by the shoelace formula. The `-k` window comes out as the `+k` one turned
half a turn about `e`, its corners `(-t, -z)` of those; that is a property of the result, not a
shortcut the build takes.

**When a gap has no room** (`room`). A window is not cut when `hi <= lo` (reason
`the flanking bores leave no band between them`), or when `right <= left`, or the hexagon has fewer
than three corners, or its shoelace area is `<= 0` (reason `the far bores leave the band no length`).
The build then logs `No window facing {d}: {reason}` with `futil.log(message)` [PB-LOGGING], `{d}`
being `+k`, `-k`, `+e` or `-e`, and leaves that window out of `self.windows`. Each window that has
room is kept as its facing `d`, its name, its `across`, and its hexagon's corners in `(t, z)`, in mm.

## S06 `[PROSE]` The component tree

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "getOccurrence",
      "owner": null,
      "reason": "base.Generator method; it creates the top occurrence under the parent component",
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
    },
    {
      "condition": null,
      "name": "activate",
      "owner": "adsk.fusion.Occurrence",
      "reason": "activating a component makes Fusion resolve the selected plane in the wrong frame [PB-NEVER-ACTIVATE]",
      "receiver": "occurrence",
      "role": "forbidden",
      "span": "occurrence.activate()"
    }
  ],
  "citations": [
    {
      "first": 422,
      "last": 440,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 929,
      "last": 946,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L422–440; `.claude/skills/generate-gear/PLAYBOOK.md` L929–946.

Five timeline entries [PB-OCCURRENCE-TREE]. The top occurrence is the inherited
`self.getOccurrence()`, which creates it under `self.parentComponent` and is what the inherited
`deleteComponent` deletes on failure; set its component's `name` to `Screw Gearing`. Never call
`addNewComponent` for the top occurrence yourself. Under its component add four children, in this
order, each `component.occurrences.addNewComponent(adsk.core.Matrix3D.create())` with its
component's `name` set: `Design` (`self.designOcc`), `Gear A`, `Gear B` (`self.gearOccs`) and `Cage`
(`self.cageOcc`). Every sketch, construction plane and feature of S07 to S26 is made in the `Design`
occurrence's component; the other three stay empty until S27 relocates the finished bodies into them
[PB-NO-CROSS-SIBLING].

**Never call `occurrence.activate()`** [PB-NEVER-ACTIVATE], and place sketches directly on the
user-selected plane rather than on a coplanar copy of it [PB-USE-SELECTED-PLANE].

## S07 `[GO]` The Anchor sketch

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
      "span": "projectedPoint = sketch.project(self.point).item(0)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "line = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(g0.x - 0.5, g0.y, 0), adsk.core.Point3D.create(g0.x + 0.5, g0.y, 0))"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "line = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(g0.x - 0.5, g0.y, 0), adsk.core.Point3D.create(g0.x + 0.5, g0.y, 0))"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(projectedPoint, line)"
    },
    {
      "condition": null,
      "name": "addMidPoint",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addMidPoint(projectedPoint, line)"
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
      "span": "sketch.sketchDimensions.addDistanceDimension(line.startSketchPoint, line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, anchorText)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(g0.x, g0.y + 0.3, 0)"
    }
  ],
  "citations": [
    {
      "first": 708,
      "last": 738,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 373,
      "last": 384,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 447,
      "last": 450,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 458,
      "last": 472,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L708–738; `spec/screwgear/fusion.md` L373–384; `.claude/skills/generate-gear/PLAYBOOK.md` L447–450; `.claude/skills/generate-gear/PLAYBOOK.md` L458–472.

Proof function `stepAnchorSketch`.

<!-- proof-run: proofkit.Run(anchorCases, stepAnchorSketch) -->

On the `Design` component: `sketch = component.sketches.add(self.plane)`, named `Anchor`. This
sketch is not deferred. Project the selected point,
`projectedPoint = sketch.project(self.point).item(0)`; this is the one projection in the build
[SCREW-F-REFERENCES], [PB-SKETCH-FIRST].

Draw the **Anchor Line** from two raw seeds 0.5 cm either side of the projected point along the
sketch's own x axis, each with `z = 0` [PB-SKETCH-ZERO-Z]: with `g0 = projectedPoint.geometry`,
`line = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(g0.x - 0.5, g0.y, 0), adsk.core.Point3D.create(g0.x + 0.5, g0.y, 0))`,
so its end is to the right of its start. Constrain it with these four things and nothing else:

- `sketch.geometricConstraints.addCoincident(projectedPoint, line)` and
  `sketch.geometricConstraints.addMidPoint(projectedPoint, line)` — both, not the midpoint alone;
- `sketch.geometricConstraints.addHorizontal(line)`, sketch-local [PB-REFLINE-DIRECTION];
- a **horizontal** distance from the line's start to its end,
  `sketch.sketchDimensions.addDistanceDimension(line.startSketchPoint, line.endSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, anchorText)`,
  with its parameter's `value` set to 1.0 (cm). Not an aligned one: midpoint, horizontal and an
  aligned length hold the line in either end-for-end orientation, and only a horizontal distance
  from start to end runs one way. `anchorText` is `adsk.core.Point3D.create(g0.x, g0.y + 0.3, 0)`.

Raise naming `Anchor` unless `sketch.isFullyConstrained` [PB-FULL-CONSTRAINT]. Only then read the
frame from world geometry [PB-WORLDGEO-CONSTRAINED], [PB-WORLD-FRAME]: `C` is
`projectedPoint.worldGeometry`, and `e` is the unit vector from `line.startSketchPoint.worldGeometry`
to `line.endSketchPoint.worldGeometry`. Keep `line` as `self.anchorLine`: it is passed once more, as
the line the Window Plane of S24 is built on. No later sketch projects anything.

In the proof the projected point is a reference point (`CreateReferencePoint`), and the engine's
midpoint carries the coincident row, so the proof writes the midpoint alone.

## S08 `[PROSE]` Gear A Axis Plane, and the sign of n

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
      "span": "axisPlaneA = component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 756,
      "last": 762,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 429,
      "last": 438,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 793,
      "last": 797,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L756–762; `spec/screwgear/fusion.md` L429–438; `.claude/skills/generate-gear/PLAYBOOK.md` L793–797.

A construction plane offset from the selected plane itself [PB-USE-SELECTED-PLANE],
[PB-CONSTRUCTION-PLANES]: `planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(-A/2/10))`,
`axisPlaneA = component.constructionPlanes.add(planeInput)`, named `Gear A Axis Plane`.

Fusion offsets along the selected entity's own normal and the build does not assume its sign
[SCREW-F-NORMAL-SIGN]. Read `axisPlaneA.geometry`, a plane with an `origin` and a `normal`:
`n = normal` if `(C - origin).normal > 0`, else `-normal`, normalised. So `C` lies `+A/2` along `n`
from gear A's plane by construction. The check that `C` is `A/2` from it comes in S09.

The proof draws no such plane: neither engine gates a construction plane. The Paths sketch of S10
is drawn in this plane's coordinates and asserts the gear's axis lies in it.

## S09 `[PROSE]` Gear B Axis Plane, and the frame

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
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(A/2/10))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(A/2/10))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "axisPlaneB = component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 740,
      "last": 762,
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

**From:** `spec/screwgear/instructions.md` L740–762; `spec/screwgear/fusion.md` L429–438.

`planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(A/2/10))`,
`axisPlaneB = component.constructionPlanes.add(planeInput)`, named `Gear B Axis Plane`. No later
plane is offset from the selected plane.

**Check both planes** [SCREW-F-NORMAL-SIGN], [PB-SELF-DIAGNOSING]: for each of the two planes,
`abs((C - origin).normal)` read off its `geometry` must be `A/2` within 1e-6 cm, and `C` must lie on
opposite sides of the two; raise naming the plane and the distance read otherwise.

**The frame** of the mechanism, in world space and cm, from `C`, `e` and `n`:

- `k = n x e`, level and square to `e`.
- Gear A: `dirA = cos(Sigma/2)*e + sin(Sigma/2)*k` (that is `e` turned by `+Sigma/2` about `n`),
  `originA = C - (A/2)*n`, `uA = +n`, `vA = dirA x uA`, `PhiA` = mountAngleA, tooth phase `Z0A = 0`.
- Gear B: `dirB = cos(Sigma/2)*e - sin(Sigma/2)*k`, `originB = C + (A/2)*n`, `uB = -n`,
  `vB = dirB x uB`, `PhiB` = mountAngleB, tooth phase `Z0B` = assemblyPhase.

`u` points at the other gear, which is why the two gears are the same part rather than mirror
images. A point of gear `g` at station `s` with section coordinates `(u, v)` is
`origin_g + s*dir_g + (u*cos(theta) - v*sin(theta))*u_g + (u*sin(theta) + v*cos(theta))*v_g`, with
`theta = s/Lambda + Phi_g`; every seed and reference point below is such a point. Directions are
named by `e`, `k` and `n`; with `C` at the origin they are the proof's X, Y and Z axes.

## S10 `[GO]` A gear's Paths sketch

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
      "span": "sketch = component.sketches.add(axisPlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "local = sketch.modelToSketchSpace(world)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "point = sketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "boreMinus = sketch.sketchCurves.sketchLines.addByTwoPoints(pointMinusOut, pointMinusIn)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "borePlus = sketch.sketchCurves.sketchLines.addByTwoPoints(pointPlusIn, pointPlusOut)"
    }
  ],
  "citations": [
    {
      "first": 764,
      "last": 783,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 579,
      "last": 603,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 386,
      "last": 416,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 591,
      "last": 608,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 632,
      "last": 641,
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

**From:** `spec/screwgear/instructions.md` L764–783; `spec/screwgear/instructions.md` L579–603; `spec/screwgear/fusion.md` L386–416; `.claude/skills/generate-gear/PLAYBOOK.md` L591–608; `.claude/skills/generate-gear/PLAYBOOK.md` L632–641; `.claude/skills/generate-gear/PLAYBOOK.md` L840–850.

Proof function `stepPathsSketch`.

<!-- proof-run: proofkit.Run(pathsCases, stepPathsSketch) -->

S10 to S18 run for gear A (`index = 0`, `gearLabel = 'Gear A'`, on the Gear A Axis Plane) and then
again for gear B (`index = 1`, `gearLabel = 'Gear B'`, on the Gear B Axis Plane).

`sketch = component.sketches.add(axisPlane)`, named `{gearLabel} Paths`; not deferred. It holds
four **reference points** at `origin_g + s*dir_g` for `s = -sOut, -sIn, sIn, sOut` (cm), each made by
the recipe of [SCREW-F-REFERENCES]: `local = sketch.modelToSketchSpace(world)`, then `local.z = 0`
(every one of these points is meant to lie on the plane) [PB-SKETCH-ZERO-Z], then
`point = sketch.sketchPoints.add(local)`. Then two solid lines sharing those points
[PB-SHARE-XOR-COINCIDENT], each drawn from its negative station to its positive one so its start is
its negative end and it runs along `+dir_g`:
`boreMinus = sketch.sketchCurves.sketchLines.addByTwoPoints(pointMinusOut, pointMinusIn)` and
`borePlus = sketch.sketchCurves.sketchLines.addByTwoPoints(pointPlusIn, pointPlusOut)`. After both
lines exist set all four points `isFixed = True` [PB-PROJECT-NOT-FIXED]. Nothing else: no constraint,
no dimension. Raise naming the sketch unless `sketch.isFullyConstrained` [PB-FULL-CONSTRAINT].
Store the lines as `self.pathLines[index]['bore-']` and `self.pathLines[index]['bore+']`.

At the defaults the stations are 7.683 and 19 mm and each line is 11.317 mm long. A line whose two
ends are fixed has a `worldGeometry` the build can trust [PB-WORLDGEO-CONSTRAINED]; each line is later
the path of its bore's sweep and the curve its bore plane stands on.

## S11 `[GO]` A gear's Cell Sections sketch

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
      "span": "sketch = component.sketches.add(axisPlane)"
    },
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
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(cornerA, cornerB)"
    }
  ],
  "citations": [
    {
      "first": 785,
      "last": 858,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 604,
      "last": 611,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 633,
      "last": 643,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 161,
      "last": 214,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 292,
      "last": 304,
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

**From:** `spec/screwgear/instructions.md` L785–858; `spec/screwgear/instructions.md` L604–611; `spec/screwgear/instructions.md` L633–643; `spec/screwgear/fusion.md` L161–214; `spec/screwgear/fusion.md` L292–304; `.claude/skills/generate-gear/PLAYBOOK.md` L878–902.

Proof function `stepCellSectionsSketch`.

<!-- proof-run: proofkit.Run(cellSectionsCases, stepCellSectionsSketch) -->

One sketch holds every section of the gear's tooth cell [SCREW-F-CELL-LOFT], [PB-3D-SKETCH-SECTIONS]:
`sketch = component.sketches.add(axisPlane)` on the gear's own Axis Plane, named
`{gearLabel} Cell Sections`. Right after naming it, set `sketch.isComputeDeferred = True`
[PB-SKETCH-DEFER], [SCREW-F-DEFER]. No section plane is made.

The cell is `c` teeth at the ribbon's negative end, `c*n + 1` sections (41 at the defaults, 11 at a
one-tooth cell): section `k`, for `k = 0 .. c*n`, stands at station `s_k = s0 + k*P/n` with
`s0 = Z0_g - L/2`. Its corners are the world points of S09 at section coordinates `(uB, -hv)`,
`(uF, -hv)`, `(uF, hv)`, `(uB, hv)`, in that order, with `uB = -W/2`, `hv = T/2` and
`uF = W/2 - H/2 + (H/2)*cos(2*pi*(s_k - Z0_g)/P)`, the section turned by
`theta_k = s_k/Lambda + Phi_g`, every length divided by 10 for cm. Each corner is
`sketch.sketchPoints.add(sketch.modelToSketchSpace(world))` with the mapped point's `z` **kept**:
these corners lie off the plane on purpose, and this is the one sketch where [PB-SKETCH-ZERO-Z] does
not apply. Four solid lines share them in order — `L1` from the first corner to the second, `L2` on to
the third, `L3` on to the fourth, `L4` back to the first — each
`sketch.sketchCurves.sketchLines.addByTwoPoints(cornerA, cornerB)` [PB-SHARE-XOR-COINCIDENT]. Keep
each section's four lines together, in order of `k`: they are what the loft is fed.

After the last line of the last section, set every point of the sketch `isFixed = True`, then
`sketch.isComputeDeferred = False`. Nothing else is in the sketch: no construction line, no
constraint, no dimension. Raise naming the sketch unless `sketch.isFullyConstrained`, and raise unless
`sketch.profiles.count` is `c*n + 1`. The profiles are not what the loft is fed, since nothing says
which profile is which station.

What the proof builds: the sketch engine is planar, so the stand-in lays the `c*n + 1` sections side by
side in one plane, each its four corners as fixed points and its four solid lines, and holds the
section count, one valid profile per section and each section's area `(uF + W/2)*T`. Fusion's verdict
on the off-plane points (fully constrained, one profile per section, measured at 11, 41 and 81
sections) is this step's part the proof cannot reach.

## S12 `[GO]` A gear's cell loft

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
      "receiver": "component.features",
      "role": "required",
      "span": "path = component.features.createPath(collection, False)"
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
    }
  ],
  "citations": [
    {
      "first": 859,
      "last": 866,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 813,
      "last": 831,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 182,
      "last": 207,
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
    },
    {
      "first": 440,
      "last": 440,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L859–866; `spec/screwgear/instructions.md` L813–831; `spec/screwgear/fusion.md` L182–207; `.claude/skills/generate-gear/PLAYBOOK.md` L742–746; `.claude/skills/generate-gear/PLAYBOOK.md` L840–850; `.claude/skills/generate-gear/PLAYBOOK.md` L440.

Proof function `stepCellLoft`.

<!-- proof-run: proofkit3d.RunSolid(cellLoftCases, stepCellLoft, assertCellLoft) -->

Loft the sections in station order [PB-LOFT], [SCREW-F-CELL-LOFT]:
`loftInput = component.features.loftFeatures.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.
For each section in order of `k`: `collection = adsk.core.ObjectCollection.create()`, then
`collection.add(line)` for its four lines `L1`, `L2`, `L3`, `L4`, then
`path = component.features.createPath(collection, False)` on the `Design` component
[PB-PATH-FROM-SKETCH] — never `adsk.fusion.Path.create`, which raises in this component — and
`loftInput.loftSections.add(path)`. Then `loft = component.features.loftFeatures.add(loftInput)`.
Set nothing else on the input: it has no ruled option, and the loft through more than two sections
is smooth between them.

Raise unless `loft.bodies.count` is exactly 1 and that body `isSolid` [PB-EMPTY-RESULT]; the message
names the gear and the count. That body is the cell, `c` pitches of twisted toothed ribbon in one body
with six faces, and the body the doubling starts from.

The proof's stand-in is the ruled loft through the same sections, closed by its two end sections, its
volume held to the ruled loft's own to within what the choice of each quad's diagonal can move it.

## S13 `[GO]` Copy the body (a doubling round, or an aside)

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
      "first": 868,
      "last": 900,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 908,
      "last": 909,
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

**From:** `spec/screwgear/instructions.md` L868–900; `spec/screwgear/instructions.md` L908–909; `spec/screwgear/fusion.md` L152–159.

Proof function `stepCopyBody`.

<!-- proof-run: proofkit3d.RunSolid(copyCases, stepCopyBody, assertCopyBody) -->

S13, S14 and S15 are the three features of a doubling **round**; S13 is also how an aside is taken.
The **schedule** for `q` cells, `m` being the cells the body holds:

1. Start with the cell, `m = 1`. Let `top` be the index of `q`'s highest set bit.
2. For each bit `b = 0 .. top - 1` of `q`, lowest first: if bit `b` is set, take a copy of the body
   (S13) and keep it unmoved as an **aside** of `m` cells; then **double**: copy the body (S13), move
   the copy by `Step(m*c)` (S14), join it to the body (S15), and `m` becomes `2*m`.
3. After the last doubling, place each aside, largest first: move it by `Step(m*c)` (S14), join it
   (S15), and add its cells to `m`. The last join brings `m` to `q`.

That is floor(log2(q)) + popcount(q) - 1 rounds: 5 at the defaults (`q = 17`: one aside of the single
cell taken before the first doubling, four doublings to 16 cells, the aside moved by `Step(64)`), and
7 at a one-tooth cell (`q = 68`: an aside of 4 cells taken when the body held 4, six doublings to 64,
the aside moved by `Step(64)`). When `q = 1` there is no round. Never add a `k = 0` move to the loop: a
zero-angle matrix is a move Fusion rejects [PB-MOVE-ROTATE].

**The copy** [SCREW-F-COPY-BODY]: `copyFeature = component.features.copyPasteBodies.add(body)`. It
returns a feature, not a body; the copy is `copyFeature.bodies.item(0)`. Never read the feature's
`sourceBody`, which is the original. Raise naming the gear unless `copyFeature.bodies.count` is 1.

The proof takes the copy with `Duplicate` and then sets the source aside by a translation, since a
copy coincides with its source face for face and decad's verifier cannot classify that pair; it holds
the copy's volume and centroid to the source's.

## S14 `[GO]` Move the copy by the screw step

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
      "first": 870,
      "last": 875,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 909,
      "last": 916,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 115,
      "last": 150,
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

**From:** `spec/screwgear/instructions.md` L870–875; `spec/screwgear/instructions.md` L909–916; `spec/screwgear/fusion.md` L115–150; `.claude/skills/generate-gear/PLAYBOOK.md` L822–831.

Proof function `stepScrewMove`.

<!-- proof-run: proofkit3d.RunSolid(moveCases, stepScrewMove, assertScrewMove) -->

`Step(k)` for `k` teeth is a translation of `k*P` along the gear's axis composed with a rotation of
`k*P/Lambda` about it [SCREW-F-SCREW-STEP], built from two matrices, in cm and radians, with
`axisVector` the unit `dir_g` as a vector and `axisPoint` the point `origin_g`:

1. `rot = adsk.core.Matrix3D.create()`, then `rot.setToRotation(k*P/Lambda, axisVector, axisPoint)`.
2. `shift = axisVector.copy()`, then `shift.scaleBy(k*P/10)`.
3. `mov = adsk.core.Matrix3D.create()`, then `mov.translation = shift`, assigning the whole vector.
4. `rot.transformBy(mov)`.

Never build the rotation and then assign `rot.translation`: `setToRotation` already writes the
translation that carries the rotation off the world origin onto `axisPoint`, and overwriting it turns
the move into a rotation about a parallel line through the world origin with nothing raised. A positive
angle turns the section the way `theta = s/Lambda + Phi` grows, which carries the section at `s` onto
the one at `s + k*P`.

Apply it with `bodies = adsk.core.ObjectCollection.create()`, `bodies.add(copy)`,
`moveInput = component.features.moveFeatures.createInput2(bodies)`,
`moveInput.defineAsFreeMove(rot)`, `component.features.moveFeatures.add(moveInput)`
[PB-MOVE-ROTATE]. A doubling moves the copy by `Step(m*c)`; an aside placed after the doublings is
moved by `Step(m*c)` with `m` the cells the body then holds.

The proof moves the copy with `Placed` and holds it, by volume and centroid, to the same cells lofted
where it lands: the body is invariant under its screw step.

## S15 `[GO]` Join the moved copy to the body

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
      "span": "tools.add(moved)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "combineInput = component.features.combineFeatures.createInput(body, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "component.features.combineFeatures",
      "role": "required",
      "span": "combineFeature = component.features.combineFeatures.add(combineInput)"
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
      "first": 896,
      "last": 900,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 910,
      "last": 913,
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

**From:** `spec/screwgear/instructions.md` L896–900; `spec/screwgear/instructions.md` L910–913; `spec/screwgear/fusion.md` L354–364.

Proof function `stepJoinBodies`.

<!-- proof-run: proofkit3d.RunSolid(joinCases, stepJoinBodies, assertJoinBodies) -->

[SCREW-F-JOIN]: `tools = adsk.core.ObjectCollection.create()`, `tools.add(moved)`,
`combineInput = component.features.combineFeatures.createInput(body, tools)`, set
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation` and
`combineInput.isKeepToolBodies = False`, then
`combineFeature = component.features.combineFeatures.add(combineInput)`. It must leave one body: raise
naming the gear and the count unless `combineFeature.bodies.count` is exactly 1 [PB-EMPTY-RESULT],
[PB-SELF-DIAGNOSING]; `combineFeature.bodies.item(0)` is the body from then on. Every move is by a whole
number of teeth already built, so each join meets its neighbour at one shared cross-section and leaves
no sliver and no overlap.

When the last round's join is done and there is no remainder (S16), name the body `{gearLabel}`
(`Gear A` or `Gear B`) and store it as `self.gearBodies[index]`; when `q = 1` and `r = 0` there is no
join and the cell itself is named and stored.

The proof's join is the stitch of the two bodies' wall sheets, the cell's ruled walls with the shared
section left out, closed by the ribbon's two end sections. decad's audit refuses a closed ribbon of more
than about 48 teeth at ten sections a tooth, so the defaults' 68 teeth are a case it records and cannot
represent; it builds the schedule at 8, 12, 28 and 48 teeth and at 13 one-tooth cells.

## S16 `[GO]` A gear's Cell Remainder sketch (only when r > 0)

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 901,
      "last": 906,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 161,
      "last": 181,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L901–906; `spec/screwgear/fusion.md` L161–181.

Proof function `stepCellRemainderSketch`.

<!-- proof-run: proofkit.Run(remainderSketchCases, stepCellRemainderSketch) -->

When `r = N % c` is greater than 0, the last `r` teeth are a second, shorter cell built where they
belong: a sketch `{gearLabel} Cell Remainder` on the gear's Axis Plane, made exactly as S11 makes the
Cell Sections sketch with `c` replaced by `r`: `r*n + 1` sections at stations `s0 + q*c*P + k*P/n`
for `k = 0 .. r*n`, the last at `s0 + N*P`, corners with their `z` kept, computing deferred from just
after naming to just after the last `isFixed`, and the checks that it reads fully constrained and
has `r*n + 1` profiles. Its first section is the body's last. The defaults have none (`r = 0`); 69
teeth in four-tooth cells would have one of one tooth. The proof lays the sections side by side, as
for S11.

## S17 `[GO]` A gear's remainder loft (only when r > 0)

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
      "span": "component.features.createPath(collection, False)"
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
      "span": "component.features.loftFeatures.add(loftInput)"
    }
  ],
  "citations": [
    {
      "first": 901,
      "last": 906,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 182,
      "last": 191,
      "path": "spec/screwgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L901–906; `spec/screwgear/fusion.md` L182–191.

Proof function `stepCellRemainderLoft`.

<!-- proof-run: proofkit3d.RunSolid(remainderLoftCases, stepCellRemainderLoft, assertCellRemainderLoft) -->

Loft the remainder's sections in station order by the recipe of S12: a new-body loft, one
`component.features.createPath(collection, False)` per section from its four lines, added with
`loftInput.loftSections.add(path)` in order, then `component.features.loftFeatures.add(loftInput)`.
Raise unless the feature leaves exactly one body and that body `isSolid` [PB-EMPTY-RESULT]. The proof
lofts the same sections ruled and holds the remainder's volume.

## S18 `[GO]` Join the remainder (only when r > 0)

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 901,
      "last": 906,
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

**From:** `spec/screwgear/instructions.md` L901–906; `spec/screwgear/fusion.md` L354–364.

Proof function `stepJoinRemainder`.

<!-- proof-run: proofkit3d.RunSolid(joinRemainderCases, stepJoinRemainder, assertJoinRemainder) -->

After the last round, join the remainder cell into the body with the join of S15: a
`JoinFeatureOperation` combine with the body as target and the remainder in an `ObjectCollection` as
the tool, `isKeepToolBodies = False`, raising unless the feature's `bodies.count` is exactly 1
[PB-EMPTY-RESULT]. Its first section is the body's last, so the join meets a shared cross-section like
every other. Then name the body `{gearLabel}` and store it as `self.gearBodies[index]`. The proof stitches
the remainder's walls to the ribbon's, at 45, 46 and 47 teeth and at 5 and 7; 69 teeth is past decad's
audit ceiling.

## S19 `[GO]` The Sleeve sketch

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
      "span": "sketch.modelToSketchSpace(C)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchCircles",
      "role": "required",
      "span": "circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)"
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
      "first": 918,
      "last": 953,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 306,
      "last": 328,
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

**From:** `spec/screwgear/instructions.md` L918–953; `spec/screwgear/fusion.md` L306–328; `.claude/skills/generate-gear/PLAYBOOK.md` L451–457; `.claude/skills/generate-gear/PLAYBOOK.md` L670–674.

Proof function `stepSleeveSketch`.

<!-- proof-run: proofkit.Run(sleeveSketchCases, stepSleeveSketch) -->

The cage is the **sleeve**: one tube about the frame's axis. Heights are along `n` from `C`; gear B's
axis is on the `+n` side and gear A's on the `-n` side.

[SCREW-F-SLEEVE]: `sketch = component.sketches.add(self.plane)` on the selected plane
[PB-USE-SELECTED-PLANE], named `Sleeve`; not deferred. With `centre` the point
`sketch.modelToSketchSpace(C)` given `z = 0` [PB-SKETCH-ZERO-Z], draw two circles, inner then outer,
each `circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(centre, radius)` with radius
`Ri/10` and then `Ro/10` (cm). After each circle set its own `circle.centerSketchPoint.isFixed = True`
[PB-CIRCLE-CENTER] — the two centres are separate points at the same place, both fixed, with no
coincident between them — and add a diameter dimension,
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)`, with its parameter's `value` set to
`2*Ri/10` or `2*Ro/10`. `textPoint` is on the circle, off the centre [PB-RADIAL-DIM]: the world point
`C + radius*e` mapped in with its `z` set to 0. Nothing else is in the sketch. Raise naming the
sketch unless `sketch.isFullyConstrained`.

It has two profiles, the inner disc and the ring. The ring is the profile whose `profileLoops.count`
is 2: iterate `sketch.profiles` and take the one profile with two loops, raising with the counts when
there is not exactly one [PB-EMPTY-RESULT]. `find_profile_by_curve_counts` cannot pick it, since it
treats a circle as a curve type that disqualifies a loop. At the defaults the ring is 565.5 mm².

## S20 `[GO]` Extrude the tube

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
      "span": "tubeFeature = component.features.extrudeFeatures.add(extrudeInput)"
    }
  ],
  "citations": [
    {
      "first": 949,
      "last": 953,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 936,
      "last": 940,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 324,
      "last": 328,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 747,
      "last": 750,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L949–953; `spec/screwgear/instructions.md` L936–940; `spec/screwgear/fusion.md` L324–328; `.claude/skills/generate-gear/PLAYBOOK.md` L747–750.

Proof function `stepSleeveExtrude`.

<!-- proof-run: proofkit3d.RunSolid(sleeveExtrudeCases, stepSleeveExtrude, assertSleeveExtrude) -->

`extrudeInput = component.features.extrudeFeatures.createInput(ring, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`extrudeInput.setSymmetricExtent(adsk.core.ValueInput.createByReal(cageRise/10), False)` — `False`
makes the value each side's length [PB-THROUGH-CUT], so the tube runs from `-cageRise` to `+cageRise`
— then `tubeFeature = component.features.extrudeFeatures.add(extrudeInput)`. Raise unless
`tubeFeature.bodies.count` is 1. That body is `self.cageBody` from here on: 37.5 mm tall at the
defaults, from radius 12 to 18 mm, with a flat ring at each end.

## S21 `[PROSE]` A bore's plane

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
      "span": "planeInput.setByDistanceOnPath(pathLine, adsk.core.ValueInput.createByReal(0))"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "planeInput.setByDistanceOnPath(pathLine, adsk.core.ValueInput.createByReal(0))"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "component.constructionPlanes",
      "role": "required",
      "span": "borePlane = component.constructionPlanes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 955,
      "last": 978,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 233,
      "last": 241,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 793,
      "last": 808,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L955–978; `spec/screwgear/fusion.md` L233–241; `.claude/skills/generate-gear/PLAYBOOK.md` L793–808.

S21, S22 and S23 run once per bore, in the order of S04: gear A `-R`, gear A `+R`, gear B `-R`, gear B
`+R`. A bore runs through the wall where its gear's axis crosses the circle of radius `cageRadius`. It is
the crest rectangle plus the clearance, `2*hw` by `2*ht` (15.9 by 4.65 mm at the defaults), turned at
every station to the ribbon's own angle. The assembly phase moves gear B's ribbon and not its bores.

The bore's plane is a construction plane on the bore's own line of S10 (`bore-` for a `-R` bore,
`bore+` for a `+R` bore), at fraction 0 [PB-CONSTRUCTION-PLANES], the line passed directly, never
wrapped in a path: `planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByDistanceOnPath(pathLine, adsk.core.ValueInput.createByReal(0))`,
`borePlane = component.constructionPlanes.add(planeInput)`, named `{gearLabel} Bore -R Plane` or
`{gearLabel} Bore +R Plane`. It stands square to the axis at the line's start, which is the cut's first
station `s1`: `-sOut` for a `-R` bore and `sIn` for a `+R` bore. Fusion put such a plane's origin on the
station to four decimals of a millimetre.

## S22 `[GO]` A bore's section sketch (the rectangle scheme)

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
      "span": "sketch = component.sketches.add(borePlane)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(world)"
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
      "span": "Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(pointO, pointCp)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(pointO, seedE)"
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
      "name": "addParallel",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addParallel(L1, K)"
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
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "sketch.geometricConstraints",
      "role": "required",
      "span": "sketch.geometricConstraints.addCoincident(K.endSketchPoint, L2)"
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
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addDistanceDimension(K.startSketchPoint, K.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, spineText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L1, lowText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(K, L3, highText)"
    },
    {
      "condition": null,
      "name": "addOffsetDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "sketch.sketchDimensions",
      "role": "required",
      "span": "sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)"
    },
    {
      "condition": null,
      "name": "find_profile_by_curve_counts",
      "owner": null,
      "reason": "utilities helper the framework provides for picking a profile by its curves",
      "receiver": null,
      "role": "inherited",
      "span": "profile = find_profile_by_curve_counts(sketch, lines=4)"
    }
  ],
  "citations": [
    {
      "first": 991,
      "last": 1027,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 612,
      "last": 632,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 233,
      "last": 241,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 386,
      "last": 416,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 653,
      "last": 669,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 624,
      "last": 631,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 239,
      "last": 251,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L991–1027; `spec/screwgear/instructions.md` L612–632; `spec/screwgear/fusion.md` L233–241; `spec/screwgear/fusion.md` L386–416; `.claude/skills/generate-gear/PLAYBOOK.md` L653–669; `.claude/skills/generate-gear/PLAYBOOK.md` L624–631; `.claude/skills/generate-gear/PLAYBOOK.md` L239–251.

Proof function `stepBoreSectionSketch`.

<!-- proof-run: proofkit.Run(boreSectionCases, stepBoreSectionSketch) -->

`sketch = component.sketches.add(borePlane)`, named `{gearLabel} Bore -R` or `{gearLabel} Bore +R`;
right after naming it set `sketch.isComputeDeferred = True` [SCREW-F-DEFER], [PB-SKETCH-DEFER].
Nothing is projected. Every point below is a world point of S09 at the station `s1`, mapped in with
`sketch.modelToSketchSpace(world)` and given `z = 0` before it is used [PB-SKETCH-ZERO-Z]; every seed is
the solved position [PB-SEED-NEAR]. With `theta = s1/Lambda + Phi_g`, `uB = -hw`, `uF = hw`, `hv = ht`
(cm):

Every length below is divided by 10 for cm.

1. **References.** `O` at `origin_g + s1*dir_g`, the point where the bore's line pierces the plane, and
   `Cp` at `origin_g + s1*dir_g + (A/2)*u_g`, each `sketch.sketchPoints.add(local)`.
2. **The zero of rotation.** `Ru = sketch.sketchCurves.sketchLines.addByTwoPoints(pointO, pointCp)`,
   construction (`Ru.isConstruction = True`).
3. **The spine.** `K` from `pointO` to a raw seed `E` at section coordinates `(uF, 0)`, drawn with
   `sketch.sketchCurves.sketchLines.addByTwoPoints(pointO, seedE)`, construction. Its end point is `E`.
4. **Fix the references.** After `Ru` and `K` exist, set `pointO.isFixed = True` and
   `pointCp.isFixed = True`, before any constraint or dimension.
5. **The rectangle.** Four solid lines sharing their corners [PB-SHARE-XOR-COINCIDENT]: `L1` from the
   seed at `(uB, -hv)` to the seed at `(uF, -hv)`, `L2` from `L1`'s end point to a seed at `(uF, hv)`,
   `L3` from `L2`'s end point to a seed at `(uB, hv)`, `L4` from `L3`'s end point to `L1`'s start point;
   each `sketch.sketchCurves.sketchLines.addByTwoPoints(start, end)` with an existing sketch point
   passed where the corner is shared. No coincident on a corner.
6. **Constraints.** `sketch.geometricConstraints.addParallel(L1, K)`,
   `sketch.geometricConstraints.addParallel(L3, K)`, `sketch.geometricConstraints.addCoincident(K.endSketchPoint, L2)`
   (`E` on `L2`), `sketch.geometricConstraints.addPerpendicular(L2, K)`, and
   `sketch.geometricConstraints.addParallel(L4, L2)` [PB-NO-OVERCONSTRAIN].
7. **Dimensions**, each with its parameter's `value` set to the value given, in cm or radians
   [PB-DIM-VALUE-SEMANTICS], [PB-DRIVING-DIM]:
   - the spine's length: `sketch.sketchDimensions.addDistanceDimension(K.startSketchPoint, K.endSketchPoint, adsk.fusion.DimensionOrientations.AlignedDimensionOrientation, spineText)`,
     value `hw`;
   - `L1`'s offset from `K`: `sketch.sketchDimensions.addOffsetDimension(K, L1, lowText)`, value `ht`;
   - `L3`'s offset from `K`: `sketch.sketchDimensions.addOffsetDimension(K, L3, highText)`, value `ht`;
   - `L4`'s offset from `L2`: `sketch.sketchDimensions.addOffsetDimension(L2, L4, widthText)`, value `2*hw`
     [PB-OFFSET-DIM];
   - **the angle** [PB-ANGULAR-DIM]: let `psi` be `theta` folded into `(-pi, pi]`. When
     `abs(sin(psi)) >= sqrt(1/2)`, dimension the angle between `Ru` and `K`,
     `sketch.sketchDimensions.addAngularDimension(Ru, K, angleText)`, value `abs(psi)`; the two rays are
     `Ru`'s direction from `O` (along `u_g`) and `K`'s from `O` (along `u(theta) = cos(theta)*u_g + sin(theta)*v_g`),
     meeting at `O`. Otherwise dimension the angle between `Ru` and the toothed side `L2`,
     `sketch.sketchDimensions.addAngularDimension(Ru, L2, angleText)`, with `phi = psi + pi/2` folded
     into `(-pi, pi]` and value `abs(phi)`; the rays are `Ru`'s direction (along `u_g`) and `L2`'s
     direction from its start to its end (along `v(theta) = -sin(theta)*u_g + cos(theta)*v_g`), from the
     point `M` where `L2`'s line meets `Ru`'s line: `M = O + (hw/cos(theta))*u_g`, `cos(theta)` being
     nonzero on this branch. Either way the value lies between 45° and 135°.

   The text points, as world points mapped in with `z = 0`: `spineText` at section coordinates
   `(hw/2, ht/4)`; `lowText` at `(hw/2, -ht/2)`; `highText` at `(hw/2, ht/2)`; `widthText` at
   `(0, -ht/4)`; `angleText` at the rays' meeting point plus `(hw/2)` times the unit bisector of the two
   ray directions, which lies inside the wedge the value measures.
8. Set `sketch.isComputeDeferred = False`. Raise naming the sketch unless `sketch.isFullyConstrained`.

Ten degrees of freedom (`E` and the four corners) against ten rows: five dimensions and five
constraints. In the proof the engine's `Offset` is the parallel and the offset dimension together, and
its angle is signed. The profile is the one loop of four solid lines:
`profile = find_profile_by_curve_counts(sketch, lines=4)` [PB-PROFILE-MATCH]; the construction lines
bound nothing.

## S23 `[GO]` Cut a bore with a twisted sweep, and check its sense

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
      "span": "path = component.features.createPath(pathLine, False)"
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
      "span": "sweepInput.twistAngle = adsk.core.ValueInput.createByReal(twist)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SweepFeatures",
      "reason": null,
      "receiver": "component.features.sweepFeatures",
      "role": "required",
      "span": "sweepFeature = component.features.sweepFeatures.add(sweepInput)"
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
      "first": 970,
      "last": 990,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1028,
      "last": 1063,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1272,
      "last": 1277,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 243,
      "last": 290,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 851,
      "last": 869,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 780,
      "last": 787,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L970–990; `spec/screwgear/instructions.md` L1028–1063; `spec/screwgear/instructions.md` L1272–1277; `spec/screwgear/fusion.md` L243–290; `.claude/skills/generate-gear/PLAYBOOK.md` L851–869; `.claude/skills/generate-gear/PLAYBOOK.md` L780–787.

Proof function `stepBoreSweepCut`.

<!-- proof-run: proofkit3d.RunSolid(boreCutCases, stepBoreSweepCut, assertBoreSweepCut) -->

[SCREW-F-TWISTED-SLOT], [PB-SWEEP-TWIST]: `path = component.features.createPath(pathLine, False)` on
the bore's own line [PB-PATH-FROM-SKETCH], then
`sweepInput = component.features.sweepFeatures.createInput(profile, path, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
set `sweepInput.twistAngle = adsk.core.ValueInput.createByReal(twist)` with
`twist = +(sOut - sIn)/Lambda` in radians (82.31° at the defaults), and
`sweepInput.participantBodies = [self.cageBody]`; set nothing else — not `orientation`, not
`solidTwistAxis`, no rail. Then `sweepFeature = component.features.sweepFeatures.add(sweepInput)`. The
sign is positive: the path runs along `+dir_g`, the section's angle grows with `s`, and a positive
`twistAngle` turns the profile that way. The ribbons are not participants, so the cut leaves them
whole.

Raise naming the bore unless `sweepFeature.bodies.count` is 1 [PB-EMPTY-RESULT]; that body is
`self.cageBody` from then on.

**The sense check** [SCREW-F-SWEEP-CHECK], [PB-SELF-DIAGNOSING]: at the crossing station
`sc = -cageRadius` for a `-R` bore and `+cageRadius` for a `+R` bore, with
`u(sc) = cos(theta)*u_g + sin(theta)*v_g`, `theta = sc/Lambda + Phi_g`, the two probes are
`origin_g + sc*dir_g + (W/2 + clearance/2)*u(sc)` and `origin_g + sc*dir_g - (W/2 + clearance/2)*u(sc)`,
every length divided by 10 for cm. Both must read `self.cageBody.pointContainment(probe)` equal to
`adsk.fusion.PointContainment.PointOutsidePointContainment`; raise naming the bore, the probe and the
containment read otherwise. At the defaults the probes stand 16.86 mm (`-R`) and 16.31 mm (`+R`) from
the frame's axis, inside the wall; under the wrong sense they would sit in the wall. There is no other
sense check.

What the proof builds: decad has no twisted sweep, so each bore is a ruled loft through
`ceil(twist/step) + 1` sections over `[s1, s1 + (sOut - sIn)]`, `step` the smaller of 5° and
`2*acos(1 - 0.04*clearance/cc)` (18 sections at the defaults, 32 at a 0.05 mm clearance), the first
section drawn by the rectangle scheme. A second decad cut on a faceted result is refused, so the proof
cuts the union of the four stand-ins from the tube in one cut and records as unrepresentable the inputs
where decad refuses that cut.

## S24 `[PROSE]` The Window Plane (only when a window has room)

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
      "first": 1241,
      "last": 1253,
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

**From:** `spec/screwgear/instructions.md` L1241–1253; `spec/screwgear/fusion.md` L329–336.

Both windows lie on one plane through the frame's axis square to `d` [PB-CONSTRUCTION-PLANES]; it is
not made when neither window has room (S05). `planeInput = component.constructionPlanes.createInput()`,
then:

- for windows facing plus or minus `k` (crossing angle at most 90°),
  `planeInput.setByAngle(self.anchorLine, adsk.core.ValueInput.createByString('90 deg'), self.plane)`:
  the plane through the Anchor Line square to the selected plane, which holds `C`, `e` and `n`;
- for windows facing plus or minus `e`,
  `planeInput.setByDistanceOnPath(self.anchorLine, adsk.core.ValueInput.createByReal(0.5))`: the plane
  square to the Anchor Line through its midpoint, `C`.

Then `windowPlane = component.constructionPlanes.add(planeInput)`, named `Window Plane`. The proof draws
each window sketch in this plane's coordinates `(t, z)` and asserts the plane holds `C` and `n` and
stands square to `d`.

## S25 `[GO]` A window's sketch

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
      "span": "sketch.sketchPoints.add(local)"
    },
    {
      "condition": null,
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "local = sketch.modelToSketchSpace(world)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "sketch.sketchCurves.sketchLines",
      "role": "required",
      "span": "sketch.sketchCurves.sketchLines.addByTwoPoints(cornerA, cornerB)"
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
      "first": 1246,
      "last": 1253,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 1210,
      "last": 1218,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 337,
      "last": 340,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 494,
      "last": 500,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1246–1253; `spec/screwgear/instructions.md` L1210–1218; `spec/screwgear/fusion.md` L337–340; `.claude/skills/generate-gear/PLAYBOOK.md` L494–500.

Proof function `stepWindowSketch`.

<!-- proof-run: proofkit.Run(windowSketchCases, stepWindowSketch) -->

S25 and S26 run once per window in `self.windows`, the one facing the first direction of S05 first.
`sketch = component.sketches.add(windowPlane)`, named `Window {d}` (`Window +k`, `Window -k`,
`Window +e` or `Window -e`); not deferred. One reference point per hexagon corner of S05, at the world
point `C + (t/10)*across + (z/10)*n` (cm), each `sketch.sketchPoints.add(local)` with
`local = sketch.modelToSketchSpace(world)` and `local.z = 0` [PB-SKETCH-ZERO-Z]; then one solid line from
each corner to the next and from the last back to the first,
`sketch.sketchCurves.sketchLines.addByTwoPoints(cornerA, cornerB)`, sharing the points
[PB-SHARE-XOR-COINCIDENT]; then every point `isFixed = True`. Nothing else is in the sketch. Raise
naming the sketch unless `sketch.isFullyConstrained`. Its profile is its one loop: raise unless
`sketch.profiles.count` is 1, then `profile = sketch.profiles.item(0)` [PB-SINGLE-PROFILE]. The two
windows are two sketches because on the shared plane their hexagons cross.

## S26 `[GO]` Cut a window, and check it

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
      "name": "modelToSketchSpace",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.modelToSketchSpace(C + d)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "extrudeInput = component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)"
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
      "receiver": "component.features.extrudeFeatures",
      "role": "required",
      "span": "windowFeature = component.features.extrudeFeatures.add(extrudeInput)"
    }
  ],
  "citations": [
    {
      "first": 1255,
      "last": 1277,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 340,
      "last": 352,
      "path": "spec/screwgear/fusion.md"
    },
    {
      "first": 747,
      "last": 750,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 780,
      "last": 787,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1255–1277; `spec/screwgear/fusion.md` L340–352; `.claude/skills/generate-gear/PLAYBOOK.md` L747–750; `.claude/skills/generate-gear/PLAYBOOK.md` L780–787.

Proof function `stepWindowCut`.

<!-- proof-run: proofkit3d.RunSolid(windowCutCases, stepWindowCut, assertWindowCut) -->

**Before the cut**, the probe: with `(tc, zc)` the average of the hexagon's corners (mm),
`a0 = sqrt(max(0, Ri^2 - tc^2))` and `a1 = sqrt(max(0, Ro^2 - tc^2))`, the probe is
`C + (tc/10)*across + (zc/10)*n + ((a0 + a1)/20)*d` (cm), the middle of the wall where the window goes.
`self.cageBody.pointContainment(probe)` must be
`adsk.fusion.PointContainment.PointInsidePointContainment` [PB-SELF-DIAGNOSING]; raise naming the window
and the containment read otherwise.

**The cut**: `direction` is `adsk.fusion.ExtentDirections.PositiveExtentDirection` when
`sketch.modelToSketchSpace(C + d)` has a positive `z` (that is, when `d` points to the sketch's
positive side), and `adsk.fusion.ExtentDirections.NegativeExtentDirection` otherwise.
`extrudeInput = component.features.extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`extrudeInput.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(adsk.core.ValueInput.createByReal((Ro + 1)/10)), direction)`,
`extrudeInput.participantBodies = [self.cageBody]` so the ribbons in the hollow are left whole
[PB-THROUGH-CUT], then `windowFeature = component.features.extrudeFeatures.add(extrudeInput)`. The cut
runs one way only: the same hexagon on the far side of the axis is the other window's side of the wall.

**After the cut**: `windowFeature.bodies.count` must be 1 [PB-EMPTY-RESULT], that body is
`self.cageBody` from then on, and the same probe must now read
`adsk.fusion.PointContainment.PointOutsidePointContainment`; raise naming the window and what was read
otherwise. A cut extruded the wrong way leaves the probe inside. What remains at the defaults is one
piece of 16,503 mm³.

What the proof builds: decad refuses a second cut on a faceted result, so each window is cut from the
uncut tube, one window per case; the proof holds the cut and the removed material to the tube's volume,
the prism to `d`'s side of the plane, and the probe inside both the tube's wall and the prism.

## S27 `[PROSE]` Relocate the bodies and clean up

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
      "reason": "solids helper the framework provides; the spec forbids re-implementing it",
      "receiver": "solids",
      "role": "inherited",
      "span": "solids.hide_construction_geometry(self.designOcc.component)"
    }
  ],
  "citations": [
    {
      "first": 1279,
      "last": 1285,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 407,
      "last": 409,
      "path": "spec/screwgear/instructions.md"
    },
    {
      "first": 941,
      "last": 949,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    },
    {
      "first": 677,
      "last": 689,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/screwgear/instructions.md` L1279–1285; `spec/screwgear/instructions.md` L407–409; `.claude/skills/generate-gear/PLAYBOOK.md` L941–949; `.claude/skills/generate-gear/PLAYBOOK.md` L677–689.

Name the cage body `Cage`. Then move the bodies into their sub-components with `moveToComponent`, which
preserves world position and needs no activation [PB-NO-CROSS-SIBLING]:
`body.moveToComponent(occurrence)` for gear A's body into the `Gear A` occurrence, then gear B's body
into `Gear B`, then the cage body into `Cage`, in that order, keeping the body each call returns.
These are three timeline entries.

Then `solids.hide_construction_geometry(self.designOcc.component)` [PB-TREE-CLEANUP]: the `Design`
sub-component is where every sketch and construction plane of this build was made, and the helper walks
it and everything under it. Do not re-implement it, and add no display settling of your own
[PB-SETTLE-DISPLAY].

At the defaults the build makes 12 sketches, 7 construction planes and 42 features: 61 timeline entries,
66 with the five component creations. A remainder cell adds one sketch, one loft and one join per gear;
a window with no room takes one sketch and one feature off, and when neither has room the Window Plane
is not made either.
