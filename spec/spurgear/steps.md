The proof for this step list is `proof/spurgear/geometry_test.go`, `proof/spurgear/sketches_test.go`,
`proof/spurgear/solids_test.go`, `proof/spurgear/finish_test.go` and the generated
`proof/spurgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `4a0bca4ab7b0fd275571f3408de06d013738ca53` |
| `spec/spurgear/fusion.md` | `7cd4e5b0fa38dcd39cbd1b5bad1cf8489e2bc2ae` |
| `spec/helicalgear/fusion.md` | `c636a3b7bb6fd13cd8a4153fe63a123137d32262` |
| `spec/spurgear/contract.json` | `fc62129c377504ba778823007f52a956767c46f2` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `b2a724566f469283211a645d83fbefe00f71ec3a` |

## Compilation contract

```json
{
  "_comment": "Machine-readable mirror of the spec's contract sections, checked by .claude/skills/generate-gear/check_contract.py. The spec prose (instructions.md/fusion.md) is authoritative — a mismatch between this file and the spec is a spec bug; fix both together. Methods listed are the pinned public/hook surface (underscore-private decomposition stays free). module_constants pins the Python identifiers dependents import (helicalgear/herringbonegear) AND their exact string values. source_guards pins the two constraint recipes the spec chose over an alternative that also solves — reverting either renames nothing, so nothing else here would see it.",
  "classes": {
    "SpurGearCommandInputsConfigurator": {
      "bases": [],
      "methods": [
        "configure"
      ]
    },
    "SpurGearGenerationContext": {
      "bases": [
        "GenerationContext"
      ],
      "ctx_fields": [
        "plane",
        "anchorPoint",
        "extrusionEndPlane",
        "gearProfileSketch",
        "toothBody",
        "gearBody",
        "centerAxis",
        "extrusionExtent",
        "toothProfileIsEmbedded"
      ],
      "methods": [
        "__init__"
      ]
    },
    "SpurGearGenerator": {
      "bases": [
        "Generator"
      ],
      "methods": [
        "__init__",
        "prefixBase",
        "newContext",
        "addExtraPrimaryParameters",
        "filletHelixFactorExpression",
        "generateName",
        "processInputs",
        "registerDerivedParameters",
        "generate",
        "prepareTools",
        "buildMainGearBody",
        "buildSketches",
        "buildTooth",
        "chamferTeeth",
        "buildBody",
        "patternTeeth",
        "createFillets",
        "buildBore",
        "cleanup"
      ]
    },
    "SpurGearInvoluteToothDesignGenerator": {
      "bases": [],
      "methods": [
        "__init__",
        "getParameter",
        "getParameterValue",
        "calculateInvolutePoint",
        "draw",
        "drawCircles",
        "drawTooth",
        "drawBore"
      ]
    }
  },
  "module": "lib/geargen/spurgear.py",
  "module_constants": {
    "INPUT_ID_ANCHOR_POINT": "anchorPoint",
    "INPUT_ID_BORE_DIAMETER": "boreDiameter",
    "INPUT_ID_CHAMFER_TOOTH": "chamferTooth",
    "INPUT_ID_MODULE": "module",
    "INPUT_ID_PARENT": "parentComponent",
    "INPUT_ID_PLANE": "plane",
    "INPUT_ID_PRESSURE_ANGLE": "pressureAngle",
    "INPUT_ID_SKETCH_ONLY": "sketchOnly",
    "INPUT_ID_THICKNESS": "thickness",
    "INPUT_ID_TOOTH_NUMBER": "toothNumber",
    "PARAM_BASE_DIAMETER": "BaseCircleDiameter",
    "PARAM_BASE_RADIUS": "BaseCircleRadius",
    "PARAM_BORE_DIAMETER": "BoreDiameter",
    "PARAM_CHAMFER_TOOTH": "ChamferTooth",
    "PARAM_FILLET_CLEARANCE": "FilletClearance",
    "PARAM_FILLET_RADIUS": "FilletRadius",
    "PARAM_INVOLUTE_STEPS": "InvoluteSteps",
    "PARAM_MODULE": "Module",
    "PARAM_PITCH_DIAMETER": "PitchCircleDiameter",
    "PARAM_PITCH_RADIUS": "PitchCircleRadius",
    "PARAM_PRESSURE_ANGLE": "PressureAngle",
    "PARAM_ROOT_DIAMETER": "RootCircleDiameter",
    "PARAM_ROOT_RADIUS": "RootCircleRadius",
    "PARAM_SKETCH_ONLY": "SketchOnly",
    "PARAM_THICKNESS": "Thickness",
    "PARAM_TIP_DIAMETER": "TipCircleDiameter",
    "PARAM_TIP_RADIUS": "TipCircleRadius",
    "PARAM_TOOTH_NUMBER": "ToothNumber",
    "PARAM_TOOTH_SPACE_ANGLE": "ToothSpaceAngleAtRoot",
    "PARAM_TOOTH_SPACE_ARC": "ToothSpaceArcAtRoot"
  },
  "source_guards": [
    {
      "banned": [
        "addCoincident\\("
      ],
      "file": "lib/geargen/spurgear.py",
      "in_function": "_drawFlankToRoot",
      "required": [
        "addDistanceDimension\\(",
        "DimensionOrientations\\.HorizontalDimensionOrientation",
        "DimensionOrientations\\.VerticalDimensionOrientation",
        "\\.parameter\\.value\\s*="
      ],
      "why": "[SPUR-F-FLANK-ROOT]: the root endpoint is pinned by exactly two signed dimensions from the local origin. Constraining it onto the root circle, or the local origin onto the stub line, also reaches DOF 0 but leaves the far root-circle intersection equally valid, and the stub runs across the gear."
    },
    {
      "banned": [
        "originPoint"
      ],
      "file": "lib/geargen/spurgear.py",
      "in_function": "buildBore",
      "required": [
        "drawBore\\(",
        "addCoincident\\(",
        "\\.anchorPoint"
      ],
      "why": "[SPUR-F-LOCAL-ORIGIN] and [PB-CIRCLE-CENTER]: the Bore Profile sketch's stray local origin is grounded on the anchor projected into that sketch, never on the sketch's own originPoint, which pins it to the plane instead of to the gear."
    },
    {
      "banned": [
        "NewPointOnCircle\\(re,",
        "NewPointOnLine\\(origin,"
      ],
      "file": "spec/spurgear/sketch/main.go",
      "required": [
        "sketch\\.NewHorizontalDistance\\(origin, re, rx\\)",
        "sketch\\.NewVerticalDistance\\(origin, re, ry\\)"
      ],
      "why": "The sketch bench is where [SPUR-F-FLANK-ROOT] was proven; it has to keep proving the recipe the generator is required to use."
    },
    {
      "banned": [
        "root-end-on-circle",
        "origin-on-line"
      ],
      "file": "spec/spurgear/sketch/README.md",
      "required": [
        "flank-to-root lines: root endpoint pinned by signed",
        "NewHorizontalDistance\\(origin, rootEnd, dx\\)",
        "NewVerticalDistance\\(origin, rootEnd, dy\\)"
      ],
      "why": "The bench README is the constraint-by-constraint map a reader trusts; a stale row there sends the next generation back to the rejected recipe."
    }
  ]
}
```

## 0a `[PROSE]` Module surface: imports, constants, the four classes, context fields, method contract

The module is `lib/geargen/spurgear.py`. It imports exactly these names and nothing by star:

```python
import math
from typing import cast
import adsk.core, adsk.fusion
from ...lib import fusion360utils as futil
from .misc import to_cm, get_design
from .base import Generator, GenerationContext, get_value, get_boolean, get_selection
from .utilities import get_normal, find_profile_by_curve_counts
```

`futil` is for `futil.log(...)` step-progress lines only [PB-LOGGING]; it is optional, so the
module may leave it out.

Module-level constants, every one exported with exactly this identifier and string value
[SPUR-EXPORTED-CONSTANTS]:

| constant | value | | constant | value |
|---|---|---|---|---|
| `INPUT_ID_PARENT` | `'parentComponent'` | | `PARAM_PITCH_DIAMETER` | `'PitchCircleDiameter'` |
| `INPUT_ID_PLANE` | `'plane'` | | `PARAM_PITCH_RADIUS` | `'PitchCircleRadius'` |
| `INPUT_ID_ANCHOR_POINT` | `'anchorPoint'` | | `PARAM_BASE_DIAMETER` | `'BaseCircleDiameter'` |
| `INPUT_ID_MODULE` | `'module'` | | `PARAM_BASE_RADIUS` | `'BaseCircleRadius'` |
| `INPUT_ID_TOOTH_NUMBER` | `'toothNumber'` | | `PARAM_ROOT_DIAMETER` | `'RootCircleDiameter'` |
| `INPUT_ID_PRESSURE_ANGLE` | `'pressureAngle'` | | `PARAM_ROOT_RADIUS` | `'RootCircleRadius'` |
| `INPUT_ID_BORE_DIAMETER` | `'boreDiameter'` | | `PARAM_TIP_DIAMETER` | `'TipCircleDiameter'` |
| `INPUT_ID_THICKNESS` | `'thickness'` | | `PARAM_TIP_RADIUS` | `'TipCircleRadius'` |
| `INPUT_ID_CHAMFER_TOOTH` | `'chamferTooth'` | | `PARAM_INVOLUTE_STEPS` | `'InvoluteSteps'` |
| `INPUT_ID_SKETCH_ONLY` | `'sketchOnly'` | | `PARAM_TOOTH_SPACE_ANGLE` | `'ToothSpaceAngleAtRoot'` |
| `PARAM_MODULE` | `'Module'` | | `PARAM_TOOTH_SPACE_ARC` | `'ToothSpaceArcAtRoot'` |
| `PARAM_TOOTH_NUMBER` | `'ToothNumber'` | | `PARAM_FILLET_CLEARANCE` | `'FilletClearance'` |
| `PARAM_PRESSURE_ANGLE` | `'PressureAngle'` | | `PARAM_FILLET_RADIUS` | `'FilletRadius'` |
| `PARAM_BORE_DIAMETER` | `'BoreDiameter'` | | | |
| `PARAM_THICKNESS` | `'Thickness'` | | | |
| `PARAM_CHAMFER_TOOTH` | `'ChamferTooth'` | | | |
| `PARAM_SKETCH_ONLY` | `'SketchOnly'` | | | |

Exactly four classes, these names, these bases, these methods (private helpers are free):

1. `SpurGearCommandInputsConfigurator` — no base; `@classmethod def configure(cls, cmd)` (step 0b).
2. `SpurGearGenerationContext(GenerationContext)` — `__init__` declares, each `cast(None)`
   initialised except the last which starts `False`: `plane`, `anchorPoint`, `extrusionEndPlane`,
   `gearProfileSketch`, `toothBody`, `gearBody`, `centerAxis`, `extrusionExtent`,
   `toothProfileIsEmbedded`. Meanings: `ctx.plane` the normalised `ConstructionPlane`;
   `ctx.anchorPoint` the `SketchPoint` that is the Tools-sketch projection of the user's anchor;
   `ctx.extrusionEndPlane` the offset plane the extrudes end on; `ctx.gearProfileSketch`;
   `ctx.toothBody` the one extruded tooth; `ctx.gearBody` the body the teeth join into;
   `ctx.centerAxis` the `Gear Center` axis; `ctx.extrusionExtent` the far end-cap face;
   `ctx.toothProfileIsEmbedded` true iff the base circle sits inside the root circle.
3. `SpurGearInvoluteToothDesignGenerator` — no base; constructor `(sketch, parent, angle=0)`;
   methods `getParameter(name)`, `getParameterValue(name) -> float`,
   `calculateInvolutePoint(baseRadius, intersectionRadius)`, `draw(anchorPoint, angle=0)`,
   `drawCircles()`, `drawTooth(angle)`, `drawBore(anchorPoint, diameter)`. The constructor stores
   `self.toothAngle = angle` (never read by `drawTooth`) and adds the movable local origin
   `self.anchorPoint = sketch.sketchPoints.add(adsk.core.Point3D.create(0, 0, 0))`
   [SPUR-F-LOCAL-ORIGIN]. `getParameter(name)` returns `self.parent.getParameter(name)`;
   `getParameterValue(name)` returns its `.value`. Inside `drawCircles`, `drawTooth` and `draw`
   (and their helpers) only these parameter keys may be read, because the bevel gear serves them
   through `VirtualSpurProxy` and nothing else [PB-PRECOMPUTED-MODE]: `Module`, `ToothNumber`,
   `PressureAngle`, `PitchCircleDiameter`, `PitchCircleRadius`, `BaseCircleDiameter`,
   `BaseCircleRadius`, `RootCircleDiameter`, `RootCircleRadius`, `TipCircleDiameter`,
   `TipCircleRadius`, `InvoluteSteps`. `drawTooth` writes `self.parent._lastToothEmbedded`.
   `calculateInvolutePoint(baseRadius, intersectionRadius)` returns `None` when
   `intersectionRadius < baseRadius`, else with `alpha = math.acos(baseRadius / intersectionRadius)`
   and `t = math.tan(alpha)` (the parameter is `tan(alpha)`, never `tan(alpha) - alpha`):
   `x = baseRadius * (math.cos(t) + t * math.sin(t))`, `y = baseRadius * (math.sin(t) - t * math.cos(t))`.
4. `SpurGearGenerator(Generator)` — methods `__init__`, `prefixBase() -> str` returning
   `'SpurGear'`, `newContext() -> SpurGearGenerationContext`, `addExtraPrimaryParameters(inputs)`
   (a no-op on this base), `filletHelixFactorExpression() -> str` returning `'1'`,
   `generateName() -> str`, `processInputs(inputs)`, `registerDerivedParameters()`,
   `generate(inputs)`, `prepareTools(ctx)`, `buildMainGearBody(ctx)`, `buildSketches(ctx)`,
   `buildTooth(ctx)`, `buildBody(ctx)`, `patternTeeth(ctx)`, `createFillets(ctx)`,
   `buildBore(ctx)`, `chamferTeeth(ctx)`, `cleanup(ctx)`. `__init__(self, design)` calls the base
   and pre-initialises `self._lastToothEmbedded = False`, `self.toolsSketch = None`,
   `self.boreSketch = None`. The five return annotations above (`prefixBase`, `generateName`,
   `filletHelixFactorExpression`, `newContext`, and the tooth generator's `getParameterValue`)
   are required: subclasses narrow on them.

The call graph is fixed; only method bodies vary:

```
generate(inputs)
  -> processInputs(inputs)                       (step 0c)
  -> component = self.getComponent(); component.name = self.generateName()
  -> normalise self.plane to a ConstructionPlane (step 1)
  -> ctx = self.newContext(); ctx.plane = self.plane
  -> prepareTools(ctx)                           (steps 2a, 2b)
  -> buildMainGearBody(ctx)
       -> buildSketches(ctx)                     (step 3)
       -> if SketchOnly: show the Gear Profile sketch and return (step 6)
       -> buildTooth(ctx)                        (step 7)
       -> buildBody(ctx)                         (step 9)
       -> patternTeeth(ctx)                      (steps 10a, 10b)
       -> createFillets(ctx)                     (step 11)
  -> buildBore(ctx)                              (step 12)
  -> chamferTeeth(ctx)                           (step 13)
  -> cleanup(ctx)                                (step 0d), unconditionally, last
```

`configure`, `generate` and `__init__` are entry points the framework calls, `prefixBase` is
called by the base class, `futil.log` is optional, and `VirtualSpurProxy` belongs to the bevel
gear, so they are named here without being calls this module makes:

<!-- check-step-calls: ignore configure generate __init__ prefixBase log VirtualSpurProxy -->

**From:** `spec/spurgear/instructions.md` L9–33, `spec/spurgear/instructions.md` L108–124, `spec/spurgear/instructions.md` L226–252, `spec/spurgear/instructions.md` L327–479, `spec/spurgear/fusion.md` L26–31, `spec/spurgear/fusion.md` L210–215, `spec/spurgear/contract.json` L1–93.

## 0b `[PROSE]` Dialog inputs: `SpurGearCommandInputsConfigurator.configure`

`configure(cls, cmd)` adds the inputs to `cmd.commandInputs` in exactly this order, never
regrouped by type; the two selections come first, Parent Component last, so Target Plane owns the
initial focus [PB-AUTOFOCUS-FIRST]. Filters are the named constants [PB-SELECTION-FILTER-ENUM];
every selection takes exactly one entity [PB-SELECTION-DECL]. Every `createByReal` default is in
Fusion's internal units, cm and radians, whatever the display unit [PB-DIALOG-DEFAULT-UNITS].

| # | id | call |
|---|---|---|
| 1 | `plane` | `cmd.commandInputs.addSelectionInput('plane', 'Target Plane', 'Select the plane to build the gear on')`; then on the result `addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPlanes)`, `addSelectionFilter(adsk.core.SelectionCommandInput.PlanarFaces)`, `setSelectionLimits(1, 1)` |
| 2 | `anchorPoint` | `cmd.commandInputs.addSelectionInput('anchorPoint', 'Anchor Point', 'Select the point the gear is centered on')`; `addSelectionFilter(adsk.core.SelectionCommandInput.ConstructionPoints)`, `addSelectionFilter(adsk.core.SelectionCommandInput.SketchPoints)`, `setSelectionLimits(1, 1)` |
| 3 | `module` | `cmd.commandInputs.addValueInput('module', 'Module', '', adsk.core.ValueInput.createByReal(1))` |
| 4 | `toothNumber` | `cmd.commandInputs.addValueInput('toothNumber', 'Tooth Number', '', adsk.core.ValueInput.createByReal(17))` |
| 5 | `pressureAngle` | `cmd.commandInputs.addValueInput('pressureAngle', 'Pressure Angle', 'deg', adsk.core.ValueInput.createByReal(math.radians(20)))` |
| 6 | `boreDiameter` | `cmd.commandInputs.addStringValueInput('boreDiameter', 'Bore Diameter', '0 mm')` |
| 7 | `thickness` | `cmd.commandInputs.addValueInput('thickness', 'Thickness', 'mm', adsk.core.ValueInput.createByReal(to_cm(10)))` |
| 8 | `chamferTooth` | `cmd.commandInputs.addValueInput('chamferTooth', 'Apply chamfer to teeth', 'mm', adsk.core.ValueInput.createByReal(0))` |
| 9 | `sketchOnly` | `cmd.commandInputs.addBoolValueInput('sketchOnly', 'Generate sketches, but do not build body', True, '', False)` |
| 10 | `parentComponent` | `cmd.commandInputs.addSelectionInput('parentComponent', 'Parent Component', 'Select the component to build the gear in')`; `addSelectionFilter(adsk.core.SelectionCommandInput.Occurrences)`, `addSelectionFilter(adsk.core.SelectionCommandInput.RootComponents)`, `setSelectionLimits(1, 1)`, then pre-select the root with `addSelection(get_design().rootComponent)` |

The ids are the `INPUT_ID_*` constants of step 0a; write them through the constants. The third
argument of each `addSelectionInput` is the command prompt shown beside the cursor, not the label;
the three strings above are reproduced surface. A subclass adds its own input by subclassing this
configurator and appending after `super().configure(cmd)`, so its input lands after Parent
Component [SPUR-SUBCLASS-INPUT]; that `super().configure(cmd)` is the subclass's call, not this
module's. The optional `validate_inputs` hook [PB-VALIDATE-INPUTS] is not used by this gear:

<!-- check-step-calls: ignore validate_inputs configure -->

**From:** `spec/spurgear/instructions.md` L35–62, `spec/spurgear/instructions.md` L90–106, `spec/spurgear/instructions.md` L167–219, `spec/spurgear/instructions.md` L245–252.

## 0c `[PROSE]` Read the inputs and register the parameters: `processInputs`

Order is load-bearing [PB-SELECTION-STASH]: the three selections are read and stashed on `self`
before anything creates the occurrence, because the first `addParameter` (through
`parameterName`, through the base's `getOccurrence`, which this module never calls itself) shifts
Fusion's active component and selection inputs holding entities from another component can drop.
Each input is read with the helper its declaration fixes [PB-INPUT-READ]; `get_value` already
returns a `ValueInput` to hand straight to `addParameter` [PB-GET-VALUE-CONTRACT].

<!-- check-step-calls: ignore getOccurrence -->

1. `get_selection(inputs, INPUT_ID_PARENT)` — exactly one entity; an `Occurrence` resolves to
   its `.component`, a `Component` is taken as is; anything else, or another count, raises. Store
   `self.parentComponent`.
2. `get_selection(inputs, INPUT_ID_PLANE)` → `self.plane`; `get_selection(inputs, INPUT_ID_ANCHOR_POINT)`
   → `self.anchorPoint`, each exactly one entity.
3. The input-sourced parameters, in this order, each through
   `self.addParameter(name, value, units, comment)`:

| `name` | read with | `units` | `comment` |
|---|---|---|---|
| `PARAM_MODULE` | `get_value(inputs, INPUT_ID_MODULE, '')` | `''` | `Module of the gear` |
| `PARAM_TOOTH_NUMBER` | `get_value(inputs, INPUT_ID_TOOTH_NUMBER, '')` | `''` | `Number of teeth` |
| `PARAM_PRESSURE_ANGLE` | `get_value(inputs, INPUT_ID_PRESSURE_ANGLE, 'deg')` | `'rad'` | `Pressure angle` |
| `PARAM_BORE_DIAMETER` | `get_value(inputs, INPUT_ID_BORE_DIAMETER, 'mm')` | `'mm'` | `Bore diameter` |
| `PARAM_THICKNESS` | `get_value(inputs, INPUT_ID_THICKNESS, 'mm')` | `'mm'` | `Thickness of the gear` |
| `PARAM_CHAMFER_TOOTH` | `get_value(inputs, INPUT_ID_CHAMFER_TOOTH, 'mm')` | `'mm'` | `Chamfer distance applied to the teeth` |
| `PARAM_SKETCH_ONLY` | `get_boolean(inputs, INPUT_ID_SKETCH_ONLY)`, then `adsk.core.ValueInput.createByReal(1)` when true, `adsk.core.ValueInput.createByReal(0)` when false | `''` | `Generate sketches only` |

   `Module` is unitless on purpose, so `generateName` renders `M=1` and the `mm` expressions
   below read it as a factor.
4. `self.addExtraPrimaryParameters(inputs)` — the no-op hook subclasses fill [SPUR-EXTRA-PARAMS].
5. `self.registerDerivedParameters()` — the derived parameters, in this order, as live
   expressions built from `self.parameterName(...)` of the parameters they reference and passed as
   `adsk.core.ValueInput.createByString(expression)`, except the three marked pre-computed, which
   are passed as `adsk.core.ValueInput.createByReal(number)`:

| `name` | `units` | expression or value | `comment` |
|---|---|---|---|
| `PARAM_PITCH_DIAMETER` | `'mm'` | `<Module> * <ToothNumber>` | `Pitch circle diameter` |
| `PARAM_PITCH_RADIUS` | `'mm'` | `<PitchCircleDiameter> / 2` | `Pitch circle radius` |
| `PARAM_BASE_DIAMETER` | `'mm'` | `<PitchCircleDiameter> * cos(<PressureAngle>)` | `Base circle diameter` |
| `PARAM_BASE_RADIUS` | `'mm'` | `<BaseCircleDiameter> / 2` | `Base circle radius` |
| `PARAM_ROOT_DIAMETER` | `'mm'` | `<PitchCircleDiameter> - 2.5 * <Module>` | `Root circle diameter` |
| `PARAM_ROOT_RADIUS` | `'mm'` | `<RootCircleDiameter> / 2` | `Root circle radius` |
| `PARAM_TIP_DIAMETER` | `'mm'` | `<PitchCircleDiameter> + 2 * <Module>` | `Tip circle diameter` |
| `PARAM_TIP_RADIUS` | `'mm'` | `<TipCircleDiameter> / 2` | `Tip circle radius` |
| `PARAM_INVOLUTE_STEPS` | `''` | pre-computed `15` | `Number of points sampled along each involute flank` |
| `PARAM_TOOTH_SPACE_ANGLE` | `''` | pre-computed `math.pi / N - 2 * (math.tan(pa) - pa)` with `N = self.getParameter(PARAM_TOOTH_NUMBER).value` and `pa = self.getParameter(PARAM_PRESSURE_ANGLE).value` (radians) | `Angular width of the tooth space at the root circle` |
| `PARAM_TOOTH_SPACE_ARC` | `'mm'` | `<RootCircleRadius> * <ToothSpaceAngleAtRoot>` | `Arc length of the tooth space at the root circle` |
| `PARAM_FILLET_CLEARANCE` | `''` | pre-computed `0.9` | `Clearance factor applied to the root fillet radius` |
| `PARAM_FILLET_RADIUS` | `'mm'` | `(<ToothSpaceArcAtRoot> / 2) * <FilletClearance> * ` + `self.filletHelixFactorExpression()` | `Radius of the root fillets` |

   `<X>` stands for `self.parameterName(PARAM_X)`, the prefixed name. `ToothSpaceAngleAtRoot`
   is pre-computed because Fusion's expression engine refuses `tan()` minus a radian value, and
   it is registered unitless, not `'rad'`, so the `mm` product below it is a length. The spur
   base's `filletHelixFactorExpression()` returns `'1'`, so the last factor is literally `* 1`.
6. `generateName()` returns
   `'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)`
   where the three are `self.getParameter(PARAM_MODULE)`, `self.getParameter(PARAM_TOOTH_NUMBER)`,
   `self.getParameter(PARAM_THICKNESS)`; `.expression`, not `.value`, so `Thickness=10 mm`.

Sketch dimensions and feature inputs in every later step take the current numeric `.value` of a
parameter, never a live expression [PB-NUMERIC-SNAPSHOT] [SPUR-F-SNAPSHOT].

**From:** `spec/spurgear/instructions.md` L35–88, `spec/spurgear/instructions.md` L126–165, `spec/spurgear/instructions.md` L221–224, `spec/spurgear/instructions.md` L391–409, `spec/spurgear/instructions.md` L481–490.

## 0d `[PROSE]` `generate` orchestration and `cleanup`

`generate(inputs)` follows the call graph in step 0a exactly. After `processInputs`, it names the
component: `component = self.getComponent()` and `component.name = self.generateName()`. It runs
the steps below through the named build methods. `buildBore` and `chamferTeeth` are called
unconditionally from `generate` and guard themselves. `cleanup(ctx)` is the very last action of
`generate`, after `chamferTeeth`, called unconditionally in both modes; the Tools sketch must stay
visible until then because step 12 re-projects `ctx.anchorPoint` from it and projection fails on a
hidden sketch [PB-HIDE-AFTER-USE].

`cleanup(ctx)` [SPUR-F-CLEANUP]: turn off the light bulb on every construction plane and axis
this gear created — `ctx.extrusionEndPlane.isLightBulbOn = False`, `ctx.centerAxis.isLightBulbOn = False`,
and the normalised plane from step 1 when one was created — in both modes; `isVisible = False`
does nothing to construction geometry. Then, only when not SketchOnly, hide the sketches:
`self.toolsSketch.isVisible = False`, `ctx.gearProfileSketch.isVisible = False`,
`self.boreSketch.isVisible = False`. Guard each entity individually: the axis and the Bore Profile
sketch do not exist in sketch-only mode, and the bore sketch does not exist when Bore Diameter is
0. SketchOnly is read back as `self.getParameterAsBoolean(PARAM_SKETCH_ONLY)`.

Full-constraint gating [PB-FULL-CONSTRAINT]: the Tools sketch and the Bore Profile sketch carry
no text, so each is gated with `if not sketch.isFullyConstrained: raise` naming the sketch. The
Gear Profile sketch carries four along-path labels, which hold degrees of freedom of their own
[PB-TEXT-HOLDS-DOF], so its `isFullyConstrained` is logged, never raised on.

**From:** `spec/spurgear/instructions.md` L344–380, `spec/spurgear/instructions.md` L481–490, `spec/spurgear/instructions.md` L560–562, `spec/spurgear/fusion.md` L219–229.

## 1 `[PROSE]` Normalise the Target Plane

If `self.plane` is not already an `adsk.fusion.ConstructionPlane` (the user picked a planar face),
build a coplanar construction plane and use it everywhere after:
`planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByOffset(self.plane, adsk.core.ValueInput.createByReal(0))`,
`self.plane = component.constructionPlanes.add(planeInput)`. The offset is a `ValueInput`, never a
bare number [PB-CONSTRUCTION-PLANES]. Keep the created plane so `cleanup` can turn its light bulb
off. `ctx.plane = self.plane` after `newContext()`.

No proof: the step creates no geometry the harness measures; decad has no construction planes.

**From:** `spec/spurgear/instructions.md` L39, `spec/spurgear/instructions.md` L331, `spec/spurgear/instructions.md` L340–342, `spec/spurgear/instructions.md` L494–496.

## 2a `[GO]` Tools sketch: project the Anchor Point

`prepareTools(ctx)` first creates the Tools sketch:
`sketch = self.createSketchObject('Tools', ctx.plane)` (the base helper names it and starts it
hidden), `sketch.isVisible = True`, `self.toolsSketch = sketch`. Project the user's anchor into it
with `projected = sketch.project(self.anchorPoint)` and keep
`ctx.anchorPoint = projected.item(0)`, a `SketchPoint`. The sketch draws nothing else; this projection
is the canonical handle every later sketch re-projects from [SPUR-F-ANCHOR-CHAIN]. A projected
point tracks its source and carries no free DOF of its own here even though its `isFixed` property
is false; it is linked reference geometry [PB-PROJECT-NOT-FIXED]. Gate
`sketch.isFullyConstrained` (raise if false). Leave the sketch visible until `cleanup`
[PB-HIDE-AFTER-USE].

The API database backing the gates declares no `project` on `Sketch`; it declares only a
`project2` taking an entity list and a linked flag. The shipped add-ins call `sketch.project`, the
spec names it, and only a Fusion run settles it, so the spec's call is kept here and the gate
reports it as unverified. See the report.

Proof: `stepToolsSketch` in `proof/spurgear/sketches_test.go` creates the projected anchor as
reference geometry and hands the sketch to the gate.

<!-- proof-run: proofkit.Run(profileCases, stepToolsSketch) -->

**From:** `spec/spurgear/instructions.md` L41, `spec/spurgear/instructions.md` L302–304, `spec/spurgear/instructions.md` L332, `spec/spurgear/instructions.md` L498–500, `spec/spurgear/fusion.md` L19–24.

## 2b `[PROSE]` Extrusion End Plane

Still in `prepareTools(ctx)`: `planeInput = component.constructionPlanes.createInput()`,
`planeInput.setByOffset(ctx.plane, adsk.core.ValueInput.createByReal(thickness))` with
`thickness = self.getParameter(PARAM_THICKNESS).value` (cm, a numeric snapshot
[PB-NUMERIC-SNAPSHOT]), `plane = component.constructionPlanes.add(planeInput)`,
`plane.name = 'Extrusion End Plane'`, `ctx.extrusionEndPlane = plane` [PB-CONSTRUCTION-PLANES]. It
is the to-entity target of the tooth and body extrudes (steps 7 and 9) and stays visible until
`cleanup` turns its light bulb off.

No proof of its own: decad has no plane extents. The proofs of steps 7 and 9 sweep by Thickness
and assert the bodies end at z = Thickness, which is what ending on this plane means.

**From:** `spec/spurgear/instructions.md` L333, `spec/spurgear/instructions.md` L501–502.

## 3 `[GO]` Gear Profile sketch: circles, involute tooth, anchoring

`buildSketches(ctx)`: `sketch = self.createSketchObject('Gear Profile', ctx.plane)`,
`sketch.isVisible = True`, `ctx.gearProfileSketch = sketch`, then
`SpurGearInvoluteToothDesignGenerator(sketch, self).draw(ctx.anchorPoint, angle=0)`, then
`ctx.toothProfileIsEmbedded = self._lastToothEmbedded`, then log `sketch.isFullyConstrained`
[PB-TEXT-HOLDS-DOF]. Everything below happens inside the tooth generator. The scheme is proven on
the bench before any code is emitted [PB-SKETCH-FIRST]; it must reach DOF 0 with no redundant or
conflicting constraint [PB-FULL-CONSTRAINT] [PB-NO-OVERCONSTRAIN]. Every parameter value below is
`self.getParameterValue(...)` in cm.

`draw(anchorPoint, angle=0)` does, in order: `self.drawCircles()`, `self.drawTooth(angle)`, the
step-5 anchoring (3.5), and, when `angle != 0`, the confirming dimension set (3.6), as its very
last action [SPUR-F-ROTATE-CONFIRM]. `drawTooth` rotates by the `angle` argument that flows in from
`draw`, never by `self.toothAngle`.

### 3.1 drawCircles

Four circles, in this order, each centred on the local origin by passing the `SketchPoint` itself
(so all four share it, no coincident added) [SPUR-F-SHARED-ADJACENCY] [PB-SHARE-XOR-COINCIDENT]:

| # | name | radius parameter | construction |
|---|---|---|---|
| 1 | `Root Circle` | `RootCircleRadius` | no (solid) |
| 2 | `Tip Circle` | `TipCircleRadius` | yes |
| 3 | `Base Circle` | `BaseCircleRadius` | yes |
| 4 | `Pitch Circle` | `PitchCircleRadius` | yes |

For each: `circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(self.anchorPoint, radius)`
[PB-SKETCHCURVES]; `circle.isConstruction = True` for 2–4; a driving diameter dimension
`sketch.sketchDimensions.addDiameterDimension(circle, adsk.core.Point3D.create(0, radius, 0))` —
never pass `isDriven` [PB-DRIVING-DIM]; the text point is on the curve, never at its centre
[PB-RADIAL-DIM]. Then the along-path label [PB-SKETCH-TEXT]: with
`size = TipCircleRadius - RootCircleRadius` and
`label = '{} (r={:.2f}, size={:.2f})'.format(name, radius, size)` (all cm `.value`s),
`textInput = sketch.sketchTexts.createInput2(label, size)`,
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
`sketch.sketchTexts.add(textInput)`. The API database backing the gates declares no `createInput2`
on `SketchTexts`, only a `createInput3` whose height is a `ValueInput`; the playbook's call is kept
here as the shipped add-in makes it, and the gate reports it as unverified (see the report). Keep
direct references to the four circles for 3.3 and 3.6; `find_circle_by_radius` is the alternative
the spec allows, not a requirement:

<!-- check-step-calls: ignore find_circle_by_radius -->

### 3.2 drawTooth: sample the flanks

With `steps = InvoluteSteps` (15), `Rb = BaseCircleRadius`, `Rt = TipCircleRadius`,
`Rp = PitchCircleRadius`, `N = ToothNumber`:

1. For `i` in `0 … steps-1`: `r = Rb + (Rt - Rb) * i / (steps - 1)` (endpoint-inclusive: first
   sample exactly `Rb`, last exactly `Rt`; never clamp the start to the root radius);
   `p = self.calculateInvolutePoint(Rb, r)`; drop `None`.
2. Mirror every sample across +X: `(x, -y)`.
3. `(px, py) = self.calculateInvolutePoint(Rb, Rp)`;
   `rotate_angle = math.pi / (2 * N) - math.atan2(-py, px)` (the `-py` is the mirror applied to
   the analytic pitch point; never `atan2(py, px)`).
4. Rotate the mirrored samples by `rotate_angle`: the **left** flank. Mirror that across X: the
   **right** flank. Then rotate both flanks by `angle`. Draw the tooth at its final position;
   never draw it at +X and swing it with the dimension afterwards [SPUR-F-ROTATE-CONFIRM].
5. Two fitted splines, each through an `adsk.core.ObjectCollection.create()` of
   `adsk.core.Point3D.create(x, y, 0)` in sample order, base to tip:
   `leftSpline = sketch.sketchCurves.sketchFittedSplines.add(leftPoints)`,
   `rightSpline = sketch.sketchCurves.sketchFittedSplines.add(rightPoints)`.
6. Embedded flag, strict, no tolerance: `firstRadius` = distance from `(0, 0)` to the left
   flank's first point; `self.parent._lastToothEmbedded = firstRadius < RootCircleRadius`
   [SPUR-F-FLANK-ROOT].

### 3.3 Tooth-top point, tooth-top arc

Rule: [SPUR-F-TOOTHTOP-ARC].

1. `toothTop = sketch.sketchPoints.add(adsk.core.Point3D.create(Rt * math.cos(angle), Rt * math.sin(angle), 0))`;
   `sketch.geometricConstraints.addCoincident(toothTop, tipCircle)` (point on the tip circle).
2. `arc = sketch.sketchCurves.sketchArcs.addByCenterStartEnd(self.anchorPoint, rightSpline.endSketchPoint, leftSpline.endSketchPoint)`
   — the two flank ends are shared by the arc; the centre is copied.
3. `sketch.geometricConstraints.addCoincident(arc.centerSketchPoint, self.anchorPoint)` — the one
   arc whose centre is coincident rather than shared; without it the centre is a free point that
   stays behind when step 3.5 drags the sketch onto the anchor.
4. No diameter dimension on this arc.

### 3.4 Spine, +X reference, angular pin

Rule: [SPUR-F-SPINE].

1. `spine = sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, toothTop)`,
   `spine.isConstruction = True`. Both endpoints shared; no extra coincident; never
   `addHorizontal` on the spine (direction without sense lets the tooth land 180° around):

   <!-- check-step-calls: ignore addHorizontal -->
2. `refEnd = sketch.sketchPoints.add(adsk.core.Point3D.create(Rt, 0, 0))`; pin it with two axis
   dimensions from the local origin, values non-negative magnitudes, seeded on +X
   [PB-DIM-VALUE-SEMANTICS]:
   `sketch.sketchDimensions.addDistanceDimension(self.anchorPoint, refEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, adsk.core.Point3D.create(Rt / 2, -Rt / 4, 0))`
   with `.parameter.value = Rt`, and the same call with
   `adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` and text point
   `adsk.core.Point3D.create(Rt * 1.1, 0, 0)` with `.parameter.value = 0`. Do not pin this point
   onto the tip circle.
3. `reference = sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, refEnd)`,
   `reference.isConstruction = True`.
4. `angleDim = sketch.sketchDimensions.addAngularDimension(reference, spine, adsk.core.Point3D.create(R * math.cos(angle / 2), R * math.sin(angle / 2), 0))`
   with `R = Rt / 2`: reference first, spine second, text on the bisector so the dimension reads
   `angle`, not its supplement [PB-ANGULAR-DIM]. Created for every `angle`, including 0.

### 3.5 Ribs, one per fit-point index, in this exact order

Rule: [SPUR-F-RIBS].

Let `across = abs(math.cos(angle)) >= abs(math.sin(angle))`; the rib takes the axis across the
spine and the chain the axis along it: when `across`, rib dimensions are
`VerticalDimensionOrientation` and chain dimensions `HorizontalDimensionOrientation`; otherwise
swapped. `prev = self.anchorPoint`. For every `i` in `0 … n-1` where `n = leftSpline.fitPoints.count`
(the first, base-circle pair and the last, tip pair included):

1. `left = leftSpline.fitPoints.item(i)`, `right = rightSpline.fitPoints.item(i)`;
   `rib = sketch.sketchCurves.sketchLines.addByTwoPoints(left, right)`, `rib.isConstruction = True`.
2. `sketch.sketchDimensions.addDistanceDimension(left, right, <rib orientation>, textPoint)` with
   the fit points at their seeded positions and the value left at the measured magnitude; an
   aligned orientation would allow the mirrored tooth [PB-DIM-VALUE-SEMANTICS].
3. Midpoint seeded on the spine at the foot of the left fit point: with
   `t = fx * math.cos(angle) + fy * math.sin(angle)` (`(fx, fy)` = `left.geometry`),
   `mid = sketch.sketchPoints.add(adsk.core.Point3D.create(t * math.cos(angle), t * math.sin(angle), 0))`
   [PB-SEED-NEAR].
4. `sketch.geometricConstraints.addCoincident(mid, spine)` first.
5. `sketch.geometricConstraints.addMidPoint(mid, rib)` second.
6. `sketch.geometricConstraints.addPerpendicular(spine, rib)` third — **skipped for the last
   rib** (`i == n-1`), whose perpendicular the tooth-top arc already implies; adding it throws
   `VCS_SKETCH_OVER_CONSTRAINTS`.

Then the chain dimension along the spine:
`sketch.sketchDimensions.addDistanceDimension(prev, mid, <chain orientation>, textPoint)`, value
left at the measured magnitude, direction captured from the seeds; `prev = mid`. The first rib's
chain dimension runs from the local origin; without it the whole chain slides along the spine.

### 3.6 Flank-to-root lines (`_drawFlankToRoot`, non-embedded only)

Rule: [SPUR-F-FLANK-ROOT].

If `self.parent._lastToothEmbedded` is false, on each side: with `(sx, sy)` the flank start (the
first fit point) and `k = RootCircleRadius / math.hypot(sx, sy)`, the root end is
`(rx, ry) = (sx * k, sy * k)`;
`line = sketch.sketchCurves.sketchLines.addByTwoPoints(adsk.core.Point3D.create(rx, ry, 0), spline.startSketchPoint)`
(the flank start shared, no coincident); then exactly two dimensions and nothing else:
`sketch.sketchDimensions.addDistanceDimension(self.anchorPoint, line.startSketchPoint, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`
with `.parameter.value = abs(rx)`, and the same with
`adsk.fusion.DimensionOrientations.VerticalDimensionOrientation` and `.parameter.value = abs(ry)`.
Never the axis-signed deltas (a negative value flips the point across the origin)
[PB-DIM-VALUE-SEMANTICS]; never "root end on the root circle" plus "origin on the line", which
also admits the far intersection. No `addCoincident` in this helper. The tooth loop then has 6
curves (2 splines, 2 lines, 2 arcs); embedded, 4 (2 splines, 2 arcs).

### 3.7 Step 5: anchor the sketch

Rules: [SPUR-F-ANCHOR-CHAIN] [SPUR-F-LOCAL-ORIGIN].

`projected = sketch.project(anchorPoint)` (the `anchorPoint` argument of `draw`, which is
`ctx.anchorPoint`, the Tools-sketch projection) and
`sketch.geometricConstraints.addCoincident(self.anchorPoint, projected.item(0))`: the local origin, not
`sketch.originPoint`, onto the projected anchor. Everything above is constrained relative to the
local origin, so this drags the whole tooth onto the anchor as a unit. Constraining to
`originPoint` instead has failed the solver [PB-CIRCLE-CENTER].

### 3.8 Very last action

Rule: [SPUR-F-ROTATE-CONFIRM].

`if angle != 0: angleDim.parameter.value = angle`.

Proof: `stepGearProfile` in `proof/spurgear/sketches_test.go` draws this scheme in the engine in the
same order (fitted splines through the same samples, the arc's centre a fresh point tied to the
origin, signed axis dimensions carrying the seed side, the anchor a reference point the origin is
made coincident with) across sizes, signed angles including 90° and 180°, low step counts, both
embedded routes, and off-origin anchors; asserts the sketch closes exactly two regions, the tooth
loop with 2 splines + 2 arcs + 2 lines (no lines when embedded) and the disc bounded by the root
circle alone, that the drawing followed the anchor and the arc kept the tip radius; then the gate
requires DOF 0, no redundancy, no conflict, no second configuration. Labels are not drawn: the
engine has no sketch text.

<!-- proof-run: proofkit.Run(profileCases, stepGearProfile) -->

**From:** `spec/spurgear/instructions.md` L254–325, `spec/spurgear/instructions.md` L384–386, `spec/spurgear/instructions.md` L411–452, `spec/spurgear/instructions.md` L504–558, `spec/spurgear/fusion.md` L19–215.

## 6 `[PROSE]` Sketch-only short-circuit

In `buildMainGearBody(ctx)`, right after `buildSketches(ctx)`: if
`self.getParameterAsBoolean(PARAM_SKETCH_ONLY)`, set `ctx.gearProfileSketch.isVisible = True` and
return, skipping steps 7–11. `buildBore` and `chamferTeeth` return on the same flag (steps 12 and
13); `cleanup` still runs and hides only planes and axes in this mode [SPUR-F-CLEANUP].

**From:** `spec/spurgear/instructions.md` L60, `spec/spurgear/instructions.md` L362, `spec/spurgear/instructions.md` L560–562.

## 7 `[GO]` Extrude the tooth

`buildTooth(ctx)`:
`profile = find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`
— the framework helper, never a hand-rolled loop search [PB-PROFILE-MATCH]. Then
`extrudes = component.features.extrudeFeatures`,
`extInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`extent = adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrude = extrudes.add(extInput)`, `extrude.name = 'Extrude tooth'`,
`ctx.toothBody = extrude.bodies.item(0)`. Enums live in `adsk.fusion` [PB-ADSK-MODULES].

Proof: `stepExtrudeTooth` with `assertExtrudeTooth` in `proof/spurgear/solids_test.go`. decad
cannot record the real tooth region (its root edge is a fragment of the solid root circle, and
fragments in a sketch holding a fitted spline are not certified exact) and cannot integrate a
15-point fitted spline, so the proof requires that refusal, then extrudes the same boundary with
chorded flanks and an explicit root arc; it asserts one body, volume = section area × Thickness,
z from 0 to Thickness, and one face per boundary curve plus two caps.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->

**From:** `spec/spurgear/instructions.md` L292–300, `spec/spurgear/instructions.md` L335, `spec/spurgear/instructions.md` L387–388, `spec/spurgear/instructions.md` L564–568.

## 9 `[GO]` Extrude the body

`buildBody(ctx)`: `profile = find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)` — the disc
inside the root circle, whose boundary is the two pieces the tooth cuts the root circle into; the
tip circle is construction and bounds nothing [PB-PROFILE-MATCH]. Extrude exactly as step 7:
`extInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`extInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrude = extrudes.add(extInput)`, `extrude.name = 'Extrude body'`,
`body = extrude.bodies.item(0)`, `body.name = 'Gear Body'`.

Walk `body.faces` once and classify each by `face.geometry.surfaceType` [PB-FACE-BY-MIDPOINT]:

- `adsk.core.SurfaceTypes.CylinderSurfaceType` (in `adsk.core` [PB-ADSK-MODULES]):
  `axisInput = component.constructionAxes.createInput()`, `axisInput.setByCircularFace(face)`,
  `axis = component.constructionAxes.add(axisInput)`, `axis.name = 'Gear Center'`,
  `axis.isLightBulbOn = False`, `ctx.centerAxis = axis` [PB-CONSTRUCTION-AXES].
- `adsk.core.SurfaceTypes.PlaneSurfaceType`: with
  `sketchPlane = ctx.gearProfileSketch.referencePlane.geometry`, the face where
  `sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry)`
  is the far cap: `ctx.extrusionExtent = face`.

Raise, naming what was not found, if either is missing [PB-EMPTY-RESULT] [PB-SELF-DIAGNOSING].
Finally `ctx.gearBody = body`.

Proof: `stepExtrudeBody` with `assertExtrudeBody` in `proof/spurgear/solids_test.go` extrudes the
disc region of the real Gear Profile sketch (decad records the whole root circle) and asserts a
cylinder of Root Circle Radius on the anchor's axis, a cap on the sketch plane and a cap at
Thickness, and that the far cap is selectable as the one planar face facing along the normal.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeBody, assertExtrudeBody) -->

**From:** `spec/spurgear/instructions.md` L292–300, `spec/spurgear/instructions.md` L336–338, `spec/spurgear/instructions.md` L570–579.

## 10a `[GO]` Circular-pattern the tooth

`patternTeeth(ctx)` first: `entities = adsk.core.ObjectCollection.create()`,
`entities.add(ctx.toothBody)`, `patterns = component.features.circularPatternFeatures`,
`patternInput = patterns.createInput(entities, ctx.centerAxis)`,
`patternInput.quantity = adsk.core.ValueInput.createByReal(toothNumber)` with
`toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value`,
`patternInput.totalAngle = adsk.core.ValueInput.createByString('360 deg')`,
`patternInput.isSymmetric = False`, `pattern = patterns.add(patternInput)`. All three inputs are
pinned explicitly [PB-CIRCULAR-PATTERN].

Proof: `stepPatternTeeth` with `assertPatternTeeth` in `proof/spurgear/solids_test.go`. decad has no
pattern feature and cannot decide whether neighbouring tooth prisms are disjoint, so the proof
walks the tooth through N−1 rotations of 360/N about the anchor's axis and reads them back from
the recipe: N−1 placement steps, each the same rotation about the Gear Center, each moving the
body the previous one produced, and the last body sitting where N−1 steps put the seed with the
seed's volume.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->

**From:** `spec/spurgear/instructions.md` L365, `spec/spurgear/instructions.md` L581–585.

## 10b `[GO]` Combine the patterned teeth into the Gear Body

Still in `patternTeeth(ctx)`: `pattern.bodies` already holds the seed and the copies, so it is fed
as is, never with the seed added again; it is a `BRepBodies`, which the combine rejects, so copy it
into a fresh collection [PB-PATTERN-BODIES]: `tools = adsk.core.ObjectCollection.create()`, then
for `i` in `range(pattern.bodies.count)`: `tools.add(pattern.bodies.item(i))`. Then
`combineInput = component.features.combineFeatures.createInput(ctx.gearBody, tools)`,
`combineInput.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation`,
`component.features.combineFeatures.add(combineInput)`. One Combine-Join.

Proof: `stepCombineTeeth` with `assertCombineTeeth` in `proof/spurgear/solids_test.go`. decad's
boolean refuses operands that share a face, which the tooth's root arc and the body's root
cylinder do, so the joined gear is built as one prism from the whole-gear outline (every tooth at
its patterned angle, valley arcs between) and asserted to be the disc plus N teeth by volume, one
lump, with two caps plus one lateral face per tooth curve and per valley.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineTeeth, assertCombineTeeth) -->

**From:** `spec/spurgear/instructions.md` L365, `spec/spurgear/instructions.md` L583–585.

## 11 `[GO]` Root fillets

`createFillets(ctx)`: `filletRadius = self.getParameter(PARAM_FILLET_RADIUS).value`; return if not
`> 0`. `rootRadius = self.getParameter(PARAM_ROOT_RADIUS).value`; `axisNormal = get_normal(ctx.plane)`.
Collect `edges = adsk.core.ObjectCollection.create()`: for every face in `ctx.gearBody.faces` with
`face.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType` and
`abs(face.geometry.radius - rootRadius) <= 0.0001` (cm; every root patch, not the first), for every
edge in `face.edges` with `edge.geometry.curveType == adsk.core.Curve3DTypes.Line3DCurveType`
[PB-PROFILE-MATCH]: `direction = edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`,
`direction.normalize()`, keep it when `abs(abs(direction.dotProduct(axisNormal)) - 1.0) < 0.01`
(exactly this tolerance; a tighter one drops valid edges). Never read the direction through
`edge.evaluator.getTangent(0)`, which raises for a parameter outside the edge's range:

<!-- check-step-calls: ignore getTangent -->

If `edges.count == 0`, return without a feature [PB-EMPTY-RESULT]. Otherwise
`filletInput = component.features.filletFeatures.createInput()`,
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadius), False)`
— the edge set goes on the input itself, `isTangentChain` is `False`, the collected edges are
exactly the root corners [PB-FILLET-CHAMFER] — and `component.features.filletFeatures.add(filletInput)`.
The API database backing the gates declares no such method on `FilletFeatureInput`; it declares
an `edgeSetInputs` property there whose class carries the method, which the playbook says does not
exist in Fusion. The spec's call is kept here as the shipped add-in makes it, and the gate reports
it as unverified. See the report.

Proof: `stepRootFillets` with `assertRootFillets` in `proof/spurgear/finish_test.go` fillets the
concave axial edges of the joined-gear prism (exactly 2N, all straight) with Fillet Radius and
asserts the volume grew by exactly 2N tangent corner fills between a radial line and the root
circle, one new face per corner, and no sharp axial concave edge left. The embedded case is
unmodelled: on chorded flanks the first chord is shorter than the fillet setback and decad refuses
rather than run onto the next chord, where Fusion has one spline face.

<!-- proof-run: proofkit3d.RunSolid(filletCases, stepRootFillets, assertRootFillets) -->

**From:** `spec/spurgear/instructions.md` L82–88, `spec/spurgear/instructions.md` L126–131, `spec/spurgear/instructions.md` L366, `spec/spurgear/instructions.md` L391–396, `spec/spurgear/instructions.md` L587–596.

## 12 `[GO]` Bore

`buildBore(ctx)` returns at once when `self.getParameterAsBoolean(PARAM_SKETCH_ONLY)` (the gear body
does not exist then) and when `boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value` is
`<= 0`. Otherwise: `boreSketch = self.createSketchObject('Bore Profile', ctx.plane)`,
`boreSketch.isVisible = True`, `self.boreSketch = boreSketch`,
`toothGen = SpurGearInvoluteToothDesignGenerator(boreSketch, self)`,
`circle = toothGen.drawBore(ctx.anchorPoint, boreDiameter)`.

`drawBore(anchorPoint, diameter)`: `projected = self.sketch.project(anchorPoint).item(0)`,
`circle = self.sketch.sketchCurves.sketchCircles.addByCenterRadius(projected, diameter / 2)` (the
projected point shared as the centre),
`self.sketch.sketchDimensions.addDiameterDimension(circle, adsk.core.Point3D.create(0, diameter / 2, 0))`
(driving [PB-DRIVING-DIM], text off-centre [PB-RADIAL-DIM]); return `circle`.

Back in `buildBore`, ground the generator's stray local origin on that same projection:
`boreSketch.geometricConstraints.addCoincident(toothGen.anchorPoint, circle.centerSketchPoint)`
[SPUR-F-LOCAL-ORIGIN]; never on the sketch's own origin point, which pins it to the plane and has
failed the solver [PB-CIRCLE-CENTER]. A contract guard bans the identifier of that property
anywhere inside `buildBore`, comments included, so do not write it there at all. Gate
`boreSketch.isFullyConstrained` (raise if false)
[PB-FULL-CONSTRAINT]. The sketch has one closed loop: `profile = boreSketch.profiles.item(0)`
[PB-SINGLE-PROFILE]. Cut: `extInput = extrudes.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
`extInput.participantBodies = [ctx.gearBody]`,
`extInput.setOneSideExtent(adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False), adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
`extrudes.add(extInput)`. The far cap as extent makes the bore go through whatever Thickness is.

Proof: `stepBore` with `assertBore` in `proof/spurgear/finish_test.go`. Without a bore it returns the
joined gear unchanged and asserts so. With one, it extrudes the bore tool onto the far cap of the
joined gear in a scratch document and asserts the tool spans exactly the Thickness, requires
decad's refusal of that cut (its analytic cut caps the toothed section's arc count and its mesh
cut refuses a tool ending on the caps), then runs the same to-face cut on the Gear Body alone and
asserts exactly one bore cylinder of Bore Diameter on the anchor's axis through the whole
Thickness with the matching volume.

<!-- proof-run: proofkit3d.RunSolid(boreCases, stepBore, assertBore) -->

**From:** `spec/spurgear/instructions.md` L54, `spec/spurgear/instructions.md` L320–322, `spec/spurgear/instructions.md` L368, `spec/spurgear/instructions.md` L430–435, `spec/spurgear/instructions.md` L598–602, `spec/spurgear/fusion.md` L26–31, `spec/spurgear/contract.json` L107–113.

## 13 `[GO]` Chamfer the completed gear

`chamferTeeth(ctx)` returns when `self.getParameterAsBoolean(PARAM_SKETCH_ONLY)` and when
`chamfer = self.getParameter(PARAM_CHAMFER_TOOTH).value` is `<= 0`. Otherwise, with
`sketchPlane = ctx.gearProfileSketch.referencePlane.geometry` and
`boreDiameter = self.getParameter(PARAM_BORE_DIAMETER).value`: walk every face in
`ctx.gearBody.faces` with `face.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType` and
`sketchPlane.isParallelToPlane(face.geometry)` (both end caps). Add every edge of those faces to
`edges = adsk.core.ObjectCollection.create()` once, keyed by `edge.tempId`, except an edge with
`edge.geometry.curveType == adsk.core.Curve3DTypes.Circle3DCurveType` whose
`abs(edge.geometry.radius - boreDiameter / 2) <= 0.001` (cm) when `boreDiameter > 0`, so a bore
never receives a chamfer. Flanks, tooth tops and root arcs are all included. Raise if no end-cap
face or no edge remains; never a partial chamfer [PB-EMPTY-RESULT]. Then
`chamferInput = component.features.chamferFeatures.createInput2()`,
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamfer), False)`
(the edge set goes on the input's `chamferEdgeSets`, unlike the fillet [PB-FILLET-CHAMFER]),
`component.features.chamferFeatures.add(chamferInput)`. Helical and herringbone inherit this
unchanged; the Fusion verification of the selection is recorded at [HELI-F-CHAMFER-COUNT].

Proof: `stepChamferTeeth` with `assertChamferTeeth` in `proof/spurgear/finish_test.go`. decad's
cap-loop chamfer of the toothed gear builds but certifies a volume bound far coarser than the
material removed, which fails verification, so the proof runs the selection rule above on the
joined gear (2 × (6N or 4N) edges kept, none excluded) and on the bored Gear Body (its two outer
rims kept, its two bore rims excluded), then applies the chamfer to the Gear Body's start-cap outer
rim beside the gear and asserts the removed ring exactly, one new conical face, and untouched bore
rims. Without a chamfer it returns the gear unchanged and asserts so.

<!-- proof-run: proofkit3d.RunSolid(chamferCases, stepChamferTeeth, assertChamferTeeth) -->

**From:** `spec/spurgear/instructions.md` L58, `spec/spurgear/instructions.md` L369, `spec/spurgear/instructions.md` L389–390, `spec/spurgear/instructions.md` L604–619, `spec/helicalgear/fusion.md` L69–80.
