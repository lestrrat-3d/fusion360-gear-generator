The proof files are `proof/spurgear/sketches_test.go`, `proof/spurgear/solids_test.go`, and `proof/spurgear/zz_registrations_test.go`.

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `8c761c4542788b3ad455fa7ae02dbf978e65b8ca` |
| `spec/spurgear/fusion.md` | `5cd1f9f96e043efba42ae42a00ca6c13403e1339` |
| `spec/helicalgear/fusion.md` | `f981173cb314094f2fd98cdd78d5bd8287cdc8ee` |
| `spec/spurgear/contract.json` | `b72ff283f74775e62f9933eb1e905b49628676ff` |
| `spec/spurgear/exact_values.json` | `fee6665556d2d9bb3673ee44b79fca03e7caf5a2` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `9ee2dcbaed7b5480aa69e9295e8b61acaea081f3` |

## Compilation contract

```json
{
  "contract": {
    "_comment": "Machine-readable contract checked by check_contract.py. This file owns exported Python constant names and values; exact_values.json refers to those names for dialog and parameter setup. The prose explains behavior and points to these checked sources. Methods are the pinned public and hook surface. source_guards pin constraint recipes whose names alone cannot reveal a regression.",
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
        "file": "lib/geargen/spurgear.py",
        "in_function": "prepareTools",
        "required": [
          "\\.project\\([^)]*\\)\\.item\\(0\\)"
        ],
        "why": "[SPUR-F-ANCHOR-CHAIN]: Sketch.project returns an ObjectCollection; ctx.anchorPoint must hold its SketchPoint item."
      },
      {
        "file": "lib/geargen/spurgear.py",
        "in_function": "draw",
        "required": [
          "\\.project\\([^)]*\\)\\.item\\(0\\)"
        ],
        "why": "[SPUR-F-ANCHOR-CHAIN]: addCoincident requires a SketchEntity, not the ObjectCollection from Sketch.project."
      },
      {
        "file": "lib/geargen/spurgear.py",
        "in_function": "drawBore",
        "required": [
          "\\.project\\([^)]*\\)\\.item\\(0\\)"
        ],
        "why": "[SPUR-F-ANCHOR-CHAIN]: the bore centre must use the SketchPoint item from Sketch.project's ObjectCollection."
      },
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
        "file": "proof/spurgear/sketches_test.go",
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
        "file": "proof/spurgear/sketches_test.go",
        "required": [
          "flank-to-root lines: root endpoint pinned by signed",
          "NewHorizontalDistance\\(origin, rootEnd, dx\\)",
          "NewVerticalDistance\\(origin, rootEnd, dy\\)"
        ],
        "why": "The bench README is the constraint-by-constraint map a reader trusts; a stale row there sends the next generation back to the rejected recipe."
      }
    ]
  },
  "schema": 1
}
```

## Exact values

```json
{
  "schema": 1,
  "constants": {
    "INPUT_ID_PARENT": "parentComponent",
    "INPUT_ID_PLANE": "plane",
    "INPUT_ID_ANCHOR_POINT": "anchorPoint",
    "INPUT_ID_MODULE": "module",
    "INPUT_ID_TOOTH_NUMBER": "toothNumber",
    "INPUT_ID_PRESSURE_ANGLE": "pressureAngle",
    "INPUT_ID_BORE_DIAMETER": "boreDiameter",
    "INPUT_ID_THICKNESS": "thickness",
    "INPUT_ID_CHAMFER_TOOTH": "chamferTooth",
    "INPUT_ID_SKETCH_ONLY": "sketchOnly",
    "PARAM_MODULE": "Module",
    "PARAM_TOOTH_NUMBER": "ToothNumber",
    "PARAM_PRESSURE_ANGLE": "PressureAngle",
    "PARAM_BORE_DIAMETER": "BoreDiameter",
    "PARAM_THICKNESS": "Thickness",
    "PARAM_CHAMFER_TOOTH": "ChamferTooth",
    "PARAM_SKETCH_ONLY": "SketchOnly",
    "PARAM_PITCH_DIAMETER": "PitchCircleDiameter",
    "PARAM_PITCH_RADIUS": "PitchCircleRadius",
    "PARAM_BASE_DIAMETER": "BaseCircleDiameter",
    "PARAM_BASE_RADIUS": "BaseCircleRadius",
    "PARAM_ROOT_DIAMETER": "RootCircleDiameter",
    "PARAM_ROOT_RADIUS": "RootCircleRadius",
    "PARAM_TIP_DIAMETER": "TipCircleDiameter",
    "PARAM_TIP_RADIUS": "TipCircleRadius",
    "PARAM_INVOLUTE_STEPS": "InvoluteSteps",
    "PARAM_TOOTH_SPACE_ANGLE": "ToothSpaceAngleAtRoot",
    "PARAM_TOOTH_SPACE_ARC": "ToothSpaceArcAtRoot",
    "PARAM_FILLET_CLEARANCE": "FilletClearance",
    "PARAM_FILLET_RADIUS": "FilletRadius"
  },
  "inputs": [
    {
      "id": "INPUT_ID_PLANE",
      "kind": "selection",
      "label": "Target Plane",
      "prompt": "Select the plane to build the gear on",
      "filters": [
        "ConstructionPlanes",
        "PlanarFaces"
      ],
      "preselect": false
    },
    {
      "id": "INPUT_ID_ANCHOR_POINT",
      "kind": "selection",
      "label": "Anchor Point",
      "prompt": "Select the point the gear is centered on",
      "filters": [
        "ConstructionPoints",
        "SketchPoints"
      ],
      "preselect": false
    },
    {
      "id": "INPUT_ID_MODULE",
      "kind": "value",
      "label": "Module",
      "unit": "",
      "default": {
        "real": 1
      }
    },
    {
      "id": "INPUT_ID_TOOTH_NUMBER",
      "kind": "value",
      "label": "Tooth Number",
      "unit": "",
      "default": {
        "real": 17
      }
    },
    {
      "id": "INPUT_ID_PRESSURE_ANGLE",
      "kind": "value",
      "label": "Pressure Angle",
      "unit": "deg",
      "default": {
        "radians": 20
      }
    },
    {
      "id": "INPUT_ID_BORE_DIAMETER",
      "kind": "string",
      "label": "Bore Diameter",
      "default": "0 mm"
    },
    {
      "id": "INPUT_ID_THICKNESS",
      "kind": "value",
      "label": "Thickness",
      "unit": "mm",
      "default": {
        "millimeters": 10
      }
    },
    {
      "id": "INPUT_ID_CHAMFER_TOOTH",
      "kind": "value",
      "label": "Apply chamfer to teeth",
      "unit": "mm",
      "default": {
        "real": 0
      }
    },
    {
      "id": "INPUT_ID_SKETCH_ONLY",
      "kind": "boolean",
      "label": "Generate sketches, but do not build body",
      "check_box": true,
      "default": false
    },
    {
      "id": "INPUT_ID_PARENT",
      "kind": "selection",
      "label": "Parent Component",
      "prompt": "Select the component to build the gear in",
      "filters": [
        "Occurrences",
        "RootComponents"
      ],
      "preselect": true
    }
  ],
  "parameters": [
    {
      "name": "PARAM_MODULE",
      "unit": "",
      "comment": "Module of the gear",
      "input": "INPUT_ID_MODULE"
    },
    {
      "name": "PARAM_TOOTH_NUMBER",
      "unit": "",
      "comment": "Number of teeth",
      "input": "INPUT_ID_TOOTH_NUMBER"
    },
    {
      "name": "PARAM_PRESSURE_ANGLE",
      "unit": "rad",
      "comment": "Pressure angle",
      "input": "INPUT_ID_PRESSURE_ANGLE"
    },
    {
      "name": "PARAM_BORE_DIAMETER",
      "unit": "mm",
      "comment": "Bore diameter",
      "input": "INPUT_ID_BORE_DIAMETER"
    },
    {
      "name": "PARAM_THICKNESS",
      "unit": "mm",
      "comment": "Thickness of the gear",
      "input": "INPUT_ID_THICKNESS"
    },
    {
      "name": "PARAM_CHAMFER_TOOTH",
      "unit": "mm",
      "comment": "Chamfer distance applied to the teeth",
      "input": "INPUT_ID_CHAMFER_TOOTH"
    },
    {
      "name": "PARAM_SKETCH_ONLY",
      "unit": "",
      "comment": "Generate sketches only",
      "input": "INPUT_ID_SKETCH_ONLY"
    },
    {
      "name": "PARAM_PITCH_DIAMETER",
      "unit": "mm",
      "comment": "Pitch circle diameter",
      "expression": "{PARAM_MODULE} * {PARAM_TOOTH_NUMBER}"
    },
    {
      "name": "PARAM_PITCH_RADIUS",
      "unit": "mm",
      "comment": "Pitch circle radius",
      "expression": "{PARAM_PITCH_DIAMETER} / 2"
    },
    {
      "name": "PARAM_BASE_DIAMETER",
      "unit": "mm",
      "comment": "Base circle diameter",
      "expression": "{PARAM_PITCH_DIAMETER} * cos({PARAM_PRESSURE_ANGLE})"
    },
    {
      "name": "PARAM_BASE_RADIUS",
      "unit": "mm",
      "comment": "Base circle radius",
      "expression": "{PARAM_BASE_DIAMETER} / 2"
    },
    {
      "name": "PARAM_ROOT_DIAMETER",
      "unit": "mm",
      "comment": "Root circle diameter",
      "expression": "{PARAM_PITCH_DIAMETER} - 2.5 * {PARAM_MODULE}"
    },
    {
      "name": "PARAM_ROOT_RADIUS",
      "unit": "mm",
      "comment": "Root circle radius",
      "expression": "{PARAM_ROOT_DIAMETER} / 2"
    },
    {
      "name": "PARAM_TIP_DIAMETER",
      "unit": "mm",
      "comment": "Tip circle diameter",
      "expression": "{PARAM_PITCH_DIAMETER} + 2 * {PARAM_MODULE}"
    },
    {
      "name": "PARAM_TIP_RADIUS",
      "unit": "mm",
      "comment": "Tip circle radius",
      "expression": "{PARAM_TIP_DIAMETER} / 2"
    },
    {
      "name": "PARAM_INVOLUTE_STEPS",
      "unit": "",
      "comment": "Number of points sampled along each involute flank",
      "expression": "15"
    },
    {
      "name": "PARAM_TOOTH_SPACE_ANGLE",
      "unit": "",
      "comment": "Angular width of the tooth space at the root circle",
      "computed": "tooth_space_angle"
    },
    {
      "name": "PARAM_TOOTH_SPACE_ARC",
      "unit": "mm",
      "comment": "Arc length of the tooth space at the root circle",
      "expression": "{PARAM_ROOT_RADIUS} * {PARAM_TOOTH_SPACE_ANGLE}"
    },
    {
      "name": "PARAM_FILLET_CLEARANCE",
      "unit": "",
      "comment": "Clearance factor applied to the root fillet radius",
      "expression": "0.9"
    },
    {
      "name": "PARAM_FILLET_RADIUS",
      "unit": "mm",
      "comment": "Radius of the root fillets",
      "expression": "({PARAM_TOOTH_SPACE_ARC} / 2) * {PARAM_FILLET_CLEARANCE} * {fillet_helix_factor}"
    }
  ]
}
```

## 01 `[PROSE]` Select and normalize the target plane

The module exports exactly the constants and four classes in `spec/spurgear/contract.json`. The
checked `spec/spurgear/exact_values.json` supplies dialog order, IDs, prompts, filters, defaults,
parameter names, units, comments, and derived expressions. `configure` is a classmethod on
`SpurGearCommandInputsConfigurator`; subclass configurators append their inputs after it returns.
Create the first two dialog inputs as Target Plane and Anchor Point and the last spur input as
Parent Component. Each selection has the JSON filter set and exactly one selection. Use the
JSON's numeric defaults in Fusion internal centimetres and radians [PB-DIALOG-DEFAULT-UNITS]
[PB-SELECTION-DECL] [PB-SELECTION-FILTER-ENUM] [SPUR-SUBCLASS-INPUT].

Before creating an occurrence or registering a parameter, read Parent Component, Target Plane,
and Anchor Point with `get_selection(inputs, id)` and save the entities on the generator. Then
create the occurrence and register the input-sourced parameters with `get_value(inputs, id, units)`
for value and string inputs, and `get_boolean(inputs, id)` for SketchOnly. Convert that boolean
to 1 or 0 before `addParameter(...)`. Call the no-op base hook
`addExtraPrimaryParameters(inputs)` between primary and derived registration. Register all derived
parameters with the exact JSON expressions, except ToothSpaceAngleAtRoot, whose numeric value is
`π / ToothNumber - 2 * (tan(PressureAngle) - PressureAngle)` and whose unit is empty. The live
FilletRadius expression ends in `filletHelixFactorExpression()`; the spur implementation returns
`'1'`. `generateName()` uses the `Module`, `ToothNumber`, and `Thickness` parameter `.expression`
strings to form `Spur Gear (M={}, Tooth={}, Thickness={})`. These actions precede timeline feature
creation [PB-INPUT-READ] [PB-GET-VALUE-CONTRACT] [PB-SELECTION-STASH] [SPUR-EXTRA-PARAMS].

Use `getComponent()` and name it with `generateName()`. If the selected plane is already a
`ConstructionPlane`, use it. Otherwise create a coplanar construction plane with
`constructionPlanes.createInput()`, `planeInput.setByOffset(selectedPlane, adsk.core.ValueInput.createByReal(0))`,
and `constructionPlanes.add(planeInput)`; store the result as both `self.plane` and `ctx.plane`.
`setByOffset` takes a `ValueInput`, including for zero [PB-CONSTRUCTION-PLANES].

`SpurGearGenerator` keeps `self.toolsSketch = None`, `self.boreSketch = None`, and
`self._lastToothEmbedded = False` initially. `newContext()` returns
`SpurGearGenerationContext` with `plane`, `anchorPoint`, `extrusionEndPlane`,
`gearProfileSketch`, `toothBody`, `gearBody`, `centerAxis`, and `extrusionExtent` initialized
with `cast(None)`, and `toothProfileIsEmbedded = False`. Preserve the distinct methods and
override boundaries listed in the contract. Preserve these exact subclass-facing return
annotations: `SpurGearGenerator.prefixBase -> str`,
`SpurGearGenerator.generateName -> str`,
`SpurGearGenerator.filletHelixFactorExpression -> str`,
`SpurGearGenerator.newContext -> SpurGearGenerationContext`, and
`SpurGearInvoluteToothDesignGenerator.getParameterValue -> float`. The spur base's
`prefixBase` returns exactly `'SpurGear'` (`spec/spurgear/instructions.md:96-100`). `generate()` calls
`processInputs`, `prepareTools`, `buildMainGearBody`,
`buildBore`, `chamferTeeth`, and `cleanup`, in that order. `buildMainGearBody` calls
`buildSketches`, then stops on SketchOnly or calls `buildTooth`, `buildBody`, `patternTeeth`, and
`createFillets`, in that order. `cleanup` is the final unconditional call. No proof-engine
primitive represents a Fusion occurrence, dialog input, or construction plane [PB-SKETCH-FIRST].
The framework calls the module's `generate` method; this mention specifies its call graph,
not a call the module makes itself.
<!-- check-step-calls: ignore generate -->

**From:** `spec/spurgear/instructions.md:12-36,95-179,240-333,384-423`;
`spec/spurgear/fusion.md:17-42`; `spec/spurgear/exact_values.json:1-37`;
`spec/spurgear/contract.json:1-95`; `.claude/skills/generate-gear/PLAYBOOK.md:42-143,205-264,657-670,775-786`.

## 02 `[PROSE]` Project the anchor in Tools and make the end plane

Create the `Tools` sketch on `ctx.plane` with `createSketchObject('Tools', ctx.plane)`, expose it
while later projections use it, and call `toolsSketch.project(selectedAnchor).item(0)`.
Store that `SketchPoint` as `ctx.anchorPoint` [SPUR-F-ANCHOR-CHAIN]
[PB-PROJECT-NOT-FIXED] [PB-HIDE-AFTER-USE]. The legacy `Sketch.project` call is a direct spec
requirement that works in the deployed add-in; the local Fusion API database omits it. The
projected point, rather than its containing collection, is the operand of later calls.

Make `Extrusion End Plane` from `ctx.plane` using `constructionPlanes.createInput()`,
`endInput.setByOffset(ctx.plane, adsk.core.ValueInput.createByReal(thicknessCm))`, and
`constructionPlanes.add(endInput)` [PB-CONSTRUCTION-PLANES] [PB-NUMERIC-SNAPSHOT]. Store the
construction plane as `ctx.extrusionEndPlane`. It remains available to both extrudes until final
cleanup. `thicknessCm` is the current numeric Thickness parameter value; feature inputs never
carry a live parameter expression [SPUR-F-SNAPSHOT]. The proof engine has no cross-sketch Fusion
projection or construction-plane operation.

**From:** `spec/spurgear/instructions.md:425-434`; `spec/spurgear/fusion.md:19-27,226-242`;
`.claude/skills/generate-gear/PLAYBOOK.md:229-238,458-472,657-670,775-786`.

## 03 `[GO]` Draw and anchor the Gear Profile sketch

`buildSketches(ctx)` creates a sketch named `Gear Profile` on `ctx.plane`, then constructs
`SpurGearInvoluteToothDesignGenerator(sketch, self)` and calls `toothGen.draw(ctx.anchorPoint)`.
After that call it copies `self._lastToothEmbedded` into `ctx.toothProfileIsEmbedded`. The tooth
generator constructor creates a fresh local-origin `SketchPoint` at `(0,0,0)`, stores it as
`self.anchorPoint`, and stores its optional constructor angle as `self.toothAngle`. Runtime
`draw(anchorPoint, angle=0)` passes the runtime angle to `drawTooth`; the constructor field does
not replace it [SPUR-F-LOCAL-ORIGIN] [SPUR-F-ROTATE-CONFIRM].

Inside `draw`, call `drawCircles()` first. Draw solid Root Circle, then construction Tip, Base,
and Pitch circles, all with `sketch.sketchCurves.sketchCircles.addByCenterRadius(self.anchorPoint, radiusCm)`.
Pass the same `SketchPoint` object as every centre [PB-SKETCHCURVES]
[PB-SHARE-XOR-COINCIDENT]. Give each a driving diameter with
`sketch.sketchDimensions.addDiameterDimension(circle, textPoint)` and set its parameter's numeric
`.value` to the current diameter in cm [PB-DRIVING-DIM] [PB-NUMERIC-SNAPSHOT]. Add the circle
label `'{} (r={:.2f}, size={:.2f})'.format(name, radius, size)`, where `radius` and
`size = TipCircleRadius - RootCircleRadius` are current internal cm values. Use
`sketch.sketchTexts.createInput2(text, size)`,
`textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`,
and `sketch.sketchTexts.add(textInput)` [PB-SKETCH-TEXT]. The legacy `createInput2` is required
by the spec and used by the deployed add-in, though the local API database omits it.

Next call `drawTooth(angle)`. Obtain each of the `InvoluteSteps` samples from BaseCircleRadius
through TipCircleRadius with inclusive radius
`base + (tip-base) * i / (steps-1)`. `calculateInvolutePoint(base, radius)` returns `None` inside
the base circle; otherwise it uses `alpha = acos(base/radius)`, `t = tan(alpha)`,
`x = base * (cos(t)+t*sin(t))`, and `y = base * (sin(t)-t*cos(t))`. Mirror each sample's y first.
Compute the analytic pitch crossing and rotate the mirrored samples by
`π/(2*ToothNumber) - atan2(-py, px)`. These become the left flank. Mirror them about +X for the
right flank, then rotate both collections by the runtime `angle`. Draw the flanks with
`sketch.sketchCurves.sketchFittedSplines.add(fitPointCollection)` using existing `SketchPoint`
objects at all shared joins [SPUR-F-SHARED-ADJACENCY] [SPUR-F-ROTATE-CONFIRM].

Create a tooth-top `SketchPoint` at `(TipRadius*cos(angle), TipRadius*sin(angle))` and constrain
it with `geometricConstraints.addCoincident(toothTopPoint, tipCircle)`. Build the tooth-top arc
with `sketch.sketchCurves.sketchArcs.addByCenterStartEnd(self.anchorPoint, rightFlank.endSketchPoint, leftFlank.endSketchPoint)`.
The arc copies its centre, so call `geometricConstraints.addCoincident(arc.centerSketchPoint, self.anchorPoint)`.
Do not add a diameter dimension to this arc [SPUR-F-TOOTHTOP-ARC]
[PB-SHARE-XOR-COINCIDENT].

Draw the construction spine with
`sketch.sketchCurves.sketchLines.addByTwoPoints(self.anchorPoint, toothTopPoint)`. Create a
separate +X reference endpoint at `(TipRadius,0)` and constrain it from the local origin with
`sketch.sketchDimensions.addDistanceDimension(origin, refEnd, HorizontalDimensionOrientation, textPoint)`
and the corresponding `VerticalDimensionOrientation` call, assigning magnitudes `TipRadius`
and `0`. Draw the construction reference line from local origin to that endpoint. Add
`sketch.sketchDimensions.addAngularDimension(reference, spine, bisectorTextPoint)` in exactly
that operand order, using `(R*cos(angle/2), R*sin(angle/2))` for the text position
[SPUR-F-SPINE] [PB-DIM-VALUE-SEMANTICS] [PB-ANGULAR-DIM].

For every matching pair of flank fit points, including first and last, draw a construction rib
with `sketch.sketchCurves.sketchLines.addByTwoPoints(left.fitPoints[i], right.fitPoints[i])`.
Give it an axis distance dimension across the spine. Use vertical across and horizontal along
when `abs(cos(angle)) >= abs(sin(angle))`; swap otherwise. Seed the midpoint at the left fit
point's projection onto the spine, then call
`geometricConstraints.addCoincident(midpoint, spine)`,
`geometricConstraints.addMidPoint(midpoint, rib)`, and
`geometricConstraints.addPerpendicular(spine, rib)` in that order. Omit only the last rib's
perpendicular. Dimension each midpoint from the previous midpoint along the spine, beginning
with the local origin. Use the seeded direction and assign only nonnegative distance magnitudes
[SPUR-F-RIBS] [PB-DIM-VALUE-SEMANTICS].

Set `embedded = firstFlankRadius < RootCircleRadius` with a strict comparison, then write it
to `self.parent._lastToothEmbedded`. If false, connect each flank start to a root endpoint with
`sketch.sketchCurves.sketchLines.addByTwoPoints(rootEnd, flank.startSketchPoint)` and constrain
each root endpoint from the local origin using exactly horizontal and vertical
`sketch.sketchDimensions.addDistanceDimension(...)` calls. Seed its intended signed side
before adding either dimension and assign only absolute magnitudes. No flank-to-root line is
drawn when embedded [SPUR-F-FLANK-ROOT] [PB-DIM-VALUE-SEMANTICS]. The tooth section has two
NURBS and two arcs, plus two lines when nonembedded; the root disc boundary has exactly two
arcs [PB-PROFILE-MATCH].

Finally, inside `draw`, re-project `ctx.anchorPoint` into this sketch with
`sketch.project(anchorPoint).item(0)` and call
`geometricConstraints.addCoincident(self.anchorPoint, projectedAnchor)` to move the whole
sketch. For nonzero `angle`, set the angular dimension's parameter `.value = angle` as the
last action [SPUR-F-ANCHOR-CHAIN] [SPUR-F-ROTATE-CONFIRM]. Do not constrain against the
immutable `sketch.originPoint` [SPUR-F-LOCAL-ORIGIN]. The sketch proof substitutes a fixed
reference point for the projected source and omits display text; the four-circle and tooth
constraint system, the embedded branch, signed angles, and the two closed regions pass the
proofkit soundness gate [PB-SKETCH-FIRST] [PB-TEXT-HOLDS-DOF]. A Fusion sketch with along-path
labels can report `isFullyConstrained` inconsistently, so log that reading for `Gear Profile`
and gate text-free sketches normally.

**Proof:** `stepGearProfile` in `proof/spurgear/sketches_test.go`.

<!-- proof-run: proofkit.Run(profileCases, stepGearProfile) -->

**From:** `spec/spurgear/instructions.md:181-239,335-380,435-500`;
`spec/spurgear/fusion.md:19-211`; `.claude/skills/generate-gear/PLAYBOOK.md:359-432,441-535,606-700`.

## 04 `[PROSE]` Stop after the profile in SketchOnly mode

When the numeric `SketchOnly` parameter reads true through `getParameterAsBoolean`, leave
`ctx.gearProfileSketch.isVisible = True` and return from `buildMainGearBody` before any solid
feature. `generate` still calls `buildBore`, `chamferTeeth`, and `cleanup`; the first two
return early and cleanup hides construction geometry while leaving Tools and Gear Profile
sketches visible [SPUR-F-CLEANUP] [PB-HIDE-AFTER-USE]. A return creates no timeline feature
and has no proof-engine geometry to build.

**From:** `spec/spurgear/instructions.md:288-312,492-501,544-571`;
`spec/spurgear/fusion.md:215-225`; `.claude/skills/generate-gear/PLAYBOOK.md:657-670`.

## 05 `[PROSE]` Extrude the tooth

Use `find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2,
lines=0 if ctx.toothProfileIsEmbedded else 2)` to select the tooth section; do not select by
index [PB-PROFILE-MATCH]. Call `extrudeFeatures.createInput(profile,
adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`, set its one-sided extent with
`ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)` and
`PositiveExtentDirection`, then call `extrudeFeatures.add(extrudeInput)`. Name the feature
`Extrude tooth` and keep the resulting body as `ctx.toothBody`. These are numeric snapshots;
the end plane was placed at the current Thickness value [PB-NUMERIC-SNAPSHOT]
[SPUR-F-SNAPSHOT]. The sketch proof gates this actual section's closure and expected curve
types. Passing that detected section directly to decad's prism builder failed: its root-circle
fragment has an uncertified trim (`TExact = false`). The nearest proof records this engine
limit and proves the root-disc extrusion through a whole-circle substitute.

**From:** `spec/spurgear/instructions.md:500-508`; `spec/spurgear/fusion.md:43-211`;
`.claude/skills/generate-gear/PLAYBOOK.md:148-154,229-238,672-681`.

## 06 `[GO]` Extrude the root body and locate its axis and far cap

Find the root-disc profile with
`find_profile_by_curve_counts(ctx.gearProfileSketch, arcs=2)`; its boundary is exactly two
root-circle arcs [PB-PROFILE-MATCH]. Create a new-body extrude with
`extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`,
`extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`,
and `extrudeFeatures.add(extrudeInput)`. Name the feature `Extrude body` and the body
`Gear Body`; store it as `ctx.gearBody`.

Scan that body's faces. On a cylindrical face, call `constructionAxes.createInput()`,
`axisInput.setByCircularFace(face)`, and `constructionAxes.add(axisInput)`; name the axis
`Gear Center`, hide it with `isLightBulbOn = False`, and store it as `ctx.centerAxis`.
Among planar faces, choose the one for which
`sketchPlane.isParallelToPlane(face.geometry)` is true and
`sketchPlane.isCoPlanarTo(face.geometry)` is false, where `sketchPlane` is
`ctx.gearProfileSketch.referencePlane.geometry`. Store that far cap as
`ctx.extrusionExtent`. Raise if the cylindrical face or far cap is absent
[PB-EMPTY-RESULT]. The solid proof substitutes the same full circle for the two split root
arcs and verifies the area and volume at two sizes; the sketch proof checks the split boundary.

**Proof:** `stepExtrudeBody` and `assertExtrudeBody` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(bodyCases, stepExtrudeBody, assertExtrudeBody) -->

**From:** `spec/spurgear/instructions.md:509-530`; `spec/spurgear/fusion.md:19-42`;
`.claude/skills/generate-gear/PLAYBOOK.md:229-238,440-447,672-681,787-790`.

## 07 `[PROSE]` Pattern and join the teeth

Collect `ctx.toothBody` into an `ObjectCollection`; call
`circularPatternFeatures.createInput(seedBodies, ctx.centerAxis)`. Set `quantity` to the current
ToothNumber `ValueInput`, `totalAngle = adsk.core.ValueInput.createByString('360 deg')`, and
`isSymmetric = False`, then call `circularPatternFeatures.add(patternInput)`
[PB-CIRCULAR-PATTERN] [PB-NUMERIC-SNAPSHOT]. Its `bodies` collection includes the seed.
Copy every `pattern.bodies.item(i)` into a new `ObjectCollection`; the combine input requires
that collection type, so call `combineFeatures.createInput(ctx.gearBody, copiedBodies)`,
set the operation to Join, and call `combineFeatures.add(combineInput)`
[PB-PATTERN-BODIES]. The pinned engine's analytic cut path has a 4,096-segment cap when a
complex plate is cut with many circular holes; no shared example yet proves a full patterned
spur join through that evaluator. The nearest proof file records this limit rather than
returning an unverified patterned body as a successful case.

**From:** `spec/spurgear/instructions.md:531-536`;
`.claude/skills/generate-gear/PLAYBOOK.md:693-703`;
`proof/examples/OPERATIONS.md:1-18`.

## 08 `[PROSE]` Fillet the axial root corners

If current FilletRadius is positive, collect every cylindrical face of `ctx.gearBody` whose
radius differs from current RootCircleRadius by at most `0.0001` cm. From each such face, keep
only `Line3DCurveType` edges whose geometry-endpoint direction is axial:
`abs(abs(dot(normalizedDirection, targetPlaneNormal)) - 1) < 0.01`. Do not take a tangent at
parameter zero. If the edge collection is empty, return without a feature. Otherwise call
`filletFeatures.createInput()`,
`filletInput.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(filletRadiusCm), False)`,
and `filletFeatures.add(filletInput)` [PB-FILLET-CHAMFER] [PB-NUMERIC-SNAPSHOT]. The radius is
the numeric result of the registered FilletRadius expression, including the overridable last
factor. The shared examples do not provide a tested root-corner fillet construction in decad,
so this Fusion topology selection remains a Fusion check.

**From:** `spec/spurgear/instructions.md:114-122,317-330,537-550`;
`.claude/skills/generate-gear/PLAYBOOK.md:569-575`.

## 09 `[GO]` Cut the optional central bore

`buildBore(ctx)` returns immediately when SketchOnly is true or BoreDiameter is at most zero.
Otherwise create `Bore Profile` on `ctx.plane`, retain it as `self.boreSketch`, construct
`SpurGearInvoluteToothDesignGenerator(boreSketch, self)`, and call
`toothGen.drawBore(ctx.anchorPoint, boreDiameterCm)`. `drawBore` calls
`boreSketch.project(ctx.anchorPoint).item(0)` and centers its solid bore circle on that
projected `SketchPoint`, using `boreSketch.sketchCurves.sketchCircles.addByCenterRadius(projectedAnchor, boreDiameterCm/2)`.
Give it a driving diameter with `boreSketch.sketchDimensions.addDiameterDimension(circle, textPoint)`
whose `.parameter.value` is the current diameter in cm. The tooth-generator constructor also
created its otherwise unused local-origin point; constrain it with
`boreSketch.geometricConstraints.addCoincident(toothGen.anchorPoint, projectedAnchor)` so the
whole sketch is fully constrained [SPUR-F-ANCHOR-CHAIN] [SPUR-F-LOCAL-ORIGIN]
[PB-DRIVING-DIM]. Do not use `boreSketch.originPoint`.

The bore circle is this sketch's only closed loop; the projected anchor and stray local-origin
point create no other profile. Select `boreSketch.profiles.item(0)` directly, with no curve-type
or loop-count filter [PB-SINGLE-PROFILE]. Use that profile to call
`extrudeFeatures.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`,
set the one-sided extent to `ToEntityExtentDefinition.create(ctx.extrusionExtent, False)` in
`PositiveExtentDirection`, set `participantBodies` to `[ctx.gearBody]`, then call
`extrudeFeatures.add(extrudeInput)`. The far cap makes this a through cut at any Thickness.
The tested example in `proof/examples/bore_profile_example_test.go` guides the proof's
substitute: draw an outer root circle and inner bore circle in one sketch, select the valid
one-hole annulus, and extrude it. The proof asserts hole count, annulus area, solid validity,
and excluded volume across no-bore and positive-bore cases. This substitution proves final
root-disc bore geometry but does not prove Fusion's separate cut, participant selection, or
timeline order. Those remain specified above and require the Fusion generation test.

**Proof:** `stepBore` and `assertBore` in `proof/spurgear/solids_test.go`.

<!-- proof-run: proofkit3d.RunSolid(boreCases, stepBore, assertBore) -->

**From:** `spec/spurgear/instructions.md:537-556`; `spec/spurgear/fusion.md:19-42,215-225`;
`.claude/skills/generate-gear/PLAYBOOK.md:441-472,494-500,614-635`;
`proof/examples/OPERATIONS.md:12,25-36`.

## 10 `[PROSE]` Chamfer the completed gear

`chamferTeeth(ctx)` returns when SketchOnly is true or ChamferTooth is zero. Otherwise scan
every planar face of `ctx.gearBody` parallel to the Gear Profile plane. Collect each end-cap
edge once by `edge.tempId`, including tooth flanks, tooth tops, and root arcs. Exclude only
`Circle3DCurveType` edges with radius within `0.001` cm of positive BoreDiameter/2. Raise
if no end-cap face or chamfer edge remains; do not make a partial chamfer. Call
`chamferFeatures.createInput2()`, then
`chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, adsk.core.ValueInput.createByReal(chamferCm), False)`,
then `chamferFeatures.add(chamferInput)` [PB-FILLET-CHAMFER]
[PB-NUMERIC-SNAPSHOT]. The shared examples have no tested chamfer substitute preserving this
end-cap edge selection, so the final Fusion test must check the feature.

**From:** `spec/spurgear/instructions.md:566-573`;
`.claude/skills/generate-gear/PLAYBOOK.md:569-575`.

## 11 `[PROSE]` Hide consumed construction and sketches

Call `cleanup(ctx)` after chamfer. Set `isLightBulbOn = False` on the normalized target plane
if one was created, `ctx.extrusionEndPlane`, and `ctx.centerAxis`, guarding each absent entity.
When SketchOnly is false, set `isVisible = False` on Tools, Gear Profile, and Bore Profile
sketches, again guarding optional entities. Leave Tools and Gear Profile visible for inspection
when SketchOnly is true [SPUR-F-CLEANUP] [PB-HIDE-AFTER-USE]. No proof-engine display or
Fusion timeline visibility exists for this step.

**From:** `spec/spurgear/instructions.md:288-312,568-573`;
`spec/spurgear/fusion.md:215-225`; `.claude/skills/generate-gear/PLAYBOOK.md:657-670`.
