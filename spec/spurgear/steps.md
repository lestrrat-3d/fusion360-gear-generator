The proof files are `proof/spurgear/sketches_test.go`, `proof/spurgear/solids_test.go`, and `proof/spurgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

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

## 1 `[PROSE]` Normalize target plane

In SpurGearGenerator.generate, normalize self.plane before creating the context. Immediately after
obtaining ctx from the overridable newContext method, assign `ctx.plane = self.plane`, then run prepareTools.
The inherited generate method performs this assignment for every context, including contexts returned
by subclass overrides, so each downstream step receives the normalized ConstructionPlane.

Fusion dialog registration, occurrence context, and construction-plane handles have no decad timeline analogue.
The neighboring sketch and prism steps prove local geometry and extrusion extent, not these Fusion handles.

#### Spur Gear Creation Instructions

This file states the gear design and geometry. The sidecar fusion.md defines the Fusion API
recipes, and the shared PLAYBOOK.md defines cross-gear rules. Follow their cited anchors.
spec/spurgear/exact_values.json owns dialog IDs, labels, prompts, filters, defaults, and
parameter names, units, comments, and expressions. The tables below explain those checked
values; the exact-value source determines their generated Python representation.

#### Component Setup

A spur gear is a single cylindrical body with straight teeth cut along the axis. Unlike the bevel generator
there is no pairing — one invocation of the command produces exactly one gear. The new gear is added as a
child occurrence of the user-selected Parent Component.

#### Architecture

Spur **opts into the playbook's four-class pattern** (PLAYBOOK "The four-class pattern") and into
base.Generator / GenerationContext. The module defines exactly these four classes; the names
are public API (helical/herringbone subclass all of them by name, and commands/spurgear/entry.py
binds two):

1. **SpurGearCommandInputsConfigurator** — a plain class (no base) with @classmethod def
   configure(cls, cmd) that adds the dialog inputs (see "Exact input ids and parameter-name
   strings"). Extension seam for subclasses: [SPUR-SUBCLASS-INPUT].
2. **SpurGearGenerationContext(GenerationContext)** — the data carrier whose __init__ declares
   the fields in "Generation Context — canonical field names", each cast(None)-initialised
   (toothProfileIsEmbedded starts False).
3. **SpurGearInvoluteToothDesignGenerator** — a plain class (no base), constructed as
   (sketch, parent, angle=0); draws the circles, the involute tooth, and the bore circle. Its
   reproduced surface is pinned in "Tooth generator … reproduced surface" below.
4. **SpurGearGenerator(Generator)** — the orchestrator, subclass of base.Generator. Holds
   processInputs, generate, parameter registration, and the per-step build methods (see
   "Method contract — call graph and override boundaries").

GenerationContext and Generator are imported from .base.

#### Variables

User inputs are listed below in the order they appear in the command dialog. Derived (calculated) parameters
are registered as Fusion user parameters under the SpurGear<N>_ prefix so they are visible and editable after
generation; they are listed after the inputs they depend on.

Target Plane: user-specified plane. The gear's front face is sketched on this plane; the gear body is extruded
away from it by Thickness. Any selection that isn't already a ConstructionPlane (for example a planar face) is
converted to a coplanar construction plane at generation time so sketch-profile detection is not confused by
the selected face itself.

Anchor Point: user-specified point. The gear center is aligned with this point. May be a ConstructionPoint or
a SketchPoint. The anchor point is projected into the Tools sketch on the target plane; the Gear Profile
sketch then constrains its own local origin (0,0,0) to that projected point, so changing the anchor point
downstream moves the gear.

Module: user-supplied number. Specifies the module of the gear. The dialog input is unitless
('') with default 1, and the Module user parameter is registered with units **''
(unitless) — NOT 'mm'** — so generateName renders M=1 (no unit suffix) and the derived
mm-registered expressions (PitchCircleDiameter = Module * ToothNumber, RootCircleDiameter =
PitchCircleDiameter - 2.5 * Module, …) read the unitless Module factor exactly as the proven
implementation does (Fusion accepts the unitless term inside those mm expressions).

Tooth Number: user-specified integer. Default 17.

Pressure Angle: user-specified angle. Default 20°. Stored in radians in the user parameter PressureAngle.

Bore Diameter: user-specified positive number in mm. Default 0mm (i.e. no bore). When > 0, a cylindrical hole
is cut through the gear body, centered on the anchor point.

Thickness: user-specified positive number in mm. Default 10mm. Axial length of the gear body.

Apply chamfer to teeth: user-specified positive number in mm. Default 0mm (i.e. no chamfer). Distance of the
equal-distance chamfer applied to the completed gear's outer end-cap edges.

Generate sketches, but do not build body: user-specified boolean, default false. When true, stop after the
Gear Profile sketch is drawn (no extrude, no pattern, no fillet, no bore, no chamfer). Useful for inspecting
the involute construction.

Parent Component: user-specified component. Defaults to the root component (pre-selected). Listed last in the
dialog because the default is correct for most uses; the user only touches it when nesting the gear inside an
existing assembly. The new gear occurrence lives as a child of this component.

Pitch Circle Diameter: calculated number. Module * Tooth Number.

Pitch Circle Radius: calculated number. Pitch Circle Diameter / 2.

Base Circle Diameter: calculated number. Pitch Circle Diameter * cos(Pressure Angle). The circle the involute
flank unrolls from.

Base Circle Radius: calculated number. Base Circle Diameter / 2.

Root Circle Diameter: calculated number. Pitch Circle Diameter - 2.5 * Module. The circle at the bottom of the
tooth valleys (dedendum = 1.25 · Module).

Root Circle Radius: calculated number. Root Circle Diameter / 2.

Tip Circle Diameter: calculated number. Pitch Circle Diameter + 2 * Module. The circle that caps the tops of
the teeth (addendum = 1.0 · Module).

Tip Circle Radius: calculated number. Tip Circle Diameter / 2.

Involute Steps: calculated integer. 15. The involute flank is drawn as a fitted spline through this many
sampled involute points between the base and tip circles.

Tooth Space Angle At Root: the angular width of the valley between adjacent teeth at the root circle, in
radians. Registered as a user parameter (so it stays visible and editable), but with a pre-evaluated numeric
value rather than a live Fusion expression — Fusion's expression engine refuses to mix the unitless output of
tan() with the radian-valued Pressure Angle in a subtraction, so we compute it in Python. The value is π /
Tooth Number − 2 · (tan(Pressure Angle) − Pressure Angle). **Register this parameter as unitless (units ''),
not 'rad'** — even though it holds a radian value. The next parameter multiplies it by a length (Root Circle
Radius), and Fusion only accepts that product as a length (mm) if this factor is unitless; registering it as
'rad' makes the product mm·rad and Fusion rejects the dependent parameter with RuntimeError: Invalid
expression. (Treating a radian magnitude as a unitless number is correct — radians are dimensionless.)

Tooth Space Arc At Root: calculated number, registered in mm. Live Fusion expression Root Circle Radius *
Tooth Space Angle At Root. The arc length of that valley along the root circle. This is why the factor above
must be unitless — so this product reads as a pure length.

Fillet Clearance: calculated number. 0.9. Fraction of the half-valley arc used for the fillet radius. A value
of 1.0 would make fillets from adjacent flanks meet at the midpoint of the valley (fully rounded root); 0.9
leaves a small flat strip.

Fillet Radius: calculated number. (Tooth Space Arc At Root / 2) * Fillet Clearance * 1. The * 1 factor is a
hook used by helical/herringbone subclasses to multiply by cos(Helix Angle) so the fillet reads correctly on
the transverse plane of a tilted tooth. For a spur gear the factor is always 1.

#### Exact input ids and parameter-name strings

The IDs and parameter names are part of saved designs. Read them from
spec/spurgear/exact_values.json and spec/spurgear/contract.json; the checked handoff carries
them into generated code. Do not maintain a second literal table here.

**Five overridable methods must carry their return annotation, because a subclass narrows on
them.** helicalgear and herringbonegear override these and annotate their own returns. Python
does not care, but a type checker reads an unannotated parent as returning the literal it happens
to return — prefixBase becomes Literal['SpurGear'] — so a subclass annotating the wider str
is then reported as an incompatible override. Write these five annotations:

| class | method | return |
|---|---|---|
| SpurGearGenerator | prefixBase | -> str |
| SpurGearGenerator | generateName | -> str |
| SpurGearGenerator | filletHelixFactorExpression | -> str |
| SpurGearGenerator | newContext | -> SpurGearGenerationContext |
| SpurGearInvoluteToothDesignGenerator | getParameterValue | -> float |

A regeneration that dropped all five made helicalgear draw two reportIncompatibleMethodOverride
complaints that no shipped gear had produced before. These annotations are contract surface for the
subclasses, not implementation taste.

**The root-fillet face search needs a radius tolerance, and it is 0.0001 cm.** Step 13 selects
every cylindrical face whose radius equals the Root Circle Radius. Floating-point radii never
compare exactly equal, so the test is abs(face.geometry.radius - rootRadius) <= 0.0001. The value
matches utilities.find_circle_by_radius's own default, so the two ways of finding a circle in this
codebase agree. Step 13 already pins 0.01 for the axis-direction dot product; this is the other
tolerance that step needs and it was previously left to the implementer.

**Every registered parameter's comment is reproduced surface.** addParameter shows that
string in Fusion's parameter table. The source JSON owns all twenty parameter units and
comments, and the renderer writes them into the module.

**The three selection prompts are reproduced surface.** The third addSelectionInput argument
is shown while the user picks and differs from the label. The source JSON owns every prompt.

**Dialog display order is fixed by the source JSON array.**

This is the order the inputs are listed in the Variables section above, and configure() must
call its add*Input(...) methods in exactly this sequence. **Do not reorder by input *type*** (e.g.
all selections together, all value inputs together). In particular: Target Plane and Anchor Point
are the **first two** inputs, and Parent Component is **last** (its default — the root component —
is correct for most uses, so it sits at the bottom). **This display order is independent of, and
must not be confused with, the processInputs *read* order** — processInputs reads the three
selection inputs first to dodge the occurrence-context shift (see Generation Order), but that
read-order has no bearing on where the inputs appear in the dialog. A generator that puts the
selections last in configure() because they are "read first" has the rule backwards.

Bore Diameter remains a string input so it accepts expressions. Numeric defaults are recorded
in the JSON with explicit conversion into Fusion's internal units: centimetres for lengths and
radians for angles, regardless of the display unit.

The JSON supplies each selection's filters, one-selection limit, and root-component preselection.

SketchOnly is persisted as a
real-valued user parameter (1 = true, 0 = false), since the framework only reads numeric
parameters as booleans (getParameterAsBoolean). The derived parameters in the list above
(Pitch/Base/Root/Tip circles, InvoluteSteps, ToothSpace…, Fillet…) keep exactly those names.

[SPUR-EXPORTED-CONSTANTS] **Module-level constants are public API.** Every input id and
user-parameter name above is exported as a module-level constant in spurgear.py, and dependent
modules import them by name — renaming any of these identifiers is a breaking change. The full
roster:

- Input ids: INPUT_ID_PARENT, INPUT_ID_PLANE, INPUT_ID_ANCHOR_POINT, INPUT_ID_MODULE,
  INPUT_ID_TOOTH_NUMBER, INPUT_ID_PRESSURE_ANGLE, INPUT_ID_BORE_DIAMETER,
  INPUT_ID_THICKNESS, INPUT_ID_CHAMFER_TOOTH, INPUT_ID_SKETCH_ONLY.
- Parameter names: PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_PRESSURE_ANGLE,
  PARAM_BORE_DIAMETER, PARAM_THICKNESS, PARAM_CHAMFER_TOOTH, PARAM_SKETCH_ONLY,
  PARAM_PITCH_DIAMETER, PARAM_PITCH_RADIUS, PARAM_BASE_DIAMETER, PARAM_BASE_RADIUS,
  PARAM_ROOT_DIAMETER, PARAM_ROOT_RADIUS, PARAM_TIP_DIAMETER, PARAM_TIP_RADIUS,
  PARAM_INVOLUTE_STEPS, PARAM_TOOTH_SPACE_ANGLE, PARAM_TOOTH_SPACE_ARC,
  PARAM_FILLET_CLEARANCE, PARAM_FILLET_RADIUS.

helicalgear.py and herringbonegear.py each import PARAM_MODULE, PARAM_TOOTH_NUMBER,
PARAM_THICKNESS from .spurgear (helical additionally imports the four classes; see
"Dependencies and dependents").

[SPUR-SUBCLASS-INPUT] **Configurator extension seam (for subclasses).** configure() is a
@classmethod on SpurGearCommandInputsConfigurator. A subclass gear (helical/herringbone) adds its
own extra dialog input by **subclassing the configurator and appending after super().configure(cmd)**
— e.g. HelicalGearCommandConfigurator.configure calls super().configure(cmd) then
cmd.commandInputs.addValueInput(...) for its Helix Angle. Because super().configure() already
added Parent Component **last**, a subclass's extra input necessarily lands **after** Parent Component

in the dialog. (This is the actual behavior; the "Parent Component last" rule above is a spur-base
statement that a subclass's appended inputs sit below.)

#### Sketch Discipline

A few rules apply across every sketch created below. They're not obvious from the step list, and
the whole construction falls apart without them. The Fusion-API mechanics are in fusion.md and
PLAYBOOK.md; this section states the *intent* and points to the binding rule for each.

The Gear Profile constraint scheme below is **proven to fully constrain** (DOF == 0, no
redundant/conflicting constraints, across a size sweep) in proof/spurgear/sketches_test.go
before any Fusion code is generated — the sketch-first gate [PB-SKETCH-FIRST]. That proof is the
executable check that these rules add up to a fully-constrained sketch; run it through proof/run.sh when
changing any of them.

**The scheme is parametric, and the regime it has to hold across is part of the design.** Proving
one gear proves nothing about the next one, so a check of these rules sweeps the regime below.
Each item is here because the scheme fails differently outside it. A check that cannot reach part
of the regime says which part and why, next to the thing it cannot reach — the bench's own
excluded case is its Scope note, and its ill-conditioning finding is why that exclusion stands.

- **Size.** Several Module and Tooth Number pairs, coarse and fine, because the rib chain's
  dimensions scale with the tooth and the conditioning of the system does not.
- **The whole signed range of the angle argument.** draw(anchorPoint, angle) is called at 0 by
  spur, at the user's Helix Angle by helical and herringbone — which is **negative for a left-hand
  helix**, and the dialog input accepts a negative value — and at 180° by the bevel virtual tooth.
  A **negative** angle has to be swept alongside a positive one: the confirming angular dimension
  [SPUR-F-ROTATE-CONFIRM] carries the sign, so a scheme that drops or flips it still solves at
  +angle and comes out mirrored at −angle. Sweep a quarter turn as well, where |sin| > |cos|
  swaps which axis the rib and chain dimensions take ([SPUR-F-RIBS]).
- **The rib count.** One rib, one across-spine dimension and one chain dimension exist **per
  involute sample**, so Involute Steps sets how many constraints the sketch carries. The scheme
  has to hold at the low end of that count as well as at the standard 15 — a handful of samples is
  the case where a single missing or redundant dimension is a large fraction of the system.
- **Both routes into the embedded shape.** The profile is embedded when the base circle falls
  inside the root circle, which by the formulas above is Tooth Number · (1 − cos(Pressure Angle))
  > 2.5. Either factor can carry it: a **high tooth count** at the ordinary 20° pressure angle,
  or a **moderate tooth count at a large pressure angle**. Reach it both ways. The two arrive at
  the same missing-stub geometry through different terms, and a derivation that fixes one factor
  gets the branch wrong for the other.

**The Gear Profile sketch closes exactly two regions, and their curve counts are a contract.** The
tooth section and the disc inside the root circle are what the two extrude steps consume, and each
is found by matching the count of curves that bound it (step 7 and step 9 below spell the counts
out, and find_profile_by_curve_counts matches on nothing else). Both loops exist only because the
tooth meets the root circle and splits it in two, at the ends of the flank-to-root lines or, in the
embedded case, where the flanks themselves cross it. A sketch that closes neither region, or closes
them with different counts, is therefore a broken *sketch* and not a later step's problem, and a
check of this scheme counts the curves on the two loops of the sketch it actually drew rather than
on a simplified stand-in.

- **Sketches follow the user's anchor through a projection chain.** The Tools-sketch projection of
  the anchor is the canonical handle; later sketches re-project it so the whole gear tracks the
  anchor if it moves — see [SPUR-F-ANCHOR-CHAIN].
- **Each anchor-following sketch keeps its own movable local origin** (a fresh (0,0,0)
  SketchPoint, not sketch.originPoint); all geometry is drawn relative to it, then anchored in
  step 5 — see [SPUR-F-LOCAL-ORIGIN].
- **Every adjacency in the tooth profile loop is a *shared* SketchPoint** (so the loop is
  recognised as a closed profile) — share the point object, never re-coincident a fresh one
  ([PB-SHARE-XOR-COINCIDENT], applied in [SPUR-F-SHARED-ADJACENCY]).
- **A requested rotation is drawn *and* confirmed** — pre-rotate the geometry in Python *and*
  set the confirming angular dimension last; both are required — see [SPUR-F-ROTATE-CONFIRM].
- **Hide each entity with the right property, after it's consumed** — isVisible=False for
  sketches, isLightBulbOn=False for construction planes/axes ([PB-HIDE-AFTER-USE]); the
  spur cleanup recipe (which entities, the per-mode split) is [SPUR-F-CLEANUP].
- **Dimensions are driving by default** — never pass isDriven=True ([PB-DRIVING-DIM]). All
  diameter dimensions here (the four gear circles and the bore circle) must be driving. The
  tooth-top arc carries no diameter dimension at all; it shares the local origin as its centre
  instead ([SPUR-F-TOOTHTOP-ARC]).
- **Every sketch here is fully constrained** ([PB-FULL-CONSTRAINT]), with no exceptions. Every
  sketch's local origin rides on the projected anchor, including the Bore Profile sketch's, whose
  local origin the tooth generator creates and nothing else uses. See step 12.
- **Dimensions and feature inputs are numeric snapshots** — editing a <prefix>_… parameter does
  not change an existing gear; regenerate ([PB-NUMERIC-SNAPSHOT], spur application
  [SPUR-F-SNAPSHOT]).

#### Generation Context — canonical field names

The SpurGearGenerationContext object is passed between generation steps and read by subclasses (helical,
herringbone). Subclasses reach in by name, so these field names are part of the public API of this module —
don't rename them when reconstructing:

- ctx.plane — the ConstructionPlane all sketches are built on (normalised in step 1).
- ctx.anchorPoint — the SketchPoint that is the Tools-sketch projection of the user's anchor. Later sketches
project *this* in again to chain to the user's original anchor entity.
- ctx.extrusionEndPlane — the offset construction plane used as the to-entity target for the tooth and body
extrudes.
- ctx.gearProfileSketch — the sketch containing the tooth profile + four gear circles.
- ctx.toothBody — the single extruded tooth, before the circular pattern.
- ctx.gearBody — the cylindrical body the teeth are joined into.
- ctx.centerAxis — the Gear Center construction axis built off the body's cylindrical face.
- ctx.extrusionExtent — the far end-cap face of the gear body, used as the to-entity for the bore cut.
- ctx.toothProfileIsEmbedded — True iff the base circle sits inside the root circle (no flank-to-root stubs
drawn); used by the tooth extrude step to pick the right profile-curve count.

(The active plane is also held on the generator as self.plane — normalised in step 1 — and
subclasses read self.plane directly; keep both available.)

#### Method contract — call graph and override boundaries

These method names **and the boundaries between them** are public API: helicalgear.py and
herringbonegear.py subclass SpurGearGenerator and override specific methods, calling
super() at specific points. A reconstruction that merges or reorders these steps — even if it
draws an identical spur gear — breaks the helical and herringbone gears. Keep the call graph
exactly as below; only the *contents* of each method (local variables, comments, further private
helpers) may vary.


generate(inputs)
  → processInputs(inputs)
  → name the component (generateName)
  → normalize self.plane to a ConstructionPlane
  → ctx = newContext()
  → ctx.plane = self.plane
  → prepareTools(ctx)            # Tools sketch + ctx.anchorPoint + ctx.extrusionEndPlane (steps 1–2)
  → buildMainGearBody(ctx)
        → buildSketches(ctx)     # Gear Profile sketch; runs the tooth generator (steps 3–5)
        → if SketchOnly: show the Gear Profile sketch and stop (step 6)
          else:
            → buildTooth(ctx)    # extrude the tooth → ctx.toothBody (step 7)
            → buildBody(ctx)     # extrude the annular body → ctx.gearBody, centerAxis, extrusionExtent (step 9)
            → patternTeeth(ctx)  # circular pattern + combine (step 10)
            → createFillets(ctx) # root fillets (step 11)
  → buildBore(ctx)               # optional bore (step 12)
  → chamferTeeth(ctx)            # optional completed-gear chamfer (step 13)
  → cleanup(ctx)                 # always: hide construction planes/axes; sketches hidden only when NOT SketchOnly


**cleanup(ctx) is the very last action of generate() — after chamferTeeth, not inside
buildMainGearBody.** Call it **unconditionally** (in both modes); the SketchOnly distinction
lives *inside* cleanup — the recipe (which entities, the per-mode split) is owned by
[SPUR-F-CLEANUP]. Placement after
buildBore matters because buildBore re-projects ctx.anchorPoint from the Tools sketch and
projection fails once that sketch is hidden — so the Tools sketch must stay visible through the
bore and chamfer. Do not move cleanup up into buildMainGearBody, and do not guard the *call*
(guard the sketch-hiding inside it instead).

Specific boundaries subclasses depend on (do not move the work elsewhere):

- **buildSketches(ctx)** owns creating the Gear Profile sketch and invoking the tooth
  generator. Helical overrides it, calls super().buildSketches(ctx), then draws a *second*
  twisted profile sketch with SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=helixAngle).
- **buildTooth(ctx)** owns turning the profile into ctx.toothBody. Helical overrides it to loft
  instead of extruding, and herringbone to loft and mirror. It does not apply a chamfer.
- **chamferTeeth(ctx)** runs from generate after buildBore, so it sees the patterned and
  filleted gear and an optional bore. It is shared unchanged by spur, helical, and herringbone.
- **filletHelixFactorExpression()** is an overridable hook. It is **not** read by
  createFillets: it returns an **expression string** (spur base: '1';
  helical: 'cos(<prefix>_HelixAngle)') that is consumed exactly once, in
  registerDerivedParameters, where it is spliced in as the last factor of the live FilletRadius
  parameter expression ((ToothSpaceArcAtRoot / 2) * FilletClearance * <factor>). createFillets
  then reads only the resulting FilletRadius parameter's numeric .value.
- [SPUR-EXTRA-PARAMS] **addExtraPrimaryParameters(self, inputs)** is an overridable hook, a **no-op
  on the spur base**, that processInputs calls **between** registering the input-sourced parameters
  and the derived ones. Subclasses override it to register their own primary user parameters from the
  extra dialog inputs they added (e.g. helical registers HelixAngle from the helixAngle input).
  It must exist (as a no-op) on the spur base so the call site in processInputs is present for
  subclasses to hook. Together with [SPUR-SUBCLASS-INPUT], these are the two seams by which a
  subclass adds a parameter: the configurator adds the dialog *input*, this hook registers the
  *parameter*.
- **generateName()** returns the component name. For the spur base it is
  'Spur Gear (M={}, Tooth={}, Thickness={})'.format(module.expression, toothNumber.expression, thickness.expression)
  — i.e. the Module, ToothNumber, and Thickness parameters' **.expression** strings (not
  .value), so units show through (e.g. Spur Gear (M=1, Tooth=17, Thickness=10 mm)). Subclasses
  override this to read their own parameters.

#### Tooth generator (SpurGearInvoluteToothDesignGenerator) reproduced surface

- Constructor (sketch, parent, angle=0). Store the constructor angle as self.toothAngle =
  angle. **This stored value is NOT what drawTooth rotates by** — see the next bullet. (It is
  retained only as an incidental field; the live rotation always comes from draw()'s runtime
  argument. Do not use self.toothAngle inside drawTooth.)
- The movable **local origin is a field named self.anchorPoint** — a fresh SketchPoint added
  at (0, 0, 0) in the constructor (see Sketch Discipline). Subclasses don't read it directly, but
  the spur base buildSketches and the draw() anchoring below depend on this exact name.
- draw(anchorPoint, angle=0) performs, in order: drawCircles(), drawTooth(angle), then the
  **step-5 anchoring**, then — as the very last action, *after* the anchoring — sets the
  confirming angular dimension's value to angle (only when angle != 0; this last action is
  [SPUR-F-ROTATE-CONFIRM]'s "very last action, after the entire constraint network exists").
  **drawTooth MUST rotate by the
  angle argument that flows in from draw() at call time — NOT by the constructor-stored
  self.toothAngle.** This is load-bearing for subclasses: helical/herringbone construct the
  generator with the default angle=0 (so self.toothAngle == 0) and then call
  draw(ctx.anchorPoint, angle=helixAngle). If drawTooth used self.toothAngle it would draw a
  flat tooth and the helical loft would have no twist.
- Methods drawCircles, drawTooth, drawBore, and calculateInvolutePoint(baseRadius,
  intersectionRadius) must all exist. The tooth generator also exposes parameter accessors
  getParameter(name) and getParameterValue(name) (these names are part of the reproduced
  surface). drawBore(anchorPoint, diameter) takes the anchor entity and the bore diameter (cm),
  projects the anchor into the sketch, draws the bore circle of that diameter centered on the
  projection with a driving diameter dimension, and returns the circle.
- When a drawing step needs one of the four circles drawn by drawCircles (the tip circle for
  the tooth-top point, the root circle for the flank-to-root lines), either keep direct references
  from drawCircles or locate it with the framework's find_circle_by_radius(sketch, radius)
  (from .utilities) — never fall back to an arbitrary circle on a failed radius match.
- **calculateInvolutePoint(baseRadius, intersectionRadius) — exact math** (this fully pins the
  flank shape; do not infer it). Returns the point on the involute of baseRadius at the radius
  where the unrolled string reaches intersectionRadius; returns None when intersectionRadius
  < baseRadius (the sample sits inside the base circle — this is the "non-positive involute
  parameter" case the sampling loop drops):
  
  alpha = acos(baseRadius / intersectionRadius)
  t     = tan(alpha)          # the curve parameter is tan(alpha) — NOT inv(alpha)=tan(alpha)-alpha
  x = baseRadius * (cos(t) + t * sin(t))
  y = baseRadius * (sin(t) - t * cos(t))
  
  Using inv(alpha) = tan(alpha) − alpha as the parameter instead of tan(alpha) is a common
  mistake and produces a wrong (mis-parameterised) flank.

#### Dependencies and dependents

Spur imports only the framework (.base — Generator, GenerationContext, get_value, get_boolean,
get_selection; .utilities — get_normal, find_profile_by_curve_counts; .misc — to_cm,
get_design). It depends on no other gear. Two dependents bind to its surface; regenerating spur
must not break either:

- **helicalgear.py / herringbonegear.py subclass the four classes.** Helical imports
  SpurGearCommandInputsConfigurator, SpurGearGenerationContext, SpurGearGenerator, and
  SpurGearInvoluteToothDesignGenerator (herringbone subclasses helical's versions), and both
  import the module-level constants PARAM_MODULE, PARAM_TOOTH_NUMBER, PARAM_THICKNESS
  ([SPUR-EXPORTED-CONSTANTS]). Everything they lean on is pinned in "Method contract" and
  "Generation Context" above.
- **bevelgear.py borrows the tooth generator without the Fusion parameter table.** It constructs
  SpurGearInvoluteToothDesignGenerator(toothSketch, proxy) where proxy is a
  spurproxy.VirtualSpurProxy — a fake parent whose getParameter(name) serves precomputed
  values in internal cm ([PB-PRECOMPUTED-MODE]). **Borrowing constraint:** inside drawCircles,
  drawTooth, and draw (including the helpers they call, e.g. _drawFlankToRoot), parameters
  may be read ONLY from the key set VirtualSpurProxy serves — Module, ToothNumber,
  PressureAngle, PitchCircleDiameter, PitchCircleRadius, BaseCircleDiameter,
  BaseCircleRadius, RootCircleDiameter, RootCircleRadius, TipCircleDiameter,
  TipCircleRadius, InvoluteSteps. Reading any other key on those paths breaks the bevel build
  (the proxy raises KeyError). The proxy also carries the _lastToothEmbedded output slot: the
  tooth generator's drawTooth writes self.parent._lastToothEmbedded (see
  [SPUR-F-FLANK-ROOT]), and bevel reads it back off the proxy after draw() — keep that output
  write in place.

#### Generation Order

The 12 steps below are preceded by a dialog-reading pass. The order matters for one specific reason: as soon
as you call parentComponent.occurrences.addNewComponent(...) (directly via getOccurrence(), or indirectly via
the first addParameter() / parameterName() call), Fusion's active component context shifts to the newly
created occurrence. SelectionCommandInputs holding entities that live in a *different* component — for example
a SketchPoint on a sketch in the root component, while the new gear is being added under the root — can drop
their selections when that context shift happens. Numeric and boolean inputs are unaffected.

So the rule is: pull every selection input (Parent, Target Plane, Anchor Point) out of inputs and stash the
entities on self *before* triggering occurrence creation. The order inside generate() is therefore:

1. Read Parent, Target Plane, Anchor Point from inputs. Don't call anything that touches the design yet.
2. Now getOccurrence() (or a parameter registration that calls it transitively).
3. Register input-sourced and derived user parameters from the still-live numeric inputs.
4. Run the 12 steps below in order.

#### Instructions


#### 1: Normalize the Target Plane

If the user-selected plane is not already a ConstructionPlane (for example they picked a planar face of an
existing body), create a coplanar construction plane via ConstructionPlaneInput.setByOffset(selectedPlane,
adsk.core.ValueInput.createByReal(0)) and use that for all subsequent operations. The offset argument is a
**ValueInput, not a bare number** — setByOffset(plane, 0) is a runtime TypeError ([PB-CONSTRUCTION-PLANES]
gives the signature). The same applies to the Extrusion End Plane in step 2: its Thickness offset is passed as
ValueInput.createByReal(thickness). This keeps the downstream profile-detection code from having to filter out
the selected face's native profile.

[PB-INPUT-READ] [PB-GET-VALUE-CONTRACT] [PB-DIALOG-DEFAULT-UNITS] [PB-SELECTION-DECL] [PB-SELECTION-FILTER-ENUM] [PB-SELECTION-STASH] [PB-CONSTRUCTION-PLANES] [PB-NUMERIC-SNAPSHOT] [PB-PRECOMPUTED-MODE]

Required call: `planes.createInput()`.

Required call: `planeInput.setByOffset(selectedPlane, zero)`.

Required call: `adsk.core.ValueInput.createByReal(0)`.

Required call: `planes.add(planeInput)`.


Required call: `inputs.addSelectionInput(id, label, prompt)`.

Required call: `selection.addSelectionFilter(filterConstant)`.

Required call: `selection.setSelectionLimits(1, 1)`.

Required call: `selection.addSelection(rootComponent)`.

Required call: `inputs.addValueInput(id, label, unit, defaultValue)`.

Required call: `inputs.addBoolValueInput(id, label, True, '', False)`.

Required call: `inputs.addStringValueInput(id, label, defaultText)`.

Required call: `adsk.core.ValueInput.createByString(expression)`.

The generated Exact values and Compilation contract sections supply the dialog and parameter literals.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "planes",
      "role": "required",
      "span": "planes.createInput()"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(selectedPlane, zero)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(0)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "planes",
      "role": "required",
      "span": "planes.add(planeInput)"
    },
    {
      "condition": null,
      "name": "addSelectionInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.addSelectionInput(id, label, prompt)"
    },
    {
      "condition": null,
      "name": "addSelectionFilter",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selection",
      "role": "required",
      "span": "selection.addSelectionFilter(filterConstant)"
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
      "name": "addSelection",
      "owner": "adsk.core.SelectionCommandInput",
      "reason": null,
      "receiver": "selection",
      "role": "required",
      "span": "selection.addSelection(rootComponent)"
    },
    {
      "condition": null,
      "name": "addValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.addValueInput(id, label, unit, defaultValue)"
    },
    {
      "condition": null,
      "name": "addBoolValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.addBoolValueInput(id, label, True, '', False)"
    },
    {
      "condition": null,
      "name": "addStringValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.addStringValueInput(id, label, defaultText)"
    },
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByString(expression)"
    }
  ],
  "citations": [
    {
      "first": 1,
      "last": 175,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 1,
      "last": 37,
      "path": "spec/spurgear/exact_values.json"
    },
    {
      "first": 1,
      "last": 152,
      "path": "spec/spurgear/contract.json"
    },
    {
      "first": 252,
      "last": 286,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L1–175; `spec/spurgear/exact_values.json` L1–37; `spec/spurgear/contract.json` L1–152; `spec/spurgear/instructions.md` L252–286.

## 2 `[GO]` Tools sketch

Create a sketch named Tools on the target plane. sketch.project(anchorPoint) returns an
ObjectCollection, even for one point. Take .item(0) from that collection and keep the resulting
SketchPoint as ctx.anchorPoint — this is the canonical handle every later sketch will re-project
from (see Sketch Discipline). The sketch draws no geometry of its own; it exists to own this one



[SPUR-F-ANCHOR-CHAIN] [PB-HIDE-AFTER-USE] [PB-SKETCH-FIRST]

Required call: `sketch.project(anchorPoint)`.

Required call: `projection.item(0)`.

Proof function: `stepTools`.

<!-- proof-run: proofkit.Run(sketchCases, stepTools) -->


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.project(anchorPoint)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "projection",
      "role": "required",
      "span": "projection.item(0)"
    }
  ],
  "citations": [
    {
      "first": 424,
      "last": 428,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L424–428.

## 3 `[PROSE]` Extrusion end plane

The proof models this plane through the common prism extent; it does not model its Fusion timeline handle.

Create Extrusion End Plane at Thickness from the target plane and store ctx.extrusionEndPlane.

isVisible = False once the gear is fully built.

Create the offset with Thickness in centimetres; keep its construction light bulb on until cleanup.

[PB-CONSTRUCTION-PLANES] [PB-NUMERIC-SNAPSHOT] [PB-HIDE-AFTER-USE]

Required call: `planes.createInput()`.

Required call: `adsk.core.ValueInput.createByReal(thickness)`.

Required call: `planeInput.setByOffset(ctx.plane, thicknessValue)`.

Required call: `planes.add(planeInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "planes",
      "role": "required",
      "span": "planes.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(thickness)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(ctx.plane, thicknessValue)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionPlanes",
      "reason": null,
      "receiver": "planes",
      "role": "required",
      "span": "planes.add(planeInput)"
    }
  ],
  "citations": [
    {
      "first": 430,
      "last": 430,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L430.

## 4 `[GO]` Gear profile sketch

Create an offset construction plane Extrusion End Plane at distance Thickness from the target plane. Its only
purpose is to serve as the to-entity target for the tooth and body extrudes, so both extrudes end on the same
well-defined face. It must be left visible while those extrudes run, then hidden at the very end of the build
with isLightBulbOn = False (see Sketch Discipline — isVisible = False does **not** hide a construction plane).
Keep a handle to it (ctx.extrusionEndPlane) so the final cleanup can switch its light bulb off.

#### 3: Gear Profile Sketch

Create a sketch named Gear Profile on the target plane. Inside this sketch the Spur Gear tooth profile
generator draws, in order:

1. **Root Circle** (solid, not construction) at radius Root Circle Radius.
2. **Tip Circle** (construction), Tip Circle Radius.
3. **Base Circle** (construction), Base Circle Radius.
4. **Pitch Circle** (construction), Pitch Circle Radius.

Center every circle on the local-origin SketchPoint by passing it **directly** as the center —
sketchCircles.addByCenterRadius(localOrigin, radius) — so all four share that one point (see Sketch
Discipline: share, don't re-coincident; do not pass localOrigin.geometry and then add a center coincident).
Give each a driving diameter dimension. Each circle is also labeled with along-path sketch text (see the
playbook for the exact sketchTexts.createInput2(...) + setAsAlongPath(...) call). The label string is '{}
(r={:.2f}, size={:.2f})'.format(name, radius, size) — the circle's name, its radius, and size, all using the
radii's internal .value (cm) — where size = Tip Circle Radius − Root Circle Radius. Pass that same size as the
text **height** argument to createInput2.

#### 4: Involute Tooth

Still inside the Gear Profile sketch, draw a single involute tooth centered on the +X direction:

1. Sample a sequence of points along the involute flank, starting on the base circle and walking outward
toward the tip circle in equal radial steps (Involute Steps samples in total). The sampling is **endpoint-
inclusive**: with steps = Involute Steps, sample i (for i = 0 … steps−1) sits at radius r = Base Circle Radius
+ (Tip Circle Radius − Base Circle Radius) · i / (steps − 1), so the first sample radius is exactly Base
Circle Radius and the **last sample is exactly Tip Circle Radius**. Do **not** clamp the start to max(Base
Circle Radius, Root Circle Radius); the flank is sampled from the base circle even when the base circle sits
inside the root circle (the embedded case is detected later in step 9 from where the flank *start* lands, not
by trimming the sampling). Each sample is calculateInvolutePoint(Base Circle Radius, r) for that step's radius
r (the exact math is pinned in the tooth-generator surface section above). Drop any sample that returns None
(i.e. whose radius is below the base circle) — those sit inside the base circle and have no valid involute.
2. **Watch the spiral direction.** A correctly-formed involute tooth narrows from base to tip, so the left
flank's angular distance above +X must *decrease* as the radius grows. The standard parametric involute
(rb*(cos t + t sin t, sin t - t cos t)) spirals the opposite way — its angular position *increases* with
radius — and using it as a left flank directly produces a tooth that splays outward (wider at the tip than at
the root). Before rotating, mirror the samples across the +X axis (negate y) so the spiral matches a left
flank's shape. The rotation in the next step lifts the mirrored curve from −Y back up into +Y where the left
flank belongs.
3. Decide how far to rotate the (mirrored) sequence so the tooth ends up symmetric about +X. Measure where the
mirrored involute crosses the pitch circle, then rotate by exactly the amount that lands that pitch-circle
crossing at angle +π / (2 · ToothNumber) above +X. (The angular width of a single tooth at the pitch circle is
π / Tooth Number, so half that — π / (2 · ToothNumber) — is where the left flank's pitch crossing must end
up.) Compute the pitch-circle crossing angle **analytically** — evaluate calculateInvolutePoint(Base Circle
Radius, Pitch Circle Radius) and take its polar angle — rather than interpolating between the sampled flank
points; the analytic value places the tooth at exactly the right angle regardless of how few involute samples
are taken. **Exact expression (pin the sign — it interacts with the step-2 mirror):** with (px, py) =
calculateInvolutePoint(Base Circle Radius, Pitch Circle Radius), the *mirrored* pitch crossing sits at polar
angle atan2(−py, px), so rotate_angle = π / (2 · ToothNumber) − atan2(−py, px). (The −py is the step-2 mirror
applied to the analytic point; do not take atan2(py, px).)
4. Rotate the (mirrored) sampled points by rotate_angle. This produces the **left** flank. Mirror that result
across the X axis to produce the **right** flank. You now have a tooth symmetric about +X.
   **Then apply the requested angle.** The generator's draw(anchorPoint, angle=0) takes an angle (0 for spur; the helix angle for helical; 180° for the bevel virtual tooth) — the seed tooth must end up rotated by exactly that. Do this by rotating the **whole** +X-centered tooth by angle right here, in the same Python point math: rotate both flank point collections by angle (and, below, place the tooth-top point and seed the rib midpoints at the rotated positions too). Draw the tooth directly at its final angular position. Do **not** instead leave the tooth at +X and rely on the spine's angular dimension to swing it into place after the fact — the wrong-solver-branch failure this causes (and why it ruins the helical loft) is owned by [SPUR-F-ROTATE-CONFIRM]. Because both the bottom (angle = 0) and top (angle = helixAngle) profiles share the same rotate_angle baseline and differ by exactly angle, the loft twists by exactly the helix angle regardless of the absolute baseline. For angle = 0 this whole step is a no-op (rotating by 0).
5. Draw the two flanks as SketchFittedSplines through the point collections.
6. Draw the **tooth-top arc** — an arc through the two flank ends, capping the tooth at the tip
   circle. This step is constraint-sensitive (over-constraining it blows up later when the last
   rib's perpendicular is added). Use the exact minimal constraint set in [SPUR-F-TOOTHTOP-ARC].
7. Draw the **spine** — a construction line from the local origin to the tooth-top point, defining
   the tooth's axis of symmetry — and pin its absolute rotation so the tooth sits at angle and the
   sketch is fully constrained. The exact construction (sharing the endpoints, the +X horizontal
   reference and its required end-pin, built for every angle including 0, and the confirming angular
   dimension) is in [SPUR-F-SPINE]; the draw-and-confirm rule is [SPUR-F-ROTATE-CONFIRM].
8. Draw a **rib** construction line between each matching pair of left/right flank fit-points, with
   a midpoint on the spine; the ribs lock the flanks to the spine so the tooth rebuilds cleanly when
   Module or Tooth Number changes, without pinning any point to an absolute coordinate. **Build a
   rib for *every* fit-point index, including the first (base-circle) pair and the last (tip) pair.**
   The last rib carries no perpendicular, because the tooth-top arc already implies it
   ([SPUR-F-TOOTHTOP-ARC]); every other constraint on it is unchanged. With N
   involute samples per flank you draw N ribs; the flank fit-points carry no other constraint, so a
   missing endpoint rib leaves that fit-point free and the sketch under-constrained. The construction
   is order-sensitive — follow the exact six-step order and the midpoint-chain rule (including the
   origin-to-first-rib dimension) in [SPUR-F-RIBS].
9. Close the tooth at the root. If the flank's first point (on the base circle) lies **outside** the
   root circle, draw a short **radial** flank-to-root line on each side (exact two-constraint
   construction in [SPUR-F-FLANK-ROOT]); the tooth loop then has **6 curves** (2 splines + 2
   flank-to-root lines + 2 arcs). If the flank starts **inside** the root circle (which happens above
   2.5 / (1 - cos(PressureAngle)) teeth — 41.5 at 20°, 78.5 at 14.5°, 26.7 at 25°), no
   flank-to-root line is drawn and the loop has **4 curves** (2
   splines + 2 arcs) — the profile is "embedded." Record which shape was drawn so the extrude step
   knows which edge count to expect; the embedded-flag mechanism (the tooth generator sets
   self.parent._lastToothEmbedded, copied to ctx.toothProfileIsEmbedded in buildSketches) is
   pinned in [SPUR-F-FLANK-ROOT].

#### 5: Anchor the Sketch

This is the step that slides the whole drawing onto the user's anchor. Project the Tools-sketch
anchor into the Gear Profile sketch (this re-projection is what chains the two sketches together),
then take .item(0) from the returned ObjectCollection. Pass that SketchPoint, not the
collection, to addCoincident(self.anchorPoint, projectedAnchor) with the Gear Profile's local
origin — the sketch point the tooth generator added at (0, 0, 0) in step 4 (the field
self.anchorPoint), *not* sketch.originPoint. Because every piece of geometry above is constrained


- [SPUR-F-ANCHOR-CHAIN] **The gear tracks the user's anchor through a chain of sketch
  projections.** A sketch can't reference a SketchPoint or curve owned by another sketch, so when
  the Gear Profile or Bore Profile sketches need the user's anchor they call sketch.project(...)
  to pull it in locally. That call returns an ObjectCollection; take .item(0) to get the
  projected SketchPoint before passing it to a sketch constraint or storing it as an anchor.
  The **Tools-sketch projection is the canonical handle**; every later sketch projects *that* in
  again, forming a chain of projections all tied back to the user's
  original anchor entity — so the whole gear moves if the anchor moves later.

- [SPUR-F-LOCAL-ORIGIN] **Each sketch that must follow the anchor keeps its own movable local
  origin.** That is a fresh SketchPoint added at (0, 0, 0) — **not** sketch.originPoint, which
  is immutable and can't be coincident-constrained to anything brought in from elsewhere. The tooth
  generator draws all its geometry relative to this local origin (the field self.anchorPoint),
  then at the very end constrains it coincident with the projected anchor (step 5); Fusion then
  slides the whole sketch onto the user's anchor as a unit.

- [SPUR-F-SHARED-ADJACENCY] **Every adjacency in the tooth profile loop is a *shared*
  SketchPoint** — not two free points that happen to share coordinates. Ribs pass through the
  flank splines' fitPoints[i]; the tooth-top arc passes through the flanks' endSketchPoints;
  flank-to-root lines end at the flanks' startSketchPoints. Handing Fusion raw Point3Ds at
  matching coordinates creates *fresh* sketch points, and then the tooth loop is not recognised as
  a closed profile when the extrude step searches for it. This is [PB-SHARE-XOR-COINCIDENT]
  applied to the profile loop: pass the existing SketchPoint object directly into the creation
  call (share it), never create from .geometry and then re-coincident. The spur points anchored
  this way are the four circle centers, the spine start, and the rib chain — all on the local
  origin; piling redundant coincidents onto that shared origin is what makes the solver fail
  (VCS_SKETCH_SOLVING_FAILED) or over-constrain.

#### Rotation (shared with helical / herringbone / bevel virtual tooth)

- [SPUR-F-ROTATE-CONFIRM] **The requested rotation is drawn AND confirmed — two distinct,
  both-required actions.** When a non-zero angle is passed, the tooth geometry is drawn **already
  rotated by angle** in the Python point math (step 4 — every flank point, the tooth-top point,
  and the rib-midpoint seeds sit at their angle-rotated positions). Then, **as the very last
  action after the entire constraint network exists**, the spine-to-horizontal angular dimension's
  value is set to angle (step 7). These are NOT alternatives — do both: the pre-rotation puts the
  geometry on the correct solver branch, and the final dimension value-set *confirms and locks*
  that rotation rather than swinging the tooth into place from +X. (Drawing the tooth flat and
  relying solely on the dimension to swing it lets Fusion pick the wrong ~180°-off branch and ruins
  the helical loft — bottom profile at 0°, top ~180° away → the loft passes through the gear
  centre.) Concretely: if angle != 0: spineAngularDimension.parameter.value = angle. The angular
  dimension itself exists for **every** angle including 0, because it is what says which way the
  spine points ([SPUR-F-SPINE]); at angle = 0 it is created at 0 and there is simply nothing to
  set afterwards.

#### Per-step constraint recipes (the over-constraint-sensitive ones)

These are the spur-specific constraint constructions whose **exact set and order** matter — a
different set or order throws VCS_SKETCH_OVER_CONSTRAINTS or VCS_SKETCH_SOLVING_FAILED. (They
build on the shared [PB-FULL-CONSTRAINT], [PB-SHARE-XOR-COINCIDENT], [PB-NO-OVERCONSTRAIN],
[PB-DRIVING-DIM] rules; here is the spur application.)

- [SPUR-F-TOOTHTOP-ARC] **Tooth-top arc — centred on the local origin (step 6).** The arc caps the
  tooth at the tip circle, so it *is* part of that circle and must bulge outward. Say that by
  **putting the arc's centre on the local origin**, and add nothing else.
  1. Materialize a **tooth-top point**: a SketchPoint at the tip, **rotated by angle** to match
     the rotated flanks — (Tip Circle Radius · cos(angle), Tip Circle Radius · sin(angle)) —
     constrained **coincident to the tip circle**.
  2. Create the arc with sketchArcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint,
     leftFlankEndPoint) — pass the two flank splines' **end SketchPoints directly**, which the
     arc does share, so they need no coincidences.
  3. **Then tie the centre back:** addCoincident(arc.centerSketchPoint, localOrigin). The call
     shares the start and end points but **copies the centre** into a fresh SketchPoint
     ([PB-SHARE-XOR-COINCIDENT]), so passing localOrigin as the first argument fixes nothing.
     This is the one arc in this sketch whose centre must be coincident rather than shared, and it
     is not the redundant double-bind that rule otherwise forbids.
  4. Add **no diameter dimension**. The coincident centre and the two shared ends already determine
     the arc, and its radius follows from the flank ends being on the tip circle.

  ⚠️ Without step 3 the centre is a **free point** carrying only the arc's own equal-radius
  relation to the two flank ends, which leaves 2 DOF. Everything else in the tooth is built about
  the local origin and then dragged onto the anchor in step 5, and the stranded centre does not
  follow: it stays behind by the drag distance and the arc's radius becomes whatever the solver
  lands on. A spur gear hides this because its anchor sits at the sketch origin, so the drag is
  zero and the copied centre happens to stay put. The bevel gear drags its tooth 8–36 mm onto K′/L′
  and the arc collapses — measured in Fusion 2026-09-02 on a default 31/31 pair, a 0.5743 mm radius
  on the pinion and 17.0204 mm on the driving gear where both should have been the 22.5 mm tip
  radius, from two sketches with byte-identical constraint counts and dimension values. The bench
  carries this as a negative control: proof/spurgear/sketches_test.go runs with the centre left free,
  reports DOF=2, underconstrained, and must keep failing. The Fusion observation on 2026-09-02
  placed the pinion centre 22.9 mm behind its origin and the driving-gear centre 5.5 mm behind.
  The maintained proof checks the unconstrained state, not those historical Fusion measurements.

  ⚠️ A **free centre plus a diameter dimension** determines the arc's size but not which way it
  curves: an arc of the same radius through the same two ends can bulge inward, back through the
  tooth. The sketch then reaches DOF 0 with two valid answers and the solver picks by where the
  centre was seeded. Putting the centre on the origin removes the choice.

  ⚠️ Putting the centre on the origin makes the **last rib's perpendicular redundant** — see
  [SPUR-F-RIBS], which omits it. Keeping both is what throws VCS_SKETCH_OVER_CONSTRAINTS.

- [SPUR-F-SPINE] **Spine + +X reference + angular pin (step 7).** Draw the spine as a construction
  line addByTwoPoints(localOrigin, toothTopPoint) — pass **both** existing SketchPoints
  directly (share them). Do **not** create it from .geometry, do **not** add a separate
  start-coincident to the origin (sharing already ties it; an extra coincident makes the solver
  fail), and do **not** constrain the spine's end onto the arc (the tooth-top point already lies on
  the tip circle).

  Build the **+X reference construction line** for **every** angle, including 0:
  1. Add a far endpoint at (Tip Circle Radius, 0) and pin it with **two axis dimensions from
     the local origin** — addDistanceDimension(..., HorizontalDimensionOrientation, Tip Circle
     Radius) and the vertical one at 0; both values are non-negative magnitudes and the
     endpoint is seeded on the +X side, per [PB-DIM-VALUE-SEMANTICS]. Pin it this way rather
     than with addCoincident(end, tipCircle): a point on a circle has two answers, and pinning
     its x at the tip radius instead touches the circle at its extreme, where the numbers go
     unstable.
  2. Draw the reference line from the origin to that endpoint and mark it construction.
  3. Add an angular dimension **from the reference to the spine, in that argument order**
     (addAngularDimension(reference, spine, …)); place its text on the **bisector of the intended
     angle** ((R·cos(angle/2), R·sin(angle/2)) for small R) so Fusion selects angle, not its
     supplement. Set its value to angle as the very last action (see [SPUR-F-ROTATE-CONFIRM]).

  ⚠️ Do **not** use a plain addHorizontal on the spine for the angle = 0 case. Horizontal fixes
  the line's direction but says nothing about which way it points, so the tooth top can settle at
  either end of the tip circle and the whole tooth comes out 180° around. The angular dimension
  against a reference that is pinned to +X is what says which way, and using it for every angle
  keeps spur, helical, herringbone and the bevel virtual tooth on one path.

- [SPUR-F-RIBS] **Ribs — exact construction order (step 8).** A rib construction line runs between
  each pair of matching left/right flank points — **one per fit-point index i for all N indices,
  endpoints included** (the base-circle pair i=0 and the tip pair i=N-1 both get a rib, even
  though the tip ends are also joined by the tooth-top arc; the fit-points have no other constraint,
  so an omitted endpoint rib leaves the sketch under-constrained). Each needs a materialized
  **midpoint sketch point** on the spine. Build each rib in **this exact order** — a different order
  over-constrains the sketch (VCS_SKETCH_OVER_CONSTRAINTS):
  1. Add the rib with addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i]) — pass the
     two fit-point SketchPoints **directly** so the rib shares them; mark it construction.
  2. Dimension the rib with an **axis** dimension (horizontal/vertical), not an aligned one: for
     angle = 0 use addDistanceDimension(left, right, VerticalDimensionOrientation, …), created
     with the fit points already at their seeded positions and its value left at the measured
     magnitude — the direction is captured from the seed at creation, per
     [PB-DIM-VALUE-SEMANTICS]. For a rotated tooth, the rib takes the axis **across** the spine
     and the midpoint chain takes the one **along** it: use vertical for the rib and horizontal
     for the chain when |cos(angle)| >= |sin(angle)|, and swap both otherwise. That reduces to
     the pair above at angle = 0, and a tooth at 90° fails without it. An **aligned** dimension
     gives only the length, which the left and right flanks satisfy equally well swapped over, so
     the tooth can come out mirrored; the axis dimension's captured direction is what forbids the
     swap.
  3. Add a fresh SketchPoint for the midpoint, created **already on the spine**. The spine is the
     line at angle through the local origin, so seed the midpoint at the **foot of the left fit
     point on that line**: with t = fitX·cos(angle) + fitY·sin(angle), the seed is
     (t·cos(angle), t·sin(angle)). (For angle = 0 this reduces to (fitX, 0).) Do **not** seed
     it at the rib's true 2-D midpoint, and do **not** seed it at (fitX, 0) for a rotated tooth.
  4. addCoincident(midpoint, spine) — pin the point onto the spine **first**.
  5. addMidPoint(midpoint, rib) — then make it the rib's midpoint.
  6. addPerpendicular(spine, rib) — then make the rib perpendicular to the spine. **Skip this for
     the last rib.** That rib joins the two flank tips, which the tooth-top arc already holds at
     equal radius either side of the spine ([SPUR-F-TOOTHTOP-ARC]), so its perpendicular says
     nothing new and Fusion rejects it with VCS_SKETCH_OVER_CONSTRAINTS.

  Then dimension the distance from each rib's midpoint to the previous rib's midpoint with an
  **axis** dimension along the spine direction (horizontal for angle = 0) — and **for the first
  rib, dimension it from the local origin to its midpoint** (start the chain with
  previous = local origin). The axis dimension's direction, captured from the seeded midpoints
  at creation ([PB-DIM-VALUE-SEMANTICS]), makes the chain run outward; an aligned dimension is
  equally happy running the other way, which is one of the ways the tooth ends up reversed. Without that origin-to-first dim the whole rib chain has
  one residual DOF (it slides along the spine as a unit) and the sketch never fully constrains. Per
  rib this is exactly determined; any further constraint, wrong order, or off-spine midpoint seed
  over-constrains it.

- [SPUR-F-FLANK-ROOT] **Flank-to-root lines — exactly two constraints, and embedded-case detection
  (step 9).** If the flank's first point (on the base circle) lies **outside** the root circle, draw
  a short radial line from the root circle up to that start point on each side. Build each as
  addByTwoPoints(rootEndGeometry, flankStartFitPoint) — pass the flank spline's **start
  SketchPoint directly** as the far endpoint (share it; no separate coincident). Then place the
  root end with **exactly these two axis dimensions from the local origin**, no others:
  - (a) addDistanceDimension(localOrigin, rootEnd, HorizontalDimensionOrientation, …);
  - (b) the same with VerticalDimensionOrientation.

  The root end is seeded at its exact computed position **before** the dimensions are created, so
  each dimension captures its direction from that seed and its created value is already the exact
  magnitude — set values only to abs(Δx) / abs(Δy), and **never to the axis-signed deltas: a
  negative parameter.value flips the point to the other side of the origin**
  ([PB-DIM-VALUE-SEMANTICS]; this exact flip mirrored the right-hand root end and left the
  tooth loop open, found in-Fusion 2026-08-24). Together the two dimensions exactly constrain the
  root end (2 DOF → 0), and their captured directions say **which side of the gear centre** it
  sits on.

  ⚠️ Do **not** place it instead with "root end on the root circle" plus "local origin on the
  line". Those two are satisfied by **two** points, because the line through the flank start and
  the centre carries on and meets the root circle again on the far side. The stub then stops being
  a stub and becomes a long line straight across the gear, and the sketch reaches DOF 0 with both
  answers available. The dimensions' captured directions are what rule the far one out. This common case yields a tooth loop
  of **6 curves** (2 splines + 2 flank-to-root lines + 2 arcs). If instead the flank starts
  **inside** the root circle (**high** tooth counts drop the base circle below the root: it happens
  above 2.5 / (1 - cos(PressureAngle)) teeth, which is 41.5 at 20°, 78.5 at 14.5° and 26.7 at
  25°, so a larger pressure angle brings it on sooner), no flank-to-root line is drawn and the loop has **4 curves** (2 splines + 2
  arcs) — the profile is "embedded."

  **The embedded test is strict <:** with firstRadius the distance from the local origin to the
  left flank's first fit point, embedded = firstRadius < Root Circle Radius (compare raw values,
  no tolerance). Exact equality therefore counts as **non**-embedded and draws a **zero-length**
  flank-to-root stub (root end and flank start coincide) — this is the ill-conditioned region the

[PB-SKETCH-FIRST] [PB-SKETCHCURVES] [PB-SKETCH-TEXT] [PB-TEXT-HOLDS-DOF] [PB-SHARE-XOR-COINCIDENT] [PB-NO-OVERCONSTRAIN] [PB-DRIVING-DIM] [PB-DIM-VALUE-SEMANTICS] [PB-RADIAL-DIM] [PB-ANGULAR-DIM] [PB-NUMERIC-SNAPSHOT] [PB-SEED-NEAR] [PB-FULL-CONSTRAINT]

Required call: `adsk.core.Point3D.create(0, 0, 0)`.

Required call: `sketch.sketchPoints.add(point)`.

Required call: `circles.addByCenterRadius(localOrigin, radius)`.

Required call: `dimensions.addDiameterDimension(circle, textPoint)`.

Required call: `sketch.sketchTexts.createInput2(text, size)`.

Required call: `textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`.

Required call: `sketch.sketchTexts.add(textInput)`.

Required call: `adsk.core.ObjectCollection.create()`.

Required call: `fitPoints.add(point)`.

Required call: `splines.add(fitPoints)`.

Required call: `arcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`.

Required call: `constraints.addCoincident(arc.centerSketchPoint, localOrigin)`.

Required call: `lines.addByTwoPoints(localOrigin, toothTopPoint)`.

Required call: `dimensions.addDistanceDimension(localOrigin, referenceEnd, orientation, textPoint)`.

Required call: `dimensions.addAngularDimension(reference, spine, textPoint)`.

Required call: `lines.addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])`.

Required call: `constraints.addCoincident(midpoint, spine)`.

Required call: `constraints.addMidPoint(midpoint, rib)`.

Required call: `constraints.addPerpendicular(spine, rib)`.

Required call: `sketch.project(anchorPoint)`.

Required call: `projection.item(0)`.

Required call: `constraints.addCoincident(localOrigin, projectedAnchor)`.

The proof checks the two detected profiles and the source curves of the tooth boundary.
The sketch engine splits a root interval at its periodic parameter seam, and coalesces the disc into one circle.
The proof therefore checks two distinct tooth contacts on that root circle, two fitted flanks, one tip arc,
and either two radial stubs or none. One connected root interval supplies the second tooth arc.
Those same two contacts divide the root disc into the two arcs Fusion selects; raw engine edge-array lengths
are not Fusion curve counts. The proof also checks the detected root disc against its analytic circular area.

The proof converts the signed radian angle to metric sketch degrees for its angular constraint.
Fusion receives the original signed radians.

Proof function: `stepGearProfile`.

<!-- proof-run: proofkit.Run(sketchCases, stepGearProfile) -->


Required call: `constraints.addCoincident(toothTopPoint, tipCircle)`.

Required call: `dimensions.addDistanceDimension(left, right, ribOrientation, textPoint)`.

Required call: `dimensions.addDistanceDimension(previous, midpoint, chainOrientation, textPoint)`.

Required call: `lines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)`.

Required call: `dimensions.addDistanceDimension(localOrigin, rootEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)`.

Required call: `dimensions.addDistanceDimension(localOrigin, rootEnd, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)`.

The draw method anchors its local origin and, as its final action for a nonzero angle, assigns the angular
dimension the original signed angle in radians. Build this final confirmation inside draw, after anchoring.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "adsk.core.Point3D",
      "role": "required",
      "span": "adsk.core.Point3D.create(0, 0, 0)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchPoints",
      "reason": null,
      "receiver": "sketch.sketchPoints",
      "role": "required",
      "span": "sketch.sketchPoints.add(point)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "circles",
      "role": "required",
      "span": "circles.addByCenterRadius(localOrigin, radius)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addDiameterDimension(circle, textPoint)"
    },
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.SketchTexts",
      "reason": null,
      "receiver": "sketch.sketchTexts",
      "role": "required",
      "span": "sketch.sketchTexts.createInput2(text, size)"
    },
    {
      "condition": null,
      "name": "setAsAlongPath",
      "owner": "adsk.fusion.SketchTextInput",
      "reason": null,
      "receiver": "textInput",
      "role": "required",
      "span": "textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchTexts",
      "reason": null,
      "receiver": "sketch.sketchTexts",
      "role": "required",
      "span": "sketch.sketchTexts.add(textInput)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "adsk.core.ObjectCollection",
      "role": "required",
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "fitPoints",
      "role": "required",
      "span": "fitPoints.add(point)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.SketchFittedSplines",
      "reason": null,
      "receiver": "splines",
      "role": "required",
      "span": "splines.add(fitPoints)"
    },
    {
      "condition": null,
      "name": "addByCenterStartEnd",
      "owner": "adsk.fusion.SketchArcs",
      "reason": null,
      "receiver": "arcs",
      "role": "required",
      "span": "arcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addCoincident(arc.centerSketchPoint, localOrigin)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "lines",
      "role": "required",
      "span": "lines.addByTwoPoints(localOrigin, toothTopPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addDistanceDimension(localOrigin, referenceEnd, orientation, textPoint)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addAngularDimension(reference, spine, textPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "lines",
      "role": "required",
      "span": "lines.addByTwoPoints(leftSpline.fitPoints[i], rightSpline.fitPoints[i])"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addCoincident(midpoint, spine)"
    },
    {
      "condition": null,
      "name": "addMidPoint",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addMidPoint(midpoint, rib)"
    },
    {
      "condition": null,
      "name": "addPerpendicular",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addPerpendicular(spine, rib)"
    },
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.project(anchorPoint)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "projection",
      "role": "required",
      "span": "projection.item(0)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addCoincident(localOrigin, projectedAnchor)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addCoincident(toothTopPoint, tipCircle)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addDistanceDimension(left, right, ribOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addDistanceDimension(previous, midpoint, chainOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "lines",
      "role": "required",
      "span": "lines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addDistanceDimension(localOrigin, rootEnd, adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation, textPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addDistanceDimension(localOrigin, rootEnd, adsk.fusion.DimensionOrientations.VerticalDimensionOrientation, textPoint)"
    }
  ],
  "citations": [
    {
      "first": 432,
      "last": 491,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 18,
      "last": 211,
      "path": "spec/spurgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L432–491; `spec/spurgear/fusion.md` L18–211.

## 5 `[GO]` Extrude tooth

Name the extrusion Extrude tooth and store the new body as ctx.toothBody.

as a unit.

**This anchoring happens inside the tooth generator's draw() method itself** — draw() does drawCircles(), then
drawTooth(), then this projection-and-coincidence, then (for angle != 0 only) sets the confirming angular
dimension's value as its very last action, after the anchoring ([SPUR-F-ROTATE-CONFIRM]) — *not* a separate
step the generator performs after draw() returns. This matters because helical/herringbone build their twisted
loft profile by calling SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=…)
directly and rely on that single call to anchor the sketch. If the anchoring were moved up into buildSketches,
the twisted sketch would be left unconstrained and the loft would float off the anchor.

#### 6: Sketch-Only Short-Circuit

If the Generate-Sketches-Only input is true, make the Gear Profile sketch visible and stop — skip the
remaining body operations (tooth/body/pattern/fillet/bore). The end-of-build cleanup still runs in this mode;
its per-mode split (planes/axes always hidden, sketches left visible for inspection — the whole point of this
mode) is owned by [SPUR-F-CLEANUP].

#### 7: Extrude the Tooth

This operation runs only on the full-build path. Use PositiveExtentDirection and NewBodyFeatureOperation.

[PB-PROFILE-MATCH] [PB-NUMERIC-SNAPSHOT]

Required call: `find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)`.

Required call: `extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.

Required call: `adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`.

Required call: `extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`.

Required call: `extrudes.add(extrudeInput)`.


The solid proof substitutes straight chords through the solved flank fit points before profile detection.
It preserves the root-circle intersections and tip arc, but proves polygonal flanks rather than fitted splines.

Proof function: `stepExtrudeTooth`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->


Required call: `extrude.bodies.item(0)`.

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
      "span": "find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.ToEntityExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.ToEntityExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrude.bodies",
      "role": "required",
      "span": "extrude.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 493,
      "last": 501,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L493–501.

## 6 `[GO]` Extrude root disc

Name the extrusion Extrude body and name its body Gear Body. Store ctx.gearBody.

Find the single tooth cross-section profile in the Gear Profile sketch. The profile has 2 NURBS (the two
flanks), 2 arcs (the tooth top and the root arc between them), and — unless toothProfileIsEmbedded — 2 short
line segments (the flank-to-root lines). Find it with the framework helper —
find_profile_by_curve_counts(sketch, nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2) (from
.utilities) — do not re-implement the loop search; the helper rejects loops whose curve counts don't match and
raises when nothing matches.

Extrude this profile from the target plane to the Extrusion End Plane (ToEntityExtentDefinition,
PositiveExtentDirection) as a **New Body**. Name the feature Extrude tooth. Store the resulting body as
ctx.toothBody.

#### 9: Extrude the Body

Find the gear body profile — the solid disc inside the root circle, whose boundary is **exactly 2 arcs**: the
two pieces the tooth's flank-to-root lines cut the root circle into. find_profile_by_curve_counts(sketch,
arcs=2) (from .utilities). It is not an annulus and the tip circle is not part of it: the tip circle is
construction geometry (step 3), and construction geometry bounds no profile. Extrude it from the target plane
to the Extrusion End Plane (ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
PositiveExtentDirection) as a **New Body**. Name the feature Extrude body and the resulting body Gear Body.

While iterating the new body's faces (extrude.bodies.item(0).faces), capture two references needed later.
Classify each face by face.geometry.surfaceType:

- **Gear Center construction axis** — from any face whose surfaceType is CylinderSurfaceType. Build it with
constructionAxes.createInput() → axisInput.setByCircularFace(cylindrical_face) →
constructionAxes.add(axisInput). Name it Gear Center; set isLightBulbOn = False. Store on ctx.centerAxis.
- **ctx.extrusionExtent** (the far end-cap face, used later by the bore cut) — among faces whose surfaceType
is PlaneSurfaceType, the one that is parallel to but **not** coplanar with the gear's sketch plane. Test it
with the plane-geometry API rather than a hand-rolled dot-product: let sketchPlane =
ctx.gearProfileSketch.referencePlane.geometry, and pick the face where
sketchPlane.isParallelToPlane(face.geometry) and not sketchPlane.isCoPlanarTo(face.geometry). (The near cap is
coplanar with the sketch plane, so isCoPlanarTo rules it out; the cylindrical and side faces aren't planar.)
Raise if either reference isn't found.

Build the Gear Center axis in the next timeline entry.

[PB-PROFILE-MATCH] [PB-NUMERIC-SNAPSHOT] [PB-SELF-DIAGNOSING]

Required call: `find_profile_by_curve_counts(sketch, arcs=2)`.

Required call: `extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`.

Required call: `adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`.

Required call: `extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`.

Required call: `extrudes.add(extrudeInput)`.

Required call: `sketchPlane.isParallelToPlane(face.geometry)`.

Required call: `sketchPlane.isCoPlanarTo(face.geometry)`.


The solid proof substitutes straight chords through the solved flank fit points before profile detection.
It preserves the root-circle intersections and tip arc, but proves polygonal flanks rather than fitted splines.

Proof function: `stepExtrudeBody`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeBody, assertExtrudeBody) -->


Required call: `extrude.bodies.item(0)`.

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
      "span": "find_profile_by_curve_counts(sketch, arcs=2)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.createInput(profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.ToEntityExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.ToEntityExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.add(extrudeInput)"
    },
    {
      "condition": null,
      "name": "isParallelToPlane",
      "owner": "adsk.core.Plane",
      "reason": null,
      "receiver": "sketchPlane",
      "role": "required",
      "span": "sketchPlane.isParallelToPlane(face.geometry)"
    },
    {
      "condition": null,
      "name": "isCoPlanarTo",
      "owner": "adsk.core.Plane",
      "reason": null,
      "receiver": "sketchPlane",
      "role": "required",
      "span": "sketchPlane.isCoPlanarTo(face.geometry)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "extrude.bodies",
      "role": "required",
      "span": "extrude.bodies.item(0)"
    }
  ],
  "citations": [
    {
      "first": 503,
      "last": 515,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L503–515.

## 7 `[PROSE]` Gear center axis

The proof uses the measured gear center as the pattern axis; decad has no Fusion construction-axis feature.

Find the gear body profile — the solid disc inside the root circle, whose boundary is **exactly 2 arcs**: the
two pieces the tooth's flank-to-root lines cut the root circle into. find_profile_by_curve_counts(sketch,
arcs=2) (from .utilities). It is not an annulus and the tip circle is not part of it: the tip circle is
construction geometry (step 3), and construction geometry bounds no profile. Extrude it from the target plane
to the Extrusion End Plane (ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False),
PositiveExtentDirection) as a **New Body**. Name the feature Extrude body and the resulting body Gear Body.

While iterating the new body's faces (extrude.bodies.item(0).faces), capture two references needed later.
Classify each face by face.geometry.surfaceType:

Use the cylindrical face of ctx.gearBody. Name the axis Gear Center and store ctx.centerAxis.

[PB-CONSTRUCTION-AXES] [PB-HIDE-AFTER-USE]

Required call: `axes.createInput()`.

Required call: `axisInput.setByCircularFace(cylindricalFace)`.

Required call: `axes.add(axisInput)`.


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ConstructionAxes",
      "reason": null,
      "receiver": "axes",
      "role": "required",
      "span": "axes.createInput()"
    },
    {
      "condition": null,
      "name": "setByCircularFace",
      "owner": "adsk.fusion.ConstructionAxisInput",
      "reason": null,
      "receiver": "axisInput",
      "role": "required",
      "span": "axisInput.setByCircularFace(cylindricalFace)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ConstructionAxes",
      "reason": null,
      "receiver": "axes",
      "role": "required",
      "span": "axes.add(axisInput)"
    }
  ],
  "citations": [
    {
      "first": 509,
      "last": 511,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L509–511.

## 8 `[GO]` Pattern teeth

#### 10: Pattern Teeth

Circular-pattern ctx.toothBody around the Gear Center axis, quantity = Tooth Number. Pin the two other pattern
inputs: patternInput.totalAngle = ValueInput.createByString('360 deg') (a full turn, set as a string
expression) and patternInput.isSymmetric = False. Combine the patterned tooth bodies into Gear Body via a
single Combine-Join.

Feed the pattern's bodies collection to the combine as-is — it already includes the original tooth body, per
[PB-PATTERN-BODIES].

The pattern is a separate timeline feature from Combine. Copy the pattern bodies to an ObjectCollection in the next step.

[PB-CIRCULAR-PATTERN] [PB-PATTERN-BODIES] [PB-NUMERIC-SNAPSHOT]

Required call: `adsk.core.ObjectCollection.create()`.

Required call: `entities.add(ctx.toothBody)`.

Required call: `patterns.createInput(entities, ctx.centerAxis)`.

Required call: `adsk.core.ValueInput.createByReal(toothNumber)`.

Required call: `adsk.core.ValueInput.createByString('360 deg')`.

Required call: `patterns.add(patternInput)`.


The solid proof substitutes straight chords through the solved flank fit points before profile detection.
It preserves the root-circle intersections and tip arc, but proves polygonal flanks rather than fitted splines.

The engine cannot decide separation between simultaneous patterned teeth and reports undecided_pair.
The proof builds every rotated placement in a separate document, checks each solid, and compares its volume
and rotated centroid with the seed. It counts the seed once and all remaining placements once.
It does not prove simultaneous inter-copy separation in a shared document.

Proof function: `stepPatternTeeth`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->


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
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "entities",
      "role": "required",
      "span": "entities.add(ctx.toothBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CircularPatternFeatures",
      "reason": null,
      "receiver": "patterns",
      "role": "required",
      "span": "patterns.createInput(entities, ctx.centerAxis)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(toothNumber)"
    },
    {
      "condition": null,
      "name": "createByString",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByString('360 deg')"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CircularPatternFeatures",
      "reason": null,
      "receiver": "patterns",
      "role": "required",
      "span": "patterns.add(patternInput)"
    }
  ],
  "citations": [
    {
      "first": 518,
      "last": 522,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L518–522.

## 9 `[GO]` Join teeth

Circular-pattern ctx.toothBody around the Gear Center axis, quantity = Tooth Number. Pin the two other pattern
inputs: patternInput.totalAngle = ValueInput.createByString('360 deg') (a full turn, set as a string
expression) and patternInput.isSymmetric = False. Combine the patterned tooth bodies into Gear Body via a
single Combine-Join.

Feed the pattern's bodies collection to the combine as-is — it already includes the original tooth body, per
[PB-PATTERN-BODIES].

Copy every body from the pattern, including its original seed exactly once, into toolBodies. Set operation to JoinFeatureOperation and keep tools False.

[PB-PATTERN-BODIES]

Required call: `adsk.core.ObjectCollection.create()`.

Required call: `toolBodies.add(body)`.

Required call: `combines.createInput(ctx.gearBody, toolBodies)`.

Required call: `combines.add(combineInput)`.


The solid proof substitutes straight chords through the solved flank fit points before profile detection.
It preserves the root-circle intersections and tip arc, but proves polygonal flanks rather than fitted splines.

The real boolean join refuses the coincident root surfaces. The proof instead extrudes the complete
patterned outline, assembled from the solved tooth boundary and root-valley arcs. It verifies total volume.
It does not prove Fusion Combine-Join, its tool-body collection, or consumption of the original seed.

Proof function: `stepJoinTeeth`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepJoinTeeth, assertJoinTeeth) -->


Required call: `pattern.bodies.item(i)`.

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
      "span": "adsk.core.ObjectCollection.create()"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "toolBodies",
      "role": "required",
      "span": "toolBodies.add(body)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combines",
      "role": "required",
      "span": "combines.createInput(ctx.gearBody, toolBodies)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combines",
      "role": "required",
      "span": "combines.add(combineInput)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.fusion.BRepBodies",
      "reason": null,
      "receiver": "pattern.bodies",
      "role": "required",
      "span": "pattern.bodies.item(i)"
    }
  ],
  "citations": [
    {
      "first": 520,
      "last": 522,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L520–522.

## 10 `[GO]` Root fillets

#### 11: Fillets

If Fillet Radius > 0, round the corner where the root valley floor meets each tooth flank — the sharp inside
corner that runs the full thickness of the gear, parallel to its main axis. This is where bending stress
concentrates at the tooth root, so it's the structurally important fillet (not the front/back rim, which is a
cosmetic rounding the user doesn't want here). Two things make picking the right edges fiddly:

- After the pattern-and-combine step, the root cylinder is usually split into one patch per valley rather than
a single continuous surface. Collect *every* cylindrical face whose radius equals Root Circle Radius, not just
the first one found.
- On each such face, keep the **axial straight edges** — the ones whose direction is parallel to the target
plane's normal (i.e. parallel to the gear's main axis). Those are the two valley-floor-to-tooth-flank corners
on each valley patch. Drop the *circular* edges that wrap around the circumference at the front and back end
caps; those are end-cap rims, not the structural root corners. Filter first to Line3DCurveType edges, then
take each line's direction from its **geometry endpoints** (geometry.startPoint.vectorTo(geometry.endPoint)),
normalize, and keep it if it is parallel to the axis within tolerance: abs(abs(dot(direction, axisNormal)) -
1.0) < 0.01. (Use exactly this tolerance — a tighter test like > 0.999 can drop valid axial edges that are
slightly off due to tessellation, leaving root fillets missing.)

Apply the fillet with filletFeatures.createInput() → filletInput.addConstantRadiusEdgeSet(edges, <Fillet
Radius value>, isTangentChain=False) — add the edge set on the input **itself**, per [PB-FILLET-CHAMFER] (do
**not** route it through a filletInput.edgeSetInputs collection — that is the chamfer-side shape, and reaching
for it on the fillet input fails). **isTangentChain must be False** — the collected edges are exactly the
axial root corners; tangent-chaining (True) would let Fusion pull in tangent-adjacent edges and round more
than the intended root corner. Do **not** read the direction via edge.evaluator.getTangent(0) — parameter 0 is
not guaranteed to lie inside the edge's parameter range and Fusion raises RuntimeError: invalid argument
parameter.

If the edge collection ends up **empty** (no axial root edge matched), silently skip the fillet — return
without creating the fillet feature, no error. An empty edge set must not reach filletFeatures.add.

#### 12: Bore (optional)

Use a radius match tolerance of 0.0001 cm. The direct FilletFeatureInput method remains required under the repository unverified API policy; do not change its receiver.

[PB-FILLET-CHAMFER] [PB-EMPTY-RESULT] [PB-NUMERIC-SNAPSHOT]

Required call: `line.startPoint.vectorTo(line.endPoint)`.

The direction still must be normalized, and its dot product with the target-plane normal must satisfy
abs(abs(dot) - 1.0) < 0.01. These two method names are examples; equivalent component arithmetic is allowed.

Example implementation: `direction.normalize()`.

Example implementation: `direction.dotProduct(axisNormal)`.

Required call: `fillets.createInput()`.

Required call: `adsk.core.ValueInput.createByReal(filletRadius)`.

Required call: `filletInput.addConstantRadiusEdgeSet(edges, radiusValue, False)`.

Required call: `fillets.add(filletInput)`.



In the positive-fillet embedded case, the first clipped flank chord is shorter than the fillet setback.
Decad cannot merge the resulting corner rewrite across the next chord. The proof coalesces only the first
flank chords on each side until the span reaches two fillet radii, then applies the original specified radius.
It verifies all axial root fillet cylinders and the solid verdict on that coarser local flank approximation.
It does not prove the fillet transition on the original fitted involute or its original fine chord chain.

Proof function: `stepRootFillets`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepRootFillets, assertRootFillets) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "vectorTo",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "line.startPoint",
      "role": "required",
      "span": "line.startPoint.vectorTo(line.endPoint)"
    },
    {
      "condition": null,
      "name": "normalize",
      "owner": "adsk.core.Vector3D",
      "reason": "The source requires normalization but does not name this method; dividing vector components by their magnitude implements the same required operation.",
      "receiver": "direction",
      "role": "example",
      "span": "direction.normalize()"
    },
    {
      "condition": null,
      "name": "dotProduct",
      "owner": "adsk.core.Vector3D",
      "reason": "The source requires the normalized dot-product test but does not name this method; component arithmetic implements the same required operation.",
      "receiver": "direction",
      "role": "example",
      "span": "direction.dotProduct(axisNormal)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.FilletFeatures",
      "reason": null,
      "receiver": "fillets",
      "role": "required",
      "span": "fillets.createInput()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(filletRadius)"
    },
    {
      "condition": null,
      "name": "addConstantRadiusEdgeSet",
      "owner": "adsk.fusion.FilletFeatureInput",
      "reason": null,
      "receiver": "filletInput",
      "role": "required",
      "span": "filletInput.addConstantRadiusEdgeSet(edges, radiusValue, False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.FilletFeatures",
      "reason": null,
      "receiver": "fillets",
      "role": "required",
      "span": "fillets.add(filletInput)"
    }
  ],
  "citations": [
    {
      "first": 524,
      "last": 535,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L524–535.

## 11 `[GO]` Bore profile sketch

buildBore runs unconditionally from generate() (after buildMainGearBody), so it MUST itself early-return in
two cases: when **SketchOnly** is set, and when **Bore Diameter ≤ 0**. The SketchOnly guard is essential — in
sketch-only mode buildMainGearBody short-circuits before buildBody, so ctx.gearBody and ctx.extrusionExtent
are never set; proceeding into the cut would dereference None. (Do not rely on the bore diameter being 0 in
sketch-only mode — the user may have set both.)

Otherwise (full build, Bore Diameter > 0), create a separate Bore Profile sketch on the target plane and
draw the bore circle **by instantiating the tooth generator on that sketch** —
SpurGearInvoluteToothDesignGenerator(boreSketch, self) — and calling its drawBore(ctx.anchorPoint,
boreDiameter), which projects the anchor in and draws the construction-less circle of that diameter with a
driving diameter dimension. drawBore takes .item(0) from the projection's ObjectCollection before using
the point as the circle centre. Note the accepted side effect: the tooth generator's **constructor** always
adds its local-origin (0, 0, 0) SketchPoint (see [SPUR-F-LOCAL-ORIGIN]), so the Bore Profile sketch
carries one stray unused sketch point at (0,0,0) — faithful behavior, don't suppress it. **Ground that point
on the projected anchor**, exactly as step 5 does for the Gear Profile sketch: drawBore already projects
ctx.anchorPoint into this sketch to place the circle's centre, so add addCoincident(toothGen.anchorPoint,
projectedAnchor) using that same projection. The local origin then rides on the anchor like every other
sketch's does, the Bore Profile sketch is fully constrained, and the bore follows the anchor if the user moves
it. Do **not** ground it on boreSketch.originPoint instead — that pins the point to the plane rather than
the gear, and [PB-CIRCLE-CENTER] records a solver failure from constraining to originPoint. Without any
grounding the point is free in two directions and the sketch never reaches isFullyConstrained. Then
extrude-cut the bore profile from the target plane to ctx.extrusionExtent (the far end-cap face), affecting
only ctx.gearBody. The ToEntityExtentDefinition to the far face guarantees the bore goes all the way

Keep the bore extrusion as the next timeline entry.

[PB-FULL-CONSTRAINT] [PB-CIRCLE-CENTER] [PB-DRIVING-DIM] [PB-RADIAL-DIM] [SPUR-F-ANCHOR-CHAIN] [SPUR-F-LOCAL-ORIGIN]

Required call: `sketch.project(ctx.anchorPoint)`.

Required call: `projection.item(0)`.

Required call: `circles.addByCenterRadius(projectedAnchor, boreDiameter / 2)`.

Required call: `dimensions.addDiameterDimension(circle, textPoint)`.

Required call: `constraints.addCoincident(toothGen.anchorPoint, projectedAnchor)`.


Proof function: `stepBoreProfile`.

<!-- proof-run: proofkit.Run(boreCases, stepBoreProfile) -->


<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "sketch",
      "role": "required",
      "span": "sketch.project(ctx.anchorPoint)"
    },
    {
      "condition": null,
      "name": "item",
      "owner": "adsk.core.ObjectCollection",
      "reason": null,
      "receiver": "projection",
      "role": "required",
      "span": "projection.item(0)"
    },
    {
      "condition": null,
      "name": "addByCenterRadius",
      "owner": "adsk.fusion.SketchCircles",
      "reason": null,
      "receiver": "circles",
      "role": "required",
      "span": "circles.addByCenterRadius(projectedAnchor, boreDiameter / 2)"
    },
    {
      "condition": null,
      "name": "addDiameterDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dimensions",
      "role": "required",
      "span": "dimensions.addDiameterDimension(circle, textPoint)"
    },
    {
      "condition": null,
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addCoincident(toothGen.anchorPoint, projectedAnchor)"
    }
  ],
  "citations": [
    {
      "first": 537,
      "last": 555,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L537–555.

## 12 `[GO]` Cut bore

grounding the point is free in two directions and the sketch never reaches isFullyConstrained. Then
extrude-cut the bore profile from the target plane to ctx.extrusionExtent (the far end-cap face), affecting
only ctx.gearBody. The ToEntityExtentDefinition to the far face guarantees the bore goes all the way
through regardless of Thickness.

Only positive Bore Diameter in full-build mode reaches this step. Use the sole Bore Profile region; participantBodies is [ctx.gearBody].

[PB-SINGLE-PROFILE]

Required call: `sketch.profiles.item(0)`.

Required call: `extrudes.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`.

Required call: `adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`.

Required call: `extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`.

Required call: `extrudes.add(extrudeInput)`.



The explicit boolean cut exceeds decad's 4,096-segment arrangement cap. The proof instead extrudes
a complete gear outline with the bore hole, then applies the root fillets. A separate filleted gear
provides the pre-bore volume, and the assertion checks that exactly the full-height bore volume is removed.
This proves the resulting geometry, not Fusion's participant-body selection or cut-after-fillet timeline.

Proof function: `stepCutBore`.

<!-- proof-run: proofkit3d.RunSolid(boreSolidCases, stepCutBore, assertCutBore) -->

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
      "span": "sketch.profiles.item(0)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)"
    },
    {
      "condition": null,
      "name": "create",
      "owner": "adsk.fusion.ToEntityExtentDefinition",
      "reason": null,
      "receiver": "adsk.fusion.ToEntityExtentDefinition",
      "role": "required",
      "span": "adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)"
    },
    {
      "condition": null,
      "name": "setOneSideExtent",
      "owner": "adsk.fusion.ExtrudeFeatureInput",
      "reason": null,
      "receiver": "extrudeInput",
      "role": "required",
      "span": "extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.add(extrudeInput)"
    }
  ],
  "citations": [
    {
      "first": 553,
      "last": 557,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L553–557.

## 13 `[GO]` Chamfer completed gear

generate calls chamferTeeth(ctx) after the optional bore, so the chamfer runs after the teeth
are patterned and joined, after the root fillets, and after the bore cut. It returns in SketchOnly
mode and when Apply-Chamfer-To-Teeth is zero.

Walk every planar face of ctx.gearBody parallel to the Gear Profile sketch plane. Add each edge
from those end-cap faces once, using edge.tempId to remove duplicates. This includes the tooth
flanks, tooth tops, and root-radius arcs. Exclude only a Circle3DCurveType edge whose radius is
the positive Bore Diameter divided by two, within 0.001 cm, so a bore never receives a chamfer.
Raise when no end-cap face or no chamfer edge remains; do not create a partial chamfer.

Apply the resulting set with chamferFeatures.createInput2() and
chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, <ChamferTooth value>, False). Helical and
herringbone inherit this completed-gear selection unchanged. The final Fusion verification is
recorded in spec/helicalgear/fusion.md [HELI-F-CHAMFER-COUNT].


  **Embedded-flag mechanism (the tooth generator has no ctx):** the tooth generator sets the
  boolean on its **parent generator** — self.parent._lastToothEmbedded = True/False — during
  drawTooth. SpurGearGenerator.__init__ MUST pre-initialise self._lastToothEmbedded = False
  (alongside self.toolsSketch = None and self.boreSketch = None). Then buildSketches (which
  holds ctx) copies it across: ctx.toothProfileIsEmbedded = self._lastToothEmbedded. Do not try
  to set ctx.toothProfileIsEmbedded from inside the tooth generator — it cannot reach ctx.

#### Cleanup

- [SPUR-F-CLEANUP] **End-of-build cleanup recipe.** Hide construction geometry and sketches per
  [PB-HIDE-AFTER-USE] (isLightBulbOn = False for construction planes/axes; isVisible = False
  for sketches — never crossed). The spur-specific recipe: the cleanup turns off the light bulb on
  **every** construction plane/axis it created — the Extrusion End Plane, the Gear Center axis,
  and the normalized target plane if one was created in step 1 — and sets isVisible = False on the
  Tools, Gear Profile, and Bore Profile sketches, so only the finished gear body shows. **Split by
  entity kind and mode:** the construction-plane/axis hiding **always runs, in both modes**
  (including Generate-Sketches-Only, so no stray plane floats); the **sketch** hiding runs **only on
  the full-build path** (sketch-only mode leaves Tools/Gear Profile visible for inspection — see
  step 6). Guard each entity individually (only hide it if it was actually created — the Gear
  Center axis and Bore Profile sketch don't exist in sketch-only mode).

#### Numeric snapshots

- [SPUR-F-SNAPSHOT] **Dimensions and feature inputs are numeric snapshots.** Per
  [PB-NUMERIC-SNAPSHOT]: every dimension and feature input (extrude offset, chamfer distance,
  fillet radius, pattern count, bore-circle diameter) is set with the *current numeric value* of its
  source parameter at generation time, not a live expression. Editing a <prefix>_… user parameter
  does **not** change an existing spur gear — re-run the dialog to regenerate. The parameters stay
  visible in the table for reference only. (This matches the original implementation.)

[PB-FILLET-CHAMFER] [PB-NUMERIC-SNAPSHOT] [PB-HIDE-AFTER-USE] [SPUR-F-CLEANUP] [SPUR-F-SNAPSHOT]

Required call: `chamfers.createInput2()`.

Required call: `adsk.core.ValueInput.createByReal(chamferTooth)`.

Required call: `chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distanceValue, False)`.

Required call: `chamfers.add(chamferInput)`.



The completed, filleted gear's cap chamfer was attempted and refused because the offset changes the
section topology and decad has no trimmed-offset kernel. The proof instead chamfers each cap of the
actual root-disc extrusion on a separate body. It checks axial setback, solid validity, and the analytic
cylinder-plus-frustum volume. This demonstrates the chamfer operation only on the prerequisite root disc.
It does not prove chamfering the completed tooth caps or root fillets, both caps on one body, or bore-edge
exclusion. The positive-bore case records the sealed selector's inability to exclude the bore loops.
All specified Fusion completed-gear edge selection and chamfer calls remain required.

Proof function: `stepChamferCompletedGear`.

<!-- proof-run: proofkit3d.RunSolid(chamferCases, stepChamferCompletedGear, assertChamferCompletedGear) -->

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput2",
      "owner": "adsk.fusion.ChamferFeatures",
      "reason": null,
      "receiver": "chamfers",
      "role": "required",
      "span": "chamfers.createInput2()"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(chamferTooth)"
    },
    {
      "condition": null,
      "name": "addEqualDistanceChamferEdgeSet",
      "owner": "adsk.fusion.ChamferEdgeSets",
      "reason": null,
      "receiver": "chamferInput.chamferEdgeSets",
      "role": "required",
      "span": "chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distanceValue, False)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.ChamferFeatures",
      "reason": null,
      "receiver": "chamfers",
      "role": "required",
      "span": "chamfers.add(chamferInput)"
    }
  ],
  "citations": [
    {
      "first": 559,
      "last": 573,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 213,
      "last": 242,
      "path": "spec/spurgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L559–573; `spec/spurgear/fusion.md` L213–242.

