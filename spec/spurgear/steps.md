The proof files are `proof/spurgear/sketches_test.go`, `proof/spurgear/solids_test.go`, and `proof/spurgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/spurgear/instructions.md` | `58efc84a524426519d090b19f3deb5854a543e0c` |
| `spec/spurgear/fusion.md` | `5cd1f9f96e043efba42ae42a00ca6c13403e1339` |
| `spec/helicalgear/fusion.md` | `6cbe03029a0b89b001479234c0af422deb589dbf` |
| `spec/spurgear/contract.json` | `4bfdaaa2b1b38b14478a59dd5c2289ad35b8d44f` |
| `spec/spurgear/exact_values.json` | `fee6665556d2d9bb3673ee44b79fca03e7caf5a2` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `b7e7811c13fe64aef683d7d6f5d78317b5bb643c` |

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
        "in_function": "processInputs",
        "required": [
          "isinstance\\(parent,\\s*adsk\\.fusion\\.Occurrence\\)",
          "self\\.parentComponent\\s*=\\s*parent\\.component"
        ],
        "why": "Parent selection accepts Occurrences and RootComponents; Generator.getOccurrence needs a Component with occurrences."
      },
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

## 0 `[PROSE]` Command setup and occurrence

Reproduce the four-class public API, the overridable method boundaries, and the call graph below.
The generated Exact values and Compilation contract sections own dialog and parameter literals;
apply those values to dialog creation and parameter registration, respectively.
Read Parent, Target Plane, and Anchor Point before any occurrence or parameter creation.
Use the inherited framework methods instead of redefining them. This setup has no geometric proof.
[PB-INPUT-READ] [PB-GET-VALUE-CONTRACT] [PB-DIALOG-DEFAULT-UNITS] [PB-SELECTION-DECL]
[PB-SELECTION-FILTER-ENUM] [PB-SELECTION-STASH] [PB-AUTOFOCUS-FIRST] [PB-NUMERIC-SNAPSHOT]
[PB-PRECOMPUTED-MODE] [PB-NEVER-ACTIVATE] [PB-LOGGING]

Component Setup.

A spur gear is a single cylindrical body with straight teeth cut along the axis. Unlike the bevel generator there is
no pairing — one invocation of the command produces exactly one gear. The new gear is added as a child occurrence of
the user-selected Parent Component.
If Parent selection is an `adsk.fusion.Occurrence`, store `parent.component` in
`self.parentComponent`; if it is the root Component, store it directly. The base generator creates
the child occurrence through `self.parentComponent.occurrences`.

Architecture.

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
   processInputs, generate, parameter registration, and the per-step build methods listed below.
   Its prefixBase() returns 'SpurGear'.

Keep these five return annotations on methods that subclasses override:

| Class | Method | Return annotation |
|---|---|---|
| SpurGearGenerator | prefixBase | -> str |
| SpurGearGenerator | generateName | -> str |
| SpurGearGenerator | filletHelixFactorExpression | -> str |
| SpurGearGenerator | newContext | -> SpurGearGenerationContext |
| SpurGearInvoluteToothDesignGenerator | getParameterValue | -> float |

Dropping these annotations makes the helical and herringbone overrides fail type checks.

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


The SpurGearGenerationContext object is passed between generation steps and read by subclasses (helical, herringbone).
Subclasses reach in by name, so these field names are part of the public API of this module — don't rename them when
reconstructing:

- ctx.plane — the ConstructionPlane all sketches are built on (normalised in step 1).
- ctx.anchorPoint — the SketchPoint that is the Tools-sketch projection of the user's anchor. Later sketches project
*this* in again to chain to the user's original anchor entity.
- ctx.extrusionEndPlane — the offset construction plane used as the to-entity target for the tooth and body extrudes.
- ctx.gearProfileSketch — the sketch containing the tooth profile + four gear circles.
- ctx.toothBody — the single extruded tooth, before the circular pattern.
- ctx.gearBody — the cylindrical body the teeth are joined into.
- ctx.centerAxis — the Gear Center construction axis built off the body's cylindrical face.
- ctx.extrusionExtent — the far end-cap face of the gear body, used as the to-entity for the bore cut.
- ctx.toothProfileIsEmbedded — True iff the base circle sits inside the root circle (no flank-to-root stubs drawn);
used by the tooth extrude step to pick the right profile-curve count.

(The active plane is also held on the generator as self.plane — normalised in step 1 — and
subclasses read self.plane directly; keep both available.)

Method contract — call graph and override boundaries.

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
  twisted profile sketch with SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint,
angle=helixAngle).
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

Tooth generator (SpurGearInvoluteToothDesignGenerator) reproduced surface.

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

Dependencies and dependents.

Spur imports only the framework (.base — Generator, GenerationContext, get_value, get_boolean,
get_selection; .utilities — get_normal, find_profile_by_curve_counts; .misc — to_cm,
get_design). It depends on no other gear. Two dependents bind to its surface; regenerating spur
must not break either:

- **helicalgear.py / herringbonegear.py subclass the four classes.** Helical imports

Use these calls with the operands described above:

- `inputs.addSelectionInput(id, label, prompt)`.
- `selection.addSelectionFilter(filterConstant)`.
- `selection.setSelectionLimits(1, 1)`.
- `parentInput.addSelection(rootComponent)`.
- `inputs.addValueInput(id, label, unit, defaultValue)`.
- `inputs.addBoolValueInput(id, label, True, "", False)`.
- `inputs.addStringValueInput(id, label, initialValue)`.
- `adsk.core.ValueInput.createByReal(value)`.
- `adsk.core.ValueInput.createByString(expression)`.

<!-- step-meta
{
  "calls": [
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
      "receiver": "parentInput",
      "role": "required",
      "span": "parentInput.addSelection(rootComponent)"
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
      "span": "inputs.addBoolValueInput(id, label, True, \"\", False)"
    },
    {
      "condition": null,
      "name": "addStringValueInput",
      "owner": "adsk.core.CommandInputs",
      "reason": null,
      "receiver": "inputs",
      "role": "required",
      "span": "inputs.addStringValueInput(id, label, initialValue)"
    },
    {
      "condition": null,
      "name": "createByReal",
      "owner": "adsk.core.ValueInput",
      "reason": null,
      "receiver": "adsk.core.ValueInput",
      "role": "required",
      "span": "adsk.core.ValueInput.createByReal(value)"
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
      "first": 9,
      "last": 177,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 253,
      "last": 386,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 1,
      "last": 37,
      "path": "spec/spurgear/exact_values.json"
    },
    {
      "first": 1,
      "last": 109,
      "path": "spec/spurgear/contract.json"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L9–177; `spec/spurgear/instructions.md` L253–386; `spec/spurgear/exact_values.json` L1–37; `spec/spurgear/contract.json` L1–109.

## 1 `[PROSE]` Normalize Target Plane

If the selected entity is already a ConstructionPlane, reuse it. Otherwise create a coplanar
construction plane and store it in self.plane and ctx.plane. Track whether this build created it.
Use a ValueInput containing zero, never a bare offset. Keep it visible until final cleanup.
[PB-CONSTRUCTION-PLANES] [PB-HIDE-AFTER-USE] [SPUR-F-CLEANUP]
This plane normalization and its external Fusion reference have no sketch-engine counterpart.

Use these calls with the operands described above:

- `planes.createInput()`.
- `adsk.core.ValueInput.createByReal(0)`.
- `planeInput.setByOffset(selectedPlane, offset)`.
- `planes.add(planeInput)`.

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
      "span": "adsk.core.ValueInput.createByReal(0)"
    },
    {
      "condition": null,
      "name": "setByOffset",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "reason": null,
      "receiver": "planeInput",
      "role": "required",
      "span": "planeInput.setByOffset(selectedPlane, offset)"
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
      "first": 405,
      "last": 407,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 217,
      "last": 235,
      "path": "spec/spurgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L405–407; `spec/spurgear/fusion.md` L217–235.

## 2 `[GO]` Tools sketch

In prepareTools, create the Tools sketch on ctx.plane with the inherited createSketchObject helper.
Set isVisible=True before projecting the user anchor. Store the projection's item zero in ctx.anchorPoint.
The Tools sketch must stay visible through the optional bore and chamfer. Check isFullyConstrained and
raise with the sketch name if false. The proof's observer point substitutes for the reference-only
sketch because the harness requires an authored point; it verifies projection without requiring that
extra point in Fusion. [SPUR-F-ANCHOR-CHAIN] [PB-PROJECT-NOT-FIXED] [PB-FULL-CONSTRAINT]
[PB-SKETCH-FIRST] [PB-HIDE-AFTER-USE]

The proof function is `stepTools`.

<!-- proof-run: proofkit.Run(sketchCases, stepTools) -->

Use these calls with the operands described above:

- `toolsSketch.project(self.anchorPoint)`.
- `projection.item(0)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "toolsSketch",
      "role": "required",
      "span": "toolsSketch.project(self.anchorPoint)"
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
      "first": 409,
      "last": 415,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 17,
      "last": 31,
      "path": "spec/spurgear/fusion.md"
    },
    {
      "first": 362,
      "last": 438,
      "path": ".claude/skills/generate-gear/PLAYBOOK.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L409–415; `spec/spurgear/fusion.md` L17–31; `.claude/skills/generate-gear/PLAYBOOK.md` L362–438.

## 3 `[PROSE]` Extrusion End Plane

In prepareTools, create Extrusion End Plane offset from ctx.plane by current numeric Thickness.
Store it as ctx.extrusionEndPlane. Both subsequent extrudes use this entity as their end.
Leave its light bulb on until cleanup, including during extrusion.
The solid proof represents its distance as a one-sided extrusion extent.
[PB-CONSTRUCTION-PLANES] [PB-NUMERIC-SNAPSHOT] [PB-HIDE-AFTER-USE] [SPUR-F-SNAPSHOT]

Use these calls with the operands described above:

- `planes.createInput()`.
- `adsk.core.ValueInput.createByReal(thickness)`.
- `planeInput.setByOffset(ctx.plane, offset)`.
- `planes.add(planeInput)`.

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
      "span": "planeInput.setByOffset(ctx.plane, offset)"
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
      "first": 415,
      "last": 415,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 236,
      "last": 242,
      "path": "spec/spurgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L415; `spec/spurgear/fusion.md` L236–242.

## 4 `[GO]` Gear Profile sketch

In buildSketches, create Gear Profile on ctx.plane, explicitly set isVisible=True, and construct the
tooth generator. Its draw method owns circles, tooth, and anchoring as one sketch timeline entry.
Read numeric snapshots in centimetres and radians. The proof uses millimetres and degrees where the
sketch API requires them. The only solid circle is Root Circle; the other three are construction.
Keep the complete circle for automatic profile splitting. Log labelled sketch constraint status;
labels prevent a reliable isFullyConstrained gate, but every geometric DOF must still be constrained.
[PB-SKETCH-FIRST] [PB-FULL-CONSTRAINT] [PB-TEXT-HOLDS-DOF] [PB-NO-OVERCONSTRAIN]
[PB-SHARE-XOR-COINCIDENT] [PB-SKETCHCURVES] [PB-SKETCH-TEXT] [PB-DRIVING-DIM]
[PB-RADIAL-DIM] [PB-DIM-VALUE-SEMANTICS] [PB-ANGULAR-DIM] [PB-SEED-NEAR]
[PB-NUMERIC-SNAPSHOT] [PB-HIDE-AFTER-USE] [PB-PROJECT-NOT-FIXED]
[SPUR-F-ANCHOR-CHAIN] [SPUR-F-LOCAL-ORIGIN] [SPUR-F-SHARED-ADJACENCY]
[SPUR-F-ROTATE-CONFIRM] [SPUR-F-TOOTHTOP-ARC] [SPUR-F-SPINE] [SPUR-F-RIBS]
[SPUR-F-FLANK-ROOT] [SPUR-F-SNAPSHOT]

Instructions.

1: Normalize the Target Plane.

If the user-selected plane is not already a ConstructionPlane (for example they picked a planar face of an existing
body), create a coplanar construction plane via ConstructionPlaneInput.setByOffset(selectedPlane,
adsk.core.ValueInput.createByReal(0)) and use that for all subsequent operations. The offset argument is a
**ValueInput, not a bare number** — setByOffset(plane, 0) is a runtime TypeError ([PB-CONSTRUCTION-PLANES] gives the
signature). The same applies to the Extrusion End Plane in step 2: its Thickness offset is passed as
ValueInput.createByReal(thickness). This keeps the downstream profile-detection code from having to filter out the
selected face's native profile.

2: Tools Sketch.

Create a sketch named Tools on the target plane. sketch.project(anchorPoint) returns an
ObjectCollection, even for one point. Take .item(0) from that collection and keep the resulting
SketchPoint as ctx.anchorPoint — this is the canonical handle every later sketch will re-project
from (see Sketch Discipline). The Tools sketch draws no geometry of its own; it exists to own this
one reference. Leave the Tools sketch visible while later sketches project from it. Set
Tools.isVisible = False only after the gear is fully built.

Create an offset construction plane Extrusion End Plane at distance Thickness from the target plane. Its only purpose
is to serve as the to-entity target for the tooth and body extrudes, so both extrudes end on the same well-defined
face. It must be left visible while those extrudes run, then hidden at the very end of the build with isLightBulbOn =
False (see Sketch Discipline — isVisible = False does **not** hide a construction plane). Keep a handle to it
(ctx.extrusionEndPlane) so the final cleanup can switch its light bulb off.

3: Gear Profile Sketch.

Create a sketch named Gear Profile on the target plane. Inside this sketch the Spur Gear tooth profile generator
draws, in order:

1. **Root Circle** (solid, not construction) at radius Root Circle Radius.
2. **Tip Circle** (construction), Tip Circle Radius.
3. **Base Circle** (construction), Base Circle Radius.
4. **Pitch Circle** (construction), Pitch Circle Radius.

Center every circle on the local-origin SketchPoint by passing it **directly** as the center —
sketchCircles.addByCenterRadius(localOrigin, radius) — so all four share that one point (see Sketch Discipline: share,
don't re-coincident; do not pass localOrigin.geometry and then add a center coincident). Give each a driving diameter
dimension. Each circle is also labeled with along-path sketch text (see the playbook for the exact
sketchTexts.createInput2(...) + setAsAlongPath(...) call). The label string is '{} (r={:.2f},
size={:.2f})'.format(name, radius, size) — the circle's name, its radius, and size, all using the radii's internal
.value (cm) — where size = Tip Circle Radius − Root Circle Radius. Pass that same size as the text **height** argument
to createInput2.

4: Involute Tooth.

Still inside the Gear Profile sketch, draw a single involute tooth centered on the +X direction:

1. Sample a sequence of points along the involute flank, starting on the base circle and walking outward toward the
tip circle in equal radial steps (Involute Steps samples in total). The sampling is **endpoint-inclusive**: with steps
= Involute Steps, sample i (for i = 0 … steps−1) sits at radius r = Base Circle Radius + (Tip Circle Radius − Base
Circle Radius) · i / (steps − 1), so the first sample radius is exactly Base Circle Radius and the **last sample is
exactly Tip Circle Radius**. Do **not** clamp the start to max(Base Circle Radius, Root Circle Radius); the flank is
sampled from the base circle even when the base circle sits inside the root circle (the embedded case is detected
later in step 9 from where the flank *start* lands, not by trimming the sampling). Each sample is
calculateInvolutePoint(Base Circle Radius, r) for that step's radius r (the exact math is pinned in the
tooth-generator surface section above). Drop any sample that returns None (i.e. whose radius is below the base circle)
— those sit inside the base circle and have no valid involute.
2. **Watch the spiral direction.** A correctly-formed involute tooth narrows from base to tip, so the left flank's
angular distance above +X must *decrease* as the radius grows. The standard parametric involute (rb*(cos t + t sin t,
sin t - t cos t)) spirals the opposite way — its angular position *increases* with radius — and using it as a left
flank directly produces a tooth that splays outward (wider at the tip than at the root). Before rotating, mirror the
samples across the +X axis (negate y) so the spiral matches a left flank's shape. The rotation in the next step lifts
the mirrored curve from −Y back up into +Y where the left flank belongs.
3. Decide how far to rotate the (mirrored) sequence so the tooth ends up symmetric about +X. Measure where the
mirrored involute crosses the pitch circle, then rotate by exactly the amount that lands that pitch-circle crossing at
angle +π / (2 · ToothNumber) above +X. (The angular width of a single tooth at the pitch circle is π / Tooth Number,
so half that — π / (2 · ToothNumber) — is where the left flank's pitch crossing must end up.) Compute the pitch-circle
crossing angle **analytically** — evaluate calculateInvolutePoint(Base Circle Radius, Pitch Circle Radius) and take
its polar angle — rather than interpolating between the sampled flank points; the analytic value places the tooth at
exactly the right angle regardless of how few involute samples are taken. **Exact expression (pin the sign — it
interacts with the step-2 mirror):** with (px, py) = calculateInvolutePoint(Base Circle Radius, Pitch Circle Radius),
the *mirrored* pitch crossing sits at polar angle atan2(−py, px), so rotate_angle = π / (2 · ToothNumber) − atan2(−py,
px). (The −py is the step-2 mirror applied to the analytic point; do not take atan2(py, px).)
4. Rotate the (mirrored) sampled points by rotate_angle. This produces the **left** flank. Mirror that result across
the X axis to produce the **right** flank. You now have a tooth symmetric about +X.
   **Then apply the requested angle.** The generator's draw(anchorPoint, angle=0) takes an angle (0 for spur; the
helix angle for helical; 180° for the bevel virtual tooth) — the seed tooth must end up rotated by exactly that. Do
this by rotating the **whole** +X-centered tooth by angle right here, in the same Python point math: rotate both flank
point collections by angle (and, below, place the tooth-top point and seed the rib midpoints at the rotated positions
too). Draw the tooth directly at its final angular position. Do **not** instead leave the tooth at +X and rely on the
spine's angular dimension to swing it into place after the fact — the wrong-solver-branch failure this causes (and why
it ruins the helical loft) is owned by [SPUR-F-ROTATE-CONFIRM]. Because both the bottom (angle = 0) and top (angle =
helixAngle) profiles share the same rotate_angle baseline and differ by exactly angle, the loft twists by exactly the
helix angle regardless of the absolute baseline. For angle = 0 this whole step is a no-op (rotating by 0).
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

  dimension itself exists for **every** angle including 0, because it is what says which way the
  spine points ([SPUR-F-SPINE]); at angle = 0 it is created at 0 and there is simply nothing to
  set afterwards.

Per-step constraint recipes (the over-constraint-sensitive ones).

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
  equally happy running the other way, which is one of the ways the tooth ends up reversed. Without that
origin-to-first dim the whole rib chain has
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
  answers available. The dimensions' captured directions are what rule the far one out. This common case yields a
tooth loop
  of **6 curves** (2 splines + 2 flank-to-root lines + 2 arcs). If instead the flank starts
  **inside** the root circle (**high** tooth counts drop the base circle below the root: it happens
  above 2.5 / (1 - cos(PressureAngle)) teeth, which is 41.5 at 20°, 78.5 at 14.5° and 26.7 at
  25°, so a larger pressure angle brings it on sooner), no flank-to-root line is drawn and the loop has **4 curves**
(2 splines + 2
  arcs) — the profile is "embedded."

  **The embedded test is strict <:** with firstRadius the distance from the local origin to the
  left flank's first fit point, embedded = firstRadius < Root Circle Radius (compare raw values,
  no tolerance). Exact equality therefore counts as **non**-embedded and draws a **zero-length**
  flank-to-root stub (root end and flank start coincide) — this is the ill-conditioned region the
  bench proof flags. Keep the strict comparison; do not "improve" it with <= or a tolerance.


For each circle label, use the exact string '{} (r={:.2f}, size={:.2f})', with name, radius,
and size in internal cm, where size = tipRadius-rootRadius. Text height is size.
Along-path text uses True, CenterHorizontalAlignment, and zero character spacing.
Anchor inside draw itself by re-projecting ctx.anchorPoint, taking item zero, and coincident-constraining
self.anchorPoint to that projected point. Set the signed confirming angular value last when angle != 0.
Copy self._lastToothEmbedded to ctx.toothProfileIsEmbedded after draw returns.
In SketchOnly mode, buildMainGearBody now returns with Gear Profile visible. The final cleanup still runs.
The active table is initially one default case. The broader source-required sweep is deferred until the orchestrator completes the first real compile instance.

The proof function is `stepGearProfile`.

<!-- proof-run: proofkit.Run(sketchCases, stepGearProfile) -->

Use these calls with the operands described above:

- `adsk.core.Point3D.create(x, y, 0)`.
- `sketch.sketchPoints.add(point)`.
- `circles.addByCenterRadius(localOrigin, radius)`.
- `dims.addDiameterDimension(circle, offCenterTextPoint)`.
- `sketch.sketchTexts.createInput2(text, size)`.
- `textInput.setAsAlongPath(circle, True, adsk.core.HorizontalAlignments.CenterHorizontalAlignment, 0)`.
- `sketch.sketchTexts.add(textInput)`.
- `adsk.core.ObjectCollection.create()`.
- `fitPoints.add(point)`.
- `splines.add(fitPoints)`.
- `constraints.addCoincident(toothTopPoint, tipCircle)`.
- `arcs.addByCenterStartEnd(localOrigin, rightFlankEndPoint, leftFlankEndPoint)`.
- `constraints.addCoincident(arc.centerSketchPoint, localOrigin)`.
- `lines.addByTwoPoints(localOrigin, toothTopPoint)`.
- `dims.addDistanceDimension(localOrigin, referenceEnd, horizontal, textPoint)`.
- `dims.addDistanceDimension(localOrigin, referenceEnd, vertical, textPoint)`.
- `lines.addByTwoPoints(localOrigin, referenceEnd)`.
- `dims.addAngularDimension(reference, spine, bisectorTextPoint)`.
- `lines.addByTwoPoints(leftFitPoint, rightFitPoint)`.
- `dims.addDistanceDimension(leftFitPoint, rightFitPoint, acrossOrientation, textPoint)`.
- `constraints.addCoincident(midpoint, spine)`.
- `constraints.addMidPoint(midpoint, rib)`.
- `constraints.addPerpendicular(spine, rib)`.
- `dims.addDistanceDimension(previousMidpoint, midpoint, alongOrientation, textPoint)`.
- `lines.addByTwoPoints(rootEndGeometry, flankStartFitPoint)`.
- `dims.addDistanceDimension(localOrigin, rootEnd, horizontal, textPoint)`.
- `dims.addDistanceDimension(localOrigin, rootEnd, vertical, textPoint)`.
- `sketch.project(anchorPoint)`.
- `projection.item(0)`.
- `constraints.addCoincident(self.anchorPoint, projectedAnchor)`.

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
      "span": "adsk.core.Point3D.create(x, y, 0)"
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
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDiameterDimension(circle, offCenterTextPoint)"
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
      "name": "addCoincident",
      "owner": "adsk.fusion.GeometricConstraints",
      "reason": null,
      "receiver": "constraints",
      "role": "required",
      "span": "constraints.addCoincident(toothTopPoint, tipCircle)"
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
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDistanceDimension(localOrigin, referenceEnd, horizontal, textPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDistanceDimension(localOrigin, referenceEnd, vertical, textPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "lines",
      "role": "required",
      "span": "lines.addByTwoPoints(localOrigin, referenceEnd)"
    },
    {
      "condition": null,
      "name": "addAngularDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dims",
      "role": "required",
      "span": "dims.addAngularDimension(reference, spine, bisectorTextPoint)"
    },
    {
      "condition": null,
      "name": "addByTwoPoints",
      "owner": "adsk.fusion.SketchLines",
      "reason": null,
      "receiver": "lines",
      "role": "required",
      "span": "lines.addByTwoPoints(leftFitPoint, rightFitPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDistanceDimension(leftFitPoint, rightFitPoint, acrossOrientation, textPoint)"
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
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDistanceDimension(previousMidpoint, midpoint, alongOrientation, textPoint)"
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
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDistanceDimension(localOrigin, rootEnd, horizontal, textPoint)"
    },
    {
      "condition": null,
      "name": "addDistanceDimension",
      "owner": "adsk.fusion.SketchDimensions",
      "reason": null,
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDistanceDimension(localOrigin, rootEnd, vertical, textPoint)"
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
      "span": "constraints.addCoincident(self.anchorPoint, projectedAnchor)"
    }
  ],
  "citations": [
    {
      "first": 417,
      "last": 479,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 17,
      "last": 213,
      "path": "spec/spurgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L417–479; `spec/spurgear/fusion.md` L17–213.

## 5 `[GO]` Extrude tooth

self.parent._lastToothEmbedded, copied to ctx.toothProfileIsEmbedded in buildSketches) is
   pinned in [SPUR-F-FLANK-ROOT].

5: Anchor the Sketch.

Use NewBodyFeatureOperation. The inherited find_profile_by_curve_counts utility selects
nurbs=2, arcs=2, lines=0 if ctx.toothProfileIsEmbedded else 2.
[PB-PROFILE-MATCH] [PB-NUMERIC-SNAPSHOT] [PB-HIDE-AFTER-USE]
The proof chords the boundary of the actual detected profile because sampled spline trims are not
exact trims for the solid evaluator. It reduces each spline to roughly 30 facets and checks that
the resulting tooth area differs by less than 0.1% from the sampled profile. It does not prove
Fusion NURBS surfaces.

The proof function is `stepExtrudeTooth`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeTooth, assertExtrudeTooth) -->

Use these calls with the operands described above:

- `extrudes.createInput(profile, operation)`.
- `adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`.
- `extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`.
- `extrudes.add(extrudeInput)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.createInput(profile, operation)"
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
    }
  ],
  "citations": [
    {
      "first": 481,
      "last": 485,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L481–485.

## 6 `[GO]` Extrude body

anchor into the Gear Profile sketch (this re-projection is what chains the two sketches together),
then take .item(0) from the returned ObjectCollection. Pass that SketchPoint, not the
collection, to addCoincident(self.anchorPoint, projectedAnchor) with the Gear Profile's local
origin — the sketch point the tooth generator added at (0, 0, 0) in step 4 (the field
self.anchorPoint), *not* sketch.originPoint. Because every piece of geometry above is constrained
relative to the local origin, constraining it here drags the entire tooth profile onto the anchor
as a unit.

**This anchoring happens inside the tooth generator's draw() method itself** — draw() does drawCircles(), then
drawTooth(), then this projection-and-coincidence, then (for angle != 0 only) sets the confirming angular dimension's
value as its very last action, after the anchoring ([SPUR-F-ROTATE-CONFIRM]) — *not* a separate step the generator
performs after draw() returns. This matters because helical/herringbone build their twisted loft profile by calling
SpurGearInvoluteToothDesignGenerator(loftSketch, self).draw(ctx.anchorPoint, angle=…) directly and rely on that single
call to anchor the sketch. If the anchoring were moved up into buildSketches, the twisted sketch would be left
unconstrained and the loft would float off the anchor.

6: Sketch-Only Short-Circuit.

If the Generate-Sketches-Only input is true, make the Gear Profile sketch visible and stop — skip the remaining body
operations (tooth/body/pattern/fillet/bore). The end-of-build cleanup still runs in this mode; its per-mode split
(planes/axes always hidden, sketches left visible for inspection — the whole point of this mode) is owned by
[SPUR-F-CLEANUP].
Use NewBodyFeatureOperation. Find the solid disc with the shared helper and arcs=2.
Name the feature Extrude body and the body Gear Body. Preserve ctx.gearBody and ctx.extrusionExtent.
The two root arcs are asserted on the real Gear Profile sketch. The solid fixture uses a faceted
root boundary containing every tooth's root vertices, which lets the final-profile proof join
the tooth outlines without relying on a near-tangent Boolean operation.
[PB-PROFILE-MATCH] [PB-HIDE-AFTER-USE] [PB-EMPTY-RESULT]

The proof function is `stepExtrudeBody`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeBody, assertExtrudeBody) -->

Use these calls with the operands described above:

- `extrudes.createInput(profile, operation)`.
- `adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionEndPlane, False)`.
- `extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`.
- `extrudes.add(extrudeInput)`.
- `sketchPlane.isParallelToPlane(face.geometry)`.
- `sketchPlane.isCoPlanarTo(face.geometry)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.ExtrudeFeatures",
      "reason": null,
      "receiver": "extrudes",
      "role": "required",
      "span": "extrudes.createInput(profile, operation)"
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
    }
  ],
  "citations": [
    {
      "first": 487,
      "last": 499,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L487–499.

## 7 `[PROSE]` Gear Center axis

During buildBody, create one construction axis from a cylindrical face of the extruded root disc.
Name it Gear Center, set isLightBulbOn=False, and store ctx.centerAxis. Raise if no cylinder exists.
The solid proof uses the same axial direction for rotation; Fusion construction-axis creation is prose.
[PB-CONSTRUCTION-AXES] [PB-HIDE-AFTER-USE] [PB-EMPTY-RESULT]

Use these calls with the operands described above:

- `axes.createInput()`.
- `axisInput.setByCircularFace(cylindricalFace)`.
- `axes.add(axisInput)`.

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
      "first": 491,
      "last": 499,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L491–499.

## 8 `[GO]` Pattern teeth

In patternTeeth, pattern ctx.toothBody about ctx.centerAxis. Quantity is the numeric ToothNumber;
totalAngle is the literal expression '360 deg', and isSymmetric=False. Supply the seed in an
ObjectCollection. The result already contains the seed and all copies. [PB-CIRCULAR-PATTERN]
[PB-PATTERN-BODIES] [PB-NUMERIC-SNAPSHOT]

The proof function is `stepPatternTeeth`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->

Use these calls with the operands described above:

- `adsk.core.ObjectCollection.create()`.
- `entities.add(ctx.toothBody)`.
- `patterns.createInput(entities, ctx.centerAxis)`.
- `adsk.core.ValueInput.createByReal(toothNumber)`.
- `adsk.core.ValueInput.createByString("360 deg")`.
- `patterns.add(patternInput)`.

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
      "span": "adsk.core.ValueInput.createByString(\"360 deg\")"
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
      "first": 501,
      "last": 509,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L501–509.

## 9 `[GO]` Combine teeth

In patternTeeth after the pattern, copy every pattern body into one ObjectCollection.
Do not separately add the original tooth because pattern.bodies already includes it.
Combine that collection into ctx.gearBody with JoinFeatureOperation in one Fusion Combine feature.
The solid proof extrudes one contour assembled from the faceted root and all patterned tooth
outlines. It checks the resulting solid and volume. It does not exercise the individual joins:
Decad's pairwise union of these sampled profiles exceeds its 4,096-segment safety cap.
[PB-PATTERN-BODIES]

The proof function is `stepCombineTeeth`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineTeeth, assertCombineTeeth) -->

Use these calls with the operands described above:

- `adsk.core.ObjectCollection.create()`.
- `tools.add(patternBody)`.
- `combines.createInput(ctx.gearBody, tools)`.
- `combines.add(combineInput)`.

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
      "receiver": "tools",
      "role": "required",
      "span": "tools.add(patternBody)"
    },
    {
      "condition": null,
      "name": "createInput",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combines",
      "role": "required",
      "span": "combines.createInput(ctx.gearBody, tools)"
    },
    {
      "condition": null,
      "name": "add",
      "owner": "adsk.fusion.CombineFeatures",
      "reason": null,
      "receiver": "combines",
      "role": "required",
      "span": "combines.add(combineInput)"
    }
  ],
  "citations": [
    {
      "first": 503,
      "last": 509,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L503–509.

## 10 `[GO]` Root fillets

While iterating the new body's faces (extrude.bodies.item(0).faces), capture two references needed later. Classify
each face by face.geometry.surfaceType:

- **Gear Center construction axis** — from any face whose surfaceType is CylinderSurfaceType. Build it with
constructionAxes.createInput() → axisInput.setByCircularFace(cylindrical_face) → constructionAxes.add(axisInput). Name
it Gear Center; set isLightBulbOn = False. Store on ctx.centerAxis.
- **ctx.extrusionExtent** (the far end-cap face, used later by the bore cut) — among faces whose surfaceType is
PlaneSurfaceType, the one that is parallel to but **not** coplanar with the gear's sketch plane. Test it with the
plane-geometry API rather than a hand-rolled dot-product: let sketchPlane =
ctx.gearProfileSketch.referencePlane.geometry, and pick the face where sketchPlane.isParallelToPlane(face.geometry)
and not sketchPlane.isCoPlanarTo(face.geometry). (The near cap is coplanar with the sketch plane, so isCoPlanarTo
rules it out; the cylindrical and side faces aren't planar.) Raise if either reference isn't found.

Finally store ctx.gearBody (the Gear Body body).

10: Pattern Teeth.

Circular-pattern ctx.toothBody around the Gear Center axis, quantity = Tooth Number. Pin the two other pattern inputs:
patternInput.totalAngle = ValueInput.createByString('360 deg') (a full turn, set as a string expression) and
patternInput.isSymmetric = False. Combine the patterned tooth bodies into Gear Body via a single Combine-Join.

Copy the pattern's bodies into an adsk.core.ObjectCollection and pass that collection to
the combine. Do not add the original tooth body separately: pattern.bodies already includes it,
per [PB-PATTERN-BODIES].

11: Fillets.
Match all cylinder radii to rootRadius within 0.0001 cm. Keep only Line3DCurveType edges.
Normalize each geometry startPoint-to-endPoint direction, and require abs(abs(dot)-1.0)<0.01.
Pass a ValueInput containing current FilletRadius. Skip FilletRadius<=0 or an empty edge set.
[PB-FILLET-CHAMFER] [PB-NUMERIC-SNAPSHOT] [PB-EMPTY-RESULT]
The local API database omits the source-required input method; its required role is unchanged.

The proof function is `stepRootFillets`.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepRootFillets, assertRootFillets) -->

Use these calls with the operands described above:

- `edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)`.
- `direction.normalize()`.
- `direction.dotProduct(axisNormal)`.
- `fillets.createInput()`.
- `adsk.core.ValueInput.createByReal(radius)`.
- `filletInput.addConstantRadiusEdgeSet(edges, radiusValue, False)`.
- `fillets.add(filletInput)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "vectorTo",
      "owner": "adsk.core.Point3D",
      "reason": null,
      "receiver": "edge.geometry.startPoint",
      "role": "required",
      "span": "edge.geometry.startPoint.vectorTo(edge.geometry.endPoint)"
    },
    {
      "condition": null,
      "name": "normalize",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "direction",
      "role": "required",
      "span": "direction.normalize()"
    },
    {
      "condition": null,
      "name": "dotProduct",
      "owner": "adsk.core.Vector3D",
      "reason": null,
      "receiver": "direction",
      "role": "required",
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
      "span": "adsk.core.ValueInput.createByReal(radius)"
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
      "first": 126,
      "last": 129,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 511,
      "last": 526,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L126–129; `spec/spurgear/instructions.md` L511–526.

## 11 `[GO]` Bore Profile sketch

If Fillet Radius > 0, round the corner where the root valley floor meets each tooth flank — the sharp inside corner
that runs the full thickness of the gear, parallel to its main axis. This is where bending stress concentrates at the
tooth root, so it's the structurally important fillet (not the front/back rim, which is a cosmetic rounding the user
doesn't want here). Two things make picking the right edges fiddly:

- After the pattern-and-combine step, the root cylinder is usually split into one patch per valley rather than a
single continuous surface. Collect *every* cylindrical face whose radius equals Root Circle Radius, not just the first
one found.
- On each such face, keep the **axial straight edges** — the ones whose direction is parallel to the target plane's
normal (i.e. parallel to the gear's main axis). Those are the two valley-floor-to-tooth-flank corners on each valley
patch. Drop the *circular* edges that wrap around the circumference at the front and back end caps; those are end-cap
rims, not the structural root corners. Filter first to Line3DCurveType edges, then take each line's direction from its
**geometry endpoints** (geometry.startPoint.vectorTo(geometry.endPoint)), normalize, and keep it if it is parallel to
the axis within tolerance: abs(abs(dot(direction, axisNormal)) - 1.0) < 0.01. (Use exactly this tolerance — a tighter
test like > 0.999 can drop valid axial edges that are slightly off due to tessellation, leaving root fillets missing.)

Apply the fillet with filletFeatures.createInput() → filletInput.addConstantRadiusEdgeSet(edges, <Fillet Radius
value>, isTangentChain=False) — add the edge set on the input **itself**, per [PB-FILLET-CHAMFER] (do **not** route it
through a filletInput.edgeSetInputs collection — that is the chamfer-side shape, and reaching for it on the fillet
input fails). **isTangentChain must be False** — the collected edges are exactly the axial root corners;
tangent-chaining (True) would let Fusion pull in tangent-adjacent edges and round more than the intended root corner.
Do **not** read the direction via edge.evaluator.getTangent(0) — parameter 0 is not guaranteed to lie inside the
edge's parameter range and Fusion raises RuntimeError: invalid argument parameter.

If the edge collection ends up **empty** (no axial root edge matched), silently skip the fillet — return without
creating the fillet feature, no error. An empty edge set must not reach filletFeatures.add.

12: Bore (optional).

buildBore runs unconditionally from generate() (after buildMainGearBody), so it MUST itself early-return in two cases:
when **SketchOnly** is set, and when **Bore Diameter ≤ 0**. The SketchOnly guard is essential — in sketch-only mode
buildMainGearBody short-circuits before buildBody, so ctx.gearBody and ctx.extrusionExtent are never set; proceeding
into the cut would dereference None. (Do not rely on the bore diameter being 0 in sketch-only mode — the user may have
set both.)

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
Build this sketch only when SketchOnly is false and BoreDiameter>0.
Explicitly set Bore Profile isVisible=True immediately after createSketchObject, before drawing or projection.
Keep the sketch visible through its cut, then cleanup hides it.
The generator constructor's unused local origin must be coincident to the same projected anchor as the
bore circle. Gate this unlabelled sketch on isFullyConstrained.
[SPUR-F-ANCHOR-CHAIN] [SPUR-F-LOCAL-ORIGIN] [PB-CIRCLE-CENTER] [PB-PROJECT-NOT-FIXED]
[PB-FULL-CONSTRAINT] [PB-DRIVING-DIM] [PB-RADIAL-DIM] [PB-SINGLE-PROFILE] [PB-HIDE-AFTER-USE]

The proof function is `stepBoreProfile`.

<!-- proof-run: proofkit.Run(sketchCases, stepBoreProfile) -->

Use these calls with the operands described above:

- `boreSketch.project(ctx.anchorPoint)`.
- `projection.item(0)`.
- `circles.addByCenterRadius(projectedAnchor, boreDiameter / 2)`.
- `dims.addDiameterDimension(circle, offCenterTextPoint)`.
- `constraints.addCoincident(toothGen.anchorPoint, projectedAnchor)`.

<!-- step-meta
{
  "calls": [
    {
      "condition": null,
      "name": "project",
      "owner": "adsk.fusion.Sketch",
      "reason": null,
      "receiver": "boreSketch",
      "role": "required",
      "span": "boreSketch.project(ctx.anchorPoint)"
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
      "receiver": "dims",
      "role": "required",
      "span": "dims.addDiameterDimension(circle, offCenterTextPoint)"
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
      "first": 528,
      "last": 552,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 17,
      "last": 31,
      "path": "spec/spurgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L528–552; `spec/spurgear/fusion.md` L17–31.

## 12 `[GO]` Bore cut

In buildBore after the bore sketch, extrude its sole profile with CutFeatureOperation to
ctx.extrusionExtent, the far planar face. Restrict participantBodies to [ctx.gearBody].
Use PositiveExtentDirection. Skip SketchOnly and BoreDiameter<=0 before touching body references.
[PB-SINGLE-PROFILE] [PB-HIDE-AFTER-USE] [PB-NUMERIC-SNAPSHOT]

The proof function is `stepBoreCut`.

The solid proof extrudes the final contour with one circular bore hole and checks its volume.
It proves the through-hole geometry. Decad's Boolean cut after root fillets exceeds the same
segment cap, so the proof does not exercise the separate Cut feature or its operation order.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepBoreCut, assertBoreCut) -->

Use these calls with the operands described above:

- `extrudes.createInput(profile, adsk.fusion.FeatureOperations.CutFeatureOperation)`.
- `adsk.fusion.ToEntityExtentDefinition.create(ctx.extrusionExtent, False)`.
- `extrudeInput.setOneSideExtent(extent, adsk.fusion.ExtentDirections.PositiveExtentDirection)`.
- `extrudes.add(extrudeInput)`.

<!-- step-meta
{
  "calls": [
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
      "first": 528,
      "last": 552,
      "path": "spec/spurgear/instructions.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L528–552.

## 13 `[PROSE]` Completed gear chamfer

the gear, and [PB-CIRCLE-CENTER] records a solver failure from constraining to originPoint. Without any
grounding the point is free in two directions and the sketch never reaches isFullyConstrained. Then
extrude-cut the bore profile from the target plane to ctx.extrusionExtent (the far end-cap face), affecting
only ctx.gearBody. The ToEntityExtentDefinition to the far face guarantees the bore goes all the way
through regardless of Thickness.

13: Chamfer Completed Gear (optional).

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
This remains PROSE pending an engine recipe for both end caps after root fillets and the bore.
The proof file records the missing confirmed edge-selector composition; the available tested recipe
only proves one cap per independent cylinder. Do not replace this operation with that cylinder recipe.
[PB-FILLET-CHAMFER] [PB-EMPTY-RESULT] [PB-NUMERIC-SNAPSHOT] [HELI-F-CHAMFER-COUNT]
The queried API names the final False argument isTangentChain; preserve its False value.

Use these calls with the operands described above:

- `sketchPlane.isParallelToPlane(face.geometry)`.
- `chamfers.createInput2()`.
- `adsk.core.ValueInput.createByReal(chamferDistance)`.
- `chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, False)`.
- `chamfers.add(chamferInput)`.

<!-- step-meta
{
  "calls": [
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
      "span": "adsk.core.ValueInput.createByReal(chamferDistance)"
    },
    {
      "condition": null,
      "name": "addEqualDistanceChamferEdgeSet",
      "owner": "adsk.fusion.ChamferEdgeSets",
      "reason": null,
      "receiver": "chamferInput.chamferEdgeSets",
      "role": "required",
      "span": "chamferInput.chamferEdgeSets.addEqualDistanceChamferEdgeSet(edges, distance, False)"
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
      "first": 558,
      "last": 573,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 62,
      "last": 73,
      "path": "spec/helicalgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L558–573; `spec/helicalgear/fusion.md` L62–73.

## 14 `[PROSE]` Final cleanup

Call cleanup unconditionally as the last action of generate, after buildBore and chamferTeeth.
Always set isLightBulbOn=False on each construction plane or axis this generation actually created:
normalized target plane, Extrusion End Plane, Gear Center. Check each handle before hiding it.
On the full-build path only, set isVisible=False on Tools, Gear Profile, and Bore Profile if present.
In SketchOnly mode, leave Tools and Gear Profile visible while still hiding created planes and axes.
This visibility orchestration has no geometric engine counterpart. [SPUR-F-CLEANUP] [PB-HIDE-AFTER-USE]

<!-- step-meta
{
  "calls": [],
  "citations": [
    {
      "first": 294,
      "last": 301,
      "path": "spec/spurgear/instructions.md"
    },
    {
      "first": 217,
      "last": 235,
      "path": "spec/spurgear/fusion.md"
    }
  ],
  "schema": 2
}
-->

**From:** `spec/spurgear/instructions.md` L294–301; `spec/spurgear/fusion.md` L217–235.
