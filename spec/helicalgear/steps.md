The proof files are `proof/helicalgear/sketches_test.go`, `proof/helicalgear/solids_test.go`, and `proof/helicalgear/zz_registrations_test.go`.

<!-- step-metadata: 2 -->

## Provenance

| file | `git hash-object` |
|---|---|
| `spec/helicalgear/instructions.md` | `631233a7c9831b72e39b60504226f4ba99b3ae32` |
| `spec/helicalgear/fusion.md` | `6cbe03029a0b89b001479234c0af422deb589dbf` |
| `spec/helicalgear/contract.json` | `b76c17b25ecc90caad199e10f4bb35308899ee79` |
| `spec/helicalgear/exact_values.json` | `e19b2492f1f74ac3289797dd6d58eddcfa94d102` |
| `spec/spurgear/fusion.md` | `5cd1f9f96e043efba42ae42a00ca6c13403e1339` |
| `spec/spurgear/instructions.md` | `859159e9415f0f4a11fce7f723f4cbcc1d3d7d9a` |
| `.claude/skills/generate-gear/PLAYBOOK.md` | `cdd32545b0f8c651752827c6697601f1e32b4d39` |

## Compilation contract

```json
{
  "contract": {
    "_comment": "Machine-readable mirror of the spec's contract sections, checked by .claude/skills/generate-gear/check_contract.py. The spec prose (instructions.md/fusion.md) is authoritative — a mismatch between this file and the spec is a spec bug; fix both together. Helical is subclass-only: it pins its own three classes and the override surface named in instructions.md 'Method contract', and inherits everything else from spur, which its own manifest pins. module_constants pins the two identifiers herringbone imports AND their exact string values. source_guards pins the two recipes the spec chose over an alternative that also builds a solid — reverting either renames nothing, so nothing else here would see it.",
    "classes": {
      "HelicalGearCommandConfigurator": {
        "bases": [
          "SpurGearCommandInputsConfigurator"
        ],
        "methods": [
          "configure"
        ]
      },
      "HelicalGearGenerationContext": {
        "bases": [
          "SpurGearGenerationContext"
        ],
        "ctx_fields": [
          "helixPlane",
          "twistedGearProfileSketch"
        ],
        "methods": [
          "__init__"
        ]
      },
      "HelicalGearGenerator": {
        "bases": [
          "SpurGearGenerator"
        ],
        "methods": [
          "newContext",
          "prefixBase",
          "generateName",
          "addExtraPrimaryParameters",
          "filletHelixFactorExpression",
          "helicalPlaneOffset",
          "buildSketches",
          "buildTooth",
          "loftTooth"
        ]
      }
    },
    "module": "lib/geargen/helicalgear.py",
    "module_constants": {
      "INPUT_ID_HELIX_ANGLE": "helixAngle",
      "PARAM_HELIX_ANGLE": "HelixAngle"
    },
    "source_guards": [
      {
        "file": "lib/geargen/helicalgear.py",
        "in_function": "buildSketches",
        "required": [
          "SpurGearInvoluteToothDesignGenerator\\(",
          "angle\\s*=",
          "PARAM_HELIX_ANGLE"
        ],
        "why": "[SPUR-F-ROTATE-CONFIRM] and [SPUR-F-SPINE]: the twist is delivered as the tooth generator's own draw() angle, which rotates the tooth in its point math and confirms the rotation with the spine's angular dimension. Drawing the tooth flat and rotating the sketch geometry afterward also produces a twisted profile, but leaves the spine dimension measuring the unrotated angle, so the sketch no longer proves its own twist."
      },
      {
        "file": "lib/geargen/helicalgear.py",
        "in_function": "loftTooth",
        "required": [
          "loftSections\\.add\\(bottomToothProfile\\)[\\s\\S]*loftSections\\.add\\(topToothProfile\\)",
          "find_profile_by_curve_counts\\("
        ],
        "why": "[HELI-F-LOFT]: the bottom section is added to loftSections before the top. Adding them in the other order also lofts a valid solid, and the handedness does NOT flip — the proof builds both orders and reads the same twist, same sign, off each centroid. What changes is the ruled walls, which are built outward from the FROM section, and with them about 12% of the volume at a 14.5 degree helix. The swap is therefore silent in the one reading a caller is most likely to check, which is why the order is pinned here rather than left to the implementation."
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
    "PARAM_HELIX_ANGLE": "HelixAngle",
    "INPUT_ID_HELIX_ANGLE": "helixAngle"
  },
  "inputs": [
    {
      "id": "INPUT_ID_HELIX_ANGLE",
      "kind": "value",
      "label": "Helix Angle",
      "unit": "deg",
      "default": {
        "radians": 14.5
      }
    }
  ],
  "parameters": [
    {
      "name": "PARAM_HELIX_ANGLE",
      "unit": "rad",
      "comment": "Helix angle for the helical gear",
      "input": "INPUT_ID_HELIX_ANGLE"
    }
  ]
}
```

## 1 `[PROSE]` Normalize the selected target plane

Helical is a thin specialization of spur. Keep the three class bases and nine generator overrides in the
generated compilation contract. Import explicitly from .spurgear: PARAM_MODULE, PARAM_TOOTH_NUMBER,
PARAM_THICKNESS, SpurGearCommandInputsConfigurator, SpurGearGenerationContext, SpurGearGenerator,
and SpurGearInvoluteToothDesignGenerator. Import get_value from .base and find_profile_by_curve_counts
from .utilities, plus math, adsk.core, and adsk.fusion. Do not import GenerationContext.
The three context-taking overrides annotate ctx as SpurGearGenerationContext, then assert that it is a
HelicalGearGenerationContext before any helical-only field access.

The configurator calls superclass configure first, then appends Helix Angle after Parent Component.
Use the generated Exact values section from spec/helicalgear/exact_values.json for its dialog and parameter fields.
Create its converted default with `adsk.core.ValueInput.createByReal(helixDefault)`, then call
`cmd.commandInputs.addValueInput(INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg', defaultValue)`.
Read the added value with get_value and register it in addExtraPrimaryParameters, after the inherited primary parameters.
Follow [PB-DIALOG-DEFAULT-UNITS], [PB-INPUT-READ], [PB-GET-VALUE-CONTRACT], and [SPUR-SUBCLASS-INPUT].
No angle clamp, warning, or maximum is permitted. Negative radians mean a left-handed twist.

The context initializer calls its superclass initializer and initializes helixPlane and twistedGearProfileSketch
using `adsk.fusion.ConstructionPlane.cast(None)` and `adsk.fusion.Sketch.cast(None)`.
The newContext override returns HelicalGearGenerationContext; prefixBase returns 'HelicalGear'.
The generated name is 'Helical Gear (M={}, Tooth={}, Thickness={}, Angle={})', filled with the four parameters'
expression strings in Module, Tooth Number, Thickness, Helix Angle order.
filletHelixFactorExpression returns the cosine expression around the prefixed HelixAngle parameter name.
helicalPlaneOffset remains a separate override and returns the inherited getParameterAsValueInput result for Thickness.
The value is a numeric snapshot, not a live feature expression [PB-NUMERIC-SNAPSHOT].

Keep the inherited generation order and method boundaries:
processInputs; prepareTools; buildMainGearBody containing buildSketches, buildTooth, buildBody, patternTeeth,
createFillets; buildBore; chamferTeeth; cleanup. buildTooth delegates only to loftTooth.
Do not reimplement the inherited methods or the tooth drawer.

The inherited generator conditionally normalizes a selected planar face to a coplanar construction plane.
Do not activate another occurrence [PB-NEVER-ACTIVATE].
This conditional construction-plane timeline entry has no standalone sketch or solid harness return.
The loft proof constructs and consumes real offset planes, but does not verify Fusion occurrence-context resolution.

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/helicalgear/instructions.md",
      "first": 23,
      "last": 141
    },
    {
      "path": "spec/spurgear/instructions.md",
      "first": 419,
      "last": 422
    }
  ],
  "calls": [
    {
      "span": "adsk.core.ValueInput.createByReal(helixDefault)",
      "name": "createByReal",
      "receiver": "adsk.core.ValueInput",
      "owner": "adsk.core.ValueInput",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "cmd.commandInputs.addValueInput(INPUT_ID_HELIX_ANGLE, 'Helix Angle', 'deg', defaultValue)",
      "name": "addValueInput",
      "receiver": "cmd.commandInputs",
      "owner": "adsk.core.CommandInputs",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "adsk.fusion.ConstructionPlane.cast(None)",
      "name": "cast",
      "receiver": "adsk.fusion.ConstructionPlane",
      "owner": "adsk.fusion.ConstructionPlane",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "adsk.fusion.Sketch.cast(None)",
      "name": "cast",
      "receiver": "adsk.fusion.Sketch",
      "owner": "adsk.fusion.Sketch",
      "role": "required",
      "condition": null,
      "reason": null
    }
  ]
}
-->

**From:** `spec/helicalgear/instructions.md` L23–141; `spec/spurgear/instructions.md` L419–422.

## 2 `[PROSE]` Tools sketch

The inherited prepareTools creates the 'Tools' sketch and projects the user's selected anchor.
Store the first projected SketchPoint as ctx.anchorPoint; every later sketch reprojects this canonical handle
[SPUR-F-ANCHOR-CHAIN]. Keep Tools visible until inherited cleanup.
This sketch contains projected geometry only. proofkit requires authored geometry, so a standalone run would
fail its empty-geometry gate. The profile proofs consume a reference anchor and check their local origins
against it; they do not prove Fusion's associative projection chain.

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 423,
      "last": 433
    },
    {
      "path": "spec/spurgear/fusion.md",
      "first": 17,
      "last": 25
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L423–433; `spec/spurgear/fusion.md` L17–25.

## 3 `[PROSE]` Extrusion End Plane

The inherited prepareTools also creates 'Extrusion End Plane' at full Thickness above the normalized target plane.
Retain ctx.extrusionEndPlane as the body's to-entity target. Leave it visible while features consume it;
inherited cleanup hides it with isLightBulbOn=False in both modes [SPUR-F-CLEANUP], [PB-HIDE-AFTER-USE].
An isolated plane cannot return a sketch or solid to either harness; the solid proofs assert full axial extent.

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 423,
      "last": 433
    },
    {
      "path": "spec/spurgear/fusion.md",
      "first": 225,
      "last": 242
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L423–433; `spec/spurgear/fusion.md` L225–242.

## 4 `[GO]` Bottom Gear Profile sketch

The helical buildSketches first calls `super().buildSketches(ctx)`, preserving the complete inherited bottom sketch.
It draws the spur tooth at angle zero with a projected Tools anchor.
Keep the bottom sketch named 'Gear Profile'. The sketch closes the root disc and one tooth profile.
For non-embedded geometry the tooth has two fitted splines, two arcs, and two root stubs.
The root disc is selected by two arc boundary fragments, not by the construction tip circle.
Use the same construction and constraints described for the twisted sketch below [PB-SKETCH-FIRST].
The proof function is `stepBottomProfile`; it proves three module/tooth-count/sample-count cases.

<!-- proof-run: proofkit.Run(bottomCases, stepBottomProfile) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/helicalgear/instructions.md",
      "first": 107,
      "last": 117
    },
    {
      "path": "spec/spurgear/instructions.md",
      "first": 434,
      "last": 496
    }
  ],
  "calls": [
    {
      "span": "super().buildSketches(ctx)",
      "name": "buildSketches",
      "receiver": null,
      "owner": null,
      "role": "required",
      "condition": null,
      "reason": null
    }
  ]
}
-->

**From:** `spec/helicalgear/instructions.md` L107–117; `spec/spurgear/instructions.md` L434–496.

## 5 `[PROSE]` Helix construction plane

After the bottom sketch, get the gear component's constructionPlanes collection as planes.
Use `planes.createInput()`, `planeInput.setByOffset(self.plane, offset)`,
and `planes.add(planeInput)`, storing the result in ctx.helixPlane.
Obtain offset from `self.helicalPlaneOffset()`, preserving that overridable hook.
Its full-Thickness numeric snapshot is consumed by the top profile and loft [HELI-F-TWIST-PLANE],
[PB-CONSTRUCTION-PLANES], [PB-NUMERIC-SNAPSHOT].
Leave this plane visible after generation, including SketchOnly mode; do not add cleanup.
A standalone plane has no solid or authored sketch for a harness gate; the loft builds a real offset plane.

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/helicalgear/fusion.md",
      "first": 7,
      "last": 42
    }
  ],
  "calls": [
    {
      "span": "planes.createInput()",
      "name": "createInput",
      "receiver": "planes",
      "owner": "adsk.fusion.ConstructionPlanes",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "planeInput.setByOffset(self.plane, offset)",
      "name": "setByOffset",
      "receiver": "planeInput",
      "owner": "adsk.fusion.ConstructionPlaneInput",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "planes.add(planeInput)",
      "name": "add",
      "receiver": "planes",
      "owner": "adsk.fusion.ConstructionPlanes",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "self.helicalPlaneOffset()",
      "name": "helicalPlaneOffset",
      "receiver": "self",
      "owner": null,
      "role": "required",
      "condition": null,
      "reason": null
    }
  ]
}
-->

**From:** `spec/helicalgear/fusion.md` L7–42.

## 6 `[GO]` Twisted Gear Profile sketch

Create the sketch through `self.createSketchObject('Twisted Gear Profile', plane=plane)`.
Keep it hidden for its entire life, including SketchOnly mode [HELI-F-TWIST-PLANE].
Construct `SpurGearInvoluteToothDesignGenerator(loftSketch, self)`, then call
`toothGenerator.draw(ctx.anchorPoint, angle=helixAngle)`, where helixAngle is the raw
HelixAngle parameter value in radians. Store loftSketch as ctx.twistedGearProfileSketch.
Do not draw flat and rotate the sketch afterward. Do not add a runtime full-constraint gate.

The inherited drawer shares one movable local origin among the four circles. Root is solid;
tip, base, and pitch are construction. Their radii are respectively (Module*ToothNumber-2.5*Module)/2,
(Module*ToothNumber+2*Module)/2, Module*ToothNumber*cos(PressureAngle)/2, and Module*ToothNumber/2.
Use driving diameter dimensions and the existing along-path labels.
The label is '{} (r={:.2f}, size={:.2f})', with circle name, internal-cm radius, and
size=TipCircleRadius-RootCircleRadius; size is also the label height.

Sample 15 endpoint-inclusive radial involute points from base to tip, mirror y before pitch alignment,
align the mirrored analytic pitch crossing to pi/(2*ToothNumber), then rotate both flanks by helixAngle.
The cap shares both flank endpoints; its copied center is coincident to the local origin and has no diameter dimension.
Create the tooth-top point at the rotated tip and constrain it onto the tip circle.
The spine shares origin and tooth-top point.
Pin a separate positive-X reference endpoint using horizontal TipCircleRadius and vertical zero dimensions.
The angular dimension's arguments are reference then spine, with text on the intended-angle bisector.

Every fit-point index gets a construction rib sharing its two flank fit points.
Dimension the rib across the spine, seed its midpoint at the left point's projection onto the spine,
then apply point-on-spine, midpoint, and perpendicular in that order.
Omit perpendicular only for the last rib. Dimension each midpoint from the previous midpoint, beginning at
the local origin. Use vertical rib/horizontal chain dimensions when abs(cos(angle)) >= abs(sin(angle));
swap them otherwise. Fusion linear values are magnitudes whose signs come from seeded sides.

For non-embedded flanks, draw two root stubs sharing the spline start points and pin their root ends with
horizontal and vertical dimensions from the local origin. The strict embedded comparison is baseRadius < rootRadius.
Keep the root circle solid so profile detection splits it at the real crossings.
Finally project the Tools anchor, constrain the local origin to the first projected point, then confirm the signed
spine angle for nonzero angles. Follow [PB-SKETCH-FIRST], [PB-SHARE-XOR-COINCIDENT], [PB-NO-OVERCONSTRAIN],
[PB-DRIVING-DIM], [PB-DIM-VALUE-SEMANTICS], [SPUR-F-SPINE], [SPUR-F-RIBS],
[SPUR-F-TOOTHTOP-ARC], [SPUR-F-FLANK-ROOT], [SPUR-F-ROTATE-CONFIRM], and [SPUR-F-LOCAL-ORIGIN].

The proof function is `stepTwistedProfile`. Its 11 cases cover signed zero/quarter/half turns,
coarse and fine sizes, low sample count, and both embedded routes.
The embedded sketches prove the inherited drawer only: helical's fixed six-curve loft selection remains unsupported there.

<!-- proof-run: proofkit.Run(sketchCases, stepTwistedProfile) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/helicalgear/instructions.md",
      "first": 150,
      "last": 175
    },
    {
      "path": "spec/helicalgear/fusion.md",
      "first": 7,
      "last": 42
    },
    {
      "path": "spec/spurgear/fusion.md",
      "first": 17,
      "last": 223
    },
    {
      "path": "spec/spurgear/instructions.md",
      "first": 434,
      "last": 496
    }
  ],
  "calls": [
    {
      "span": "self.createSketchObject('Twisted Gear Profile', plane=plane)",
      "name": "createSketchObject",
      "receiver": "self",
      "owner": null,
      "role": "inherited",
      "condition": null,
      "reason": "base.Generator creates and hides the named sketch."
    },
    {
      "span": "toothGenerator.draw(ctx.anchorPoint, angle=helixAngle)",
      "name": "draw",
      "receiver": "toothGenerator",
      "owner": null,
      "role": "required",
      "condition": null,
      "reason": null
    }
  ]
}
-->

**From:** `spec/helicalgear/instructions.md` L150–175; `spec/helicalgear/fusion.md` L7–42; `spec/spurgear/fusion.md` L17–223; `spec/spurgear/instructions.md` L434–496.

## 7 `[GO]` Loft tooth

Full builds call loftTooth through buildTooth. Find bottomToothProfile with
`find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)` and topToothProfile with
`find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)`.
Both fixed six-curve searches are required; do not branch on ctx.toothProfileIsEmbedded.
Use lofts from the gear component features, then
`lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)`,
`loftInput.loftSections.add(bottomToothProfile)`,
`loftInput.loftSections.add(topToothProfile)`, and `lofts.add(loftInput)`.
Take `loftResult.bodies.item(0)`, store it as ctx.toothBody, and name it 'Tooth Body'.
Bottom precedes top [HELI-F-LOFT], [PB-LOFT], [PB-PROFILE-MATCH].

The proof function is `stepLoftTooth`. The engine refuses fitted-spline section pairs, so the proof
chords both flanks and circular boundaries while preserving all 15 sampled flank points.
It asserts the exact volume of the resulting triangular walls, including the signed diagonal contribution.
It does not establish Fusion spline-surface volume or BRep edge counts.
The measured solid cases are 0 and +/-14.5 degrees at 10 mm thickness and +14.5 degrees at 2 mm.
These are proof cases, not dialog limits or a guarantee for all angles between them.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepLoftTooth, assertLoftTooth) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/helicalgear/fusion.md",
      "first": 44,
      "last": 66
    },
    {
      "path": "spec/helicalgear/instructions.md",
      "first": 33,
      "last": 47
    }
  ],
  "calls": [
    {
      "span": "find_profile_by_curve_counts(ctx.gearProfileSketch, nurbs=2, arcs=2, lines=2)",
      "name": "find_profile_by_curve_counts",
      "receiver": null,
      "owner": null,
      "role": "inherited",
      "condition": null,
      "reason": "The shared utilities helper selects the exact curve-count profile."
    },
    {
      "span": "find_profile_by_curve_counts(ctx.twistedGearProfileSketch, nurbs=2, arcs=2, lines=2)",
      "name": "find_profile_by_curve_counts",
      "receiver": null,
      "owner": null,
      "role": "inherited",
      "condition": null,
      "reason": "The shared utilities helper selects the exact curve-count profile."
    },
    {
      "span": "lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)",
      "name": "createInput",
      "receiver": "lofts",
      "owner": "adsk.fusion.LoftFeatures",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "loftInput.loftSections.add(bottomToothProfile)",
      "name": "add",
      "receiver": "loftInput.loftSections",
      "owner": "adsk.fusion.LoftSections",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "loftInput.loftSections.add(topToothProfile)",
      "name": "add",
      "receiver": "loftInput.loftSections",
      "owner": "adsk.fusion.LoftSections",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "lofts.add(loftInput)",
      "name": "add",
      "receiver": "lofts",
      "owner": "adsk.fusion.LoftFeatures",
      "role": "required",
      "condition": null,
      "reason": null
    },
    {
      "span": "loftResult.bodies.item(0)",
      "name": "item",
      "receiver": "loftResult.bodies",
      "owner": "adsk.fusion.BRepBodies",
      "role": "required",
      "condition": null,
      "reason": null
    }
  ]
}
-->

**From:** `spec/helicalgear/fusion.md` L44–66; `spec/helicalgear/instructions.md` L33–47.

## 8 `[GO]` Extrude root body

The inherited buildBody selects the root disc by two arcs and extrudes it as a New Body to ctx.extrusionEndPlane,
using PositiveExtentDirection and a non-chained to-entity extent. Name the feature 'Extrude body' and body 'Gear Body'.
Store ctx.gearBody and the far planar face in ctx.extrusionExtent.
Select the latter by parallel but not coplanar geometry against the Gear Profile reference plane.
Raise if the far cap or the cylindrical axis source is absent [PB-PROFILE-MATCH], [PB-NUMERIC-SNAPSHOT].
The proof function is `stepExtrudeBody`; it measures the exact root cylinder's volume and bounds.
It models the final disc with one circle; the sketch proof separately checks the tooth-root profile split.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepExtrudeBody, assertExtrudeBody) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 507,
      "last": 517
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L507–517.

## 9 `[PROSE]` Gear Center construction axis

The inherited buildBody creates ctx.centerAxis from a cylindrical face of ctx.gearBody.
Name it 'Gear Center' and set isLightBulbOn=False [PB-CONSTRUCTION-AXES].
A construction axis has no standalone solid or authored sketch output.
The circular placement proof consumes the analytic cylinder axis but does not model Fusion face classification.

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 507,
      "last": 517
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L507–517.

## 10 `[GO]` Circular tooth pattern

The inherited patternTeeth patterns ctx.toothBody around ctx.centerAxis with quantity ToothNumber,
totalAngle '360 deg', and isSymmetric=False [PB-CIRCULAR-PATTERN].
The resulting collection includes the original tooth exactly once [PB-PATTERN-BODIES].
The proof function is `stepPatternTeeth`. It verifies 17 exact rotated copies, including the seed,
in separate documents because shared-document verification cannot decide the faceted pairs' separation.
It compares volume readings with both bounds and asserts each rotated copy's coordinate bounds.
It proves each copy's placement and solidity, but not mutual separation of the full pattern.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepPatternTeeth, assertPatternTeeth) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 518,
      "last": 523
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L518–523.

## 11 `[GO]` Combine patterned teeth

The inherited patternTeeth copies every pattern body, including the original exactly once, into a fresh
ObjectCollection and performs one Combine Join with ctx.gearBody as target [PB-PATTERN-BODIES].
Do not pass BRepBodies where ObjectCollection is required.
The proof function is `stepCombineTeeth`; it performs real sequential unions and requires one surviving connected body.

The exact cylinder-to-chorded-tooth boundary is refused at near-chord contact.
The proof substitutes a circumscribed 68-sided root prism with inradius 7.25 mm, oriented at 0.01 radians.
Its outward radial deviation is bounded by 7.25*(sec(pi/68)-1), approximately 0.00775 mm.
The orientation avoids a collapsed intermediate cap triangle; it does not change the radial bound.
The root prism extends 0.01 mm beyond each tooth cap, adding exactly 0.02 mm to its axial extent,
because coplanar overlapping caps are refused by the engine.
The proof therefore loses the exact cylindrical root and flush root/tooth caps; tooth thickness and placement remain exact.
Keep the specified Fusion cylinder and feature extents unchanged.

<!-- proof-run: proofkit3d.RunSolid(solidCases, stepCombineTeeth, assertCombineTeeth) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 518,
      "last": 523
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L518–523.

## 12 `[PROSE]` Root fillets

The inherited createFillets uses FilletRadius=(RootCircleRadius*ToothSpaceAngleAtRoot/2)*0.9*cos(HelixAngle),
where ToothSpaceAngleAtRoot=pi/ToothNumber-2*(tan(PressureAngle)-PressureAngle).
Skip if the radius is nonpositive.
Select every cylindrical root face within 0.0001 cm of RootCircleRadius, then only straight edges whose
normalized direction satisfies abs(abs(dot(direction,axisNormal))-1)<0.01.
Use geometry endpoints for the direction; do not sample an evaluator at arbitrary parameter zero.
Silently return when no edges match. Set isTangentChain=False on the required inherited constant-radius edge set
[PB-FILLET-CHAMFER]. Preserve the input-owned addConstantRadiusEdgeSet recipe in the source.

The engine's fillet implementation accepts straight-prism lateral edges, but refuses completed non-prism loft/union receivers.
A straight-prism demonstration would remove the requested helical geometry and its root joins.
No equivalent completed-gear fillet substitute is established, so this operation remains unproved.
The local API database disagrees with the inherited input-owned method; this advisory does not authorize changing it.

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 524,
      "last": 534
    },
    {
      "path": "spec/helicalgear/instructions.md",
      "first": 26,
      "last": 31
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L524–534; `spec/helicalgear/instructions.md` L26–31.

## 13 `[GO]` Bore Profile sketch

The inherited buildBore returns before creating this step when SketchOnly is true or BoreDiameter<=0.
Otherwise create 'Bore Profile' on the target plane and use the inherited tooth drawer's drawBore method.
It projects ctx.anchorPoint, uses the first SketchPoint as the circle center, draws one solid bore circle with
a driving diameter, and constrains the drawer's otherwise unused local origin to the same projected anchor.
Do not constrain that origin to the immutable sketch origin [PB-DRIVING-DIM], [SPUR-F-LOCAL-ORIGIN],
[SPUR-F-ANCHOR-CHAIN]. The proof function is `stepBoreProfile`.

<!-- proof-run: proofkit.Run(boreSketchCases, stepBoreProfile) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 535,
      "last": 557
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L535–557.

## 14 `[GO]` Cut the bore

The inherited buildBore extrude-cuts the bore profile to ctx.extrusionExtent with a non-chained to-entity
extent and PositiveExtentDirection. Restrict participants to ctx.gearBody [PB-NUMERIC-SNAPSHOT].
The proof function is `stepBoreCut`. It consumes the actual substituted combined gear and checks the exact
removed cylinder volume against bounded before/after readings. A zero-radius case proves the disabled path.
The proof cutting tool extends beyond the proof-only root overhang; it does not prove Fusion's to-entity feature.
Keep this proof step serial because its build hands the consumed body's bounded volume to the assertion.

<!-- proof-run: proofkit3d.RunSolid(boreCases, stepBoreCut, assertBoreCut) -->

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 535,
      "last": 557
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L535–557.

## 15 `[PROSE]` Completed-gear chamfer and final visibility

The inherited chamferTeeth runs after root fillets and the optional bore.
Return for SketchOnly or zero ChamferTooth. Walk every planar completed-gear face parallel to the Gear Profile plane,
including both ends; gather all boundary edges once by tempId.
Keep root-radius arcs. Exclude only circular edges matching a positive BoreDiameter/2 within 0.001 cm.
Raise if no end-cap face or no chamfer edge remains.
Use the inherited createInput2 chamfer and equal-distance edge set with the final boolean False.
Follow [HELI-F-CHAMFER-COUNT] and [PB-FILLET-CHAMFER]; do not reintroduce a tooth-cap edge-count predicate.
Completed helical union bodies are non-prism receivers refused by the engine chamfer operation.
No substitute preserving that final geometry and edge selection is established.
The spec also leaves final Fusion end-cap selection verification pending.

Cleanup is the last inherited action, in both modes [SPUR-F-CLEANUP], [PB-HIDE-AFTER-USE].
Always hide normalized and Extrusion End planes and Gear Center axis, individually guarding absent entities.
Only the full-build path hides Tools, Gear Profile, and Bore Profile sketches.
SketchOnly shows the bottom Gear Profile, skips every solid operation, and still creates the hidden twisted sketch.
The helix plane remains visible; the twisted sketch remains hidden. Add no helical cleanup override.

<!-- step-meta
{
  "schema": 2,
  "citations": [
    {
      "path": "spec/spurgear/instructions.md",
      "first": 558,
      "last": 573
    },
    {
      "path": "spec/helicalgear/fusion.md",
      "first": 26,
      "last": 42
    },
    {
      "path": "spec/helicalgear/fusion.md",
      "first": 67,
      "last": 80
    },
    {
      "path": "spec/spurgear/fusion.md",
      "first": 225,
      "last": 242
    }
  ],
  "calls": []
}
-->

**From:** `spec/spurgear/instructions.md` L558–573; `spec/helicalgear/fusion.md` L26–42; `spec/helicalgear/fusion.md` L67–80; `spec/spurgear/fusion.md` L225–242.
