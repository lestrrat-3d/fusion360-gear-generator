# Projection constraint probe

This script records the native Fusion behavior needed to resolve the conflicting spur-gear source
rules. It does not generate a gear. It creates one new, unsaved design containing a generic point,
three projection sketches, and a circle. It verifies that the new document is active before it reads
the active Design, and it never saves or closes a document.

The fixture uses a fixed source point in a root-component XY sketch. It projects that point with the
legacy `Sketch.project(entity)` call into a child-component Tools sketch, then reprojects the Tools
point into a Bore sketch. The Bore sketch contains a movable local-origin point, a circle centered on
the reprojected anchor, a driving diameter dimension, and a coincidence between the local origin and
that exact projected point. A separate sketch tests the playbook's proposed coincidence to an
already-fixed point.

## Run in Fusion

1. Copy this whole `projection_probe` directory into Fusion's API Scripts directory:
   - Windows: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\Scripts\projection_probe`
   - macOS: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/projection_probe`
2. Open **Utilities > Scripts and Add-Ins**, select the **Scripts** tab, select
   **projection_probe**, and click **Run**.
3. Keep the new unsaved **Projection Constraint Probe** design open for visual inspection.
4. Return the newest `projection_probe_results_<timestamp>.json` beside this script. Each run gets a
   unique file, and each completed stage is written before the next native operation starts.

The script never substitutes `Sketch.project2`. If the legacy call is unavailable, its stage records
`projection_call.status = "unavailable"` and stops with all earlier evidence intact. The repository's
API database does not declare the legacy member, while Autodesk's current API reference documents
the newer [`Sketch.project2`][sketch-project2] signature. Only the native probe establishes whether
the exact legacy call used by the source is available in the installed Fusion version.

The documented [`SketchCircles.addByCenterRadius`][circle-center-radius] signature accepts an existing
`SketchPoint` as its center. The probe passes the reprojected anchor directly, then records that point
and the returned circle's `centerSketchPoint` separately. The coincidence still targets the exact
projection variable, matching the source recipe without assuming object identity.

The two movement stages unfix only the probe's local source point, call the documented
[`SketchPoint.move`][point-move],
refix it, and force a design recompute. Every point's world position is captured before and after.
Autodesk documents `move` as respecting constraints and returning whether it succeeded. A `false`
return is recorded as `false`; it is never treated as a successful move.

## Read the result

Treat a property with `status = "unavailable"` or `status = "error"` as unknown. It is distinct from
an available property whose `value` is `false`. Autodesk documents point and whole-sketch constraint
state separately through
[`SketchPoint.isFullyConstrained`][point-constraint]
and
[`Sketch.isFullyConstrained`][sketch-constraint].

- If the reference-only Tools point or sketch reports `false`, that supports the playbook claim for
  reference-only projection geometry. If both report `true`, the playbook's universal statement needs
  correction. Keep this result separate from the Bore result.
- If `build_bore_recipe.after_coincidence.sketch.is_fully_constrained.value` is `true` and the first
  movement stage shows matching nonzero world deltas through the chain, that supports the current
  movable-local-origin Bore recipe and its anchor-following claim.
- If the Bore sketch remains under-constrained after coincidence, that supports the playbook's claim
  only when the report also verifies that the source was fixed, the diameter dimension was driving,
  and the local origin was free before coincidence. The source still needs a repair that preserves
  anchor following.
- If the fixed-anchor repair becomes fully constrained and the second source move also propagates,
  that supports the playbook's proposed repair in this one component context.
- If the fixed-anchor repair blocks the second move or leaves the projected point unchanged, it
  freezes tracking and cannot satisfy the spur rule that the gear follows its source anchor.

This fixture covers one root-to-child projection chain on parallel XY planes. It does not establish
behavior for construction points, tilted planes, deeper assemblies, or every geometry type.

[sketch-project2]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Sketch_project2.htm
[circle-center-radius]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/SketchCircles_addByCenterRadius.htm
[point-move]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/SketchPoint_move.htm
[point-constraint]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/SketchPoint_isFullyConstrained.htm
[sketch-constraint]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/Sketch_isFullyConstrained.htm
