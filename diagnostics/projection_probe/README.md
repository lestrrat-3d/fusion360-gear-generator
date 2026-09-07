# Projection constraint probe

This script records the native Fusion behavior needed to resolve the conflicting spur-gear source
rules. It does not generate a gear. It creates one new, unsaved design containing a generic point,
three projection sketches, and a circle. It verifies that the new document is active before it reads
the active Design, and it never saves or closes a document.

The fixture uses a fixed source point in a root-component XY sketch. It projects that point with the
legacy `Sketch.project(entity)` call into a child-component Tools sketch, then reprojects the Tools
point into a Bore sketch. The Bore sketch contains a movable local-origin point, a circle centered on
the reprojected anchor, a driving diameter dimension, and a coincidence between the local origin and
that exact projected point. A separate sketch tests the fixed-local-anchor repair hypothesized when
probe commit `a128e1e22babac4adebcab8154b36ce6bfee3660` was authored.

## Retained native result

The repository retains
[`projection_probe_results_20260907T192923_209867.json.gz`](results/projection_probe_results_20260907T192923_209867.json.gz).
The gzip stream preserves the returned JSON's exact CRLF bytes. The run started at
`2026-09-07T10:29:23.209916+00:00` in Fusion 2704.1.53. The decompressed JSON's SHA-256 is
`71564ab2f16c12e6004fddf4d221bdc68e25b09e8d62be5bd59cfe7263e89885`. The exact probe came from
commit `a128e1e22babac4adebcab8154b36ce6bfee3660`; `projection_probe.py` at that commit has SHA-256
`ad2fb2012681021303e217e68fcfc0209644cdbbc656258e8553a68eafb7f516`.

The run records these version-scoped facts:

- The Tools projection reported `isFixed=false`, `isFullyConstrained=true`, `isLinked=true`, and
  `isReference=true`; its reference-only sketch reported fully constrained.
- Before the Bore coincidence, the diameter dimension reported driving, and the projected anchor
  and circle reported fully constrained, while the free local origin and the whole sketch did not.
  After
  `addCoincident(local_origin, projected_anchor)`, the origin, circle, and sketch all reported fully
  constrained.
- Moving the source succeeded. The source, Tools projection, Bore projection, Bore circle centre,
  and Bore local origin each moved by `(0.5, 0.25, 0)` cm in world space.
- The later attempt to coincide a second, independently fixed local point with a projection raised
  `VCS_SKETCH_OVER_CONSTRAINTS`.

The retained run is a partial error report: its first five stages completed, the
fixed-anchor repair stage failed, and the overall native run status is `error`. Therefore
post-repair source movement is unknown. The fixture does not establish behavior for construction
points, tilted planes, deeper assemblies, or other geometry types.

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

## Historical decision rules

Treat a property with `status = "unavailable"` or `status = "error"` as unknown. It is distinct from
an available property whose `value` is `false`. Autodesk documents point and whole-sketch constraint
state separately through
[`SketchPoint.isFullyConstrained`][point-constraint]
and
[`Sketch.isFullyConstrained`][sketch-constraint].

The following rules record the hypotheses used to interpret a run when probe commit
`a128e1e22babac4adebcab8154b36ce6bfee3660` was authored. The retained result above resolves them;
they are not current construction guidance.

- If the reference-only Tools point or sketch reports `false`, that supports the historical
  universal free-DOF hypothesis. If both report `true`, that hypothesis needs correction. Keep this
  result separate from the Bore result.
- If `build_bore_recipe.after_coincidence.sketch.is_fully_constrained.value` is `true` and the first
  movement stage shows matching nonzero world deltas through the chain, that supports the current
  movable-local-origin Bore recipe and its anchor-following claim.
- If the Bore sketch remains under-constrained after coincidence, that supports the historical
  hypothesis only when the report also verifies that the source was fixed, the diameter dimension
  was driving, and the local origin was free before coincidence. The source still needs a repair
  that preserves anchor following.
- If the fixed-anchor repair becomes fully constrained and the second source move also propagates,
  that supports the historical repair hypothesis in this one component context.
- If the fixed-anchor repair blocks the second move or leaves the projected point unchanged, it
  freezes tracking and cannot satisfy the spur rule that the gear follows its source anchor.

This fixture covers one root-to-child projection chain on parallel XY planes. It does not establish
behavior for construction points, tilted planes, deeper assemblies, or every geometry type.

[sketch-project2]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Sketch_project2.htm
[circle-center-radius]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/SketchCircles_addByCenterRadius.htm
[point-move]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/SketchPoint_move.htm
[point-constraint]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/SketchPoint_isFullyConstrained.htm
[sketch-constraint]: https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/Sketch_isFullyConstrained.htm
