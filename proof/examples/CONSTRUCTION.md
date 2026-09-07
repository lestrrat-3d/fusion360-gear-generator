# Solid proof construction

Use these generic recipes to construct proof bodies through public engine APIs.
Read [the executable source](proofkit3d_construction_example_test.go) for implementation.
Each recipe shares its construction helpers between its regression Test and executable Example.
These examples do not replace a gear's generated proof or establish Fusion runtime behavior.

## Verification and units

- Use millimeters for lengths, square millimeters for profile areas, and cubic millimeters for volumes.
- Use `t.Context()` in tests. Construction helpers accept `context.Context` and return bodies with
  their owning documents or an error; they do not accept `testing.T`.
- Run `proofkit3d.RequireSound` or `proofkit3d.RequireSolid` before regression measurement assertions.
  Read each body's verified measurement with `proofkit3d.BodyReport`.
- Executable Examples call `doc.Verify` with the default tolerance and require `report.Trustworthy()`.
- Compare each volume with its independent closed form using
  `abs(measured - expected) <= reported_bound + 1e-8` in cubic millimeters.
  For removed volume, add the two reported bounds. The allowance covers floating-point evaluation
  of the closed form; it does not waive a harness volume-bound failure.
- `Profile.Area` has no reported bound. These line/circle fixtures compare its closed-form value
  with the same numerical allowance in square millimeters.
- Keep the default gate and tolerance. Report unsupported constructions instead of weakening assertions.

## Independent comparison bodies

Run `Example_proofkit3d_independent_documents` and `TestConstructionIndependentDocuments` in
[the executable source](proofkit3d_construction_example_test.go).

Construct each 20 by 20 by 10 comparison block in its own document and verify each document.
Each volume is 4000. Alternatively, keep two blocks in one document at x = 0 and x = 100,
then verify both volumes. The separation prevents interference between comparison bodies.
Resolve body reports only within the owning document; never compare body pointers across documents.

## A profile with a bore

Run `Example_proofkit3d_annular_profile` and `TestConstructionAnnularProfile` in
[the executable source](proofkit3d_construction_example_test.go).

Solve concentric radius-10 and radius-2 circles. Select the unique valid profile with one hole,
rather than assuming `Profiles()[0]` is the annulus. Its area is `96*pi`; the inner disk's area is `4*pi`.
Extrude the annulus by 8 for volume `768*pi`. Compare with a radius-10, height-8 disk in another
document, whose volume is `800*pi`. The removed volume is `32*pi`.

`Profile.Entities` contains distinct source entities on the outer boundary. `Profile.Outer` contains
ordered boundary edges, including fragments created by crossings; it is not an entity list.
`Profile.Holes` contains inner boundary loops. Read these fields with `Profile.Valid` after solving.
The engine owns their semantics in sketch `profiles.go`.

This proves resulting geometry and material removal. It does not prove Fusion's participant-body
selection or a native Cut operation. Use the explicit-cut recipe when the proof requires that operation.

## An explicit bore cut

Run `Example_proofkit3d_explicit_cut` and `TestConstructionExplicitCut` in
[the executable source](proofkit3d_construction_example_test.go).

Construct a 20 by 20 by 8 plate and a radius-2, height-20 tool centered at (14, 6).
Use `r3.Translation` and `Placed` to lower the tool by 6, then call `decad.CutContext`.
The expected volume is `3200 - 32*pi`. The regression passes `RequireSolid`.
This follows decad's `Example_decad_cut` in `examples/decad_boolean_example_test.go`.
The annular-profile recipe is a separate construction alternative, never a fallback for a failed cut.

## Separate cap chamfers

Run `Example_proofkit3d_separate_cap_chamfers` and `TestConstructionSeparateCapChamfers` in
[the executable source](proofkit3d_construction_example_test.go).

Construct two radius-10, height-8 cylinders in separate documents. Chamfer one with
`decad.Edges(decad.CreatedBy(decad.CapStart(body)))` and the other with
`decad.Edges(decad.CreatedBy(decad.CapEnd(body)))`, using setback 0.5.
Verify each document independently. With `R=10`, `H=8`, and `d=0.5`, each volume must match:

```text
pi*R*R*(H-d) + pi*d/3*(R*R + R*(R-d) + (R-d)*(R-d))
```

Each measured volume must also be strictly below the original cylinder volume `800*pi`.
This does not prove one body with both caps chamfered. It does not prove a chamfer applied after
a gear's root fillets.

## A bounded evaluator refusal

Run `Example_proofkit3d_arrangement_limit` and `TestConstructionArrangementLimit` in
[the executable source](proofkit3d_construction_example_test.go).

Construct a 100 by 100 rectangle with 17 disjoint radius-1 holes. Place their centers at the first
17 row-major positions of a five-column grid starting at (10, 10), spaced by 15.
Select the unique valid profile with 17 holes and extrude it by 8.
Construct a radius-0.5 tool centered at (90, 90), extending from z = -1 to z = 9.

Keep both sketches on XY. Use `decad.TwoSided` with `DistanceSide` lengths 9 along and 1 against
the plane for the tool. Translating the tool would move its sketch plane and route this geometry
to the mesh evaluator before the analytic arrangement budget is checked.

The same-plane `decad.CutContext` call must return an error containing `arranger segments` and `cap of`.
Do not match the complete wording, apply a success gate to the failed operation, or increase the limit.
The engine owns the limit as
[`prismMaxArrangementSegments` in `prism_boolean.go`](https://github.com/lestrrat-3d/decad/blob/f200324e2a14ac657cd190b0e90cad54acb2f4c9/prism_boolean.go#L351-L361).
This reference records inspected decad revision `f200324e2a14ac657cd190b0e90cad54acb2f4c9`;
`proof/go.mod` remains the only authority for engine pins. Recheck this recipe if the pin changes.

## Run the recipes

Use `proof/run.sh` so local engine checkouts are verified against `proof/go.mod`:

```sh
bash proof/run.sh --package ./examples -- -vet=all \
  -run '^(TestConstruction|Example_proofkit3d_)' -count=1
```

Run the complete proof suite before handoff, as required by the repository verification procedure.
