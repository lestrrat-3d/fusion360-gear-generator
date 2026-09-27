# Proof construction examples

These examples use the public sketch and decad APIs at the revisions in
[`proof/go.mod`](../go.mod). Run them through `proof/run.sh`; it checks both engine
revisions before testing. They show proof constructions, not Fusion behavior.
The generated gear proof still needs its own assertions and the complete
`proof/run.sh` gate.

| Operation or boundary | Tested source | What the example establishes |
|---|---|---|
| Analytic cut segment limit | [`arrangement_limit_test.go`](arrangement_limit_test.go) | A cut passes with 14 holes and fails with 17 at the pinned 4,096 segment cap. |
| Bore through a solid | [`bore_profile_example_test.go`](bore_profile_example_test.go) | A solved annulus extrudes to a sound body with the expected hole, area, and volume. |
| Independent comparison bodies | [`comparison_bodies_example_test.go`](comparison_bodies_example_test.go) | Equal blocks pass soundness and volume checks in separate documents. |
| Profile boundaries | [`TestBoreProfile`](bore_profile_example_test.go) | The annulus has one source circle, one outer edge, and one hole loop. |
| One-cap chamfer | [`cap_chamfer_example_test.go`](cap_chamfer_example_test.go) | Each cap is chamfered and volume-checked on its own cylinder. |

## Analytic cut limit

The paired tests keep the plate and tool sketches on the same plane. A plate
with 14 circular holes accepts a further cut; the result is sound and has the
expected removed volume. The same cut on a plate with 17 holes returns the
arrangement-budget refusal. The engine owns the limit; this document records
the behavior at the revisions in `proof/go.mod`.

## Bore substitute

When a gear's explicit bore cut exceeds decad's analytic segment limit, draw
the outer boundary and bore circle in one sketch. Solve the sketch, select the
valid profile with exactly one hole, then extrude it. Check its area, hole count,
solid verdict, and independently calculated volume. `TestBoreProfile` also
checks that the volume assertion rejects a different bore radius.

This substitutes the final geometry for the cut operation. It does not prove
Fusion's cut feature, its participant-body selection, or its timeline order.
The gear proof must state that limit beside the substituted construction and
retain the Fusion cut in its step list when the spec requires it.

## Independent comparison bodies

Build comparison solids in separate decad documents. Verify each document and
measure each solid against its own expected volume. The example rejects a
shared-document pair and an incorrect volume. Separate documents prevent the
comparison solids from interfering with one another during verification.

## Profile boundaries

`Profile.Entities` lists distinct source entities on the outer boundary.
`Profile.Outer` lists ordered boundary edges, including fragments when a curve
is split. `Profile.Holes` lists inner loops. The bore example checks all three
views on a concentric-circle annulus. Select by validity and hole count before
extrusion; the first profile in a sketch is not always the desired region.

## Cap chamfer

Build two cylinders in separate documents. Select the edge created by
`CapStart` on one and `CapEnd` on the other. Chamfer each by 0.5 mm and verify
its solid verdict and the independently calculated volume. The test rejects a
different setback. This demonstrates one cap per body. It does not prove a
two-cap chamfer or a chamfer after a gear's tooth fillets.
