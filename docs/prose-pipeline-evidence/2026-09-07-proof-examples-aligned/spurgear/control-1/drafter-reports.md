# Round 1 final report

I wrote the draft in `.worktrees/chore-prose-2026-09-07-proof-examples-aligned-control-1-spurgear`:

- The step list is `.tmp/spurgear.steps.md`.
- The proof files are `.tmp/spurgear-proof/sketches_test.go` and `.tmp/spurgear-proof/solids_test.go`.

The saved prompt hash remains unchanged. Go typechecking passed. I did not generate registrations, stage files, or run operator gates.

The source gaps and limits are:

- `Sketch.project`, `SketchTexts.createInput2`, and `FilletFeatureInput.addConstantRadiusEdgeSet` have unverified API status.
- The required construction axis conflicts with the playbook’s warning about inactive components.
- Exact base/root equality creates zero-length stubs; the source bench excludes that region.
- Final completed-gear chamfer verification remains pending in the referenced helical notes.
- The proof records spline, extrusion and pattern substitutions. Fillet and chamfer remain `[PROSE]`; the bore proof uses unfilleted geometry.

The operator must generate registrations and validate the draft.

# Subsequent read-only construction-axis clarification

The passages establish a conditional API limitation, not a confirmed requirement conflict. My report overstated it.

- `spec/spurgear/instructions.md`, lines 576–579, requires creating `Gear Center` from a cylindrical face.
- `.claude/skills/generate-gear/PLAYBOOK.md`, lines 758–766, warns that axis creation fails when its owning component is inactive.
- The playbook, lines 788–795, prohibits explicitly calling `occurrence.activate()`.
- The spec, lines 484–490, and playbook, lines 83–86, say occurrence creation shifts the active component context.

Those passages do not establish that spur’s component is inactive when it creates the axis. That runtime condition remains unverified. I changed no files and ran no gates.

# Round 2 findings

I verified the saved gate report's supplied hash and kept that report and the original drafting prompt unchanged.
I revised `.tmp/spurgear.steps.md`, `.tmp/spurgear-proof/sketches_test.go`, and `.tmp/spurgear-proof/solids_test.go`.

The draft passed the focused complete spur package check using an untracked Go overlay and the operator-generated
registrations. The command used the recorded environment and was `go test -overlay .tmp/t8/draft-overlay.json
./spurgear -count=1 -timeout=120s`, run from the proof module with absolute paths for the overlay and GOWORK file.
The saved result is `.tmp/t8/draft-r2-proof.log`. I did not change placed artifacts, generate registrations,
stage files, or run the operator gate runner.

- I corrected the sketch engine angle constructor's degrees input; the source and involute math still use radians.
- I corrected boundary counting for the engine's artificial circle seam and coalesced disc boundary.
  The proof derives attachment splits from the actual tooth region's intersection parameters on the same circle.
- I substituted chorded solid tooth boundaries because decad rejects the actual circle fragment's uncertified trim.
  The proof bounds the tooth-area change below 0.1 percent and checks volume against the chorded area.
- I substituted an extrusion of the assembled exterior because the first tooth/disc union reports an invalid arrangement.
  The exterior uses actual tooth-boundary samples and root arcs, with embedded root-contact adjustments bounded
  below 0.01 percent of root radius.
- I extended the proof's bore tool 1 mm beyond each cap because equal-height subtraction exceeds the analytic
  arrangement's segment cap. The resulting mesh subtraction passes the solid gate and removed-volume assertion.
- I reduced repeated document verification in the pattern assertion and bounded chord density while retaining its area check.
- I corrected the submitted axis discussion to describe a conditional API limitation instead of a confirmed conflict.

The three API advisories, excluded exact base/root equality, and pending Fusion chamfer verification remain unchanged.
Root fillets and completed-gear chamfer remain PROSE coverage limits; the passing proof checks unfilleted geometry.
The root-face selector is not exercised by the replacement outline, and decad's chamfer supports lateral edges,
not the required end-cap edges. These limits are recorded in the proof and step list.

The operator must place the revised drafts and run its validation workflow.
