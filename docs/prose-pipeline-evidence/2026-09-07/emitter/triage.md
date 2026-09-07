The direct-emission pilot stopped on a compile-contract fault after its first submitted draft.

The full runner failed the contract and API-call gates. Both required names,
`registerDerivedParameters` and `_drawFlankToRoot`, are absent from the compiled step list.
The first is required by `spec/spurgear/contract.json:79`; the second anchors a source guard
at line 97. The emit prompt does not give the drafter the contract manifest. The step list
describes the parameter registrations and flank construction without imposing those names.
The drafter implemented them under other methods. Repairing this requires the compiler's
output or contract to state the intended interface, rather than guessing during emission.

The Bore Profile guard requires a coincident call directly inside `buildBore`. The candidate
performs it inside `drawBore`, which `buildBore` calls. This is a structural gate refusal,
not evidence that the candidate omitted grounding. The candidate remains failed; nothing
was waived. Contract ownership must decide the intended placement.

All 28 API gate findings report unknown receiver ownership. The methods are present in the
database; the first chain begins with an untyped `cmd` argument. Other chains involve
typing.cast expressions the checker's analysis does not resolve. These are emission/analysis
compatibility failures, not 28 missing Fusion methods. A later retry should add or preserve
verifiable receiver types, but the compile-contract fault stops this run first.

The two novel type complaints concern optional/unknown planes passed to `setByOffset`.
`processInputs` validates that the plane is a ConstructionPlane or BRepFace before use.
The normalized face becomes a ConstructionPlane. The candidate therefore supplies the
expected entity on the checked path, but annotations fail to carry that fact across methods.
I classified these as type-narrowing limitations on the inspected path. I did not add global
accepted-noise entries or claim Fusion runtime verification.

The complete gate JSON and unchanged candidate are retained as `emit/gates-round-1.json`
and `emit-round-1.py`. No Python module was placed and no source inputs were edited.
The timing summary's `complete: true` describes complete timing records, not a successful
compilation. Its `first_pass: false` and gate failures record the actual outcome.
