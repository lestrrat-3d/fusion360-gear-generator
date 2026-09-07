# T2. Render citations from metadata

## Assignment

The implementer removes citation-format choices from prose compilation in `fusion360-gear-generator`.
T0 must be complete. The [common execution contract](README.md#common-execution-contract) applies.
[Step metadata formats](formats.md) fixes the new syntax and interfaces.

## Allowed edits

The implementer may add `step_metadata.py`, `render_step_metadata.py`, `test_step_metadata.py`,
and `test_render_step_metadata.py`. Existing edits are limited to `check_compile.py`, `scaffold_proof.py`,
their tests, `test_render_prompt.py`, and the compile `prompt.md` and `SKILL.md` files.
`gen_provenance.py` is not the owner of citations and does not gain citation logic.

The implementer reads `check_compile.steps_of`, `from_block`, `citations_from_block`,
`validate_citations`, and the scaffolder's imports before editing.

## Ordered edits

1. The implementer moves the current `steps_of` implementation into `step_metadata.py` without changing its output.
   `PATH_REF` moves with it as the shared path-syntax constant. `check_compile` re-exports both imported
   names so existing consumers continue to work. The new module does not import `check_compile`.
2. The new module implements version 1 from the format owner. Version 2 is rejected until T3 lands.
3. `render_step_metadata.py <gear> --write <path>` reads the complete draft, validates all metadata, then
   replaces each `**From:**` block with the canonical rendered line. It creates a missing From block immediately
   after the metadata comment. It refuses repeated From blocks instead of guessing which one to keep.
4. The command leaves all unrelated text unchanged. It writes only after every step validates.
   A temporary sibling file and `os.replace` provide atomic replacement; the temporary file is removed on error.
5. Usage and I/O errors exit 2. Metadata or citation content errors exit 1. Success exits 0.
   A failed run prints deterministic step-specific diagnostics and leaves the original bytes unchanged.
6. The compile checker retains existing legacy citation checks. In metadata mode it additionally compares
   the existing From block with the canonical line and rejects drift.
7. The compile prompt requires version 1 and leaves From rendering to the tool. The compile skill runs the
   new renderer after each draft and before provenance stamping, scaffolding, or staging.
8. The scaffolder continues to consume the rendered steps and unchanged proof annotations.

The CLI requires the gear's source specification to exist. A legacy file supplied to the new renderer
is rejected with a content diagnostic requesting metadata; the checker still accepts valid legacy files.
No production step file is automatically migrated in this change.

## Fixture matrix

Each fixture contains a three-line `spec/fixturegear/instructions.md` and a single `1` PROSE step unless stated.

| Test name | Input | Expected result |
|---|---|---|
| `test_single_line` | `first=2,last=2` | The line is exactly `**From:** ` followed by the quoted path and ` L2.`. |
| `test_range` | `first=1,last=3` | The canonical suffix is ` L1–3.`. |
| `test_out_of_bounds` | `last=4` | Exit 1; the diagnostic names the step and file; bytes are unchanged. |
| `test_reversed_range` | `first=3,last=2` | Exit 1. |
| `test_boolean_line` | `first=true` | Exit 1. |
| `test_unknown_schema` | File marker `99` | Exit 1; no legacy fallback. |
| `test_duplicate_json_key` | The payload repeats `first`. | Exit 1. |
| `test_duplicate_step_payload` | Two metadata comments appear in one step. | Exit 1. |
| `test_from_drift` | Markdown says L1; metadata says L2. | Checker failure; renderer corrects it. |
| `test_second_step_invalid_is_atomic` | The first step is valid and the second is invalid. | Neither step is written. |
| `test_legacy_checker_compatibility` | A valid existing citation style has no marker. | Existing checker result is unchanged. |
| `test_repeated_render` | A valid draft is rendered twice. | Output bytes are identical. |
| `test_proof_annotations_preserved` | The step carries a proof-run annotation. | Scaffolder registrations remain identical. |

## Validation and completion

```sh
python3 -m unittest discover -s "$P" -p 'test_step_metadata.py' > .tmp/t2-metadata-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_render_step_metadata.py' > .tmp/t2-render-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_scaffold_proof.py' > .tmp/t2-scaffold-tests.txt 2>&1
```

The [common validation](README.md#common-validation) also applies.
A need to change step-heading syntax, proof annotations, or source-file content is an escalation.
Completion requires atomic rendering, unchanged legacy parsing, and every fixture row passing.
