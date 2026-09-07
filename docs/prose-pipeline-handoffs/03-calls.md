# T3. Declare required calls explicitly

## Assignment

The implementer gives the prose compiler and emitted-code checker one call-declaration format.
T1, T2, and T5 must be complete. The [common execution contract](README.md#common-execution-contract) applies.
[Step metadata formats](formats.md#version-2) owns every payload field and role.

## Allowed edits and entry points

Allowed edits are `step_metadata.py`, `render_step_metadata.py`, `call_parser.py`, `check_compile.py`,
`check_step_calls.py`, their tests, `test_render_prompt.py`, and both drafting prompts and skills.
The implementer reads both current `named_call_shapes` functions and
`check_step_calls.ReachableCallCollector`, `actual_call_shapes`, and `main` before editing.
The reachable-code collector and source-guard semantics remain unchanged.

## Ordered edits

1. `step_metadata.py` implements version 2 parsing and `declared_calls` exactly as the format owner specifies.
   The implementation strips metadata comments before scanning spans, so declarations cannot satisfy themselves.
2. Whole-file validation rejects undeclared spans, declarations without matching spans, invalid role fields,
   legacy ignore directives in version 2, and call-shaped spans in the preamble.
3. The metadata renderer accepts versions 1 and 2 and preserves calls while rendering citations.
4. For version 2, `check_compile` derives API candidates only from `required` declarations.
   An API declaration requires a qualified owner and uses T5's typed resolver.
5. A required declaration with `owner=null` must name an existing Python/math or shared-framework name
   from the current `PYTHON_METHODS`, `defined_names(FRAMEWORK)`, or `contract_names` sets.
   A dotted non-API local call retains its receiver requirement for reachable-call checking.
6. An `inherited` declaration must name an actual shared-framework definition from `defined_names(FRAMEWORK)`.
   A contract-only method name is not enough to qualify as inherited behavior.
7. `check_step_calls` converts each required declaration to the existing `(name, has_receiver)` requirement.
   Conditional requirements still require a reachable executable call; this task does not prove branch coverage.
8. Both checkers keep legacy and version 1 behavior unchanged. Their global ignore directives are not reinterpreted.
9. The compile prompt switches new drafts to version 2 and gives the exact role rules through the format owner.
   The emit prompt treats required declarations as its execution checklist and preserves their stated conditions.

`example`, `forbidden`, and `prose` entries create no positive execution requirement.
`forbidden` does not replace existing source guards. A newly required prohibition needs its normal source/contract change.
Metadata does not prove that the drafter captured every prose requirement; the proof and source review still do that.

## Migration rule

The implementer does not convert production step files by guessing roles for their existing spans.
New drafts use version 2; legacy files keep legacy checks until normal recompilation replaces them.
Changing an existing required call to example, forbidden, prose, or inherited status is a source-review escalation.
The implementer cannot remove a call requirement merely because the current module lacks that call.

## Fixture matrix

Fixture database responses use the invented `adsk.fusion.WidgetTools.addWidget` member.
The candidate fixture defines a class method `generate` so the existing reachable-call collector visits it.

| Test name | Fixture | Expected result |
|---|---|---|
| `test_required_call_present` | A declaration and reachable `tools.addWidget(item)` agree. | Both checks pass. |
| `test_required_call_missing` | The candidate omits that call. | The step-call checker fails with `addWidget`. |
| `test_comment_is_not_execution` | Only a comment contains the required call. | The step-call checker fails. |
| `test_wrong_owner` | Metadata names an unrelated owner. | API validation fails despite the member name existing elsewhere. |
| `test_prose_parenthesis` | Span `dimensionless (units '')` has role prose and a reason. | Neither checker requires execution. |
| `test_real_call_cannot_be_prose` | Span `tools.addWidget(item)` has role prose. | Metadata validation fails. |
| `test_example_has_reason` | A call is marked example without a reason. | Metadata validation fails. |
| `test_missing_declaration` | A second call-shaped span lacks a declaration. | Metadata validation fails. |
| `test_declaration_without_span` | A declaration's span was deleted from the step. | Metadata validation fails. |
| `test_declaration_not_self_evidence` | The only matching text is in the metadata comment. | Metadata validation fails. |
| `test_conflicting_duplicate` | One key has required and example roles. | Metadata validation fails. |
| `test_inherited_name_must_exist` | An invented helper claims inherited status. | Compile validation fails. |
| `test_legacy_ignore_unchanged` | Existing versionless fixture uses its old directive. | Existing result is unchanged. |
| `test_v2_global_ignore_rejected` | A version 2 fixture adds an old ignore directive. | Metadata validation fails. |
| `test_preamble_call_rejected` | A required span moves above the first step. | Metadata validation fails. |

## Validation and completion

```sh
python3 -m unittest discover -s "$P" -p 'test_step_metadata.py' > .tmp/t3-metadata-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_check_step_calls.py' > .tmp/t3-execution-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_check_compile.py' > .tmp/t3-compile-tests.txt 2>&1
```

The [common validation](README.md#common-validation) also applies.
Completion requires every fixture row and unchanged reachable-code and legacy behavior.
A role conflict that needs interpretation of a production spec is an escalation, not an automatic exemption.
