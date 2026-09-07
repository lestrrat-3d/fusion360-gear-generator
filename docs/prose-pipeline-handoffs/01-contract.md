# T1. Generate a complete contract handoff

## Assignment

The compiler and emitter in `fusion360-gear-generator` must see every manifest rule that checks their output.
T0 must be complete. The [common execution contract](README.md#common-execution-contract) applies.
This task preserves every current manifest requirement, including helper-location guards.

## Allowed edits and entry points

The implementer may add `contract_handoff.py` and `test_contract_handoff.py` under the shared tool directory.
Allowed existing edits are `gen_provenance.py`, `provenance.py`, `check_compile.py`, `check_step_calls.py`, their tests,
`test_check_contract.py`, `test_render_prompt.py`, and the compile and emit `prompt.md` and `SKILL.md` files.
The implementer reads `check_contract.main`, `guard_problems`, `function_source`,
`gen_provenance.write_section`, and `provenance.provenance_inputs` before editing.
Production `contract.json` files and `check_contract.py` behavior are read-only.

## Fixed design

The handoff contains the entire parsed manifest, not a model-written summary.
The new module owns these interfaces:

```python
class ContractHandoffError(ValueError):
    pass

def load_contract(root: str, gear: str) -> dict | None: ...
def render_contract(manifest: dict) -> str: ...
def replace_contract(text: str, rendered: str) -> str: ...
def validate_contract(text: str, root: str, gear: str) -> list[str]: ...
def mask_contract(text: str) -> str: ...
```

The renderer emits `## Compilation contract`, a blank line, and the manifest as sorted, indented UTF-8 JSON
inside a backtick fence. The fence length is one longer than the longest backtick run in the JSON,
with a minimum of three. Its opening fence has the `json` language label.
The section belongs after `## Provenance` and before the first timeline step.
Insertion and replacement leave text outside this section byte-for-byte unchanged.
Both call checkers use `mask_contract` before scanning inline spans. It replaces non-newline characters
in the complete generated section with spaces, preserving diagnostic line positions. JSON descriptions
and guard patterns must not become additional API-call requirements, even when they contain backticks.

`load_contract` permits the existing top-level keys `_comment`, `module`, `module_constants`, `classes`,
and `source_guards`. Class objects permit `bases`, `methods`, and `ctx_fields`. Guard objects permit
`file`, `in_function`, `why`, `required`, and `banned`. Unknown keys and wrong field types raise
`ContractHandoffError`. All original values and list order are preserved.
Empty optional sections are valid. An existing unreadable file raises `OSError`; an absent manifest returns `None`.
Guard regexes are validated with `re.compile`. Duplicate JSON keys are rejected.

The manifest remains owned by the prose contract as described in `check_contract.py`.
This change does not infer or relax a guard because its location seems unnecessary.

## Ordered edits

1. The implementer adds the pure parser, renderer, replacer, and comparison functions with fixture tests.
2. `provenance_inputs` includes the existing per-gear contract file in its required input set.
3. `write_section` computes the provenance and contract replacements completely before writing either.
   Invalid manifest data leaves the target file untouched. The print-only provenance command remains unchanged.
4. `check_compile.check` requires semantic equality between the embedded manifest and the current manifest.
   Missing, repeated, malformed, or stale contract sections are content failures; unreadable inputs are setup errors.
5. The compiler prompt explicitly reads the manifest. The emitter prompt explicitly reads the generated section.
   The compile skill documents that the existing provenance command also generates this section.
6. The implementer adds a no-manifest path that neither requires nor invents a contract section.
   A leftover contract section after manifest removal is a content error.

Existing steps with a contract become stale and require normal recompilation. The implementer does not
silently restamp their hashes or edit generated steps in this tooling change.

## Fixture matrix

The invented manifest declares `FixtureGenerator.buildFixture` and a `buildFixture` guard requiring `anchorPoint`.

| Test name | Fixture change | Expected result |
|---|---|---|
| `test_complete_handoff` | The renderer receives that manifest. | The parsed section equals every original field. |
| `test_missing_method_in_handoff` | The embedded method is removed. | Compile content failure. |
| `test_guard_location_preserved` | The guard is scoped to `buildFixture`. | The section keeps that scope exactly. |
| `test_guard_moved_to_helper_still_fails` | Candidate grounding moves to another helper. | Existing contract gate still fails. |
| `test_manifest_change_invalidates_steps` | A method is added after stamping. | Provenance and handoff drift are detected. |
| `test_unknown_manifest_field` | The source adds `future_rule`. | Generator exits 2 without writing. |
| `test_no_manifest` | The fixture has no contract file. | No new section or provenance row is required. |
| `test_duplicate_section` | Two contract sections exist. | Content failure. |
| `test_atomic_render_failure` | A guard regex is invalid. | Existing draft bytes remain unchanged. |
| `test_idempotent_stamp` | The generator runs twice. | The second output is identical. |
| `test_contract_text_not_scanned_as_calls` | A guard reason contains nested backticks and example calls. | Both call scanners ignore the section. |

## Validation and escalation

```sh
python3 -m unittest discover -s "$P" -p 'test_contract_handoff.py' > .tmp/t1-handoff-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_gen_provenance.py' > .tmp/t1-provenance-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_check_contract.py' > .tmp/t1-contract-tests.txt 2>&1
```

The [common validation](README.md#common-validation) follows these commands.
A current manifest field outside the enumerated schema or a manifest/spec contradiction is an escalation.
Completion requires every fixture row, unchanged production manifests, and explicit regeneration requirements in the record.
