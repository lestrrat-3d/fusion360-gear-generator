# T6. Supply typed emitter entry points

## Assignment

The implementer reduces unknown receiver types in fresh Python emission for `fusion360-gear-generator`.
T1 and T5 must be complete. The [common execution contract](README.md#common-execution-contract) applies.
This submission provides typed guidance; it does not expand the AST resolver's trust in casts.

## Allowed edits and entry points

The implementer may add `render_emitter_interfaces.py` and `test_render_emitter_interfaces.py`.
Allowed existing edits are `test_check_api_calls.py`, `test_render_prompt.py`, and the emit prompt and skill.
Read-only inputs are `lib/geargen/base.py`, `commands/_gear_command.py`, and the per-gear contract manifest.
The implementer reads `Generator.generate`, `GearCommand.command_created`, `GearCommand.command_execute`,
`check_api_calls.infer_api_receiver_types`, and `fusion_annotation_type` before editing.
Generated modules and `check_api_calls.py` are read-only in this task.

## Fixed output

`render_emitter_interfaces.py <gear> --out <path>` writes a Markdown interface sheet.
The sheet contains only signatures and typing conventions, with no executable placeholder implementations.
The tool uses AST parsing; it never imports `adsk`, the command module, or a generated gear module.

For each manifest class with method `configure`, the sheet emits this canonical classmethod signature,
substituting the class name and preserving any bases declared by its manifest:

```python
class FixtureConfigurator:
    @classmethod
    def configure(cls, cmd: adsk.core.Command) -> None: ...
```

For each manifest class that names base `Generator` and method `generate`, it emits the signature
derived from `Generator.generate`, preserving its parameter names and annotations.
The derived return annotation remains absent when the owner has none; the tool does not invent one.
It renders the class header with that declared base and lists other required methods by name only.
The tool does not guess parameter or field types for those other methods.

The current owner does not declare a typed configure protocol. Therefore the configure template is backed
by an AST invariant: `GearCommand.command_created` has an `adsk.core.CommandCreatedEventArgs` parameter
and calls `self.configurator.configure(args.command)` using that parameter name.
The template's `adsk.core.Command` type is an explicit interface decision in this handoff.
An owner change that breaks that invariant makes the renderer exit 2 for design review.

## Typing conventions

The generated sheet includes these fixed instructions:

1. The emitter keeps entry-point parameter annotations from the sheet.
2. A local API object receives an explicit qualified annotation when its type cannot be inferred from a typed receiver.
3. Optional objects are checked before use. A null check remains runtime behavior, not a type-only cast.
4. `typing.cast` is not used solely to make an API receiver recognizable to the custom checker.
   A typed parameter or checked local binding supplies that information instead.
5. Calls on refuted or unverified API pairs still follow the shared API policy.

The sheet is advisory generation scaffolding. It does not authorize overriding a conflicting source contract.

## Ordered edits

1. The implementer adds the AST reader, stable signature renderer, and invariant tests.
2. The CLI requires readable framework sources and a valid contract when a manifest exists.
   A gear without a manifest receives the generic conventions and the base Generator signature only.
3. Success exits 0; bad inputs or an incompatible framework shape exit 2. Failed rendering leaves output unchanged.
4. The emit skill generates `.tmp/<gear>.emitter-interfaces.md` before drafting.
5. The emit prompt adds that sheet to its allowed inputs and explains that final code must implement the signatures.
6. Tests feed typed fixture candidates into the unchanged API checker to establish that the proposed conventions work.

## Fixture matrix

| Test name | Fixture | Expected result |
|---|---|---|
| `test_configure_signature` | An invented configurator manifest declares configure. | The sheet contains the typed classmethod above. |
| `test_generate_signature_follows_base` | A fake base changes a parameter name. | The rendered signature follows the owner exactly. |
| `test_configure_callsite_drift` | The command passes a different object. | Renderer exit 2. |
| `test_no_adsk_import_execution` | Importing fake framework files would raise. | AST rendering still succeeds. |
| `test_typed_command_chain` | `cmd: adsk.core.Command` accesses commandInputs. | Mocked API ownership resolves correctly. |
| `test_typed_local_binding` | A qualified API annotation names a local object. | The existing checker resolves the local receiver. |
| `test_wrong_member_still_fails` | A typed receiver invokes a nonexistent member. | API checker failure. |
| `test_untyped_cast_still_unresolved` | An unknown source is wrapped in typing.cast. | The existing unresolved-receiver failure remains. |
| `test_refuted_call_still_fails` | A typed receiver uses a refuted pair. | T5 still blocks it. |
| `test_idempotent_sheet` | The same inputs are rendered twice. | Identical output. |

## Validation and completion

```sh
python3 -m unittest discover -s "$P" -p 'test_render_emitter_interfaces.py' > .tmp/t6-sheet-tests.txt 2>&1
python3 -m unittest discover -s "$P" -p 'test_check_api_calls.py' > .tmp/t6-type-tests.txt 2>&1
```

The [common validation](README.md#common-validation) also applies.
If the existing resolver cannot accept the typed-local fixture, the implementer reports that fixture;
the task does not expand into an inference-engine rewrite. Contradictory source signatures also require escalation.
Completion requires generated sheets, unchanged receiver rejection, and all fixture rows passing.
