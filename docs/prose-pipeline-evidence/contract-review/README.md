# Contract rule classification

This directory records the T1 review of every rule enforced from a production `contract.json` at
commit `37e47b33a57b2bc61e98c743ed5b6ab011279a72`. The review date is 2026-09-07.

The original manifests remain authoritative machine-readable mirrors. The prose in each gear's
`instructions.md` and `fusion.md` remains the source owner. This inventory does not replace either
owner and does not change a manifest or `check_contract.py` behavior.

## Classification result

| Gear | Interface | Behavior | Intentional implementation structure | Enforced total | Metadata |
|---|---:|---:|---:|---:|---:|
| `spurgear` | 74 | 0 | 24 | 98 | 6 |
| `helicalgear` | 21 | 0 | 9 | 30 | 4 |
| **Total** | **95** | **0** | **33** | **128** | **10** |

`classification.json` groups exact JSON pointers with one-level `*` selectors. The coverage checker
expands each selector against the named manifest. It rejects a missing, duplicate, or unknown rule,
an empty selector, an unknown classification, undocumented metadata, or a changed manifest hash.

The classifications describe contract intent:

- `interface` covers names, literal values, inheritance, method boundaries, and context fields that
  callers, subclasses, commands, or saved designs reproduce or consume.
- `behavior` would cover an outcome without prescribing source shape. No current manifest entry does
  this directly.
- `intentional_implementation_structure` covers each source guard's selected file, optional function
  scope, required patterns, and banned patterns. These rules preserve a chosen construction recipe.

The source guards protect behavior-sensitive recipes, but the checker enforces their lexical shape and
location. Classifying them as structure avoids claiming that a regular-expression match proves runtime
geometry. Runtime and proof behavior remains owned by the cited prose, benches, and complete gates.

## Enforced rule semantics

`check_contract.py` enforces these manifest entries:

- Every `module_constants` member requires one top-level constant with the exact value.
- Every `classes` member requires one top-level class.
- Every `bases`, `methods`, and `ctx_fields` list item requires the named base, class-body method, or
  initialized `self` field.
- Every source guard `file` must exist. A guard on the manifest's `module` reads the candidate output;
  other guards read the repository file.
- Every `in_function` value requires exactly one Python function with that name and limits all of that
  guard's pattern searches to its source.
- Every `required` pattern must match, and every `banned` pattern must not match, within the selected
  file or function.

The implementation-structure rationales in `classification.json` come from the corresponding guard
reason, its cited specification anchor, the current source, and focused checker tests. All six function
location choices are supported by the source specification or compiled steps. This snapshot has no
unverified source-intent classification.

## Metadata

The inventory records these separately from enforced rules:

- `_comment` documents manifest ownership and scope. The checker does not inspect it.
- `module` routes guards on the generated module to the candidate file. It is control metadata rather
  than an independent assertion about candidate content.
- Each guard's `why` text appears in a failure diagnostic. It does not create a match requirement.

The checker also performs helper-shadowing, relative-import resolution, and star-import checks without
reading manifest rules. Those manifest-free gates are outside this rule inventory.

## Audit

Run the coverage check from the repository root:

```sh
python3 docs/prose-pipeline-evidence/contract-review/check_classification.py
```

The expected final line for this snapshot is:

```text
total: 128 enforced rules, 10 metadata entries, behavior=0, intentional_implementation_structure=33, interface=95
```
