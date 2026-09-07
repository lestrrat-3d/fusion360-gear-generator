# Step metadata formats

## Ownership

This document fixes the proposed wire formats for T2 and T3. The implementation owner will be
`step_metadata.py`; once implemented, its parser and renderer define accepted syntax.
No other parser may maintain a separate copy of these schema rules.
The examples below are invented fixtures, not instructions for a real gear.

## File mode

A file without a metadata marker uses the existing legacy parsers without new requirements.
A new file contains exactly one marker before its first step:

```text
<!-- step-metadata: 1 -->
```

Version 2 replaces the integer with `2`. Unknown versions, repeated markers, malformed markers, and
markers appearing inside a step are content errors. A malformed marker cannot select legacy mode.
The existing `check_compile.steps_of` heading syntax remains unchanged.

Every metadata-mode step contains exactly one JSON comment before `**From:**`.
Its opening line is exactly `<!-- step-meta`, and its closing line is exactly `-->`.
The JSON decoder rejects duplicate keys. Unknown keys, booleans used as integers, and non-finite numbers
are content errors. String fields cannot contain a newline or the comment terminator `-->`.

## Version 1

The step payload has exactly these keys:

```json
{
  "schema": 1,
  "citations": [
    {"path": "spec/fixturegear/instructions.md", "first": 2, "last": 4}
  ]
}
```

`citations` is nonempty. Each path is a normalized repository-relative POSIX path using the character
set already accepted by `check_compile.PATH_REF`. Absolute paths and `..` components are rejected.
The resolved path must remain inside the worktree. `first` and `last` are integers satisfying
`1 <= first <= last <= number of source lines`. Line counts use UTF-8 `splitlines()`.
An unreadable source is a setup error; an absent or invalid cited range is a content error.

The renderer retains declaration order and produces this exact line:

```text
**From:** `spec/fixturegear/instructions.md` L2–4.
```

Single-line ranges render as `L2`. Multiple entries are separated by `; ` before the final period.
The metadata comment uses `json.dumps(payload, sort_keys=True, ensure_ascii=False, indent=2)`.
Repeated identical citation entries are rejected rather than silently removed.

## Version 2

Version 2 retains version 1 citation rules and adds the required `calls` array:

```json
{
  "schema": 2,
  "citations": [
    {"path": "spec/fixturegear/instructions.md", "first": 2, "last": 4}
  ],
  "calls": [
    {
      "span": "tools.addWidget(item)",
      "name": "addWidget",
      "receiver": "tools",
      "owner": "adsk.fusion.WidgetTools",
      "role": "required",
      "condition": null,
      "reason": null
    }
  ]
}
```

Every call entry has exactly those seven keys. `span` is the exact content of an inline code span in
that step's title or body, excluding metadata comments and fenced blocks. `name` and `receiver` must
match a pair returned by `call_parser.call_shapes(span)`. `receiver` is null for a bare function call.
`owner` is a fully qualified `adsk.core.*` or `adsk.fusion.*` class for an API call, and null for a local call.
The invented owner above is accepted by syntax parsing; API validation requires a fixture database entry.

| Role | Required fields | Validator behavior |
|---|---|---|
| `required` | `reason` is null; `condition` is null or nonempty text | The API and reachable-call gates check it. |
| `inherited` | `owner` is null; `condition` is null; `reason` is nonempty | A shared framework definition must back the name. |
| `example` | `condition` is null; `reason` is nonempty | The declaration does not require execution. |
| `forbidden` | `condition` is null; `reason` is nonempty | Existing source guards remain responsible for prohibitions. |
| `prose` | `owner` and `condition` are null; `reason` is nonempty | Non-code wording does not require execution. |

`prose` additionally requires that the entire span fail `ast.parse(span, mode='eval')`.
A syntactically valid function call cannot be hidden as ordinary prose by this role.
`example` and `forbidden` classify intent; syntax validation does not prove that classification correct.
Changes to those roles for an existing requirement require source review, not automatic migration.

Every call-shaped pair in every inline span has exactly one declaration keyed by `(span, name, receiver)`.
Every declaration must point to a span that exists. Repeated identical spans share one declaration;
conflicting roles for the same key are rejected. Fenced blocks remain non-requirements, as in legacy parsing.
`calls: []` is valid only when the step has no call-shaped inline spans.

Metadata-mode preambles may contain the generated contract fence and ordinary non-call text.
Call-shaped inline spans outside steps are rejected with an instruction to place them in the relevant step.
This prevents moving a required call into a preamble to evade declaration checks.

Existing global ignore directives are rejected in version 2. The corresponding explicit role replaces them.
Version 1 and legacy files retain their existing call-checking behavior until regenerated in version 2.

## Shared API

T2 introduces these functions in `step_metadata.py`:

```python
class MetadataError(ValueError):
    pass

def file_version(text: str) -> int | None: ...
def parse_step(body: str, version: int) -> dict: ...
def validate_citations(payload: dict, root: str) -> list[str]: ...
def render_from(payload: dict) -> str: ...
def render_steps(text: str, root: str) -> str: ...
```

T3 adds `declared_calls(body: str, payload: dict) -> list[dict]` and version 2 support.
Schema errors raise `MetadataError`. Source I/O failures remain `OSError` for callers to classify as setup errors.
Returned diagnostics are deterministic and include the step ID when rendered by a whole-file caller.
The metadata module must not import `check_compile.py`; T2 extracts shared step splitting without a circular import.
