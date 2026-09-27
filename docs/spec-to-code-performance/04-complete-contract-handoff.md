# Carry the complete contract into gear emission

## Goal

Give the Python emitter every requirement that `check_contract.py` will enforce, before it
drafts `lib/geargen/<gear>.py`. Reduce rejected drafts without changing the spec as the source
of truth or weakening the final gates.

## Current path

As of merged PR #171, `exact_values.py` transfers selected `INPUT_ID_` and `PARAM_` constants,
dialog inputs, and user parameters. It does not transfer all `contract.json` class requirements
or method-scoped `source_guards`. The emitter reads checked steps, proofs, a playbook extract,
and framework files; its standard prompt does not list `contract.json` as an input. The spur
manifest requires operations inside `_drawFlankToRoot` and `buildBore` that a geometry-only
description may not communicate. A September 2026 failed emit trial reported three contract
findings; it did not measure accepted-output savings from fixing this handoff.

The local `feat-prose-contract-handoff` branch has a candidate implementation and fixture
matrix. Audit it against current `exact_values.py`, contract validation, and provenance before
writing another format.

## Design

- Keep `spec/<gear>/contract.json` authoritative. Render its complete parsed manifest into a
  versioned `## Compilation contract` section in `steps.md`; do not ask a model to summarize it.
- Carry `module`, `module_constants`, `classes`, and `source_guards`, including each guard's
  `in_function`, `required`, and `banned` fields. Preserve field values and list order.
- Generate the section during compile placement, after provenance and before timeline steps.
  Validate source digest and semantic equality before any emit draft begins.
- Reject missing, repeated, malformed, or stale sections. Keep the source and placed steps
  unchanged when rendering fails. A gear without a manifest must not gain an invented one.
- Mask the generated JSON from call-span scanning while preserving line positions. A guard
  description containing a call-shaped string must not create a step-call requirement.
- Tell the emitter to read the generated contract section as part of its checked steps. Keep
  the original manifest and prose outside its drafting input; the complete emit contract gate
  remains authoritative.
- Define how the full contract section and `## Exact values` share constants. Keep one source
  value for each constant and reject disagreement rather than accepting whichever section wins.

## Tasks

- [x] Compare current `contract.json`, `check_contract.py`, provenance, and `exact_values.py`
      with the candidate on `feat-prose-contract-handoff`. List every manifest field consumed
      by the validator and every field absent from the emitter's checked input.
- [x] Specify the versioned section and no-manifest behavior. Validate types, duplicate keys,
      guard regexes, idempotent rendering, and unchanged bytes outside the section.
- [x] Add focused tests for a missing method, moved guard operation, changed manifest, stale
      section, failed atomic render, and call-shaped text inside contract JSON.
- [x] Make compile provenance and `check_compile.py` reject incomplete or stale handoffs.
      Update the standard compile and emit prompts only after this behavior is fixed.
- [x] Run one real spur spec → steps/proof → full compile gates → fresh emitter → full emit
      gates flow. Compare its output with the checked-in final Python and run it in Fusion.
      Stop expansion on the first failed boundary; report the blocker, repair scope, and cost.
- [x] After the first real pass, compare a pinned baseline and candidate with identical source
      digests, model, and gate policy. Record rejected drafts, repair rounds, accepted-output
      time, prompt size, and Fusion result. Do not infer a speedup from the historical failure.
- [x] Extend to helical only after spur passes; add gears without manifests only when their
      contract source and validation policy are defined.

## Completion

The fresh emitter receives every applicable manifest rule before drafting. The exact placed
steps pass compile checks, the final Python passes every emit gate, and Fusion accepts the
generated spur gear. Report accepted-output time and any added input size.

## Implementation and trial

The v1 `## Compilation contract` section contains the parsed `contract.json` object under a
`contract` key. It carries `module`, every `module_constants` entry, each class's `bases`,
`methods`, and `ctx_fields`, and every source guard's `file`, `in_function`, `why`, `required`,
and `banned` fields. The renderer also carries the optional `_comment` field. Before this change,
`## Exact values` carried only `INPUT_ID_` and `PARAM_` constants plus dialog and parameter
setup. Both generated sections are checked against `contract.json`, so conflicting copies of a
constant fail the compile check. A gear without `contract.json` gets no contract section.

The spur comparison used `origin/main` at `c2766f3` as the baseline source and the same proof,
playbook extract, framework, model (`gpt-6-astra`), and full emit gates for both drafts. The
baseline emitter read the real pre-handoff steps from that commit. The candidate emitter read
the steps with the generated contract section. No prose spec or existing implementation was an
emitter input. The checked source specs, proof, playbook, and framework files had no diff from
`origin/main` during the comparison.

| Spur trial | Pre-handoff steps | Complete-contract steps |
|---|---:|---:|
| Step-list bytes | 59,079 | 65,795 |
| Emitter prompt bytes | 5,789 | 6,203 |
| First draft contract findings | 2 | 0 |
| Rejected drafts before all gates passed | 2 | 2 |
| Repair rounds | 2 | 2 |
| Prompt-file creation to accepted gate report | 424 s | 433 s |
| Fusion result | Not tested | Default spur generated successfully |

The elapsed times include agent startup, review, and gate execution. The baseline draft also
overlapped helical work, so this pair does not establish a drafting-time difference. The added
contract section cost 6,716 bytes; the updated prompt cost another 414 bytes. The trial removed
first-draft contract failures but did not reduce rejected drafts or accepted-output time.

Helical steps gained the same v1 handoff. The full helical compile gate and the first fresh emit
draft passed, and a default helical gear generated successfully in Fusion. Fixture tests cover
missing manifests; gears without a defined contract source were not regenerated.
