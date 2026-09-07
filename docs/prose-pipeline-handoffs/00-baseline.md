# T0. Preserve the measured baseline

## Assignment

The implementer preserves the failed prose-to-code study as durable evidence in `fusion360-gear-generator`.
This task copies and summarizes existing observations. It does not launch generation or estimate missing usage.
The [common execution contract](README.md#common-execution-contract) applies.

## Inputs and allowed edits

The dispatcher supplies the project root containing the two study worktrees named in the backlog.
The implementer may add only `docs/prose-pipeline-evidence/2026-09-07/` and files beneath it.
Scratch output belongs in `.tmp/t0/`. Source study files are read-only.

The compiler input directory is `.worktrees/chore-prose-compile-study/.tmp/prose-study/`.
The emitter input directory is `.worktrees/chore-prose-emit-study/.tmp/prose-study/`.
These paths are relative to the supplied project root, not the task worktree.

## Fixed output layout

| Destination | Source |
|---|---|
| `compiler/summary.json` | Compiler `compile-summary.json` |
| `compiler/gates-round-N.json` for N = 1, 2, 3 | Compiler `compile/gates-round-N.json` |
| `compiler/run.json` and `compiler/events/` | Compiler timing run metadata and event JSON files |
| `compiler/context-manifest.json` | Compiler `context-manifest.json` |
| `emitter/summary.json` | Emitter `emit-summary.json` |
| `emitter/gates-round-1.json` | Emitter `emit/gates-round-1.json` |
| `emitter/run.json` and `emitter/events/` | Emitter timing run metadata and event JSON files |
| `emitter/triage.md` | Emitter `triage.md` |
| `study.md` | Compiler `report.md`, with local evidence references rewritten for this layout |
| `manifest.json` | A generated inventory of the copied files |

`pipeline_timing.RUN_FILE` owns the timing metadata filename. If it differs from `run.json` at this
baseline, the implementer maps that source to the destination above; the source JSON bytes remain unchanged.
The implementer includes all event JSON files and no build caches, binaries, or generated gear source files.

## Mechanical procedure

1. The implementer checks that both summaries and all four gate reports are present and parse as JSON.
2. The implementer creates the destination tree and copies the allowlisted evidence byte for byte.
3. The implementer rewrites only relative evidence and skill links in the copied `study.md`.
4. The implementer generates `manifest.json` using the schema below, sorting files by destination path.
5. The implementer verifies destination hashes against the source files and the manifest.

```json
{
  "schema": 1,
  "observed_date": "2026-09-07",
  "timezone": "Asia/Tokyo",
  "base_commit": "f38c26da97f27dd9ec962a65646d19c3cf106cf0",
  "serial_pipeline": false,
  "accepted_outputs": 0,
  "files": [
    {"path": "compiler/summary.json", "sha256": "<computed digest>", "bytes": 0}
  ]
}
```

The digest and byte count in the example are placeholders for computed values. The manifest inventories
every destination file except itself. The rewritten study is hashed after its link edits.
`study.md` explicitly states that timing-record completeness does not mean accepted generated output.

## Required verification

| Check | Expected result |
|---|---|
| Compiler summary `completed_rounds` | `3` |
| Compiler summary `drafting_time_s`, rounded to two decimals | `1582.98` |
| Emitter summary `drafting_time_s`, rounded to two decimals | `401.87` |
| Both summaries `first_pass` | `false` |
| Each copied gate `verdict` | `fail` |
| File hashes | Every computed SHA-256 and byte count matches the manifest. |
| Scope | No file outside the destination tree is edited. |

The implementer runs a Python verification command saved as `.tmp/t0/verify.py`, then runs:

```sh
python3 .tmp/t0/verify.py > .tmp/t0/verification.txt 2>&1
```

The verification script asserts every row above and exits nonzero on a mismatch.
The [common validation](README.md#common-validation) also applies.

## Stop conditions and completion

A missing source, unexpected measured value, changed base commit, or non-failing gate is an escalation.
The implementer does not replace missing data with values from this document.
T0 is complete when the copied evidence passes verification and the common completion record names its directory.
