# Gear generator

Gear implementations in `lib/geargen/` are generated from the specs in `spec/`. The skills under
`.claude/skills/` own how that generation runs. This file covers the one thing that happens
outside a skill run.

## Which skill regenerates a gear

A gear that has a compiled step list `spec/<gear>/steps.md` is regenerated with the
compile+emit pipeline: run `/emit-gear <gear>` when
`python3 .claude/skills/generate-gear/check_compile.py <gear>` passes, and `/compile-gear <gear>`
first when it does not. `/generate-gear` re-interprets the full prose spec and playbook on
every round, so it is only for a gear with no step list yet, or when the user asks for it by
name.

## Running the proofs

The engine revisions are pinned in `proof/go.mod`, in the pseudo-version Go records for a module
with no tags, and that is the only place they are written. `proof/run.sh` reads them and verifies
the checkout before it runs anything; CI reads the same lines to decide what to check out. So a
local run and CI test one revision, and `go get` is the only thing that moves a pin.

A `sketch` or `decad` checkout beside the repo at any other revision makes the run refuse rather
than quietly proceed. Point `SKETCH_DIR` and `DECAD_DIR` at checkouts of the pinned commits — a
detached `git worktree` of each is the cheapest way — or pass `PROOF_VERIFY_REVISIONS=0` to run
against whatever is there.

Use that escape hatch only to decide whether an engine bump is acceptable. A green run with it
set proves nothing about the pinned engine: this was learned twice, both times by trusting a
local pass that CI then failed. Raising a pin is its own PR, and the proofs are what decide it.

## Running less than the whole proof

`proof/run.sh` with no arguments runs the whole suite, which is what CI runs and what a handoff
means. For a focused local run it also takes `--package ./<dir>`, which may repeat, and forwards
every token after `--` to `go test`:

```sh
proof/run.sh --package ./bevelgear -- -run '^TestGearProfiles$' -count=1
```

Selection is a way to wait less while working on one gear, not a way to be told the suite passed.
It changes nothing else: the engine revisions are still verified first, a bad option fails before
any of that output appears, and `go test` still decides the exit status. Run the whole suite
before handing work over.

## When Fusion gives a verdict

Loading a gear into Fusion is the only check that sees the real thing, and it happens with no
skill loaded. What comes back is knowledge that has to be written down before it evaporates.

The fix never goes into `lib/geargen/<gear>.py`. That file is build output, and editing it leaves
the spec, the step list and the proof describing a gear that no longer exists.

Send the finding to whichever of these fits:

- A Fusion behaviour that holds for every gear goes in `PLAYBOOK.md` under a new `[PB-…]` anchor.
- A behaviour specific to one gear goes in that gear's `fusion.md` under a new `[<GEAR>-F-…]`
  anchor.
- A geometry or constraint rule goes in that gear's proof as a new case.

Then regenerate. A verdict that is only remembered in a conversation is lost.

## Ask what the proof missed

Every Fusion verdict gets one more question: could the proof have caught this?

If it could, add the case before anything else, so the next run fails in seconds instead of in
the GUI. If it could not, write down why not, in the proof file, next to the thing it cannot
reach. That record is the honest edge of what the middle stage checks, and it is what tells you
where to strengthen it next.
