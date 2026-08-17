# Gear generator

Gear implementations in `lib/geargen/` are generated from the specs in `spec/`. The skills under
`.claude/skills/` own how that generation runs. This file covers the one thing that happens
outside a skill run.

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
