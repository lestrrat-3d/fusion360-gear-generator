# Which model a spawned agent runs on

Every skill in this directory spawns subagents. This file says which model each one gets, and
it is the only place that says it — a SKILL.md names the *role* and lets the tier follow.

## The rule

| Role | Tier |
|---|---|
| The orchestrator itself | the session's default model |
| A subagent that interprets prose or designs | the session's default model |
| A subagent doing mechanical transcription | one step down the ladder |

Work that interprets prose stays on the default model because getting it wrong produces a
wrong artifact that reads as a right one. Mechanical transcription steps down because the
gates, not the drafter, judge that output, so a cheaper model costs a round at worst.

## The ladder

Highest to lowest: `opus`, then `sonnet`, then `haiku`. `fable` is not on the ladder.

Two fallbacks, both of which mean "run it on the default":

- The default already sits at the bottom of the ladder, so there is no rung below it.
- The default is not on the ladder at all. The ladder is allowed to lag the models a harness
  offers, and an unfamiliar name must not fail a run.

Where the harness offers no model option at all, skip the option and let the spawn take
whatever it takes.

## Resolving it

Never work the ladder out by hand, and never write a model name into a SKILL.md. Ask:

    python3 .claude/skills/generate-gear/pick_model.py --role <design|mechanical> --default <model>

`--default` is the session's default model, which is the one fact the script cannot discover
for itself. stdout is the model name and nothing else; the reason goes to stderr.

    $ python3 .claude/skills/generate-gear/pick_model.py --role mechanical --default opus
    sonnet

Pass `--escalated` for a mechanical role that has already burned its rounds, and it returns the
session default instead of a step down. That is how a skill's "respawn on the default model
after two failed rounds" rule gets applied, rather than remembered.

## What this does not cover

A resumed agent keeps the model it was spawned on. The `SendMessage` continue-paths in these
skills therefore make no tier decision, and there is nothing to resolve before sending one.

## Why the role and not the model

Naming a model outright is wrong the moment the session default moves. Pinned to `haiku`, an
Opus session drops its mechanical drafter two rungs instead of one. That is not hypothetical:
it is what a measured `/emit-gear spurgear` run did on 2026-08-24, and the extra distance cost
a full retry round on gate failures the next rung up would likely have avoided.
