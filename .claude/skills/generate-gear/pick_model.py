#!/usr/bin/env python3
"""Resolve which model a spawned subagent should run on.

The skills spawn subagents for two kinds of work, and the tier follows the
kind rather than a pinned model name:

  * work that interprets prose or designs runs on the session's default
    model, because getting it wrong costs a wrong artifact rather than a
    failed gate;
  * mechanical transcription runs one step down the ladder, because the
    gates, not the drafter, judge that output.

Naming a model outright ("spawn on haiku") gets this wrong the moment the
session default moves: pinned to `haiku`, an Opus session drops the drafter
two rungs instead of one. This script keeps the ladder and its floor in one
tested place so a SKILL.md can state the *role* and let the tier follow.

The session's default model is the one fact the script cannot discover — only
the orchestrating agent knows it — so it is an argument, not a probe.

Usage:  python3 pick_model.py --role <design|mechanical> --default <model>
                              [--escalated] [--mapping <json-path>]

stdout is the model name and nothing else, so the caller can read one token.
The one-line reason goes to stderr. Exit 0 = resolved; exit 2 = bad usage.
"""
import argparse
import json
import re
import sys

# Highest to lowest. A model absent from this list is off-ladder: legal to
# pass, but there is no defined step below it.
LADDER = ('opus', 'sonnet', 'haiku')

DESIGN = 'design'
MECHANICAL = 'mechanical'
# The orchestrator is not spawned by anything, but naming its role lets
# MODELS.md state one rule for every agent in the pipeline.
ROLES = {DESIGN: DESIGN, 'orchestrator': DESIGN, MECHANICAL: MECHANICAL}

USAGE_EXIT = 2
MODEL_ID = re.compile(r'\S+\Z')


class MappingError(ValueError):
    """A model mapping that cannot be read or does not match schema 1."""


def _object(pairs):
    """Build one JSON object while rejecting duplicate keys."""
    result = {}
    for key, value in pairs:
        if key in result:
            raise MappingError("duplicate key %r" % key)
        result[key] = value
    return result


def _constant(value):
    raise MappingError('invalid JSON constant %s' % value)


def load_mapping(path):
    """Read and validate a complete schema-1 model mapping."""
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            mapping = json.load(
                handle, object_pairs_hook=_object, parse_constant=_constant)
    except MappingError:
        raise
    except (OSError, UnicodeError, ValueError) as exc:
        raise MappingError('%s could not be read as JSON (%s)' % (path, exc))

    if not isinstance(mapping, dict):
        raise MappingError('mapping root must be an object')
    if set(mapping) != {'schema', 'mechanical'}:
        raise MappingError('mapping must contain exactly schema and mechanical')
    if type(mapping['schema']) is not int or mapping['schema'] != 1:
        raise MappingError('mapping schema must be the integer 1')
    mechanical = mapping['mechanical']
    if not isinstance(mechanical, dict):
        raise MappingError('mapping mechanical value must be an object')
    for source, target in mechanical.items():
        if not _valid_model_id(source):
            raise MappingError('mechanical mapping source must be a nonempty, whitespace-free string')
        if not _valid_model_id(target):
            raise MappingError('mechanical mapping target must be a nonempty, whitespace-free string')
    return mapping


def _valid_model_id(value):
    if not isinstance(value, str) or not MODEL_ID.fullmatch(value):
        return False
    try:
        value.encode('utf-8')
    except UnicodeEncodeError:
        return False
    return True


def step_down(model):
    """The rung below `model`, or None when there is not one.

    None covers both floors: the bottom of the ladder, and a model that is
    not on the ladder at all.
    """
    try:
        index = LADDER.index(model)
    except ValueError:
        return None
    if index + 1 >= len(LADDER):
        return None
    return LADDER[index + 1]


def resolve(role, default, escalated=False, mapping=None):
    """Return (model, reason) for a role against a session default.

    Never raises on an unrecognised `default`: an off-ladder name is a
    fallback case, not an error, because the ladder is allowed to lag the
    set of models a harness offers.
    """
    canonical = ROLES[role]
    if canonical == DESIGN:
        return default, 'design role runs on the session default'
    if escalated:
        return default, 'escalated: mechanical role stepped back up to the session default'
    if mapping is not None and default in mapping['mechanical']:
        target = mapping['mechanical'][default]
        return target, 'mechanical mapping sends %s to %s' % (default, target)
    lower = step_down(default)
    if lower is None:
        if default in LADDER:
            return default, 'default %s is the bottom of the ladder; no step down' % default
        return default, 'default %s is not on the ladder; no step down' % default
    return lower, 'mechanical role steps %s down to %s' % (default, lower)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='pick_model.py',
        description='Resolve the model a spawned subagent should run on.')
    parser.add_argument(
        '--role', required=True, choices=sorted(ROLES),
        help='what the subagent does: design (or orchestrator) interprets and '
             'designs, mechanical transcribes')
    parser.add_argument(
        '--default', required=True, metavar='MODEL',
        help="the session's default model, which only the orchestrator knows")
    parser.add_argument(
        '--escalated', action='store_true',
        help='a mechanical role that has already failed its rounds; run it on '
             'the session default instead of a step down')
    parser.add_argument(
        '--mapping', default=None, metavar='JSON_PATH',
        help='optional schema-1 mechanical-model mapping')
    return parser.parse_args(argv)


def main(argv, out=sys.stdout, err=sys.stderr):
    args = parse_args(argv)
    try:
        mapping = load_mapping(args.mapping) if args.mapping is not None else None
    except MappingError as exc:
        err.write('pick_model.py: %s\n' % exc)
        return USAGE_EXIT
    model, reason = resolve(args.role, args.default, args.escalated, mapping)
    out.write('%s\n' % model)
    err.write('pick_model: %s\n' % reason)
    return 0


def cli():
    try:
        sys.exit(main(sys.argv[1:]))
    except SystemExit as exc:
        # argparse already printed the complaint; normalise its exit code so a
        # usage error is distinguishable from a resolved answer.
        raise SystemExit(USAGE_EXIT if exc.code not in (0, None) else 0)


if __name__ == '__main__':
    cli()
