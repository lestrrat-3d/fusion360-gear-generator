#!/usr/bin/env python3
"""Render a skill's standard drafting prompt with the gear name substituted.

Each of the three drafting skills keeps its fixed subagent prompt in a
`prompt.md` sibling to its `SKILL.md`, with `{{gear}}` as the only placeholder.
This script prints that template with `{{gear}}` replaced, so "use the prompt
verbatim" is the output of a program instead of a promise the orchestrating
model has to keep by retyping.

Angle-bracket tokens (`<Class>`, `<file>`, `<n>`, …) are literal prompt text and
are never touched; only `{{…}}` is a placeholder.

Usage:  python3 render_prompt.py <skill> [<gear>]
Exit 0 = prompt printed to stdout; exit 2 = anything else (broken input).
"""
import re
import sys
from pathlib import Path

KNOWN_SKILLS = ('compile-gear', 'emit-gear', 'generate-gear')
DEFAULT_GEAR = 'spurgear'

PLACEHOLDER_RE = re.compile(r'\{\{([A-Za-z0-9_-]+)\}\}')
GEAR_RE = re.compile(r'^[a-z][a-z0-9_-]*$')

USAGE = 'usage: render_prompt.py <skill> [<gear>]'


class RenderError(Exception):
    """A broken input: the message is written to stderr and nothing is printed."""


def template_path(skill, skills_root):
    """Return the `prompt.md` path for `skill` under `skills_root`."""
    if skill not in KNOWN_SKILLS:
        raise RenderError(
            'unknown skill {!r}; known skills are {}'.format(
                skill, ', '.join(KNOWN_SKILLS)))
    return Path(skills_root) / skill / 'prompt.md'


def render(template_text, values):
    """Substitute `{{name}}` placeholders in `template_text` from `values`.

    Every placeholder must be a key of `values`, and every key of `values` must
    occur at least once — a template someone hard-coded a gear name into must
    not render silently. Any `{{` or `}}` surviving substitution (a malformed
    placeholder such as `{{gear}`) is an error too.
    """
    found = set(PLACEHOLDER_RE.findall(template_text))
    unknown = sorted(found - set(values))
    if unknown:
        raise RenderError(
            'template uses unknown placeholder(s): {}'.format(
                ', '.join('{{{{{}}}}}'.format(name) for name in unknown)))
    missing = sorted(set(values) - found)
    if missing:
        raise RenderError(
            'template contains no {}'.format(
                ', '.join('{{{{{}}}}}'.format(name) for name in missing)))

    rendered = PLACEHOLDER_RE.sub(lambda match: values[match.group(1)], template_text)
    if '{{' in rendered or '}}' in rendered:
        raise RenderError('template has stray {{ or }} left after substitution')
    return rendered


def main(argv, skills_root=None):
    if skills_root is None:
        skills_root = Path(__file__).resolve().parent.parent

    args = list(argv[1:])
    if not 1 <= len(args) <= 2:
        sys.stderr.write(USAGE + '\n')
        return 2
    skill = args[0]
    gear = args[1] if len(args) == 2 else DEFAULT_GEAR

    try:
        if not GEAR_RE.match(gear):
            raise RenderError(
                'invalid gear name {!r}; expected {}'.format(gear, GEAR_RE.pattern))
        path = template_path(skill, skills_root)
        try:
            template_text = path.read_text(encoding='utf-8')
        except OSError as exc:
            raise RenderError('cannot read template {}: {}'.format(path, exc))
        sys.stdout.write(render(template_text, {'gear': gear}))
    except RenderError as exc:
        sys.stderr.write('render_prompt: {}\n'.format(exc))
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
