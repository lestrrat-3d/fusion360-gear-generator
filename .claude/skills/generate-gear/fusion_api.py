#!/usr/bin/env python3
"""Ask the `fusion` plugin's compiled API database what the Fusion 360 Python API contains.

This module is the one place the checkers learn Fusion API names. It shells out to the query
script the plugin ships, so nothing here parses stubs and nothing here maintains a copy of the
API. The plugin's database is compiled from the same source our old index was derived from
(AutodeskFusion360/FusionAPIReference), at a newer commit, and it records what class declares
each member — which our index never did.

Resolving the script: the checkers run outside a Claude session, so they cannot invoke the
plugin's skill. `~/.claude/plugins/installed_plugins.json` records an `installPath` for
`fusion@lestrrat-ai`, and the query script sits at a fixed place inside it. Set
`FUSION_QUERY_API` to override that for testing.

The three shapes of question the checkers ask:

    lookup('addByTwoPoints')   -> every symbol or member of exactly that name, with its kind
    owners('addByTwoPoints')   -> just the classes that declare a member of that name
    class_members('Sketch')    -> member name -> declaring class, inherited members included

Calls the database does not back are listed in UNVERIFIED_CALLS below. The API-call checks report
them and never suppress them. The novel-type check uses the same list to recognize matching stub
diagnostics as non-gating; see that list's comment for why the calls cannot simply be deleted or
fixed.
"""
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

PLUGIN = 'fusion@lestrrat-ai'
INSTALLED = os.path.expanduser('~/.claude/plugins/installed_plugins.json')
SCRIPT_WITHIN_PLUGIN = os.path.join('skills', 'query-api', 'scripts', 'query_fusion_api.py')

INSTALL_HINT = (
    "the Fusion API database is unavailable.\n"
    "  Install the plugin in Claude Code with:  /plugin install fusion@lestrrat-ai\n"
    "  Or point FUSION_QUERY_API at a checkout's %s." % SCRIPT_WITHIN_PLUGIN)

# One line of `show`/`search` output: a fully qualified name and its kind.
_HIT = re.compile(r'^\s*(adsk\.[A-Za-z0-9_.]+)\s+\[(\w+)[^]]*\]\s*$')
_MEMBER_HIT = re.compile(r'^\s*(adsk\.[A-Za-z0-9_.]+)\.(\w+)\s+\[(\w+)[^]]*\]\s*$')
_INHERITED_FROM = re.compile(r'^inherited from:\s+(adsk\.[A-Za-z0-9_.]+)\s*$')
_TYPE = re.compile(r'^type:\s+([A-Za-z0-9_.]+)')
_RETURNS = re.compile(r'^signature:.*\s->\s*([A-Za-z0-9_.]+)')
# `members` groups its output by declaring class: a header, then two-space-indented members.
_GROUP = re.compile(r'^(adsk\.[A-Za-z0-9_.]+)\s+\(\d+\s+members?\):\s*$')
_MEMBER = re.compile(r'^\s{2}(\w+)')

# Calls the shipped add-in makes that the API database does not back. Every entry is
# (member, the class the shipped code calls it on, receiver names that identify that class in the
# source, a note). Receiver names are exact so an unrelated namesake cannot be exempted.
#
# The checkers re-ask the database about each entry on every run rather than trusting this
# comment, so an entry whose member has since been documented is reported as stale and deleted.
#
# These are NOT waived. The API-call checks print them on every run and never count them as
# resolved. The novel-type check exempts only matching stub diagnostics, not the calls themselves.
# They are not failures either, because nothing has established that Fusion rejects them — the
# shipped add-in makes all three and is believed to run. Only loading the add-in settles it, and
# whichever way it goes the fix belongs in the spec, not in a generated file.
UNVERIFIED_CALLS = (
    ('project', 'adsk.fusion.Sketch', ('sketch', 'toolsSketch'),
     'the shipped add-ins call `sketch.project(entity)`, and the spur step list names it'),
    ('createInput2', 'adsk.fusion.SketchTexts', ('sketchTexts',),
     'the shipped add-ins call `sketch.sketchTexts.createInput2(text, height)`, and the spur '
     'step list names it; '
     '`ChamferFeatures.createInput2` and `MoveFeatures.createInput2` are real and not at issue'),
    ('addConstantRadiusEdgeSet', 'adsk.fusion.FilletFeatureInput', ('filletInput',),
     'the shipped add-in calls `filletInput.addConstantRadiusEdgeSet(...)`, and the spur '
     'step list states that `filletInput.edgeSetInputs` does not exist'),
)


class Unavailable(Exception):
    """The plugin's query script could not be found."""


_script = None


def query_script():
    """Absolute path to the plugin's query script."""
    global _script
    if _script:
        return _script
    override = os.environ.get('FUSION_QUERY_API')
    if override:
        if not os.path.exists(override):
            raise Unavailable('FUSION_QUERY_API points at %s, which does not exist' % override)
        _script = override
        return _script
    if not os.path.exists(INSTALLED):
        raise Unavailable('%s does not exist, so %s' % (INSTALLED, INSTALL_HINT))
    try:
        with open(INSTALLED) as fh:
            entries = (json.load(fh).get('plugins') or {}).get(PLUGIN) or []
    except ValueError as exc:
        raise Unavailable('%s is not readable JSON (%s)' % (INSTALLED, exc))
    for entry in entries:
        path = os.path.join(entry.get('installPath') or '', SCRIPT_WITHIN_PLUGIN)
        if os.path.exists(path):
            _script = path
            return _script
    raise Unavailable('%s records no usable install of %s, so %s' % (INSTALLED, PLUGIN, INSTALL_HINT))


def _run(*args):
    """Run one query and return its stdout.

    The exit code is deliberately ignored. The script exits non-zero both for a name it cannot
    find and for a name that reaches several members, and the second case still prints every
    candidate, which is exactly what we came for.
    """
    proc = subprocess.run([sys.executable, query_script()] + list(args),
                          capture_output=True, text=True)
    return proc.stdout


def lookup(name):
    """Every symbol or member the database records under exactly this name, as (qualname, kind).

    Matching is exact on the last path segment, so `add` never answers for `addByTwoPoints`.
    """
    out = _run('show', name)
    hits = []
    for line in out.splitlines():
        m = _HIT.match(line)
        if m and m.group(1).rsplit('.', 1)[-1] == name:
            hits.append((m.group(1), m.group(2)))
    return hits


def lookup_many(names, workers=8):
    """lookup() over many names at once. One process each, run concurrently."""
    names = list(names)
    if not names:
        return {}
    query_script()  # fail once, before fanning out
    with ThreadPoolExecutor(max_workers=min(workers, len(names))) as pool:
        return dict(zip(names, pool.map(lookup, names)))


def similar(name, limit=5):
    """Qualified names the database has that look like this one, best effort.

    A miss is worth more than "no such name" when it can say what the database does have and on
    what class: `sketchCircles` is real, but on SketchCurves rather than Sketch, and that is the
    difference between a dead end and a fix.
    """
    hits = []
    for term in (name, name[:8] if len(name) > 8 else None):
        if not term:
            continue
        for line in _run('search', term).splitlines():
            m = _HIT.match(line)
            if m and m.group(1) not in hits:
                hits.append(m.group(1))
        if hits:
            break
    return hits[:limit]


def owners(name):
    """The classes that declare a member of this name, as qualified names."""
    return [q.rsplit('.', 1)[0] for q, kind in lookup(name) if kind != 'class']


def class_members(cls):
    """member name -> the class that declares it, inherited members included."""
    out = _run('members', cls)
    found = {}
    owner = cls
    for line in out.splitlines():
        head = _GROUP.match(line)
        if head:
            owner = head.group(1)
            continue
        m = _MEMBER.match(line)
        if m:
            found.setdefault(m.group(1), owner)
    return found


def member_info(cls, name):
    """The member reached by `cls.name`, including inherited members and return type."""
    out = _run('show', '%s.%s' % (cls, name))
    info = None
    for line in out.splitlines():
        m = _MEMBER_HIT.match(line)
        if m and m.group(2) == name:
            info = {
                'lookup': m.group(1).rsplit('.', 1)[0],
                'name': m.group(2),
                'kind': m.group(3),
                'declared_on': m.group(1).rsplit('.', 1)[0],
                'returns': None,
            }
            continue
        if info is None:
            continue
        inherited = _INHERITED_FROM.match(line)
        if inherited:
            info['declared_on'] = inherited.group(1)
            continue
        typed = _TYPE.match(line) or _RETURNS.match(line)
        if typed:
            info['returns'] = typed.group(1)
    return info


def receiver_matches(receivers, tail):
    """Does a call written on `tail` belong to a watchlist entry wanting `receivers`?"""
    return receivers is None or tail in receivers


def unverified_findings(called):
    """One finding per watchlist call the caller actually saw, saying what the database knows.

    `called` maps a name to where it was seen (a "file:line" string, or any label). Names the
    caller did not see are skipped, so a gear that makes none of these calls prints nothing.
    """
    out = []
    for name, cls, _, note in UNVERIFIED_CALLS:
        where = called.get(name)
        if where is None:
            continue
        members = class_members(cls)
        if name in members:
            out.append("%s: '%s' is now declared on %s (from %s). The database has caught up — "
                       "delete this entry from UNVERIFIED_CALLS in fusion_api.py."
                       % (where, name, cls, members[name]))
            continue
        elsewhere = owners(name)
        if elsewhere:
            known = 'the database declares that name only on %s' % ', '.join(sorted(elsewhere))
        else:
            known = 'no class in the database declares that name at all'
        out.append("%s: '%s(' on %s — %s does not declare it, and %s (%s). Whether Fusion accepts "
                   "the call is unsettled; only running the add-in decides, and the fix then goes "
                   "in the spec." % (where, name, cls, cls, known, note))
    return out


if __name__ == '__main__':
    print(query_script())
