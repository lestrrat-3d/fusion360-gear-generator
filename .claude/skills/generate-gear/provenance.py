#!/usr/bin/env python3
"""The provenance input set, its hashes, and the table format that stamps them.

This module owns the one definition of what a compiled step list's `## Provenance` table must
contain and how that table is rendered. `check_compile.py` imports the discovery functions from
here to gate a step list's provenance table against drift; `gen_provenance.py` imports the same
functions, plus the table-rendering ones below, to generate that table instead of asking an LLM
to hand-type `git hash-object` output. Both callers must run from the repository root, since every
path here is built relative to the current working directory.
"""
import os
import re
import subprocess

# The edges here are not symmetric, and that is left as it is. The lookbehind excludes `.`, `/`
# and `-` while `\b` on the right excludes only word characters, so a name that continues past the
# extension yields the `.md` prefix inside it. What that costs is one extra path in the provenance
# input set, which is a file the table must stamp; the complaint is BLOCKING and names the file,
# so the failure is loud. Closing it by excluding `.` on the right would drop a reference at the
# end of a sentence, and a reference dropped from that set is a source whose edit leaves a stale
# step list looking healthy, which is the silent direction this gate must not fail in.
DOCUMENT_REF = re.compile(r'(?<![\w./-])[\w./-]+\.md\b')


def referenced_documents(path):
    """Return existing Markdown documents referenced from one gear's input spec."""
    found = set()
    for reference in DOCUMENT_REF.findall(read(path)):
        candidates = (
            os.path.normpath(os.path.join(os.path.dirname(path), reference)),
            os.path.normpath(reference),
        )
        for candidate in candidates:
            if not os.path.isfile(candidate):
                continue
            if candidate != path:
                found.add(candidate)
            break
    return found


PLAYBOOK = os.path.join('.claude', 'skills', 'generate-gear', 'PLAYBOOK.md')


def provenance_inputs(gear):
    """Existing source files whose hashes define a compiled step list."""
    instructions = os.path.join('spec', gear, 'instructions.md')
    fusion = os.path.join('spec', gear, 'fusion.md')
    playbook = PLAYBOOK
    specs = [path for path in (instructions, fusion) if os.path.isfile(path)]
    inputs = {
        path for path in (instructions, fusion, playbook) if os.path.isfile(path)
    }
    for path in specs:
        inputs.update(referenced_documents(path))
    return inputs


def read(path):
    with open(path) as fh:
        return fh.read()


def blob_hash(path):
    return subprocess.run(['git', 'hash-object', path],
                          capture_output=True, text=True).stdout.strip()


HEADING = '## Provenance'
TABLE_HEADER = ('| file | `git hash-object` |', '|---|---|')
BLOB_HASH = re.compile(r'^[0-9a-f]{40}$')
STAMPED_ROW = re.compile(r'\|\s*`([\w./-]+)`\s*\|\s*`([0-9a-f]{40})`\s*\|')
SECTION_END = re.compile(r'^##\s', re.M)


class ProvenanceError(Exception):
    """A provenance input cannot be hashed, or a step list has nowhere to put the table."""


def ordered_provenance_inputs(gear):
    """The provenance input set in the order the table writes it.

    Returns a list: the gear's instructions, its fusion sidecar if present, every other
    member sorted, and the playbook last. Order is cosmetic to check_compile.py, which reads
    the rows into a dict, and load-bearing here: a stable order makes a regenerated table a
    no-op diff when nothing changed.
    """
    inputs = provenance_inputs(gear)
    instructions = os.path.join('spec', gear, 'instructions.md')
    fusion = os.path.join('spec', gear, 'fusion.md')
    ordered = []
    if instructions in inputs:
        ordered.append(instructions)
    if fusion in inputs:
        ordered.append(fusion)
    remainder = inputs - {instructions, fusion, PLAYBOOK}
    ordered.extend(sorted(remainder))
    if PLAYBOOK in inputs:
        ordered.append(PLAYBOOK)
    return ordered


def table_rows(paths):
    """[(path, blob hash)] for paths, in the order given.

    Raises ProvenanceError naming the path when `git hash-object` returns anything that is not
    40 hex characters, so a broken git never silently produces a table with an empty cell.
    """
    rows = []
    for path in paths:
        digest = blob_hash(path)
        if not BLOB_HASH.match(digest):
            raise ProvenanceError(
                "%s: `git hash-object` did not return a 40-character hash (got %r)"
                % (path, digest))
        rows.append((path, digest))
    return rows


def render_table(rows):
    """The two-column markdown table, header included, with no trailing newline."""
    lines = list(TABLE_HEADER)
    lines.extend('| `%s` | `%s` |' % (path, digest) for path, digest in rows)
    return '\n'.join(lines)


def provenance_section(gear):
    """`## Provenance`, a blank line, and the table for one gear. No trailing newline."""
    rows = table_rows(ordered_provenance_inputs(gear))
    return '%s\n\n%s' % (HEADING, render_table(rows))


def replace_section(src, section):
    """Return src with its provenance section replaced by `section`.

    The section runs from the `## Provenance` heading to the next line starting `## ` or to
    the end of the file. Raises ProvenanceError when src has no such heading: the heading's
    position is the drafter's decision (check_compile.py reads proof paths from the text above
    it), so this never invents one.
    """
    start = src.find(HEADING)
    if start < 0:
        raise ProvenanceError("no `%s` heading found; the drafter must write it" % HEADING)
    after_heading = start + len(HEADING)
    match = SECTION_END.search(src, after_heading)
    end = match.start() if match else len(src)
    before = src[:start]
    tail = src[end:]
    if tail:
        return '%s%s\n\n%s' % (before, section, tail)
    return '%s%s\n' % (before, section)
