#!/usr/bin/env python3
"""Extract the playbook rules a compiled step list actually cites.

`emit-gear` hands its drafting subagent the whole 67 KB `PLAYBOOK.md` so it can
resolve the two dozen `[PB-…]` anchors `spec/<gear>/steps.md` cites. Every other
rule in that file is dead weight for a stage that only transcribes an already
compiled step list: the emit drafter never chooses a new citation, so it never
needs a rule the step list did not name.

This script writes the cited rules — and only those, plus a small fixed core of
sections every draft leans on — to `.tmp/<gear>.playbook-extract.md`. It is a
deterministic text slice of the playbook, not a rewrite: every emitted line is a
line of `PLAYBOOK.md`, so a rule reads in the extract exactly as it reads at its
source.

`compile-gear` still reads the full playbook, because that is the stage that
decides which anchors a step list cites.

Two anchor-definition forms exist, and this script honors both, the same two
`check_anchors.py` encodes:

  * a bold span opening a bullet or a paragraph — `**[PB-INPUT-READ] …`;
  * an anchor anywhere on a markdown heading line — `## … ([PB-SKETCH-FIRST])`.

A bold anchor's rule is the block it opens, up to the next definition at the
same or shallower indent, the next heading, or the next top-level bullet.
Nested anchors (`[PB-CIRCLE-CENTER]` inside `[PB-FULL-CONSTRAINT]`) therefore
travel with their parent when the parent is cited, and extract alone when only
the nested anchor is. A heading anchor's rule is its whole section.

Anchors the step list cites that the playbook does not define — the per-gear
`[SPUR-F-…]` sidecar anchors — are not errors. They live in `spec/<gear>/`
files the emit stage is forbidden to read, so they were unresolvable at emit
time before this script existed and remain so after. The generated header names
them, so a drafter meeting one knows why it cannot be looked up.

Usage:
    python3 extract_playbook.py <gear> [--steps PATH] [--playbook PATH] [--out PATH]

Run from the repo root. Exit 0 = written; 1 = a cited `PB-…` anchor (or a core
section) has no definition in the playbook; 2 = a missing or unreadable input.
"""
import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_anchors  # noqa: E402  (sibling module; sys.path is fixed up just above)

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, os.pardir, os.pardir, os.pardir))

# The citation grammar is check_anchors.py's, imported rather than copied so the
# extractor can never disagree with the gate about what counts as a citation.
CITE_RE = check_anchors.CITE_RE
BOLD_DEF_RE = check_anchors.BOLD_DEF_RE

# Anchors under this prefix are the playbook's own; a cited one the playbook does
# not define is a defect. Everything else is a per-gear sidecar anchor.
PLAYBOOK_PREFIX = "PB-"

# Sections every emitted module leans on whether or not a step cites them: the
# module skeleton, the orchestration shape, the parameter mode, and the explicit
# licence to vary what the spec does not pin. Matched by exact heading text.
CORE_SECTIONS = (
    "## Module layout & imports",
    "## `generate` orchestration",
    "## Parameters: live-expression mode vs all-Python-precomputed mode",
    "## What the spec need not pin (free to vary)",
)

GEAR_RE = re.compile(r"^[a-z][a-z0-9_]*$")
HEADING_RE = re.compile(r"^(#{1,6})(?:\s|$)")
# A bold anchor that OPENS its line, after an optional bullet or list marker.
# `check_anchors.py` counts a bold anchor anywhere on a line as a definition, so
# the playbook's own prose example ("stable IDs like `**[PB-RADIAL-DIM]**`") is a
# definition to that gate. It is not the rule's home, and slicing a block there
# would drop the real rule, so an opening match outranks an inline one.
OPENING_DEF_RE = re.compile(
    r"^(?:[-*+]\s+|\d+\.\s+)?\*\*\[(%s)\]" % check_anchors.ANCHOR)

# Definition ranks, strongest first: a heading owns a whole section, a bold span
# opening a line owns the block it starts, an inline mention owns nothing better.
RANK_HEADING, RANK_OPENING, RANK_INLINE = 3, 2, 1


class InputError(Exception):
    """A missing or unreadable input file — exit 2, not a content defect."""


class DefectError(Exception):
    """The steps file and the playbook disagree — exit 1."""


def read_lines(path, what):
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read().splitlines()
    except OSError as exc:
        raise InputError("cannot read %s %s: %s" % (what, path, exc))


def indent_of(line):
    """Leading spaces; a `- ` bullet in column 0 has indent 0."""
    return len(line) - len(line.lstrip(" "))


def heading_level(line):
    """`#` count for a heading line, else 0."""
    match = HEADING_RE.match(line.lstrip())
    return len(match.group(1)) if match else 0


def line_definitions(line):
    """Anchor ids this line defines, each with its rank.

    Both of `check_anchors.py`'s forms count: a bold span, and any anchor on a
    heading line. The rank records which of them, so a rule with more than one
    mention is sliced at the mention that owns it.
    """
    found = {}
    opening = OPENING_DEF_RE.match(line.lstrip(" "))
    for match in BOLD_DEF_RE.finditer(line):
        anchor = match.group(1)
        rank = RANK_OPENING if opening and opening.group(1) == anchor else RANK_INLINE
        found[anchor] = max(rank, found.get(anchor, 0))
    if heading_level(line):
        for match in CITE_RE.finditer(line):
            found[match.group(1)] = RANK_HEADING
    return found


def collect_cites(lines):
    """Cited ids in first-citation order."""
    seen = []
    for line in lines:
        for match in CITE_RE.finditer(line):
            if match.group(1) not in seen:
                seen.append(match.group(1))
    return seen


def index_definitions(lines):
    """Map anchor id -> (line index, rank, indent), strongest definition wins.

    An id defined more than once resolves to its strongest mention, first one
    of those if several tie. `[PB-VALIDATE-INPUTS]` sits on its `###` heading
    and opens the paragraph below it, so it resolves to the heading, whose
    section contains that paragraph anyway.
    """
    defs = {}
    for index, line in enumerate(lines):
        for anchor, rank in line_definitions(line).items():
            existing = defs.get(anchor)
            if existing is None or rank > existing[1]:
                defs[anchor] = (index, rank, indent_of(line))
    return defs


def section_end(lines, start):
    """End (exclusive) of the section opened by the heading at `start`."""
    level = heading_level(lines[start])
    for index in range(start + 1, len(lines)):
        candidate = heading_level(lines[index])
        if candidate and candidate <= level:
            return index
    return len(lines)


def bold_block_end(lines, start):
    """End (exclusive) of the rule block opened by the bold anchor at `start`."""
    base = indent_of(lines[start])
    for index in range(start + 1, len(lines)):
        line = lines[index]
        if heading_level(line):
            return index
        current = indent_of(line)
        if line_definitions(line) and current <= base:
            return index
        if line.strip() and current == 0 and line.startswith("- "):
            return index
    return len(lines)


def block_for(lines, definition):
    start, rank, _ = definition
    if rank == RANK_HEADING:
        return (start, section_end(lines, start))
    return (start, bold_block_end(lines, start))


def find_core_blocks(lines, core_sections):
    """Locate each core section by exact heading text; a missing one is a defect."""
    positions = {}
    for index, line in enumerate(lines):
        if heading_level(line) and line.rstrip() not in positions:
            positions[line.rstrip()] = index

    blocks, missing = [], []
    for heading in core_sections:
        index = positions.get(heading)
        if index is None:
            missing.append(heading)
            continue
        blocks.append((index, section_end(lines, index)))
    if missing:
        raise DefectError(
            "core section(s) missing from the playbook: %s" % ", ".join(missing))
    return blocks


def preamble_block(lines):
    """Everything before the first `##` — title, purpose, anchor convention."""
    for index, line in enumerate(lines):
        if heading_level(line) >= 2:
            return (0, index)
    return (0, len(lines))


def dedupe_blocks(blocks):
    """Drop repeats and any block fully contained in another kept block."""
    kept, reach = [], -1
    for start, end in sorted(set(blocks), key=lambda b: (b[0], -b[1])):
        if end <= reach:
            continue
        kept.append((start, end))
        reach = end
    return kept


def heading_trail(lines, start):
    """Section headings still open above `start`, outermost first.

    A block that opens with a heading of its own gets only that heading's
    ancestors, never the sibling section that happens to precede it. The file's
    `#` title is left out: it is already the first line of the always-included
    preamble.
    """
    limit = heading_level(lines[start]) or 7
    trail = []
    for index in range(start - 1, -1, -1):
        level = heading_level(lines[index])
        if level < 2 or level >= limit:
            continue
        trail.insert(0, lines[index])
        limit = level
        if level == 2:
            break
    return trail


def strip_trailing_blanks(chunk):
    while chunk and not chunk[-1].strip():
        chunk.pop()
    return chunk


def build_header(gear, playbook_rel, unresolved):
    lines = [
        "# Playbook extract for `%s` — generated, do not edit" % gear,
        "",
        "This file is generated by `extract_playbook.py` from `%s`" % playbook_rel,
        "for `%s`. Do not edit it: edit the playbook and re-run the extractor." % gear,
        "",
        "It holds the fixed core sections plus every rule the step list cites by anchor. The",
        "playbook's other rules were omitted deliberately — their absence is a choice, not a gap,",
        "and this extract replaces reading `PLAYBOOK.md` for this stage.",
        "",
    ]
    if unresolved:
        lines += [
            "The step list also cites the anchors below. They are defined in the per-gear spec",
            "sidecars this stage does not read, so they are intentionally unresolved here:",
            "",
        ]
        lines += ["- `[%s]`" % anchor for anchor in unresolved]
        lines.append("")
    else:
        lines += [
            "Every anchor the step list cites is defined in the playbook and appears below.",
            "",
        ]
    lines.append("---")
    lines.append("")
    return lines


def render(lines, blocks, header):
    out = list(header)
    emitted_trail = []
    for start, end in blocks:
        trail = heading_trail(lines, start)
        shared = 0
        while (shared < len(trail) and shared < len(emitted_trail)
               and trail[shared] == emitted_trail[shared]):
            shared += 1
        for heading in trail[shared:]:
            out.append(heading)
            out.append("")
        emitted_trail = trail
        out.extend(strip_trailing_blanks(list(lines[start:end])))
        out.append("")
    return "\n".join(out).rstrip("\n") + "\n"


def extract(gear, steps_path, playbook_path, core_sections=None):
    """Return the extract text plus a summary tuple. Raises on any defect."""
    core_sections = CORE_SECTIONS if core_sections is None else core_sections
    steps_lines = read_lines(steps_path, "steps file")
    playbook_lines = read_lines(playbook_path, "playbook")

    cites = collect_cites(steps_lines)
    defs = index_definitions(playbook_lines)

    cited_here, unresolved, undefined = [], [], []
    for anchor in cites:
        if anchor in defs:
            cited_here.append(anchor)
        elif anchor.startswith(PLAYBOOK_PREFIX):
            undefined.append(anchor)
        else:
            unresolved.append(anchor)
    if undefined:
        raise DefectError(
            "cited but defined nowhere in the playbook: %s" % ", ".join(sorted(undefined)))

    blocks = [preamble_block(playbook_lines)]
    blocks += find_core_blocks(playbook_lines, core_sections)
    blocks += [block_for(playbook_lines, defs[a]) for a in cited_here]
    blocks = dedupe_blocks(blocks)

    playbook_rel = os.path.relpath(os.path.abspath(playbook_path), REPO_ROOT)
    header = build_header(gear, playbook_rel.replace(os.sep, "/"), unresolved)
    text = render(playbook_lines, blocks, header)
    playbook_bytes = len(("\n".join(playbook_lines) + "\n").encode("utf-8"))
    return text, (len(cites), len(blocks), playbook_bytes)


def main(argv, core_sections=None):
    parser = argparse.ArgumentParser(
        prog="extract_playbook.py",
        description="Write the playbook rules a gear's step list cites.")
    parser.add_argument("gear", help="gear name, e.g. spurgear")
    parser.add_argument("--steps", help="step list (default spec/<gear>/steps.md)")
    parser.add_argument("--playbook", help="playbook (default the one beside this script)")
    parser.add_argument("--out", help="output (default .tmp/<gear>.playbook-extract.md)")
    args = parser.parse_args(argv[1:])

    if not GEAR_RE.match(args.gear):
        print("extract_playbook: not a gear name: %r" % args.gear, file=sys.stderr)
        return 2

    steps = args.steps or os.path.join(REPO_ROOT, "spec", args.gear, "steps.md")
    playbook = args.playbook or os.path.join(HERE, "PLAYBOOK.md")
    out = args.out or os.path.join(REPO_ROOT, ".tmp", "%s.playbook-extract.md" % args.gear)

    try:
        text, (cited, blocks, playbook_bytes) = extract(
            args.gear, steps, playbook, core_sections)
    except InputError as exc:
        print("extract_playbook: %s" % exc, file=sys.stderr)
        return 2
    except DefectError as exc:
        print("extract_playbook: %s" % exc, file=sys.stderr)
        print("extract_playbook: run check_anchors.py and fix before drafting.", file=sys.stderr)
        return 1

    parent = os.path.dirname(os.path.abspath(out))
    try:
        os.makedirs(parent, exist_ok=True)
        with open(out, "w", encoding="utf-8") as handle:
            handle.write(text)
    except OSError as exc:
        print("extract_playbook: cannot write %s: %s" % (out, exc), file=sys.stderr)
        return 2

    share = 100.0 * len(text.encode("utf-8")) / playbook_bytes if playbook_bytes else 0.0
    print("extract_playbook: %s: %d anchors cited, %d blocks written, %d bytes vs %d "
          "playbook bytes (%.0f%%) -> %s"
          % (args.gear, cited, blocks, len(text.encode("utf-8")), playbook_bytes, share, out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
