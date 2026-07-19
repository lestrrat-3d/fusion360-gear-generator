#!/usr/bin/env python3
"""Anchor-citation linter for the spec/playbook rule-anchor convention.

The playbook and the per-gear spec sidecars declare rules under stable IDs
(`[PB-…]`, `[SPUR-F-…]`, `[CYCLOIDAL-F-…]`, …); specs cite an ID instead of
restating the rule. A citation that resolves to nothing — a typo, a renamed
anchor, or a truncated cite like "([SPUR-F parity)" — silently unbinds the
rule at that point of use, and no other gate notices. This script makes the
link mechanical:

  * every cited ID must have a definition somewhere in the scanned set
    (definition = the ID at the start of a bold span `**[ID]…`, or anywhere
    on a markdown heading line);
  * an ID must not be *defined* in more than one file (drift);
  * no malformed/truncated anchor-like cites (`[ABC-DEF` with no closing
    bracket).

Cross-directory citations (one gear citing another gear's `*-F-*` anchor) are
listed as information only — legitimate for declared dependencies, worth
eyeballing otherwise.

Usage:  python3 check_anchors.py [repo-root]
Scans <root>/.claude/skills/generate-gear/*.md and <root>/spec/**/*.md.
Exit 0 = clean; exit 1 = at least one unresolved / multiply-defined /
malformed anchor.
"""
import glob
import os
import re
import sys

ANCHOR = r"[A-Z][A-Z0-9]*(?:-[A-Z0-9]+)+"
CITE_RE = re.compile(r"\[(%s)\]" % ANCHOR)
BOLD_DEF_RE = re.compile(r"\*\*\[(%s)\]" % ANCHOR)
# an anchor-looking token whose bracket never closes: "[SPUR-F parity" etc.
# The ellipsis forms "[PB-…]" / "[SPUR-F…]" are legitimate family-placeholder
# prose (a reference to the whole anchor family), not truncation — allow them.
MALFORMED_RE = re.compile(r"\[(%s)(?![\]A-Z0-9…-])" % ANCHOR)


def scan_files(root):
    paths = sorted(
        glob.glob(os.path.join(root, ".claude/skills/generate-gear/*.md"))
        + glob.glob(os.path.join(root, "spec/**/*.md"), recursive=True)
    )
    return [p for p in paths if os.path.isfile(p)]


def main(root):
    defs = {}       # id -> set of defining files
    cites = {}      # id -> list of (file, lineno)
    malformed = []  # (file, lineno, token)

    files = scan_files(root)
    if not files:
        print("anchor check: no .md files found under %s" % root, file=sys.stderr)
        return 2

    for path in files:
        rel = os.path.relpath(path, root)
        with open(path, encoding="utf-8") as fh:
            for lineno, line in enumerate(fh, 1):
                for m in CITE_RE.finditer(line):
                    cites.setdefault(m.group(1), []).append((rel, lineno))
                is_heading = line.lstrip().startswith("#")
                for m in BOLD_DEF_RE.finditer(line):
                    defs.setdefault(m.group(1), set()).add(rel)
                if is_heading:
                    for m in CITE_RE.finditer(line):
                        defs.setdefault(m.group(1), set()).add(rel)
                for m in MALFORMED_RE.finditer(line):
                    # ignore tokens that are also a well-formed cite on the
                    # same position (CITE_RE requires the closing bracket the
                    # malformed pattern forbids, so no overlap in practice)
                    malformed.append((rel, lineno, m.group(1)))

    problems = []
    for aid, sites in sorted(cites.items()):
        if aid not in defs:
            where = ", ".join("%s:%d" % s for s in sites[:5])
            problems.append("  [%s] cited but defined nowhere (%s)" % (aid, where))
    for aid, dfiles in sorted(defs.items()):
        if len(dfiles) > 1:
            problems.append(
                "  [%s] defined in %d files: %s" % (aid, len(dfiles), ", ".join(sorted(dfiles))))
    for rel, lineno, token in malformed:
        problems.append("  %s:%d: malformed/truncated anchor cite '[%s'" % (rel, lineno, token))

    # informational: cross-directory citations of per-gear (*-F-*) anchors
    cross = []
    for aid, sites in sorted(cites.items()):
        dfiles = defs.get(aid, set())
        gear_dirs = {os.path.dirname(f) for f in dfiles if f.startswith("spec" + os.sep)}
        if not gear_dirs:
            continue
        for rel, lineno in sites:
            if rel.startswith("spec" + os.sep) and os.path.dirname(rel) not in gear_dirs:
                cross.append("  [%s] cited from %s:%d, defined in %s" % (
                    aid, rel, lineno, ", ".join(sorted(gear_dirs))))

    if problems:
        print("anchor check: %d problem(s):" % len(problems))
        for p in problems:
            print(p)
        if cross:
            print("cross-directory citations (informational):")
            for c in cross:
                print(c)
        return 1

    print("anchor check: OK (%d anchors defined, %d cited, %d files)"
          % (len(defs), len(cites), len(files)))
    if cross:
        print("cross-directory citations (informational — fine for declared dependencies):")
        for c in cross:
            print(c)
    return 0


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else "."
    sys.exit(main(root))
