#!/usr/bin/env python3
"""Generate the `## Provenance` table of a compiled step list, so nothing hand-types a hash.

`provenance.py` owns the input set for one gear (the gear's `instructions.md`, its optional
`fusion.md`, the shared `PLAYBOOK.md`, and any auxiliary Markdown document those reference) and the
`git hash-object` of each one. `check_compile.py` shares that same module to gate a step list's
provenance table against drift. This script is the other side of that gate: it renders the table
and, with `--write`, replaces the provenance section of a step list with it in place.

No language model should ever type a hash into a step list. Run this instead, from the repo root:

    python3 gen_provenance.py <gear>                       # print the table
    python3 gen_provenance.py <gear> --write PATH           # stamp it into PATH

Run from the repo root, like every other script in this directory: every path in the input set is
built relative to the current working directory.

Exit 0 = the table was printed or written. Exit 2 = bad usage, a missing or unreadable input, a
target with no `## Provenance` heading, or a `git hash-object` failure — message on stderr,
prefixed `gen_provenance:`. Exit 1 is deliberately unused; this script makes no findings about an
artifact, so it never gates anything. That is check 4 of `check_compile.py`, and a second copy
here could disagree with it.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import provenance  # noqa: E402  (sibling module; sys.path is fixed up just above)

USAGE = 'usage: gen_provenance.py <gear> [--write <steps.md>]'


def parse_args(argv):
    """Return (gear, write_path or None), or raise ProvenanceError on a bad argument list.

    Hand-rolled to match the other scripts in this directory (check_compile, check_step_calls,
    check_input_read all test len(argv) and print a usage line); argparse is not used here.
    Accepts `--write PATH` and `--write=PATH`. Any other flag, a missing gear, or extra
    positionals is a usage error.
    """
    args = list(argv[1:])
    if not args:
        raise provenance.ProvenanceError(USAGE)
    gear = args[0]
    if gear.startswith('-'):
        raise provenance.ProvenanceError(USAGE)
    rest = args[1:]
    write_path = None
    if rest:
        first = rest[0]
        if first == '--write':
            if len(rest) != 2:
                raise provenance.ProvenanceError(USAGE)
            write_path = rest[1]
        elif first.startswith('--write='):
            if len(rest) != 1:
                raise provenance.ProvenanceError(USAGE)
            write_path = first[len('--write='):]
        else:
            raise provenance.ProvenanceError(USAGE)
        if not write_path:
            raise provenance.ProvenanceError(USAGE)
    return gear, write_path


def write_section(path, gear):
    """Stamp one step list. Reads path, replaces the section, writes it back.

    Returns the number of rows written, for the summary line.
    """
    try:
        src = provenance.read(path)
    except OSError as exc:
        raise provenance.ProvenanceError('%s: %s' % (path, exc.strerror or exc)) from exc
    rows = provenance.table_rows(provenance.ordered_provenance_inputs(gear))
    section = '%s\n\n%s' % (provenance.HEADING, provenance.render_table(rows))
    updated = provenance.replace_section(src, section)
    with open(path, 'w') as fh:
        fh.write(updated)
    return len(rows)


def main(argv):
    """Parse argv, dispatch, translate ProvenanceError and OSError into exit 2.

    argv[0] is the program name, matching check_compile.main so tests can call
    main(['gen_provenance.py', 'gear']) the way they already call the checker.
    """
    try:
        gear, write_path = parse_args(argv)
        instructions = os.path.join('spec', gear, 'instructions.md')
        if not os.path.isfile(instructions):
            raise provenance.ProvenanceError(
                '%s does not exist; is %r a real gear?' % (instructions, gear))
        if write_path is None:
            rows = provenance.table_rows(provenance.ordered_provenance_inputs(gear))
            print('%s\n\n%s' % (provenance.HEADING, provenance.render_table(rows)))
            return 0
        count = write_section(write_path, gear)
        print('gen_provenance: stamped %d source(s) into %s' % (count, write_path))
        return 0
    except provenance.ProvenanceError as exc:
        print('gen_provenance: %s' % exc, file=sys.stderr)
        return 2
    except OSError as exc:
        print('gen_provenance: %s' % exc, file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv))
