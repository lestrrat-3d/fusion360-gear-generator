#!/usr/bin/env python3
"""Static check: every dialog input is read with the helper that matches the
type it was declared with in `configure()`.

The pyright gate cannot catch this class of bug: the framework readers resolve
an input by a runtime string key (`inputs.itemById(name)` → generic
`CommandInput`), so the concrete `BoolValueCommandInput` / `ValueCommandInput`
type is invisible at the call site. This script pairs the `add*Input(id, …)`
declaration with the `get_*(inputs, id, …)` read by **input id** and flags a
mismatch — e.g. reading an `addBoolValueInput` with `get_value` (which accesses
`.expression`, a member `BoolValueCommandInput` does not have, the exact crash
this guards against).

Usage:  python3 check_input_read.py <generated.py>
Exit 0 = clean (BLOCKING-free); exit 1 = at least one mismatch (a real bug).
"""
import ast
import sys

# add*Input method  ->  the family the input belongs to
ADD_KIND = {
    "addValueInput": "value",
    "addStringValueInput": "value",      # string value inputs accept expressions; read with get_value
    "addBoolValueInput": "bool",
    "addSelectionInput": "selection",
    "addDropDownCommandInput": "dropdown",
    "addIntegerSpinnerCommandInput": "int",
    "addFloatSpinnerCommandInput": "value",
    "addTextBoxCommandInput": "text",
}
# framework reader  ->  the family it is valid for
READER_KIND = {
    "get_value": "value",
    "get_boolean": "bool",
    "get_selection": "selection",
}


def _call_name(func):
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def main(path):
    src = open(path).read()
    tree = ast.parse(src)

    # 1. resolve INPUT_ID_* constants to their literal id strings (module + class level)
    const = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Constant) \
                and isinstance(node.value.value, str):
            const[node.targets[0].id] = node.value.value

    def resolve(arg):
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            return arg.value
        if isinstance(arg, ast.Name) and arg.id in const:
            return const[arg.id]
        if isinstance(arg, ast.Name):
            return "<name:%s>" % arg.id   # unresolved constant — still a stable key
        return None

    adds = {}    # id -> (add_method, lineno)
    reads = {}   # id -> list of (reader, lineno)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        name = _call_name(node.func)
        if name in ADD_KIND and node.args:
            iid = resolve(node.args[0])
            if iid is not None:
                adds[iid] = (name, node.lineno)
        elif name in READER_KIND and len(node.args) >= 2:
            iid = resolve(node.args[1])   # get_*(inputs, id, ...) -> id is 2nd arg
            if iid is not None:
                reads.setdefault(iid, []).append((name, node.lineno))

    problems = []
    for iid, rs in reads.items():
        add = adds.get(iid)
        for reader, lineno in rs:
            want = READER_KIND[reader]
            if add is None:
                # read but no add* found (id may be added with an unresolved
                # expression, or declared elsewhere) — informational, not blocking
                continue
            add_method, add_line = add
            add_kind = ADD_KIND[add_method]
            if add_kind != want:
                problems.append(
                    "  id '%s': declared %s (%s, L%d) but read with %s (L%d) "
                    "— expected %s" % (
                        iid, add_kind, add_method, add_line, reader, lineno,
                        {"value": "get_value", "bool": "get_boolean",
                         "selection": "get_selection"}.get(add_kind, "a matching reader"),
                    ))

    if problems:
        print("input-read check: %d MISMATCH(es) — a real bug:" % len(problems))
        for p in problems:
            print(p)
        return 1
    print("input-read check: OK (%d inputs declared, %d read; all helpers match)"
          % (len(adds), len(reads)))
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: check_input_read.py <generated.py>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
