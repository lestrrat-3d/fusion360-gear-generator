#!/usr/bin/env python3
"""Build a grep-able, class-scoped index of the Fusion Python API from its stub defs.

Why this exists: when generating gear code, the two BLOCKING bug classes are (1) undefined
names / wrong casing (`addColinear` vs `addCollinear`) and (2) wrong adsk submodule
(`adsk.fusion.SurfaceTypes` — it's in `adsk.core`). pyright_check.py catches these AFTER the
fact. This index shifts the check LEFT: before writing an `adsk.*` call, grep the index to
confirm the name exists, on which class, in which submodule, with what signature.

Class-scoping is the point. A flat grep of `def sketchCircles` over the raw stubs hits
`SketchCurves.sketchCircles` and would wrongly imply `Sketch.sketchCircles` exists (it does
not — `AttributeError`). The index records each member's OWNING class, so that lookup is
unambiguous.

Output: one JSON object per line (JSONL), optimized for Grep. One record per class, method,
property, and enum member, each carrying:
    name   — the identifier (method/property/enum-member/class name)
    class  — the owning class (for a class record, its own name, so grepping
             "class":"ValueInput" returns the class AND every member on it)
    module — adsk.core / adsk.fusion / ...
    kind   — class | method | staticmethod | property | enum-member
    sig    — reconstructed signature (params + types + return), or the enum int value
    doc    — first line of the docstring (one-line summary; "" if none)

This serves two query needs:
  - verification (point lookup): does X exist? on what class? which module? exact signature?
  - concept discovery: grep "chamfer"/"revolve"/"split" matches names AND the `doc` summary.
For the deep-prose tail (units notes, "returns null if ...", concept synonyms), fall back to
grepping the raw defs (core.py / fusion.py) directly — they are already cloned.

Storage is JSONL, not SQLite, deliberately: the consumer is an agent whose cheapest lookup is
the Grep tool, the corpus is small (~thousands of records, sub-MB), and JSONL stays readable,
diffable, and trivially rebuilt. The index is a DERIVED artifact — written to the cache next
to the clone by default, never committed.

Usage:
    python3 build_fusion_index.py [--stubs <dir>] [--out <path>] [--modules core,fusion]

Stubs are resolved/cloned-on-demand via fusion_stubs.resolve_defs (shared with
pyright_check.py). Default --out is <cache>/fusion-api-index.jsonl next to the clone.

Exit codes:
    0  index written
    2  setup error (stubs unresolvable / clone failed)
"""
import argparse
import ast
import json
import os
import sys
import warnings

# Sibling import: when run as a script, sys.path[0] is this file's dir.
from fusion_stubs import cache_repo_dir, resolve_defs, StubsUnavailable

# adsk submodules to index. The gear generators only ever touch core + fusion; keeping the
# index to those two keeps every `module` value meaningful for the gear domain. Extend here
# if a gear ever needs cam/drawing/sim/volume.
DEFAULT_MODULES = ["core", "fusion"]


def _render_arg(a):
    """Render one ast.arg as `name` or `name: Annotation`."""
    if a.annotation is not None:
        return f"{a.arg}: {ast.unparse(a.annotation)}"
    return a.arg


def _render_signature(fn):
    """Reconstruct a readable signature string from a FunctionDef node, dropping `self`.
    Covers positional, *args, keyword-only, **kwargs, and defaults — enough for the stubs."""
    a = fn.args
    parts = []

    posonly = list(a.posonlyargs)
    pos = list(a.args)
    # Defaults align to the tail of posonly + pos.
    all_pos = posonly + pos
    ndef = len(a.defaults)
    default_for = {}
    if ndef:
        for i, d in enumerate(a.defaults):
            default_for[len(all_pos) - ndef + i] = d

    for idx, arg in enumerate(all_pos):
        if arg.arg == "self":
            continue
        s = _render_arg(arg)
        if idx in default_for:
            s += f"={ast.unparse(default_for[idx])}"
        parts.append(s)
        if posonly and idx == len(posonly) - 1:
            parts.append("/")

    if a.vararg is not None:
        parts.append("*" + _render_arg(a.vararg))
    elif a.kwonlyargs:
        parts.append("*")
    for arg, dflt in zip(a.kwonlyargs, a.kw_defaults):
        s = _render_arg(arg)
        if dflt is not None:
            s += f"={ast.unparse(dflt)}"
        parts.append(s)
    if a.kwarg is not None:
        parts.append("**" + _render_arg(a.kwarg))

    sig = f"{fn.name}({', '.join(parts)})"
    if fn.returns is not None:
        sig += f" -> {ast.unparse(fn.returns)}"
    return sig


def _docline(node):
    """First non-empty line of the node's docstring, or ''."""
    doc = ast.get_docstring(node)
    if not doc:
        return ""
    for line in doc.splitlines():
        line = line.strip()
        if line:
            return line
    return ""


def _decorators(fn):
    return {d.id for d in fn.decorator_list if isinstance(d, ast.Name)}


def _enum_value(node):
    """If a class-body Assign/AnnAssign is a simple `NAME = <const>` enum member, return
    (name, rendered_value); else None. Stubs write enum members as `Member = 0`."""
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
        return node.targets[0].id, ast.unparse(node.value)
    if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name) and node.value is not None:
        return node.target.id, ast.unparse(node.value)
    return None


def index_module(module_label, path, out):
    """Parse one stub module and write its records to `out`. Returns count written."""
    with open(path, "r", encoding="utf-8") as fh:
        src = fh.read()
    # The stub docstrings contain raw backslash sequences (e.g. \S, \A) that trigger
    # cosmetic SyntaxWarnings on parse; they don't affect the AST, so suppress them.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", SyntaxWarning)
        tree = ast.parse(src, filename=path)

    n = 0

    def emit(rec):
        nonlocal n
        out.write(json.dumps(rec, separators=(",", ":"), ensure_ascii=False) + "\n")
        n += 1

    for cls in tree.body:
        if not isinstance(cls, ast.ClassDef):
            continue
        bases = ", ".join(ast.unparse(b) for b in cls.bases)
        emit({
            "name": cls.name,
            "class": cls.name,  # self-owner, so grepping "class":"X" returns class + members
            "module": module_label,
            "kind": "class",
            "sig": f"{cls.name}({bases})" if bases else cls.name,
            "doc": _docline(cls),
        })
        for member in cls.body:
            if isinstance(member, ast.FunctionDef):
                if member.name == "__init__":
                    continue  # stub __init__ is always an empty `pass`, no signal
                decs = _decorators(member)
                if "property" in decs:
                    kind = "property"
                elif "staticmethod" in decs:
                    kind = "staticmethod"
                else:
                    kind = "method"
                emit({
                    "name": member.name,
                    "class": cls.name,
                    "module": module_label,
                    "kind": kind,
                    "sig": _render_signature(member),
                    "doc": _docline(member),
                })
            else:
                ev = _enum_value(member)
                if ev is not None:
                    name, value = ev
                    emit({
                        "name": name,
                        "class": cls.name,
                        "module": module_label,
                        "kind": "enum-member",
                        "sig": f"{name} = {value}",
                        "doc": "",
                    })
    return n


def main():
    ap = argparse.ArgumentParser(description="Build the grep-able Fusion API index (JSONL).")
    ap.add_argument("--stubs", help="defs dir or FusionAPIReference checkout (else $FUSION_API_STUBS, else clone)")
    ap.add_argument("--out", help="output JSONL path (default: <cache>/fusion-api-index.jsonl)")
    ap.add_argument("--modules", default=",".join(DEFAULT_MODULES),
                    help="comma-separated adsk submodules to index (default: core,fusion)")
    args = ap.parse_args()

    try:
        defs = resolve_defs(args.stubs)
    except StubsUnavailable as e:
        print(f"ERROR: {e}")
        sys.exit(2)

    out_path = args.out or os.path.join(os.path.dirname(cache_repo_dir()), "fusion-api-index.jsonl")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    modules = [m.strip() for m in args.modules.split(",") if m.strip()]
    total = 0
    per_module = {}
    with open(out_path, "w", encoding="utf-8") as out:
        for m in modules:
            src = os.path.join(defs, "adsk", f"{m}.py")
            if not os.path.isfile(src):
                print(f"WARNING: {src} not found; skipping module '{m}'")
                continue
            c = index_module(f"adsk.{m}", src, out)
            per_module[m] = c
            total += c

    summary = ", ".join(f"{m}:{c}" for m, c in per_module.items())
    print(f"wrote {total} records ({summary}) -> {out_path}")
    sys.exit(0)


if __name__ == "__main__":
    main()
