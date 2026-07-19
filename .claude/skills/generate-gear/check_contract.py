#!/usr/bin/env python3
"""Mechanical contract check for a generated gear module.

The generate-gear skill's validation step requires a "contract self-check":
every class, hook method, ctx field, module constant, input id, and parameter
string the spec's Contract sections declare must exist in the generated file.
Until now that check was prose-matching by an agent — which is exactly how a
renamed `PARAM_*` export (imported by a dependent gear) can slip through every
other gate. This script makes it mechanical against a machine-readable
manifest `spec/<gear>/contract.json` that mirrors the spec's contract
sections. The spec PROSE stays authoritative: a manifest/spec mismatch is a
spec bug — fix both together.

Manifest shape (all sections optional):
  {
    "module": "lib/geargen/spurgear.py",
    "module_constants": {"PARAM_MODULE": "Module", ...},   // name -> exact string value
    "classes": {
      "SpurGearGenerator": {
        "bases": ["Generator"],          // base names, matched by unqualified name
        "methods": ["generate", ...],    // must be defined in the class body
        "ctx_fields": ["plane", ...]     // self.<field> assigned in __init__
      }, ...
    }
  }

Also checked, manifest-free (they need only the repo):
  * helper shadowing — the generated module must not re-define (def/class) a
    name provided by the framework helper library (top-level names of
    lib/geargen/{utilities,solids,spurproxy}.py);
  * import resolution — every `from .mod import name` against lib/geargen/
    must name something that exists top-level in that module;
  * no `import *` in a gear module (playbook module-layout rule).

Usage:  python3 check_contract.py <contract.json> <generated.py> [repo-root]
Exit 0 = clean; exit 1 = at least one BLOCKING contract violation.
"""
import ast
import json
import os
import sys

HELPER_MODULES = ("utilities", "solids", "spurproxy")


def _top_level_names(tree):
    names = set()
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            names.add(node.name)
        elif isinstance(node, ast.Assign):
            for t in node.targets:
                if isinstance(t, ast.Name):
                    names.add(t.id)
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.add(node.target.id)
    return names


def _base_name(node):
    if isinstance(node, ast.Attribute):
        return node.attr
    if isinstance(node, ast.Name):
        return node.id
    return ast.unparse(node)


def _ctx_fields(classnode):
    fields = []
    for node in classnode.body:
        if isinstance(node, ast.FunctionDef) and node.name == "__init__":
            for sub in ast.walk(node):
                if isinstance(sub, ast.Assign):
                    for t in sub.targets:
                        if isinstance(t, ast.Attribute) and \
                                isinstance(t.value, ast.Name) and t.value.id == "self":
                            fields.append(t.attr)
    return fields


def main(manifest_path, gen_path, root="."):
    manifest = json.load(open(manifest_path, encoding="utf-8"))
    tree = ast.parse(open(gen_path, encoding="utf-8").read())
    problems = []

    # --- module-level constants: identifier AND exact string value ---
    consts = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name) \
                and isinstance(node.value, ast.Constant):
            consts[node.targets[0].id] = node.value.value
    for name, want in manifest.get("module_constants", {}).items():
        if name not in consts:
            problems.append("  constant %s missing (dependents may import it)" % name)
        elif consts[name] != want:
            problems.append("  constant %s = %r, manifest says %r" % (name, consts[name], want))

    # --- classes: existence, bases, methods, ctx fields ---
    classes = {n.name: n for n in tree.body if isinstance(n, ast.ClassDef)}
    for cname, decl in manifest.get("classes", {}).items():
        cls = classes.get(cname)
        if cls is None:
            problems.append("  class %s missing" % cname)
            continue
        have_bases = [_base_name(b) for b in cls.bases]
        for base in decl.get("bases", []):
            if base not in have_bases:
                problems.append("  class %s: base %s missing (has %s)" % (cname, base, have_bases))
        have_methods = {m.name for m in cls.body
                        if isinstance(m, (ast.FunctionDef, ast.AsyncFunctionDef))}
        for meth in decl.get("methods", []):
            if meth not in have_methods:
                problems.append("  class %s: method %s missing" % (cname, meth))
        want_fields = decl.get("ctx_fields", [])
        if want_fields:
            have_fields = set(_ctx_fields(cls))
            for f in want_fields:
                if f not in have_fields:
                    problems.append("  class %s: ctx field self.%s not assigned in __init__" % (cname, f))

    # --- helper shadowing (manifest-free) ---
    helper_names = {}
    for mod in HELPER_MODULES:
        path = os.path.join(root, "lib", "geargen", mod + ".py")
        if os.path.isfile(path):
            for n in _top_level_names(ast.parse(open(path, encoding="utf-8").read())):
                if not n.startswith("_"):
                    helper_names.setdefault(n, mod)
    own_defs = {n.name: n.lineno for n in tree.body
                if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))}
    for name, lineno in sorted(own_defs.items(), key=lambda kv: kv[1]):
        if name in helper_names:
            problems.append(
                "  L%d: re-defines framework helper %s (lives in lib/geargen/%s.py — call it instead)"
                % (lineno, name, helper_names[name]))

    # --- import resolution + star-import ban (manifest-free) ---
    for node in tree.body:
        if isinstance(node, ast.ImportFrom):
            if any(a.name == "*" for a in node.names):
                problems.append("  L%d: `from %s import *` — gear modules never star-import"
                                % (node.lineno, node.module or ""))
                continue
            if node.level >= 1 and node.module:
                path = os.path.join(root, "lib", "geargen", node.module.split(".")[0] + ".py")
                if os.path.isfile(path):
                    avail = _top_level_names(ast.parse(open(path, encoding="utf-8").read()))
                    for a in node.names:
                        if a.name not in avail:
                            problems.append("  L%d: `from .%s import %s` — name not defined there"
                                            % (node.lineno, node.module, a.name))

    if problems:
        print("contract check: %d BLOCKING violation(s) vs %s:"
              % (len(problems), os.path.relpath(manifest_path, root)))
        for p in problems:
            print(p)
        return 1
    print("contract check: OK (%d constants, %d classes declared; shadowing/imports clean)"
          % (len(manifest.get("module_constants", {})), len(manifest.get("classes", {}))))
    return 0


if __name__ == "__main__":
    if len(sys.argv) not in (3, 4):
        print("usage: check_contract.py <contract.json> <generated.py> [repo-root]", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(*sys.argv[1:]))
