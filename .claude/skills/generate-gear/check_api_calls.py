#!/usr/bin/env python3
"""Gate every method call in a generated gear against the real Fusion API.

Why this exists: a generated file can call a Fusion method that does not exist and pass every
other gate. That is not hypothetical. The second task-list pilot passed parse, pyright,
check_input_read, check_contract AND check_task_calls while calling `addPointOnObject` five
times (the real name is `addCoincident`) and `setExtentDefinition`/`setExtentDirection` on all
three extrudes (the real method is `setOneSideExtent`). Both are immediate runtime
`AttributeError`s.

Nothing else catches them:
  - pyright never mentioned `addPointOnObject`. The intellisense stubs type
    `sketch.geometricConstraints` loosely enough that any method name type-checks.
  - pyright DID flag the extrude pair, but only as REVIEW, and the workflow tells authors to
    ignore REVIEW as stub pessimism — correct advice in general, which is exactly why a real
    bug hiding there goes unseen.
  - check_task_calls.py only asks whether a call the task list NAMES appears somewhere in the
    file. A substitution passes it: `addCoincident` does appear, in a different task.

This check automates [PB-API-LOOKUP] — "look the API up before you write the name" — instead of
trusting that the author did.

What it does: collect every `<something>.name(...)` call in the file, then require each name to
exist in the Fusion API database, to be a method on the target receiver, to be an inherited
framework method, to come from an imported framework module, or to be a plain Python method.
Anything left over is a name that exists nowhere and will fail the moment the add-in runs.

Unverified calls are reported, not waived. The shipped add-in makes three calls the API database
does not back, listed as UNVERIFIED_CALLS in fusion_api.py. This tool prints every one it sees,
on every run, and says which class does declare the name if any does. It does not fail the run,
because nothing has established that Fusion rejects them — that needs a Fusion session, and the
fix then belongs in the spec.

Usage:
    python3 check_api_calls.py <generated.py> [--framework <dir>]

Exit 0 = OK, 1 = BLOCKING, 2 = the API database could not be reached.

The API comes from the `fusion` plugin's compiled database, via fusion_api.py. Nothing is built
or cloned here, so a fresh machine needs only that plugin installed.
"""
import argparse
import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fusion_api  # noqa: E402  (sibling module; sys.path is fixed up just above)

# Plain Python: builtins, str/list/dict methods, and the math functions the gear math uses.
# None of these are Fusion names, so the API database rightly does not carry them.
PYTHON_METHODS = {
    'append', 'extend', 'insert', 'pop', 'remove', 'sort', 'reverse', 'count', 'index',
    'format', 'join', 'split', 'strip', 'lstrip', 'rstrip', 'replace', 'startswith',
    'endswith', 'lower', 'upper', 'items', 'keys', 'values', 'update', 'copy', 'setdefault',
    'acos', 'asin', 'atan', 'atan2', 'cos', 'sin', 'tan', 'sqrt', 'hypot', 'radians',
    'degrees', 'floor', 'ceil', 'fabs', 'pow', 'log', 'exp', 'isclose',
}


def defined_names(paths):
    """Every function/class/method name the given Python files define."""
    names = set()
    for path in paths:
        if not os.path.exists(path):
            continue
        try:
            tree = ast.parse(open(path).read())
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
                names.add(node.name)
    return names


def receiver_tail(func):
    """The last identifier of what a call is made ON: `sketch.sketchTexts.createInput2` -> sketchTexts.

    It is a name, not a type, so it only ever narrows a report — never widens one.
    """
    value = func.value
    if isinstance(value, ast.Attribute):
        return value.attr
    if isinstance(value, ast.Name):
        return value.id
    return None


def framework_files(root):
    out = []
    for base, _, files in os.walk(root):
        for name in files:
            if name.endswith('.py'):
                out.append(os.path.join(base, name))
    return out


def framework_methods(paths):
    """Every method declared directly on a framework class."""
    names = set()
    for path in paths:
        try:
            tree = ast.parse(open(path).read())
        except (OSError, SyntaxError):
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                names.update(child.name for child in node.body
                             if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)))
    return names


def imported_roots(tree):
    """Names that the target imports as modules or classes."""
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                roots.add(alias.asname or alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                roots.add(alias.asname or alias.name)
    return roots


def inherited_framework_methods(target_tree, framework_paths):
    """Methods inherited by target classes from framework classes."""
    framework_classes = {}
    for path in framework_paths:
        if not os.path.exists(path):
            continue
        try:
            tree = ast.parse(open(path).read())
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                framework_classes[node.name] = {
                    child.name for child in node.body
                    if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
                }

    inherited = set()
    for node in ast.walk(target_tree):
        if not isinstance(node, ast.ClassDef):
            continue
        for base in node.bases:
            base_name = base.id if isinstance(base, ast.Name) else base.attr \
                if isinstance(base, ast.Attribute) else None
            inherited.update(framework_classes.get(base_name, set()))
    return inherited


def target_methods(tree):
    """Map each target class to its own methods and methods of target base classes."""
    own = {}
    bases = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        own[node.name] = {
            child.name for child in node.body
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        bases[node.name] = {
            base.id if isinstance(base, ast.Name) else base.attr
            for base in node.bases
            if isinstance(base, (ast.Name, ast.Attribute))
        }

    resolved = {}

    def methods_for(class_name, visiting=()):
        if class_name in resolved:
            return resolved[class_name]
        if class_name in visiting:
            return set()
        methods = set(own.get(class_name, ()))
        for base in bases.get(class_name, ()):
            methods.update(methods_for(base, visiting + (class_name,)))
        resolved[class_name] = methods
        return methods

    return {class_name: methods_for(class_name) for class_name in own}


def target_receiver_types(tree, classes):
    """Infer simple target-instance bindings such as `candidate = Candidate()`."""
    types = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            value = node.value
            target_type = (value.func.id if isinstance(value, ast.Call)
                           and isinstance(value.func, ast.Name)
                           and value.func.id in classes else None)
            if target_type:
                for target in node.targets:
                    if isinstance(target, (ast.Name, ast.Attribute)):
                        types[ast.unparse(target)] = target_type
        elif isinstance(node, ast.AnnAssign):
            annotation = node.annotation
            target_type = (annotation.id if isinstance(annotation, ast.Name)
                           and annotation.id in classes else None)
            if target_type and isinstance(node.target, (ast.Name, ast.Attribute)):
                types[ast.unparse(node.target)] = target_type
    return types


def call_classes(tree):
    """Map calls to the target class whose method contains each call."""
    found = {}

    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.classes = []

        def visit_ClassDef(self, node):
            self.classes.append(node.name)
            self.generic_visit(node)
            self.classes.pop()

        def visit_Call(self, node):
            found[node] = self.classes[-1] if self.classes else None
            self.generic_visit(node)

    Visitor().visit(tree)
    return found


def receiver_root(func):
    """The first identifier in the receiver, such as `futil` in `futil.log(...)`."""
    value = func.value
    while isinstance(value, ast.Attribute):
        value = value.value
    return value.id if isinstance(value, ast.Name) else None


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument('target')
    ap.add_argument('--framework', default='lib',
                    help='directory holding the framework modules (default: lib)')
    args = ap.parse_args()

    src = open(args.target).read()
    tree = ast.parse(src)

    framework_paths = framework_files(args.framework)
    framework_names = defined_names(framework_paths)
    framework_method_names = framework_methods(framework_paths)
    inherited = inherited_framework_methods(tree, framework_paths)
    target_class_methods = target_methods(tree)
    target_types = target_receiver_types(tree, target_class_methods)
    containing_classes = call_classes(tree)
    imported = imported_roots(tree)

    called = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            called.setdefault(node.func.attr, []).append((
                node.lineno, receiver_tail(node.func), node, containing_classes[node]))

    def allowed(name, func, containing_class):
        if name in PYTHON_METHODS:
            return True
        receiver = func.value
        receiver_type = None
        if isinstance(receiver, ast.Name) and receiver.id == 'self':
            receiver_type = containing_class
        elif isinstance(receiver, ast.Name) and receiver.id in target_class_methods:
            receiver_type = receiver.id
        elif isinstance(receiver, (ast.Name, ast.Attribute)):
            receiver_type = target_types.get(ast.unparse(receiver))
        if name in target_class_methods.get(receiver_type, ()):
            return True
        # A target object may pass a framework-backed object through an attribute, as in
        # `self.parent.getParameter()`. Keep that existing framework allowance scoped to self.
        if receiver_root(func) == 'self' and name in framework_method_names:
            return True
        if receiver_tail(func) == 'self' and name in inherited:
            return True
        return receiver_root(func) in imported and name in framework_names

    candidates = sorted(
        name for name in called
        if not name.startswith('_')
        and any(not allowed(node.func.attr, node.func, containing_classes[node]) for node in ast.walk(tree)
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == name))

    # Where each watchlist call is actually made, receiver and all, so a legitimate namesake on
    # another class is not dragged into the report.
    seen = {}
    for name, _, receivers, _ in fusion_api.UNVERIFIED_CALLS:
        lines = sorted(line for line, tail, _, _ in called.get(name, [])
                       if fusion_api.receiver_matches(receivers, tail))
        if lines:
            seen[name] = '%s:%s' % (args.target, ','.join(str(line) for line in lines))

    try:
        hits = fusion_api.lookup_many(candidates)
        findings = fusion_api.unverified_findings(seen)
    except fusion_api.Unavailable as exc:
        print('check_api_calls: %s' % exc, file=sys.stderr)
        return 2

    # A call already reported as unverified is not also reported as unknown; the finding below
    # says more about it than "no such name" ever could.
    watched = set(seen)
    unknown = {name: called[name][0][0] for name in candidates
               if not hits[name] and name not in watched}

    if unknown:
        print('api-call check: BLOCKING (%d)' % len(unknown))
        for name, lineno in sorted(unknown.items(), key=lambda kv: kv[1]):
            near = fusion_api.similar(name)
            print("  %s:%d calls '%s(' — no such name in the Fusion API database, the framework, "
                  "or this file%s"
                  % (args.target, lineno, name,
                     '' if not near else
                     '; the nearest names the database has are %s' % ', '.join(near)))
        print('  Ask the database what a class offers: '
              'python3 %s members <Class>' % fusion_api.query_script())
        return 1

    print('api-call check: OK (%d call names resolve)' % len(candidates))
    if findings:
        print('api-call check: %d UNVERIFIED call(s) — reported, not blocking, not waived'
              % len(findings))
        for line in findings:
            print('  %s' % line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
