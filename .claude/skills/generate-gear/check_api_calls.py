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

# Only these modules are shared by generated gear files.  Gear implementations are
# deliberately excluded: a method on one gear must not become an API allowance for
# an unrelated receiver in another generated file.
SHARED_FRAMEWORK_MODULES = (
    'geargen/base.py',
    'geargen/misc.py',
    'geargen/utilities.py',
    'geargen/solids.py',
    'geargen/spurproxy.py',
    'fusion360utils/__init__.py',
    'fusion360utils/event_utils.py',
    'fusion360utils/general_utils.py',
)


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
    for relative in SHARED_FRAMEWORK_MODULES:
        path = os.path.join(root, relative)
        if not os.path.exists(path):
            # Permit --framework to point directly at either shared package.  This
            # keeps the checker useful with a small hermetic fixture as well as lib/.
            path = os.path.join(root, os.path.basename(relative))
        if os.path.exists(path):
            out.append(path)
    return out


def framework_methods(paths):
    """Map each shared framework class to its directly declared methods."""
    methods = {}
    for path in paths:
        try:
            tree = ast.parse(open(path).read())
        except (OSError, SyntaxError):
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                methods.setdefault(node.name, set()).update(
                    child.name for child in node.body
                    if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)))
    return methods


def framework_exports(paths):
    """Map each shared module name to its top-level exported names."""
    exports = {}
    for path in paths:
        try:
            tree = ast.parse(open(path).read())
        except (OSError, SyntaxError):
            continue
        filename = os.path.basename(path)
        module = os.path.splitext(filename)[0]
        if module == '__init__':
            module = os.path.basename(os.path.dirname(path))
        names = {
            node.name for node in tree.body
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        }
        exports.setdefault(module, set()).update(names)
        if os.path.basename(os.path.dirname(path)) == 'fusion360utils':
            # The package re-exports the two utility modules with star imports.
            exports.setdefault('fusion360utils', set()).update(names)
    return exports


def imported_framework_modules(tree, exports):
    """Map imported aliases to verified shared framework modules."""
    modules = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                module = alias.name.rsplit('.', 1)[-1]
                if module in exports:
                    modules[alias.asname or alias.name.split('.')[0]] = module
        elif isinstance(node, ast.ImportFrom):
            for alias in node.names:
                module = alias.name
                if module in exports:
                    modules[alias.asname or alias.name] = module
    return modules


def imported_class_methods(tree, root, inherited_methods):
    """Resolve methods only for classes explicitly imported by the target."""
    imported_paths = []

    def module_path(module):
        candidates = (
            os.path.join(root, 'geargen', module + '.py'),
            os.path.join(root, module + '.py'),
        )
        return next((path for path in candidates if os.path.exists(path)), None)

    def add_imports(source):
        for node in ast.walk(source):
            if not isinstance(node, ast.ImportFrom):
                continue
            module = node.module.rsplit('.', 1)[-1] if node.module else None
            if module:
                path = module_path(module)
                if path is not None:
                    imported_paths.append(path)
            else:
                imported_paths.extend(
                    path for path in (module_path(alias.name) for alias in node.names)
                    if path is not None)

    for node in ast.walk(tree):
        if not isinstance(node, ast.ImportFrom):
            continue
        module = node.module.rsplit('.', 1)[-1] if node.module else None
        if module:
            path = module_path(module)
            if path is not None:
                imported_paths.append(path)
        else:
            imported_paths.extend(
                path for path in (module_path(alias.name) for alias in node.names)
                if path is not None)

    own = {}
    bases = {}
    seen_paths = set()
    while imported_paths:
        path = imported_paths.pop()
        if path in seen_paths:
            continue
        seen_paths.add(path)
        try:
            source = ast.parse(open(path).read())
        except (OSError, SyntaxError):
            continue
        add_imports(source)
        for class_node in source.body:
            if not isinstance(class_node, ast.ClassDef):
                continue
            own[class_node.name] = {
                child.name for child in class_node.body
                if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
            }
            bases[class_node.name] = {
                base.id if isinstance(base, ast.Name) else base.attr
                for base in class_node.bases
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
            methods.update(inherited_methods.get(base, ()))
        resolved[class_name] = methods
        return methods

    return {class_name: methods_for(class_name) for class_name in own}


def target_methods(tree, framework_class_methods=None):
    """Map each target class to its own methods and methods of target base classes."""
    framework_class_methods = framework_class_methods or {}
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
            methods.update(own.get(base, ()))
            methods.update(framework_class_methods.get(base, ()))
            if base in own:
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


def constructor_field_parameters(tree):
    """Map (class, field) to the constructor parameter assigned to that field."""
    fields = {}
    parameters = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        init = next((child for child in node.body
                     if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef))
                     and child.name == '__init__'), None)
        if init is None:
            continue
        args = [arg.arg for arg in init.args.args[1:]]
        parameters[node.name] = args
        for statement in ast.walk(init):
            if not isinstance(statement, ast.Assign):
                continue
            if not isinstance(statement.value, ast.Name):
                continue
            for target in statement.targets:
                if (isinstance(target, ast.Attribute)
                        and isinstance(target.value, ast.Name)
                        and target.value.id == 'self'
                        and statement.value.id in args):
                    fields[(node.name, target.attr)] = statement.value.id
    return fields, parameters


def inferred_field_types(tree, classes, receiver_types, containing_classes):
    """Infer field types from constructor calls with already verified receivers."""
    fields, parameters = constructor_field_parameters(tree)
    inferred = {}

    def expression_type(expression, containing_class):
        if isinstance(expression, ast.Name) and expression.id == 'self':
            return containing_class
        if isinstance(expression, (ast.Name, ast.Attribute)):
            return receiver_types.get(ast.unparse(expression))
        if (isinstance(expression, ast.Call)
                and isinstance(expression.func, ast.Name)
                and expression.func.id in classes):
            return expression.func.id
        return None

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        class_name = node.func.id
        if class_name not in parameters:
            continue
        containing_class = containing_classes.get(node)
        for argument, parameter in zip(node.args, parameters[class_name]):
            argument_type = expression_type(argument, containing_class)
            if argument_type is None:
                continue
            for (field_class, field), field_parameter in fields.items():
                if field_class == class_name and field_parameter == parameter:
                    inferred.setdefault((field_class, field), set()).add(argument_type)
    return inferred


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
    framework_class_methods = framework_methods(framework_paths)
    framework_module_exports = framework_exports(framework_paths)
    imported_methods = imported_class_methods(
        tree, args.framework, framework_class_methods)
    inherited_methods = dict(framework_class_methods)
    inherited_methods.update(imported_methods)
    target_class_methods = target_methods(tree, inherited_methods)
    known_class_methods = dict(inherited_methods)
    known_class_methods.update(target_class_methods)
    target_types = target_receiver_types(tree, known_class_methods)
    containing_classes = call_classes(tree)
    framework_modules = imported_framework_modules(tree, framework_module_exports)
    field_types = inferred_field_types(
        tree, known_class_methods, target_types, containing_classes)

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
        elif (isinstance(receiver, ast.Call)
              and isinstance(receiver.func, ast.Name)
              and receiver.func.id == 'super'):
            return name in known_class_methods.get(containing_class, ())
        elif (isinstance(receiver, ast.Call)
              and isinstance(receiver.func, ast.Name)):
            receiver_type = receiver.func.id
        elif isinstance(receiver, (ast.Name, ast.Attribute)):
            receiver_type = target_types.get(ast.unparse(receiver))
        if (isinstance(receiver, ast.Attribute)
                and isinstance(receiver.value, ast.Name)
                and receiver.value.id == 'self'):
            receiver_type = next(iter(field_types.get(
                (containing_class, receiver.attr), ())), None)
        if name in known_class_methods.get(receiver_type, ()):
            return True
        module = framework_modules.get(receiver_root(func))
        return module is not None and name in framework_module_exports[module]

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
