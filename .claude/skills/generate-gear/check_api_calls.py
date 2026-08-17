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
import re
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

UNVERIFIED_RETURN_TYPES = {
    ('project', 'sketch'): 'ObjectCollection',
    ('project', 'toolsSketch'): 'ObjectCollection',
    ('createInput2', 'sketchTexts'): 'SketchTextInput',
}

IMPLIED_MEMBER_RETURNS = {
    ('Curve3D', 'startPoint'): 'Point3D',
    ('Curve3D', 'endPoint'): 'Point3D',
}


def read_source(path):
    with open(path) as fh:
        return fh.read()


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


def receiver_expression(func):
    """Return the complete expression the call is made on."""
    return ast.unparse(func.value)


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
            tree = ast.parse(read_source(path))
        except (OSError, SyntaxError):
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                methods.setdefault(node.name, set()).update(
                    child.name for child in node.body
                    if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)))
    return methods


def class_bases_from_tree(tree):
    bases = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.ClassDef):
            continue
        bases[node.name] = {
            base.id if isinstance(base, ast.Name) else base.attr
            for base in node.bases
            if isinstance(base, (ast.Name, ast.Attribute))
        }
    return bases


def framework_bases(paths):
    bases = {}
    for path in paths:
        try:
            tree = ast.parse(read_source(path))
        except (OSError, SyntaxError):
            continue
        bases.update(class_bases_from_tree(tree))
    return bases


def normalize_api_type(name):
    if not name:
        return None
    name = str(name).strip().strip('"\'')
    name = re.sub(r'\s*\|\s*None$', '', name)
    optional = re.match(r'(?:typing\.)?Optional\[(.+)\]$', name)
    if optional:
        name = optional.group(1)
    if name.startswith(('adsk.core.', 'adsk.fusion.')):
        return name.rsplit('.', 1)[-1]
    if name.startswith(('core.', 'fusion.')):
        return name.rsplit('.', 1)[-1]
    if re.match(r'^[A-Z][A-Za-z0-9_]*(?:\[[^]]+\])?$', name):
        return name.split('[', 1)[0]
    return None


def annotation_return_type(annotation):
    if annotation is None:
        return None
    return normalize_api_type(ast.unparse(annotation))


def fusion_annotation_type(annotation):
    """Return a type only when an annotation names a qualified Fusion class."""
    if annotation is None:
        return None
    text = ast.unparse(annotation).strip()
    if not text.startswith(('adsk.core.', 'adsk.fusion.', 'core.', 'fusion.')):
        optional = re.match(r'(?:typing\.)?Optional\[(.+)\]$', text)
        if optional is None:
            return None
        text = optional.group(1).strip()
    return normalize_api_type(text)


def method_returns_from_tree(tree, known_classes):
    returns = {}
    for class_node in ast.walk(tree):
        if not isinstance(class_node, ast.ClassDef):
            continue
        for child in class_node.body:
            if not isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            ret = annotation_return_type(child.returns)
            if ret is None:
                for statement in child.body:
                    if not isinstance(statement, ast.Return):
                        continue
                    value = statement.value
                    if (isinstance(value, ast.Call)
                            and isinstance(value.func, ast.Name)
                            and value.func.id in known_classes):
                        ret = value.func.id
                        break
            if ret is not None:
                returns[(class_node.name, child.name)] = ret
    return returns


def method_parameters_from_tree(tree):
    parameters = {}
    for class_node in ast.walk(tree):
        if not isinstance(class_node, ast.ClassDef):
            continue
        for child in class_node.body:
            if not isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                continue
            parameters[(class_node.name, child.name)] = [
                arg.arg for arg in child.args.args[1:]]
    return parameters


def framework_method_returns(paths, known_classes):
    returns = {}
    for path in paths:
        try:
            tree = ast.parse(read_source(path))
        except (OSError, SyntaxError):
            continue
        returns.update(method_returns_from_tree(tree, known_classes))
    # `createSketchObject` is part of the shared generator surface, but it has no annotation in
    # the shipped framework. Its body returns the result of `Component.sketches.add(...)`.
    if ('Generator', 'createSketchObject') in {
            (class_name, method)
            for class_name, methods in framework_methods(paths).items()
            for method in methods}:
        returns.setdefault(('Generator', 'createSketchObject'), 'Sketch')
    return returns


def attr_chain(expr):
    parts = []
    while isinstance(expr, ast.Attribute):
        parts.append(expr.attr)
        expr = expr.value
    if isinstance(expr, ast.Name):
        parts.append(expr.id)
        return list(reversed(parts))
    return None


def fusion_class_expr(expr):
    parts = attr_chain(expr)
    if parts and len(parts) >= 3 and parts[0] == 'adsk' and parts[1] in ('core', 'fusion'):
        return parts[2]
    return None


def unverified_return_type(func):
    if not isinstance(func, ast.Attribute):
        return None
    return UNVERIFIED_RETURN_TYPES.get((func.attr, receiver_tail(func)))


def implied_member_return(owner_type, name, expression=None):
    if (owner_type, name) in IMPLIED_MEMBER_RETURNS:
        return IMPLIED_MEMBER_RETURNS[(owner_type, name)]
    if (name == 'geometry'
            and isinstance(expression, ast.Attribute)
            and isinstance(expression.value, ast.Attribute)
            and expression.value.attr == 'referencePlane'):
        return 'Plane'
    return None


def framework_exports(paths):
    """Map each shared module name to its top-level exported names."""
    exports = {}
    for path in paths:
        try:
            tree = ast.parse(read_source(path))
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
            source = ast.parse(read_source(path))
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


def node_classes(tree):
    """Map every visited node to the class whose body contains it."""
    found = {}

    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.classes = []

        def generic_visit(self, node):
            found[node] = self.classes[-1] if self.classes else None
            super().generic_visit(node)

        def visit_ClassDef(self, node):
            found[node] = self.classes[-1] if self.classes else None
            self.classes.append(node.name)
            for child in node.body:
                self.visit(child)
            self.classes.pop()

    Visitor().visit(tree)
    return found


def node_scopes(tree):
    """Map nodes to their containing class/function pair."""
    found = {}

    class Visitor(ast.NodeVisitor):
        def __init__(self):
            self.classes = []
            self.functions = []

        def generic_visit(self, node):
            found[node] = (
                self.classes[-1] if self.classes else None,
                self.functions[-1] if self.functions else None,
            )
            super().generic_visit(node)

        def visit_ClassDef(self, node):
            found[node] = (
                self.classes[-1] if self.classes else None,
                self.functions[-1] if self.functions else None,
            )
            self.classes.append(node.name)
            for child in node.body:
                self.visit(child)
            self.classes.pop()

        def visit_FunctionDef(self, node):
            found[node] = (
                self.classes[-1] if self.classes else None,
                self.functions[-1] if self.functions else None,
            )
            self.functions.append(node.name)
            for child in node.body:
                self.visit(child)
            self.functions.pop()

        visit_AsyncFunctionDef = visit_FunctionDef

    Visitor().visit(tree)
    return found


def receiver_root(func):
    """The first identifier in the receiver, such as `futil` in `futil.log(...)`."""
    value = func.value
    while isinstance(value, ast.Attribute):
        value = value.value
    return value.id if isinstance(value, ast.Name) else None


def infer_api_receiver_types(tree, classes, bases, method_returns, api_member_info):
    """Infer receiver types used to check Fusion calls, staying within simple AST facts."""
    containing = node_classes(tree)
    scopes = node_scopes(tree)
    types = {}
    field_types = {}
    verified_bindings = set()
    verified_fields = set()
    constructor_fields, constructor_parameters = constructor_field_parameters(tree)
    method_parameters = method_parameters_from_tree(tree)

    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        scope = (containing.get(node), node.name)
        arguments = (*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs)
        for argument in arguments:
            argument_type = fusion_annotation_type(argument.annotation)
            if argument_type is not None:
                key = (scope, argument.arg)
                types[key] = argument_type
                verified_bindings.add(key)

    def method_return_for(class_name, name, visiting=()):
        if class_name is None or class_name in visiting:
            return None
        ret = method_returns.get((class_name, name))
        if ret is not None:
            return ret
        for base in bases.get(class_name, ()):
            ret = method_return_for(base, name, visiting + (class_name,))
            if ret is not None:
                return ret
        return None

    def method_parameters_for(class_name, name, visiting=()):
        if class_name is None or class_name in visiting:
            return None
        params = method_parameters.get((class_name, name))
        if params is not None:
            return class_name, params
        for base in bases.get(class_name, ()):
            found = method_parameters_for(base, name, visiting + (class_name,))
            if found is not None:
                return found
        return None

    def expression_type(expression, containing_class):
        if expression is None:
            return None
        direct_class = fusion_class_expr(expression)
        if direct_class is not None:
            return direct_class
        if isinstance(expression, ast.Name):
            if expression.id == 'self':
                return containing_class
            return (types.get((scopes.get(expression), expression.id))
                    or types.get(expression.id))
        if isinstance(expression, ast.Attribute):
            text = ast.unparse(expression)
            scoped = (scopes.get(expression), text)
            if scoped in types:
                return types[scoped]
            if (isinstance(expression.value, ast.Name)
                    and expression.value.id == 'self'):
                known = field_types.get((containing_class, expression.attr))
                if known:
                    return next(iter(known))
            owner_type = expression_type(expression.value, containing_class)
            if owner_type is None:
                return None
            known_field = field_types.get((owner_type, expression.attr))
            if known_field:
                return next(iter(known_field))
            info = api_member_info(owner_type, expression.attr)
            if info:
                return normalize_api_type(info.get('returns'))
            return implied_member_return(owner_type, expression.attr, expression)
        if isinstance(expression, ast.Call):
            if isinstance(expression.func, ast.Name):
                if expression.func.id in classes:
                    return expression.func.id
                return None
            if isinstance(expression.func, ast.Attribute):
                owner_type = expression_type(expression.func.value, containing_class)
                if owner_type is None:
                    return None
                ret = method_return_for(owner_type, expression.func.attr)
                if ret is not None:
                    return ret
                info = api_member_info(owner_type, expression.func.attr)
                if info:
                    return normalize_api_type(info.get('returns'))
                implied = implied_member_return(owner_type, expression.func.attr)
                if implied is not None:
                    return implied
                return unverified_return_type(expression.func)
        return None

    def expression_is_verified(expression, containing_class):
        if expression is None:
            return False
        if fusion_class_expr(expression) is not None:
            return True
        if isinstance(expression, ast.Name):
            return (scopes.get(expression), expression.id) in verified_bindings
        if isinstance(expression, ast.Attribute):
            key = (scopes.get(expression), ast.unparse(expression))
            if key in verified_bindings:
                return True
            if (isinstance(expression.value, ast.Name)
                    and expression.value.id == 'self'
                    and (containing_class, expression.attr) in verified_fields):
                return True
            owner = expression_type(expression.value, containing_class)
            if not expression_is_verified(expression.value, containing_class):
                return False
            info = api_member_info(owner, expression.attr)
            return info is not None and info.get('returns') is not None
        if isinstance(expression, ast.Call) and isinstance(expression.func, ast.Attribute):
            owner = expression_type(expression.func.value, containing_class)
            owner_verified = expression_is_verified(expression.func.value, containing_class)
            if owner_verified:
                return True
            if method_return_for(owner, expression.func.attr) is not None:
                return True
            info = api_member_info(owner, expression.func.attr)
            return info is not None and info.get('returns') is not None
        return False

    def assign_type(target, assigned, verified=False, preserve_unknown_field=False):
        nonlocal field_types
        changed = False
        if isinstance(target, (ast.Name, ast.Attribute)):
            name = target.id if isinstance(target, ast.Name) else ast.unparse(target)
            key = (scopes.get(target), name)
            if assigned is None:
                if key in types:
                    del types[key]
                    changed = True
                if key in verified_bindings:
                    verified_bindings.remove(key)
                    changed = True
            elif types.get(key) != assigned:
                types[key] = assigned
                changed = True
                if verified:
                    verified_bindings.add(key)
                else:
                    verified_bindings.discard(key)
            elif verified and key not in verified_bindings:
                verified_bindings.add(key)
                changed = True
        if isinstance(target, ast.Attribute):
            owner = expression_type(target.value, containing.get(target))
            if owner is not None:
                field_key = (owner, target.attr)
                values = field_types.get(field_key, set())
                if assigned is None and not preserve_unknown_field:
                    if field_key in field_types:
                        del field_types[field_key]
                        changed = True
                    if field_key in verified_fields:
                        verified_fields.remove(field_key)
                        changed = True
                elif assigned is not None and values != {assigned}:
                    field_types[field_key] = {assigned}
                    changed = True
                    if verified:
                        verified_fields.add(field_key)
                    else:
                        verified_fields.discard(field_key)
                elif verified and field_key not in verified_fields:
                    verified_fields.add(field_key)
                    changed = True
        return changed

    for node in ast.walk(tree):
        if isinstance(node, ast.AnnAssign):
            assigned = annotation_return_type(node.annotation)
            assign_type(node.target, assigned, fusion_annotation_type(node.annotation) is not None)

    changed = True
    while changed:
        previous_types = types.copy()
        previous_field_types = {
            key: set(values) for key, values in field_types.items()}
        previous_verified_bindings = verified_bindings.copy()
        previous_verified_fields = verified_fields.copy()
        changed = False
        for node in ast.walk(tree):
            containing_class = containing.get(node)
            if isinstance(node, ast.Assign):
                assigned = expression_type(node.value, containing_class)
                verified = expression_is_verified(node.value, containing_class)
                preserve_unknown_field = (
                    isinstance(node.value, ast.Name)
                    and scopes.get(node) is not None
                    and scopes[node][1] == '__init__'
                    and node.value.id in constructor_parameters.get(containing_class, ()))
                for target in node.targets:
                    changed = assign_type(
                        target, assigned, verified, preserve_unknown_field) or changed
            elif isinstance(node, ast.AnnAssign):
                assigned = annotation_return_type(node.annotation) or expression_type(
                    node.value, containing_class)
                verified = (fusion_annotation_type(node.annotation) is not None
                            or expression_is_verified(node.value, containing_class))
                preserve_unknown_field = (
                    isinstance(node.value, ast.Name)
                    and scopes.get(node) is not None
                    and scopes[node][1] == '__init__'
                    and node.value.id in constructor_parameters.get(containing_class, ()))
                changed = assign_type(
                    node.target, assigned, verified, preserve_unknown_field) or changed
            elif isinstance(node, ast.For):
                iter_type = expression_type(node.iter, containing_class)
                item = None
                if iter_type is not None:
                    info = api_member_info(iter_type, 'item')
                    item = normalize_api_type(info.get('returns')) if info else None
                changed = assign_type(node.target, item, False) or changed
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                class_name = node.func.id
                if class_name not in constructor_parameters:
                    continue
                for argument, parameter in zip(node.args, constructor_parameters[class_name]):
                    argument_type = expression_type(argument, containing_class)
                    if argument_type is None:
                        continue
                    parameter_key = ((class_name, '__init__'), parameter)
                    if types.get(parameter_key) != argument_type:
                        types[parameter_key] = argument_type
                        changed = True
                    argument_verified = expression_is_verified(
                        argument, containing_class)
                    if argument_verified and parameter_key not in verified_bindings:
                        verified_bindings.add(parameter_key)
                        changed = True
                    elif not argument_verified and parameter_key in verified_bindings:
                        verified_bindings.remove(parameter_key)
                        changed = True
                    for (field_class, field), field_parameter in constructor_fields.items():
                        if field_class == class_name and field_parameter == parameter:
                            values = field_types.setdefault((field_class, field), set())
                            if argument_type not in values:
                                values.clear()
                                values.add(argument_type)
                                changed = True
                            field_key = (field_class, field)
                            if argument_verified and field_key not in verified_fields:
                                verified_fields.add(field_key)
                                changed = True
                            elif not argument_verified:
                                verified_fields.discard(field_key)
            elif isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                receiver_type = expression_type(node.func.value, containing_class)
                found = method_parameters_for(receiver_type, node.func.attr)
                if found is None:
                    continue
                method_class, parameters = found
                for argument, parameter in zip(node.args, parameters):
                    argument_type = expression_type(argument, containing_class)
                    if argument_type is None:
                        continue
                    key = ((method_class, node.func.attr), parameter)
                    if key not in types:
                        types[key] = argument_type
                        changed = True

        changed = (
            types != previous_types
            or field_types != previous_field_types
            or verified_bindings != previous_verified_bindings
            or verified_fields != previous_verified_fields)

    return types, field_types, expression_type, verified_bindings, verified_fields


def main():
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument('target')
    ap.add_argument('--framework', default='lib',
                    help='directory holding the framework modules (default: lib)')
    args = ap.parse_args()

    src = read_source(args.target)
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
    bases = framework_bases(framework_paths)
    bases.update(class_bases_from_tree(tree))
    method_returns = framework_method_returns(framework_paths, known_class_methods)
    method_returns.update(method_returns_from_tree(tree, known_class_methods))
    containing_classes = call_classes(tree)
    framework_modules = imported_framework_modules(tree, framework_module_exports)
    qualified_fusion_types = {
        fusion_class_expr(node) for node in ast.walk(tree)
        if fusion_class_expr(node) is not None}
    member_cache = {}

    def api_member_info(cls, name):
        cls = normalize_api_type(cls)
        if cls is None:
            return None
        if cls in target_class_methods and cls not in qualified_fusion_types:
            return None
        key = (cls, name)
        if key not in member_cache:
            member_cache[key] = fusion_api.member_info(cls, name)
        return member_cache[key]

    (receiver_bindings, field_types, expression_type,
     verified_bindings, verified_fields) = infer_api_receiver_types(
        tree, known_class_methods, bases, method_returns, api_member_info)
    receiver_scopes = node_scopes(tree)

    called = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            called.setdefault(node.func.attr, []).append((
                node.lineno, receiver_expression(node.func), node, containing_classes[node]))

    def allowed(name, func, containing_class):
        if name in PYTHON_METHODS:
            return True
        receiver = func.value
        if (isinstance(receiver, ast.Call)
              and isinstance(receiver.func, ast.Name)
              and receiver.func.id == 'super'):
            return name in known_class_methods.get(containing_class, ())
        receiver_type = expression_type(receiver, containing_class)
        if (receiver_type not in qualified_fusion_types
                and name in known_class_methods.get(receiver_type, ())):
            return True
        module = framework_modules.get(receiver_root(func))
        return module is not None and name in framework_module_exports[module]

    def has_verified_receiver_binding(receiver, receiver_type, containing_class):
        if receiver_type is None:
            return False
        if fusion_class_expr(receiver) == receiver_type:
            return True

        def explicitly_bound(expression):
            if isinstance(expression, ast.Name):
                key = (receiver_scopes.get(expression), expression.id)
                return (receiver_bindings.get(key)
                        if key in verified_bindings else None)
            if isinstance(expression, ast.Attribute):
                text = ast.unparse(expression)
                key = (receiver_scopes.get(expression), text)
                bound = (receiver_bindings.get(key)
                         if key in verified_bindings else None)
                if bound is not None:
                    return bound
                if (isinstance(expression.value, ast.Name)
                        and expression.value.id == 'self'
                        and (containing_class, expression.attr) in verified_fields
                        and field_types.get((containing_class, expression.attr))):
                    return next(iter(field_types[(containing_class, expression.attr)]))
                return explicitly_bound(expression.value)
            return None

        return explicitly_bound(receiver) is not None

    def exact_unverified(name, func, receiver_type, containing_class):
        receiver = receiver_expression(func)
        for watched, cls, receivers, _ in fusion_api.UNVERIFIED_CALLS:
            if watched != name or not fusion_api.receiver_matches(receivers, receiver):
                continue
            expected = normalize_api_type(cls)
            # `self.sketch` is a generated receiver shape, but its field must still be
            # proven to be a Fusion Sketch. Exact matching alone would waive an arbitrary
            # namesake field, while tail matching would reopen `other.sketch`.
            if receiver.startswith('self.') and not has_verified_receiver_binding(
                    func.value, receiver_type, containing_class):
                continue
            if receiver_type == expected and has_verified_receiver_binding(
                    func.value, receiver_type, containing_class):
                return True
        return False

    # Where each watchlist call is actually made, receiver and all, so a legitimate namesake on
    # another class is not dragged into the report or exempted from receiver validation.
    seen = {}
    for name, _, receivers, _ in fusion_api.UNVERIFIED_CALLS:
        lines = sorted(line for line, receiver, _, _ in called.get(name, [])
                       if fusion_api.receiver_matches(receivers, receiver))
        if lines:
            seen[name] = '%s:%s' % (args.target, ','.join(str(line) for line in lines))

    unresolved = []
    resolved_names = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        name = node.func.attr
        if name.startswith('_') or allowed(name, node.func, containing_classes[node]):
            continue
        tail = receiver_tail(node.func)
        receiver_type = expression_type(node.func.value, containing_classes[node])
        if exact_unverified(name, node.func, receiver_type, containing_classes[node]):
            continue
        if receiver_type is None:
            unresolved.append((name, node.lineno, None, 'unknown receiver'))
            continue
        info = api_member_info(receiver_type, name)
        if info:
            resolved_names.add(name)
            continue
        unresolved.append((name, node.lineno, receiver_type, 'wrong receiver'))

    candidates = sorted({name for name, _, _, _ in unresolved})

    try:
        hits = fusion_api.lookup_many(candidates)
        findings = fusion_api.unverified_findings(seen)
    except fusion_api.Unavailable as exc:
        print('check_api_calls: %s' % exc, file=sys.stderr)
        return 2

    if unresolved:
        print('api-call check: BLOCKING (%d)' % len(unresolved))
        for name, lineno, receiver_type, reason in sorted(unresolved, key=lambda row: row[1]):
            near = fusion_api.similar(name)
            if hits.get(name):
                if receiver_type is None:
                    detail = "the receiver type is not known"
                else:
                    detail = "%s does not declare it" % receiver_type
                print("  %s:%d calls '%s(' — %s; receiver ownership is required%s"
                      % (args.target, lineno, name, detail,
                         '' if not near else
                         '; the nearest names the database has are %s' % ', '.join(near)))
            else:
                print("  %s:%d calls '%s(' — no such name in the Fusion API database, the "
                      "framework, or this file%s"
                      % (args.target, lineno, name,
                         '' if not near else
                         '; the nearest names the database has are %s' % ', '.join(near)))
        print('  Ask the database what a class offers: '
              'python3 %s members <Class>' % fusion_api.query_script())
        return 1

    print('api-call check: OK (%d call names resolve)' % len(resolved_names))
    if findings:
        print('api-call check: %d UNVERIFIED call(s) — reported, not blocking, not waived'
              % len(findings))
        for line in findings:
            print('  %s' % line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
