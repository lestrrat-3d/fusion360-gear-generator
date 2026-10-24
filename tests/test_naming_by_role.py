#!/usr/bin/env python3
"""Every object a gear add-in creates in Fusion is named by its role (see CLAUDE.md, "Naming").

The scan reads lib/geargen/*.py with the ast module. In each function it finds a sketch, construction plane, axis or
point, timeline feature or new component that is created with `<collection>.add(...)` or `.addNewComponent(...)`,
including through a local alias such as `extrudes = component.features.extrudeFeatures`. A creation is named when its
result is bound to a variable that the same function names with `<variable>.name = ...` or `name_feature(<variable>, ...)`,
or when the call is itself wrapped in `name_feature(...)`. A creation that is not bound to a variable cannot be named.
"""
import ast
import unittest
from pathlib import Path

GEARGEN = Path(__file__).resolve().parents[1] / 'lib' / 'geargen'

COLLECTIONS = {'sketches', 'constructionPlanes', 'constructionAxes', 'constructionPoints', 'occurrences'}
CREATE_METHODS = {'add', 'addNewComponent'}

# Creations that cannot be named where they are made. A subclass names the occurrence's component once the dialog
# has been read, because the name is built from the inputs.
EXEMPT = {('base.py', 'self.occurrence')}

# Gears that have not been regenerated with the rule. The test fails when a count changes, so an entry is removed when
# its gear is fixed and can never grow.
PENDING_REGENERATION = {}


def is_collection(name):
    return name in COLLECTIONS or name.endswith('Features')


def collection_of(node, aliases):
    """The collection an expression stands for, or None."""
    if isinstance(node, ast.Attribute) and is_collection(node.attr):
        return node.attr
    if isinstance(node, ast.Name) and node.id in aliases:
        return aliases[node.id]
    return None


def names_in(function):
    """The variables a function names, with `<variable>.name = ...` or `name_feature(<variable>, ...)`."""
    named = set()
    components = {}
    for node in ast.walk(function):
        if (isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.value, ast.Attribute)
                and node.value.attr == 'component'):
            # component = occurrence.component: naming the component names the occurrence.
            components[ast.unparse(node.targets[0])] = ast.unparse(node.value.value)
    for node in ast.walk(function):
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if (isinstance(target, ast.Attribute) and target.attr == 'name'
                        and isinstance(target.value, (ast.Name, ast.Attribute))):
                    owner = ast.unparse(target.value)
                    named.add(owner)
                    if owner in components:
                        named.add(components[owner])
                    if owner.endswith('.component'):
                        # An occurrence is named through its component.
                        named.add(owner[:-len('.component')])
        if (isinstance(node, ast.Call) and isinstance(node.func, (ast.Name, ast.Attribute))
                and ast.unparse(node.func).endswith('name_feature') and node.args):
            named.add(ast.unparse(node.args[0]))
    return named


def unnamed(path):
    """Returns (line, collection, variable) for each creation in path that is never named."""
    tree = ast.parse(path.read_text())
    found = []
    for function in [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        aliases = {}
        for node in ast.walk(function):
            if isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                if len(targets) == 1 and isinstance(targets[0], ast.Name) and node.value is not None:
                    collection = collection_of(node.value, {})
                    if collection:
                        aliases[targets[0].id] = collection
        named = names_in(function)
        wrapped = {id(arg) for call in ast.walk(function) if isinstance(call, ast.Call)
                   and ast.unparse(call.func).endswith('name_feature') for arg in call.args}
        bound = {}
        returned = set()
        for node in ast.walk(function):
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    bound[id(node.value)] = ast.unparse(target)
            elif isinstance(node, ast.AnnAssign) and node.value is not None:
                bound[id(node.value)] = ast.unparse(node.target)
            elif isinstance(node, ast.Return) and node.value is not None:
                # A helper that returns the new object leaves the naming to its caller.
                returned.add(id(node.value))
        for node in ast.walk(function):
            if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr in CREATE_METHODS):
                continue
            collection = collection_of(node.func.value, aliases)
            if collection is None or id(node) in wrapped or id(node) in returned:
                continue
            variable = bound.get(id(node))
            if (path.name, variable) in EXEMPT:
                continue
            if variable is None or variable not in named:
                found.append((node.lineno, collection, variable))
    return found


class NamingByRoleTest(unittest.TestCase):
    def test_every_created_object_is_named(self):
        files = sorted(GEARGEN.glob('*.py'))
        self.assertTrue(files)
        for path in files:
            if path.name in PENDING_REGENERATION:
                continue
            with self.subTest(gear=path.name):
                self.assertEqual(
                    [], unnamed(path),
                    f'{path.name} creates objects without a name: name each by its role (CLAUDE.md, "Naming"); '
                    'change its fragment under port/<gear>/fragments/ and run gearport gen')

    def test_pending_regeneration_list_is_exact(self):
        for name, count in PENDING_REGENERATION.items():
            with self.subTest(gear=name):
                self.assertEqual(count, len(unnamed(GEARGEN / name)))

    def test_the_scan_sees_an_unnamed_creation(self):
        import tempfile
        source = ("def build(component):\n"
                  "    extrudes = component.features.extrudeFeatures\n"
                  "    feature = extrudes.add(1)\n"
                  "    sketch = component.sketches.add(2)\n"
                  "    sketch.name = 'S'\n"
                  "    solids.name_feature(extrudes.add(3), 'Wrapped')\n")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'x.py'
            path.write_text(source)
            self.assertEqual([(3, 'extrudeFeatures', 'feature')], unnamed(path))


if __name__ == '__main__':
    unittest.main()
