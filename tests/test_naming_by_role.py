#!/usr/bin/env python3
"""Every object a gear add-in creates in Fusion is named by its role (see CLAUDE.md, "Naming").

The scan reads lib/geargen/*.py with the ast module. In each function it finds a sketch, construction plane, axis or
point, timeline feature, copied body or new component that is created with `<collection>.add(...)` or
`.addNewComponent(...)`, including through a local alias such as `extrudes = component.features.extrudeFeatures`. It
also finds each body a feature made, read as `<feature>.bodies.item(...)` or by a loop over `<feature>.bodies`, where
`<feature>` holds a creation of the same function. A creation is named when its result is bound to a variable that the
same function names with `<variable>.name = ...` or `name_feature(<variable>, ...)`, or when the call is itself
wrapped in `name_feature(...)`. A naming statement counts only where it runs: never in a branch a constant condition
skips, and, for a creation inside a loop, only inside that loop, so each pass names what it made. A creation that is
not bound to a variable cannot be named.
"""
import ast
import tempfile
import unittest
from pathlib import Path

GEARGEN = Path(__file__).resolve().parents[1] / 'lib' / 'geargen'

COLLECTIONS = {'sketches', 'constructionPlanes', 'constructionAxes', 'constructionPoints', 'occurrences',
               'copyPasteBodies'}
CREATE_METHODS = {'add', 'addNewComponent'}
LOOPS = (ast.For, ast.AsyncFor, ast.While)
COMPREHENSIONS = (ast.ListComp, ast.SetComp, ast.GeneratorExp, ast.DictComp)

# Creations that cannot be named where they are made. A subclass names the occurrence's component once the dialog
# has been read, because the name is built from the inputs.
EXEMPT = {('base.py', 'self.occurrence')}


def is_collection(name):
    return name in COLLECTIONS or name.endswith('Features')


def collection_of(node, aliases):
    """The collection an expression stands for, or None."""
    if isinstance(node, ast.Attribute) and is_collection(node.attr):
        return node.attr
    if isinstance(node, ast.Name) and node.id in aliases:
        return aliases[node.id]
    return None


def is_naming_call(node):
    return (isinstance(node, ast.Call) and isinstance(node.func, (ast.Name, ast.Attribute))
            and ast.unparse(node.func).endswith('name_feature') and node.args)


def constant_truth(node):
    """True or False for a condition made only of constants, else None."""
    if isinstance(node, ast.Constant):
        return bool(node.value)
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.Not):
        inner = constant_truth(node.operand)
        return None if inner is None else not inner
    if isinstance(node, ast.BoolOp):
        values = [constant_truth(v) for v in node.values]
        if isinstance(node.op, ast.And):
            if False in values:
                return False
            return True if all(v is True for v in values) else None
        if True in values:
            return True
        return False if all(v is False for v in values) else None
    return None


class Scope:
    """One function, with a parent map to place each node."""

    def __init__(self, function):
        self.function = function
        self.parents = {}
        for parent in ast.walk(function):
            for child in ast.iter_child_nodes(parent):
                self.parents[id(child)] = parent

    def ancestors(self, node):
        while id(node) in self.parents:
            parent = self.parents[id(node)]
            yield node, parent
            node = parent

    def is_dead(self, node):
        """True when a constant condition skips the node."""
        for child, parent in self.ancestors(node):
            if isinstance(parent, (ast.If, ast.While)):
                truth = constant_truth(parent.test)
                if truth is False and any(child is n for n in parent.body):
                    return True
                if truth is True and isinstance(parent, ast.If) and any(child is n for n in parent.orelse):
                    return True
        return False

    def loop_of(self, node):
        """The innermost loop or comprehension the node runs in, or None."""
        for child, parent in self.ancestors(node):
            if isinstance(parent, LOOPS):
                # A for loop's iterable and a loop's else branch run once, outside the repeated body.
                if child is getattr(parent, 'iter', None) or any(child is n for n in parent.orelse):
                    continue
                return parent
            if isinstance(parent, COMPREHENSIONS):
                return parent
        return None

    def inside(self, node, outer):
        return outer is None or any(parent is outer for _, parent in self.ancestors(node))

    def namings(self):
        """variable -> the live nodes that name it."""
        components = {}
        for node in ast.walk(self.function):
            if (isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.value, ast.Attribute)
                    and node.value.attr == 'component'):
                # component = occurrence.component: naming the component names the occurrence.
                components[ast.unparse(node.targets[0])] = ast.unparse(node.value.value)
        named = {}

        def add(variable, node):
            named.setdefault(variable, []).append(node)

        for node in ast.walk(self.function):
            if self.is_dead(node):
                continue
            if isinstance(node, ast.Assign):
                for target in node.targets:
                    if (isinstance(target, ast.Attribute) and target.attr == 'name'
                            and isinstance(target.value, (ast.Name, ast.Attribute))):
                        owner = ast.unparse(target.value)
                        add(owner, node)
                        if owner in components:
                            add(components[owner], node)
                        if owner.endswith('.component'):
                            # An occurrence is named through its component.
                            add(owner[:-len('.component')], node)
            if is_naming_call(node):
                add(ast.unparse(node.args[0]), node)
        return named


def unnamed(path):
    """Returns (line, kind, variable) for each creation in path that is never named."""
    tree = ast.parse(path.read_text())
    found = []
    for function in [n for n in ast.walk(tree) if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))]:
        scope = Scope(function)
        aliases = {}
        for node in ast.walk(function):
            if isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                if len(targets) == 1 and isinstance(targets[0], ast.Name) and node.value is not None:
                    collection = collection_of(node.value, {})
                    if collection:
                        aliases[targets[0].id] = collection
        named = scope.namings()
        wrapped = {id(arg) for call in ast.walk(function) if is_naming_call(call) for arg in call.args}
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

        def deleted_in(variable, loop):
            """True when the loop deletes what it made under variable, so only the last pass's creation survives."""
            return loop is not None and any(
                isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) and call.func.attr == 'deleteMe'
                and ast.unparse(call.func.value) == variable for call in ast.walk(loop))

        def check(node, kind, variable, loop):
            if id(node) in wrapped or id(node) in returned or (path.name, variable) in EXEMPT:
                return
            if variable is not None and deleted_in(variable, loop):
                loop = None
            if variable is not None and any(scope.inside(n, loop) for n in named.get(variable, [])):
                return
            found.append((node.lineno, kind, variable))

        creations = set()
        for node in ast.walk(function):
            if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                    and node.func.attr in CREATE_METHODS):
                continue
            collection = collection_of(node.func.value, aliases)
            if collection is None:
                continue
            variable = bound.get(id(node))
            if variable is not None:
                creations.add(variable)
            check(node, collection, variable, scope.loop_of(node))

        def feature_bodies(node):
            """True for `<feature>.bodies` where <feature> holds a creation of this function."""
            return (isinstance(node, ast.Attribute) and node.attr == 'bodies'
                    and ast.unparse(node.value) in creations)

        for node in ast.walk(function):
            if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == 'item'
                    and feature_bodies(node.func.value)):
                check(node, 'bodies', bound.get(id(node)), scope.loop_of(node))
            elif isinstance(node, (ast.For, ast.AsyncFor)) and feature_bodies(node.iter):
                variable = ast.unparse(node.target)
                if not any(scope.inside(n, node) for n in named.get(variable, [])):
                    found.append((node.lineno, 'bodies', variable))
            elif isinstance(node, ast.comprehension) and feature_bodies(node.iter):
                variable = ast.unparse(node.target)
                comprehension = scope.parents[id(node)]
                element = comprehension.elt if hasattr(comprehension, 'elt') else comprehension.value
                if not any(is_naming_call(call) and ast.unparse(call.args[0]) == variable
                           for call in ast.walk(element)):
                    found.append((node.iter.lineno, 'bodies', variable))
    return found


def scan(source):
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'x.py'
        path.write_text(source)
        return unnamed(path)


class NamingByRoleTest(unittest.TestCase):
    def test_every_created_object_is_named(self):
        files = sorted(GEARGEN.glob('*.py'))
        self.assertTrue(files)
        for path in files:
            with self.subTest(gear=path.name):
                self.assertEqual(
                    [], unnamed(path),
                    f'{path.name} creates objects without a name: name each by its role (CLAUDE.md, "Naming"); '
                    'for a generated gear change its fragment under port/<gear>/fragments/ and run gearport gen')

    def test_the_scan_sees_an_unnamed_creation(self):
        source = ("def build(component):\n"
                  "    extrudes = component.features.extrudeFeatures\n"
                  "    feature = extrudes.add(1)\n"
                  "    sketch = component.sketches.add(2)\n"
                  "    sketch.name = 'S'\n"
                  "    solids.name_feature(extrudes.add(3), 'Wrapped')\n")
        self.assertEqual([(3, 'extrudeFeatures', 'feature')], scan(source))

    def test_the_scan_sees_an_unnamed_copied_body(self):
        source = ("def build(component, entities):\n"
                  "    copies = component.features.copyPasteBodies\n"
                  "    feature = copies.add(entities)\n")
        self.assertEqual([(3, 'copyPasteBodies', 'feature')], scan(source))

    def test_a_name_in_a_dead_branch_does_not_count(self):
        source = ("def build(component):\n"
                  "    feature = component.features.extrudeFeatures.add(1)\n"
                  "    if False:\n"
                  "        feature.name = 'Extrude'\n"
                  "    while 0:\n"
                  "        solids.name_feature(feature, 'Extrude')\n")
        self.assertEqual([(2, 'extrudeFeatures', 'feature')], scan(source))
        live = ("def build(component):\n"
                "    feature = component.features.extrudeFeatures.add(1)\n"
                "    if True:\n"
                "        feature.name = 'Extrude'\n")
        self.assertEqual([], scan(live))

    def test_a_name_after_a_loop_names_only_the_last_creation(self):
        source = ("def build(component, n):\n"
                  "    for i in range(n):\n"
                  "        feature = component.features.extrudeFeatures.add(i)\n"
                  "    feature.name = 'Extrude'\n")
        self.assertEqual([(3, 'extrudeFeatures', 'feature')], scan(source))
        replaced = ("def build(component, n):\n"
                    "    for i in range(n):\n"
                    "        feature = component.features.extrudeFeatures.add(i)\n"
                    "        if good(feature):\n"
                    "            break\n"
                    "        feature.deleteMe()\n"
                    "    feature.name = 'Extrude'\n")
        self.assertEqual([], scan(replaced), 'a loop that deletes each rejected creation keeps only the last')
        inside = ("def build(component, n):\n"
                  "    for i in range(n):\n"
                  "        feature = component.features.extrudeFeatures.add(i)\n"
                  "        feature.name = f'Extrude {i}'\n")
        self.assertEqual([], scan(inside))

    def test_the_scan_sees_an_unnamed_body(self):
        source = ("def build(component):\n"
                  "    extrude = component.features.extrudeFeatures.add(1)\n"
                  "    extrude.name = 'Extrude'\n"
                  "    body = extrude.bodies.item(0)\n"
                  "    use(Body(extrude.bodies.item(0)))\n"
                  "    for piece in extrude.bodies:\n"
                  "        use(piece)\n"
                  "    copies = [Body(b) for b in extrude.bodies]\n"
                  "    named = [Body(name_feature(b, 'Copy')) for b in extrude.bodies]\n")
        self.assertEqual([(4, 'bodies', 'body'), (5, 'bodies', None), (6, 'bodies', 'piece'), (8, 'bodies', 'b')],
                         sorted(scan(source)))


if __name__ == '__main__':
    unittest.main()
