"""Shared stand-ins for Fusion, for tests that load the add-in's modules outside Fusion.

FusionTestCase stubs the adsk modules with MagicMocks, loads the add-in as the package `addin`, and points the
runtime's progress file at a temporary path, so a test run never writes into the real
`<temp>/geargen-progress.log`. The fakes below answer the calls the runtime makes the way Fusion does, and record
what was made.
"""
import importlib
import importlib.util
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def install_stubs(core=None, fusion=None):
    """Replace adsk and the add-in package in sys.modules. Returns the modules it replaced, for restore_modules."""
    saved = {n: m for n, m in sys.modules.items() if n == 'adsk' or n.startswith(('adsk.', 'addin'))}
    for name in saved:
        del sys.modules[name]
    adsk = types.ModuleType('adsk')
    adsk.__path__ = []
    for name, stub in (('core', core), ('fusion', fusion)):
        stub = stub if stub is not None else mock.MagicMock(name=f'adsk.{name}')
        sys.modules[f'adsk.{name}'] = stub
        setattr(adsk, name, stub)
    sys.modules['adsk'] = adsk
    spec = importlib.util.spec_from_file_location(
        'addin', ROOT / '__init__.py', submodule_search_locations=[str(ROOT)])
    sys.modules['addin'] = importlib.util.module_from_spec(spec)
    return saved


def restore_modules(saved):
    for name in [n for n in sys.modules if n == 'adsk' or n.startswith(('adsk.', 'addin'))]:
        del sys.modules[name]
    sys.modules.update(saved)


def redirect_progress(fx):
    """Point the runtime's progress file at a new temporary directory, which the caller cleans up."""
    directory = tempfile.TemporaryDirectory()
    fx.PROGRESS_FILE = str(Path(directory.name) / 'geargen-progress.log')
    return directory


class FusionTestCase(unittest.TestCase):
    """Loads addin.lib.geargen.decadfusion as cls.fx with Fusion stubbed."""

    @classmethod
    def make_core(cls):
        core = mock.MagicMock(name='adsk.core')
        core.Point3D.create = lambda x, y, z: SimpleNamespace(x=x, y=y, z=z)
        core.ObjectCollection.create = FakeCollection
        core.ValueInput.createByReal = lambda v: ('real', v)
        core.ValueInput.createByString = lambda v: ('string', v)
        return core

    @classmethod
    def setUpClass(cls):
        cls.saved = install_stubs(core=cls.make_core())
        cls.fx = importlib.import_module('addin.lib.geargen.decadfusion')
        cls.progress_dir = redirect_progress(cls.fx)

    @classmethod
    def tearDownClass(cls):
        restore_modules(cls.saved)
        cls.progress_dir.cleanup()


class FakeCollection:
    def __init__(self):
        self.items = []

    def add(self, item):
        self.items.append(item)

    @property
    def count(self):
        return len(self.items)

    def item(self, i):
        return self.items[i]


class Entity:
    """A Fusion object whose entityToken differs on every read, as the API reference allows. Two wrappers of one
    entity compare equal with ==, as Fusion's objects do; nothing else about them is equal."""

    reads = 0

    def __init__(self, ident=None):
        self.ident = ident if ident is not None else object()
        self.name = ''

    @property
    def entityToken(self):
        Entity.reads += 1
        return f'token-{Entity.reads}'

    def __eq__(self, other):
        return isinstance(other, Entity) and other.ident is self.ident

    def __hash__(self):
        raise TypeError('a Fusion object is not hashable')

    def again(self):
        """Another Python wrapper of the same entity, as a second read of a Fusion collection returns."""
        twin = type(self).__new__(type(self))
        twin.__dict__.update(self.__dict__)
        return twin


class FakeBRep(Entity):
    """A body with a centre of mass, in the design's centimetres."""

    def __init__(self, x=0.0, y=0.0, z=0.0, ident=None):
        super().__init__(ident)
        self.physicalProperties = SimpleNamespace(centerOfMass=SimpleNamespace(x=x, y=y, z=z))
        self.faces = []
        self.edges = []
        self.isSolid = True

    def moveToComponent(self, target):
        moved = FakeBRep(self.physicalProperties.centerOfMass.x, 0.0, 0.0)
        moved.movedTo = target
        return moved


class BodyList(list):
    """A feature's bodies: iterable, with count and item()."""

    @property
    def count(self):
        return len(self)

    def item(self, i):
        return self[i]


class Recorder:
    """component.features for the runtime's feature calls. Each add records (kind, input) in log and answers as Fusion
    does. A test changes what an add returns through the make_* hooks."""

    def __init__(self, log):
        self.log = log
        recorder = self

        def adder(kind, make):
            def add(feature_input):
                recorder.log.append((kind, feature_input))
                return make(feature_input)
            return add

        def recording(feature_input_kind):
            # An input object that records every method called on it.
            def create(*args, **kwargs):
                return RecordingInput(feature_input_kind, args)
            return create

        self.combineFeatures = SimpleNamespace(createInput=recording('combine'),
                                               add=adder('combine', lambda i: SimpleNamespace(name='')))
        self.moveFeatures = SimpleNamespace(createInput2=recording('move'),
                                            add=adder('move', lambda i: SimpleNamespace(name='')))
        self.copyPasteBodies = SimpleNamespace(add=adder('copy', lambda i: recorder.make_copy(i)))
        self.circularPatternFeatures = SimpleNamespace(createInput=recording('pattern'),
                                                       add=adder('pattern', lambda i: recorder.make_pattern(i)))
        self.mirrorFeatures = SimpleNamespace(createInput=recording('mirror'),
                                              add=adder('mirror', lambda i: recorder.make_mirror(i)))
        self.loftFeatures = SimpleNamespace(createInput=recording('loft'),
                                            add=adder('loft', lambda i: recorder.make_single(i)))
        self.extrudeFeatures = SimpleNamespace(createInput=recording('extrude'),
                                               add=adder('extrude', lambda i: recorder.make_single(i)))
        self.removeFeatures = SimpleNamespace(add=adder('remove', lambda i: SimpleNamespace(name='')))
        self.splitBodyFeatures = SimpleNamespace(createInput=recording('split'),
                                                 add=adder('split', lambda i: recorder.make_split(i)))
        self.scaleFeatures = SimpleNamespace(createInput=recording('scale'),
                                             add=adder('scale', lambda i: SimpleNamespace(name='')))
        self.filletFeatures = SimpleNamespace(createInput=recording('fillet'),
                                              add=adder('fillet', lambda i: SimpleNamespace(name='')))
        self.split_pieces = None

    def make_copy(self, entities):
        source = entities.items[0]
        return SimpleNamespace(bodies=BodyList([FakeBRep(*_centre(source))]), name='')

    def make_pattern(self, pattern_input):
        """Fusion's circular pattern: the source (as another wrapper) and count - 1 copies turned about the axis. A test
        that patches out the axis lookup sets the axis as pattern_axis, a centre and a unit direction."""
        source = pattern_input.args[0].items[0]
        count = int(pattern_input.attrs['quantity'][1])
        centre, direction = self.pattern_axis
        bodies = [source.again()]
        for k in range(1, count):
            bodies.append(FakeBRep(*_turn(_centre(source), centre, direction, 2 * math.pi * k / count)))
        return SimpleNamespace(bodies=BodyList(bodies), name='')

    def make_mirror(self, mirror_input):
        source = mirror_input.args[0].items[0]
        return SimpleNamespace(bodies=BodyList([source.again(), FakeBRep(*_centre(source))]), name='')

    def make_single(self, feature_input):
        return SimpleNamespace(bodies=BodyList([FakeBRep()]), name='')

    def make_split(self, split_input):
        pieces = self.split_pieces or [FakeBRep(1.0, 0, 0), FakeBRep(5.0, 0, 0)]
        return SimpleNamespace(bodies=BodyList(pieces), name='', deleteMe=lambda: self.log.append(('deleteMe',)))

    pattern_axis = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))

    def kinds(self):
        return [entry[0] for entry in self.log]


class RecordingInput:
    """A Fusion feature input: remembers its creation arguments, the methods called and the properties set."""

    def __init__(self, kind, args):
        object.__setattr__(self, 'kind', kind)
        object.__setattr__(self, 'args', args)
        object.__setattr__(self, 'calls', [])
        object.__setattr__(self, 'attrs', {})

    def __setattr__(self, name, value):
        self.attrs[name] = value

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        if name in self.attrs:
            return self.attrs[name]
        return _Member(self, name)

    def called(self, name):
        """The arguments of each call of the named method, such as 'setTwoSidesExtent' or 'loftSections.add'."""
        return [args for called, args in self.calls if called == name]


class _Member:
    """A method or a member object of a RecordingInput: calling it records the call, and its attributes are members."""

    def __init__(self, owner, name):
        self._owner, self._name = owner, name

    def __call__(self, *args, **kwargs):
        self._owner.calls.append((self._name, args))
        return True

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)
        return _Member(self._owner, f'{self._name}.{name}')


def _centre(body):
    c = body.physicalProperties.centerOfMass
    return c.x, c.y, c.z


def _turn(point, centre, direction, angle):
    """point turned by angle about the line through centre along the unit direction (Rodrigues)."""
    p = [point[i] - centre[i] for i in range(3)]
    k = direction
    cos, sin = math.cos(angle), math.sin(angle)
    dot = sum(p[i] * k[i] for i in range(3))
    cross = (k[1] * p[2] - k[2] * p[1], k[2] * p[0] - k[0] * p[2], k[0] * p[1] - k[1] * p[0])
    return tuple(centre[i] + p[i] * cos + cross[i] * sin + k[i] * dot * (1 - cos) for i in range(3))
