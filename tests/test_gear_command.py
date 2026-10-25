#!/usr/bin/env python3
"""A command runs the generator the files on disk define, and its Build line names every module it uses.

Fusion keeps imported modules across an add-in Stop and Run, so a deploy can leave old code running [PB-STALE-MODULE].
The command reloads the geargen modules a generator uses when a file changed, in dependency order, so a generated gear
never runs against a runtime from another deploy. These tests change what the command sees as the file on disk; they do
not edit any file.
"""
import importlib
import sys
import unittest
from unittest import mock

from fakefusion import FusionTestCase


class GearCommandTest(FusionTestCase):
    def setUp(self):
        self.command = importlib.import_module('addin.commands._gear_command')
        self.gears = 'addin.lib.geargen.'

    def tearDown(self):
        # A test that fakes a hash leaves it recorded; the next test starts from the files as they are.
        self.command._LOADED.clear()
        for module in self.command._gear_modules(self.module('herringbonegear').HerringboneGearGenerator):
            self.command._record(module)
        for name in ('bevelgear', 'cycloidal'):
            self.command._record(self.module(name))

    def module(self, name):
        return sys.modules[self.gears + name]

    def changed_on_disk(self, *names):
        """Make the command read a new hash for the named modules' files, as after a deploy that changed them."""
        real = self.command._file_hash
        return mock.patch.object(self.command, '_file_hash', lambda module: (
            'deployed' if module.__name__.rpartition('.')[2] in names else real(module)))

    def test_the_modules_of_a_generator_come_after_the_modules_they_use(self):
        helical = self.module('helicalgear').HelicalGearGenerator
        names = [m.__name__.rpartition('.')[2] for m in self.command._gear_modules(helical)]
        self.assertEqual('helicalgear', names[-1])
        self.assertLess(names.index('decadfusion'), names.index('spurgear'))
        self.assertLess(names.index('spurgear'), names.index('helicalgear'))

    def test_a_changed_gear_file_reloads_it_and_the_gears_built_on_it(self):
        old_spur = self.module('spurgear').SpurGearGenerator
        old_helical = self.module('helicalgear').HelicalGearGenerator
        old_herringbone = self.module('herringbonegear').HerringboneGearGenerator
        with self.changed_on_disk('spurgear'):
            helical = self.command.fresh_generator_class(old_helical)
            self.assertIsNot(old_helical, helical)
            self.assertIsNot(old_spur, self.module('spurgear').SpurGearGenerator)
            self.assertTrue(issubclass(helical, self.module('spurgear').SpurGearGenerator))
            # The herringbone command's class was built on the helical class reloaded above.
            herringbone = self.command.fresh_generator_class(old_herringbone)
            self.assertIsNot(old_herringbone, herringbone)
            self.assertTrue(issubclass(herringbone, helical))
            # Nothing is reloaded again once every module matches its file.
            self.assertIs(herringbone, self.command.fresh_generator_class(herringbone))

    def test_a_changed_runtime_reloads_the_runtime_before_the_gear(self):
        reloaded = []
        real = importlib.reload
        with self.changed_on_disk('decadfusion'), \
                mock.patch.object(self.command.importlib, 'reload', lambda m: reloaded.append(m.__name__) or real(m)):
            self.command.fresh_generator_class(self.module('spurgear').SpurGearGenerator)
        names = [name.rpartition('.')[2] for name in reloaded]
        self.assertEqual('decadfusion', names[0])
        self.assertIn('spurgear', names)

    def test_a_module_that_stays_stale_fails_the_build(self):
        counter = {'n': 0}

        def always_new(module):
            counter['n'] += 1
            return f'hash-{counter["n"]}'
        with mock.patch.object(self.command, '_file_hash', always_new):
            with self.assertRaisesRegex(RuntimeError, 'STALE'):
                self.command.fresh_generator_class(self.module('spurgear').SpurGearGenerator)

    def test_the_build_line_names_the_code_every_gear_runs(self):
        for gear, cls in (('spurgear', 'SpurGearGenerator'), ('helicalgear', 'HelicalGearGenerator'),
                          ('herringbonegear', 'HerringboneGearGenerator'), ('bevelgear', 'BevelGearGenerator'),
                          ('cycloidal', 'CycloidalDriveGenerator')):
            with self.subTest(gear=gear):
                module = self.module(gear)
                label = self.command._build_label(getattr(module, cls))
                self.assertIn(f'{gear} {module.GEAR_BUILD_ID}', label)
                self.assertIn('decadfusion ', label)
                self.assertNotIn('STALE', label)
        with self.changed_on_disk('spurgear'):
            label = self.command._build_label(self.module('helicalgear').HelicalGearGenerator)
        self.assertIn('STALE (file on disk changed)', label)


if __name__ == '__main__':
    unittest.main()
