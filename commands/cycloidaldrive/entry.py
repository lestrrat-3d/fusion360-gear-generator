import os
from ...lib import geargen
from .._gear_command import GearCommand

command = GearCommand(
    gear_type='CycloidalDrive',
    name='Cycloidal Drive Generator',
    description='Generates a Cycloidal Drive speed reducer (disk(s), ring pins, eccentric, output)',
    icon_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', ''),
    configurator=geargen.CycloidalDriveCommandInputsConfigurator,
    generator_class=geargen.CycloidalDriveGenerator,
)

start = command.start
stop = command.stop
