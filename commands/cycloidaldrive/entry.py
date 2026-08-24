import os
from ...lib import geargen
from .._gear_command import GearCommand

command = GearCommand(
    gear_type='CycloidalDrive',
    name='Cycloidal Drive Generator',
    # The ring is a pinless casing, so the tooltip names the housing rather
    # than discrete ring pins (see "Pinless ring casing" in the spec).
    description='Generates a Cycloidal Drive speed reducer (disc(s), pinless ring housing, eccentric cam, output)',
    icon_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', ''),
    configurator=geargen.CycloidalDriveCommandInputsConfigurator,
    generator_class=geargen.CycloidalDriveGenerator,
)

start = command.start
stop = command.stop
