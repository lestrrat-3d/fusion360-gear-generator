import os
from ...lib import geargen
from .._gear_command import GearCommand

command = GearCommand(
    gear_type='SpurGear',
    name='Spur Gear Generator',
    description='Generates a Spur Gear',
    icon_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', ''),
    configurator=geargen.SpurGearCommandInputsConfigurator,
    generator_class=geargen.SpurGearGenerator,
)

start = command.start
stop = command.stop
