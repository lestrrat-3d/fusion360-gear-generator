import os
from ...lib import geargen
from .._gear_command import GearCommand

command = GearCommand(
    gear_type='HelicalGear',
    name='Helical Gear Generator',
    description='Helical Gear Generator',
    icon_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', ''),
    configurator=geargen.HelicalGearCommandConfigurator,
    generator_class=geargen.HelicalGearGenerator,
)

start = command.start
stop = command.stop
