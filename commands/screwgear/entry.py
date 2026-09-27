import os
from ...lib import geargen
from .._gear_command import GearCommand

command = GearCommand(
    gear_type='ScrewGear',
    name='Screw Gear Generator',
    description='Generates a screw/screw gearing',
    icon_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', ''),
    configurator=geargen.ScrewGearCommandInputsConfigurator,
    generator_class=geargen.ScrewGearGenerator,
)

start = command.start
stop = command.stop
