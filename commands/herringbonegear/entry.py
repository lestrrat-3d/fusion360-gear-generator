import os
from ...lib import geargen
from .._gear_command import GearCommand

command = GearCommand(
    gear_type='HerringboneGear',
    name='Herringbone Gear Generator',
    description='Herringbone Gear Generator',
    icon_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', ''),
    configurator=geargen.HerringboneGearCommandConfigurator,
    generator_class=geargen.HerringboneGearGenerator,
)

start = command.start
stop = command.stop
