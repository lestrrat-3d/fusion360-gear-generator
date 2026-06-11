import os
from ...lib import geargen
from .._gear_command import GearCommand

command = GearCommand(
    gear_type='BevelGear',
    name='Bevel Gear Generator',
    description='Bevel Gear Generator',
    icon_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', ''),
    configurator=geargen.BevelGearCommandInputsConfigurator,
    generator_class=geargen.BevelGearGenerator,
    # Drives conditional visibility of the spiral-only inputs (shown only when ψ > 0).
    input_changed=geargen.BevelGearCommandInputsConfigurator.handle_input_changed,
)

start = command.start
stop = command.stop
