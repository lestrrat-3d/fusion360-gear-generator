# Here you define the commands that will be added to your add-in.
# To add a command: create a directory with an entry.py declaring a GearCommand
# (see commands/_gear_command.py), import it here, and add it to the list.
from . import _gear_command
from .spurgear import entry as spurgear
from .helicalgear import entry as helicalgear
from .herringbonegear import entry as herringbonegear
from .bevelgear import entry as bevelgear
from .cycloidaldrive import entry as cycloidaldrive

# Fusion will automatically call the start() and stop() functions.
commands = [
    spurgear,
    helicalgear,
    herringbonegear,
    bevelgear,
    cycloidaldrive,
]


def start():
    for command in commands:
        command.start()


def stop():
    for command in commands:
        command.stop()
    # Each command removed its own control above; the shared Gears dropdown is
    # owned by this package and deleted once here.
    _gear_command.delete_dropdown()
