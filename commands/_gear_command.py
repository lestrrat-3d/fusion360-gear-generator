# Shared implementation of a gear-generator command. Each commands/<gear>/entry.py
# declares one GearCommand and re-exports its start/stop (see those modules).

import adsk.core
from ..lib import fusion360utils as futil
from ..lib import geargen
from .. import config

ui = adsk.core.Application.get().userInterface


def _panel():
    workspace = ui.workspaces.itemById(config.WORKSPACE_ID)
    return workspace.toolbarPanels.itemById(config.PANEL_ID)


def get_or_create_dropdown():
    panel = _panel()
    dropdown = panel.controls.itemById(config.DROPDOWN_ID)
    if not dropdown:
        dropdown = panel.controls.addDropDown('Gears', '', config.DROPDOWN_ID)
    return dropdown


def delete_dropdown():
    # The dropdown is shared by all gear commands and owned by the commands
    # package: deleted once, after every command has removed its own control
    # (see commands/__init__.py stop()).
    dropdown = _panel().controls.itemById(config.DROPDOWN_ID)
    if dropdown:
        dropdown.deleteMe()


class GearCommand:
    def __init__(self, *, gear_type, name, description, icon_folder,
                 configurator, generator_class, input_changed=None):
        self.cmd_id = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_{gear_type}_cmdDialog'
        self.name = name
        self.description = description
        self.icon_folder = icon_folder
        self.configurator = configurator
        self.generator_class = generator_class
        self.input_changed = input_changed
        # Keeps references to the per-dialog handlers so they are not garbage
        # collected while the dialog is open.
        self.local_handlers = []

    # Executed when the add-in is run.
    def start(self):
        cmd_def = ui.commandDefinitions.itemById(self.cmd_id)
        if not cmd_def:
            cmd_def = ui.commandDefinitions.addButtonDefinition(
                self.cmd_id, self.name, self.description, self.icon_folder)

        futil.add_handler(cmd_def.commandCreated, self.command_created)

        dropdown = get_or_create_dropdown()
        if not dropdown.controls.itemById(self.cmd_id):
            dropdown.controls.addCommand(cmd_def, '', False)

    # Executed when the add-in is stopped. Removes only this command's control
    # and definition; the shared dropdown is deleted by the commands package.
    def stop(self):
        dropdown = _panel().controls.itemById(config.DROPDOWN_ID)
        if dropdown:
            control = dropdown.controls.itemById(self.cmd_id)
            if control:
                control.deleteMe()

        cmd_def = ui.commandDefinitions.itemById(self.cmd_id)
        if cmd_def:
            cmd_def.deleteMe()

    # Called when the user clicks the command's button: builds the dialog
    # inputs and connects the command events.
    def command_created(self, args: adsk.core.CommandCreatedEventArgs):
        futil.log(f'{self.name} Command Created Event')

        self.configurator.configure(args.command)

        futil.add_handler(args.command.execute, self.command_execute,
                          local_handlers=self.local_handlers)
        futil.add_handler(args.command.inputChanged, self.command_input_changed,
                          local_handlers=self.local_handlers)
        futil.add_handler(args.command.validateInputs, self.command_validate_input,
                          local_handlers=self.local_handlers)
        futil.add_handler(args.command.destroy, self.command_destroy,
                          local_handlers=self.local_handlers)

    def command_execute(self, args: adsk.core.CommandEventArgs):
        futil.log(f'{self.name} Command Execute Event')
        g = None
        try:
            g = self.generator_class(geargen.get_design())
            g.generate(args.command.commandInputs)
        except Exception:
            futil.handle_error('Generation error', show_message_box=True)
            if g:
                g.deleteComponent()

    def command_input_changed(self, args: adsk.core.InputChangedEventArgs):
        futil.log(f'{self.name} Input Changed Event fired from a change to {args.input.id}')
        if self.input_changed:
            self.input_changed(args)

    def command_validate_input(self, args: adsk.core.ValidateInputsEventArgs):
        args.areInputsValid = True

    def command_destroy(self, args: adsk.core.CommandEventArgs):
        futil.log(f'{self.name} Command Destroy Event')
        self.local_handlers = []
