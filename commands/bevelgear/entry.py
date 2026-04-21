import adsk.core
import os
from ...lib import fusion360utils as futil
from ...lib import geargen
from ... import config
app = adsk.core.Application.get()
ui = app.userInterface

GEAR_TYPE = 'BevelGear'
CMD_ID = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_{GEAR_TYPE}_cmdDialog'
CMD_NAME = 'Bevel Gear Generator'
CMD_Description = 'Bevel Gear Generator'

IS_PROMOTED = True

ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

local_handlers = []


def start():
    cmd_def = ui.commandDefinitions.itemById(CMD_ID)
    if not cmd_def:
        cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_Description, ICON_FOLDER)

    futil.add_handler(cmd_def.commandCreated, command_created)

    workspace = ui.workspaces.itemById(config.WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(config.PANEL_ID)

    dropDown = panel.controls.itemById(config.DROPDOWN_ID)
    if not dropDown:
        dropDown = panel.controls.addDropDown('Gears', '', config.DROPDOWN_ID)

    control = dropDown.controls.itemById(CMD_ID)
    if not control:
        control = dropDown.controls.addCommand(cmd_def, '', False)


def stop():
    workspace = ui.workspaces.itemById(config.WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(config.PANEL_ID)
    dropDown = panel.controls.itemById(config.DROPDOWN_ID)
    command_definition = ui.commandDefinitions.itemById(CMD_ID)

    if dropDown:
        dropDown.deleteMe()

    if command_definition:
        command_definition.deleteMe()


def command_created(args: adsk.core.CommandCreatedEventArgs):
    futil.log(f'{CMD_NAME} Command Created Event')

    geargen.BevelGearCommandInputsConfigurator.configure(args.command)

    futil.add_handler(args.command.execute, command_execute, local_handlers=local_handlers)
    futil.add_handler(args.command.inputChanged, command_input_changed, local_handlers=local_handlers)
    futil.add_handler(args.command.executePreview, command_preview, local_handlers=local_handlers)
    futil.add_handler(args.command.validateInputs, command_validate_input, local_handlers=local_handlers)
    futil.add_handler(args.command.destroy, command_destroy, local_handlers=local_handlers)


def command_execute(args: adsk.core.CommandEventArgs):
    futil.log(f'{CMD_NAME} Command Execute Event')
    g = None
    try:
        inputs = args.command.commandInputs
        design = geargen.get_design()
        g = geargen.BevelGearGenerator(design)
        g.generate(inputs)
    except:
        futil.handle_error("Generation error", show_message_box=True)
        if g:
            g.deleteComponent()


def command_preview(args: adsk.core.CommandEventArgs):
    pass


def command_input_changed(args: adsk.core.InputChangedEventArgs):
    pass


def command_validate_input(args: adsk.core.ValidateInputsEventArgs):
    args.areInputsValid = True


def command_destroy(args: adsk.core.CommandEventArgs):
    global local_handlers
    local_handlers = []
