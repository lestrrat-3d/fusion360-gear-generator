import adsk.core
import os, math
from ...lib import fusion360utils as futil
from ...lib import geargen
from ... import config
app = adsk.core.Application.get()
ui = app.userInterface

# TODO *** Specify the command identity information. ***
GEAR_TYPE='SpurGear'
CMD_ID = f'{config.COMPANY_NAME}_{config.ADDIN_NAME}_{GEAR_TYPE}_cmdDialog'
CMD_NAME = 'Spur Gear Generator'
CMD_Description = 'Generates a Spur Gear'

# Specify that the command will be promoted to the panel.
IS_PROMOTED = True

# Resource location for command icons, here we assume a sub folder in this directory named "resources".
ICON_FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'resources', '')

# Local list of event handlers used to maintain a reference so
# they are not released and garbage collected.
local_handlers = []


# Executed when add-in is run.
def start():
    # Create a command Definition.
    cmd_def = ui.commandDefinitions.itemById(CMD_ID)
    if not cmd_def:
        cmd_def = ui.commandDefinitions.addButtonDefinition(CMD_ID, CMD_NAME, CMD_Description, ICON_FOLDER)

    # Define an event handler for the command created event. It will be called when the button is clicked.
    futil.add_handler(cmd_def.commandCreated, command_created)

    # Get the target workspace the button will be created in.
    workspace = ui.workspaces.itemById(config.WORKSPACE_ID)

    # Get the panel the button will be created in.
    panel = workspace.toolbarPanels.itemById(config.PANEL_ID)

    dropDown = panel.controls.itemById(config.DROPDOWN_ID)
    if not dropDown:
        dropDown = panel.controls.addDropDown('Gears', '', config.DROPDOWN_ID)
        # Create the button command control in the UI after the specified existing command.
    
    control = dropDown.controls.itemById(CMD_ID)
    if not control:
        control = dropDown.controls.addCommand(cmd_def, '', False)

# Executed when add-in is stopped.
def stop():
    # Get the various UI elements for this command
    workspace = ui.workspaces.itemById(config.WORKSPACE_ID)
    panel = workspace.toolbarPanels.itemById(config.PANEL_ID)
    dropDown = panel.controls.itemById(config.DROPDOWN_ID)
    command_definition = ui.commandDefinitions.itemById(CMD_ID)

    if dropDown:
        dropDown.deleteMe()

    # Delete the command definition
    if command_definition:
        command_definition.deleteMe()



# Function that is called when a user clicks the corresponding button in the UI.
# This defines the contents of the command dialog and connects to the command related events.
def command_created(args: adsk.core.CommandCreatedEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Created Event')

    geargen.SpurGearCommandInputsConfigurator.configure(args.command)

    # TODO Connect to the events that are needed by this command.
    futil.add_handler(args.command.execute, command_execute, local_handlers=local_handlers)
    futil.add_handler(args.command.inputChanged, command_input_changed, local_handlers=local_handlers)
    futil.add_handler(args.command.executePreview, command_preview, local_handlers=local_handlers)
    futil.add_handler(args.command.validateInputs, command_validate_input, local_handlers=local_handlers)
    futil.add_handler(args.command.destroy, command_destroy, local_handlers=local_handlers)


# This event handler is called when the user clicks the OK button in the command dialog or 
# is immediately called after the created event not command inputs were created for the dialog.
def command_execute(args: adsk.core.CommandEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Execute Event')
    g = None
    try:
        inputs = args.command.commandInputs

        futil.log(f'selection count (OUTSIDE) = {inputs.itemById("plane").selectionCount}')
        spec = geargen.SpurGearSpecification.from_inputs(inputs)

        design = geargen.get_design()
        g = geargen.SpurGearGenerator(design)
        g.generate(spec)
    except:
        futil.handle_error("Generation error", show_message_box=True)
        if g:
            g.deleteComponent()


# This event handler is called when the command needs to compute a new preview in the graphics window.
def command_preview(args: adsk.core.CommandEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Preview Event')
    inputs = args.command.commandInputs


# This event handler is called when the user changes anything in the command dialog
# allowing you to modify values of other inputs based on that change.
def command_input_changed(args: adsk.core.InputChangedEventArgs):
    changed_input = args.input
    inputs = args.inputs


# This event handler is called when the user interacts with any of the inputs in the dialog
# which allows you to verify that all of the inputs are valid and enables the OK button.
def command_validate_input(args: adsk.core.ValidateInputsEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Validate Input Event')

    args.areInputsValid = True
    


# This event handler is called when the command terminates.
def command_destroy(args: adsk.core.CommandEventArgs):
    # General logging for debug.
    futil.log(f'{CMD_NAME} Command Destroy Event')

    global local_handlers
    local_handlers = []
