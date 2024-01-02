# Application Global Variables
# This module serves as a way to share variables across different
# modules (global variables).

import os

# Flag that indicates to run in Debug mode or not. When running in Debug mode
# more information is written to the Text Command window. Generally, it's useful
# to set this to True while developing an add-in and set it to False when you
# are ready to distribute it.
DEBUG = True

# This flag controls if the commands should be loaded into the "Add-In"
# panel instead of the "Create" panel where these sort of things should normally
# go into. This exists because if you are developing the module you are most
# likely loading/unloading the module multiple times and it's much easier to
# execute the command(s) if the commands are located in the same tab which
# saves you some clicking.
#
# Recommended value while you are developing is to set it equal to DEBUG
# like this:
# LOCATE_MENU_IN_ADDIN = True
LOCATE_MENU_IN_ADDIN = False

# Gets the name of the add-in from the name of the folder the py file is in.
# This is used when defining unique internal names for various UI elements 
# that need a unique name. It's also recommended to use a company name as 
# part of the ID to better ensure the ID is unique.
ADDIN_NAME = os.path.basename(os.path.dirname(__file__))
COMPANY_NAME = 'endeworks'

WORKSPACE_ID = 'FusionSolidEnvironment'
PANEL_ID = 'SolidScriptsAddinsPanel' if LOCATE_MENU_IN_ADDIN else 'SolidCreatePanel'
COMMAND_BESIDE_ID = 'ScriptsManagerCommand'
DROPDOWN_ID = 'GearGeneratorDropDown'



# Palettes
sample_palette_id = f'{COMPANY_NAME}_{ADDIN_NAME}_palette_id'