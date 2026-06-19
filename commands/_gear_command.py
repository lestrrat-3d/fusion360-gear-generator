# Shared implementation of a gear-generator command. Each commands/<gear>/entry.py
# declares one GearCommand and re-exports its start/stop (see those modules).

import sys
import adsk.core
from ..lib import fusion360utils as futil
from ..lib import geargen
from .. import config

ui = adsk.core.Application.get().userInterface

# Bump this string on every deploy of THIS file. It proves whether the
# commands/ layer reloaded: if the dialog's Build line shows an old tag, the
# add-in Stop/Run did not re-import commands/_gear_command.py.
BUILD_TAG = 'r4-2026-06-19'

# Plain (undecorated) generator methods used as canaries to detect whether the
# *loaded* generator module matches the file on disk. co_firstlineno comes from
# the bytecode Fusion actually executed; if it disagrees with where the `def`
# sits in the current source file, Fusion is running a stale copy.
_CANARY_METHODS = ('generate', 'buildChamfers', '_resolveDimensions')


def _staleness(generator_class):
    # Returns (is_stale, [(method, loaded_line, disk_line), ...]) or (None, None)
    # if it could not be determined.
    try:
        module = sys.modules[generator_class.__module__]
        path = getattr(module, '__file__', None)
        if not path:
            return None, None
        with open(path, encoding='utf-8') as fh:
            disk_lines = fh.read().splitlines()
        results = []
        for name in _CANARY_METHODS:
            member = generator_class.__dict__.get(name)
            code = getattr(member, '__code__', None)
            if code is None:
                continue
            needle = 'def {}('.format(name)
            disk_line = None
            for i, line in enumerate(disk_lines, 1):
                if line.strip().startswith(needle):
                    disk_line = i
                    break
            results.append((name, code.co_firstlineno, disk_line))
        is_stale = any(
            disk is not None and loaded != disk
            for _, loaded, disk in results)
        return is_stale, results
    except Exception:
        return None, None


def _build_label(generator_class):
    is_stale, results = _staleness(generator_class)
    if is_stale is None:
        body = 'generator version could not be read'
    elif is_stale:
        bad = ', '.join(
            '{} running line {} but file says {}'.format(n, loaded, disk)
            for n, loaded, disk in results if disk is not None and loaded != disk)
        body = 'STALE generator module — restart Fusion ({})'.format(bad)
    else:
        matched = ', '.join('{}@{}'.format(n, loaded)
                            for n, loaded, _ in results)
        body = 'generator matches file on disk ({})'.format(matched)
    return 'Build: command {} | {}'.format(BUILD_TAG, body)


# A gear opts into live validation by giving each editable input a trailing,
# hidden read-only TextBox whose id is the input's id + this suffix; the handler
# below shows the one nearest the field the user just edited.
STATUS_SUFFIX = '__status'


def _format_problems(problems):
    # Render live-validation problems as a command TextBox's formatted text.
    # The emitted strings are plain (no raw '<'/'>'), so embedding them in this
    # light HTML is safe. Empty list -> empty string (caller hides the box).
    if not problems:
        return ''
    return '<br>'.join(problems)


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

        # Per-dialog live-validation state (see command_validate_input).
        self._last_changed_input_id = None
        self._shown_status_id = None

        # Version marker: logged only (NOT shown in the dialog) and surfaced in
        # the generation-error box, so a reported traceback is attributable to a
        # specific running build without cluttering the input dialog.
        futil.log(f'{self.name} {_build_label(self.generator_class)}')

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
            # Put the build marker in the error box so any traceback the user
            # reports is attributable to a specific running version.
            futil.handle_error(
                'Generation error\n' + _build_label(self.generator_class),
                show_message_box=True)
            if g:
                g.deleteComponent()

    def command_input_changed(self, args: adsk.core.InputChangedEventArgs):
        futil.log(f'{self.name} Input Changed Event fired from a change to {args.input.id}')
        # Remember the field the user just touched so command_validate_input
        # (which fires right after) can show the message next to it.
        self._last_changed_input_id = args.input.id
        if self.input_changed:
            self.input_changed(args)

    def command_validate_input(self, args: adsk.core.ValidateInputsEventArgs):
        # Optional per-gear live validation. A generator class may expose a
        # `validate_inputs(inputs) -> list[str]` static method returning the
        # human-readable problems with the current values (empty == valid). When
        # present, invalid geometry disables OK so the dialog stays open and
        # editable; the messages are shown in the hidden TextBox that sits right
        # after the field the user last edited (id == that input's id +
        # STATUS_SUFFIX), so guidance appears where the user is looking. For
        # inputs without their own slot (selections, or the initial open) the
        # gear's DEFAULT_STATUS_INPUT_ID slot is used. Gears that don't define
        # validate_inputs keep the previous always-valid behavior; the
        # execute-time guard in the generator is the backstop in every case.
        validator = getattr(self.generator_class, 'validate_inputs', None)
        if validator is None:
            args.areInputsValid = True
            return

        try:
            problems = validator(args.inputs)
        except Exception:
            # A half-typed or invalid expression can make a value read raise.
            # Don't block on it: Fusion's own expression validation handles that,
            # and the generator re-validates at execute time.
            args.areInputsValid = True
            return

        self._show_status(args.inputs, problems)
        args.areInputsValid = not problems

    def _show_status(self, inputs, problems):
        # Pick the message slot nearest the last-edited field, falling back to
        # the gear's default slot. Hide whichever slot we showed previously so
        # exactly one message is ever visible.
        def slot(slot_id):
            return inputs.itemById(slot_id) if slot_id else None

        target_id = None
        if problems:
            last = getattr(self, '_last_changed_input_id', None)
            if last and slot(last + STATUS_SUFFIX) is not None:
                target_id = last + STATUS_SUFFIX
            else:
                target_id = getattr(
                    self.generator_class, 'DEFAULT_STATUS_INPUT_ID', None)

        prev = getattr(self, '_shown_status_id', None)
        if prev and prev != target_id:
            prev_box = slot(prev)
            if prev_box is not None:
                prev_box.formattedText = ''
                prev_box.isVisible = False

        box = slot(target_id)
        if box is not None:
            box.formattedText = _format_problems(problems)
            box.isVisible = True
            self._shown_status_id = target_id
        else:
            self._shown_status_id = None

    def command_destroy(self, args: adsk.core.CommandEventArgs):
        futil.log(f'{self.name} Command Destroy Event')
        self.local_handlers = []
