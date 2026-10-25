# Shared implementation of a gear-generator command. Each commands/<gear>/entry.py
# declares one GearCommand and re-exports its start/stop (see those modules).

import importlib
import sys
import types
import adsk.core
import adsk.fusion
from ..lib import fusion360utils as futil
from ..lib import geargen
from ..lib.geargen import decadfusion as _runtime
from .. import config

ui = adsk.core.Application.get().userInterface

# Bump this string on every deploy of THIS file. It proves whether the
# commands/ layer reloaded: if the dialog's Build line shows an old tag, the
# add-in Stop/Run did not re-import commands/_gear_command.py.
BUILD_TAG = 'r8-2026-10-25'

# Fusion keeps imported modules across an add-in Stop and Run, so a deploy can leave
# old code running [PB-STALE-MODULE]. Each geargen module is recorded with the hash of
# its file when it was loaded, and with the reload count at that time. Before a build
# the command reloads, in dependency order, every module the generator uses whose
# file changed or which uses a module reloaded after it.
_PACKAGE = _runtime.__name__.rpartition('.')[0]
_LOADED = {}
_RELOADS = {'count': 0}


def _file_hash(module):
    path = getattr(module, '__file__', None)
    return _runtime.source_hash(path) if path else None


def _record(module, loaded=None):
    _LOADED[module.__name__] = (loaded if loaded is not None else _file_hash(module), _RELOADS['count'])


def _loaded(module):
    if module.__name__ not in _LOADED:
        _record(module)
    return _LOADED[module.__name__]


for _name, _module in list(sys.modules.items()):
    if _name.startswith(_PACKAGE + '.') and _module is not None:
        # The runtime hashed its file when it was imported, which is the closest record of the code loaded.
        _record(_module, _runtime.RUNTIME_BUILD if _module is _runtime else None)


def _module_of(value):
    """The geargen module a value is or comes from, or None."""
    if isinstance(value, types.ModuleType):
        module = value
    else:
        try:
            name = getattr(value, '__module__', None)
        except Exception:
            return None
        module = sys.modules.get(name) if isinstance(name, str) else None
    if module is None or not module.__name__.startswith(_PACKAGE + '.'):
        return None
    return module


def _uses(module):
    """The geargen modules a module uses: the ones it imported and the ones its imported names come from."""
    used = []
    for value in list(vars(module).values()):
        other = _module_of(value)
        if other is not None and other is not module and not any(other is u for u in used):
            used.append(other)
    return used


def _gear_modules(generator_class):
    """The geargen modules the generator's module uses, its own last, each after the modules it uses."""
    order = []
    visiting = set()

    def visit(module):
        if module.__name__ in visiting:
            return
        visiting.add(module.__name__)
        for used in _uses(module):
            visit(used)
        order.append(module)

    visit(sys.modules[generator_class.__module__])
    return order


def _staleness(modules):
    """For each module, None when it matches its file and nothing it uses was reloaded after it, else the reason."""
    reasons = []
    for module in modules:
        loaded, generation = _loaded(module)
        if _file_hash(module) != loaded:
            reasons.append('file on disk changed')
        elif any(_loaded(used)[1] > generation for used in _uses(module)):
            reasons.append('uses a module reloaded after it')
        else:
            reasons.append(None)
    return reasons


def fresh_generator_class(generator_class):
    """The generator class as the files on disk define it. Reloads the stale geargen modules the generator uses, in
    dependency order, and raises when one still differs from its file afterwards."""
    modules = _gear_modules(generator_class)
    reasons = _staleness(modules)
    stale = [i for i, reason in enumerate(reasons) if reason is not None]
    if stale:
        for module in modules[stale[0]:]:
            importlib.reload(module)
            _RELOADS['count'] += 1
            _record(module)
            futil.log(f'reloaded {module.__name__} ({reasons[modules.index(module)] or "after a module it uses"})')
        left = [m.__name__ for m, reason in zip(modules, _staleness(modules)) if reason is not None]
        if left:
            raise RuntimeError('STALE geargen modules after a reload, restart Fusion: ' + ', '.join(left))
    fresh = getattr(sys.modules[generator_class.__module__], generator_class.__name__, None)
    if fresh is None:
        raise RuntimeError(f'{generator_class.__module__} no longer defines {generator_class.__name__}')
    return fresh


def _build_label(generator_class):
    """The command tag, and each geargen module the generator uses with the id of the code loaded: the build id a
    generated module logs (gearport id), or the hash of the file for a hand-written one."""
    try:
        modules = _gear_modules(generator_class)
        parts = []
        for module, reason in zip(modules, _staleness(modules)):
            loaded = getattr(module, 'GEAR_BUILD_ID', None) or _loaded(module)[0]
            parts.append('{} {}{}'.format(module.__name__.rpartition('.')[2], loaded,
                                          '' if reason is None else ' STALE ({})'.format(reason)))
        body = ' | '.join(parts)
    except Exception as error:
        body = 'module versions could not be read ({})'.format(error)
    return 'Build: command {} | {}'.format(BUILD_TAG, body)


# A gear opts into live validation by giving each editable input a trailing,
# hidden read-only TextBox whose id is the input's id + this suffix; the handler
# below shows the one nearest the field the user just edited.
STATUS_SUFFIX = '__status'


def _can_create_components(design):
    return design is not None and design.designIntent in (
        adsk.fusion.DesignIntentTypes.HybridDesignIntentType,
        adsk.fusion.DesignIntentTypes.AssemblyDesignIntentType,
    )


def _component_design_or_message():
    try:
        design = geargen.get_design()
    except Exception:
        design = None

    if _can_create_components(design):
        return design

    ui.messageBox(
        'Gear Generator needs an active Hybrid or Assembly design.\n\n'
        'Open one or convert the current design, then try again.',
        'Unsupported design type')
    return None


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

        if _component_design_or_message() is None:
            return

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
            design = _component_design_or_message()
            if design is None:
                return
            # Fusion keeps imported modules across an add-in Stop and Run, so the runtime and the generated
            # modules are reloaded from the files on disk when they changed. [PB-STALE-MODULE]
            self.generator_class = fresh_generator_class(self.generator_class)
            futil.log(f'{self.name} {_build_label(self.generator_class)}')
            # [PB-PROGRESS-FILE]: the progress file holds the steps of this build only.
            _runtime.progress_reset()
            # [PB-SETTLE-DISPLAY]: remember the user's own sketches, so the settle below only
            # touches the ones this generator is about to author.
            preexisting = geargen.existing_sketches(design)
            g = self.generator_class(design)
            g.generate(args.command.commandInputs)
            # Every gear runs through here, so one call covers all of them and no generated
            # module carries its own copy.
            settled = geargen.settle_sketch_display(design, preexisting)
            futil.log(f'{self.name} settled the display of {settled} new sketches')
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
