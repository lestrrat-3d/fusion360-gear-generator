#!/usr/bin/env python3
"""Render typed entry points for a fresh generated gear module without importing it."""
import ast
import os
import re
import sys
import tempfile
from pathlib import Path

import contract_handoff


GEAR_RE = re.compile(r'^[a-z][a-z0-9_-]*$')
USAGE = 'usage: render_emitter_interfaces.py <gear> --out <path>'

CONVENTIONS = (
    'Keep entry-point parameter annotations from this sheet.',
    'Give a local Fusion API object an explicit qualified annotation when its type cannot be '
    'inferred from a typed receiver.',
    'Check optional objects before use. A null check remains runtime behavior, not a type-only '
    'cast.',
    'Do not use `typing.cast` solely to make an API receiver recognizable to the custom checker. '
    'Use a typed parameter or checked local binding instead.',
    'Calls on refuted or unverified API pairs still follow the shared API policy.',
    'Treat this sheet as advisory generation scaffolding. Do not override a conflicting source '
    'contract.',
)


class RenderError(ValueError):
    """A bad input or incompatible framework shape."""


def _read_tree(path):
    try:
        text = path.read_text(encoding='utf-8')
    except OSError as exc:
        raise RenderError('cannot read {}: {}'.format(path, exc)) from exc
    except UnicodeDecodeError as exc:
        raise RenderError('{} is not valid UTF-8: {}'.format(path, exc)) from exc
    try:
        return ast.parse(text, filename=str(path))
    except SyntaxError as exc:
        raise RenderError('cannot parse {}: {}'.format(path, exc)) from exc


def _class_method(tree, class_name, method_name):
    classes = [node for node in tree.body
               if isinstance(node, ast.ClassDef) and node.name == class_name]
    if len(classes) != 1:
        raise RenderError('{} must declare exactly one class {}'.format(
            getattr(tree, '_source_path', 'framework source'), class_name))
    methods = [node for node in classes[0].body
               if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
               and node.name == method_name]
    if len(methods) != 1:
        raise RenderError('{} must declare exactly one {}.{}'.format(
            getattr(tree, '_source_path', 'framework source'), class_name, method_name))
    if isinstance(methods[0], ast.AsyncFunctionDef):
        raise RenderError('{}.{} must be synchronous'.format(class_name, method_name))
    return methods[0]


def _annotation(node):
    return ': {}'.format(ast.unparse(node)) if node is not None else ''


def _default(node):
    return ' = {}'.format(ast.unparse(node)) if node is not None else ''


def _arguments(arguments):
    positional = list(arguments.posonlyargs) + list(arguments.args)
    first_default = len(positional) - len(arguments.defaults)
    parts = []
    for index, argument in enumerate(positional):
        default = arguments.defaults[index - first_default] if index >= first_default else None
        parts.append(argument.arg + _annotation(argument.annotation) + _default(default))
        if arguments.posonlyargs and index + 1 == len(arguments.posonlyargs):
            parts.append('/')
    if arguments.vararg is not None:
        parts.append('*' + arguments.vararg.arg + _annotation(arguments.vararg.annotation))
    elif arguments.kwonlyargs:
        parts.append('*')
    for argument, default in zip(arguments.kwonlyargs, arguments.kw_defaults):
        parts.append(argument.arg + _annotation(argument.annotation) + _default(default))
    if arguments.kwarg is not None:
        parts.append('**' + arguments.kwarg.arg + _annotation(arguments.kwarg.annotation))
    return ', '.join(parts)


def _method_signature(node):
    suffix = ' -> {}'.format(ast.unparse(node.returns)) if node.returns is not None else ''
    return 'def {}({}){}: ...'.format(node.name, _arguments(node.args), suffix)


def _validate_configure_callsite(command_tree):
    method = _class_method(command_tree, 'GearCommand', 'command_created')
    positional = list(method.args.posonlyargs) + list(method.args.args)
    parameters = [argument for argument in positional if argument.arg != 'self']
    typed = [argument for argument in parameters
             if argument.annotation is not None
             and ast.unparse(argument.annotation) == 'adsk.core.CommandCreatedEventArgs']
    if len(typed) != 1:
        raise RenderError(
            'GearCommand.command_created must have one adsk.core.CommandCreatedEventArgs parameter')
    parameter = typed[0].arg

    for node in ast.walk(method):
        if not isinstance(node, ast.Call) or node.keywords or len(node.args) != 1:
            continue
        function = node.func
        argument = node.args[0]
        if (isinstance(function, ast.Attribute) and function.attr == 'configure'
                and isinstance(function.value, ast.Attribute)
                and function.value.attr == 'configurator'
                and isinstance(function.value.value, ast.Name)
                and function.value.value.id == 'self'
                and isinstance(argument, ast.Attribute) and argument.attr == 'command'
                and isinstance(argument.value, ast.Name) and argument.value.id == parameter):
            return
    raise RenderError(
        'GearCommand.command_created must call self.configurator.configure({}.command)'.format(
            parameter))


def _class_header(name, bases):
    return 'class {}{}:'.format(name, '({})'.format(', '.join(bases)) if bases else '')


def _render_class(name, bases, signatures, other_methods=()):
    lines = ['```python', _class_header(name, bases)]
    for decorators, signature in signatures:
        for decorator in decorators:
            lines.append('    {}'.format(decorator))
        lines.append('    {}'.format(signature))
    lines.append('```')
    if other_methods:
        lines.extend(('Other required methods:', ''))
        lines.extend('- `{}`'.format(item) for item in other_methods)
    return '\n'.join(lines)


def render(gear, root):
    root = Path(root)
    base_path = root / 'lib' / 'geargen' / 'base.py'
    command_path = root / 'commands' / '_gear_command.py'
    base_tree = _read_tree(base_path)
    command_tree = _read_tree(command_path)
    base_tree._source_path = base_path
    command_tree._source_path = command_path
    generate = _class_method(base_tree, 'Generator', 'generate')
    _validate_configure_callsite(command_tree)

    try:
        manifest = contract_handoff.load_contract(str(root), gear)
    except (OSError, contract_handoff.ContractHandoffError) as exc:
        raise RenderError(str(exc)) from exc

    sections = []
    if manifest is None:
        sections.append(_render_class(
            'Generator', (), (((), _method_signature(generate)),)))
    else:
        classes = manifest.get('classes', {})
        for name in sorted(classes):
            declaration = classes[name]
            bases = declaration.get('bases', [])
            methods = declaration.get('methods', [])
            signatures = []
            included = set()
            if 'configure' in methods:
                signatures.append((('@classmethod',),
                                   'def configure(cls, cmd: adsk.core.Command) -> None: ...'))
                included.add('configure')
            if 'Generator' in bases and 'generate' in methods:
                signatures.append(((), _method_signature(generate)))
                included.add('generate')
            if signatures:
                sections.append(_render_class(
                    name, bases, signatures,
                    [method for method in methods if method not in included]))

    lines = ['# Emitter interfaces for `{}`'.format(gear), '']
    if sections:
        lines.extend(['\n\n'.join(sections), ''])
    lines.extend(['## Typing conventions', ''])
    lines.extend('{}. {}'.format(index, convention)
                 for index, convention in enumerate(CONVENTIONS, 1))
    return '\n'.join(lines) + '\n'


def _atomic_write(path, text):
    path = Path(path)
    descriptor, temporary = tempfile.mkstemp(
        prefix='.{}.'.format(path.name), dir=str(path.parent))
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as handle:
            descriptor = -1
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except Exception:
        if descriptor >= 0:
            os.close(descriptor)
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def main(argv=None, repo_root=None):
    argv = list(sys.argv if argv is None else argv)
    args = argv[1:]
    if len(args) != 3 or args[1] != '--out' or GEAR_RE.fullmatch(args[0]) is None:
        sys.stderr.write(USAGE + '\n')
        return 2
    gear, output_text = args[0], args[2]
    root = Path(repo_root) if repo_root is not None else Path(__file__).resolve().parents[3]
    try:
        sheet = render(gear, root)
        output = Path(output_text)
        output.parent.mkdir(parents=True, exist_ok=True)
        _atomic_write(output, sheet)
    except (OSError, RenderError) as exc:
        sys.stderr.write('render_emitter_interfaces: {}\n'.format(exc))
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
