#!/usr/bin/env python3
"""Validate exact gear values, carry them through steps, and render Python setup."""
import argparse
import ast
import json
import os
import re
import string
import sys

HEADING = '## Exact values'
OPEN = HEADING + '\n\n```json\n'
END = '\n```'
UNITS = {'', 'mm', 'cm', 'deg', 'rad'}
KINDS = {'selection', 'value', 'string', 'boolean'}
EXPRESSION_TOKEN = re.compile(r'^[\d\s.+*/()\-A-Za-z_{}]+$')


class ExactValueError(ValueError):
    pass


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ExactValueError('duplicate JSON key: %s' % key)
        result[key] = value
    return result


def _json(path):
    with open(path, encoding='utf-8') as handle:
        return json.load(handle, object_pairs_hook=_object,
                         parse_constant=lambda x: (_ for _ in ()).throw(ExactValueError(x)))


def _keys(value, expected, label):
    if not isinstance(value, dict) or set(value) != set(expected):
        raise ExactValueError('%s must have exactly: %s' % (label, ', '.join(expected)))


def _text(value, label):
    if not isinstance(value, str) or not value or '\n' in value or '\r' in value:
        raise ExactValueError('%s must be nonempty single-line text' % label)


def _unit(value, label):
    if not isinstance(value, str) or value not in UNITS:
        raise ExactValueError('%s has invalid unit %r' % (label, value))


def validate(data, constants):
    _keys(data, ('schema', 'inputs', 'parameters'), 'exact values')
    if type(data['schema']) is not int or data['schema'] != 1:
        raise ExactValueError('unsupported exact-values schema')
    if not isinstance(data['inputs'], list) or not isinstance(data['parameters'], list):
        raise ExactValueError('inputs and parameters must be arrays')
    input_ids = set()
    input_symbols = set()
    for item in data['inputs']:
        if not isinstance(item, dict) or not isinstance(item.get('kind'), str) or item['kind'] not in KINDS:
            raise ExactValueError('input kind is missing or invalid')
        kind = item['kind']
        fields = {'id', 'kind', 'label'}
        fields |= {
            'selection': {'prompt', 'filters', 'preselect'},
            'value': {'unit', 'default'},
            'string': {'default'},
            'boolean': {'check_box', 'default'},
        }[kind]
        _keys(item, fields, 'input')
        symbol = item['id']
        if symbol not in constants or not symbol.startswith('INPUT_ID_'):
            raise ExactValueError('unknown input constant %r' % symbol)
        value = constants[symbol]
        if value in input_ids or symbol in input_symbols:
            raise ExactValueError('duplicate input id %r' % value)
        input_ids.add(value)
        input_symbols.add(symbol)
        _text(item['label'], '%s label' % symbol)
        if kind == 'selection':
            _text(item['prompt'], '%s prompt' % symbol)
            if (not isinstance(item['filters'], list) or not item['filters'] or
                    any(not isinstance(v, str) or not re.fullmatch(r'[A-Za-z]+', v)
                        for v in item['filters']) or len(set(item['filters'])) != len(item['filters'])):
                raise ExactValueError('%s filters are invalid' % symbol)
            if type(item['preselect']) is not bool:
                raise ExactValueError('%s preselect must be boolean' % symbol)
        elif kind == 'value':
            _unit(item['unit'], '%s display' % symbol)
            default = item['default']
            if not isinstance(default, dict) or len(default) != 1:
                raise ExactValueError('%s default must have one numeric conversion' % symbol)
            conversion, number = next(iter(default.items()))
            if conversion not in ('real', 'radians', 'millimeters') or type(number) not in (int, float):
                raise ExactValueError('%s default conversion is invalid' % symbol)
        elif kind == 'string':
            _text(item['default'], '%s default' % symbol)
        elif type(item['check_box']) is not bool or type(item['default']) is not bool:
            raise ExactValueError('%s check_box and default must be boolean' % symbol)
    expected_inputs = {key for key in constants if key.startswith('INPUT_ID_')}
    if input_symbols != expected_inputs:
        raise ExactValueError('input constants missing or extra: %s' % sorted(input_symbols ^ expected_inputs))

    names = set()
    resolved_names = set()
    dependencies = {}
    for item in data['parameters']:
        if not isinstance(item, dict):
            raise ExactValueError('parameter must be an object')
        source = set(item) & {'input', 'expression', 'computed'}
        if len(source) != 1:
            raise ExactValueError('parameter needs exactly one source')
        field = source.pop()
        _keys(item, {'name', 'unit', 'comment', field}, 'parameter')
        symbol = item['name']
        if symbol not in constants or not symbol.startswith('PARAM_') or symbol in names:
            raise ExactValueError('unknown or duplicate parameter %r' % symbol)
        if constants[symbol] in resolved_names:
            raise ExactValueError('duplicate parameter name %r' % constants[symbol])
        names.add(symbol)
        resolved_names.add(constants[symbol])
        _unit(item['unit'], '%s parameter' % symbol)
        _text(item['comment'], '%s comment' % symbol)
        if field == 'input':
            if item[field] not in input_symbols:
                raise ExactValueError('%s refers to unknown input %r' % (symbol, item[field]))
            dependencies[symbol] = set()
        elif field == 'computed':
            if item[field] != 'tooth_space_angle':
                raise ExactValueError('%s has unsupported computation' % symbol)
            dependencies[symbol] = {'PARAM_TOOTH_NUMBER', 'PARAM_PRESSURE_ANGLE'}
        else:
            expression = item[field]
            if not isinstance(expression, str) or not EXPRESSION_TOKEN.fullmatch(expression):
                raise ExactValueError('%s has invalid expression' % symbol)
            try:
                fields = [name for _, name, _, _ in string.Formatter().parse(expression) if name]
            except ValueError as exc:
                raise ExactValueError('%s has invalid expression brackets' % symbol) from exc
            literals = re.sub(r'\{[^}]+\}', '0', expression)
            if set(re.findall(r'[A-Za-z_]+', literals)) - {'cos'}:
                raise ExactValueError('%s has an unknown expression function or name' % symbol)
            allowed = set(constants) | {'fillet_helix_factor'}
            if set(fields) - allowed:
                raise ExactValueError('%s refers to unknown parameter %s' % (symbol, sorted(set(fields) - allowed)))
            if any(not name.startswith('PARAM_') and name != 'fillet_helix_factor' for name in fields):
                raise ExactValueError('%s expression refers to an input' % symbol)
            dependencies[symbol] = set(fields) - {'fillet_helix_factor'}
    expected_params = {key for key in constants if key.startswith('PARAM_')}
    if names != expected_params:
        raise ExactValueError('parameter constants missing or extra: %s' % sorted(names ^ expected_params))
    seen = set()
    for item in data['parameters']:
        name = item['name']
        if not dependencies[name] <= seen:
            raise ExactValueError('%s has a forward reference or dependency cycle: %s' %
                                  (name, sorted(dependencies[name] - seen)))
        seen.add(name)
    return data


def load(root, gear):
    path = os.path.join(root, 'spec', gear, 'exact_values.json')
    if not os.path.isfile(path):
        return None
    contract = _json(os.path.join(root, 'spec', gear, 'contract.json'))
    constants = contract['module_constants']
    return validate(_json(path), constants), constants


def payload(data, constants):
    return {'schema': 1, 'constants': {k: v for k, v in constants.items()
                                       if k.startswith(('INPUT_ID_', 'PARAM_'))},
            'inputs': data['inputs'], 'parameters': data['parameters']}


def section(data, constants):
    return OPEN + json.dumps(payload(data, constants), ensure_ascii=False, indent=2) + END


def _section_bounds(steps):
    matches = list(re.finditer(r'^## Exact values\s*$', steps, re.M))
    if len(matches) > 1:
        raise ExactValueError('multiple exact-value sections')
    if not matches:
        return None
    start = matches[0].start()
    next_heading = re.search(r'^## ', steps[matches[0].end():], re.M)
    end = matches[0].end() + next_heading.start() if next_heading else len(steps)
    return start, end


def sync_steps(steps, data, constants):
    rendered = section(data, constants) + '\n\n'
    bounds = _section_bounds(steps)
    if bounds:
        return steps[:bounds[0]] + rendered + steps[bounds[1]:]
    first_step = re.search(r'^## \S+ `\[(?:GO|PROSE)\]`', steps, re.M)
    if first_step is None:
        raise ExactValueError('step list has no timeline')
    return steps[:first_step.start()] + rendered + steps[first_step.start():]


def check_steps(steps, data, constants):
    bounds = _section_bounds(steps)
    if not bounds:
        raise ExactValueError('compiled steps have no exact-value handoff')
    embedded = steps[bounds[0]:bounds[1]].strip()
    if embedded != section(data, constants):
        raise ExactValueError('compiled exact values differ from spec source or contract')
    return payload(data, constants)


def _default(item):
    value = item['default']
    kind, number = next(iter(value.items()))
    if kind == 'radians':
        return 'math.radians(%s)' % repr(number)
    if kind == 'millimeters':
        return 'to_cm(%s)' % repr(number)
    return repr(number)


def _configure(data):
    lines = ['        inputs: adsk.core.CommandInputs = cmd.commandInputs']
    for index, item in enumerate(data['inputs']):
        symbol = item['id']
        kind = item['kind']
        if kind == 'selection':
            var = 'selection%d' % index
            lines.append('        %s: adsk.core.SelectionCommandInput = inputs.addSelectionInput(%s, %r, %r)' %
                         (var, symbol, item['label'], item['prompt']))
            lines.extend('        %s.addSelectionFilter(adsk.core.SelectionCommandInput.%s)' % (var, value)
                         for value in item['filters'])
            lines.append('        %s.setSelectionLimits(1, 1)' % var)
            if item['preselect']:
                lines.append('        %s.addSelection(get_design().rootComponent)' % var)
        elif kind == 'value':
            lines.append('        inputs.addValueInput(%s, %r, %r, adsk.core.ValueInput.createByReal(%s))' %
                         (symbol, item['label'], item['unit'], _default(item)))
        elif kind == 'string':
            lines.append('        inputs.addStringValueInput(%s, %r, %r)' %
                         (symbol, item['label'], item['default']))
        else:
            lines.append('        inputs.addBoolValueInput(%s, %r, %s, %r, %s)' %
                         (symbol, item['label'], item['check_box'], '', item['default']))
    return '\n'.join(lines)


def _primary(data):
    inputs = {item['id']: item for item in data['inputs']}
    lines = []
    for item in data['parameters']:
        if 'input' not in item:
            continue
        source = item['input']
        kind = inputs[source]['kind']
        if kind == 'boolean':
            value = ('adsk.core.ValueInput.createByReal('
                     '1 if get_boolean(inputs, %s) else 0)' % source)
        else:
            value = 'get_value(inputs, %s, %r)' % (source, item['unit'])
        lines.append('        self.addParameter(\n'
                     '            %s, %s,\n'
                     '            %r, %r)' %
                     (item['name'], value, item['unit'], item['comment']))
    lines.extend(['        self.addExtraPrimaryParameters(inputs)',
                  '        self.registerDerivedParameters()'])
    return '\n'.join(lines)


def _derived(data):
    lines = []
    for item in data['parameters']:
        if 'input' in item:
            continue
        if 'computed' in item:
            lines.extend([
                '        toothNumber = self.getParameter(PARAM_TOOTH_NUMBER).value',
                '        pressureAngle = self.getParameter(PARAM_PRESSURE_ANGLE).value',
                '        toothSpaceAngle = math.pi / toothNumber - 2 * (math.tan(pressureAngle) - pressureAngle)',
            ])
            value = 'adsk.core.ValueInput.createByReal(toothSpaceAngle)'
        else:
            expression = item['expression']
            fields = [name for _, name, _, _ in string.Formatter().parse(expression) if name]
            arguments = []
            for field in fields:
                if field == 'fillet_helix_factor':
                    arguments.append('self.filletHelixFactorExpression()')
                else:
                    arguments.append('self.parameterName(%s)' % field)
            format_string = re.sub(r'\{[^}]+\}', '{}', expression)
            if arguments:
                lines.append('        expression = %r.format(\n%s        )' %
                             (format_string, ''.join('            %s,\n' % arg for arg in arguments)))
            else:
                lines.append('        expression = %r' % expression)
            value = 'adsk.core.ValueInput.createByString(expression)'
        lines.append('        self.addParameter(\n'
                     '            %s, %s,\n'
                     '            %r, %r)' %
                     (item['name'], value, item['unit'], item['comment']))
    return '\n'.join(lines)


def _method(tree, cls, name):
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == cls:
            for method in node.body:
                if isinstance(method, ast.FunctionDef) and method.name == name:
                    return method
    raise ExactValueError('candidate lacks %s.%s' % (cls, name))


def render_module(source, handoff):
    """Replace only spur's exact setup; keep geometry and selection handling."""
    data = {'inputs': handoff['inputs'], 'parameters': handoff['parameters']}
    constants = handoff['constants']
    tree = ast.parse(source)
    lines = source.splitlines(keepends=True)
    positions = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in constants:
                positions[name] = (node.lineno, node.end_lineno)
    if positions and set(positions) != set(constants):
        raise ExactValueError('candidate declares only part of the exact constants')
    const_source = '\n'.join('%s = %r' % (name, value) for name, value in constants.items()) + '\n'
    if positions:
        start = min(v[0] for v in positions.values())
        end = max(v[1] for v in positions.values())
        for node in tree.body:
            if start <= node.lineno <= end and not (
                    isinstance(node, ast.Assign) and len(node.targets) == 1 and
                    isinstance(node.targets[0], ast.Name) and node.targets[0].id in constants):
                raise ExactValueError('candidate interleaves non-constant code with exact constants')
        replacements = [(start, end, const_source)]
    else:
        first_class = next((node for node in tree.body if isinstance(node, ast.ClassDef)), None)
        if first_class is None:
            raise ExactValueError('candidate has no class before which to insert constants')
        insertion = first_class.decorator_list[0].lineno if first_class.decorator_list else first_class.lineno
        replacements = [(insertion, insertion - 1, const_source + '\n')]
    config = _method(tree, 'SpurGearCommandInputsConfigurator', 'configure')
    replacements.append((config.body[0].lineno, config.body[-1].end_lineno, _configure(data) + '\n'))
    process = _method(tree, 'SpurGearGenerator', 'processInputs')
    calls = [node for node in process.body if isinstance(node, ast.Expr) and
             isinstance(node.value, ast.Call) and isinstance(node.value.func, ast.Attribute)]
    first = next((node for node in calls if node.value.func.attr in (
        'addParameter', 'addExtraPrimaryParameters', 'registerDerivedParameters')), None)
    last = next((node for node in reversed(calls)
                 if node.value.func.attr == 'registerDerivedParameters'), None)
    if first is None or last is None or first.lineno > last.lineno:
        raise ExactValueError('candidate processInputs has no parameter setup boundary')
    between = [node for node in process.body if first.lineno <= node.lineno <= last.lineno]
    for node in between:
        if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call) and isinstance(
                node.value.func, ast.Attribute) and node.value.func.attr in (
                    'addParameter', 'addExtraPrimaryParameters', 'registerDerivedParameters'):
            continue
        if (isinstance(node, ast.Assign) and len(node.targets) == 1 and
                isinstance(node.targets[0], ast.Name) and node.targets[0].id == 'sketchOnly'):
            continue
        raise ExactValueError('candidate interleaves non-parameter code with exact setup')
    replacements.append((first.lineno, last.end_lineno, _primary(data) + '\n'))
    derived = _method(tree, 'SpurGearGenerator', 'registerDerivedParameters')
    replacements.append((derived.body[0].lineno, derived.body[-1].end_lineno, _derived(data) + '\n'))
    for start, end, replacement in sorted(replacements, reverse=True):
        lines[start - 1:end] = [replacement]
    result = ''.join(lines)
    ast.parse(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('gear')
    parser.add_argument('action', choices=('sync-steps', 'check-steps', 'render', 'check-module'))
    parser.add_argument('candidate', nargs='?')
    args = parser.parse_args()
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    try:
        loaded = load(root, args.gear)
        if loaded is None:
            raise ExactValueError('no exact-value source for %s' % args.gear)
        data, constants = loaded
        steps_path = os.path.join(root, 'spec', args.gear, 'steps.md')
        with open(steps_path, encoding='utf-8') as handle:
            steps = handle.read()
        if args.action == 'sync-steps':
            with open(steps_path, 'w', encoding='utf-8') as handle:
                handle.write(sync_steps(steps, data, constants))
            return 0
        handoff = check_steps(steps, data, constants)
        if args.action == 'check-steps':
            return 0
        if args.gear != 'spurgear' or not args.candidate:
            raise ExactValueError('render/check-module require spur and a candidate path')
        with open(args.candidate, encoding='utf-8') as handle:
            candidate = handle.read()
        rendered = render_module(candidate, handoff)
        if args.action == 'check-module':
            if rendered != candidate:
                raise ExactValueError('candidate setup differs from checked exact-value handoff')
        else:
            with open(args.candidate, 'w', encoding='utf-8') as handle:
                handle.write(rendered)
        return 0
    except (ExactValueError, OSError, ValueError, SyntaxError) as exc:
        print('exact-value check: %s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
