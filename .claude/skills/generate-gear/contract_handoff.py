#!/usr/bin/env python3
"""Load, render, replace, and validate a compiled step list's contract handoff."""
import json
import os
import re


HEADING = '## Compilation contract'
CONTRACT_PATH = os.path.join('spec', '{gear}', 'contract.json')
HEADING_LINE = re.compile(r'^##[ \t]+Compilation contract[ \t]*$', re.M)
PROVENANCE_HEADING = re.compile(r'^##[ \t]+Provenance[ \t]*$', re.M)
TIMELINE_HEADING = re.compile(r'^##[ \t]+\S+[ \t]+`\[(?:GO|PROSE)\]`', re.M)
FENCE_RUN = re.compile(r'`+')

TOP_LEVEL_FIELDS = {'_comment', 'module', 'module_constants', 'classes', 'source_guards'}
CLASS_FIELDS = {'bases', 'methods', 'ctx_fields'}
GUARD_FIELDS = {'file', 'in_function', 'why', 'required', 'banned'}


class ContractHandoffError(ValueError):
    pass


def _object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ContractHandoffError('duplicate JSON key %r' % key)
        result[key] = value
    return result


def _strings(value, label):
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise ContractHandoffError('%s must be a list of strings' % label)


def _fields(value, allowed, label):
    if not isinstance(value, dict):
        raise ContractHandoffError('%s must be an object' % label)
    unknown = sorted(set(value) - allowed)
    if unknown:
        raise ContractHandoffError('%s has unknown field %r' % (label, unknown[0]))


def _validate_manifest(manifest):
    _fields(manifest, TOP_LEVEL_FIELDS, 'manifest')
    if '_comment' in manifest and not isinstance(manifest['_comment'], str):
        raise ContractHandoffError('manifest._comment must be a string')
    if 'module' in manifest and not isinstance(manifest['module'], str):
        raise ContractHandoffError('manifest.module must be a string')

    if 'module_constants' in manifest:
        constants = manifest['module_constants']
        if not isinstance(constants, dict) or any(
                not isinstance(name, str) or not isinstance(value, str)
                for name, value in constants.items()):
            raise ContractHandoffError('manifest.module_constants must map strings to strings')

    if 'classes' in manifest:
        classes = manifest['classes']
        if not isinstance(classes, dict):
            raise ContractHandoffError('manifest.classes must be an object')
        for name, declaration in classes.items():
            if not isinstance(name, str):
                raise ContractHandoffError('manifest.classes keys must be strings')
            label = 'manifest.classes[%r]' % name
            _fields(declaration, CLASS_FIELDS, label)
            for field in CLASS_FIELDS:
                if field in declaration:
                    _strings(declaration[field], '%s.%s' % (label, field))

    if 'source_guards' in manifest:
        guards = manifest['source_guards']
        if not isinstance(guards, list):
            raise ContractHandoffError('manifest.source_guards must be a list')
        for index, guard in enumerate(guards):
            label = 'manifest.source_guards[%d]' % index
            _fields(guard, GUARD_FIELDS, label)
            for field in ('file', 'in_function', 'why'):
                if field in guard and not isinstance(guard[field], str):
                    raise ContractHandoffError('%s.%s must be a string' % (label, field))
            for field in ('required', 'banned'):
                if field not in guard:
                    continue
                _strings(guard[field], '%s.%s' % (label, field))
                for pattern in guard[field]:
                    try:
                        re.compile(pattern)
                    except re.error as exc:
                        raise ContractHandoffError(
                            '%s.%s has invalid regex %r: %s' % (label, field, pattern, exc)) from exc
    return manifest


def _parse_json(text):
    try:
        manifest = json.loads(text, object_pairs_hook=_object)
    except ContractHandoffError:
        raise
    except (TypeError, ValueError) as exc:
        raise ContractHandoffError('invalid contract JSON: %s' % exc) from exc
    return _validate_manifest(manifest)


def load_contract(root: str, gear: str) -> dict | None:
    """Load and validate ``spec/<gear>/contract.json`` below root."""
    path = os.path.join(root, CONTRACT_PATH.format(gear=gear))
    try:
        with open(path, encoding='utf-8') as handle:
            text = handle.read()
    except FileNotFoundError:
        return None
    except UnicodeDecodeError as exc:
        raise ContractHandoffError('%s is not valid UTF-8: %s' % (path, exc)) from exc
    return _parse_json(text)


def render_contract(manifest: dict) -> str:
    """Render one complete compilation-contract section."""
    _validate_manifest(manifest)
    body = json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True)
    longest = max((len(match.group(0)) for match in FENCE_RUN.finditer(body)), default=0)
    fence = '`' * max(3, longest + 1)
    return '%s\n\n%sjson\n%s\n%s' % (HEADING, fence, body, fence)


def _section(text, match):
    start = match.start()
    opening = re.match(
        r'## Compilation contract[ \t]*\r?\n[ \t]*\r?\n(?P<fence>`{3,})json[ \t]*\r?\n',
        text[start:])
    if opening is None:
        raise ContractHandoffError('malformed compilation contract section')
    fence = opening.group('fence')
    body_start = start + opening.end()
    closing = re.search(r'^%s[ \t]*(?=\r?$)' % re.escape(fence), text[body_start:], re.M)
    if closing is None:
        raise ContractHandoffError('compilation contract section has no matching closing fence')
    body_end = body_start + closing.start()
    end = body_start + closing.end()
    return start, end, text[body_start:body_end].rstrip('\r\n')


def _headings(pattern, text):
    return list(pattern.finditer(text))


def _placement_problem(text, contract_start):
    provenance = _headings(PROVENANCE_HEADING, text)
    if len(provenance) != 1:
        return 'the step list must carry exactly one ## Provenance heading'
    if contract_start <= provenance[0].start():
        return 'the compilation contract section must follow ## Provenance'
    timeline = TIMELINE_HEADING.search(text)
    if timeline is not None and contract_start >= timeline.start():
        return 'the compilation contract section must precede the first timeline step'
    return None


def replace_contract(text: str, rendered: str) -> str:
    """Replace one contract section, or insert it between provenance and the timeline."""
    headings = _headings(HEADING_LINE, text)
    if len(headings) > 1:
        raise ContractHandoffError('multiple compilation contract sections')
    if headings:
        start, end, _ = _section(text, headings[0])
        problem = _placement_problem(text, start)
        if problem:
            raise ContractHandoffError(problem)
        return text[:start] + rendered + text[end:]

    provenance = _headings(PROVENANCE_HEADING, text)
    if len(provenance) != 1:
        raise ContractHandoffError('the step list must carry exactly one ## Provenance heading')
    following = re.search(r'^##[ \t]+', text[provenance[0].end():], re.M)
    insertion = len(text) if following is None else provenance[0].end() + following.start()
    timeline = TIMELINE_HEADING.search(text)
    if timeline is not None and insertion > timeline.start():
        insertion = timeline.start()
    prefix = text[:insertion]
    separator = '' if prefix.endswith('\n\n') else ('\n' if prefix.endswith('\n') else '\n\n')
    return prefix + separator + rendered + '\n\n' + text[insertion:]


def validate_contract(text: str, root: str, gear: str) -> list[str]:
    """Return content failures for the embedded handoff against the source manifest."""
    manifest = load_contract(root, gear)
    headings = _headings(HEADING_LINE, text)
    if manifest is None:
        if headings:
            return ['the step list carries a compilation contract but no manifest exists']
        return []
    if not headings:
        return ['the step list carries no compilation contract section']
    if len(headings) > 1:
        return ['the step list carries multiple compilation contract sections']
    try:
        start, _, body = _section(text, headings[0])
        embedded = _parse_json(body)
    except ContractHandoffError as exc:
        return ['the compilation contract section is malformed: %s' % exc]
    problems = []
    placement = _placement_problem(text, start)
    if placement:
        problems.append(placement)
    if embedded != manifest:
        problems.append('the compilation contract does not match spec/%s/contract.json' % gear)
    return problems


def mask_contract(text: str) -> str:
    """Blank complete contract sections while preserving every newline and byte offset."""
    masked = list(text)
    for heading in reversed(_headings(HEADING_LINE, text)):
        try:
            start, end, _ = _section(text, heading)
        except ContractHandoffError:
            continue
        for index in range(start, end):
            if masked[index] not in ('\n', '\r'):
                masked[index] = ' '
    return ''.join(masked)
