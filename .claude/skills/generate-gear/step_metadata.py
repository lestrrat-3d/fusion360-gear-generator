#!/usr/bin/env python3
"""Parse and render the metadata shared by compiled gear steps."""
import json
import os
import posixpath
import re

from contract_handoff import mask_contract


PATH_REF = r'[\w./-]+\.(?:md|go|py|json|sh)'
STEP_HEADING = re.compile(r'^##\s+(\S+)\s+`\[(GO|PROSE)\]`\s+(.*)$', re.M)
FILE_MARKER = re.compile(r'^<!-- step-metadata: ([0-9]+) -->$', re.M)
FILE_MARKER_RESERVED = re.compile(r'<!--[ \t\r\n]*step-metadata\b')
STEP_META_OPEN = '<!-- step-meta'
FROM_MARKER = re.compile(r'^\*\*From:\*\*', re.M)
FROM_BLOCK = re.compile(
    r'^\*\*From:\*\*.*?(?=\r?\n[ \t]*\r?\n|(?:\r?\n)?\Z)', re.M | re.S)


class MetadataError(ValueError):
    """A step list's metadata does not match its declared schema."""


def steps_of(src):
    """Split the step list into (id, tag, body) triples, in file order."""
    out = []
    heads = list(STEP_HEADING.finditer(src))
    for i, match in enumerate(heads):
        end = heads[i + 1].start() if i + 1 < len(heads) else len(src)
        # The title counts as part of the step. A step may name its proof function
        # there rather than in the prose below, and both readings are reasonable.
        out.append((match.group(1), match.group(2),
                    match.group(3) + '\n' + src[match.end():end]))
    return out


def file_version(text: str) -> int | None:
    """Return the declared metadata version, None for legacy, or reject a bad marker."""
    inspected = mask_contract(text)
    reserved = list(FILE_MARKER_RESERVED.finditer(inspected))
    if not reserved:
        return None
    if len(reserved) != 1:
        raise MetadataError('the step list carries repeated step-metadata markers')
    markers = list(FILE_MARKER.finditer(inspected))
    if len(markers) != 1 or markers[0].start() != reserved[0].start():
        raise MetadataError('the step list carries a malformed step-metadata marker')
    marker = markers[0]
    first_step = STEP_HEADING.search(text)
    if first_step is not None and marker.start() > first_step.start():
        raise MetadataError('the step-metadata marker must appear before the first step')

    version = int(marker.group(1))
    if version != 1:
        raise MetadataError('unsupported step-metadata version %d; expected version 1' % version)
    return version


def _json_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise MetadataError('metadata JSON repeats key %r' % key)
        result[key] = value
    return result


def _reject_constant(value):
    raise MetadataError('metadata JSON contains non-finite number %s' % value)


def _metadata_comment(body):
    """Return (payload text, comment start, comment end) for one exact comment."""
    candidate_lines = []
    offset = 0
    for line in body.splitlines(keepends=True):
        content = line.rstrip('\r\n')
        stripped = content.lstrip(' \t')
        if stripped.startswith('<!-- step-meta'):
            candidate_lines.append((offset, content))
        offset += len(line)
    if not candidate_lines:
        raise MetadataError('has no step-meta payload')
    if len(candidate_lines) != 1:
        raise MetadataError('carries repeated step-meta payloads')

    start, opening = candidate_lines[0]
    if opening != STEP_META_OPEN:
        raise MetadataError('has a malformed step-meta opening line')
    after_opening = start + len(STEP_META_OPEN)
    if body.startswith('\r\n', after_opening):
        newline = '\r\n'
    elif body.startswith('\n', after_opening):
        newline = '\n'
    else:
        raise MetadataError('has a malformed step-meta opening line')
    payload_start = after_opening + len(newline)

    closing = re.search(r'^-->(?=\r?$)', body[payload_start:], re.M)
    if closing is None:
        raise MetadataError('has an unterminated step-meta payload')
    close_start = payload_start + closing.start()
    close_end = payload_start + closing.end()
    return body[payload_start:close_start], start, close_end, newline


def _require_exact_keys(value, expected, label):
    if not isinstance(value, dict):
        raise MetadataError('%s must be a JSON object' % label)
    actual = set(value)
    wanted = set(expected)
    missing = sorted(wanted - actual)
    unknown = sorted(actual - wanted)
    if missing:
        raise MetadataError('%s omits required key(s): %s' % (label, ', '.join(missing)))
    if unknown:
        raise MetadataError('%s has unknown key(s): %s' % (label, ', '.join(unknown)))


def _validate_payload(payload, version):
    if isinstance(version, bool) or not isinstance(version, int) or version != 1:
        raise MetadataError('unsupported step-metadata version %r; expected version 1' % version)
    _require_exact_keys(payload, ('schema', 'citations'), 'step metadata')
    schema = payload['schema']
    if isinstance(schema, bool) or not isinstance(schema, int) or schema != version:
        raise MetadataError('step metadata schema must be integer %d' % version)
    citations = payload['citations']
    if not isinstance(citations, list) or not citations:
        raise MetadataError('step metadata citations must be a nonempty array')

    seen = set()
    for index, citation in enumerate(citations, 1):
        label = 'citation %d' % index
        _require_exact_keys(citation, ('path', 'first', 'last'), label)
        path = citation['path']
        if not isinstance(path, str) or '\n' in path or '\r' in path or '-->' in path:
            raise MetadataError('%s path must be single-line text without -->' % label)
        if re.fullmatch(PATH_REF, path) is None:
            raise MetadataError('%s path %r does not match the repository path syntax' % (label, path))
        if path.startswith('/') or posixpath.normpath(path) != path or '..' in path.split('/'):
            raise MetadataError('%s path %r is not a normalized repository-relative path' % (label, path))
        first = citation['first']
        last = citation['last']
        if isinstance(first, bool) or not isinstance(first, int):
            raise MetadataError('%s first must be an integer' % label)
        if isinstance(last, bool) or not isinstance(last, int):
            raise MetadataError('%s last must be an integer' % label)
        key = (path, first, last)
        if key in seen:
            raise MetadataError('%s repeats an identical citation' % label)
        seen.add(key)


def parse_step(body: str, version: int) -> dict:
    """Parse and structurally validate one step's metadata payload."""
    payload_text, _, metadata_end, _ = _metadata_comment(body)
    from_markers = list(FROM_MARKER.finditer(body))
    if len(from_markers) > 1:
        raise MetadataError('carries repeated **From:** blocks')
    from_blocks = list(FROM_BLOCK.finditer(body))
    if from_blocks and from_blocks[0].start() < metadata_end:
        raise MetadataError('step-meta payload must appear before **From:**')
    try:
        payload = json.loads(
            payload_text, object_pairs_hook=_json_object, parse_constant=_reject_constant)
    except MetadataError:
        raise
    except (json.JSONDecodeError, TypeError) as exc:
        raise MetadataError('has invalid step-meta JSON: %s' % exc) from exc
    _validate_payload(payload, version)
    return payload


def _source_line_count(path):
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            return len(handle.read().splitlines())
    except UnicodeDecodeError as exc:
        raise OSError('%s is not valid UTF-8' % path) from exc


def validate_citations(payload: dict, root: str) -> list[str]:
    """Return every source-path or line-range problem in declaration order."""
    _validate_payload(payload, 1)
    root_path = os.path.realpath(os.path.abspath(root))
    problems = []
    for citation in payload['citations']:
        path = citation['path']
        first = citation['first']
        last = citation['last']
        label = '%s L%d' % (path, first) if first == last else '%s L%d–%d' % (path, first, last)
        if first < 1 or last < 1:
            problems.append('cites %s, but line numbers start at 1' % label)
            continue
        if first > last:
            problems.append('cites %s, but the first line is after the last line' % label)
            continue
        source = os.path.realpath(os.path.join(root_path, *path.split('/')))
        try:
            inside = os.path.commonpath((root_path, source)) == root_path
        except ValueError:
            inside = False
        if not inside:
            problems.append('cites %s, which resolves outside the worktree' % path)
            continue
        try:
            total = _source_line_count(source)
        except FileNotFoundError:
            problems.append('cites %s, which does not exist' % path)
            continue
        if last > total:
            problems.append('cites %s, but that file has %d lines' % (label, total))
    return problems


def render_from(payload: dict) -> str:
    """Render one validated payload's canonical From line."""
    _validate_payload(payload, 1)
    rendered = []
    for citation in payload['citations']:
        suffix = 'L%d' % citation['first']
        if citation['first'] != citation['last']:
            suffix = 'L%d–%d' % (citation['first'], citation['last'])
        rendered.append('`%s` %s' % (citation['path'], suffix))
    return '**From:** %s.' % '; '.join(rendered)


def _render_body(body, payload):
    canonical = render_from(payload)
    if len(FROM_MARKER.findall(body)) > 1:
        raise MetadataError('carries repeated **From:** blocks')
    matches = list(FROM_BLOCK.finditer(body))
    if matches:
        match = matches[0]
        return body[:match.start()] + canonical + body[match.end():]

    _, _, metadata_end, newline = _metadata_comment(body)
    suffix = body[metadata_end:]
    insertion = newline * 2 + canonical
    if suffix.startswith(newline * 2) or not suffix:
        pass
    elif suffix.startswith(newline):
        insertion += newline
    else:
        insertion += newline * 2
    return body[:metadata_end] + insertion + suffix


def render_steps(text: str, root: str) -> str:
    """Validate every step, then return the text with canonical From lines."""
    version = file_version(text)
    if version is None:
        raise MetadataError('legacy step list has no step-metadata marker; add version 1 metadata')
    heads = list(STEP_HEADING.finditer(text))
    if not heads:
        raise MetadataError('the step list declares no steps')

    parsed = []
    problems = []
    for index, match in enumerate(heads):
        end = heads[index + 1].start() if index + 1 < len(heads) else len(text)
        body = match.group(3) + '\n' + text[match.end():end]
        try:
            payload = parse_step(body, version)
            citation_problems = validate_citations(payload, root)
        except MetadataError as exc:
            problems.append('%s: %s' % (match.group(1), exc))
            continue
        problems.extend('%s: %s' % (match.group(1), problem) for problem in citation_problems)
        parsed.append((match, end, body, payload))
    if problems:
        raise MetadataError('\n'.join(problems))

    updated = text
    for match, end, body, payload in reversed(parsed):
        rendered = _render_body(body, payload)
        title_prefix = match.group(3) + '\n'
        replacement = rendered[len(title_prefix):]
        updated = updated[:match.end()] + replacement + updated[end:]
    return updated
