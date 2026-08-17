"""Extract calls from explicit step-list code spans.

Bare words keep a minimum length because prose such as ``tan()`` appears in formula
examples. Dotted calls are unambiguous API references, so their method names have no
minimum length.
"""
import re


CALL_PATTERN = re.compile(
    r'\b(?:(?P<receiver>[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*)\.'
    r'(?P<method>[a-z][A-Za-z0-9_]*)|'
    r'(?P<bare>[a-z][A-Za-z0-9_]{5,}))\s*\(')


def call_shapes(span):
    """Return ``(name, receiver)`` pairs from one code span.

    The complete receiver is retained for dotted calls. Callers that need the object
    immediately owning the method can take the final component themselves.
    """
    return {
        (match.group('method') or match.group('bare'), match.group('receiver'))
        for match in CALL_PATTERN.finditer(span)
    }
