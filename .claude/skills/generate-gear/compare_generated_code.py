#!/usr/bin/env python3
"""Compare generated Python against a frozen baseline using significant tokens.

Metric version 1: tokenize both files; remove ENCODING, COMMENT, NL, NEWLINE,
INDENT, DEDENT, and ENDMARKER; compare ordered (token type, token text) pairs
with difflib.SequenceMatcher(autojunk=False).ratio(). Acceptance needs >= 0.95.
This measures source agreement, not Fusion behavior or gate correctness.
"""
import argparse
import difflib
import hashlib
import io
import json
import tokenize
from pathlib import Path

OMIT = {tokenize.ENCODING, tokenize.COMMENT, tokenize.NL, tokenize.NEWLINE,
        tokenize.INDENT, tokenize.DEDENT, tokenize.ENDMARKER}
THRESHOLD = 0.95


def tokens(data):
    return [(item.type, item.string) for item in tokenize.tokenize(io.BytesIO(data).readline)
            if item.type not in OMIT]


def compare(baseline, candidate):
    old = baseline.read_bytes()
    new = candidate.read_bytes()
    old_tokens, new_tokens = tokens(old), tokens(new)
    ratio = difflib.SequenceMatcher(None, old_tokens, new_tokens,
                                    autojunk=False).ratio()
    return {'metric_version': 1, 'ratio': ratio, 'threshold': THRESHOLD,
            'accepted': ratio >= THRESHOLD,
            'baseline_sha256': hashlib.sha256(old).hexdigest(),
            'candidate_sha256': hashlib.sha256(new).hexdigest(),
            'baseline_tokens': len(old_tokens), 'candidate_tokens': len(new_tokens)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('candidate', type=Path)
    args = parser.parse_args()
    result = compare(args.baseline, args.candidate)
    print(json.dumps(result, sort_keys=True, indent=2))
    return 0 if result['accepted'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
