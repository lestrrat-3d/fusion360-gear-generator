#!/usr/bin/env python3
"""Render one complete gate-runner JSON report as canonical retry feedback.

Usage: render_retry_feedback.py --report <gate.json> --out <feedback.txt>
Exit 0 writes the complete report atomically. Exit 2 leaves the output unchanged.
"""
import argparse
import json
import os
import sys
import tempfile
from pathlib import Path


class FeedbackError(Exception):
    """The report or output cannot satisfy the retry-feedback contract."""


def _object_without_duplicates(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise FeedbackError("duplicate JSON key {!r}".format(key))
        result[key] = value
    return result


def _reject_nonstandard_number(value):
    raise FeedbackError("invalid JSON number {}".format(value))


def load_report(path):
    """Read and validate a complete compile- or emit-runner report."""
    try:
        raw = Path(path).read_bytes()
    except OSError as error:
        raise FeedbackError("cannot read report {}: {}".format(path, error)) from error
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as error:
        raise FeedbackError("report {} is not valid UTF-8: {}".format(path, error)) from error
    try:
        report = json.loads(
            text,
            object_pairs_hook=_object_without_duplicates,
            parse_constant=_reject_nonstandard_number,
        )
    except FeedbackError:
        raise
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise FeedbackError("report {} is not valid JSON: {}".format(path, error)) from error

    if not isinstance(report, dict):
        raise FeedbackError("report root must be a JSON object")
    schema = report.get("schema")
    if isinstance(schema, bool) or not isinstance(schema, int) or schema != 1:
        raise FeedbackError("report schema must be the integer 1")

    recognized = [name for name in ("stages", "gates") if name in report]
    if len(recognized) != 1:
        raise FeedbackError("report must contain exactly one of 'stages' or 'gates'")
    if not isinstance(report[recognized[0]], list):
        raise FeedbackError("report field {!r} must be a list".format(recognized[0]))
    return report


def canonical_feedback(report):
    """Return the full report in the one canonical retry representation."""
    try:
        return json.dumps(
            report, sort_keys=True, ensure_ascii=False, indent=2, allow_nan=False
        ) + "\n"
    except (TypeError, ValueError, UnicodeEncodeError) as error:
        raise FeedbackError("report cannot be rendered as UTF-8 JSON: {}".format(error)) from error


def write_atomic(path, text):
    """Replace ``path`` atomically after the complete UTF-8 output is available."""
    output = Path(path)
    try:
        data = text.encode("utf-8")
    except UnicodeEncodeError as error:
        raise FeedbackError("feedback is not valid UTF-8: {}".format(error)) from error

    descriptor = None
    temporary = None
    try:
        descriptor, temporary = tempfile.mkstemp(
            prefix=".{}-".format(output.name), suffix=".tmp", dir=str(output.parent)
        )
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = None
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, output)
        temporary = None
    except OSError as error:
        raise FeedbackError("cannot write feedback {}: {}".format(output, error)) from error
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            try:
                os.unlink(temporary)
            except OSError:
                pass


def parse_args(argv):
    parser = argparse.ArgumentParser(
        prog="render_retry_feedback.py",
        description="Render a complete runner JSON report as canonical retry feedback.",
    )
    parser.add_argument("--report", required=True)
    parser.add_argument("--out", required=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        report = load_report(args.report)
        write_atomic(args.out, canonical_feedback(report))
    except FeedbackError as error:
        sys.stderr.write("render_retry_feedback: {}\n".format(error))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
