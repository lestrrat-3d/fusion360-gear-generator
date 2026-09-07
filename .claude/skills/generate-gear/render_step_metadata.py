#!/usr/bin/env python3
"""Render canonical citation lines from a compiled step list's metadata."""
import os
import stat
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import step_metadata  # noqa: E402


USAGE = 'usage: render_step_metadata.py <gear> --write <steps.md>'


class UsageError(ValueError):
    pass


def parse_args(argv):
    args = list(argv[1:])
    if len(args) != 3 or args[0].startswith('-') or args[1] != '--write' or not args[2]:
        raise UsageError(USAGE)
    return args[0], args[2]


def _read_utf8(path):
    try:
        with open(path, 'rb') as handle:
            return handle.read().decode('utf-8')
    except UnicodeDecodeError as exc:
        raise OSError('%s is not valid UTF-8' % path) from exc


def write_rendered(path, root):
    """Validate the complete draft and atomically replace it when rendering changes bytes."""
    original = _read_utf8(path)
    rendered = step_metadata.render_steps(original, root)
    encoded = rendered.encode('utf-8')
    if encoded == original.encode('utf-8'):
        return False

    directory = os.path.dirname(os.path.abspath(path))
    prefix = '.%s.' % os.path.basename(path)
    descriptor, temporary = tempfile.mkstemp(prefix=prefix, dir=directory)
    try:
        os.fchmod(descriptor, stat.S_IMODE(os.stat(path).st_mode))
        with os.fdopen(descriptor, 'wb') as handle:
            descriptor = -1
            handle.write(encoded)
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
    return True


def main(argv):
    try:
        gear, path = parse_args(argv)
    except UsageError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    instructions = os.path.join('spec', gear, 'instructions.md')
    try:
        with open(instructions, 'rb'):
            pass
    except FileNotFoundError:
        print('render_step_metadata: %s does not exist; is %r a real gear?'
              % (instructions, gear), file=sys.stderr)
        return 2
    except OSError as exc:
        print('render_step_metadata: %s' % exc, file=sys.stderr)
        return 2
    try:
        changed = write_rendered(path, '.')
    except step_metadata.MetadataError as exc:
        print('render_step_metadata: %s' % exc, file=sys.stderr)
        return 1
    except OSError as exc:
        print('render_step_metadata: %s' % exc, file=sys.stderr)
        return 2
    message = 'rendered' if changed else 'already current'
    print('render_step_metadata: %s is %s' % (path, message))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
