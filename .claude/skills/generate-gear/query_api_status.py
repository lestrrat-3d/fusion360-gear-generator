#!/usr/bin/env python3
"""Print the shared Fusion API support decision for one call."""
import argparse
import json
import sys

import fusion_api


def parser():
    result = argparse.ArgumentParser(
        description='Describe whether one Fusion API call is documented, unverified, or blocked.')
    result.add_argument('--owner', help='qualified receiver class, such as adsk.fusion.Sketch')
    result.add_argument('--member', required=True, help='member name')
    return result


def main(argv=None):
    arguments = parser().parse_args(argv)
    try:
        fusion_api.validate_call(arguments.owner, arguments.member)
        with fusion_api.query_session():
            result = fusion_api.describe_call(arguments.owner, arguments.member)
    except ValueError as exc:
        parser().error(str(exc))
    except fusion_api.Unavailable as exc:
        result = fusion_api._call_result(
            arguments.owner, arguments.member, 'unavailable', 'setup_error', evidence=(str(exc),))
    json.dump(result, sys.stdout, sort_keys=True)
    sys.stdout.write('\n')
    if result['disposition'] in ('allow', 'advisory'):
        return 0
    if result['disposition'] == 'block':
        return 1
    return 2


if __name__ == '__main__':
    sys.exit(main())
