#!/usr/bin/env python3
"""Guardrail: catch wrong-module adsk references in a generated gear file.

`adsk.core` vs `adsk.fusion` is a runtime distinction the Python tooling here can't
see: `adsk` is a native module that can't be imported outside Fusion, so `ast.parse`
and `pyflakes` both accept `adsk.fusion.SurfaceTypes` even though `SurfaceTypes` lives
in `adsk.core` — it only blows up as `AttributeError: module 'adsk.fusion' has no
attribute 'SurfaceTypes'` at Fusion runtime.

This lint learns the canonical module for every `adsk.core/fusion.<Name>` from the
TRUSTED code in the repo (the framework + every *other* gear — the gear under generation
is excluded so it can't be its own oracle), seeds a few well-known core/fusion names in
case they don't appear elsewhere yet, then flags any reference in the target file that
uses a different module than the trusted code uses for that name.

Usage:  python3 check_adsk_modules.py <generated.py> [<repo-root>]
Exit 1 and print the mismatches if any are found; exit 0 and print "adsk modules OK" otherwise.
"""
import re, glob, os, sys

# Well-known split, seeded so a name still gets checked even if no trusted file uses it yet.
KNOWN_CORE = {'Point3D', 'Point2D', 'Vector3D', 'Vector2D', 'Matrix3D', 'Matrix2D',
              'Line3D', 'ObjectCollection', 'ValueInput', 'SurfaceTypes',
              'Curve3DTypes', 'Curve2DTypes'}
KNOWN_FUSION = {'FeatureOperations', 'SurfaceProjectTypes', 'PatternComputeOptions'}

_REF = re.compile(r'adsk\.(core|fusion)\.([A-Za-z]\w*)')


def main():
    if len(sys.argv) < 2:
        print('usage: check_adsk_modules.py <generated.py> [<repo-root>]')
        sys.exit(2)
    target = sys.argv[1]
    root = sys.argv[2] if len(sys.argv) > 2 else '.'
    target_base = os.path.basename(target).replace('.generated', '')

    canon = {n: {'core'} for n in KNOWN_CORE}
    canon.update({n: {'fusion'} for n in KNOWN_FUSION})
    for f in glob.glob(os.path.join(root, 'lib/**/*.py'), recursive=True):
        if os.path.basename(f) == target_base:   # the gear under generation is not its own oracle
            continue
        for mod, name in _REF.findall(open(f, encoding='utf-8').read()):
            canon.setdefault(name, set()).add(mod)

    bad = set()
    for mod, name in _REF.findall(open(target, encoding='utf-8').read()):
        if name in canon and mod not in canon[name]:
            bad.add(f'adsk.{mod}.{name}  ->  should be adsk.{"/".join(sorted(canon[name]))}.{name}')

    if bad:
        for b in sorted(bad):
            print('ADSK-MODULE MISMATCH: ' + b)
        sys.exit(1)
    print('adsk modules OK')


if __name__ == '__main__':
    main()
