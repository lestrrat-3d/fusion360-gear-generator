import math
import os
import sys
import time
import traceback
import types
import adsk.core, adsk.fusion

# diag_pattern2 drew the disc's structure with plain Fusion calls and every variant came out fully constrained, yet
# the add-in's disc sketch is not. This drives the add-in's own code instead: the runtime's Sketch class and the
# translated _draw_patterned_loop, on a scratch document, with a disc-like lobe of 3 splines (11, 10 and 11 fit
# points), a bore circle at (1.5, 0) mm as the pattern centre, 7 output holes, a diameter bound to a user parameter
# and the anchor coincident at the end. The variants then take parts away to find which one leaves the junctions free:
#   real       everything
#   nobind     the diameter not bound to a user parameter
#   noholes    no output holes
#   noanchor   the anchor coincident skipped
#   bare       none of the three
# Run it from the Text Commands window in Py mode (the Gear Generator add-in must be running):
#   exec(open(r'<AddIns>\Gear Generator\diag_pattern3.py').read())
# The results are written to diag_pattern3_out.txt beside this file after every variant.

app = adsk.core.Application.get()
lines = []


def appdata_dir():
    appdata = os.environ.get('APPDATA')
    if appdata:
        return os.path.join(appdata, 'Autodesk', 'Autodesk Fusion 360', 'API', 'AddIns', 'Gear Generator')
    return os.path.expanduser('~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/Gear Generator')


def log(s):
    lines.append(s)
    print(s)
    try:
        with open(os.path.join(appdata_dir(), 'diag_pattern3_out.txt'), 'w') as f:
            f.write('\n'.join(lines))
    except OSError:
        pass


def loaded(suffix):
    for name, module in list(sys.modules.items()):
        if name.endswith(suffix):
            return module
    raise RuntimeError('module ending in %s is not loaded: run the Gear Generator add-in first' % suffix)


fx = loaded('lib.geargen.decadfusion')
cyc = loaded('lib.geargen.cycloidal')

LOBES = 15
COUNTS = [11, 10, 11]
RADIUS = 40.0
DEPTH = 2.0
CX = 1.5


def lobe_runs():
    """The fit points in mm of one lobe, three splines chained counter-clockwise, as objects with x and y."""
    steps = sum(c - 1 for c in COUNTS)
    points = []
    for j in range(steps + 1):
        theta = 0.3 + 2 * math.pi * (j / steps) / LOBES
        r = RADIUS + DEPTH * math.cos(LOBES * (theta - 0.3))
        points.append(types.SimpleNamespace(x=CX + r * math.cos(theta), y=r * math.sin(theta)))
    runs, at = [], 0
    for c in COUNTS:
        runs.append(points[at:at + c])
        at += c - 1
    return runs


def run_variant(name, bind, holes, anchor):
    doc = None
    try:
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        fsketch = root.sketches.add(root.xYConstructionPlane)
        design.userParameters.add('DiagBore', adsk.core.ValueInput.createByReal(1.2), 'cm', '')
        runtime = types.SimpleNamespace(scale=0.1, anchor=root.originConstructionPoint,
                                        _parameters=lambda n: ('DiagBore', None))
        s = fx.Sketch(runtime, fsketch)
        if not anchor:
            s._anchor = lambda: None
        start = time.perf_counter()
        # circleAt: a dimensioned centre, the circle, its diameter bound to a parameter.
        centre = s.dimension_point(CX, 0.0)
        bore = s.create_circle(centre, 6.0)
        diameter = fx.sketch_new_diameter(bore, 12.0)
        s.add_constraint(diameter)
        if bind:
            s.bind(diameter, None, 'disc_bore_diameter')
        # The lobe, patterned about the bore centre.
        runs = lobe_runs()
        cyc._draw_patterned_loop(s, bore.Center, runs, LOBES)
        if holes:
            for k in range(7):
                a = 2 * math.pi * k / 7
                hole_centre = s.dimension_point(CX + 20.0 * math.cos(a), 20.0 * math.sin(a))
                hole = s.create_circle(hole_centre, 4.0)
                hole_d = fx.sketch_new_diameter(hole, 8.0)
                s.add_constraint(hole_d)
        s.solve()
        text = 'constrained %s' % fsketch.isFullyConstrained
        if not fsketch.isFullyConstrained:
            text += '; free: ' + '; '.join(fx.unconstrained_entities(fsketch, 6))
        log('%-9s %5.2f s  %s' % (name, time.perf_counter() - start, text))
    except Exception:
        log('%-9s FAILED %s' % (name, traceback.format_exc().splitlines()[-1]))
    finally:
        if doc is not None:
            doc.close(False)


try:
    run_variant('real', True, True, True)
    run_variant('nobind', False, True, True)
    run_variant('noholes', True, False, True)
    run_variant('noanchor', True, True, False)
    run_variant('bare', False, False, False)
finally:
    log('done')
